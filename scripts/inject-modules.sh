#!/bin/bash
# inject-modules.sh — voeg /lib/modules/<kver>/ toe aan een bestaande rootfs.ext4
#
# Bedoeld om vóór UUU-flash de kernel-modules in de deploy-rootfs te injecteren.
# Werkt op iedere ext4-image; auto-resized het image als er onvoldoende vrije
# ruimte is en shrinkt 'm aan het eind weer terug.
#
# WSL2-quirk: `mount -o loop` op een file in /mnt/c (DrvFs/9P) persisteert
# writes niet betrouwbaar naar de Windows-host — umount lijkt te slagen maar
# het backing-file blijft ongewijzigd. We werken daarom altijd op een kopie in
# een Linux-native pad (mktemp -d, = WSL ext4) en kopiëren het resultaat aan
# het eind terug naar het origineel.
#
# Gebruik:
#   sudo ./inject-modules.sh <rootfs.ext4> [modules-dir]
#
# Argumenten:
#   <rootfs.ext4>   Pad naar de ext4-image die geüpdatet wordt (in-place)
#   [modules-dir]   Pad naar dir met /lib/modules/<kver>/ subdir (default: ./modules
#                   relatief tov dit script — = deploy/modules/)
#
# Voorbeelden:
#   sudo ./inject-modules.sh rootfs13_headless.ext4
#   sudo ./inject-modules.sh /tmp/my-rootfs.ext4 /tmp/modules
#
# Vereist: root (mount), e2fsprogs (resize2fs/e2fsck/tune2fs), rsync, depmod.

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: must run as root (mount + chroot vereisen privileges)" >&2
    exit 1
fi

DST="${1:?usage: $0 <rootfs.ext4> [modules-dir]}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MOD_SRC="${2:-$SCRIPT_DIR/modules}"

[ -f "$DST" ] || { echo "ERROR: $DST niet gevonden" >&2; exit 1; }
[ -d "$MOD_SRC" ] || { echo "ERROR: modules-dir $MOD_SRC niet gevonden" >&2; exit 1; }

# Detect kernel-version: enige subdir in de modules-dir
KVER=$(find "$MOD_SRC" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | head -1)
if [ -z "$KVER" ]; then
    echo "ERROR: geen <kver>/ subdir in $MOD_SRC" >&2
    exit 1
fi

KMOD_BYTES=$(du -sb "$MOD_SRC/$KVER" | awk '{print $1}')
KMOD_MIB=$((KMOD_BYTES / 1024 / 1024))

echo "[inject-modules] image=$DST"
echo "[inject-modules] modules=$MOD_SRC/$KVER (${KMOD_MIB} MiB)"

# Werk altijd op een kopie in /tmp (= WSL ext4) om 9P-loop-mount issues te
# vermijden. /tmp is ramfs-achtig in WSL2 dus dit is bovendien snel.
WORK=$(mktemp -d)
IMG="$WORK/rootfs.ext4"
MNT="$WORK/mnt"
mkdir -p "$MNT"
cleanup() {
    umount "$MNT" 2>/dev/null || true
    rm -rf "$WORK"
}
trap cleanup EXIT

echo "[inject-modules] copying $DST → $IMG (Linux-native werkkopie)"
cp "$DST" "$IMG"

# 1. Resize ext4 UP — ruimte voor modules + 32 MiB headroom
HEADROOM=$((32 * 1024 * 1024))
NEW_BYTES=$(($(stat -c%s "$IMG") + KMOD_BYTES + HEADROOM))
echo "[inject-modules] resizing werkkopie up to $((NEW_BYTES / 1024 / 1024)) MiB"
truncate -s "$NEW_BYTES" "$IMG"
e2fsck -fy "$IMG" >/dev/null
resize2fs "$IMG" >/dev/null

# 2. Mount + rsync + depmod + umount (loop op /tmp = ext4, dus betrouwbaar)
mount -o loop "$IMG" "$MNT"

echo "[inject-modules] rsync /lib/modules/$KVER → image:/lib/modules/$KVER"
mkdir -p "$MNT/lib/modules"
rsync -a --delete "$MOD_SRC/$KVER/" "$MNT/lib/modules/$KVER/"

echo "[inject-modules] running depmod -a $KVER inside image"
chroot "$MNT" /sbin/depmod -a "$KVER"

sync
umount "$MNT"

# 3. Shrink terug naar minimum
echo "[inject-modules] shrinking back to minimum"
e2fsck -fy "$IMG" >/dev/null
resize2fs -M "$IMG" >/dev/null
BS=$(tune2fs -l "$IMG" | awk -F: '/Block size/{gsub(/ /,"",$2); print $2}')
BC=$(tune2fs -l "$IMG" | awk -F: '/Block count/{gsub(/ /,"",$2); print $2}')
truncate -s $((BS * BC)) "$IMG"

# 4. Kopieer werkkopie terug naar origineel (cp via 9P is wel betrouwbaar)
echo "[inject-modules] copying werkkopie → $DST"
cp "$IMG" "$DST"
sync

FINAL_MIB=$(($(stat -c%s "$DST") / 1024 / 1024))
echo "[inject-modules] done — $DST is now ${FINAL_MIB} MiB"
