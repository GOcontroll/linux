#!/bin/bash
# inject-modules.sh — voeg /lib/modules/<kver>/ toe aan een bestaande rootfs.ext4
#
# Bedoeld om vóór UUU-flash de kernel-modules in de deploy-rootfs te injecteren.
# Werkt op iedere ext4-image; auto-resized het image als er onvoldoende vrije
# ruimte is en shrinkt 'm aan het eind weer terug.
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

IMG="${1:?usage: $0 <rootfs.ext4> [modules-dir]}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MOD_SRC="${2:-$SCRIPT_DIR/modules}"

[ -f "$IMG" ] || { echo "ERROR: $IMG niet gevonden" >&2; exit 1; }
[ -d "$MOD_SRC" ] || { echo "ERROR: modules-dir $MOD_SRC niet gevonden" >&2; exit 1; }

# Detect kernel-version: enige subdir in de modules-dir
KVER=$(find "$MOD_SRC" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | head -1)
if [ -z "$KVER" ]; then
    echo "ERROR: geen <kver>/ subdir in $MOD_SRC" >&2
    exit 1
fi

KMOD_BYTES=$(du -sb "$MOD_SRC/$KVER" | awk '{print $1}')
KMOD_MIB=$((KMOD_BYTES / 1024 / 1024))

echo "[inject-modules] image=$IMG"
echo "[inject-modules] modules=$MOD_SRC/$KVER (${KMOD_MIB} MiB)"

# 1. Resize ext4 UP — ruimte voor modules + 32 MiB headroom
HEADROOM=$((32 * 1024 * 1024))
NEW_BYTES=$(($(stat -c%s "$IMG") + KMOD_BYTES + HEADROOM))
echo "[inject-modules] resizing $IMG up to $((NEW_BYTES / 1024 / 1024)) MiB"
truncate -s "$NEW_BYTES" "$IMG"
e2fsck -fy "$IMG" >/dev/null
resize2fs "$IMG" >/dev/null

# 2. Mount + rsync + depmod + umount
TMP=$(mktemp -d)
trap "umount '$TMP' 2>/dev/null || true; rmdir '$TMP'" EXIT
mount -o loop "$IMG" "$TMP"

echo "[inject-modules] rsync /lib/modules/$KVER → image:/lib/modules/$KVER"
mkdir -p "$TMP/lib/modules"
rsync -a --delete "$MOD_SRC/$KVER/" "$TMP/lib/modules/$KVER/"

echo "[inject-modules] running depmod -a $KVER inside image"
chroot "$TMP" /sbin/depmod -a "$KVER"

umount "$TMP"
rmdir "$TMP"
trap - EXIT

# 3. Shrink terug naar minimum
echo "[inject-modules] shrinking back to minimum"
e2fsck -fy "$IMG" >/dev/null
resize2fs -M "$IMG" >/dev/null
BS=$(tune2fs -l "$IMG" | awk -F: '/Block size/{gsub(/ /,"",$2); print $2}')
BC=$(tune2fs -l "$IMG" | awk -F: '/Block count/{gsub(/ /,"",$2); print $2}')
truncate -s $((BS * BC)) "$IMG"

FINAL_MIB=$(($(stat -c%s "$IMG") / 1024 / 1024))
echo "[inject-modules] done — $IMG is now ${FINAL_MIB} MiB"
