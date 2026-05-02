#!/bin/bash
set -e

export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
JOBS="${JOBS:-$(nproc)}"
DEFCONFIG="${DEFCONFIG:-gocontroll_imx8mm_defconfig}"

cd "$(dirname "$0")"

case "${1:-all}" in
    config)
        make "$DEFCONFIG"
        ;;
    menuconfig)
        make menuconfig
        ;;
    kernel)
        make -j"$JOBS" Image Image.zst
        ;;
    modules)
        make -j"$JOBS" modules
        ;;
    dtbs)
        make -j"$JOBS" dtbs
        ;;
    clean)
        make mrproper
        ;;
    all)
        make -j"$JOBS" Image Image.zst modules dtbs
        ;;
    install)
        OUT="${OUT:-$PWD/out}"
        DEPLOY="${DEPLOY:-$PWD/../deploy}"
        mkdir -p "$OUT/boot" "$OUT/modules" "$OUT/uboot-stage"
        cp arch/arm64/boot/Image "$OUT/boot/" 2>/dev/null || true
        cp arch/arm64/boot/Image.zst "$OUT/boot/"
        cp arch/arm64/boot/Image "$OUT/uboot-stage/Image"
        cp arch/arm64/boot/Image.zst "$OUT/uboot-stage/Image.zst"
        find arch/arm64/boot/dts -name "*moduline*.dtb" -exec cp {} "$OUT/boot/" \;
        find arch/arm64/boot/dts -name "imx8mm-tx8m-1610-moduline-*.dtb" -exec cp {} "$OUT/uboot-stage/" \;
        make INSTALL_MOD_PATH="$OUT/modules" modules_install

        # Strip kbuild's `build` en `source` symlinks. Die wijzen op de host
        # naar de kernel-source-tree (out-of-tree builds + DKMS), maar zijn
        # op de Moduline-target dangling en irrelevant — we builden niet
        # on-device. Bovendien volgen robocopy/OneDrive ze richting Windows
        # en creëren daar 0-byte placeholders die niet te verwijderen zijn.
        find "$OUT/modules/lib/modules" -maxdepth 2 -type l -delete

        # Modules ook naar deploy/modules/ — convergence point voor alle
        # firmware-artefacten. Module-injectie in rootfs.ext4 is een aparte
        # stap die de gebruiker handmatig doet (deploy/inject-modules.bat).
        if [ -d "$DEPLOY" ]; then
            rm -rf "$DEPLOY/modules"
            cp -r "$OUT/modules/lib/modules" "$DEPLOY/modules"
            echo "  - $DEPLOY/modules/      (kernel modules)"
        fi

        echo "Artifacts in: $OUT"
        echo "  - $OUT/boot/            (full kernel install)"
        echo "  - $OUT/uboot-stage/     (Image.zst + moduline DTBs for projects/Uboot/scripts/mkkernel_itb.sh)"
        ;;
    deploy)
        # End-to-end: rebuild + install + regenerate kernel.itb + copy naar deploy/.
        # Zelfstandig commando — na `./build.sh deploy` is alle linux-output in
        # ~/projects/deploy/ up-to-date (modules + kernel.itb).
        #
        # kernel.itb regen vereist bl31.bin (uit Uboot/ATF). Als die ontbreekt
        # (eerste run, alleen linux gebouwd) wordt de regen overgeslagen met
        # een waarschuwing — draai dan eenmalig `cd ../Uboot && make atf`.
        "$0" all
        "$0" install

        DEPLOY="${DEPLOY:-$PWD/../deploy}"
        UBOOT_DIR="${UBOOT_DIR:-$PWD/../Uboot}"

        if [ ! -d "$DEPLOY" ]; then
            echo "[deploy] WARN: $DEPLOY bestaat niet — sla kernel.itb regen over"
            exit 0
        fi

        if [ ! -x "$UBOOT_DIR/scripts/mkkernel_itb.sh" ]; then
            echo "[deploy] WARN: $UBOOT_DIR/scripts/mkkernel_itb.sh niet gevonden — sla kernel.itb regen over"
            exit 0
        fi

        if [ ! -f "$UBOOT_DIR/out/bl31.bin" ]; then
            echo "[deploy] WARN: $UBOOT_DIR/out/bl31.bin ontbreekt — sla kernel.itb regen over"
            echo "[deploy]       Draai eenmalig: cd $UBOOT_DIR && make atf"
            exit 0
        fi

        echo "[deploy] regenerating kernel.itb"
        ( cd "$UBOOT_DIR" && ./scripts/mkkernel_itb.sh )
        cp -v "$UBOOT_DIR/out/kernel.itb" "$DEPLOY/"

        # Auto-inject de zojuist gebouwde modules in elke rootfs in deploy/.
        # Houdt deploy/ "ready-to-flash" zonder dat de gebruiker er op moet
        # letten. Idempotent — slaat per image over als die niet bestaat.
        # Vereist sudo (mount + chroot); als geen NOPASSWD geconfigureerd
        # vraagt sudo om een wachtwoord.
        INJECT="$PWD/scripts/inject-modules.sh"
        if [ -x "$INJECT" ]; then
            for img in "$DEPLOY"/rootfs13_headless.ext4 "$DEPLOY"/rootfs13_display.ext4; do
                [ -f "$img" ] || continue
                echo "[deploy] injecting modules into $img"
                sudo "$INJECT" "$img" "$DEPLOY/modules"
            done
        else
            echo "[deploy] WARN: $INJECT niet executable — sla module-inject over"
        fi

        echo "[deploy] done — kernel + modules + kernel.itb (+ rootfs met modules) in $DEPLOY"
        ;;
    *)
        echo "Usage: $0 {config|menuconfig|kernel|modules|dtbs|all|install|deploy|clean}"
        exit 1
        ;;
esac
