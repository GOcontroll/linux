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

        cat > "$OUT/modules/kernel-build-info" <<KBINFO
KERNEL_VERSION="$(make -s kernelversion)"
KERNEL_BUILD_DATE="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
KERNEL_BUILD_SHA="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
KBINFO

        # Modules ook naar deploy/modules/ — convergence point voor alle
        # firmware-artefacten. Module-injectie in rootfs.ext4 is een aparte
        # stap die de gebruiker handmatig doet (deploy/inject-modules.bat).
        if [ -d "$DEPLOY" ]; then
            rm -rf "$DEPLOY/modules"
            cp -r "$OUT/modules/lib/modules" "$DEPLOY/modules"
            cp "$OUT/modules/kernel-build-info" "$DEPLOY/modules/"
            echo "  - $DEPLOY/modules/      (kernel modules + build-info)"
        fi

        echo "Artifacts in: $OUT"
        echo "  - $OUT/boot/            (full kernel install)"
        echo "  - $OUT/uboot-stage/     (Image.zst + moduline DTBs for projects/Uboot/scripts/mkkernel_itb.sh)"
        ;;
    multi)
        # Multi-defconfig orchestrator. Bouwt voor elke defconfig in $DEFCONFIGS
        # een complete set (Image + modules + DTBs) en plaatst per-cfg output in
        # $OUT/uboot-stage/<cfg>/ en modules in $OUT/modules-<cfg>/. Voor
        # backwards-compat copie van de eerste defconfig naar flat
        # $OUT/uboot-stage/Image + DTBs (zodat de bestaande imx8mm-gebaseerde
        # mkkernel_itb.sh ongewijzigd blijft werken).
        #
        # Gebruik:   DEFCONFIGS="gocontroll_imx8mm_defconfig gocontroll_imx8mp_defconfig" ./build.sh multi
        OUT="${OUT:-$PWD/out}"
        DEPLOY="${DEPLOY:-$PWD/../deploy}"
        DEFCONFIGS="${DEFCONFIGS:-$DEFCONFIG}"
        mkdir -p "$OUT/boot" "$OUT/uboot-stage"
        FIRST_CFG=""
        for cfg in $DEFCONFIGS; do
            [ -z "$FIRST_CFG" ] && FIRST_CFG="$cfg"
            echo "==> [multi] building with defconfig=$cfg"
            make mrproper
            make "$cfg"
            make -j"$JOBS" Image Image.zst modules dtbs

            STAGE="$OUT/uboot-stage/$cfg"
            rm -rf "$STAGE"
            mkdir -p "$STAGE"
            cp arch/arm64/boot/Image     "$STAGE/Image"
            cp arch/arm64/boot/Image.zst "$STAGE/Image.zst"
            # Filter DTBs per SoC-family op basis van defconfig — voorkomt
            # dat M1/L4 (imx8mm) DTBs in de imx8mp stage belanden en
            # andersom. Elke per-cfg stage bevat dus alleen de DTBs die
            # bij dat SoC horen, conform de gebouwde Image.
            case "$cfg" in
                *imx8mm*)  DTB_PREFIX="imx8mm-tx8m-1610-moduline" ;;
                *imx8mp*)  DTB_PREFIX="imx8mp-tx8p-ml81-moduline" ;;
                *)         DTB_PREFIX="moduline" ;;  # fallback: alles
            esac
            find arch/arm64/boot/dts -name "${DTB_PREFIX}*.dtb"  -exec cp {} "$STAGE/" \;
            find arch/arm64/boot/dts -name "${DTB_PREFIX}*.dtbo" -exec cp {} "$STAGE/" \;

            MODS="$OUT/modules-$cfg"
            rm -rf "$MODS"
            mkdir -p "$MODS"
            make INSTALL_MOD_PATH="$MODS" modules_install
            find "$MODS/lib/modules" -maxdepth 2 -type l -delete

            cat > "$MODS/kernel-build-info" <<KBINFO
KERNEL_VERSION="$(make -s kernelversion)"
KERNEL_BUILD_DATE="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
KERNEL_BUILD_SHA="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
KERNEL_BUILD_DEFCONFIG="$cfg"
KBINFO
        done

        # Backwards-compat: flat output van de eerste defconfig naar uboot-stage/.
        # Reden: mkkernel_itb.sh (M1/L4) leest uit $OUT/uboot-stage/Image + flat
        # DTBs — die path-conventie blijft ongewijzigd.
        if [ -d "$OUT/uboot-stage/$FIRST_CFG" ]; then
            cp -f "$OUT/uboot-stage/$FIRST_CFG/Image"     "$OUT/uboot-stage/Image"
            cp -f "$OUT/uboot-stage/$FIRST_CFG/Image.zst" "$OUT/uboot-stage/Image.zst"
            find "$OUT/uboot-stage/$FIRST_CFG" -maxdepth 1 -name "*moduline*.dtb" -exec cp -f {} "$OUT/uboot-stage/" \;
        fi

        # Modules naar deploy/. Per-cfg subdir + flat-kopie van eerste defconfig
        # voor backwards-compat met inject-modules.sh (rootfs13_headless gebruikt
        # de imx8mm-set, rootfs13_display de imx8mp-set — selectie in inject step).
        if [ -d "$DEPLOY" ]; then
            for cfg in $DEFCONFIGS; do
                rm -rf "$DEPLOY/modules-$cfg"
                cp -r "$OUT/modules-$cfg/lib/modules" "$DEPLOY/modules-$cfg"
                cp "$OUT/modules-$cfg/kernel-build-info" "$DEPLOY/modules-$cfg/"
            done
            rm -rf "$DEPLOY/modules"
            cp -r "$OUT/modules-$FIRST_CFG/lib/modules" "$DEPLOY/modules"
            cp "$OUT/modules-$FIRST_CFG/kernel-build-info" "$DEPLOY/modules/"
            echo "  - $DEPLOY/modules/             (flat = $FIRST_CFG)"
            for cfg in $DEFCONFIGS; do
                echo "  - $DEPLOY/modules-$cfg/"
            done
        fi

        echo "Multi-defconfig build done: $DEFCONFIGS"
        echo "  - $OUT/uboot-stage/<cfg>/    (per-defconfig Image + DTBs + DTBOs)"
        echo "  - $OUT/uboot-stage/          (flat = $FIRST_CFG, voor mkkernel_itb.sh)"
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

        # HMI1 (i.MX8MP) kernel.itb regen — alleen als de overlay is gebouwd:
        # vereist $UBOOT_DIR/out/bl31-mp.bin (uit `make atf-mp`) én een
        # uboot-stage/gocontroll_imx8mp_defconfig/ subdir met Image+DTBs (uit
        # `./build.sh multi` met imx8mp_defconfig erbij).
        if [ -x "$UBOOT_DIR/scripts/mkkernel_itb_hmi1.sh" ] \
           && [ -f "$UBOOT_DIR/out/bl31-mp.bin" ] \
           && [ -d "$OUT/uboot-stage/gocontroll_imx8mp_defconfig" ]; then
            echo "[deploy] regenerating kernel_hmi1.itb"
            ( cd "$UBOOT_DIR" && ./scripts/mkkernel_itb_hmi1.sh )
            cp -v "$UBOOT_DIR/out/kernel_hmi1.itb" "$DEPLOY/"
        fi

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
        echo "Usage: $0 {config|menuconfig|kernel|modules|dtbs|all|install|multi|deploy|clean}"
        echo "       multi: bouwt voor elke defconfig in \$DEFCONFIGS (spatie-gescheiden)"
        echo "              en plaatst per-cfg output in \$OUT/uboot-stage/<cfg>/"
        exit 1
        ;;
esac
