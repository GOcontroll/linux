#!/bin/bash
set -e

export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
JOBS="${JOBS:-$(nproc)}"

cd "$(dirname "$0")"

case "${1:-all}" in
    config)
        make gocontroll_defconfig
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

        # Modules ook naar deploy/modules/ — convergence point voor alle
        # firmware-artefacten. Geen mounten/injecteren hier; dat doet
        # Uboot/make pkg straks via deploy/inject-modules.sh.
        if [ -d "$DEPLOY" ]; then
            rm -rf "$DEPLOY/modules"
            cp -r "$OUT/modules/lib/modules" "$DEPLOY/modules"
            echo "  - $DEPLOY/modules/      (kernel modules, gekopieerd voor Uboot pkg-step)"
        fi

        echo "Artifacts in: $OUT"
        echo "  - $OUT/boot/            (full kernel install)"
        echo "  - $OUT/uboot-stage/     (Image.zst + moduline DTBs for projects/Uboot/scripts/mkkernel_itb.sh)"
        ;;
    *)
        echo "Usage: $0 {config|menuconfig|kernel|modules|dtbs|all|install|clean}"
        exit 1
        ;;
esac
