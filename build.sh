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
        mkdir -p "$OUT/boot" "$OUT/modules" "$OUT/uboot-stage"
        cp arch/arm64/boot/Image "$OUT/boot/" 2>/dev/null || true
        cp arch/arm64/boot/Image.zst "$OUT/boot/"
        cp arch/arm64/boot/Image "$OUT/uboot-stage/Image"
        cp arch/arm64/boot/Image.zst "$OUT/uboot-stage/Image.zst"
        find arch/arm64/boot/dts -name "*moduline*.dtb" -exec cp {} "$OUT/boot/" \;
        find arch/arm64/boot/dts -name "imx8mm-tx8m-1610-moduline-*.dtb" -exec cp {} "$OUT/uboot-stage/" \;
        make INSTALL_MOD_PATH="$OUT/modules" modules_install
        echo "Artifacts in: $OUT"
        echo "  - $OUT/boot/            (full kernel install)"
        echo "  - $OUT/uboot-stage/     (Image.zst + moduline DTBs for projects/Uboot/scripts/mkkernel_itb.sh)"
        ;;
    *)
        echo "Usage: $0 {config|menuconfig|kernel|modules|dtbs|all|install|clean}"
        exit 1
        ;;
esac
