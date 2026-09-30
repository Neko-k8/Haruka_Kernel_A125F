#!/bin/bash

export TC_DIR="/home/shiroyura/toolchain"
export CC="${TC_DIR}/neutron-clang-06092026/bin/clang"
export CLANG_TRIPLE=aarch64-linux-gnu-
export ARCH=arm64

export CROSS_COMPILE=aarch64-linux-gnu-

export AR="${TC_DIR}/neutron-clang-06092026/bin/llvm-ar"
export NM="${TC_DIR}/neutron-clang-06092026/bin/llvm-nm"
export OBJCOPY="${TC_DIR}/neutron-clang-06092026/bin/llvm-objcopy"
export OBJDUMP="${TC_DIR}/neutron-clang-06092026/bin/llvm-objdump"
export STRIP="${TC_DIR}/neutron-clang-06092026/bin/llvm-strip"
export OBJSIZE="${TC_DIR}/neutron-clang-06092026/bin/llvm-size"
export LD="${TC_DIR}/neutron-clang-06092026/bin/ld.lld"

export PATH="${TC_DIR}/neutron-clang-06092026/bin:${PATH}"

export OPT_FLAGS="-O3 -mcpu=cortex-a53+crypto+crc -ffp-contract=fast -mllvm -enable-epilogue-vectorization -mllvm -polly"

export KCFLAGS="${OPT_FLAGS}"

JOBS=$(nproc)

echo "=========================================="
echo " Select Build Target Variant"
echo "=========================================="
echo "1) Normal Build (A125F_defconfig)"
echo "2) DroidSpaces Build (A125F_defconfig + droidspaces_defconfig)"
echo "=========================================="
read -rp "Enter choice [1-2]: " BUILD_CHOICE

mkdir -p "$(pwd)/out"

export MAKE_FLAGS="ARCH=${ARCH} LLVM=1 LLVM_IAS=1 CC=${CC} LD=${LD} AR=${AR} NM=${NM} OBJCOPY=${OBJCOPY} OBJDUMP=${OBJDUMP} STRIP=${STRIP} CLANG_TRIPLE=${CLANG_TRIPLE} CROSS_COMPILE=${CROSS_COMPILE}"

case "${BUILD_CHOICE}" in
    1)
        echo "[+] Applying A125F_defconfig..."
        make -C "$(pwd)" O="$(pwd)/out" A125F_defconfig ${MAKE_FLAGS}
        ;;
    2)
        echo "[+] Applying A125F_defconfig + droidspaces_defconfig..."
        make -C "$(pwd)" O="$(pwd)/out" A125F_defconfig droidspaces_defconfig ${MAKE_FLAGS}
        ;;
    *)
        echo "[-] Invalid option selected. Exiting."
        exit 1
        ;;
esac

echo "[+] Starting build process with ${JOBS} CPU threads..."
make -C "$(pwd)" O="$(pwd)/out" -j"${JOBS}" ${MAKE_FLAGS}
