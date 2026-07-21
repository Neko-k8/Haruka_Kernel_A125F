#!/bin/bash

export TC_DIR="/run/media/hakyura/261dea50-a8a4-4d2a-8b3e-540a49827e31/toolchain"
export CROSS_COMPILE="${TC_DIR}/aarch64--glibc--stable-2025.08-1/bin/aarch64-buildroot-linux-gnu-"
export CC="${TC_DIR}/Clang-22/bin/clang"
export CLANG_TRIPLE=aarch64-linux-gnu-
export ARCH=arm64
export ANDROID_MAJOR_VERSION=s

export AR="${TC_DIR}/Clang-22/bin/llvm-ar"
export NM="${TC_DIR}/Clang-22/bin/llvm-nm"
export OBJCOPY="${TC_DIR}/Clang-22/bin/llvm-objcopy"
export OBJDUMP="${TC_DIR}/Clang-22/bin/llvm-objdump"
export STRIP="${TC_DIR}/Clang-22/bin/llvm-strip"
export OBJSIZE="${TC_DIR}/Clang-22/bin/llvm-size"
export LD="${TC_DIR}/Clang-22/bin/ld.lld"

export PATH="${TC_DIR}/Clang-22/bin:${TC_DIR}/aarch64--glibc--stable-2025.08-1/bin:${PATH}"

export OPT_FLAGS="-O3 -mcpu=cortex-a53 -mtune=cortex-a53"
export KCFLAGS="-w ${OPT_FLAGS}"
export CONFIG_SECTION_MISMATCH_WARN_ONLY=y

mkdir -p $(pwd)/out

export MAKE_FLAGS="ARCH=${ARCH} CC=${CC} LD=${LD} AR=${AR} NM=${NM} OBJCOPY=${OBJCOPY} OBJDUMP=${OBJDUMP} STRIP=${STRIP} CLANG_TRIPLE=${CLANG_TRIPLE} CROSS_COMPILE=${CROSS_COMPILE}"

make -C $(pwd) O=$(pwd)/out a12_defconfig ${MAKE_FLAGS}
make -C $(pwd) O=$(pwd)/out -j8 ${MAKE_FLAGS}

if [ -f "out/arch/arm64/boot/Image" ]; then 
    cp out/arch/arm64/boot/Image $(pwd)/arch/arm64/boot/Image
fi



