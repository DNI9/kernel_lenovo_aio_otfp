#!/bin/sh

# Inits
PATH=${PATH}:/home/sandeep/Documents/Kernel/UBER_TC/UBERTC-aarch64-linux-android-6.0-kernel-2e6398ac9e86/UBERTC-aarch64-linux-android-6.0-kernel-2e6398ac9e86/bin
BUILD_START=$(date +"%s")

# Exports
export ARCH=arm64
export SUBARCH=arm64
export LOCALVERSION="--ULTIMATE™-V1.2"
export KBUILD_BUILD_USER="Sandeep"
export KBUILD_BUILD_HOST="APRICITY-OS"
make aio_5m_defconfig ARCH=arm64 CROSS_COMPILE=aarch64-linux-android-

# Compiling Process
#make clean && make mrproper
make menuconfig
make -j8 ARCH=arm64 CROSS_COMPILE=aarch64-linux-android-
BUILD_END=$(date +"%s")
DIFF=$(($BUILD_END - $BUILD_START))
echo -e "$Yellow Build completed in $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) seconds.$nocol"
