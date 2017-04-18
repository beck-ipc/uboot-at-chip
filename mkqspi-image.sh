#!/bin/sh
# Construct combined i.MX6 QSPI images
# Takes two parameters:
# 1. QSPI configuration block file
# 2. U-Boot i.MX image file
#
# e.g. ./mkqspi-image.sh qspi-nor-micron-n25q512ax3.imx u-boot.imx

CFGBLOCK=$1
UBOOTIMX=$2


if [ -z $CFGBLOCK ]; then
	echo No configuration file given
	exit 1
fi

if [ -z $UBOOTIMX ]; then
	echo No bootloader file given
	exit 1
fi

if [ ! -r $CFGBLOCK ]; then
	echo No valid configuration file
	exit 1
fi

if [ ! -r $UBOOTIMX ]; then
	echo No valid bootloader file
	exit 1
fi

dd if=/dev/zero of=pad1k bs=1k count=1
cat pad1k ${CFGBLOCK} > tmpimg
dd if=tmpimg of=tmpimg2 bs=4k count=1 conv=sync
cat tmpimg2 ${UBOOTIMX} > qspi-${UBOOTIMX}

rm pad1k tmpimg tmpimg2

echo Wrote output to qspi-${UBOOTIMX}
