#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2013 OpenWrt.org

set -ex
[ $# -eq 7 ] || {
    echo "SYNTAX: $0 <file> <bootfs image> <rootfs image> <bootfs size> <rootfs size> <u-boot image> <d1 spl>"
    exit 1
}

OUTPUT="$1"
BOOTFS="$2"
ROOTFS="$3"
BOOTFSSIZE="$4"
ROOTFSSIZE="$5"
UBOOT="$6"
UBOOT_SPL="$7"

ROOTFSPTOFFSET=$(($BOOTFSSIZE + 20 + 1))

head=4
sect=63

set $(ptgen -o $OUTPUT -h $head -s $sect -l 1024 -t c -p ${BOOTFSSIZE}M@20M -t 83 -p ${ROOTFSSIZE}M@${ROOTFSPTOFFSET}M)

BOOTOFFSET="$(($1 / 512))"
BOOTSIZE="$(($2 / 512))"
ROOTFSOFFSET="$(($3 / 512))"
ROOTFSSIZE="$(($4 / 512))"

dd bs=512 if="$UBOOT_SPL" of="$OUTPUT" seek=256 conv=notrunc
dd bs=512 if="$UBOOT" of="$OUTPUT" seek=32800 conv=notrunc
dd bs=512 if="$BOOTFS" of="$OUTPUT" seek="$BOOTOFFSET" conv=notrunc
dd bs=512 if="$ROOTFS" of="$OUTPUT" seek="$ROOTFSOFFSET" conv=notrunc



#dd bs=512 if="$UBOOT_SPL" of="$OUTPUT" seek=256 conv=notrunc
#dd bs=512 if="$UBOOT" of="$OUTPUT" seek=2082 conv=notrunc
#16M + 34
#dd bs=512 if="$BOOTFS" of="$OUTPUT" seek=32802 conv=notrunc
#dd bs=512 if="$ROOTFS" of="$OUTPUT" seek=${ROOTFSOFFSET} conv=notrunc
