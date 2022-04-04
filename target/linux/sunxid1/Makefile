#
# Copyright (C) 2021 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk

ARCH:=riscv64
BOARD:=sunxid1
BOARDNAME:=AllWinner D1-based boards
FEATURES:=ext4
KERNELNAME:=Image dtbs
MAINTAINER:=Zoltan HERPAI <wigyori@uid0.hu>

KERNEL_PATCHVER:=5.15

include $(INCLUDE_DIR)/target.mk

define Target/Description
	Build firmware images for AllWinner D1-based boards
endef

DEFAULT_PACKAGES += partx-utils mkf2fs e2fsprogs

$(eval $(call BuildTarget))
