#
# Copyright (C) 2008-2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=rtl8821cs
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0

include $(INCLUDE_DIR)/package.mk

define KernelPackage/rtl8821cs
  SUBMENU:=Wireless Drivers
  TITLE:=RTL8821cs wifi Driver 
  FILES:=$(PKG_BUILD_DIR)/8821cs.ko
  AUTOLOAD:=$(call AutoProbe,8821cs)
  KCONFIG:=
  DEPENDS+=
endef

define KernelPackage/rtl8821cs/description
 This is a rtl8821cs drivers:
endef

MAKE_OPTS:= \
	$(KERNEL_MAKE_FLAGS) \
	CONFIG_RTL836x=m \
	M="$(PKG_BUILD_DIR)"

#define Build/Compile
#	$(MAKE) -C "$(LINUX_DIR)" \
#		$(MAKE_OPTS) \
#		modules
#endef

$(info $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(TARGET_CROSS) -C $(LINUX_DIR) M=$(shell pwd)  modules)

define Build/Compile
       $(MAKE) -C $(PKG_BUILD_DIR) KSRC=$(LINUX_DIR) ARCH=riscv64 SRCARCH=riscv CROSS_COMPILE=$(TARGET_CROSS)
endef

$(eval $(call KernelPackage,rtl8821cs))
