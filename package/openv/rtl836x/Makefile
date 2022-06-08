#
# Copyright (C) 2008-2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=rtl836x
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0

include $(INCLUDE_DIR)/package.mk

define KernelPackage/rtl836x
  SUBMENU:=Other modules
  TITLE:=RTL836x Driver 
  FILES:=$(PKG_BUILD_DIR)/rtl836x.ko
  AUTOLOAD:=$(call AutoProbe,rtl836x)
  KCONFIG:=
  DEPENDS+= +kmod-swconfig
endef

define KernelPackage/rtl836x/description
 This is a rtl8363 drivers:
endef

MAKE_OPTS:= \
	$(KERNEL_MAKE_FLAGS) \
	CONFIG_RTL836x=m \
	M="$(PKG_BUILD_DIR)"

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
		modules
endef

$(eval $(call KernelPackage,rtl836x))
