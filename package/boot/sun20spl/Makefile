#
# Copyright (C) 2020 Zoltan HERPAI
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=sun20spl
PKG_RELEASE:=1

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL=https://github.com/smaeul/sun20i_d1_spl
PKG_SOURCE_DATE:=2021-10-28
PKG_SOURCE_VERSION:=771192d0b3737798d7feca87263c8fa74a449787
PKG_MIRROR_HASH:=8a2b26661569b6d3bce012721fa9bdc3036c3dead625e8a63c7f58022390e470

PKG_BUILD_DIR = $(BUILD_DIR)/$(PKG_NAME)-$(BUILD_VARIANT)/$(PKG_NAME)-$(PKG_VERSION)

PKG_TARGETS:=bin
PKG_FLAGS:=nonshared
PKG_LICENSE:=GPL-2.0
PKG_LICENSE_FILES:=COPYING.BSD
PKG_BUILD_PARALLEL:=1

PKG_MAINTAINER:=Zoltan HERPAI <wigyori@uid0.hu>

include $(INCLUDE_DIR)/package.mk

define Package/sun20spl
    SECTION:=boot
    CATEGORY:=Boot Loaders
    DEPENDS:=@TARGET_sunxid1
    TITLE:=SPL for sun20 (D1)
    URL:=https://github.com/smaeul/sun20i_d1_spl/blob/mainline/README
endef

export GCC_HONOUR_COPTS=s

MAKE_VARS = \
	CROSS_COMPILE="$(TARGET_CROSS)"

define Build/Compile
	$(MAKE_VARS) $(MAKE) -C $(PKG_BUILD_DIR) p=sun20iw1p1 mmc
endef

define Build/InstallDev
	$(INSTALL_DIR) $(STAGING_DIR_IMAGE)
	$(CP) $(PKG_BUILD_DIR)/nboot/boot0_sdcard_sun20iw1p1.bin $(STAGING_DIR_IMAGE)
endef

$(eval $(call BuildPackage,sun20spl))
