#
# Copyright (C) 2020 Zoltan HERPAI
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=opensbi-d1
PKG_RELEASE:=0.9

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL=https://github.com/riscv/opensbi
PKG_SOURCE_DATE:=2021-11-02
PKG_SOURCE_VERSION:=0979ffda12b636fab58228c2b40bab1e2f6b327b
PKG_MIRROR_HASH:=94e256d246713076329cbd99f43db2c9b038c3adc14fa5ecb53a96a7e195b257

PKG_BUILD_DIR = $(BUILD_DIR)/$(PKG_NAME)-$(BUILD_VARIANT)/$(PKG_NAME)-$(PKG_VERSION)

PKG_TARGETS:=bin
PKG_FLAGS:=nonshared
PKG_LICENSE:=BSD-2-Clause
PKG_LICENSE_FILES:=COPYING.BSD
PKG_BUILD_PARALLEL:=1

PKG_MAINTAINER:=Zoltan HERPAI <wigyori@uid0.hu>

include $(INCLUDE_DIR)/package.mk

define Package/opensbi-d1
    SECTION:=boot
    CATEGORY:=Boot Loaders
    DEPENDS:=@TARGET_sunxid1
    VARIANT:=$(subst _,/,$(subst opensbi-d1_,,$(1)))
    URL:=https://github.com/riscv/opensbi/blob/master/README.md
endef

define Package/opensbi-d1_generic
  $(Package/opensbi-d1)
  TITLE:=OpenSBI generic
  OPENSBI_IMAGE:=fw_dynamic.bin
  OPENSBI_PLATFORM:=generic
endef

export GCC_HONOUR_COPTS=s

MAKE_VARS = \
	CROSS_COMPILE="$(TARGET_CROSS)"

define Build/Compile
	$(eval $(Package/opensbi-d1_$(BUILD_VARIANT))) \
		+$(MAKE_VARS) $(MAKE) -C $(PKG_BUILD_DIR) \
		PLATFORM=$(OPENSBI_PLATFORM)
endef

define Build/InstallDev
	$(INSTALL_DIR) $(STAGING_DIR_IMAGE)
	$(CP) $(PKG_BUILD_DIR)/build/platform/$(OPENSBI_PLATFORM)/firmware/fw_dynamic.bin $(STAGING_DIR_IMAGE)/fw_dynamic-${BUILD_VARIANT}.bin
endef

$(eval $(call BuildPackage,opensbi-d1_generic))
