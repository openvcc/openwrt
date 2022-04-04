#
# Copyright (C) 2021 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_RELEASE:=1

PKG_VERSION:=2021.10
PKG_HASH:=cde723e19262e646f2670d25e5ec4b1b368490de950d4e26275a988c36df0bd4

include $(INCLUDE_DIR)/u-boot.mk
include $(INCLUDE_DIR)/package.mk

define U-Boot/Default
  BUILD_TARGET:=sunxid1
  BUILD_DEVICES=$(1)
  UBOOT_IMAGE:=u-boot.toc1
  DTS_DIR:=arch/riscv/dts
  UBOOT_DTS:=sun20i-d1-nezha.dtb
  UENV:=default
  DEFAULT:=y
endef

define U-Boot/nezha
  NAME:=Nezha D1
  OPENSBI:=generic
  DEPENDS:=+opensbi-d1_generic +sun20spl
  BUILD_DEVICES:=nezha
endef

UBOOT_TARGETS := \
	nezha

UBOOT_MAKE_FLAGS += \
	OPENSBI=$(STAGING_DIR_IMAGE)/fw_dynamic-${OPENSBI}.bin

define Build/InstallDev
	$(INSTALL_DIR) $(STAGING_DIR_IMAGE)
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/$(DTS_DIR)/$(UBOOT_DTS) $(STAGING_DIR_IMAGE)/$(UBOOT_DTS)

	$(CP) ./files/sunxi_toc1.cfg $(STAGING_DIR_IMAGE)
	sed -i "s:|OPENSBI|:$(STAGING_DIR_IMAGE)/fw_dynamic-$(OPENSBI).bin:" $(STAGING_DIR_IMAGE)/sunxi_toc1.cfg
	sed -i "s:|UBOOTDTB|:$(STAGING_DIR_IMAGE)/$(UBOOT_DTS):" $(STAGING_DIR_IMAGE)/sunxi_toc1.cfg
	sed -i "s:|UBOOTNODTBBIN|:$(PKG_BUILD_DIR)/u-boot-nodtb.bin:" $(STAGING_DIR_IMAGE)/sunxi_toc1.cfg
	$(PKG_BUILD_DIR)/tools/mkimage -T sunxi_toc1 -d $(STAGING_DIR_IMAGE)/sunxi_toc1.cfg $(PKG_BUILD_DIR)/$(UBOOT_IMAGE)

	$(INSTALL_BIN) $(PKG_BUILD_DIR)/$(UBOOT_IMAGE) $(STAGING_DIR_IMAGE)/$(BUILD_VARIANT)-$(UBOOT_IMAGE)

	mkimage -C none -A riscv -T script -d uEnv-$(UENV).txt \
		$(STAGING_DIR_IMAGE)/$(BUILD_DEVICES)-boot.scr
endef

$(eval $(call BuildPackage/U-Boot))
