#
# Copyright (C) 2021 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

FAT32_BLOCK_SIZE=1024
FAT32_BLOCKS=$(shell echo $$(($(CONFIG_SUNXID1_SD_BOOT_PARTSIZE)*1024*1024/$(FAT32_BLOCK_SIZE))))

KERNEL_LOADADDR:=0x40200000
#KERNEL_LOADADDR:=0x40200000

define Build/riscv-sdcard
	rm -f $@.boot #$(KDIR_TMP)/$(IMG_PREFIX)-$(PROFILE)-boot.img
	mkfs.fat $@.boot  -C $(FAT32_BLOCKS)

	mcopy -i $@.boot $(STAGING_DIR_IMAGE)/$(DEVICE_NAME)-boot.scr ::boot.scr
	mcopy -i $@.boot $(DTS_DIR)/$(DEVICE_DTS).dtb ::dtb
	mcopy -i $@.boot $(IMAGE_KERNEL) ::uImage

	./gen_sunxi_sdcard_img.sh \
		$@ \
		$@.boot \
		$(IMAGE_ROOTFS) \
		$(CONFIG_SUNXID1_SD_BOOT_PARTSIZE) \
		$(CONFIG_TARGET_ROOTFS_PARTSIZE) \
		$(STAGING_DIR_IMAGE)/$(DEVICE_NAME)-u-boot.toc1 \
		$(STAGING_DIR_IMAGE)/boot0_sdcard_sun20iw1p1.bin
endef

define Device/Default
  PROFILES := Default
  KERNEL_NAME := Image
#  KERNEL := kernel-bin | uImage gzip -a $(KERNEL_LOADADDR)
  KERNEL := kernel-bin | uImage
  IMAGES := sdcard.img
  IMAGE/sdcard.img := riscv-sdcard | append-metadata
endef

define Device/FitImageGzip
	KERNEL_SUFFIX := -fit-uImage.itb
	KERNEL = kernel-bin | gzip | fit gzip $$(DTS_DIR)/$$(DEVICE_DTS).dtb
	KERNEL_NAME := Image
endef

define Device/FitImage
	KERNEL_SUFFIX := -fit-uImage.itb
	KERNEL = kernel-bin | fit none $$(DTS_DIR)/$$(DEVICE_DTS).dtb
	KERNEL_NAME := Image
endef

define Device/nezha
#  $(call Device/FitImage)
  DEVICE_VENDOR := Nezha
  DEVICE_MODEL := D1
  DEVICE_DTS := allwinner/sun20i-d1-nezha
  UBOOT := nezha
endef

define Image/Build
	$(call Image/Build/$(1),$(1))
endef

TARGET_DEVICES += nezha

define Device/openv-d100
  KERNEL = kernel-bin | fit none $$(DTS_DIR)/$$(DEVICE_DTS).dtb
  KERNEL_NAME := Image
  DEVICE_VENDOR := OpenV
  DEVICE_MODEL := D100
  DEVICE_DTS := allwinner/sun20i-d1-openv-d100
  UBOOT := openv-d100
endef

TARGET_DEVICES += openv-d100

$(eval $(call BuildImage))
