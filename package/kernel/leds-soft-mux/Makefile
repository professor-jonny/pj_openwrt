#
# Copyright (C) 2008-2010 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=leds-soft-mux
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0

include $(INCLUDE_DIR)/package.mk

define KernelPackage/leds-soft-mux
  SUBMENU:=LED modules
  TITLE:=LED soft multiplexer
  FILES:= \
	$(PKG_BUILD_DIR)/leds-soft-mux.ko

  AUTOLOAD:=$(call AutoLoad,66,leds-soft-mux,1)
endef

define KernelPackage/leds-soft-mux/description
  LED soft multiplexer: logical LEDs share physical LEDs by priority
endef

define Build/Compile
	$(KERNEL_MAKE) M="$(PKG_BUILD_DIR)" modules
endef

$(eval $(call KernelPackage,leds-soft-mux))
