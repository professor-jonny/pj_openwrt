#
# Copyright (C) 2008-2010 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=leds-group-multicolor
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0

include $(INCLUDE_DIR)/package.mk

define KernelPackage/leds-group-multicolor
  SUBMENU:=LED modules
  TITLE:=Monocromatic group LED support
  FILES:= \
	$(PKG_BUILD_DIR)/leds-group-multicolor.ko
  AUTOLOAD:=$(call AutoProbe,leds-group-multicolor,1)
  #DEPENDS:=
endef

define KernelPackage/leds-group-multicolor/description
  tristate "LEDs group multi-color support"
	This option enables support for monochrome LEDs that are grouped
	into multicolor LEDs which is useful in the case where LEDs of
	different colors are physically grouped in a single multi-color LED
	and driven by a controller that doesn't have multi-color support.
 	(e.g. for TP-Link Deco Status Led)
endef

define Build/Compile
	$(KERNEL_MAKE) M="$(PKG_BUILD_DIR)" modules
endef

$(eval $(call KernelPackage,leds-group-multicolor))
