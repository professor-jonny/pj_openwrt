![OpenWrt logo](include/logo.png)

OpenWrt Project is a Linux operating system targeting embedded devices. Instead
of trying to create a single, static firmware, OpenWrt provides a fully
writable filesystem with package management. This frees you from the
application selection and configuration provided by the vendor and allows you
to customize the device through the use of packages to suit any application.
For developers, OpenWrt is the framework to build an application without having
to build a complete firmware around it; for users this means the ability for
full customization, to use the device in ways never envisioned.

Sunshine!

## Download

Built firmware images are available for many architectures and come with a
package selection to be used as WiFi home router. To quickly find a factory
image usable to migrate from a vendor stock firmware to OpenWrt, try the
*Firmware Selector*.

* [OpenWrt Firmware Selector](https://firmware-selector.openwrt.org/)

If your device is supported, please follow the **Info** link to see install
instructions or consult the support resources listed below.

##

An advanced user may require additional or specific package. (Toolchain, SDK, ...) For everything else than simple firmware download, try the wiki download page:

* [OpenWrt Wiki Download](https://openwrt.org/downloads)

## Development

To build your own firmware you need a GNU/Linux, BSD or macOS system (case
sensitive filesystem required). Cygwin is unsupported because of the lack of a
case sensitive file system.

### Requirements

You need the following tools to compile OpenWrt, the package names vary between
distributions. A complete list with distribution specific packages is found in
the [Build System Setup](https://openwrt.org/docs/guide-developer/build-system/install-buildsystem)
documentation.

```
binutils bzip2 diff find flex gawk gcc-6+ getopt grep install libc-dev libz-dev
make4.1+ perl python3.7+ rsync subversion unzip which
```

### Quickstart

1. Run `./scripts/feeds update -a` to obtain all the latest package definitions
   defined in feeds.conf / feeds.conf.default

2. Run `./scripts/feeds install -a` to install symlinks for all obtained
   packages into package/feeds/

3. Run `make menuconfig` to select your preferred configuration for the
   toolchain, target system & firmware packages.

4. Run `make` to build your firmware. This will download all sources, build the
   cross-compile toolchain and then cross-compile the GNU/Linux kernel & all chosen
   applications for your target system.

### Related Repositories

The main repository uses multiple sub-repositories to manage packages of
different categories. All packages are installed via the OpenWrt package
manager called `opkg`. If you're looking to develop the web interface or port
packages to OpenWrt, please find the fitting repository below.

* [LuCI Web Interface](https://github.com/openwrt/luci): Modern and modular
  interface to control the device via a web browser.

* [OpenWrt Packages](https://github.com/openwrt/packages): Community repository
  of ported packages.

* [OpenWrt Routing](https://github.com/openwrt/routing): Packages specifically
  focused on (mesh) routing.

* [OpenWrt Video](https://github.com/openwrt/video): Packages specifically
  focused on display servers and clients (Xorg and Wayland).

## Support Information

For a list of supported devices see the [OpenWrt Hardware Database](https://openwrt.org/supported_devices)

### Documentation

* [Quick Start Guide](https://openwrt.org/docs/guide-quick-start/start)
* [User Guide](https://openwrt.org/docs/guide-user/start)
* [Developer Documentation](https://openwrt.org/docs/guide-developer/start)
* [Technical Reference](https://openwrt.org/docs/techref/start)

### Support Community

* [Forum](https://forum.openwrt.org): For usage, projects, discussions and hardware advise.
* [Support Chat](https://webchat.oftc.net/#openwrt): Channel `#openwrt` on **oftc.net**.

### Developer Community

* [Bug Reports](https://bugs.openwrt.org): Report bugs in OpenWrt
* [Dev Mailing List](https://lists.openwrt.org/mailman/listinfo/openwrt-devel): Send patches
* [Dev Chat](https://webchat.oftc.net/#openwrt-devel): Channel `#openwrt-devel` on **oftc.net**.

## License

OpenWrt is licensed under GPL-2.0

### Fork Information

This Fork is for Wallys devices supporting.
Currently, Following boards are supported (based on Openwrt master (21.02 at the time of editing this)
DR531
DR342
DR344
DR4029/DR4028 Nor+Nand flash.

### Manufacture Instalation Procedures

Below is the information on the uart of the DR4029 board it is necessary to have a serial adaptor to update the partition layout to install OpenWrt initially afterwards you may try to use sysupgade images but I have had the odd brick.

you will need a terminal program such as SmarTTY to update the factory firmware
The serial setting are 115200 8,n,1

you will also need a tftp server for uboot to get the upgrade files onto the device.

![Wallys_DR4029_uart](include/dr4029uart.png)

### How to update openwrt firmware  from factory firmware

1. download the firmware from https://github.com/professor-jonny/images/blob/main/wallys2openwrt.img
2. setup your tftp server and copy the file above into the server directory and start it.
Run the following commands from your uboot terminal: (you can press any key to interrupt boot upon power up of the device)
3. tftpboot 0x84000000 wallys2openwrt.img
4. imgaddr=0x84000000 && source $imgaddr:script
5. reset
tftpboot 0x84000000 wallys2openwrt.img

after above steps , we can run the following command to update openwrt firmware
you can find the latest prebuilt firmware on my repo here https://github.com/professor-jonny/images

1. copy the prebuilt firmware to your tftp server
Run the following commands from your uboot terminal:
2. tftpboot 0x84000000 openwrt-ipq40xx-generic-wallystech_dr40x9-squashfs-nand-factory.ubi
3. nand device 0
4. nand erase 0x0 0x8000000
5. nand write 0x84000000 0x0 0x4000000
6. reset

### update script Information

I have created an update script you can Run `./update_script` to automatically download new sources and feeds and update the config. note this will delete files in your build directory you could stash them or back them up prior to running the update_script if you have made local changes.

### notes

This fork has been set up with my personal settings for me and my family with apps I find useful in my home environment.

The default password is `asdf1234` for both the wifi and root access to OpenWrt.

The dr40x9 device came factory with differing nand and ram sizes and I cant guarantee compatibility with each device as my board may not have the same config as your one my board has 128mb and it works for me.

There is some weirdness in uboot and when an image is flashed proper partition size is not passed or updated and images are not correctly written to the whole nand if they excede the static layout.
the loader was designed for QSDK images and to work around this issue openwrt is only writen to the first 64mb.

Possibly writing a MIBIB partition dump from another model with a larger flash may allow one to extend the partition size by modifying the DTS file.
