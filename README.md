Linux Kernel for SHIELD and Tegra Note 7
========================================
This is a temporary project that maintains a version of the Linux kernel that can boot in NVIDIA's SHIELD and TN7. It will be removed once all necessary patches have been approved and merged upstream.

CAUTION: without any kind of thermal control, it is very possible that your device will get considerably hot if you push it. Always check its temperature and in the case of SHIELD, see below how you can enable the fan.

WARNING: this branch will be regularly rebased to the latest upstream and to update the set of patches in progress. Be aware of this if you plan to base your work on it.

What's here
-----------
A (hopefully) up-to-date Linux mainline kernel with a few extra patches that improve support for SHIELD and TN7. These includes missing drivers, custom hacks, special configuration files and embedded firmwares for convenience).

What is supported (all platforms)
---------------------------------
Internal display: supported using tegradrm and a few hacks to keep the panel alive. X should work with both the fbdev (https://gist.github.com/Gnurou/7424833) and opentegra drivers. Backlight is also controllable. It is recommended to not leave the display in any blank mode for an extended period of time as this leaves some (temporary) phantom traces on it, which is possibly bad for its health.

eMMC: internal memory is working, with expected performance. It is recommended to store the system there for serious installations (this will of course bork Android).

Charging: no particular support has been implemented, but it seems like the device is charging by itself when plugged to a USB power source.

USB host: USB devices are working. For TN7, you will need to provide the power externally using a Y-cable or self-powered hub. Device charging while in host mode using a Y cable should be possible, but not yet supported.

UART: retail devices do not have a UART, but if you happen to own a development board, the UART is supported since earlyprintk.

What is supported (SHIELD only)
-------------------------------
Joystick: works perfectly. Can be used as a mouse in X (use https://gist.github.com/Gnurou/6576773).

SD card: working with reasonable performance and (apparently) stable for cards < SDR50. Transcend 32GB SDHC HS Class 10 is working. A Panasonic 32GB UHS-1 displayed I/O errors.

Wifi: working with good performance. Sometimes delayed, probably because of a missing interrupt source. See below for details on how to enable it.

Fan: you can control the fan manually by exporting the PWM.

What's (not yet) supported
--------------------------
HDMI (should be easy to get to work), Audio (again, support should be here already), sensors, battery level, bluetooth.

Touch input will probably never work because of the lack of open-source drivers.

How to compile and boot
-----------------------
Use the tegra\_defconfig configuration and build zImage and dtbs. The kernel configuration uses appended DTB so you can boot the kernel without altering any system partition. To append the DTB to your kernel image:

    $ cat arch/arm/boot/zImage arch/arm/boot/dts/tegra114-roth.dtb >zImage_dtb

(Note for TN7: the dtb file should be tegra114-tn7.dtb instead)

The kernel boots into a ramdisk by default, which will be provided by fastboot. You can obtain a basic Busybox ramdisk here: https://github.com/Gnurou/bbfs

The go into the bootloader to boot the image using fastboot. Your bootloader must be unlocked for this to work:

    $ fastboot boot zImage_dtb /path/to/ramfs.img.gz

Note that doing this will not alter your Android system.

How to boot from a storage device
---------------------------------

Edit `arch/arm/boot/dts/tegra114-roth.dts` and add `root=/dev/mmcblk0p1 rootwait rw` to the bootargs, replacing `mmcblk0p1` with your actual partition (`mmcblk0p1` corresponds to the SD card). Note that if a SD card is inserted at boot time, it will be identified as `mmcblk0`, otherwise the MMC will and a SD card inserted later will become `mmcblk1`.

Then boot without a ramdisk:

    $ fastboot boot zImage_dtb

How to boot into a loopback-mounted root filesystem
---------------------------------------------------

While you want to boot into a full-fledged Linux, you probably also want to preserve your Android installation and still be able to use it. It is possible to do so by storing your root filesystem into a disk image file, and mounting it as the root filesystem.

The creation of the filesystem image depends on your target distribution and is outside the scope of this document ; we will just assume you have a disk image named `linux_root.img` copied under `/sdcard`. You can then use a different Busybox initramfs that will look for this image file, mount it as root, and continue booting from it:

https://github.com/linux-shield/bbinitramfs

Build the ramdisk, and boot it as usual:

    $ fastboot boot zImage_dtb /path/to/ramfs.img.gz

If everything goes well, the distribution in your image file should boot after a few seconds. If it doesn't, check that the `init` script is doing things the way it should.

Enabling Wifi (SHIELD only)
---------------------------
tegra\_roth\_defconfig will automatically include the Wifi driver and necessary firmware files into the kernel, so you don't need to have them on your user filesystem (of course, feel free to compile brcmfmac as a module and copy the firmware files on your root storage if you feel like so).

Wifi needs to be brought out of reset by setting GPIO 229 high:

    $ echo 229 >/sys/class/gpio/export
    $ echo high >/sys/class/gpio/gpio229/direction

Here is the output you should expect on the console:

    mmc0: queuing unknown CIS tuple 0x80 (7 bytes)
    mmc0: new high speed SDIO card at address 0001
    brcmfmac: brcmf_sdio_drivestrengthinit: No SDIO Drive strength init done for chip 4324 rev 2 pmurev 17
    brcmfmac: brcmf_c_preinit_dcmds: Firmware version = wl0: May 17 2013 18:53:00 version 6.10.197.41.200.2 (r403180) FWID 01-8dcc2f96

After that, `wlan0` should show up in ifconfig and an access point can be associated to using `iw` (or your favorite tool).

It may happen that probing does not go on as expected, with wlan0 not showing up. In that case, write 0 into /sys/class/gpio/gpio229/value and then 1 to restart probing until it goes through.

Controlling the fan (SHIELD only)
---------------------------------
There is no proper driver for the fan, but you can activate it by programming the PWM that controls it.

    $ echo 0 >/sys/class/pwm/pwmchip0/export
    $ echo 1 >/sys/class/pwm/pwmchip0/pwm0/enable
    $ echo 256 >/sys/class/pwm/pwmchip0/pwm0/period

At this stage the fan should already start and be quite noisy. You can make it slower (and quieter) by setting a higher duty cycle:

    $ echo 64 >/sys/class/pwm/pwmchip0/pwm0/duty_cycle

Values of 32, 64 and 128 are known to work.

Disclaimer
----------
Tampering with your device is dangerous and you should only do this if you perfectly understand what you are doing. Unlocking your device will void its warranty and exposes it to being permanently broken, so be wise.
