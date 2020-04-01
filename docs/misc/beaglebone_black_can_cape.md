# Setup CAN on Beaglebone Black

CAN/RS485 Cape: https://item.taobao.com/item.htm?spm=a1z09.2.0.0.68b02e8d02l2uT&id=42637485181&_u=a4mg52ma183

* Install latest Debian image from offical site
* Build and install overlays
```
$ sudo apt install device-tree-compiler
$ git clone https://github.com/beagleboard/bb.org-overlays
$ cd ./bb.org-overlays/
$ ./install.sh
```
* Edit /boot/uEnv.txt
```
# 1. Add the following overlay
###Custom Cape
dtb_overlay=/lib/firmware/BB-CAN1-00A0.dtbo

# 2. Disable auto loading of virutal capes (uncomment the two lines)
disable_uboot_overlay_video=1
disable_uboot_overlay_audio=1
```
* Reboot the system