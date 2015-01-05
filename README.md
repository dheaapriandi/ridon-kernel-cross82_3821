# WIP: Evercoss Elevate Y A66A kernel

This is a work-in-progress. There are still reverse engineering process to be done. Some drivers might be missing or not configured properly.
It could break your device.

1. Adjust PATH in mbldenv to point to your toolchain
1. Do the dance

```
./mk cross82_3821 n k
```

## What's missing
1. Partition is wrongly populated, e.g. we can't get the /emmc@android link yet
1. Peripherals are not yet verified. Notably the screen is not yet working.

## Known information
1. LCM driver #1: `nt35590_hd720_dsi_vdo_truly` (driver available)
2. LCM driver #2 (?): `otm1283a` (driver not available)
2. Modem: ?
3. Bluetooth: ?
4. WiFi: ?
5. Touch panel: ?
6. USB: ?
7. Vibrator: ?
8. GPU: ?
9. PMIC: ?
10. NFC: none
11. LED: ?
12. Thermal: ?
13. MMC: ?
14. NAND: ?
15. RTC: ?
16. Sound: ?

## How can you help
You can help in many ways. Either:
1. Try to get the source code from MediaTek
2. Reverse engineer the original kernel and try to get the names of the drivers, then find the drivers

### How to reverse engineer the kernel
1. Unpack kernel from boot.img
2. Unpack zImage from the kernel image
3. Unpack piggy.gz from the zImage
4. View the contents of the piggy.gz with a text editor and populate ProjectConfig.mk based on the information you can get
