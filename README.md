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
| Subsystem | Driver name | Availability | Working |
|-----------|-------------|--------------|---------|
| LCM driver #1 | `nt35590_hd720_dsi_vdo_truly` | Yes | No |
| LCM driver #2 | `otm1283a_hd720_dsi_vdo`| No| - |
| LCM driver #3 | `nt35521_hd720_dsi_vdo` | No | - |
| Modem | ? | ? | ? |
| Bluetooth | `mt_consys_mt6582` | Yes | ? |
| WiFi | `mt_consys_mt6582` | Yes | ? |
| Touch panel | `GT9XX` | Yes | ? |
| USB | ? | Yes | Yes |
| Vibrator | `vibrator` | Yes | ? |
| GPU | `mali` | Yes | ? |
| PMIC | `mt6323` | Yes | ? |
| NFC | `mt6605` | Yes | - |
| LED | `mt65xx` | Yes | ? |
| Thermal | `mtk_thermal` | Yes | ? |
| MMC | `emmc` | Yes | No |
| NAND | Not configured | - | - |
| RTC | `mtk_rtc_common.c` | Yes | ? |
| Audio | `AudioMTKBTCVSD` | Yes | ? |

## How can you help
You can help in many ways. Either:
1. Try to get the source code from MediaTek
2. Reverse engineer the original kernel and try to get the names of the drivers, then find the drivers

### How to reverse engineer the kernel
1. Unpack kernel from boot.img
2. Unpack zImage from the kernel image
3. Unpack piggy.gz from the zImage
4. View the contents of the piggy.gz with a text editor and populate ProjectConfig.mk based on the information you can get
