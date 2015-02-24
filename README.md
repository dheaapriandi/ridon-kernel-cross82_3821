# WIP: Evercoss Elevate Y A66A kernel

This is a work-in-progress. There are still reverse engineering process to be done. Some drivers might be missing or not configured properly.
It could break your device.

1. Adjust PATH in mbldenv to point to your toolchain
1. Do the dance

```
./mk cross82_3821 n k
```

Then:

1. Unpack `boot.img` you already have
2. Repack with the new kernel after the compilation is successfully done
3. Flash both the newly packed `boot.img` along with the scatter file and EBR1 partition produced by step above. You need also to reflash `system.img` and other partitions due to the changes made by the scatter file.
4. Enjoy 

## What's missing
1. Not all peripherals are verified yet.

## Known information
| Subsystem | Driver name | Availability | Working |
|-----------|-------------|--------------|---------|
| LCM driver | `otm1283a_hd720_dsi_vdo`| Yes | Yes |
| Modem | ? | ? | ? |
| Bluetooth | `mt_consys_mt6582` | Yes | ? |
| WiFi | `mt_consys_mt6582` | Yes | ? |
| Touch panel | `GT9XX` | Yes | ? |
| USB | ? | Yes | Yes |
| Vibrator | `vibrator` | Yes | ? |
| GPU | `mali` | Yes | Yes |
| PMIC | `mt6323` | Yes | ? |
| NFC | `mt6605` | Yes | - |
| LED | `mt65xx` | Yes | ? |
| Thermal | `mtk_thermal` | Yes | ? |
| MMC | `emmc` | Yes | Yes |
| NAND | Not configured | - | - |
| RTC | `mtk_rtc_common.c` | Yes | ? |
| Audio | `AudioMTKBTCVSD` | Yes | ? |
| Back camera | `OV8850` | No | ? |
| Front camera | `GC2235` | No | ? |
| Accelerometer | `BM222` | Yes | ? |
| ALS/PS | `em3071` | No | ? |

