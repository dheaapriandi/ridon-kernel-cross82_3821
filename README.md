# WIP: Evercoss Elevate Y A66A kernel

This is a work-in-progress. There are still reverse engineering process to be done. Some drivers might be missing or not configured properly.
It could break your device.

1. Adjust PATH in mbldenv to point to your toolchain
1. Do the dance

```
./mk cross82_3821 n k
```

## How can you help
You can help in many ways. Either:
1. Try to get the source code from MediaTek
2. Reverse engineer the original kernel and try to get the names of the drivers, then find the drivers

### How to reverse engineer the kernel
1. Unpack kernel from boot.img
2. Unpack zImage from the kernel image
3. Unpack piggy.gz from the zImage
4. View the contents of the piggy.gz with a text editor and populate ProjectConfig.mk based on the information you can get
