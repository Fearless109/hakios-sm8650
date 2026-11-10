# UEFI for shennong

The Xiaomi 14 Pro's platform for HakiOS's UEFI (`uefi/` of the common repository: Mu-Silicium with HakiOS's changes and banner);
modelled on `redwood` (a copy of `redwoodPkg`, retargeted to SM8650 "pineapple").

What is here, in `files/`:

- `Platforms/Xiaomi/shennongPkg`: the memory map and the driver list (SM8650: DRAM 12GB@0x80000000, SMEM 0x81D00000, AOP CMD 0x81C60000, UEFI FD at 0xA7100000 3MB, panel 1440x3200 framebuffer at 0xFC400000);
- `Resources/Configs/shennong.toml`, `Resources/DTBs/shennong.dtb` (the device tree from the phone);
- `Binaries/shennong`: the vendor's UEFI drivers, **not tracked**: you make this directory yourself from your own phone
  (`uefi/README.md` says how); `files/Binaries/shennong/README.md` is kept: it lists what to fill in.

```sh
devices/shennong/uefi/apply.sh     # = uefi/apply.sh shennong
```

The build and the vendor's drivers: [uefi/README.md](../../../uefi/README.md).
