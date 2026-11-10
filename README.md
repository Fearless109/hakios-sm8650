# Xiaomi 14 Pro (shennong, SM8650)

A HakiOS device repository: what is true of this device and of no other. The kernel, the loader, the drivers and the tools are
the common repository's; the `sm8650` family's facts are in `soc/`, carried with the device. How the three are put together:
[docs/porting/repositories.md](https://github.com/oosexgang/hakios-xnu-kernel/blob/main/docs/porting/repositories.md).

## Device

- **Codename:** shennong
- **Model:** Xiaomi 14 Pro (23116PN5BC)
- **SoC:** Qualcomm SM8650 "pineapple" (Snapdragon 8 Gen 3), 8 cores, Adreno 750
- **Panel:** 6.73" 1440x3200 LTPO AMOLED, 120 Hz, DSC cmd
- **Reference bring-up (UniXNU):** <https://github.com/YX2748198202-lab/UniXNU-shennong>

## Repository layout

| File | What |
|---|---|
| `device.conf` | what the device is, and how far each part of it works (status is HakiOS-specific) |
| `configs/shennong_defconfig` | `#include common_defconfig`, `#include sm8650.config`, `#include shennong.config` |
| `configs/shennong.config` | what is this device's own: all phone drivers enabled (common ones with `soc_*` from `soc.args`, device ones where the controller differs) |
| `drivers/` | device drivers, where the controller differs from the reference: `input/hakios.synaptics.c` (Synaptics S3910P), `gpu/hakios.adreno750.c` (Adreno 750), `audio/hakios.speaker_shennong.c` (WCD939x), `haptic/hakios.haptic_shennong.c`, `camera/` (CCI + PM8550B flash) |
| `soc/` | its SM8650 "pineapple" layer (`soc.conf`, `configs/sm8650.config`), carried with this device |
| `soc.args` | the modem/IPA/SoC addresses of this phone, from its device tree (`tools/soc-args.py` + hand checks) |
| `metal.args` | arguments for `tools/metal-run.sh` |
| `uefi/` | the Mu-Silicium platform (`shennongPkg`, FD at 0xA7100000), `apply.sh`/`README.md` |
| `u-boot/` | `files/`: DTS (`sm8650-xiaomi-shennong.dts`), `hakios_shennong_defconfig`, environment |
| `prebuilt/` | `device-tree.dtb` (from the phone's tree), `mkbootimg.py`, `BootShim.bin` |

## The SoC addresses

The common drivers carry the reference phone's (redwood, SM7325) addresses as defaults and take any
that differ from boot-args `soc_*`. The ones this phone differs at are in `soc.args` (modem/IPA
via `tools/soc-args.py` on the phone's own tree, the hand-checked ones from `dts/shennong.dts`):
SMEM at 0x81d00000, command DB at 0x81c60000, ADSP memory at 0x9df00000, EPSS at 0x17d91000,
RSC at 0x17a20000, TSENS at 0xc228000/c229000/c22a000, the PMIC arbiter at 0xc42d000/0xc400000/
0xc440000, TLMM at 0xf000000, GPU GMU at 0x3d68000 (RSCC 0x3d50000, PDC 0xb280000). The UFS
controller stays at 0x1d84000, the apps SMMU at 0x15000000 and the GIC at 0x17100000 (new
platform in `xnuloader/loader.c`).

## GPU / Metal

`hakios-metal/`'s tables are for chip 0x06030500 (Adreno 642L); Adreno 750 is refused there and
the window server stays on OpenGL ES through Mesa (which supports A750). The kernel wakes the
A750 with `HAKIOS_ADRENO750` (the `gpu=1` boot-arg) from this phone's `a750_*` firmware
(`devices/shennong/pull-firmware.sh`). Porting Metal proper to A750 is the remaining GPU step
(`hakios-metal/include/hakios-metal/a6xx.hpp`).

## Status

Every part is `partial`: the port (drivers, addresses, firmware paths and the UEFI/U-Boot
platform) is written for SM8650, and nothing is marked `works` until it has been seen on this
device under HakiOS. UniXNU-shennong (above) has reached UEFI display and XNU early boot with
UFS on real hardware; HakiOS validation re-runs each driver on this phone one at a time
(docs/devices/shennong.md records what has been seen).

Check with `tools/hakios-device.py check shennong`.
Build the kernel with `make shennong_defconfig && make kernel`; start it with `./xl` (docs/windows.md).
