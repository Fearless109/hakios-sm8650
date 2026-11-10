#!/bin/bash
# Pull the phone's own firmware out of its Android into the fw-*/ directories at the top of the
# repository (never committed: it is the vendor's). The phone is on adb, in Android.
# Model: Xiaomi 14 Pro (shennong, 23116PN5BC), SM8650, HyperOS V816/OS4.0.0.12.
# Paths verified on HyperOS:
#  GPU  : /vendor/firmware/gen70900_*/gen71100_* (gen7, not a750_* - driver maps via rename)
#  ADSP : /vendor/firmware_mnt/image/adsp.* (b00-b54, 54 files + .mdt)
#  WiFi : /vendor/firmware_mnt/image/kiwi/amss.bin, amss20.bin, bdwlan.e*, bd_*.elf (amss, not wpss.mdt)
#  Modem: /vendor/modem_firmware/image/modem.* (NOT /vendor/firmware_mnt/image/modem.* on HyperOS)
set -e
cd "$(dirname "$0")/../.."
pull() { # DIR SOURCE...  (missing files are told, not fatal)
	local dir=$1; shift
	mkdir -p "$dir"
	for f in "$@"; do adb pull "$f" "$dir/" >/dev/null 2>&1 || echo "pull-firmware.sh: $f: not on the phone"; done
	echo "pull-firmware.sh: $dir: $(ls -1 "$dir" | wc -l | tr -d ' ') files"
}
adb get-state >/dev/null
# GPU: Adreno 750 (gen70900/gen71100_*), from /vendor/firmware (devices/shennong/drivers/gpu/: ADRENO750);
# zap shaders live in /odm/firmware as .mbn (PIL-split into a750_zap.mdt/.b02 below)
pull fw-gpu /vendor/firmware/gen70900_sqe.fw /vendor/firmware/gen71100_gmu.bin /vendor/firmware/gen71100_sqe.fw /vendor/firmware/gmu_gen70900.bin /vendor/firmware/gen70900_aqe.fw
adb pull /odm/firmware/gen71100_zap.mbn fw-gpu/ >/dev/null 2>&1
adb pull /odm/firmware/gen70900_zap.mbn fw-gpu/ >/dev/null 2>&1
# gen71100 = Adreno 750: the names drivers/shennong/drivers/gpu/ asks for (a750_*); the .mbn is PIL-split
python3 -c "
d = open('fw-gpu/gen71100_zap.mbn','rb').read()
open('fw-gpu/a750_zap.mdt','wb').write(d[:148] + d[0x2000:0x2000+3896])
open('fw-gpu/a750_zap.b02','wb').write(d[0x1000:0x1000+1012])
import shutil
shutil.copy('fw-gpu/gen71100_gmu.bin','fw-gpu/a750_gmu.bin')
shutil.copy('fw-gpu/gen71100_sqe.fw','fw-gpu/a750_sqe.fw')
" 2>/dev/null || true
echo "pull-firmware.sh: fw-gpu: $(ls -1 fw-gpu | wc -l | tr -d ' ') files (a750_* mapped from gen71100_*)"
# Wi-Fi + Bluetooth: WCN7850 (kiwi, CNSS, ath11k) firmware -- HyperOS uses amss.bin + bdwlan.e* + bd_*.elf
# driver expects wpss.mdt/wpss.b* - this is a known mismatch, map amss.bin -> wpss.mdt on host side if needed
pull fw-wifi /vendor/firmware_mnt/image/kiwi/amss.bin /vendor/firmware_mnt/image/kiwi/amss20.bin /vendor/firmware_mnt/image/kiwi/Data.msc /vendor/firmware_mnt/image/kiwi/Data20.msc
adb shell "ls /vendor/firmware_mnt/image/kiwi/bd_*.elf /vendor/firmware_mnt/image/kiwi/bdwlan.e* /vendor/firmware_mnt/image/kiwi/phy_ucode*.elf 2>/dev/null" | tr -d '\r' | xargs -r adb pull 2>/dev/null || true
# pull kiwi dir as-is for fidelity
adb pull /vendor/firmware_mnt/image/kiwi fw-wifi/kiwi >/dev/null 2>&1 || true
echo "pull-firmware.sh: fw-wifi: $(ls -1 fw-wifi | wc -l | tr -d ' ') files (amss, not wpss)"
# Audio DSP + battery: HyperOS has adsp.mdt + b00-b54 (54 files)
pull fw-audio /vendor/firmware_mnt/image/adsp.mdt $(adb shell "ls /vendor/firmware_mnt/image/adsp.b* 2>/dev/null" | tr ' ' '\n' | grep -E 'adsp\.b[0-9]+$' | tr -d '\r')
# Modem: HyperOS modem is at /vendor/modem_firmware/image/modem.* (54 files) - NOT firmware_mnt
pull fw-modem $(adb shell "ls /vendor/modem_firmware/image/modem.* 2>/dev/null" | tr ' ' '\n' | grep -E 'modem\.(mdt|b[0-9]+)$' | tr -d '\r')
pull fw-modem $(adb shell "ls /vendor/modem_firmware/image/modem_dtb.* 2>/dev/null" | tr ' ' '\n' | grep -E 'modem_dtb\.(mdt|b[0-9]+)$' | tr -d '\r')
mkdir -p fw-modem/image && adb pull /vendor/modem_firmware/image fw-modem/ >/dev/null 2>&1 && echo "pull-firmware.sh: fw-modem/image: modem TFTP from modem_firmware"
# Cameras (the CCI sensors' tables, pulled by the sensor names the kernels use; on HyperOS
# they live in /odm/lib64/camera, with /vendor/lib64/camera holding only symlinks there)
pull fw-camera $(adb shell ls /odm/lib64/camera/com.qti.sensormodule.shennong_*.bin /odm/lib64/camera/com.qti.tuned.shennong_*.bin /vendor/lib64/camera/com.qti.sensormodule.shennong_*.bin /vendor/lib64/camera/com.qti.tuned.shennong_*.bin 2>/dev/null | tr -d '\r')
echo "pull-firmware.sh: done; Wi-Fi also wants fw-wifi/mac (the radio's address) and fw-wifi/network (the network's name and passphrase)"
