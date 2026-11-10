#!/bin/sh
# Pull the phone's device tree and regenerate soc.args for shennong (SM8650/pineapple).
# Requires: adb, device connected with USB debugging (and root/su if /sys/firmware/fdt is restricted).
# soc-args.py does not find six of this tree's nodes (AOSS mailbox, command DB, modem region,
# IPA firmware region, SMEM): the six hand-checked values below are re-appended after the run.
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$DIR/../.." && pwd)
TMP=${TMPDIR:-/tmp}/shennong-fdt.dtb
if [ "${1:-}" = "--fdt" ] && [ -n "${2:-}" ]; then
  cp "$2" "$TMP"
else
  adb shell "cat /sys/firmware/fdt" > "$TMP" 2>/dev/null || adb shell su -c "cat /sys/firmware/fdt" > "$TMP"
fi
python3 "$ROOT/tools/new-device.py" shennong --fdt "$TMP" --out "$(dirname "$DIR")/_socgen" --force >/dev/null
cp "$(dirname "$DIR")/_socgen/soc.args" "$DIR/soc.args"
for line in soc_aoss=0xc300000 soc_cmd_db=0x81c60000 soc_mpss_mem=0x8bc00000 soc_mpss_size=0xf400000 modem_ipa_fw_at=0x9b090000 soc_smem=0x81d00000 soc_pas_mpss=0x4; do
  grep -q "^${line%%=*}=" "$DIR/soc.args" || echo "$line" >> "$DIR/soc.args"
done
for line in soc_adsp_mem=0x9df00000 soc_adsp_size=0x4680000 soc_epss=0x17d91000 soc_gmu=0x3d68000 soc_rscc=0x3d50000 soc_pdc=0xb280000 soc_gpucc=0x3d90000 soc_kgsl=0x3d00000 soc_kgsl_smmu=0x3da0000 soc_tsens=0xc229000 soc_tlmm=0xf000000 soc_arb_core=0xc400000 soc_arb_cnfg=0xc42d000 soc_arb_obs=0xc440000 soc_arb_chnls=0xc500000; do
  grep -q "^${line%%=*}=" "$DIR/soc.args" || echo "$line" >> "$DIR/soc.args"
done
echo "wrote $DIR/soc.args (tools/soc-args.py output + the hand-checked values above)"
