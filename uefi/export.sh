#!/bin/bash
# Save the shennong platform from Mu-Silicium/ into files/ here (uefi/export.sh runs this; HakiOS's own changes are uefi/'s).
set -e
cd "$(dirname "$(readlink -f "$0")")"
MU=${MU:-../../../Mu-Silicium}
rm -rf files
mkdir -p files/Resources/Configs files/Resources/DTBs files/Platforms/Xiaomi files/Binaries
cp -r $MU/Platforms/Xiaomi/shennongPkg files/Platforms/Xiaomi/
cp $MU/Resources/Configs/shennong.toml files/Resources/Configs/
cp $MU/Resources/DTBs/shennong.dtb files/Resources/DTBs/
cp -r $MU/Binaries/shennong files/Binaries/
find files -name __pycache__ -prune -exec rm -rf {} +
echo "devices/shennong/uefi/export.sh: $(find files -type f | wc -l) files"
