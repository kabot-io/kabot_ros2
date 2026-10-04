#!/usr/bin/env bash
set -euo pipefail

zenbedded_root="${ZENBEDDED_ROOT:-$PIXI_PROJECT_ROOT/../deps/modules/zenbedded}"
if [[ ! -f "$zenbedded_root/zenbedded_hardware_interface/package.xml" ]]; then
  printf 'Zenbedded checkout not found: %s\nSet ZENBEDDED_ROOT to its repository root.\n' "$zenbedded_root" >&2
  exit 1
fi

exec colcon build \
  --paths "$PIXI_PROJECT_ROOT/kabot_robot" "$PIXI_PROJECT_ROOT/twist_stamper" "$zenbedded_root/zenbedded_hardware_interface" \
  --packages-up-to kabot_robot \
  --symlink-install --cmake-args "-DPython3_EXECUTABLE=$(python -c 'import sys; print(sys.executable)')" "$@"