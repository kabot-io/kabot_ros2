#!/usr/bin/env bash
set -e

source "$PIXI_PROJECT_ROOT/install/local_setup.bash"
exec ros2 launch kabot_robot native_sim.launch.py \
  "schema_path:=${KABOT_NATIVE_SIM_SCHEMA:-$PIXI_PROJECT_ROOT/../app/config/zenbedded_native_sim.yaml}" "$@"