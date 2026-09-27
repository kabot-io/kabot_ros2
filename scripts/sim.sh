#!/usr/bin/env bash
set -e

source "$PIXI_PROJECT_ROOT/install/local_setup.bash"
exec ros2 launch kabot_robot gazebo.launch.py "$@"