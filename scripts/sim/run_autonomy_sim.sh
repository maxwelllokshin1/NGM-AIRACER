#!/bin/bash
# =============================================================
# run_autonomy_sim.sh — Build + launch the pure-pursuit + AEB autonomy
# stack against the sim bridge.
# Run run_gym_bridge.sh FIRST in another terminal.
# Usage: bash ~/scripts/sim/run_autonomy_sim.sh
# =============================================================
set -e

source /opt/ros/humble/setup.bash
cd ~/ros2_workspaces
colcon build --packages-select AEB_System movement_node autonomy
source install/setup.bash

ros2 launch autonomy autonomy_sim.launch.py
