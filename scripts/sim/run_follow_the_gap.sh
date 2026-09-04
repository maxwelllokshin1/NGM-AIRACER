#!/bin/bash
# =============================================================
# run_follow_the_gap.sh — Build + launch the follow-the-gap reactive
# driving stack against the sim bridge.
# Run run_gym_bridge.sh FIRST in another terminal.
# Usage: bash ~/scripts/sim/run_follow_the_gap.sh
# =============================================================
set -e

source /opt/ros/humble/setup.bash
cd ~/ros2_workspaces
colcon build --packages-select follow_the_gap
source install/setup.bash

ros2 launch follow_the_gap follow_the_gap.launch.py
