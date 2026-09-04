#!/bin/bash
# =============================================================
# run_gym_bridge.sh — Start the f1tenth_gym simulator bridge + RViz
# Run this FIRST, in its own terminal, and leave it running.
# Usage: bash ~/scripts/sim/run_gym_bridge.sh
# =============================================================
set -e

source /opt/ros/humble/setup.bash
source ~/ros2_workspaces/install/setup.bash

# Guard for containers that predate transforms3d being added to the
# Dockerfile (dev target) — see scripts/sim/README.md "Known issues".
python3 -c "import transforms3d" 2>/dev/null || pip3 install --user -q transforms3d

# Interactive map picker, then hands off to gym_bridge_launch.py itself.
python3 ~/scripts/sim/select_map.py
