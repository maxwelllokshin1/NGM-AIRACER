#!/bin/bash
# =============================================================
# run_car_dashboard.sh — Build + launch the live telemetry dashboard.
# Needs a driving stack already publishing /car_info_debug
# (run_follow_the_gap.sh). Opens in-container if a browser is
# installed (rebuild the image to get one); otherwise open the
# printed URL in your host browser — network_mode: host makes it
# reachable at http://localhost:8080 either way.
# Usage: bash ~/scripts/sim/run_car_dashboard.sh
# =============================================================
set -e

# Containers don't reap orphaned background processes on their own — an
# old copy left over from a killed terminal/interrupted run stays bound
# to the port, silently serving its frozen initial state forever while
# looking "up" to curl. Always clear it before starting a new one.
pkill -f car_dashboard.py 2>/dev/null || true
sleep 0.3

source /opt/ros/humble/setup.bash
cd ~/ros2_workspaces
colcon build --packages-select follow_the_gap
source install/setup.bash

ros2 run follow_the_gap car_dashboard
