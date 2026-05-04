#!/bin/bash
# =============================================================
# start_motor.sh — Launch motor control with Xbox controller
# Run this every time you want to drive
# Usage: bash ~/scripts/start_motor.sh
# =============================================================

echo "===== STARTING MOTOR CONTROL ====="

source /opt/ros/humble/setup.bash
source ~/ros2_workspaces/install/setup.bash

# Fix VESC permissions
sudo chmod 777 /dev/ttyACM0 2>/dev/null

echo ""
echo "This will open the VESC launch in the foreground."
echo "Open a SECOND terminal and run inside the container:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/ros2_workspaces/install/setup.bash"
echo "  sudo -E env PYTHONPATH=\$PYTHONPATH LD_LIBRARY_PATH=\$LD_LIBRARY_PATH python3 ~/controller_bridge.py"
echo ""
echo "Starting VESC nodes..."

sudo -E env PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH PATH=$PATH \
    ros2 launch ~/scripts/motor_test_launch.py
