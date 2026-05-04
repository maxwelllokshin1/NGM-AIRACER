#!/bin/bash
# =============================================================
# setup_motor.sh — One-time setup for motor control inside container
# Run this ONCE after a fresh docker-compose up -d
# Usage: bash ~/scripts/setup_motor.sh
# =============================================================
set -e

echo "===== MOTOR CONTROL SETUP ====="

# Source ROS
source /opt/ros/humble/setup.bash

cd ~/ros2_workspaces/src

# Clone packages if not already present
if [ ! -d "vesc" ]; then
    echo "[1/3] Cloning vesc (ros2 branch)..."
    git clone -b ros2 https://github.com/f1tenth/vesc.git
else
    echo "[1/3] vesc already cloned, skipping."
fi

if [ ! -d "ackermann_mux" ]; then
    echo "[2/3] Cloning ackermann_mux..."
    git clone https://github.com/f1tenth/ackermann_mux.git
else
    echo "[2/3] ackermann_mux already cloned, skipping."
fi

if [ ! -d "teleop_tools" ]; then
    echo "[3/3] Cloning teleop_tools..."
    git clone https://github.com/f1tenth/teleop_tools.git
else
    echo "[3/3] teleop_tools already cloned, skipping."
fi

if [ -d "f1tenth_system" ]; then
    echo "Cleaning empty dirs in f1tenth_system..."
    rm -rf f1tenth_system/vesc f1tenth_system/ackermann_mux f1tenth_system/teleop_tools 2>/dev/null || true
fi

# Install apt deps
echo "Installing apt dependencies..."
sudo apt-get update -qq
sudo apt-get install -y -qq ros-humble-udp-msgs ros-humble-asio-cmake-module \
    ros-humble-serial-driver ros-humble-io-context > /dev/null
sudo pip3 install pyusb -q

# Clean stale local builds of system packages
rm -rf ~/ros2_workspaces/build/io_context ~/ros2_workspaces/install/io_context
rm -rf ~/ros2_workspaces/build/serial_driver ~/ros2_workspaces/install/serial_driver

cd ~/ros2_workspaces

# Build in order
echo "Building vesc_msgs..."
colcon build --packages-select vesc_msgs
source install/setup.bash

echo "Building vesc_driver vesc_ackermann vesc..."
colcon build --packages-select vesc_driver vesc_ackermann vesc
source install/setup.bash

echo "Building ackermann_mux..."
colcon build --packages-select ackermann_mux
source install/setup.bash

echo "Building teleop_tools_msgs joy_teleop..."
colcon build --packages-select teleop_tools_msgs joy_teleop
source install/setup.bash

echo "Building f1tenth_stack..."
colcon build --packages-select f1tenth_stack
source install/setup.bash

# Fix VESC port
VESC_YAML=~/ros2_workspaces/src/f1tenth_system/f1tenth_stack/config/vesc.yaml
if grep -q "/dev/sensors/vesc" "$VESC_YAML" 2>/dev/null; then
    echo "Fixing VESC port to /dev/ttyACM0..."
    sed -i 's|port: /dev/sensors/vesc|port: /dev/ttyACM0|' "$VESC_YAML"
    colcon build --packages-select f1tenth_stack
    source install/setup.bash
fi

echo ""
echo "===== SETUP COMPLETE ====="
echo "Now run: bash ~/scripts/start_motor.sh"
