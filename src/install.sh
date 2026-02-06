#!/usr/bin/env bash
set -e

echo "====================================="
echo " YOLO Keypoint ROS2 Project Installer "
echo "====================================="

# Check OS
if ! grep -qi ubuntu /etc/os-release; then
  echo "ERROR: Ubuntu 22.04 is required."
  exit 1
fi

echo "[1/9] Updating system..."
sudo apt update && sudo apt upgrade -y

echo "[2/9] Installing ROS 2 Humble + core tools..."
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  gazebo \
  ros-humble-gazebo-ros-pkgs

echo "[3/9] Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update

echo "[4/9] Installing ROS message dependencies..."
sudo apt install -y \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-visualization-msgs \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-std-msgs \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime

echo "[5/9] Installing Python dependencies..."
pip3 install --upgrade pip

REQ_FILE=""
if [ -f requirements.txt ]; then
  REQ_FILE="requirements.txt"
elif [ -f yolo_keypoint_sim/requirements.txt ]; then
  REQ_FILE="yolo_keypoint_sim/requirements.txt"
elif [ -f ros2_ws/src/yolo_keypoint_sim/requirements.txt ]; then
  REQ_FILE="ros2_ws/src/yolo_keypoint_sim/requirements.txt"
fi

if [ -n "$REQ_FILE" ]; then
  echo "Using requirements file: $REQ_FILE"
  pip3 install -r "$REQ_FILE"
else
  echo "WARNING: requirements.txt not found"
fi

echo "[6/9] Applying WSL FastDDS fix..."
if ! grep -q RMW_FASTRTPS_USE_SHM ~/.bashrc; then
  echo "export RMW_FASTRTPS_USE_SHM=0" >> ~/.bashrc
fi

echo "[7/9] Sourcing ROS environment..."
if ! grep -q "/opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
source /opt/ros/humble/setup.bash

echo "[8/9] Creating ROS2 workspace (if missing)..."
mkdir -p ros2_ws/src

if [ -d yolo_keypoint_sim ]; then
  if [ ! -d ros2_ws/src/yolo_keypoint_sim ]; then
    echo "Linking package into ros2_ws/src/"
    ln -s "$(pwd)/yolo_keypoint_sim" ros2_ws/src/yolo_keypoint_sim
  fi
fi

echo "[9/9] Building ROS2 workspace..."
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

echo "====================================="
echo " INSTALLATION COMPLETE "
echo " Open a NEW terminal before running "
echo "====================================="
