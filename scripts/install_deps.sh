#!/bin/bash
# PerceptionBOT — Phase 0: Install ROS2 Jazzy + Gazebo Harmonic + MoveIt 2
# Run with: bash scripts/install_deps.sh
set -euo pipefail

echo "=== Step 1: Add ROS2 apt repository ==="
sudo apt-get update && sudo apt-get install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "=== Step 2: Install ROS2 Jazzy Desktop Full ==="
sudo apt-get update
sudo apt-get install -y ros-jazzy-desktop-full

echo "=== Step 3: Install Gazebo Harmonic + ROS bridge ==="
sudo apt-get install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-gz-ros2-control-demos

echo "=== Step 4: Install MoveIt 2 ==="
sudo apt-get install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-moveit-task-constructor-capabilities \
  ros-jazzy-moveit-task-constructor-visualization

echo "=== Step 5: Install ROS2 dev tools ==="
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-image-geometry \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-message-filters \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs

echo "=== Step 6: Initialize rosdep ==="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

echo "=== Step 7: Source ROS2 in bashrc ==="
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
  echo "Added ROS2 Jazzy source to ~/.bashrc"
fi

echo ""
echo "=== Installation complete! ==="
echo "Run: source /opt/ros/jazzy/setup.bash"
echo "Then verify: ros2 --help"
