#!/bin/bash
set -e
set -x

export ROS_DISTRO=jazzy
export ROS2_DIR=/ros2_ws
USERNAME=racecar

# Install utility packages
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    net-tools \
    nmap \
    htop
    # Add your packages here

# === Install ROS ===

# Set locale
sudo apt-get update
sudo apt-get install -y --no-install-recommends locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale # Verify settings

# Enable required repositories
sudo apt-get install -y --no-install-recommends software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG key
sudo apt-get update
sudo apt-get install -y --no-install-recommends curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install dependencies
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    git \
    python3-pip \
    python3-rosdep \
    ros-dev-tools

# Install ROS 2
sudo apt-get upgrade -y
sudo apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-desktop

# === EOF ===

# Configure racecar's workspace and install ROS2 package dependencies
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo mkdir -p ${ROS2_DIR}/src
sudo chown --recursive ${USERNAME}:${USERNAME} ${ROS2_DIR}
cd ${ROS2_DIR}/src
sudo apt-get update
git clone --branch ros2 --depth 1 https://github.com/RobotWebTools/web_video_server.git
git clone --branch ros2 --depth 1 https://github.com/rst-tu-dortmund/costmap_converter.git
git clone --branch ros2-master --depth 1 https://github.com/rst-tu-dortmund/teb_local_planner.git
git clone --branch ros2 --depth 1 https://github.com/SherbyRobotics/racecar.git
cd ${ROS2_DIR}
if [ ! -f '/etc/ros/rosdep/sources.list.d/20-default.list' ]; then
    sudo rosdep init --rosdistro=${ROS_DISTRO}
fi
rosdep update --rosdistro=${ROS_DISTRO}
rosdep install --rosdistro=${ROS_DISTRO} --from-paths src --ignore-src -y
colcon build --cmake-clean-cache && \
source ${ROS2_DIR}/install/local_setup.bash

# Setup ROS2 environment
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source ${ROS2_DIR}/install/local_setup.bash" >> ~/.bashrc

exec "$@"