#!/bin/bash

set -ex

export ROS_DISTRO=jazzy
export ROS2_DIR=~/ros2_ws


# Install utility packages
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    net-tools \
    nmap \
    htop \
    git \
    wget \
    gpg \
    python3-venv
    # Add your packages here


# Install vscode and extention
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
rm -f packages.microsoft.gpg

sudo apt install -y apt-transport-https
sudo apt update
sudo apt install -y code

code --install-extension platformio.platformio-ide
code --install-extension ms-vscode-remote.vscode-remote-extensionpack
code --install-extension ms-iot.vscode-ros


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
sudo add-apt-repository universe -y

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

# Install Additionnal DDS implementation
sudo apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rmw-cyclonedds-cpp


# Configure racecar's workspace and install ROS2 package dependencies
source /opt/ros/${ROS_DISTRO}/setup.bash
mkdir -p ${ROS2_DIR}/src

cd ${ROS2_DIR}/src
sudo apt-get update
git clone --branch ros2 --depth 1 https://github.com/SherbyRobotics/racecar.git
cd ${ROS2_DIR}
if [ ! -f '/etc/ros/rosdep/sources.list.d/20-default.list' ]; then
    sudo rosdep init --rosdistro=${ROS_DISTRO}
fi
rosdep update --rosdistro=${ROS_DISTRO}
rosdep install --rosdistro=${ROS_DISTRO} --from-paths src --ignore-src -y

# Get the total RAM of the device
get_total_ram () {
  local TOTALRAM=$(cat /proc/meminfo | grep -i 'memtotal' | grep -o '[[:digit:]]*')
  echo $TOTALRAM
}
RAM_SIZE=$(get_total_ram)

if [ $RAM_SIZE < 6291456 ]; then
    echo "Detecting memory-constrained environments using low-memory build strategies"
    export MAKEFLAGS="-j1"
	colcon build --cmake-clean-cache --parallel-workers 1 --symlink-install
    
else
	colcon build --cmake-clean-cache --symlink-install
fi

# Setup ROS2 environment
source ${ROS2_DIR}/install/local_setup.bash
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source ${ROS2_DIR}/install/local_setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc

echo "# use cyclone as default DDS" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
