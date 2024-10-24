#!/bin/bash

# set distro
export ROS_DISTRO=humble

#################### install utils ############################################

sudo apt update
sudo apt install net-tools -y
sudo apt install nmap -y 
sudo apt install htop -y

#################### install ROS ############################################


# locale  # check for UTF-8
sudo apt update && sudo apt install locales -y 
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-dev-tools -y

sudo apt update
sudo apt upgrade

sudo apt install ros-${ROS_DISTRO}-desktop -y

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

#################### setup repos ############################################

cd ~/.
mkdir -p ros_ws/src
cd ~/ros_ws/src

git clone -b ros2 https://github.com/chameau5050/web_video_server
git clone -b ros2 https://github.com/rst-tu-dortmund/costmap_converter.git
git clone -b ros2-master https://github.com/rst-tu-dortmund/teb_local_planner.git
git clone -b ros2 https://github.com/SherbyRobotics/racecar.git

cd ~/ros_ws

sudo rosdep init
rosdep update 
rosdep install --rosdistro=${ROS_DISTRO} --from-paths src --ignore-src -y
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install

echo "source ~/ros_ws/install/setup.bash" >> ~/.bashrc


