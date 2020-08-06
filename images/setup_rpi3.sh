#!/usr/bin/env bash

# From Ubuntu Mate 18.04

# Phase 1 - ROS packages

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install -y ros-melodic-desktop-full \
                    ros-melodic-ros-controllers \
                    ros-melodic-joy \
                    ros-melodic-rosserial-python \
                    ros-melodic-imu-filter-madgwick \
                    ros-melodic-robot-localization \
                    ros-melodic-move-base \
                    ros-melodic-global-planner \
                    ros-melodic-teb-local-planner \
                    ros-melodic-ackermann-msgs \
                    ros-melodic-rtabmap-ros \
                    ros-melodic-rosbridge-server  \
                    ros-melodic-web-video-server \
                    ros-melodic-roswww \
                    ros-melodic-rplidar \
                    vim \
                    git \
                    net-tools \
                    nodejs \
                    sed 

# Phase 2 - Workspace setup
source /opt/ros/melodic/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

# specific package(s) compiled from source
git clone https://github.com/UbiquityRobotics/raspicam_node

# Phase 3- ROS Build
cd ~/catkin_ws
catkin_make

# Phase 4 - For automatic sourcing of ROS scripts when starting a new session
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Phase 5 - Install latest Arduino IDE

# Phase 6 - Setup dhcp server

# Phase 7 - Manual steps

# A) Setup minimum resolution for VNC:
# sudo raspi-config
# Select option 5, then A5 (Resolution)
# Choose "1024x768 60 Hz"

# B) To avoid a problem with HDMI->DVI converter
# Edit /boot/config.txt
# In section "chooses between HDMI and DVI modes", set "hdmi_Drive=1". In section "defines screen resolution in CEA or DMT format", uncomment "hdmi_Drive=1".

