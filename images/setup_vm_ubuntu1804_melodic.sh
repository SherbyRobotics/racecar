#!/usr/bin/env bash

# From Ubuntu 18.04

# Phase 1 - ROS packages
sudo apt update
sudo apt install -y curl
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install -y ros-melodic-desktop-full \
                    ros-melodic-gazebo-ros \
                    ros-melodic-ros-controllers \
                    ros-melodic-gazebo-ros-control \
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
                    ros-melodic-rviz-plugin-tutorials \
                    vim \
                    git \
                    net-tools \
                    nodejs \
                    sed \
                    ntpdate \
                    ntp

# Phase 2 - Workspace setup
source /opt/ros/melodic/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

# Phase 3- ROS Build
cd ~/catkin_ws
catkin_make

# Phase 4 - For automatic sourcing of ROS scripts when starting a new session
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# VMWare GPU acceleration disabling (optional)
echo "export SVGA_VGPU10=0" >> ~/.bashrc

echo "Installation completed! Open a new terminal to use ROS."
