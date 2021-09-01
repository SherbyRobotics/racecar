#!/usr/bin/env bash

# From Ubuntu 20.04

# Phase 1 - ROS packages
sudo apt update
sudo apt install -y curl
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install -y ros-noetic-desktop-full \
                    ros-noetic-gazebo-ros \
                    ros-noetic-ros-controllers \
                    ros-noetic-gazebo-ros-control \
                    ros-noetic-joy \
                    ros-noetic-rosserial-python \
                    ros-noetic-imu-filter-madgwick \
                    ros-noetic-robot-localization \
                    ros-noetic-move-base \
                    ros-noetic-global-planner \
                    ros-noetic-teb-local-planner \
                    ros-noetic-ackermann-msgs \
                    ros-noetic-rtabmap-ros \
                    ros-noetic-rosbridge-server  \
                    ros-noetic-rviz-plugin-tutorials \
                    ros-noetic-async-web-server-cpp \
                    vim \
                    git \
                    net-tools \
                    nodejs \
                    sed \
                    ntpdate \
                    ntp \
                    python3-rosdep \
                    python-is-python3 \
                    python3-selenium \
                    ffmpeg

# Phase 2 - Workspace setup
source /opt/ros/noetic/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

# Phase 3- ROS Build empty workspace
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

# Phase 4- ROS Build dependencies
cd ~/catkin_ws/src
git clone https://github.com/tork-a/roswww.git
git clone https://github.com/RobotWebTools/web_video_server.git
sudo rosdep init
rosdep update
cd ~/catkin_ws
rosdep install roswww
rosdep install web_video_server
catkin_make

# Phase 5 - For automatic sourcing of ROS scripts when starting a new session
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# VMWare GPU acceleration disabling (optional)
echo "export SVGA_VGPU10=0" >> ~/.bashrc

echo "Installation completed! Open a new terminal to use ROS."
