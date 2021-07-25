#!/usr/bin/env bash

# From Ubuntu Mate 18.04

# Increase swap size
sudo swapoff -a
sudo dd if=/dev/zero of=/swapfile bs=1M count=512
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install -y ros-noetic-rosbash \
                    ros-noetic-ros-controllers \
                    ros-noetic-robot-state-publisher \
                    ros-noetic-rqt \
                    ros-noetic-rqt-image-view \
                    ros-noetic-rqt-reconfigure \
                    ros-noetic-rqt-graph \
                    ros-noetic-image-view \
                    ros-noetic-depth-image-proc \
                    ros-noetic-joy \
                    ros-noetic-gazebo-ros \
                    ros-noetic-xacro \
                    ros-noetic-camera-info-manager \
                    ros-noetic-rosserial-python \
                    ros-noetic-imu-filter-madgwick \
                    ros-noetic-robot-localization \
                    ros-noetic-move-base \
                    ros-noetic-global-planner \
                    ros-noetic-teb-local-planner \
                    ros-noetic-ackermann-msgs \
                    ros-noetic-rtabmap-ros \
                    ros-noetic-rosbridge-server  \
                    ros-noetic-ros-control \
                    ros-noetic-rosserial-arduino \
                    ros-noetic-rviz-plugin-tutorials \
                    ros-noetic-async-web-server-cpp \
                    python3-rosdep \
                    vim \
                    git \
                    net-tools \
                    nodejs \
                    isc-dhcp-server ufw \
                    openssh-server \
                    wget \
                    libraspberrypi-dev \
                    evince \
                    sed \
                    wireshark-qt \
                    gparted \
                    ntpdate \
                    ntp \
                    python3-rosdep \
                    python-is-python3 \
                    python3-selenium \
                    ffmpeg

# ROS Workspace setup
source /opt/ros/noetic/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd ~/catkin_ws
catkin_make

sudo rosdep init
rosdep update

# For automatic sourcing of ROS scripts when starting a new session
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_PARALLEL_JOBS=-j1" >> ~/.bashrc
source ~/.bashrc

# Specific package(s) compiled from source
cd ~/catkin_ws/src
git clone https://github.com/UbiquityRobotics/raspicam_node
git clone https://github.com/Slamtec/rplidar_ros
git clone https://github.com/tork-a/roswww.git
git clone https://github.com/RobotWebTools/web_video_server.git
git clone https://github.com/SherbyRobotics/racecar.git

# ROS Build
cd ~/catkin_ws
catkin_make

# Install Arduino IDE Version 1.8.15
sudo snap install arduino
# Create ros_lib for arduino
mkdir -p ~/snap/arduino/current/Arduino/libraries
cd ~/snap/arduino/current/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .

# Init SSH keys
sudo dpkg-reconfigure openssh-server
sudo /lib/systemd/systemd-sysv-install enable sshguard
sudo /lib/systemd/systemd-sysv-install enable ssh

# Install VNC server
cd ~/Downloads
wget https://archive.raspberrypi.org/debian/pool/main/r/realvnc-vnc/realvnc-vnc-server_6.7.2.43081_arm64.deb
sudo dpkg -i realvnc-vnc-server_6.7.2.43081_arm64.deb
cd /usr/lib/aarch64-linux-gnu
sudo ln libvcos.so /usr/lib/libvcos.so.0
sudo ln libvchiq_arm.so /usr/lib/libvchiq_arm.so.0
sudo ln libbcm_host.so /usr/lib/libbcm_host.so.0
sudo systemctl enable vncserver-virtuald.service
sudo systemctl enable vncserver-x11-serviced.service
sudo systemctl start vncserver-virtuald.service
sudo systemctl start vncserver-x11-serviced.service

# Setup DHCP server 192.168.10.1
# Make sure the address above is set as IpV4->"Manual" in Network Manager
sudo ufw allow 67/udp
sudo ufw reload

sudo patch /etc/default/isc-dhcp-server ~/catkin_ws/src/racecar/images/isc-dhcp-server.patch
sudo patch /lib/systemd/system/isc-dhcp-server.service ~/catkin_ws/src/racecar/images/isc-dhcp-server.service.patch

sudo bash -c 'echo "subnet 192.168.10.0 netmask 255.255.255.0 {
        option routers                  192.168.10.1;
        option subnet-mask              255.255.255.0;
        option domain-search            \"tecmint.lan\";
        option domain-name-servers      192.168.10.1;
        range   192.168.10.10   192.168.10.100;
        range   192.168.10.110   192.168.10.200;
}" >> /etc/dhcp/dhcpd.conf' 

sudo systemctl start isc-dhcp-server.service
sudo systemctl enable isc-dhcp-server.service

# raspi-config: fix headless 
sudo bash -c 'echo "
# To support headless VNC
hdmi_force_hotplug=1
hdmi_drive=1
hdmi_group=2
hdmi_mode=16" >> /boot/firmware/config.txt' 

# Add user to 'dialout' group to have permissions on /dev/ttyACM0
sudo adduser $USER dialout

# cleanup cache
sudo apt-get clean

echo "Reboot needed!"
