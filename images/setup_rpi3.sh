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

sudo apt install -y ros-melodic-rosbash \
                    ros-melodic-ros-controllers \
                    ros-melodic-robot-state-publisher \
                    ros-melodic-rqt \
                    ros-melodic-rqt-image-view \
                    ros-melodic-rqt-reconfigure \
                    ros-melodic-rqt-graph \
                    ros-melodic-image-view \
                    ros-melodic-depth-image-proc \
                    ros-melodic-joy \
                    ros-melodic-gazebo-ros \
                    ros-melodic-camera-info-manager \
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
                    ros-melodic-ros-control \
                    ros-melodic-rosserial-arduino \
                    ros-melodic-rviz-plugin-tutorials \
                    python-rosdep \
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
                    ntp

# ROS Workspace setup
source /opt/ros/melodic/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd ~/catkin_ws
catkin_make

sudo rosdep init
rosdep update

# For automatic sourcing of ROS scripts when starting a new session
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_PARALLEL_JOBS=-j1" >> ~/.bashrc
source ~/.bashrc

# Specific package(s) compiled from source
cd ~/catkin_ws/src
git clone https://github.com/UbiquityRobotics/raspicam_node
git clone https://github.com/Slamtec/rplidar_ros
git clone https://github.com/SherbyRobotics/racecar.git

# ROS Build
cd ~/catkin_ws
catkin_make

# Create ros_lib for arduino
mkdir -p ~/Arduino/libraries
cd ~/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .

# Init SSH keys
sudo dpkg-reconfigure openssh-server
sudo /lib/systemd/systemd-sysv-install enable sshguard
sudo /lib/systemd/systemd-sysv-install enable ssh

# Install VNC server
cd ~/Downloads
wget https://www.realvnc.com/download/file/vnc.files/VNC-Server-6.7.2-Linux-ARM.deb
sudo dpkg -i VNC-Server-6.7.2-Linux-ARM.deb
sudo systemctl start vncserver-x11-serviced.service
sudo systemctl enable vncserver-x11-serviced.service

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

sudo service isc-dhcp-server.service start
sudo service isc-dhcp-server.service enable

# raspi-config: enable camera, fix headless resolution
cp /boot/config.txt config.txt
patch config.txt ~/catkin_ws/src/racecar/images/config.txt.patch
sudo cp config.txt /boot/config.txt

# Add user to 'dialout' group to have permissions on /dev/ttyACM0
sudo adduser $USER dialout

# cleanup cache
sudo apt-get clean

echo "Reboot needed!"

