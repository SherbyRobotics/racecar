#!/bin/bash

# Download update
sudo apt update && sudo apt upgrade -y

# Configure remote access
sudo apt install -y openssh-server xrdp

# Add user to groups
sudo usermod -a -G render $USER
sudo usermod -a -G video $USER
sudo usermod -a -G dialout $USER
sudo usermod -a -G input $USER

# Add swap
sudo fallocate -l 8G /extra-swapfile
sudo chmod 600 /extra-swapfile
sudo mkswap /extra-swapfile
sudo swapon /extra-swapfile
sudo echo "/extra-swapfile swap swap defaults 0 0" | sudo tee -a /etc/fstab

# Configure ROS environment
wget https://raw.githubusercontent.com/SherbyRobotics/racecar/ros2/images/setup_vm_ubuntu2404_jazzy.bash
chmod +x setup_vm_ubuntu2404_jazzy.bash
./setup_vm_ubuntu2404_jazzy.bash

# configure Hotspot
sudo nmcli device wifi hotspot ssid racecar_x password racecar_x ifname wlan0
sudo nmcli connection modify Hotspot connection.autoconnect yes connection.autoconnect-priority 100 802-11-wireless.band a

sudo reboot
