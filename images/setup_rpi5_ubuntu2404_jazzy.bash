#!/bin/bash

# Download update
sudo apt update && sudo apt upgrade -y

# Configure remote access
sudo apt install -y openssh-server xrdp

# Enable to GPU
sudo usermod -a -G render $USER

# Add swap
sudo fallocate -l 8G /extra-swapfile
sudo chmod 600 /extra-swapfile
sudo mkswap /extra-swapfile
sudo swapon /extra-swapfile
sudo echo "/extra-swapfile swap swap defaults 0 0" | sudo tee -a /etc/fstab

# configure Hotspot
sudo nmcli device wifi hotspot ssid racecar_x password racecar_x ifname wlan0
sudo nmcli connection modify Hotspot connection.autoconnect yes connection.autoconnect-priority 100


# Configure ROS environment
wget https://raw.githubusercontent.com/SherbyRobotics/racecar/ros2/images/setup_vm_ubuntu2404_jazzy.bash
chmod +x setup_vm_ubuntu2404_jazzy.bash
./setup_vm_ubuntu2404_jazzy.bash

sudo reboot