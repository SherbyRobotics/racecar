# Restore RaspberryPi image
1. Download the following SD-CARD image for RPI5:
    * RPI5: [racecar_ros2_ubuntu24](https://usherbrooke.sharepoint.com/:u:/s/genie-robotique-montage/EdZ4YGHksDpNmYVVE7WL4kUB_faGdlo3MOQZyPbNS8iuTQ?e=JYxpvC) (5.7GB)
2. Use [Etcher](https://www.balena.io/etcher/) to flash the image on a SD-CARD (min 16GB). 
3. Boot the RPI with HDMI, a mouse and a keyboard connected. Default username is `racecar` and password is `racecar`. 
4. (Optional) Resize partition to use the full SD-CARD.
    1. Open terminal.
    2. `sudo gparted`, enter password `racecar`.
    3. Right-click on the main partition (`/dev/mmcblk0p2`), then click "Resize/Move" in the menu.
    4. Set 0 in "Free space following", then click on "Resize".
    5. Click on "Apply".
    
5. To [connect by ethernet or hotspot](https://github.com/SherbyRobotics/racecar/tree/master/doc), as your RPI has different hardware, you may have to update the ethernet and hotspot connections with the right device if not working already out-of-the-box. In Network Manager (top right), click on "Edit Connections…". 
    1. Edit "Wired connection 192.168.10.1". Under "Ethernet" panel, select device with "eth0", then save. Remove "Wired connection 1" if it exists.
    2. Edit "Hotspot 10.42.0.1". Under "Wi-Fi" panel, select device with "wlan0", then save. Change SSID name of the connection with ID of your racecar. The default password `racecar0` can also be changed. 

6. Make sure to have latest code. Normally we would do:
    ```bash
    $ cd ~/ros2_ws/src/racecar
    $ git pull
    $ cd ~/ros2_ws
    $ colcon build
    ```
    * Note to have Internet on the RaceCar: disconnect from the Hotspot connection and select your preferred Wi-Fi network. To do so remotely, use VNC with the Ethernet connection, then change the Wi-Fi network.


# Importing a Virtual Machine

1. Install [VirtualBox](https://www.virtualbox.org/). Optionally, the VirtualBox Extension Pack can also be installed for USB2-USB3 support.
2. Click here to [download the default virtual machine](https://usherbrooke-my.sharepoint.com/:u:/g/personal/lali3401_usherbrooke_ca/ET_KS0N6qFpPjCdlN4sOIrYBsIMreMN1X6O5qblisWM_Rw?e=6nY1pi),
3. Open VirtualBox the import the VM using the file->import option. Then select the VM and edit its Settings:
    1. System->Processor: set at least 2 to 6 processors (or 50% of your CPUs). 
	2. Network->Adapter 2: Enable it, attached to "Bridged Adapter" with your wireless network adapter. This will be used to connect the virtual machine to ROS on the RaceCar.
4. Start the VM

---
**NOTE**

To see the topic publish by the racecar using a VM you need the VM need to be mount with a Bridged Adapter to your wifi card if you are for exemple connected to your racecar using the hotspot 

---

# Creating a Virtual Machine
If your are configuring a native Ubuntu 24.04 computer skip to step 6. If you want to create a Virtual machine from scratch follow all the steps : 

1. Install [VirtualBox](https://www.virtualbox.org/). Optionally, the VirtualBox Extension Pack can also be installed for USB2-USB3 support.
2. Download Ubuntu 24.04.2 LTS (Noble Numbat) [64-bit PC (AMD64) desktop image](https://www.releases.ubuntu.com/noble/)
3. Open VirtualBox, create a new image called "Ubuntu 24.04" with at least 4-6 GB of RAM (or 50% of your computer RAM). Click default next options up to hard drive size, which can be set to 20 GB. Before starting the virtual machine, edit its Settings:
    1. System->Processor: set at least 2 to 6 processors (or 50% of your CPUs). 
	2. Network->Adapter 2: Enable it, attached to "Bridged Adapter" with your wireless network adapter. This will be used to connect the virtual machine to ROS on the RaceCar (see [ROS on multiple computers](https://github.com/SherbyRobotics/racecar/tree/master/doc) example).
4. Start the virtual machine, it will ask for an ISO file, select the Ubuntu 24.04 Desktop ISO file previously downloaded. Install Ubuntu with all default settings.
5. After installation, the virtual machine will reboot, connect to your account and in VirtualBox's Devices menu, select "Insert Guest Additions CD Image...", click on "Run" button to install them. After installation, reboot the virtual machine. You can then enable the shared clipboard (Devices->Shared Clipboard) and resize the window as you wish.
6. To install automatically the RaceCar's developement environment, open a terminal and execute thoses commands (make sure the virtual machine has access to Internet):
    ```bash
    $ wget https://raw.githubusercontent.com/SherbyRobotics/racecar/ros2/images/setup_vm_ubuntu2404_jazzy.bash
    $ chmod +x setup_vm_ubuntu2404_jazzy.bash
    $ ./setup_vm_ubuntu2404_jazzy.bash
    ```

# Setup RPI5 from scratch

1. Download the official ubuntu 24.04 image for RPI:
    * RPI5 and RPI4: [ubuntu24.04](https://ubuntu.com/download/raspberry-pi)
2. Use [Etcher](https://www.balena.io/etcher/) to flash the image on a SD-CARD (min 16GB). 

3. Boot the RPI with HDMI, a mouse and a keyboard connected. Create a user with name `racecar` and password is `racecar`.

4. Make sure the RPI have internet acces.

5. Download and launch the install script for RPI.

    ```bash
    $ wget https://raw.githubusercontent.com/SherbyRobotics/racecar/ros2/images/setup_rpi5_ubuntu2404_jazzy.bash
    $ chmod +x setup_rpi5_ubuntu2404_jazzy.bash
    $ ./setup_rpi5_ubuntu2404_jazzy.bash
    ```
