# Restore RaspberryPi image
1. Download the following SD-CARD image for RPI3 or RPI4 (created following this [section](https://github.com/SherbyRobotics/racecar/tree/master/images#create-raspberrypi-image) below):
    * RPI3: [racecar_a21_rpi3_melodic.zip](https://tinyurl.com/RPI3-racecar-image) (3.28GB)
    * RPI4: [racecar_a21_rpi4_noetic.zip](https://tinyurl.com/RPI4-racecar-image) (3.98GB)
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
    $ cd ~/catkin_ws/src/racecar
    $ git pull
    $ cd ~/catkin_ws
    $ catkin_make
    ```
    * Note to have Internet on the RaceCar: disconnect from the Hotspot connection and select your preferred Wi-Fi network. To do so remotely, use VNC with the Ethernet connection, then change the Wi-Fi network.

# Virtual Machine
1. Install [VirtualBox](https://www.virtualbox.org/). Optionally, the VirtualBox Extension Pack can also be installed for USB2-USB3 support.
2. Download Ubuntu 20.04 LTS (Focal Fossa) [64-bit PC (AMD64) desktop image](https://releases.ubuntu.com/focal)
3. Open VirtualBox, create a new image called "Ubuntu 20.04" with at least 4-6 GB of RAM (or 50% of your computer RAM). Click default next options up to hard drive size, which can be set to 20 GB. Before starting the virtual machine, edit its Settings:
    1. System->Processor: set at least 2 to 6 processors (or 50% of your CPUs). 
	2. Network->Adapter 2: Enable it, attached to "Bridged Adapter" with your wireless network adapter. This will be used to connect the virtual machine to ROS on the RaceCar (see [ROS on multiple computers](https://github.com/SherbyRobotics/racecar/tree/master/doc) example).
4. Start the virtual machine, it will ask for an ISO file, select the Ubuntu 20.04 Desktop ISO file previously downloaded. Install Ubuntu with all default settings.
5. After installation, the virtual machine will reboot, connect to your account and in VirtualBox's Devices menu, select "Insert Guest Additions CD Image...", click on "Run" button to install them. After installation, reboot the virtual machine. You can then enable the shared clipboard (Devices->Shared Clipboard) and resize the window as you wish.
6. To install automatically the RaceCar's developement environment, open a terminal and execute thoses commands (make sure the virtual machine has access to Internet):
    ```bash
    $ wget https://raw.githubusercontent.com/SherbyRobotics/racecar/master/images/setup_vm_ubuntu2004_noetic.sh
    $ chmod +x setup_vm_ubuntu2004_noetic.sh
    $ ./setup_vm_ubuntu2004_noetic.sh
    ```
7. When re-opening a new terminal, ROS will be configured. You can then clone the RaceCar's repository:
    ```bash
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/SherbyRobotics/racecar
    $ cd ~/catkin_ws
    $ catkin_make
    ```

# Create RaspberryPi image

1. Install [Ubuntu Mate](https://ubuntu-mate.org/download/) with login name `racecar`:
    1. For RPI3, because of some [issues](https://ubuntu-mate.community/t/ubuntu-mate-20-04-1-first-boot-hung-on-rpi-3/23748/3), download Ubuntu 18.04 armhf (32 bits) image from [here](https://releases.ubuntu-mate.org/archived/bionic/armhf/).
    2. For RPI4, select Ubuntu 20.04 64 bits image for RPI4 ([direct link](https://ubuntu-mate.org/download/arm64/focal/))
3. In Network Manager, edit "Wired connection 1" and rename it to "Wired connection 192.168.10.1":
    1. In IPv4 settings, set connection type to Manual instead of Automatic (DHCP)
    2. Add address `192.168.10.1` with mask `24`, leave Gateaway field empty
    3. Click Save
4. Execute install script:
    1. For RPI3, execute [setup_rpi3_ubuntu1804_melodic.sh](https://github.com/SherbyRobotics/racecar/blob/master/images/setup_rpi3_ubuntu1804_melodic.sh)
        ```bash
        $ wget https://raw.githubusercontent.com/SherbyRobotics/racecar/master/images/setup_rpi3_ubuntu1804_melodic.sh
        $ chmod +x setup_rpi3_ubuntu1804_melodic.sh
        $ ./setup_rpi3_ubuntu1804_melodic.sh
        ```
	    * Download [Arduino IDE 1.8 for Linux ARM 32 bits](https://www.arduino.cc/en/main/software) and install:
            ```bash
            $ cd arduino-1.8.13
            $ sudo ./install.sh
            ```
    2. For RPI4, execute [setup_rpi4_ubuntu2004_noetic.sh](https://github.com/SherbyRobotics/racecar/blob/master/images/setup_rpi4_ubuntu2004_noetic.sh)
        ```bash
        $ wget https://raw.githubusercontent.com/SherbyRobotics/racecar/master/images/setup_rpi4_ubuntu2004_noetic.sh
        $ chmod +x setup_rpi4_ubuntu2004_noetic.sh
        $ ./setup_rpi4_ubuntu2004_noetic.sh
        ```
5. Open Arduino IDE, from Tools->"Manage Libraries..." menu, install `Bolder_Flight_Systems_MPU9250` library
6. Reboot
7. At this point, you can connect by ethernet to RPI by VNC at address 192.168.10.1 or by SSH:
    ```bash
    $ ssh racecar@192.168.10.1
    ```
8. Setup Hotspot
    1. In Network Manager, add new Wi-Fi connection named "Hotspot 10.42.0.1"
    2. Under Wi-Fi tab:
        ```
        SSID: "hotspot_racecar_0" (racecar number)
        Mode: Hotspot
        Device: wlan0
        ```
    3. Under Wi-Fi Security tab:
        ```
        Security: WPA
        Password: racecar0
        ```
    4. If you have still difficulty to connect to Hotspot after rebooting, try changing `Band` to 5 GHz.
    
## Backup/Shrink RaspberryPi image
1. Backup
    ```bash
    $ sudo fdisk -l
    $ sudo dd bs=4M if=/dev/sdb of=racecar.img
    ```

2. Shrink image
    ```bash
    # Ref https://raspberrypi.stackexchange.com/questions/46450/reduce-ubuntu-mate-16-04-img-file-size
    
    # Create loopback
    $ sudo modprobe loop 
    $ sudo losetup -f  
    
    # This will return the path to a free loopback device. This is /dev/loop0 for me
    $ sudo losetup /dev/loop0 racecar.img
    $ sudo partprobe /dev/loop0
    $ sudo gparted /dev/loop0
    
    # In gparted, resize the partition (may require to keep 1-2 GB free to not have errors)
    
    # Close gparted and the loopback
    $ sudo losetup -d /dev/loop0 
    
    # Truncate the image
    $ fdisk -lu racecar.img
    Disk racecar.img: 29.7 GiB, 31914983424 bytes, 62333952 sectors
    Units: sectors of 1 * 512 = 512 bytes
    Sector size (logical/physical): 512 bytes / 512 bytes
    I/O size (minimum/optimal): 512 bytes / 512 bytes
    Disklabel type: dos
    Disk identifier: 0x93df4e89
    
    Device       Boot  Start      End  Sectors  Size Id Type
    racecar.img1        2048   409599   407552  199M  c W95 FAT32 (LBA)
    racecar.img2      409600 25178111 24768512 11.8G 83 Linux
    
    # Note the last value under "End" column
    $ sudo truncate --size=$[(25178111+1)*512] racecar.img
    ```
