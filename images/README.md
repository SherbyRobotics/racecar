# Virtual Machine
1. Install [Ubuntu 18.04 Desktop](https://ubuntu.com/download/alternative-downloads)
2. Execute [setup_vm.sh](https://github.com/SherbyRobotics/racecar/blob/master/images/setup_vm.sh) script
    ```bash
    $ chmod +x setup_vm.sh
    $ setup_vm.sh
    ```
    
# RaspberryPi3 image
1. Install [Ubuntu Mate 18.04 for RPI3](https://ubuntu-mate.org/download/) with login name `racecar`
2. In Network Manager, edit "Wired connection 1" and rename it to "Wired connection 192.168.10.1":
    1. In IPv4 settings, set connection type to Manual instead of Automatic (DHCP)
    2. Add address `192.168.10.1` with mask `24`, leave Gateaway field empty
    3. Click Save
3. Execute [setup_rpi3.sh](https://github.com/SherbyRobotics/racecar/blob/master/images/setup_rpi3.sh) script
    ```bash
    $ wget https://raw.githubusercontent.com/SherbyRobotics/racecar/master/images/setup_rpi3.sh
    $ chmod +x setup_rpi3.sh
    $ ./setup_rpi3.sh
    ```
4. Download [Arduino IDE 1.8 for Linux ARM 32 bits](https://www.arduino.cc/en/main/software)
5. Open Arduino IDE, from Tools->"Manage Libraries..." menu, install `Bolder_Flight_Systems_MPU9250` library
6. Reboot
7. At this point, you can connect by ethernet to RPI3 by VNC at address 192.168.10.1 or by SSH:
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
    5. Under Wi-Fi Security tab:
        ```
        Security: WPA
        Password: racecar
        ```
    
## Backup/Shrink RaspberryPi3 image
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
## Restore RaspberryPi3 image
1. Use [Etcher](https://www.balena.io/etcher/) to flash the image on a SD-CARD (min 16GB).
2. Boot the RPI3. Default username is `racecar` and password is `racecar`. 
3. (Optional) Resize partition to use the full SD-CARD.
    1. Open terminal.
    2. `sudo gparted`, enter password `racecar`.
    3. Right-click on /dev/mmcblk0p2 partition, then click "Resize/Move" in the menu.
    4. Set 0 in "Free space following", then click on "Resize".
    5. Click on "Apply".
    
4. To connect by ethernet or hotspot, as your RPI3 has different hardware, we should update the ethernet and hotspot connection with the right device. In Network Manager (top left), click on "Edit Connections…". 
    1. Edit "Wired connection 192.168.10.1". Under "Ethernet" panel, select device with "eth0", then save.
    2. Edit "Edit "Hotspot 10.42.0.1". Under "Wi-Fi" panel, select device with "wlan0", then save.
