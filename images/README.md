# Virtual Machine
1. Install [Ubuntu 18.04 Desktop](https://ubuntu.com/download/alternative-downloads)
2. Execute [setup_vm.sh](https://github.com/SherbyRobotics/racecar/blob/master/images/setup_vm.sh) script
    ```bash
    $ chmod +x setup_vm.sh
    $ setup_vm.sh
    ```
    
# RaspberryPi3 image
1. Install [Ubuntu Mate 18.04 for RPI3](https://ubuntu-mate.org/download/) with login name `racecar`
2. In Network Manager, edit "Wired connection 1":
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
8. Setup Hotspot... todo
    
## Backup/Shrink/Restore RaspberryPi3 image
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

3. Restore with [Etcher](https://www.balena.io/etcher/)
