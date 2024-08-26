# Restore RaspberryPi image
1. Download the following SD-CARD image for RPI4 (created following this [section](https://github.com/SherbyRobotics/racecar/tree/master/images#create-raspberrypi-image) below):
    * RPI4: [racecar_ros2_ubuntu22 1](https://tinyurl.com/RacecarRos2Ubuntu2204) (5.05GB)
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
