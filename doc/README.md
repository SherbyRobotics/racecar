* [Hardware Connections](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#hardware-connections)
* [Remote Connection (SSH/VNC)](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#remote-connection-sshvnc)
* [ROS on multiple computers (ROS_IP)](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#ros-on-multiple-computers-ros_ip)
* [Simulated environment (Gazebo)](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#simulated-environment-gazebo)
* [Recharging the RaceCar](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#recharging-the-racecar)
* [The Killswitch](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#the-killswitch)

# Hardware Connections

![](https://github.com/SherbyRobotics/racecar/blob/master/doc/racecar_connections.jpg "connections" )

# Remote Connection (SSH/VNC)

First, make sure the ethernet and hotspot interfaces are properly configured on your Raspberry Pi (see Step 5 of [this section](https://github.com/SherbyRobotics/racecar/tree/master/images#restore-raspberrypi3-image)). The default login is `racecar` with password `racecar`.

 * SSH (command line):
     ```bash
     # By ethernet:
     $ ssh racecar@192.168.10.1
    
    # By Hotspot
    $ ssh racecar@10.42.0.1
    ```
    
 * [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/) (Remote Desktop):
   * By ethernet: set IP to `192.168.10.1`
   * By Hotspot: set IP to `10.42.0.1`
   * Disable encryption and open the connection.
    
# ROS on multiple computers (ROS_IP)

If you have ROS on your laptop (ubuntu native or in a virtual machine), to make sure to receive all messages in both directions, set `ROS_IP` environment variable on both the Raspberry Pi and the laptop. Here is an example based on the [main example](https://github.com/SherbyRobotics/racecar#launch) where we will launch RVIZ on the remote computer instead of the Raspberry Pi:
* From the laptop, connect to RaceCar by its Hotspot (using SSH for this example, but VNC can also be used)
    ```bash
    (Laptop) $ ssh racecar@10.42.0.1
    (RPI3)   $ export ROS_IP=10.42.0.1
    (RPI3)   $ roslaunch racecar_bringup teleop.launch
    ```
* On another terminal on the laptop (use `ifconfig` to get your laptop IP `10.42.0.###`):
    ```bash
    $ export ROS_MASTER_URI=https://10.42.0.1:11311
    $ export ROS_IP=10.42.0.###
    $ roslaunch racecar_bringup rviz.launch
    ```
    
# Simulated environment (Gazebo)

Without the actual RaceCar, it is still possible to develop using the RaceCar's simulator on your computer ([with dualboot Ubuntu or in a virtual machine](https://github.com/SherbyRobotics/racecar/tree/master/images#virtual-machine)). To do so, we provide a simulated RaceCar for the Gazebo simulator. Launch the simulator:

```bash
$ roslaunch racecar_gazebo racecar_tunnel.launch joy:=js0
```
    
If you have joystick errors, try js1 or js2. If you are using a virtual machine, make sure the USB device is redirected to the virtual machine (for VirtualBox: Menu->Devices->USB...).
    
Like with the real RaceCar, you can launch rviz like this:

```bash
$ roslaunch racecar_bringup rviz.launch
```
    
What you should be seeing (on left is the simulator, on right is RVIZ):
![](https://github.com/SherbyRobotics/racecar/blob/master/doc/gazebo.jpg "gazebo" )
    
# Recharging the RaceCar
* The RaceCar contains two batteries:
    * a 8.4V Traxxas battery to power the motors and
    * a 5V Anker battery to power the Raspberry Pi and the LiDAR.

* To charge the Anker battery, use the small USB connector, plug it in the "input" port, then connect the other side to a computer's powered USB or a phone charger 5V. If the battery is empty, it can take many hours to charge (maybe charge during the night), but it should last long.
    
    ![](https://github.com/SherbyRobotics/racecar/blob/master/doc/racecar_anker_battery.jpg "anker_battery" )
    
* For the Traxxas battery, use the charger coming with the Kit (it may differ from the figure below depending on the version). Make sure the switch of the power board is on left ("Charge"). Connect the wires like in the figure below. **To avoid short circuit, make sure to connect the "banana" plugs first in the charger before plugging the other end to the power board (do the reverse when removing the wires after charging!)**. On the charger, make sure the battery type is set to NiMH and the maximum current limit is 3A. Hold « Start » to start the charging. When the battery is charged, the charger would stop by itself with a sound. Stop the charging manually if it has been charging for more than 1 hour. Normally, the charging time is around 30-40 minutes if the battery is empty. 
    
    ![](https://github.com/SherbyRobotics/racecar/blob/master/doc/Charging_Traxxas_with_imax_B6.jpg "traxxas_battery" )

# The Killswitch

* The RaceCar features a big red mushroom: the killswitch. When pressed, the motor drive is disabled and the car stops moving. This is a hardware killswitch connected directly to the motor drive. It will disable propulsion regardless of software.

* As it is mounted on the car, you can use it to make sure the car does not start moving unexpectedly. But when the RaceCar moves around, you will have to run after it if you want to make an emergency stop. Make yourself a remote killswitch.
