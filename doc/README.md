* [Hardware Connections](https://github.com/SherbyRobotics/racecar/blob/ros2/doc/README.md#hardware-connections)
* [USB Hub woes](https://github.com/SherbyRobotics/racecar/blob/ros2/doc/README.md#usb-hub-woes)
* [Steering Offset](https://github.com/SherbyRobotics/racecar/blob/ros2/doc/README.md#steering-offset)
* [Remote Connection (SSH/VNC)](https://github.com/SherbyRobotics/racecar/blob/ros2/doc/README.md#remote-connection-sshvnc)
* [The RaceCar batteries](https://github.com/SherbyRobotics/racecar/blob/ros2/doc/README.md#the-racecar-batteries)
* [Charging the Anker battery](https://github.com/SherbyRobotics/racecar/blob/ros2/doc/README.md#charging-the-anker-battery)
* [Charging the Traxxas battery](https://github.com/SherbyRobotics/racecar/blob/ros2/doc/README.md#charging-the-traxxas-battery)
* [The Killswitch](https://github.com/SherbyRobotics/racecar/blob/ros2/doc/README.md#the-killswitch)
  

# Hardware Connections

  <img src="https://github.com/SherbyRobotics/racecar/blob/master/doc/racecar_connections.jpg" alt="connections" width="800">

**WARNING**: Make sure the Raspberry Pi4 is unpowered when you connect and deconnect the raspicam. The connector is very sensitive and small sparks could happen making the camera unusable. See also this [post](https://stackoverflow.com/questions/31354280/raspberry-camera-error-mmal-no-data-received-from-sensor). Also make sure you **shutdown properly** the Pi4 (`sudo halt` in a terminal or doing a shutdown in the interface) before unplugging the power. If the Pi4 is talking to camera while you unplug the power, **it may damage the camera and your SD-CARD!**

First, get some metric Allen Keys so you can mount your ArduinoMega and your Raspberry Pi4.

* Connect the ArduinoMega under the black PCB. Its USB port must face toward the back of the RaceCar. If you have a transparent base under your Mega, remove it first! It will only get in the way.

   * Connect the grey flat cable in J1 "Controles".
   * Connect the IMU in J4 "I2C".
   * Connect the propulsion encoder in J2 "EncProp".

* Mount your Rapberry Pi4 on its 4 studs. Do not overtighten the screws, they are fragile. Its USB ports must face toward the back of the RaceCar.

   * Connect the flex cable of the RaspiCAM (#6) in J3 "CAMERA". Beware of the orientation.

* Connect the Logitech Controller's USB dongle into the USB hub (#10).
* Connect the USB hub (#10) into one USB-A port of the Pi4.
* USB cable #12: Connect into ArduinoMega USB-B and Pi4 USB-A.
* USB cable #14: Connect into Pi4 USB3 and LIDAR (#2) Micro-USB.
* USB cable #16: Connect into LIDAR (#2) round connector and USB Power Pack (#5) USB-A.

Finally, **CONNECT ONLY WHEN READY TO POWER UP YOUR Raspberry Pi**:

* USB cable #13: Connect into USB Power Pack (#5) USB-A and Pi4 USB-C.

# USB hub woes
If the USB hub doesn't seem to work or is not recognized, try the following first before calling it defective:

1. Some of you will be shocked to learn that quality control on the Raspberry Pi assembly line is not very stringent. Way too many brand new Raspberry Pi are shipped with defective micro SD cards, some even have non-functional USB ports. So, first test the USB ports of your Pi by connecting something else directly on them, for example a USB mouse or a USB keyboard. And make them work.

2. You can see four pushbuttons on the hub, one for each USB port. These power on/off each port. Unless the button is lighted up in blue (the yellow arrow below points to one), the associated USB port is disabled.

    <img src="https://github.com/SherbyRobotics/racecar/blob/master/doc/Sabrent_Hub.jpg" alt="usb_hub" width="400">
    <p align="center"><i>In this picture, the USB port on the right is the only one enabled</i></p>

# Steering Offset
If the RaceCar doesn't move straight when no steering commands are sent, it is possible to adjust the steering mechanically and by software. If the steering offset is small, you may skip the mechanical calibration and just [adjust in software](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#sofware-calibration).

## Mechanical calibration
1. Make sure the Arduino is flashed with the default firmware: [racecar_propulsion_firmware.ino](https://github.com/SherbyRobotics/racecar/tree/master/racecar_arduino/racecar_propulsion_firmware). When the RaceCar's power board is activated, the arduino will send a zero steering value.
2. Unscrew that screw:

    <img src="https://github.com/SherbyRobotics/racecar/blob/master/doc/steering_1.jpg" alt="steering_1" width="400">

3. Adjust the screw indicated by a green circle to make the piece indicated by a red circle be straight:
    
    <img src="https://github.com/SherbyRobotics/racecar/blob/master/doc/steering_2.jpg" alt="steering_2" width="400">

4. Screw back this screw:
    
    <img src="https://github.com/SherbyRobotics/racecar/blob/master/doc/steering_1.jpg" alt="steering_3" width="400">

5. Adjust this screw (called "Toe" suspension adjustement) to make each wheel straight between each other:
    
    <img src="https://github.com/SherbyRobotics/racecar/blob/master/doc/steering_3.jpg" alt="steering_4" width="400">

## Sofware calibration
Adjust this steering offset [here](https://github.com/SherbyRobotics/racecar/blob/c2b9be312659f862fb1f99670c1edd1874b51246/racecar_autopilot/scripts/slash_controller.py#L29). 

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
    
# The RaceCar batteries
* The RaceCar contains two batteries:
    * a 5V Anker battery to power the Raspberry Pi and the LiDAR;
    * a 8.4V Traxxas Ni-Mh battery to power the motors.

# Charging the Anker battery
* To charge the Anker battery, use the small USB connector, plug it in the "input" port, then connect the other side to a computer's powered USB or a phone charger 5V. If the battery is empty, it can take many hours to charge (maybe charge during the night), but it should last long.
    
    <img src="https://github.com/SherbyRobotics/racecar/blob/master/doc/racecar_anker_battery.jpg" alt="anker_battery" width="500">

# Charging the Traxxas battery
* For the Traxxas battery, use the charger coming with the Kit (it may differ from the figure below depending on the version). Make sure the switch of the motor drive is on left ("Charge"). Connect the wires like in the figure below. **To avoid short circuit, make sure to connect the "banana" plugs first in the charger before plugging the other end to the power board (do the reverse when removing the wires after charging!)**. On the charger, make sure the battery type is set to NiMH and the maximum current limit is 2A. Hold « Start » to start charging. When the battery is charged, the charger should stop by itself with a sound. Stop charging manually if it has been charging for more than 1 hour. Normally, the charging time is around 30-40 minutes if the battery is empty.

    <img src="https://github.com/SherbyRobotics/racecar/blob/master/doc/Charging_Traxxas_with_imax_B6.jpg" alt="traxxas_battery" width="500">

* Sometimes, the charger will "charge" the Traxxas battery for like 20 seconds and then decide it is full. In reality, the battery has not been recharged. Wait a minute and try again (Hold Start). Repeat until the charging cycle starts for real (at least 25 minutes or more).

* If it still doesn't want to charge, you must perform a Discharge/Charge cycle. Set the Discharge rate at 100mA and the Charge rate at the usual 2A.

```
If the charger is not already in NiMH Mode:
- Press "Batt. Type" until you get "Program Select NiMH Batt"

To set the charger into Discharge -> Charge Mode:

- Press "Start"
- If the Current is other than 2.0A, press "Start" once and adjust value with "Dec." and "Inc.", then press "Start" once to confirm
- Press "Inc." until you find "NiMH Discharge"
- Press "Start" to adjust values: 0.1A and 1.2V
- Press "Inc." Until you find "NiMH Cycle"
- Press "Start" to adjust cycle to "DCHG>CHG" (discharge, then charge)
- Hold "Start" until the charger commences the cycle. You must hold it for quite a few seconds...
```
    
# The Killswitch

* The RaceCar features a big red mushroom: the killswitch. When pressed, the motor drive is disabled and the car stops moving. This is a hardware killswitch connected directly to the motor drive. It will disable propulsion regardless of software. The killswitch is a « normally close » switch. When you press it, the circuit open.

* When the RaceCar moves around, you have to run after it if you want to make an emergency stop. Make yourself a remote killswitch when you are ready to test drive. The switch is connected into a detacheable header on the motor drive. The simplest remote killswitch is a long loop of wire that you hold in your hand. Should the RaceCar go too far away from you, the detachable header will pop out of its socket (as long as you hold firmly your end), opening the circuit.

    <img src="https://github.com/SherbyRobotics/racecar/blob/master/doc/Killswitch_Header.jpg" alt="Killswitch header" width="500">

    <img src="https://github.com/SherbyRobotics/racecar/blob/master/doc/Simplest_remote_killswitch.jpg" alt="Simplest Killswitch" width="500">
<p align="center"><i>The simplest remote killswitch: a long loop of wire</i></p>

* Alternatively you can dismount the red mushroom from the RaceCar and lenghten its wires so you can hold it in your hand during live tests instead of the simple wire loop.

