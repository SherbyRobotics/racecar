# racecar
Software sources for the UdeS 1/10th autonomous car platform

## Table of contents
* [Technologies](#technologies)
* [Requirements](#requirements)
* [How it works](#how-it-works)
* [Launch](#launch)
* [Controller Modes](#controller-modes)

## Technologies
The project is created with:
* Python 2.7 or 3.6
* Arduino programming language
* [ROS](http://wiki.ros.org/ROS/Installation)

## Requirements
To be able to test our code, there are a few requirements.

### Hardware
* [RaceCar](https://cad.onshape.com/documents/9d3f435f340b50b281de3ac4/w/60d94a6915ed0711b2290521/e/45c11c7ee9e7e6dfaec5c7e5) of the Université de Sherbrooke
* Logitech Wireless Gamepad F710 (DirectInput Mode)

### Software
* Ubuntu 18.04 
* Main dependencies (baseline):
	* [rosserial arduino](http://wiki.ros.org/rosserial_arduino)
	* [rosserial](http://wiki.ros.org/rosserial)
	* [joy](http://wiki.ros.org/joy)
	* MPU  --> TODO Mettre le lien vers la librairie arduino nécessaire pour compiler le firmware avec l'IMU
* Other dependencies (all fonctionnalities):
	* [angle](http://wiki.ros.org/angles)
	* [cv-camera](http://wiki.ros.org/cv_camera)
	* [cv-bridge](http://wiki.ros.org/cv_bridge)

### Quick installation guide for ROS packages
```
sudo apt-get install ros-<distro>-rosserial-arduino
sudo apt-get install ros-<distro>-rosserial
sudo apt-get install ros-<distro>-joy
sudo apt-get install ros-<distro>-angles
sudo apt-get install ros-<distro>-cv-camera
sudo apt-get install ros-<distro>-cv-bridge
```	
## How it works
![](https://github.com/SherbyRobotics/racecar/blob/master/images/Racecar_rqt_graph_teleop.png "teleop" )
In the graph, you can see a representation of the communication between the nodes and the topics on the base launch file `teleop.launch`. 

The RaceCar has many operating modes that allow you to test different functionnalities. The ROS controller Node, which control commands to the motor and the steering servo, has multiple high-level mode that are selected by joystick inputs (see [Controller modes](#controller-modes) table for more details). For instance, open-loop, closed-loop in velocity, closed-loop in position modes are available for both the propulsion and the steering.  The Arduino has also multiple internal low-level modes (different than the high-level ones!!!) that are selected by the ROS controller Node. Empty operating mode templates are available in the source code for you to test custom control modes.

## Launch
1. Connect the Arduino mega 2560 of the RaceCar to the computer.
2. Flash the Arduino mega 2560 with the firmware file `racecar_propulsion_firmware.ino` (see [arduino firmware](https://github.com/SherbyRobotics/racecar/tree/master/racecar_arduino/racecar_propulsion_firmware)).
3. Reconnect the Arduino mega 2560 to the USB-hub of the RaceCar.
4. Connect the Raspberry Pi 3 to the the battery.
5. Connect the Raspberry Pi 3 to a monitor with a HDMI cable.
6. With the help of a keyboard, open a terminal using the command `ctrl` + `alt` + `t`.
7. Launch ROS with the launch file `teleop.launch`. (see [launch file](https://github.com/SherbyRobotics/slash/tree/master/racecar/racecar_bringup/launch)).
8. Disconnect the HDMI cable and turn on the motors by flipping the switch on the left side of the car. A green light will turn on.
9. Enable the joystick by performing an input combination to start.
10. Enjoy!

## High-level Controller Modes

Note: LB is used as a deadman switch and must be always pressed for the car to operate. 

Mode | Input sequence | Function
-|-|-
1|`LB`| Closed-loop velocity, open-loop steering
2|`LB` + `RB`|Fully Open-loop
3|`LB` + `A`|Closed-loop position, open-loop steering
4|`LB` + `B`|Closed-loop velocity, closed-loop steering
5|`LB` + `X`|Closed-loop velocity, closed-loop steering
6|`LB` + `Y`|Reset encoder command
7|`LB` + `LT`|Empty template
8|`LB` + `Button stick left` + `Button stick right`|Joystick disabled
9|`LB` + `Croos key Up/Down`|Template
10|None|Nothing 
