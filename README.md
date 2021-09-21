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
* For Raspberry Pi on RaceCar, we provide RPI3 and RPI4 images with everything already installed, see this [page](https://github.com/SherbyRobotics/racecar/tree/master/images#restore-raspberrypi-image) to flash your RPI. To do it manually, see this [section](https://github.com/SherbyRobotics/racecar/tree/master/images#create-raspberrypi-image).
* For Desktop/Laptop development, see this [section](https://github.com/SherbyRobotics/racecar/tree/master/images#virtual-machine) to setup ROS in a virtual machine or in a dual boot with Ubuntu 20.04.

### Documentation
* [Hardware Connections](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#hardware-connections)
* [USB Hub woes](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#usb-hub-woes)
* [Steering Offset](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#steering-offset)
* [Remote Connection (SSH/VNC)](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#remote-connection-sshvnc)
* [ROS on multiple computers (ROS_IP)](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#ros-on-multiple-computers-ros_ip)
* [Simulated environment (Gazebo)](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#simulated-environment-gazebo)
* [The RaceCar batteries](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#the-racecar-batteries)
* [Charging the Anker battery](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#charging-the-anker-battery)
* [Charging the Traxxas battery](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#charging-the-traxxas-battery)
* [The Killswitch](https://github.com/SherbyRobotics/racecar/blob/master/doc/README.md#the-killswitch)

## How it works
In the graph, you can see a representation of the communication between the nodes and the topics of the most basic working mode:
![134103581-986b1f90-49de-4c44-837e-c3292f4de27f](https://user-images.githubusercontent.com/16725496/134103763-9a2e7839-77fe-411a-9f8b-ce5df9a0cd32.jpg)
using the base launch file [cruisecontrol.launch](https://github.com/SherbyRobotics/racecar/blob/master/racecar_bringup/launch/cruisecontrol.launch). 

The RaceCar has many operating modes that allow you to test different functionnalities. The ROS controller Node, which control commands to the motor and the steering servo, has multiple high-level mode that are selected by joystick inputs (see [Controller modes](#controller-modes) table for more details). For instance, open-loop, closed-loop in velocity, closed-loop in position modes are available for both the propulsion and the steering.  The Arduino has also multiple internal low-level modes (different than the high-level ones!!!) that are selected by the ROS controller Node. Empty operating mode templates are available in the source code for you to test custom control modes.

## Launch
1. Follow those [instructions](https://github.com/SherbyRobotics/racecar/tree/master/images#restore-raspberrypi-image) to flash your Raspberry Pi with a pre-built image, or follow those [instructions](https://github.com/SherbyRobotics/racecar/tree/master/images#create-raspberrypi-image) to setup from scratch.
2. Connect the battery to Raspberry Pi. If installed as instructions above, you can connect your laptop by Wi-Fi on RaceCar's hotspot. Launch VNC from your laptop and connect to `10.42.0.1` (Raspberry Pi's IP). 
2. Flash the Arduino (need to be done only 1 time):
    1. Connect the Arduino mega 2560 of the RaceCar to Raspberry Pi if not already done.
    2. Flash the Arduino mega 2560 with the firmware file [racecar_propulsion_firmware.ino](https://github.com/SherbyRobotics/racecar/tree/master/racecar_arduino/racecar_propulsion_firmware) using Arduino IDE (with Tools->Board: select "Arduino Mega or Mega 2560" as target).
3. Turn on the motors by flipping the switch on the left side of the car. A green light will turn on.
4. From a terminal (`ctrl+alt+t`), launch ROS with the launch file [teleop.launch](https://github.com/SherbyRobotics/racecar/tree/master/racecar_bringup/launch/teleop.launch):
    ```bash
    $ roslaunch racecar_bringup teleop.launch
    ```
5. To visualize the racecar in rviz:
    ```bash
    $ roslaunch racecar_bringup rviz.launch
    ```
    ![](https://github.com/SherbyRobotics/racecar/blob/master/doc/racecar_rviz_teleop.jpg "rviz" )
6. Enable the joystick by performing an input combination below to start.
7. Enjoy!

## High-level Controller Modes

The high-level mode is the operating mode of the "controller" node running on the rasberry pi. The high-level mode is selected in the "teleop" node, that translate joystick buttons into a "ctl_ref" message. The mode is placed in the "ctl_ref.linear.z" channel.

Joystick inputs: LB is used as a deadman switch and must be always pressed for the car to operate. Make sure the switch mode behind the gamepad is on D and not X. 

High-level Mode | Input buttons | Function
-|-|-
-1|None| Disabled
0|`LB`| Closed-loop velocity, open-loop steering
1|`LB` + `RB`|Fully Open-loop
2|`LB` + `RT`|Closed-loop position, open-loop steering
3|`LB` + `A`|Closed-loop velocity, closed-loop steering
4|`LB` + `B`|Closed-loop position, closed-loop steering
5|`LB` + `X`|Closed-loop velocity, closed-loop steering
6|`LB` + `Y`|Reset encoder command
7|`LB` + `LY`|Empty Template
8|`LB` + `Croos key Up/Down`| Empty Template
NaN|`LB` + `LT`|Joystick-based control disabled (no ctl_ref published)


## Low-level Controller Modes (Arduino modes)

The low-level mode is the operating mode of the Arduino. The low-level mode is selected in the "controller" node running on the rasberry pi. The mode is placed in the "prop_cmd.linear.z" channel.

Low-level Mode | Function
-|-
0|Disabled
1|Open-loop PWM control
2|Closed-loop velocity (based on wheel-encoder feedback)
2|Closed-loop position (based on wheel-encoder feedback)
4|Reset encoder command

