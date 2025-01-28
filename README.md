# racecar

Software repository for Université de Sherbrooke (UdeS)'s 1/10th autonomous car platform. Originally inspired from the [MIT racecar](https://github.com/mit-racecar).

## Table of Contents

* [Requirements](#requirements)
* [Installation](#installation)

## Requirements

### Hardware

* [RaceCar](https://cad.onshape.com/documents/9d3f435f340b50b281de3ac4/w/60d94a6915ed0711b2290521/e/45c11c7ee9e7e6dfaec5c7e5) of UdeS;
* Logitech Wireless Gamepad F710 (XInput mode)\*\*;
* RaspberryPi5 (RPi5) with at least 4GB RAM (8GB recommended; SHOULD work with 2GB, but WILL result in a slower experience);
* Micro-SD card (minimum 32GB, 64/128GB recommended)\*\*\*;
* Arduino Mega2560 Rev3.

[Hardware documentation](doc/README.md)

> \*\* The mappings of the F710 in **XInput** mode are 1:1 with an Xbox Series X controller, and the code SHOULD be compatible.  
> \*\*\* Not all SD cards are the same. Please inform yourselves BEFORE purchasing one. [Here](https://a.co/d/dtF65ZB) is our recommendation.

### Software

* A fresh Ubuntu 24.04.X LTS install on the RPi5 ([Image install link](https://ubuntu.com/download/raspberry-pi/thank-you?version=24.04.1&architecture=desktop-arm64+raspi)) ([How-to flash an image to an SD card (**TODO**)]());
* An internet connection (Ethernet connection recommended) (**IdO** Wi-Fi if on campus)

## Installation

On the RPi5 (with a fresh install), open a terminal (<kbd>CTRL</kbd> + <kbd>ALT</kbd> + <kbd>T</kbd>), and executes the following commands:

> *N.B.* You MUST execute the commands as a user, NOT as root.

```bash
# Create and change into the `src` directory in the ROS 2 workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone and change into this repository
git clone --branch ros2 --depth 1 https://github.com/SherbyRobotics/racecar.git && cd ./racecar

# Add execution permissions to the install script, and launch it.
sudo chmod +x ./images/setup_vm_ubuntu2404_jazzy.bash
./images/setup_vm_ubuntu2404_jazzy.bash
```

Once the install script has finished (successfully), you are ready to test the racecar: see [this section (**TODO**)]().

## Launch

TODO

## High-level Controller Modes

TODO

<!-- The high-level mode is the operating mode of the "controller" node running on the rasberry pi. The high-level mode is selected in the "teleop" node, that translate joystick buttons into a "ctl_ref" message. The mode is placed in the "ctl_ref.linear.z" channel.

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
NaN|`LB` + `LT`|Joystick-based control disabled (no ctl_ref published) -->


## Low-level Controller Modes (Arduino modes)

TODO

<!-- The low-level mode is the operating mode of the Arduino. The low-level mode is selected in the "controller" node running on the rasberry pi. The mode is placed in the "prop_cmd.linear.z" channel.

Low-level Mode | Function
-|-
0|Disabled
1|Open-loop PWM control
2|Closed-loop velocity (based on wheel-encoder feedback)
2|Closed-loop position (based on wheel-encoder feedback)
4|Reset encoder command -->
