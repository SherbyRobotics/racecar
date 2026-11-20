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

* Python 3.8
* Arduino programming language
* [ROS 2](https://docs.ros.org/en/humble/Installation.html)

## Requirements

To be able to test our code, there are a few requirements.

### Hardware

* [RaceCar](https://cad.onshape.com/documents/9d3f435f340b50b281de3ac4/w/60d94a6915ed0711b2290521/e/45c11c7ee9e7e6dfaec5c7e5) of the Université de Sherbrooke
* Logitech Wireless Gamepad F710 (DirectInput Mode)

### Software

* For Raspberry Pi on RaceCar, we provide RPI5 images with everything already installed, see this [page](images/README.md#restore-raspberrypi-image) to flash your RPI.

* For Desktop/Laptop development, see this [section](images/README.md#creating-a-virtual-machine) to setup ROS in a virtual machine or in a dual boot with Ubuntu 24.04

* Click here to [download the default virtual machine](https://usherbrooke.sharepoint.com/:u:/s/genie-robotique-montage/IQBziR3UqTC4RLEV-InRo3QyASJViTz5hc1ibc5v5FGeuy0?e=O65S3v), see this [section](images/README.md#importing-a-virtual-machine) for how to import it in Virtual box
    Use the VM to show topics and view them in RVIZ to limit the usage of the RaspberryPi. You might need to change the [ROS_DOMAIN_ID](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Domain-ID.html) on both machine before using it : `export ROS_DOMAIN_ID = X`
* As the racecar repository is still under continuous development, we recommand to pull the code at least once before starting any new project as the source code might be more up to date than the version available on a RPI5 image.

### Documentation

* [Architecture Diagrams](doc/architecture/README.md#architecture-diagram)
* [Hardware Connections](doc/README.md#hardware-connections)
* [USB Hub woes](doc/README.md#usb-hub-woes)
* [Steering Offset](doc/README.md#steering-offset)
* [Remote Connection (SSH/XRDP)](doc/README.md#remote-connection-sshxrdp)
* [ROS2 on multiple computers (ROS_IP)](doc/README.md#ros-on-multiple-computers-ros_ip)
* [Simulated environment (Gazebo)](doc/README.md#simulated-environment-gazebo)
* [The RaceCar batteries](doc/README.md#the-racecar-batteries)
* [Charging the Anker battery](doc/README.md#charging-the-anker-battery)
* [Charging the Traxxas battery](doc/README.md#charging-the-traxxas-battery)
* [The Killswitch](doc/README.md#the-killswitch)

## How it works

In the graph, you can see a representation of the communication between the nodes and the topics of the most basic working mode:
![134103581-986b1f90-49de-4c44-837e-c3292f4de27f](https://user-images.githubusercontent.com/16725496/134103763-9a2e7839-77fe-411a-9f8b-ce5df9a0cd32.jpg)

using the base launch file with:

```bash
$ ros2 launch racecar_bringup basic_control.launch.py
$ rqt_plot
```

The RaceCar has many operating modes that allow you to test different functionnalities. The ROS controller Node, which control commands to the motor and the steering servo, has multiple high-level mode that are selected by joystick inputs (see [Controller modes](#controller-modes) table for more details). For instance, open-loop, closed-loop in velocity, closed-loop in position modes are available for both the propulsion and the steering.  The Arduino has also multiple internal low-level modes (different than the high-level ones!!!) that are selected by the ROS controller Node. Empty operating mode templates are available in the source code for you to test custom control modes.

## Launch

1. Follow those [instructions](images#restore-raspberrypi-image) to flash your Raspberry Pi with a pre-built image.
2. Connect the battery to Raspberry Pi.
3. Flash the Arduino (need to be done only 1 time):
    1. Connect the Arduino mega 2560 of the RaceCar to Raspberry Pi if not already done.
    2. Flash the Arduino mega 2560 with the firmware file [`main.cpp`](racecar_arduino/Controller/src/main.cpp) using Visual Studio Code with PlatformIO IDE .
    3. Open Visual Studio Code, then navigate to File > Open Folder. Select the folder: [racecar_arduino](racecar_arduino/Controller) and open the file [`main.cpp`](racecar_arduino/Controller/src/main.cpp).
    4. Once the project loads, wait for PlatformIO Core to finish initializing
    5. In the top-right corner, click the checkmark icon, then select Upload to flash the code to your device.

4. Turn on the motors by flipping the switch on the left side of the car. A green light will turn on.
5. From a terminal (`ctrl+alt+t`), launch ROS with the launch file [teleop.launch.py](racecar_bringup/launch/teleop.launch.py):

    ```bash
    ros2 launch racecar_bringup teleop.launch.py
    ```

6. To visualize the racecar in rviz:

    ```bash
    ros2 launch racecar_navigation rviz.launch.py
    ```

    ![](doc/racecar_rviz_teleop.jpg "rviz" )
7. Enable the joystick by performing an input combination below to start.
8. Enjoy!

# Controller Modes

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


# Architecture Documentation

This section describes the architecture of the racecar system.

## Overview

The system is built using a modular ROS2 architecture that supports both:

- Simulation (Gazebo)
- Real vehicle hardware (Racecar)

The same control flow is reused for both environments.

---

## Architecture Diagrams

- [System Context]  
  This diagram shows how the system interacts with external actors such as the user and the simulation environment.

```mermaid

flowchart LR

    User[User]

    subgraph Input [Input Interfaces]
        Control["(Joystick / Web Interface)"]
    end

    subgraph Core [ROS2 System]
        ROS[ROS2 Nodes]
    end

    subgraph Execution [Execution Systems]
        Gazebo["Simulation (Gazebo)"]
        Hardware[Racecar]
    end

    User --> Control
    Control --> ROS
    ROS --> Gazebo
    ROS --> Hardware

```

- [Component Map]  
  This diagram shows the relationship between the major modules of the system.
  
```mermaid

flowchart LR

    User[User]

%% --------------------
%% INPUT INTERFACES
%% --------------------
    subgraph Input [Input Interfaces]
        Control["(Joystick / Web Interface)"]
    end

%% --------------------
%% ROS SYSTEM
%% --------------------
    subgraph ROS [ROS2 System]
        direction LR
%% Control (grouped)
    subgraph Controls [Arduino Motor Controls]
        direction LR
        SpeedControl[SpeedControl]
        PositionControl[PositionControl]
        Openloop[Openloop]
    end

%% Perception
        subgraph Perception [Perception]
            Lidar[Lidar]
            Camera[Camera]
        end

%% Odometry / state
        subgraph State [Odometry]
            direction LR
            IMU[IMU]
            Encoder[Wheel Encoder]
            Steering[Steering Angle]
        end

%% Navigation / decision
    subgraph Navigation [Navigation Algorithms]
        direction LR
        SLAM[SLAM]
        Blob[Blob Detection]
        Brushfire[Brushfire]
        Autopilot[Autopilot]
    end

end

%% --------------------
%% EXECUTION
%% --------------------
    subgraph Execution [Execution Systems]
        direction LR
        Racecar[Racecar]
        Gazebo[Gazebo Simulation]
    end
%% --------------------
%% MONITORING INTERFACES
%% --------------------

    subgraph Monitoring [Monitoring and Tools]
        direction LR
        RViz[RViz]
        RQt[RQt]
        ROS2Bag[ROS2Bag]
    end

    ROS --> Monitoring

%% --------------------
%% FLOW
%% --------------------

%% Inputs into system
    User --> Input
    Input --> ROS

%% Sensor pipeline
    State --> Navigation
    State --> Controls
    Perception --> Navigation


%% Navigation feeds control
    Navigation --> Controls

%% Brushfire separate (map-level)

%% Control outputs
    ROS --> Execution
```

- [Sequence Diagram]  
  This diagram illustrates the most complex core logic flow of the system during autonomous operation.

```mermaid
sequenceDiagram

    participant Sensors
    participant ROS2
    participant Navigation as Navigation and SLAM
    participant Blob as Blob Detection
    participant Autopilot
    participant Arduino

    %% Initialization
    ROS2->>Navigation: load map and initialize path

    %% Continuous loop
    loop Autonomous Navigation

        Sensors->>ROS2: publish sensor data (lidar, camera, imu, encoder)

        %% Localization and navigation
        ROS2->>Navigation: send lidar and odometry
        Navigation->>ROS2: position estimate

        %% Blob detection
        ROS2->>Blob: send lidar and camera data
        Blob->>ROS2: detected balloons

        %% Autopilot (reactive wall avoidance)
        ROS2->>Autopilot: send lidar data
        Autopilot->>ROS2: generate steering and speed commands

        %% Execution
        ROS2->>Arduino: send control commands
        Arduino-->>ROS2: odometry feedback

    end
```

- [Packages Overview]  
  This diagram provides a high-level overview of the project packages and their primary responsibilities and scripts.
This also display APP-package relation.

```mermaid
flowchart TB

    %% ======================
    %% FOUNDATION
    %% ======================

    subgraph Foundation["Foundation"]
        direction LR

        subgraph Beacon["racecar_beacon (APP1)"]
            BC["Introduces ROS2 distributed systems concepts, including remote communication, monitoring and network-based data exchange.

Important files:
- remote_client.py
- ros_monitor.py
- vehicle_tracker.py"]
        end

        subgraph Bringup["racecar_bringup (APP1 to APP5)"]
            B["Contains the launch files and integration nodes used to start the different laboratories and system configurations.

Important files:
- bringup.launch.py
- autopilot.launch.py
- teleop.launch.py
- rviz.launch.py
- cmd_vel_arbitration.py
- arduino_sensors.py"]
        end
    end

    %% ======================
    %% USER CONTROL
    %% ======================

    subgraph UserControl["User Control"]
        direction LR

        subgraph Teleop["racecar_teleop (APP2 to APP5)"]
            T["Allows manual control of the racecar using a joystick interface.

Important files:
- slash_teleop.py"]
        end

        subgraph Web["racecar_web_interface (APP3)"]
            W["Provides a browser-based control interface for the racecar.

Important files:
- index.html
- links.html
- virtualjoystick.js
- roslib.min.js"]
        end
    end

    %% ======================
    %% AUTONOMOUS FUNCTIONS
    %% ======================

    subgraph Autonomy["Autonomous Driving"]
        direction LR

        subgraph Navigation["racecar_navigation (APP5)"]
            N["Provides localization, filtering and navigation capabilities used throughout the project.

Important files:
- slam.launch.py
- navigation.launch.py
- navigation_stack.launch.py
- dual_ekf_params.yaml"]
        end

        subgraph Behaviors["racecar_behaviors (APP5)"]
            BH["Implements perception and navigation algorithms such as blob detection, path following and brushfire analysis.

Important files:
- blob_detector.py
- path_following.py
- labo_brushfire.py
- obstacle_detector.py
- laserscan_to_pointcloud.py
- libbehaviors.py"]
        end

        subgraph Autopilot["racecar_autopilot (APP4)"]
            A["Implements autonomous vehicle control using LiDAR-based wall following and steering control.

Important files:
- wall_estimator.py
- slash_controller.py"]
        end
    end

    %% ======================
    %% HARDWARE / SIMULATION
    %% ======================

    subgraph Platform["Hardware and Simulation"]
        direction LR

        subgraph Serial["racecar_serial (APP2)"]
            S["Handles communication between ROS2 and the Arduino controller.

Important files:
- libserial
- pb2roscpp"]
        end

        subgraph Arduino["racecar_arduino (APP2)"]
            AR["Contains the embedded firmware responsible for motor control and sensor acquisition.

Important files:
- main.cpp
- PBUtils.cpp"]
        end

    subgraph Gazebo["racecar_gazebo (APP5)"]
        G["Provides the simulation environment used to test algorithms before deploying them on the physical vehicle.

Important resources:
- gazebo_control.launch.py
- simulation.launch.py"]
    end

    subgraph Description["racecar_description"]
        D["Stores the robot model, configuration files, and hardware descriptions used by ROS2 and Gazebo.

    Important resources:
    - URDF/Xacro files
    - robot configuration"]
        end
    end
```

- [Nodes Overview]  
This diagram provides a simplified ROS2 node graph, highlighting the flow of information between the nodes responsible for sensing, localization, perception, autonomous behaviors, and vehicle control.

| Color     | Package              |
| --------- | -------------------- |
| 🔵 Blue   | `racecar_teleop`     |
| 🟢 Green  | `racecar_navigation` |
| 🟠 Orange | `racecar_behaviors`  |
| 🔴 Red    | `racecar_autopilot`  |
| 🟣 Purple | `racecar_bringup`    |
| 🟡 Yellow | `racecar_serial`     |
| ⚪ Gray    | `External packages`   |


```mermaid

flowchart LR

    Joy["joy_node<br>Reads joystick input"]

    Teleop["slash_teleop<br>Converts joystick input to driving commands"]

    Lidar["rplidar<br>Publishes LiDAR scans"]

    Camera["camera<br>Publishes camera images"]

    IMU["imu_filter_madgwick<br>Publishes filtered IMU data"]

    Odom["arduino_sensors<br>Publishes vehicle odometry"]

    EKF["Kalman Filter<br>Fuses IMU and odometry"]

    RTABMap["rtabmap<br>Performs SLAM and localization"]

    Nav2["navigation_stack<br>Plans navigation paths"]

    Path["path_following<br>Follows planned trajectories"]

    Obstacle["obstacle_detector<br>Detects nearby obstacles"]

    ScanCloud["laserscan_to_pointcloud<br>Converts scan to point cloud"]

    Depth["pointcloud_to_depthimage<br>Generates depth image"]

    Blob["blob_detector<br>Detects colored balloons"]

    WallEstimator["wall_estimator<br>Estimates wall position"]

    Controller["slash_controller<br>Generates steering commands"]

    Arbitration["cmd_vel_arbitration<br>Selects active command source"]

    Arduino["pb2roscpp<br>Bridges ROS2 and Arduino"]

    Joy -- joy --> Teleop

    %% LOCALIZATION

    IMU -- racecar/imu --> EKF

    Odom -- racecar/odom --> EKF

    EKF -- odom/filtered --> RTABMap

    Lidar -- racecar/scan --> RTABMap

    RTABMap -- map --> Nav2

    %% NAVIGATION

    Nav2 -- navigation goals --> Path

    Lidar -- racecar/scan --> Path

    Odom -- odom/filtered --> Path

    %% OBSTACLE DETECTION

    Lidar -- racecar/scan --> Obstacle

    %% BLOB DETECTION

    Camera -- racecar/camera --> Blob

    Camera -- camera_info --> Blob

    Lidar -- racecar/scan --> ScanCloud

    ScanCloud -- scan_cloud --> Depth

    Depth -- depth image --> Blob

    %% AUTOPILOT

    Lidar -- racecar/scan --> WallEstimator

    WallEstimator -- wall estimate --> Controller

    %% COMMAND SOURCES

    Teleop -- cmd_vel_abtr_0 --> Arbitration

    Obstacle -- cmd_vel_abtr_1 --> Arbitration

    Path -- cmd_vel_abtr_5 --> Arbitration

    Controller -- ctl_ref --> Arbitration

    %% VEHICLE

    Arbitration -- cmd_vel --> Arduino

    %% STYLES

    classDef teleop fill:#dbeafe,stroke:#2563eb,color:#000;
    classDef navigation fill:#dcfce7,stroke:#16a34a,color:#000;
    classDef behaviors fill:#fed7aa,stroke:#ea580c,color:#000;
    classDef autopilot fill:#fecaca,stroke:#dc2626,color:#000;
    classDef bringup fill:#e9d5ff,stroke:#9333ea,color:#000;
    classDef serial fill:#fde68a,stroke:#ca8a04,color:#000;
    classDef external fill:#f3f4f6,stroke:#6b7280,color:#000;

    class Joy,Teleop teleop;
    class EKF,RTABMap,Nav2 navigation;
    class Path,Obstacle,ScanCloud,Blob behaviors;
    class WallEstimator,Controller autopilot;
    class Odom,Arbitration bringup;
    class Arduino serial;
    class Lidar,Camera,IMU,Depth external;

```



