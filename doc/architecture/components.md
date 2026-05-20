# System Context

This diagram shows how the system interacts with external actors such as the user and simulation environment.

```mermaid
flowchart LR
    User[User / Developer]
    Teleop[Teleoperation Nodes]
    ROS[ROS2 System]
    Bridge[ros_gz_bridge]
    Gazebo[Gazebo Simulation]
    Car[Racecar Model]
    Hardware[Physical Vehicle]

    User --> Teleop
    Teleop --> ROS
    ROS --> Bridge
    Bridge --> Gazebo
    Gazebo --> Car

    ROS --> Hardware

