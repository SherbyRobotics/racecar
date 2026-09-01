
# System Context

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











