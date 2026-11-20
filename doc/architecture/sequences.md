# Sequence Diagram

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
