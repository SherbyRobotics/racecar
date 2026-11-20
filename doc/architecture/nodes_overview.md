# Racecar Nodes Overview

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
