
# Component Map

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
