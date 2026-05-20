
# Command Flow Sequence

This diagram describes how a command flows from the user to the vehicle.

```mermaid
sequenceDiagram
    participant User
    participant Teleop
    participant Arbiter
    participant Converter
    participant Controller
    participant Bridge
    participant Gazebo

    User->>Teleop: joystick input
    Teleop->>Arbiter: cmd_vel
    Arbiter->>Converter: selected cmd_vel
    Converter->>Converter: convert to Ackermann
    Converter->>Controller: send command
    Controller->>Bridge: publish cmd_vel
    Bridge->>Gazebo: gz.msgs.Twist
    Gazebo->>Gazebo: compute steering + motion
