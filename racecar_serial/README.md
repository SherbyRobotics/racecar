# pb2ros

## Overview

`pb2ros` is a ROS2 node written in Python that facilitates communication between an Arduino microcontroller and a ROS2 system using Protocol Buffers (protobuf) for serialization and deserialization of messages.

The node establishes a serial connection with the Arduino over a specified serial port and exchanges messages between the two systems. The Arduino sends data in protobuf format, which is then deserialized and converted into ROS2 messages for further processing within the ROS2 environment.

This exemple sends the number received from the talker demo node on the chatter topic and sends it to the arduino. The arduino echoes back this number, which is then published on the ard_pub topic. 

## Prerequisites

Before using `pb2ros`, ensure that you have the following dependencies installed:

- ROS 2: Install ROS 2 following the official instructions for your system ([ROS 2 Installation](https://docs.ros.org/en/foxy/Installation.html)).

- Python Packages: Install the required Python packages using the following command:

    1. Install ROS2 py:
    ```bash
    pip install rclpy
    ```

    2. Install Protobuf:
Clone or download the nanoPB library and move the folder into the Arduino's libraries folder (usually ~/snap/arduino/current/Arduino/libraries)
```bash
clone https://github.com/nanopb/nanopb.git
```

## Getting Started

1. **Clone the repository:**

    ```bash
    git clone <repository_url>
    ```

2. **Navigate to the `pb2ros2` directory:**

    ```bash
    cd pb2ros2
    ```

3. **Update the serial port:**

    Open the `ArduinoCommunicationNode` class in the script and replace `'/dev/ttyACM0'` with the correct serial port for your Arduino.

4. **Run the node:**

    ```bash
    ros2 run pb2ros2 arduino_agent
    ```
    In another terminal:
    ```bash
    ros2 run demo_nodes_py talker
    ```

## Configuration

- **Serial Port Configuration:** Update the `serial_port` variable in the `ArduinoCommunicationNode` constructor to match the serial port of your Arduino.

- **Message Definitions:** Modify the message definitions in the script based on your protobuf message structure. Ensure that the protobuf messages align with the messages exchanged between the ROS 2 node and the Arduino.

## ROS 2 Topics

### Published Topics

- `/ard_pub` (std_msgs/Float32MultiArray): ROS 2 topic where deserialized data from the Arduino is published.

### Subscribed Topics

- `/chatter` (std_msgs/String): ROS 2 topic for receiving string messages to be processed by the node.

## Customization

1. **Message Handling:** Adjust the `callback_function` method to handle incoming messages according to your specific use case.

2. **Message Publishing:** Customize the `chatter_callback` method to process incoming string messages and publish corresponding protobuf messages to the Arduino.

3. **Message Serialization:** Update the serialization and deserialization logic in the `PBSerialHandler` class to match the structure of your protobuf messages.

## Troubleshooting

- **Serial Connection:** If the serial connection cannot be established, check the serial port configuration and ensure that the Arduino is connected and powered.

- **Message Handling:** Verify that the message handling logic in the `callback_function` and `chatter_callback` methods is correctly implemented for your application.

- **Dependencies:** Ensure that all required dependencies are installed and up-to-date.



