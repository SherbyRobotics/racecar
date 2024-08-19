import sys
import os

# Add the current working directory to sys.path
sys.path.append(os.path.dirname(os.path.realpath(__file__)))

import serial

from proto_gen_classes import floatarray_pb2
from msg_utils import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from PBUtils import *
from geometry_msgs.msg import Twist


class ArduinoCommunicationNode(Node):
    def __init__(self):
        super().__init__('pb2ros')

        # Replace '/dev/ttyACM0' with the correct serial port for your Arduino
        serial_port = '/dev/ttyACM0'
        self.ser = serial.Serial(port=serial_port, baudrate=250000, timeout=0.1)

        self.started = False

        # Ensure the serial port is open
        if not self.ser.is_open:
            try:
                self.ser.open()
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port {serial_port}: {e}")
                sys.exit(1)

        # Define your protobuf message object

        self.ard_pub = self.create_publisher(Float32MultiArray, 'prop_sensors', 1)
        self.subscription = self.create_subscription(Twist, 'prop_cmd', self.controller_callback, 1)

        self.msg_obj = {
            0: floatarray_pb2.FloatArray(),
            1: floatarray_pb2.FloatArray(),
            # Add more message objects if needed, replace '1' with the corresponding message ID
        }

        # Create a PBSerialHandler instance
        self.pb_serial_handler = PBSerialHandler(self.ser, self.callback_function, self.msg_obj)

        # Create a timer to periodically send messages
        # self.timer = self.create_timer(2.0, self.send_pb_message)

    def callback_function(self, response):
        if response is not None:
            #self.get_logger().info(f"Received: {response}")
            deserialized_msgs = self.pb_serial_handler._serialization_handler.deserialize(response)
            for msg_id, msg_obj in deserialized_msgs:
                if msg_id == 0:  # Replace '0' with the corresponding message ID
                    #self.get_logger().info(f"Received message ID {msg_id}: {msg_obj}")
                    # Convert FloatArray to Float32MultiArray
                    self.started = True
                    ros2_msg = Float32MultiArray()
                    ros2_msg.data = msg_obj.data  # Assuming 'data' is the field name in FloatArray

                    # Publish the ROS 2 message
                    self.ard_pub.publish(ros2_msg)
                else :
                    self.get_logger().info(f"Wrong ID: {msg_id}")
                    self.started = False


    def controller_callback(self, msg):
        
            float_array_msg = floatarray_pb2.FloatArray()

            # Send the number to the Arduino

            float_array_msg.data.append(float(msg.angular.z))
            float_array_msg.data.append(float(msg.linear.x))
            float_array_msg.data.append(float(msg.linear.z))

            #self.get_logger().info(f"Sending: {float_array_msg.data}")

            # Replace '1' with the corresponding message ID
            if self.started:
                self.pb_serial_handler.write_pb_msg(1, float_array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommunicationNode()

    try:
        rclpy.spin(node)
    finally:
        # Ensure the serial handler is killed before shutting down the node
        node.pb_serial_handler.kill()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
