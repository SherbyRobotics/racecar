#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry


def yaw_from_quaternion(quaternion: Quaternion):
    """
    Use TF transforms to convert a quaternion to a rotation angle around the Z axis.

    Usage with an `Odometry` message:
        ```yaw = yaw_from_quaternion(msg.pose.pose.orientation)
    """
    (_, _, yaw) = euler_from_quaternion(*quaternion)
    return yaw


class PositionPoller(Node):
    def __init__(self):
        super().__init__("position_poller")
        # TODO: Add your subscription(s) here. Use the following syntax:
        #   `self.subscription = self.create_subscription(<type>, <topic>, <self.callback>, 1)`
        self.get_logger().info(f"{self.get_name()} started.")

    # TODO: Add your subscription callback(s) here. Use the following syntax:
    #   ```python3
    #    def <callback>(self, msg: <type>) -> None:
    #       pass
    #   ```


def main(args=None):
    try:
        rclpy.init(args=args)
        node = PositionPoller()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()
