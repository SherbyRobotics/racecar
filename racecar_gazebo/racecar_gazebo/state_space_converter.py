#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class StateSpaceConverter(Node):

    def __init__(self):
        super().__init__('statespaceconverter')

        # Vehicle parameters
        self.wheelbase = self.declare_parameter('wheelbase',0.325).value
        self.wheel_spacing = self.declare_parameter('wheel_spacing',0.25).value
        self.max_steering = self.declare_parameter('max_steering',0.667).value

        # Subscribers
        self.sub = self.create_subscription(Twist,'cmd_in',self.cmd_callback,10)
        # Publishers
        self.pub = self.create_publisher(Twist,'cmd_out',10)

    def convert_trans_rot_vel_to_steering_angle(self, v, omega):

        # Prevent division by zero
        if abs(v) == 0 or abs(omega) == 0:
            return 0.0

        radius = v / omega
        # source https://www.racecar-engineering.com/articles/tech-explained-ackermann-steering-geometry/
        return max(min(math.atan(self.wheelbase / (radius - self.wheel_spacing / 2.0)), self.max_steering),-self.max_steering)

    def cmd_callback(self, msg):

        out = Twist()

        out.linear.x = msg.linear.x

        steering_angle = self.convert_trans_rot_vel_to_steering_angle(msg.linear.x, msg.angular.z)

        out.angular.z = steering_angle

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    statespaceconverter = StateSpaceConverter()
    try:
        rclpy.spin(statespaceconverter)
    except KeyboardInterrupt:
        pass
    finally:
        statespaceconverter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
