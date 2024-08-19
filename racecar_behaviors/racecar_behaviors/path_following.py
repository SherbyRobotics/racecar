#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Odometry

class PathFollowing(Node):
    def __init__(self):
        super().__init__('path_following')
        self.angle_div = self.declare_parameter('angle_div', 8).value
        self.distance = self.declare_parameter('distance', 0.7).value
        self.distance_short = self.declare_parameter('distance_short', 0.45).value

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)

    def scan_callback(self, msg):
        
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        #l2 = len(msg.ranges)/2;
        #ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]
        
        twist = Twist()
        twist.linear.x = self.max_speed
        twist.angular.z = 0
           
        self.cmd_vel_pub.publish(twist)

    def odom_callback(self, msg):
        self.get_logger().info('Current speed: %f' % msg.twist.twist.linear.x)

def main(args=None):
    rclpy.init(args=args)
    path_following = PathFollowing()
    rclpy.spin(path_following)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
