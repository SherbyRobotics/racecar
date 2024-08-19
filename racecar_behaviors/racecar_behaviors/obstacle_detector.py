#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.angle_div = self.declare_parameter('angle_div', 8).value
        self.distance = self.declare_parameter('distance', 0.7).value
        self.distance_short = self.declare_parameter('distance_short', 0.45).value

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)

    def scan_callback(self, msg):
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        l2 = int(len(msg.ranges)/2)
        ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]
        
        # Obstacle front?
        obstacleDetected = False
        for i in range(l2-int(l2/8), l2+int(l2/8)) :
            if np.isfinite(ranges[i]) and ranges[i]>0 and ranges[i] < self.distance:
                obstacleDetected = True
                break

        if obstacleDetected:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('Obstacle detected! Stop!')

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
