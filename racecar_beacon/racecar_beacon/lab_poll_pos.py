#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw
    
class PosPoll(Node):
    def __init__(self):
        super().__init__('pos_poll')
        # Add your subscribers to the class instance here, ex. :
        # self.sub_laser = self.create_subscription(LaserScan, "/scan", self.scan_cb, 1)
        self.get_logger().info("pos_poll node started.")

        # Creates a ROS Timer that will call the timer_cb method every 1.0 sec:
        dt = 1.0
        self.timer = self.create_timer( dt, self.timer_cb)

    # Timer callback:
    def timer_cb(self):
        self.get_logger().info("Timer event.")

    # Subscriber callback:
    # def scan_cb(self, msg):
    #    self.get_logger().info("Got msg from /scan")

def main(args=None):
    rclpy.init(args=args)
    node = PosPoll()
    rclpy.spin(node)
    rclpy.shutdown()    


