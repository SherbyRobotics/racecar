#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import socket
import threading
from struct import *

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

HOST = "127.0.0.1"
BROADCAST = "127.0.0.255"

# example:
#   python ros_monitor.py scan:=/racecar/scan odom:=/racecar/odometry/filtered

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw

class ROSMonitor(Node):
    def __init__(self):
        super().__init__('ros_monitor')
        # Add your subscriber here (odom? laserscan?):
        # self.sub_laser = self.create_subscription(...)
        # self.sub_odom = self.create_subscription(...)

        # Current robot state:
        self.id = 0xFFFF
        self.pos = (0,0,0)
        self.obstacle = False

        # Params :
        self.remote_request_port = self.declare_parameter('remote_request_port', 65432).value
        self.pos_broadcast_port = self.declare_parameter('pos_broadcast_port', 65431).value

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)

        self.get_logger().info("ROSMonitor started.")
  

    def rr_loop(self):
        # Init your socket here :
        # self.rr_socket = socket.Socket(...)
        
        while rclpy.ok():
            pass



def main(args=None):
    rclpy.init(args=args)
    node = ROSMonitor()
    rclpy.spin(node)
    rclpy.shutdown()   

if __name__=="__main__":
    main()


