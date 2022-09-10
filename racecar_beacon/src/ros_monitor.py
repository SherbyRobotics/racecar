#!/usr/bin/env python

import rospy
import socket
import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw

class ROSMonitor:
    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
        self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.odo_cb)
        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.scan_cb)

        # Current robot state:
        self.id = 0xabcd
        self.pos = (0,0,0)
        self.obstacle = False

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)

        # Thread for Broadcast handling:
        self.br_thread = threading.Thread(target=self.br_loop)

        print("ROSMonitor started.")

    def rr_loop(self):
        # Init your socket here :
        #self.rr_socket = socket.Socket(...)
        while True:
            pass

    def br_loop(self):
        # Init your socket here :
        # self.br_socket = socket.Socket(socket.AF_INET, socket.SOCK_DGRAM)
        # br_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        while True:
            pass

    # Subscriber callback:
    def scan_cb(self, msg):
        self.obstacle = False
        dist = msg.ranges
        for i in range(len(dist)):
            if dist[i] < 1:
                self.obstacle = True
                break
    
    # Subscriber callback:
    def odo_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = quaternion_to_yaw(msg.pose.pose.orientation)
        self.pos = (x,y,theta)

if __name__=="__main__":
    rospy.init_node("ros_monitor")

    node = ROSMonitor()

    rospy.spin()


