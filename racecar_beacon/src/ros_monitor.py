#!/usr/bin/env python

import rospy
import socket
import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class ROSMonitor:
    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
        # self.sub_odom = rospy.Subcriber(...)
        # self.sub_laser = rospy.Subscriber(...)

        # Current robot state:
        self.id = 0xFFFF
        self.pos = (0,0,0)
        self.obstacle = False

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)

        print("ROSMonitor started.")

    def rr_loop(self):
        # Init your socket here :
        # self.rr_socket = socket.Socket(...)
        while True:
            pass

if __name__=="__main__":
    rospy.init_node("ros_monitor")

    node = ROSMonitor()

    rospy.spin()


