#!/usr/bin/env python

import rospy
import math 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleDetector:
    def __init__(self):
        self.distance = rospy.get_param('~distance', 0.75)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)

    def scan_callback(self, msg):
    
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        l2 = int(len(msg.ranges)/2);
        ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]
        
        # Obstacle front?
        obstacleDetected = False
        for i in range(l2-int(l2/8), l2+int(l2/8)) :
            if np.isfinite(ranges[i]) and ranges[i]>0 and ranges[i] < self.distance:
                obstacleDetected = True
                break
                
        if obstacleDetected:
            self.cmd_vel_pub.publish(Twist()); # zero twist  
            rospy.loginfo("Obstacle detected! Stop!")      

def main():
    rospy.init_node('obstacle_detector')
    obstacleDetector = ObstacleDetector()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

