#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from nav_msgs.srv import GetMap
from libbehaviors import *

def main():
    rospy.init_node('brushfire')
    prefix = "racecar"
    rospy.wait_for_service(prefix + '/get_map')
    try:
        get_map = rospy.ServiceProxy(prefix + '/get_map', GetMap)
        response = get_map()
    except (rospy.ServiceException) as e:
        print("Service call failed: %s"%e)
        return
    
    rospy.loginfo("Got map=%dx%d resolution=%f", response.map.info.height, response.map.info.width, response.map.info.resolution)    
    grid = np.reshape(response.map.data, [response.map.info.height, response.map.info.width])
    
    brushfireMap = brushfire(grid)
        
    # Export brusfire map for visualization
    # Adjust color: 0 (black) = obstacle, 10-255 (white) = safest cells
    maximum = np.amax(brushfireMap)
    if maximum > 1:
        mask = brushfireMap==1; 
        brushfireMap = brushfireMap.astype(float) / float(maximum) *225.0 + 30.0
        brushfireMap[mask] = 0
        # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
        cv2.imwrite('brushfire.bmp', cv2.transpose(cv2.flip(brushfireMap, -1)))
        rospy.loginfo("Exported brushfire.bmp")
    else:
        rospy.loginfo("brushfire failed! Is brusfire implemented?")
    
    # Example to show grid with same color than RVIZ
    grid[grid == -1] = 89
    grid[grid == 0] = 178
    grid[grid == 100] = 0
    # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
    cv2.imwrite('map.bmp', cv2.transpose(cv2.flip(grid, -1))) 
    rospy.loginfo("Exported map.bmp")

if __name__ == '__main__':
    main()
