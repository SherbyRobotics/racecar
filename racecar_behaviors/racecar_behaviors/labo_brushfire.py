#!/usr/bin/env python3

# NOTE: "Band-Aid" for `ros2 run` issue. Not required when using `python3` to
# run the script.
#import sys
#if "/usr/local/lib/python3.12/dist-packages" in sys.path:
#    sys.path.remove("/usr/local/lib/python3.12/dist-packages")

import rclpy
import rclpy.logging
from rclpy.node import Node
import cv2
import numpy as np
from nav_msgs.srv import GetMap
from racecar_behaviors.libbehaviors import brushfire

class Brushfire(Node):
    def __init__(self):
        super().__init__('brushfire')
        self.prefix = "rtabmap"
        self.get_map_client = self.create_client(GetMap, self.prefix + '/get_map')
        while not self.get_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def export_brushfire_map(self, brushfire_map):
        # Adjust color: 0 (black) = obstacle, 10-255 (white) = safest cells
        maximum = np.amax(brushfire_map)
        if maximum > 0:
            mask = brushfire_map == 1
            brushfire_map = brushfire_map.astype(float) / float(maximum) * 225.0 + 30.0
            brushfire_map[mask] = 0
            brushfire_map = brushfire_map.astype(np.uint8)  # Removes "type warning" from OpenCV
            # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
            cv2.imwrite('brushfire.bmp', cv2.transpose(cv2.flip(brushfire_map, -1)))
            self.get_logger().info("Exported brushfire.bmp")
        else:
            self.get_logger().info("Brushfire failed! Is brushfire implemented?")

    def export_grid_map(self, grid):
        # Example to show grid with same color as RVIZ
        img = np.zeros_like(grid).astype(np.uint8)  # Avoids OverflowError from NumPy
        img[grid == -1] = 89
        img[grid == 0] = 178
        img[grid == 100] = 0
        # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
        cv2.imwrite('map.bmp', cv2.transpose(cv2.flip(img, -1)))
        self.get_logger().info("Exported map.bmp")

    def get_map_callback(self, future):
        response = future.result()
        self.get_logger().info("Got map=%dx%d resolution=%f" %(response.map.info.height, response.map.info.width, response.map.info.resolution))
        #rospy.loginfo("Got map=%dx%d resolution=%f", response.map.info.height, response.map.info.width, response.map.info.resolution)
        grid = np.reshape(response.map.data, [response.map.info.height, response.map.info.width])
        brushfire_map = brushfire(grid)
        self.export_brushfire_map(brushfire_map)
        self.export_grid_map(grid)

    def main(self):
        request = GetMap.Request()
        future = self.get_map_client.call_async(request)
        future.add_done_callback(self.get_map_callback)

def main(args=None):
    rclpy.init(args=args)
    brushfire_node = Brushfire()
    brushfire_node.main()
    rclpy.spin(brushfire_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
