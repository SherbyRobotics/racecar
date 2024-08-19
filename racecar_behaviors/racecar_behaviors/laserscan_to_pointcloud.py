#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math
import sensor_msgs_py.point_cloud2 as pc2

class LaserScanToPointCloud(Node):
    def __init__(self):
        super().__init__('laserscan_to_pointcloud')
        self.lp = lg.LaserProjection()
        self.pc_pub = self.create_publisher(PointCloud2, 'converted_pc', 1)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            1)

    def scan_callback(self, msg):
        pc2_msg = self.lp.projectLaser(msg)

        # Publish the PointCloud2 message
        self.pc_pub.publish(pc2_msg)
        
        # Convert it to a generator of the individual points
        point_generator = pc2.read_points(pc2_msg)

        # Access the generator in a loop
        total = 0.0
        num_points = 0
        for point in point_generator:
            if not math.isnan(point[2]):
                total += point[2]
                num_points += 1
        average_z = total / num_points
        self.get_logger().info("Average z value: %f"%(average_z))

        # Convert it to a list of individual points
        point_list = pc2.read_points_list(pc2_msg)

        # Access the point list with an index
        middle_point_index = len(point_list) // 2
        middle_point = point_list[middle_point_index]
        self.get_logger().info("Middle point x value: %f"%(middle_point[0]))

def main(args=None):
    rclpy.init(args=args)
    laser_scan_to_point_cloud = LaserScanToPointCloud()
    rclpy.spin(laser_scan_to_point_cloud)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
