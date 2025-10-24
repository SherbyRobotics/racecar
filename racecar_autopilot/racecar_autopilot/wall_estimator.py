#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallEstimator(Node):

    def __init__(self):
        super().__init__("estimator")

        # Init subscribers
        self.sub_ref = self.create_subscription(LaserScan, "scan", self.read_scan, 1)

        # Init publishers
        self.pub_y = self.create_publisher(Twist, "car_position", 1)

        # Parameters
        self.cone_angle_deg = 20.0  # deg # Angle around left/right reference to consider for the estimation

        # Outputs
        self.y_estimation = 0.0
        self.dy_estimation = 0.0
        self.theta_estimation = 0.0

        # Memory
        self.y_left = 0.0
        self.y_right = 0.0
        self.theta_left = 0.0
        self.theta_right = 0.0

        # Info about LiDAR read from the first scan
        self.lidar_index_to_compute = True
        self.index_left_start = 0  # 140
        self.index_left_end = 0  # 220
        self.index_right_start = 0  # 500
        self.index_right_end = 0  # 580

    ########################################
    def read_scan(self, scan_msg):

        if self.lidar_index_to_compute:
            self.compute_lidar_ranges(scan_msg)
            self.lidar_index_to_compute = False

        self.estimate_car_position(scan_msg)

    ########################################
    def compute_lidar_ranges(self, scan_msg):

        angle_min_deg = np.degrees(scan_msg.angle_min)
        angle_increment_deg = np.degrees(scan_msg.angle_increment)
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        angles_deg = (
            angle_min_deg
            + np.arange(len(ranges), dtype=np.float32) * angle_increment_deg
        )

        left_min_deg = -90.0 - self.cone_angle_deg
        left_max_deg = -90.0 + self.cone_angle_deg
        right_min_deg = 90.0 - self.cone_angle_deg
        right_max_deg = 90.0 + self.cone_angle_deg

        # Find indices corresponding to left and right min/max angles
        self.index_left_start = np.searchsorted(angles_deg, left_min_deg, side="left")
        self.index_left_end = (
            np.searchsorted(angles_deg, left_max_deg, side="right") - 1
        )
        self.index_right_start = np.searchsorted(angles_deg, right_min_deg, side="left")
        self.index_right_end = (
            np.searchsorted(angles_deg, right_max_deg, side="right") - 1
        )

        self.get_logger().info(
            f"LiDAR left indices: {self.index_left_start} to {self.index_left_end}"
        )
        self.get_logger().info(
            f"LiDAR right indices: {self.index_right_start} to {self.index_right_end}"
        )

    ########################################
    def estimate_line_from_points(self, d, theta):

        # Convert to Cartesian
        y = d * np.sin(theta)
        x = d * np.cos(theta)

        # Formulate A matrix of the linear system
        ones = np.ones(x.shape)
        A = np.column_stack((x, ones))

        # Least square solution
        estimation = np.linalg.lstsq(A, y, rcond=None)[0]  # (ATA)^-1 ATy

        m = estimation[0]  # slope
        b = estimation[1]  # offset

        # Return angle and offset
        theta = np.arctan(m)
        y = b

        return theta, y

    ########################################
    def estimate_car_position(self, scan_msg):

        ranges = np.array(scan_msg.ranges)

        # Left side
        d_data = []
        theta_data = []
        n_good_scan = 0

        # Get data in the left cone
        for i in range(self.index_left_start, self.index_left_end + 1):

            scan_is_good = (ranges[i] > scan_msg.range_min) & (
                ranges[i] < scan_msg.range_max
            )

            if scan_is_good:
                d_data.append(ranges[i])
                theta_data.append(scan_msg.angle_min + i * scan_msg.angle_increment)
                n_good_scan = n_good_scan + 1

        if n_good_scan > 2:

            # Estimate left wall line from points using least squares
            d = np.array(d_data)
            theta = np.array(theta_data)
            self.theta_left, self.y_left = self.estimate_line_from_points(d, theta)

        else:

            self.get_logger().warning("Not enough good scans on the left side.")

        # Right side
        d_data = []
        theta_data = []
        n_good_scan = 0

        # Get data in the right cone
        for i in range(self.index_right_start, self.index_right_end + 1):

            scan_is_good = (ranges[i] > scan_msg.range_min) & (
                ranges[i] < scan_msg.range_max
            )

            if scan_is_good:

                d_data.append(ranges[i])
                theta_data.append(scan_msg.angle_min + i * scan_msg.angle_increment)

                n_good_scan = n_good_scan + 1

        if n_good_scan > 2:

            # Estimate left wall line from points
            d = np.array(d_data)
            theta = np.array(theta_data)
            self.theta_right, self.y_right = self.estimate_line_from_points(d, theta)

        else:

            self.get_logger().warning("Not enough good scans on the right side.")

        # Combine left and right estimates
        self.theta_estimation = 0.5 * (self.theta_left + self.theta_right)
        self.y_estimation = 0.5 * (self.y_left + self.y_right)

        self.pub_estimate()

    ########################################
    def pub_estimate(self):

        msg = Twist()

        msg.linear.y = float(self.y_estimation)
        msg.angular.z = float(self.theta_estimation)

        # debug

        msg.linear.x = float(self.y_left)
        msg.linear.z = float(self.y_right)

        msg.angular.x = float(self.theta_left)
        msg.angular.y = float(self.theta_right)

        self.pub_y.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


##############################################

if __name__ == "__main__":
    main()
