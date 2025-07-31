#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np


class WallEstimator(Node):
    def __init__(self):
        super().__init__("wall_estimator")

        # Bounds for the LiDAR's measurements used for estimate
        self.LEFT_REFERENCE_ANGLE = -90.0  # deg
        self.RIGHT_REFERENCE_ANGLE = 90.0  # deg
        self.REFERENCE_ANGLE_DELTA = 15.0  # deg

        # Outputs
        self.y_estimate = float(0)
        self.dy_estimate = float(0)
        self.theta_estimate = float(0)

        # Memory
        self.y_left = float(0)
        self.y_right = float(0)
        self.theta_left = float(0)
        self.theta_right = float(0)
        self.nb_lidar_point = int(0)

        self.lidar_subscription = self.create_subscription(LaserScan, "racecar/scan", self.read_scan, 1)
        self.pos_publisher = self.create_publisher(Twist, "car_position", 1)

    def read_scan(self, scan_msg: LaserScan):
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        angle_min_deg: float = np.degrees(scan_msg.angle_min)
        angle_increment_deg: float = np.degrees(scan_msg.angle_increment)

        left_lower_bound = self.LEFT_REFERENCE_ANGLE - self.REFERENCE_ANGLE_DELTA
        left_upper_bound = self.LEFT_REFERENCE_ANGLE + self.REFERENCE_ANGLE_DELTA

        right_lower_bound = self.RIGHT_REFERENCE_ANGLE - self.REFERENCE_ANGLE_DELTA
        right_upper_bound = self.RIGHT_REFERENCE_ANGLE + self.REFERENCE_ANGLE_DELTA

        # Precompute angles in degress for all indices
        angles_deg = angle_min_deg + np.arange(len(ranges), dtype=np.float32) * angle_increment_deg

        half_length = len(scan_msg.ranges) // 2

        # Left side: only use the first half of the ranges
        left_mask = (
            (angles_deg[:half_length] >= left_lower_bound)
            & (angles_deg[:half_length] <= left_upper_bound)
            & (scan_msg.range_min < ranges[:half_length])
            & (ranges[:half_length] < scan_msg.range_max)
        )
        left_distances = ranges[:half_length][left_mask]
        left_thetas = angles_deg[:half_length][left_mask]

        if len(left_distances) > 2:
            left_distances = np.array(left_distances, dtype=np.float32)
            left_thetas = np.radians(left_thetas)

            y = left_distances * np.sin(left_thetas)
            x = left_distances * np.cos(left_thetas)

            A = np.column_stack((x, np.ones(x.shape, dtype=np.float32)))

            # Least Squares method
            estimate = np.linalg.lstsq(A, y, rcond=None)[0]  # (A'A)^-1 * A'y

            slope = estimate[0]
            offset = estimate[1]

            self.theta_left = np.arctan(slope)
            self.y_left = offset

        # Right side: only use the second half of the ranges
        right_mask = (
            (angles_deg[half_length:] >= right_lower_bound)
            & (angles_deg[half_length:] <= right_upper_bound)
            & (scan_msg.range_min < ranges[half_length:])
            & (ranges[half_length:] < scan_msg.range_max)
        )
        right_distances = ranges[half_length:][right_mask]
        right_thetas = angles_deg[half_length:][right_mask]

        if len(right_distances) > 2:
            right_distances = np.array(right_distances)
            right_thetas = np.radians(right_thetas)

            y = right_distances * np.sin(right_thetas)
            x = right_distances * np.cos(right_thetas)

            A = np.column_stack((x, np.ones(x.shape, dtype=np.float32)))

            # Least Squares method
            estimate = np.linalg.lstsq(A, y, rcond=None)[0]  # (ATA)^-1 ATy

            slope = estimate[0]
            offset = estimate[1]

            self.theta_right = np.arctan(slope)
            self.y_right = offset

        self.theta_estimate = (self.theta_left + self.theta_right) / 2
        self.y_estimate = (self.y_left + self.y_right) / 2

        self.publish_estimate()

    def convert_angle_to_index(self, angle: float):
        return round(self.nb_lidar_point * angle / 360)

    def publish_estimate(self):
        msg = Twist()

        msg.linear.y = float(self.y_estimate)
        msg.angular.z = float(self.theta_estimate)

        # These are for debugging purposes
        msg.linear.x = float(self.y_left)
        msg.linear.z = float(self.y_right)
        msg.angular.x = float(self.theta_left)
        msg.angular.y = float(self.theta_right)

        self.pos_publisher.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = WallEstimator()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
