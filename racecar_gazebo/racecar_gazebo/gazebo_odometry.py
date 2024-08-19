import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math
from threading import Thread, Lock
from tf_transformations import quaternion_from_euler

class OdometryNode(Node):
    def __init__(self):
        super().__init__('gz_odometry')
        self.link_prefix = self.declare_parameter('link_prefix', 'racecar').value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value
        self.frame_id = self.declare_parameter('frame_id', 'base_footprint').value
        self.radius = self.declare_parameter('wheel_radius', 0.05).value
        self.l = self.declare_parameter('wheelbase', 0.34).value
        self.w = self.declare_parameter('axle_track', 0.20).value
        self.last_odom = Odometry()
        self.last_odom.header.frame_id = self.odom_frame_id
        self.last_odom.child_frame_id = self.frame_id
        self.previous_left_joint_position = 0
        self.previous_right_joint_position = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.tf_pub = self.create_publisher(TransformStamped, 'tf', 1)
        self.sub_robot_pose_update = self.create_subscription(JointState, 'joint_states', self.sub_robot_pose_update_callback, 1)

    def quaternion_to_yaw(self, quat):
        (roll, pitch, yaw) = quaternion_from_euler(quat.x, quat.y, quat.z, quat.w)
        return yaw

    def sub_robot_pose_update_callback(self, msg):
        left_joint_position = msg.position[1]
        right_joint_position = msg.position[4]

        if self.last_odom.pose.pose.orientation.w == 0:
            self.previous_left_joint_position = left_joint_position
            self.previous_right_joint_position = right_joint_position
            self.last_odom.pose.pose.orientation.w = 1
        else:
            delta_left = (left_joint_position - self.previous_left_joint_position) * self.radius
            delta_right = (right_joint_position - self.previous_right_joint_position) * self.radius

            self.previous_left_joint_position = left_joint_position
            self.previous_right_joint_position = right_joint_position

            distance = (delta_left + delta_right) / 2.0
            elapsed_seconds = msg.header.stamp.to_sec() - self.last_odom.header.stamp.to_sec()
            speed = distance / elapsed_seconds

            steering_angle = (msg.position[2] + msg.position[5]) / 2.0
            v_x = speed * math.cos(self.theta)
            v_y = speed * math.sin(self.theta)
            v_theta = speed * math.tan(steering_angle) / self.l

            self.x = self.x + v_x * elapsed_seconds
            self.y = self.y + v_y * elapsed_seconds
            self.theta = self.theta + v_theta * elapsed_seconds

            self.last_odom.pose.pose.position.x = self.x
            self.last_odom.pose.pose.position.y = self.y
            quaternion = quaternion_from_euler(0, 0, self.theta)
            self.last_odom.pose.pose.orientation.x = quaternion[0]
            self.last_odom.pose.pose.orientation.y = quaternion[1]
            self.last_odom.pose.pose.orientation.z = quaternion[2]
            self.last_odom.pose.pose.orientation.w = quaternion[3]
            self.last_odom.twist.twist.linear.x = v_x
            self.last_odom.twist.twist.linear.y = v_y
            self.last_odom.twist.twist.angular.z = v_theta

        self.last_odom.header.stamp = msg.header.stamp
        self.publish_odom()
        if self.publish_tf:
            self.publish_transform()

    def publish_odom(self):
        self.odom_pub.publish(self.last_odom)

    def publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.last_odom.header.stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        quaternion = quaternion_from_euler(0, 0, self.theta)
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.tf_pub.publish(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
