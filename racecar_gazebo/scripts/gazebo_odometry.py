#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from random import seed
from random import gauss
from threading import Thread, Lock


def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw

class OdometryNode:
    # Set publishers
    pub_odom = rospy.Publisher('odom', Odometry, queue_size=1)

    def __init__(self):
        # init internals
        self.link_prefix = rospy.get_param('~link_prefix', 'racecar')
        self.publish_tf = rospy.get_param('~publish_tf', True)
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.frame_id = rospy.get_param('~frame_id', 'base_footprint')
        self.radius = rospy.get_param('~wheel_radius', 0.05)
        self.l = rospy.get_param('~wheelbase', 0.34)
        self.w = rospy.get_param('~axle_track', 0.20)
        self.lastOdom = Odometry()
        self.lastOdom.header.frame_id = self.odom_frame_id
        self.lastOdom.child_frame_id = self.frame_id
        
        self.previousLeftJointPosition = 0
        self.previousRightJointPosition = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        seed(1)

        self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers
        rospy.Subscriber('joint_states', JointState, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
    
        leftJointPosition = msg.position[1]
        rightJointPosition = msg.position[4]
    
        if self.lastOdom.pose.pose.orientation.w == 0:
            # initialization
            self.previousLeftJointPosition = leftJointPosition
            self.previousRightJointPosition = rightJointPosition
            self.lastOdom.pose.pose.orientation.w = 1
        else:
            deltaLeft = (leftJointPosition - self.previousLeftJointPosition) * self.radius
            deltaRight = (rightJointPosition - self.previousRightJointPosition) * self.radius
            
            self.previousLeftJointPosition = leftJointPosition
            self.previousRightJointPosition = rightJointPosition
            
            distance = (deltaLeft + deltaRight) / 2.0
            elapsed_seconds = msg.header.stamp.to_sec() - self.lastOdom.header.stamp.to_sec()
            speed = distance / elapsed_seconds
            
            # With same approach (car-like kinematic model) than in arduino_bridge.py
            steering_angle = (msg.position[2] + msg.position[5]) / 2.0
            v_x = speed * math.cos(self.theta)
            v_y = speed * math.sin(self.theta)
            v_theta = speed * math.tan(steering_angle) / self.l
            
            ################################
            # Adding noise to odometry
            ################################
            # scaleError = 1.2
            # randomNoise = gauss(0, 0.01) * v_x #  1 cm/m random error
            # v_x = (v_x+randomNoise) * scaleError
            # fixedError = 1.0 * np.pi / 180.0 * v_x # 1 deg/m systematic error on left
            # randomNoise = gauss(0, 1.0 * np.pi / 180.0) * v_x #  1 deg/m random error
            # v_theta = v_theta + randomNoise + fixedError

            self.x = self.x + v_x * elapsed_seconds
            self.y = self.y + v_y * elapsed_seconds
            self.theta = self.theta + v_theta * elapsed_seconds
     
            self.lastOdom.pose.pose.position.x = self.x
            self.lastOdom.pose.pose.position.y = self.y
            quaternion = quaternion_from_euler(0, 0, self.theta)
            self.lastOdom.pose.pose.orientation.x = quaternion[0]
            self.lastOdom.pose.pose.orientation.y = quaternion[1]
            self.lastOdom.pose.pose.orientation.z = quaternion[2]
            self.lastOdom.pose.pose.orientation.w = quaternion[3]
            self.lastOdom.twist.twist.linear.x = v_x
            self.lastOdom.twist.twist.linear.y = v_y
            self.lastOdom.twist.twist.angular.z = v_theta
        
        self.lastOdom.header.stamp = msg.header.stamp
        self.pub_odom.publish(self.lastOdom)

        if self.publish_tf:
            tf = TransformStamped(
                header=Header(
                    frame_id=self.lastOdom.header.frame_id,
                    stamp=self.lastOdom.header.stamp
                ),
                child_frame_id=self.lastOdom.child_frame_id,
                transform=Transform(
                    translation=self.lastOdom.pose.pose.position,
                    rotation=self.lastOdom.pose.pose.orientation
                )
            )
            self.tf_pub.sendTransform(tf)

# Start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
    rospy.spin()
