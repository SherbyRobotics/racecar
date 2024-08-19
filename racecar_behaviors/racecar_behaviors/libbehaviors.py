#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import cv2
import tf2_ros
import numpy as np
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformListener
from tf2_ros import Buffer
from tf2_geometry_msgs import do_transform_point
import tf_transformations as transformations



def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw
    
def multiply_transforms(trans1, rot1, trans2, rot2):
    trans1_mat = transformations.translation_matrix(trans1)
    rot1_mat   = transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2_mat = transformations.translation_matrix(trans2)
    rot2_mat    = transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = transformations.translation_from_matrix(mat3)
    rot3 = transformations.quaternion_from_matrix(mat3)
    
    return trans3, rot3

def brushfire(occupancyGrid):
    mapOfWorld = np.zeros(occupancyGrid.shape, dtype=int)
    mapOfWorld[occupancyGrid==100] = 1 # set all unknowns and obstacles to -1
    mapOfWorld[occupancyGrid==-1] = 1 
    
    # do brushfire algorithm here
    
    # brushfire: -1 = obstacle or unknown, safer cells have higher value)
    
    return mapOfWorld
    
        

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('your_node_name')  # Change 'your_node_name' to a suitable name

    # Your ROS 2 specific setup here

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
