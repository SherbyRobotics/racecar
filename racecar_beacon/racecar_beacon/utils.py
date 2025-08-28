#!/usr/bin/env python3

from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import Quaternion


def yaw_from_quaternion(quaternion: Quaternion):
    """
    Use TF transforms to convert a quaternion to a rotation angle around the Z axis.

    Usage with an `Odometry` message:
        ```yaw = yaw_from_quaternion(msg.pose.pose.orientation)
    """
    (_, _, yaw) = euler_from_quaternion([quaternion.w, quaternion.x, quaternion.y, quaternion.z])
    return yaw
