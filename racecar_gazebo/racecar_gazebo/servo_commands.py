#!/usr/bin/env python

import rclpy
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

def set_throttle_steer(data):
    throttle = data.drive.speed / 0.1
    steer = data.drive.steering_angle

    pub_vel_left_rear_wheel.publish(Float64(data=throttle))
    pub_vel_right_rear_wheel.publish(Float64(data=throttle))
    pub_vel_left_front_wheel.publish(Float64(data=throttle))
    pub_vel_right_front_wheel.publish(Float64(data=throttle))
    pub_pos_left_steering_hinge.publish(Float64(data=steer))
    pub_pos_right_steering_hinge.publish(Float64(data=steer))

def servo_commands():
    rclpy.init()
    node = rclpy.create_node('servo_commands')

    global pub_vel_left_rear_wheel, pub_vel_right_rear_wheel, pub_vel_left_front_wheel
    global pub_vel_right_front_wheel, pub_pos_left_steering_hinge, pub_pos_right_steering_hinge

    pub_vel_left_rear_wheel = node.create_publisher(Float64, 'left_rear_wheel_velocity_controller/command', 1)
    pub_vel_right_rear_wheel = node.create_publisher(Float64, 'right_rear_wheel_velocity_controller/command', 1)
    pub_vel_left_front_wheel = node.create_publisher(Float64, 'left_front_wheel_velocity_controller/command', 1)
    pub_vel_right_front_wheel = node.create_publisher(Float64, 'right_front_wheel_velocity_controller/command', 1)
    pub_pos_left_steering_hinge = node.create_publisher(Float64, 'left_steering_hinge_position_controller/command', 1)
    pub_pos_right_steering_hinge = node.create_publisher(Float64, 'right_steering_hinge_position_controller/command', 1)

    node.create_subscription(AckermannDriveStamped, 'ackermann_cmd', set_throttle_steer, 1)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        servo_commands()
    except KeyboardInterrupt:
        pass
