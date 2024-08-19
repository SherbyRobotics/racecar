#!/usr/bin/env python3

import rclpy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped




def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)


def cmd_callback(data):
  global wheelbase
  global frame_id
  global pub
  global max_velocity
  global max_steering

  v = data.linear.x
  if rotvel_instead_cmd_angle:
    steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  else:
    steering = data.angular.z

  msg = AckermannDriveStamped()
  msg.header.stamp = data.header.stamp
  msg.header.frame_id = frame_id
  msg.drive.steering_angle = steering
  if steering < 0:
    msg.drive.steering_angle = max([steering, -max_steering])
  else:
    msg.drive.steering_angle = min([steering, max_steering])
  if v < 0:
    msg.drive.speed = max([v, -max_velocity])
  else:
    msg.drive.speed = min([v, max_velocity])

  pub.publish(msg)


def main(args=None):
  global wheelbase
  global frame_id
  global rotvel_instead_cmd_angle
  global max_velocity
  global max_steering
  global pub

  rclpy.init(args=args)

  node = rclpy.create_node('cmd_vel_to_ackermann_drive')


  wheelbase = node.declare_parameter('wheelbase', 0.34).value
  frame_id = node.declare_parameter('frame_id', 'odom').value
  rotvel_instead_cmd_angle = node.declare_parameter('~rotvel_instead_cmd_angle', False).value
  max_velocity = node.declare_parameter('max_velocity', 4).value
  max_steering = node.declare_parameter('max_steering', 0.37).value

  pub = node.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 1)

  node.create_subscription(Twist, 'cmd_vel', cmd_callback, 1)

  node.get_logger().info("Node 'cmd_vel_to_ackermann_drive' started.\nFrame id: {frame_id}, wheelbase: {wheelbase}")

  rclpy.spin(node)

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
