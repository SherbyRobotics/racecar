#!/usr/bin/env python3
# cmd_vel_shim.py
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelGazebo(Node):
  def __init__(self):
    super().__init__('cmd_vel_gz')
    self.declare_parameter('wheelbase', 0.325)  # match your <wheel_base> in Gazebo
    self.L = float(self.get_parameter('wheelbase').value)

    # After-arbitration input (final command)
    self.sub = self.create_subscription(Twist, '/racecar/cmd_vel', self.callback, 10)
    # Output to bridge (choose either overwrite or a new topic)
    self.pub = self.create_publisher(Twist, '/racecar/cmd_vel_to_gz', 10)

  def callback(self, m):
    out = Twist()
    out.linear.x = m.linear.x
    if m.linear.x < 0.0 :
      out.angular.z = -m.angular.z
    else:
      out.angular.z = m.angular.z

    self.pub.publish(out)


def main():
  rclpy.init()
  rclpy.spin(CmdVelGazebo())
  rclpy.shutdown()


if __name__ == '__main__':
  main()



