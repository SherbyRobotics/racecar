#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math, numpy
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
  
  v = data.linear.x
  if rotvel_instead_cmd_angle:
    steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  else:
    steering = data.angular.z
  
  msg = AckermannDriveStamped()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = frame_id
  msg.drive.steering_angle = steering
  if steering<0:
    msg.drive.steering_angle = max([steering, -max_steering])
  else:
    msg.drive.steering_angle = min([steering, max_steering])
  if v<0:
    msg.drive.speed = max([v, -max_velocity])
  else:
    msg.drive.speed = min([v, max_velocity])
  
  
  pub.publish(msg)
  

if __name__ == '__main__': 
  try:
    
    rospy.init_node('cmd_vel_to_ackermann_drive')
        
    wheelbase = rospy.get_param('~wheelbase', 0.34) # used only if rotvel_instead_cmd_angle is false
    frame_id = rospy.get_param('~frame_id', 'odom')
    rotvel_instead_cmd_angle = rospy.get_param('~rotvel_instead_cmd_angle', False)
    max_velocity = rospy.get_param('~max_velocity', 4)
    max_steering = rospy.get_param('~max_steering', 0.37)
    
    rospy.Subscriber('cmd_vel', Twist, cmd_callback, queue_size=1)
    pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)
    
    rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nFrame id: %s, wheelbase: %f", frame_id, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

