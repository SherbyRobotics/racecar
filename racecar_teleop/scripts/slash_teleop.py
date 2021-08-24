#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


#########################################
class teleop(object):
    """
    teleoperation
    """
    def __init__(self):
        self.pub_cmd   = rospy.Publisher("ctl_ref", Twist , queue_size=1  ) 

        self.max_vel  = rospy.get_param('~max_vel',   4.0) # Max linear velocity (m/s)
        self.max_volt = rospy.get_param('~max_volt',  8.0)   # Max voltage is set at 8 volts   
        self.maxStAng = rospy.get_param('~max_angle', 40)  # Supposing +/- 40 degrees max for the steering angle
        self.ps4 = rospy.get_param('~ps4', False)  # PlayStation4 controller: speed axis is 4
        self.cmd2rad   = self.maxStAng*2*3.1416/360
        self.joystickCompatibilityWarned = False
        
        # Always create subscribers last
        self.sub_joy   = rospy.Subscriber("joy", Joy , self.joy_callback , queue_size=1)

    ####################################### 
        
    def joy_callback( self, joy_msg ):
        """ """
        min_axes = 5 if self.ps4 else 4
        if len(joy_msg.axes) < min_axes or len(joy_msg.buttons) < 7:
            if not self.joystickCompatibilityWarned:
                rospy.logwarn("slash_teleop: Received \"%s\" topic doesn't have enough axes (has %d, min=%d) and/ro buttons (has %d, min=7). If a Logitech gamepad is used, make sure also it is in D mode. This warning is only shown once. Until then, this node won't publish any \"%s\".", rospy.names.resolve_name("joy"), len(joy_msg.axes), min_axes, len(joy_msg.buttons), rospy.names.resolve_name("ctl_ref"))
                self.joystickCompatibilityWarned = True
            return

        self.joystickCompatibilityWarned = False   # reset in case we switch mode on the gamepad

        propulsion_user_input = joy_msg.axes[4 if self.ps4 else 3]    # Up-down Right joystick 
        steering_user_input   = joy_msg.axes[0]    # Left-right left joystick
        
        self.cmd_msg = Twist()             
                
        # Software deadman switch
        #If left button is active 
        if (joy_msg.buttons[4]):
            
            #No button pressed (see below)
            # Closed-loop velocity, Open-loop steering, control mode = 0
            
            #If right button is active       
            if (joy_msg.buttons[5]):   
                # Fully Open-Loop
                self.cmd_msg.linear.x  = propulsion_user_input * self.max_volt #[volts]
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
                self.cmd_msg.linear.z  = 1   #CtrlChoice
                
            #If right trigger is active       
            elif (joy_msg.buttons[7]):   
                # Closed-loop position, Open-loop steering
                self.cmd_msg.linear.x  = propulsion_user_input # [m]
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
                self.cmd_msg.linear.z  = 2   #CtrlChoice
                
            #If button A is active 
            elif(joy_msg.buttons[1]):   
                # Closed-loop velocity, Closed-loop steering 
                self.cmd_msg.linear.x  = propulsion_user_input * self.max_vel #[m/s]
                self.cmd_msg.angular.z = steering_user_input # [m]
                self.cmd_msg.linear.z  = 3  # Control mode
                
            #If button B is active 
            elif(joy_msg.buttons[2]):   
                # Closed-loop position, Closed-loop steering 
                self.cmd_msg.linear.x  = propulsion_user_input # [m]
                self.cmd_msg.angular.z = steering_user_input # [m]
                self.cmd_msg.linear.z  = 4  # Control mode
                
            #If button x is active 
            elif(joy_msg.buttons[0]):   
                # Closed-loop velocity with fixed 1 m/s ref, Closed-loop steering
                self.cmd_msg.linear.x  = 2 #[m/s]
                self.cmd_msg.angular.z = 0 # [m]
                self.cmd_msg.linear.z  = 5  # Control mode
                
            #If button y is active 
            elif(joy_msg.buttons[3]):   
                # Reset Encoder
                self.cmd_msg.linear.x  = 0
                self.cmd_msg.angular.z = 0
                self.cmd_msg.linear.z  = 6  # Control mode
                
            #If left trigger is active 
            elif (joy_msg.buttons[6]):
                # No ctl_ref msg published!
                return;
                
            #If right joy pushed
            elif(joy_msg.buttons[11]):
                 # Template for a custom mode
                self.cmd_msg.linear.x  = 0
                self.cmd_msg.angular.z = 0
                self.cmd_msg.linear.z  = 7 # Control mode
                
            #If bottom arrow is active
            elif(joy_msg.axes[5]):
                # Template for a custom mode
                self.cmd_msg.linear.x  = 0
                self.cmd_msg.angular.z = 0
                self.cmd_msg.linear.z  = 8 # Control mode

            # Defaults operation
            # No active button
            else:
                # Closed-loop velocity, Open-loop steering
                self.cmd_msg.linear.x  = propulsion_user_input * self.max_vel #[m/s]
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
                self.cmd_msg.linear.z  = 0  # Control mode
        
        # Deadman is un-pressed
        else:
            # All-stop
            self.cmd_msg.linear.x = 0 
            self.cmd_msg.linear.y = 0
            self.cmd_msg.linear.z = -1            
            self.cmd_msg.angular.x = 0
            self.cmd_msg.angular.y = 0
            self.cmd_msg.angular.z = 0 


        # Publish cmd msg
        self.pub_cmd.publish( self.cmd_msg )
            

#########################################
if __name__ == '__main__':
    
    rospy.init_node('teleop',anonymous=False)
    node = teleop()
    rospy.spin()
