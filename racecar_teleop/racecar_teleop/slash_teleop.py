#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


#########################################
class Teleop(Node):
    """
    teleoperation
    """
    def __init__(self):

        super().__init__('Teleop')


        self.max_vel = self.declare_parameter('max_vel', 4.0).value
        self.max_volt = self.declare_parameter('max_volt', 8.0).value
        self.maxStAng = self.declare_parameter('max_angle', 40).value
        self.ps4 = self.declare_parameter('ps4', False).value

        self.cmd2rad   = self.maxStAng*2*3.1416/360
        self.joystickCompatibilityWarned = False


        self.pub_cmd = self.create_publisher(Twist, 'ctl_ref', 1)
        
        # Always create subscribers last
        self.sub_joy = self.create_subscription(Joy, 'joy', self.joy_callback, 1)

        

    ####################################### 
        
    def joy_callback( self, joy_msg ):
        """ """
        min_axes = 5 if self.ps4 else 4
        if len(joy_msg.axes) < min_axes or len(joy_msg.buttons) < 7:
            if not self.joystickCompatibilityWarned:
                self.get_logger().info("slash_teleop: Received topic doesn't have enough axes and/or buttons. If a Logitech gamepad is used, make sure also it is in X mode. Will not warn again.")
                self.joystickCompatibilityWarned = True
            return

        self.joystickCompatibilityWarned = False   # reset in case we switch mode on the gamepad

        propulsion_user_input = joy_msg.axes[3]    # Up-down Right joystick 
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
                self.cmd_msg.linear.z  = 1.0   #CtrlChoice
            
            elif (joy_msg.buttons[10]): # RJP
                """
                GRO501-1: closed-loop velocity fixed @ X m/s, open-loop
                steering, where X is determined "on-site".
                """
                self.cmd_msg.linear.x = 2.0 # m/s
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
                self.cmd_msg.linear.z = 0.0 # high-level mode
                
            #If right trigger is active       
            elif (joy_msg.buttons[7]):   # START
                """
                GRO501-1: closed-loop position fixed @ X m, open-loop
                steering, where X is determined "on-site".
                """
                self.cmd_msg.linear.x  = 2.0 # [m]
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
                self.cmd_msg.linear.z  = 2.0   #CtrlChoice
                
            #If button A is active 
            elif(joy_msg.buttons[1]):   
                # Closed-loop velocity, Closed-loop steering 
                self.cmd_msg.linear.x  = propulsion_user_input * self.max_vel #[m/s]
                self.cmd_msg.angular.z = steering_user_input # [m]
                self.cmd_msg.linear.z  = 3.0  # Control mode
                
            #If button B is active 
            elif(joy_msg.buttons[2]):   
                # Closed-loop position, Closed-loop steering 
                self.cmd_msg.linear.x  = propulsion_user_input # [m]
                self.cmd_msg.angular.z = steering_user_input # [m]
                self.cmd_msg.linear.z  = 4.0  # Control mode
                
            #If button x is active 
            elif(joy_msg.buttons[0]):   
                # Closed-loop velocity with fixed 1 m/s ref, Closed-loop steering
                self.cmd_msg.linear.x  = 2.0 #[m/s]
                self.cmd_msg.angular.z = 0.0 # [m]
                self.cmd_msg.linear.z  = 5.0 # Control mode
                
            #If button y is active 
            elif(joy_msg.buttons[3]):   
                # Reset Encoder
                self.cmd_msg.linear.x  = 0.0
                self.cmd_msg.angular.z = 0.0
                self.cmd_msg.linear.z  = 6.0  # Control mode
                
            #If left trigger is active 
            elif (joy_msg.buttons[6]):
                # No ctl_ref msg published!
                return
                
            #If right joy pushed
            # elif(joy_msg.buttons[11]):
            #      # Template for a custom mode
            #     self.cmd_msg.linear.x  = 0.0
            #     self.cmd_msg.angular.z = 0.0
            #     self.cmd_msg.linear.z  = 7.0 # Control mode
                
            #If bottom arrow is active
            # elif(joy_msg.axes[7]):
            #     # Template for a custom mode
            #     self.cmd_msg.linear.x  = 0.0
            #     self.cmd_msg.angular.z = 0.0
            #     self.cmd_msg.linear.z  = 8.0 # Control mode

            # Defaults operation
            # No active button
            else:
                # Closed-loop velocity, Open-loop steering
                self.cmd_msg.linear.x  = propulsion_user_input * self.max_vel #[m/s]
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
                self.cmd_msg.linear.z  = 0.0  # Control mode
        
        # Deadman is un-pressed
        else:
            # All-stop
            self.cmd_msg.linear.x = 0.0 
            self.cmd_msg.linear.y = 0.0
            self.cmd_msg.linear.z = -1.0            
            self.cmd_msg.angular.x = 0.0
            self.cmd_msg.angular.y = 0.0
            self.cmd_msg.angular.z = 0.0 


        # Publish cmd msg
        self.pub_cmd.publish( self.cmd_msg )
            

def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    rclpy.spin(node)
    rclpy.shutdown()

##############
if __name__ == '__main__':
    main()
