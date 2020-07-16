#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


#########################################
class WallEstimator(object):

    #######################################
    
    def __init__(self):

        # Init subscribers  
        self.sub_ref     = rospy.Subscriber("scan", LaserScan, self.read_scan, queue_size=1)

        # Init publishers
        self.pub_y    = rospy.Publisher("car_position", Twist , queue_size=1)
        
        # Outputs
        self.y_estimation     = 0
        self.dy_estimation    = 0
        self.theta_estimation = 0
        
        # Memory
        self.y_left      = 0 
        self.y_right     = 0
        self.theta_left  = 0
        self.theta_right = 0
        
    ########################################
    def read_scan( self, scan_msg ):
        
        ranges = np.array( scan_msg.ranges )

        #######################################################################
        # Least square
        #######################################################################
        
        # Left side
        
        d_data      = []
        theta_data  = []
        n_good_scan = 0
        
        for i in range(70,110):
            
            scan_is_good = (( ranges[i] > scan_msg.range_min) &
                            ( ranges[i] < scan_msg.range_max) )
                           
            if scan_is_good:
                
                d_data.append( ranges[i] )
                theta_data.append( scan_msg.angle_min + i * scan_msg.angle_increment )
                
                n_good_scan = n_good_scan + 1
                
        if n_good_scan > 2 :
                
            d     = np.array( d_data )
            theta = np.array( theta_data ) 
            
            y     = d * np.sin( theta )
            x     = d * np.cos( theta )
            
            ones  = np.ones( x.shape )
            
            A = np.column_stack( ( x ,  ones ) )
            
            # Least square
            estimation = np.linalg.lstsq(A,y)[0] # (ATA)^-1 ATy
            
            m = estimation[0] # slope
            b = estimation[1] # offset
            
            self.theta_left = np.arctan( m )
            self.y_left     = b
            
        # Right side
        
        d_data      = []
        theta_data  = []
        n_good_scan = 0
        
        for i in range(250,290):
            
            scan_is_good = (( ranges[i] > scan_msg.range_min) &
                            ( ranges[i] < scan_msg.range_max) )
                           
            if scan_is_good:
                
                d_data.append( ranges[i] )
                theta_data.append( scan_msg.angle_min + i * scan_msg.angle_increment )
                
                n_good_scan = n_good_scan + 1
                
        if n_good_scan > 2 :
                
            d     = np.array( d_data )
            theta = np.array( theta_data ) 
            
            y     = d * np.sin( theta )
            x     = d * np.cos( theta )
            
            ones  = np.ones( x.shape )
            
            A = np.column_stack( ( x ,  ones ) )
            
            # Least square
            estimation = np.linalg.lstsq(A,y)[0] # (ATA)^-1 ATy
            
            m = estimation[0] # slope
            b = estimation[1] # offset
            
            self.theta_right = np.arctan( m )
            self.y_right    = b
            
        self.theta_estimation = 0.5 * ( self.theta_left + self.theta_right )
        self.y_estimation     = 0.5 * ( self.y_left + self.y_right )
        
        self.pub_estimate()
    
    ########################################
    def pub_estimate( self ):
        
        msg = Twist()
        
        msg.linear.y  = self.y_estimation
        msg.angular.z = self.theta_estimation
        
        # debug
        
        msg.linear.x  = self.y_left
        msg.linear.z  = self.y_right
        
        msg.angular.x = self.theta_left
        msg.angular.y = self.theta_right
        
        
        self.pub_y.publish( msg )


#########################################

if __name__ == '__main__':
    
    rospy.init_node('estimator',anonymous=False)
    node = WallEstimator()
    rospy.spin()
