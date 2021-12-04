#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class Arbitration:
    def __init__(self):
        self._delay_sec = rospy.get_param('~delay_sec', 0.5)
        t = rospy.get_time()
        self._timeCalled = [t,t,t,t,t,t,t,t,t]

        self._cmd_vel_pub = rospy.Publisher('cmd_vel_output', Twist, queue_size=1)
        self._status_pub = rospy.Publisher('status', Int32, queue_size=1)
        
        self._cmd_vel_sub0 = rospy.Subscriber('cmd_vel_abtr_0', Twist, self.cmd_vel_callback0, queue_size=1)
        self._cmd_vel_sub1 = rospy.Subscriber('cmd_vel_abtr_1', Twist, self.cmd_vel_callback1, queue_size=1)
        self._cmd_vel_sub2 = rospy.Subscriber('cmd_vel_abtr_2', Twist, self.cmd_vel_callback2, queue_size=1)
        self._cmd_vel_sub3 = rospy.Subscriber('cmd_vel_abtr_3', Twist, self.cmd_vel_callback3, queue_size=1)
        self._cmd_vel_sub4 = rospy.Subscriber('cmd_vel_abtr_4', Twist, self.cmd_vel_callback4, queue_size=1)
        self._cmd_vel_sub5 = rospy.Subscriber('cmd_vel_abtr_5', Twist, self.cmd_vel_callback5, queue_size=1)
        self._cmd_vel_sub6 = rospy.Subscriber('cmd_vel_abtr_6', Twist, self.cmd_vel_callback6, queue_size=1)
        self._cmd_vel_sub7 = rospy.Subscriber('cmd_vel_abtr_7', Twist, self.cmd_vel_callback7, queue_size=1)

    def cmd_vel_callback(self, msg, priority):
        self._timeCalled[priority] = rospy.get_time();
        pub = True
        for i in range(1,priority+1):
            if self._timeCalled[priority] - self._timeCalled[i-1] < self._delay_sec:
                pub=False
                break
        if pub:
            self._cmd_vel_pub.publish(msg);
            status = Int32()
            status.data = priority
            self._status_pub.publish(status);

    def cmd_vel_callback0(self, msg):
        self.cmd_vel_callback(msg, 0)

    def cmd_vel_callback1(self, msg):
        self.cmd_vel_callback(msg, 1)

    def cmd_vel_callback2(self, msg):
        self.cmd_vel_callback(msg, 2)

    def cmd_vel_callback3(self, msg):
        self.cmd_vel_callback(msg, 3)
        
    def cmd_vel_callback4(self, msg):
        self.cmd_vel_callback(msg, 4)
        
    def cmd_vel_callback5(self, msg):
        self.cmd_vel_callback(msg, 5)
        
    def cmd_vel_callback6(self, msg):
        self.cmd_vel_callback(msg, 6)
        
    def cmd_vel_callback7(self, msg):
        self.cmd_vel_callback(msg, 7)

def main():
    rospy.init_node('arbitration')
    arbitration = Arbitration()
    r = rospy.Rate(1)
    twist = Twist() # null twist
    twist.linear.z = -1 # emergency stop mode
    while not rospy.is_shutdown():
        # If nothing received, we send zero
        arbitration.cmd_vel_callback(twist, 8) # lowest priority
        try:
            r.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    main()

