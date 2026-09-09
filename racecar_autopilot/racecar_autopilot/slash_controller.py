#!/usr/bin/env python
import rclpy
import math
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class SlashController(Node):

    def __init__(self):
        super().__init__("controller")

        # Init subscribers
        self.sub_ref = self.create_subscription(Twist, "ctl_ref", self.read_ref, 1)
        self.sub_prop = self.create_subscription(
            Float32MultiArray, "prop_sensors", self.read_arduino, 1
        )
        self.sub_laser = self.create_subscription(
            Twist, "car_position", self.read_laser, 1
        )

        # Init publishers
        self.pub_cmd = self.create_publisher(Twist, "prop_cmd", 1)

        # Timer
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timed_controller)

        # Parameters

        # Controller
        self.steering_offset = 0.0  # To adjust according to the vehicle

        # TODO: test on real racecar to validate
        # Design offline, paste here. Shapes must match the x you assemble below.
        self.params_autopilot = {
            "K": None,  # (m, n)
            "ubar": None,  # (m,) feedforward; often zeros
        }
        self.params_parking = {
            "K": None,
            "ubar": None,
        }

        # Memory

        # References Inputs
        self.propulsion_ref = 0
        self.steering_ref = 0
        self.high_level_mode = 0  # Control mode of this controller node

        # Ouput commands
        self.propulsion_cmd = 0  # Command sent to propulsion
        self.arduino_mode = 0  # Control mode
        self.steering_cmd = 0  # Command sent to the steering servo

        # Sensings inputs
        self.laser_y = 0
        self.laser_theta = 0
        self.velocity = 0
        self.position = 0

        # Filters
        self.laser_y_old = 0
        self.laser_dy_fill = 0

    #######################################
    def timed_controller(self):

        # Computation of dy / dt with filtering
        self.laser_dy_fill = (
            0.9 * self.laser_dy_fill + 0.1 * (self.laser_y - self.laser_y_old) / self.dt
        )
        self.laser_y_old = self.laser_y

        if self.high_level_mode < 0:
            # Full stop mode
            self.propulsion_cmd = 0  # Command sent to propulsion
            self.arduino_mode = 0  # Control mode
            self.steering_cmd = 0  # Command sent to the steering servo

        else:

            # APP2 (open-loop steering) Controllers Bellow
            if self.high_level_mode == 1:
                # Open-Loop propulsion and steering
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode = 1
                self.steering_cmd = self.steering_ref + self.steering_offset

            # For compatibility mode 0 needs to be closed-loop velocity
            elif self.high_level_mode == 0:
                # Closed-loop velocity on arduino, open-loop steering
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode = 2
                self.steering_cmd = self.steering_ref + self.steering_offset

            elif self.high_level_mode == 2:
                # Closed-loop position on arduino, open-loop steering
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode = 3
                self.steering_cmd = self.steering_ref + self.steering_offset

            # ----------------------------------------------------------
            # TODO: test on real racecar to validate
            # APP4 — catalog of signals available
            #
            # 1. Measurements (updated every self.dt)
            #    self.position       encoder, longitudinal position [m]
            #    self.velocity       encoder, longitudinal speed [m/s]
            #    self.laser_y        lidar, lateral position [m]
            #    self.laser_theta    lidar, heading [rad]
            #    self.laser_dy_fill  filtered dy/dt [m/s]
            #
            # 2. References (from ctl_ref)
            #    self.propulsion_ref  longitudinal ref (speed or position,
            #                         depends on the high-level joystick mode)
            #    self.steering_ref    steering / lateral ref
            #
            # 3. Controllable actions — two channels; meaning depends on
            #    arduino_mode
            #    u[0]  propulsion
            #          arduino_mode == 0 → open-loop voltage V (PWM)
            #          arduino_mode == 2 → closed-loop velocity setpoint
            #          arduino_mode == 3 → closed-loop position setpoint
            #    u[1]  steering δ  (steering_offset is added after the policy)
            #    self.arduino_mode  is a choice you must set below (0 / 2 / 3)
            # ----------------------------------------------------------

            elif self.high_level_mode == 3 or self.high_level_mode == 5:
                # Autopilot (high-level mode 3 or 5)

                #########################################################
                # TODO: complètez — assemblage de x, r et arduino_mode

                # x = np.array([ ... ])   # pick from the catalog
                # r = np.array([ ... ])

                x = None
                r = None

                u = self.ctl_autopilot(x, r)

                self.propulsion_cmd = u[0]
                self.steering_cmd = u[1] + self.steering_offset
                self.arduino_mode = 0  # complètez: 0, 2 ou 3
                #########################################################

            elif self.high_level_mode == 4:
                # Parking (high-level mode 4)

                #########################################################
                # TODO: complètez — assemblage de x, r et arduino_mode

                # x = np.array([ ... ])   # pick from the catalog
                # r = np.array([ ... ])

                x = None
                r = None

                u = self.ctl_parking(x, r)

                self.propulsion_cmd = u[0]
                self.steering_cmd = u[1] + self.steering_offset
                self.arduino_mode = 0  # complètez: 0, 2 ou 3
                #########################################################

            elif self.high_level_mode == 6:
                # Reset encoders
                self.propulsion_cmd = 0
                self.arduino_mode = 4
                self.steering_cmd = 0

            elif self.high_level_mode == 7:
                # Closed-loop velocity open-loop steering
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode = 2
                self.steering_cmd = self.steering_ref + self.steering_offset

            elif self.high_level_mode == 8:
                # Template for custom controllers

                self.steering_cmd = 0 + self.steering_offset
                self.propulsion_cmd = 0
                self.arduino_mode = 0  # Mode ??? on arduino

        self.send_arduino()

    #######################################
    def ctl_autopilot(self, x, r, t=0, params=None):

        params = self.params_autopilot if params is None else params

        # K = params["K"]
        # ubar = params["ubar"]
        # u = ubar - K @ (x - r)

        u = np.zeros(2)
        return u

    #######################################
    def ctl_parking(self, x, r, t=0, params=None):

        params = self.params_parking if params is None else params

        # K = params["K"]
        # ubar = params["ubar"]
        # u = ubar - K @ (x - r)

        u = np.zeros(2)
        return u

    #######################################
    def read_ref(self, ref_msg):

        # Read received references
        self.propulsion_ref = ref_msg.linear.x
        self.high_level_mode = ref_msg.linear.z
        self.steering_ref = ref_msg.angular.z

    #######################################
    def read_laser(self, msg):

        self.laser_y = msg.linear.y
        self.laser_theta = msg.angular.z

    #######################################
    def read_arduino(self, msg):

        # Read feedback from arduino
        self.velocity = msg.data[1]
        self.position = msg.data[0]

    ##########################################################################################
    def send_arduino(self):

        # Init encd_info msg
        cmd_prop = Twist()

        # Msg
        cmd_prop.linear.x = float(self.propulsion_cmd)  # Command sent to propulsion
        cmd_prop.linear.z = float(self.arduino_mode)  # Control mode

        cmd_prop.angular.z = float(
            self.steering_cmd
        )  # Command sent to the steering servo

        # Publish cmd msg
        self.pub_cmd.publish(cmd_prop)

    #######################################
    def pub_kinematic(self):
        # init encd_info msg
        pos = Twist()
        vel = Twist()
        acc = Twist()

        # Msg
        pos.linear.x = 0
        vel.linear.x = 0
        acc.linear.x = 0

        # Publish cmd msg
        self.pub_pos.publish(pos)
        self.pub_vel.publish(vel)
        self.pub_acc.publish(acc)


def main(args=None):
    rclpy.init(args=args)
    node = SlashController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


#########################################

if __name__ == "__main__":
    main()
