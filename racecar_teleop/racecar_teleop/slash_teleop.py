#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class Teleop(Node):
    """
    This class takes inputs from `joy_node`, and converts them into commands
    sent to the autopilot controller.
    """

    def __init__(self):
        super().__init__("teleop")

        self.max_vel = (
            self.declare_parameter("max_vel", 4.0).get_parameter_value().double_value
        )
        self.max_volt = (
            self.declare_parameter("max_volt", 8.0).get_parameter_value().double_value
        )
        self.maxStAng = (
            self.declare_parameter("max_angle", 40).get_parameter_value().double_value
        )
        self.ps4 = self.declare_parameter("ps4", False).get_parameter_value().bool_value
        self.input_mode = (
            self.declare_parameter("input_mode", "X").get_parameter_value().string_value
        )  # X: XInput, D: DirectInput

        self.cmd2rad = self.maxStAng * 2 * 3.141592 / 360
        self.joystickCompatibilityWarned = False

        self.pub_cmd = self.create_publisher(Twist, "ctl_ref", 1)

        # Always create subscribers last
        self.sub_joy = self.create_subscription(Joy, "joy", self.joy_callback, 1)

        self.axes: dict[str, float] = {
            "LJH": -1.0,
            "LJV": -1.0,
            "RJH": -1.0,
            "RJV": -1.0,
            "LT": -1.0,
            "RT": -1.0,
            "DPH": 0.0,
            "DPV": 0.0,
        }

        self.buttons: dict[str, bool] = {
            "A": False,
            "B": False,
            "X": False,
            "Y": False,
            "LB": False,
            "RB": False,
            "BACK": False,
            "START": False,
            "HOME": False,
            "LJP": False,
            "RJP": False,
        }

    def __get_inputs(self, msg: Joy):
        if self.input_mode == "X":
            self.axes["LJH"] = msg.axes[0]
            self.axes["LJV"] = msg.axes[1]
            self.axes["RJH"] = msg.axes[2]
            self.axes["RJV"] = msg.axes[3]
            self.axes["LT"] = msg.axes[4]
            self.axes["RT"] = msg.axes[5]
            self.axes["DPH"] = msg.axes[6]
            self.axes["DPH"] = msg.axes[7]

            self.buttons["A"] = bool(msg.buttons[0])
            self.buttons["B"] = bool(msg.buttons[1])
            self.buttons["X"] = bool(msg.buttons[2])
            self.buttons["Y"] = bool(msg.buttons[3])
            self.buttons["LB"] = bool(msg.buttons[4])
            self.buttons["RB"] = bool(msg.buttons[5])
            self.buttons["BACK"] = bool(msg.buttons[6])
            self.buttons["START"] = bool(msg.buttons[7])
            self.buttons["HOME"] = bool(msg.buttons[8])
            self.buttons["LJP"] = bool(msg.buttons[9])
            self.buttons["RJP"] = bool(msg.buttons[10])
        elif self.ps4 or self.input_mode == "D":
            self.axes["LJH"] = msg.axes[0]
            self.axes["LJV"] = msg.axes[1]
            self.axes["RJH"] = msg.axes[2]
            self.axes["RJV"] = msg.axes[3]
            self.axes["DPH"] = msg.axes[4]
            self.axes["DPH"] = msg.axes[5]

            self.axes["LT"] = msg.buttons[6]
            self.axes["RT"] = msg.buttons[7]

            self.buttons["A"] = bool(msg.buttons[1])
            self.buttons["B"] = bool(msg.buttons[2])
            self.buttons["X"] = bool(msg.buttons[0])
            self.buttons["Y"] = bool(msg.buttons[3])
            self.buttons["LB"] = bool(msg.buttons[4])
            self.buttons["RB"] = bool(msg.buttons[5])
            self.buttons["BACK"] = bool(msg.buttons[8])
            self.buttons["START"] = bool(msg.buttons[9])
            self.buttons["LJP"] = bool(msg.buttons[10])
            self.buttons["RJP"] = bool(msg.buttons[11])
        else:
            raise ValueError(
                f"Invalid value for parameter `input_mode`: {self.input_mode}"
            )

    def joy_callback(self, msg: Joy):
        min_axes = 5 if self.ps4 else 4
        if len(msg.axes) < min_axes or len(msg.buttons) < 7:
            if not self.joystickCompatibilityWarned:
                self.get_logger().info(
                    "slash_teleop: Received topic doesn't have enough axes and/or buttons. If a Logitech gamepad is used, make sure also it is in X mode. Will not warn again."
                )
                self.joystickCompatibilityWarned = True
            return

        self.joystickCompatibilityWarned = (
            False  # reset in case we switch mode on the gamepad
        )

        self.__get_inputs(msg)

        propulsion_user_input = self.axes["RJV"]  # Right joystick vertical
        steering_user_input = self.axes["LJH"]  # Left joystick horizontal

        self.cmd_msg = Twist()

        # "Poor man's deadman switch"
        if self.buttons["LB"]:
            # If right button is active
            if self.buttons["RB"]:
                # Fully Open-Loop (WARNING: may result in crashes)
                self.cmd_msg.linear.x = propulsion_user_input * self.max_volt  # V
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
                self.cmd_msg.linear.z = 1.0  # CtrlChoice

            elif self.buttons["RJP"]:
                """
                GRO501-1: closed-loop velocity fixed @ X m/s, open-loop
                steering, where X is determined "on-site".
                """
                self.cmd_msg.linear.x = 2.0  # m/s
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
                self.cmd_msg.linear.z = 0.0  # high-level mode

            # If right trigger is active
            elif self.buttons["START"]:
                """
                GRO501-1: closed-loop position fixed @ X m, open-loop
                steering, where X is determined "on-site".
                """
                self.cmd_msg.linear.x = 2.0  # [m]
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
                self.cmd_msg.linear.z = 2.0  # CtrlChoice

            elif self.buttons["A"]:
                # Closed-loop velocity, Closed-loop steering
                self.cmd_msg.linear.x = propulsion_user_input * self.max_vel  # [m/s]
                self.cmd_msg.angular.z = steering_user_input  # [m]
                self.cmd_msg.linear.z = 3.0  # Control mode

            elif self.buttons["B"]:
                # Closed-loop position, Closed-loop steering
                self.cmd_msg.linear.x = propulsion_user_input  # [m]
                self.cmd_msg.angular.z = steering_user_input  # [m]
                self.cmd_msg.linear.z = 4.0  # Control mode

            elif self.buttons["X"]:
                # Closed-loop velocity with fixed 1 m/s ref, Closed-loop steering
                self.cmd_msg.linear.x = 2.0  # [m/s]
                self.cmd_msg.angular.z = 0.0  # [m]
                self.cmd_msg.linear.z = 5.0  # Control mode

            elif self.buttons["Y"]:
                # Reset Encoder
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.0
                self.cmd_msg.linear.z = 6.0  # Control mode

            elif (
                self.axes["LT"] >= 0.0
                if (self.ps4 or self.input_mode == "D")
                else self.axes["LT"] <= 0.0
            ):
                # Joystick deactivated. No reference published.
                return

            # Template for a custom mode
            # elif(self.buttons[<ID>]):
            #     self.cmd_msg.linear.x  = 0.0
            #     self.cmd_msg.angular.z = 0.0
            #     self.cmd_msg.linear.z  = 7.0 # Control mode

            else:
                # Open-loop velocity (saturated), Open-loop steering
                self.cmd_msg.linear.x = propulsion_user_input * self.max_vel  # [m/s]
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
                self.cmd_msg.linear.z = 7.0  # Control mode

        else:
            # All-stop
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.linear.y = 0.0
            self.cmd_msg.linear.z = -1.0
            self.cmd_msg.angular.x = 0.0
            self.cmd_msg.angular.y = 0.0
            self.cmd_msg.angular.z = 0.0

        self.pub_cmd.publish(self.cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
