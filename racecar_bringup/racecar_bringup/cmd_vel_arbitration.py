#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy


class Arbitration(Node):
    def __init__(self):
        super().__init__("arbitration")

        self._delay_sec = (
            self.declare_parameter("delay_sec", 0.5).get_parameter_value().double_value
        )
        self._time_called = [self.get_clock().now() for _ in range(9)]

        self._cmd_vel_pub = self.create_publisher(Twist, "cmd_vel_output", 1)
        self._status_pub = self.create_publisher(Int32, "status", 1)

        self.joy_subscription = self.create_subscription(
            Joy, "joy", self.check_autopilot_status, 1
        )

        self._cmd_vel_subs = []
        for i in range(8):
            sub = self.create_subscription(
                Twist,
                f"cmd_vel_abtr_{i}",
                lambda msg, priority=i: self.cmd_vel_callback(msg, priority),
                1,
            )
            self._cmd_vel_subs.append(sub)

        self.__autopilot_active = bool(False)

    def check_autopilot_status(self, msg: Joy):
        """Checks if `LB` + `LT` are pressed (i.e. deadman switch + autopilot-using-behaviors mode)."""
        self.__autopilot_active = msg.buttons[4] and msg.buttons[6]
        # NOTE: On DirectInput mode, LB is msg.buttons[6]. On XInput mode, LB is msg.axes[2] (== -1.0).

    def cmd_vel_callback(self, msg, priority: int):
        self._time_called[priority] = self.get_clock().now()
        pub = True
        for i in range(1, priority + 1):
            if (
                self._time_called[priority] - self._time_called[i - 1]
            ).nanoseconds / 1e9 < self._delay_sec:
                pub = False
                break
        if pub:
            if self.__autopilot_active:
                self._cmd_vel_pub.publish(msg)
            elif priority == 0:
                self._cmd_vel_pub.publish(msg)

            status = Int32(data=priority)
            self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    arbitration = Arbitration()
    try:
        rclpy.spin(arbitration)
    except KeyboardInterrupt:
        pass
    finally:
        arbitration.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
