#!/usr/bin/env python3
"""Small dependency-free velocity mux for the Jaguar bringup.

Xbox has priority while its heartbeat is fresh.  Keyboard is used after the
Xbox input times out.  A zero command is published when neither source is
available so the controller never retains a stale velocity.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class JaguarCmdVelMux(Node):
    def __init__(self) -> None:
        super().__init__("jaguar_cmd_vel_mux")
        self.declare_parameter("xbox_timeout", 0.30)
        self.declare_parameter("keyboard_timeout", 0.30)
        self.xbox_timeout = float(self.get_parameter("xbox_timeout").value)
        self.keyboard_timeout = float(self.get_parameter("keyboard_timeout").value)

        self.xbox_cmd = Twist()
        self.keyboard_cmd = Twist()
        self.xbox_stamp = 0.0
        self.keyboard_stamp = 0.0

        self.create_subscription(Twist, "/cmd_vel/xbox", self._xbox_cb, 10)
        self.create_subscription(Twist, "/cmd_vel/keyboard", self._keyboard_cb, 10)
        self.output = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.02, self._publish)

        self.get_logger().info(
            "Jaguar cmd_vel mux active: xbox(priority=100) -> keyboard(priority=10)"
        )

    def _xbox_cb(self, msg: Twist) -> None:
        self.xbox_cmd = msg
        self.xbox_stamp = time.monotonic()

    def _keyboard_cb(self, msg: Twist) -> None:
        self.keyboard_cmd = msg
        self.keyboard_stamp = time.monotonic()

    def _publish(self) -> None:
        now = time.monotonic()
        if now - self.xbox_stamp <= self.xbox_timeout:
            cmd = self.xbox_cmd
        elif now - self.keyboard_stamp <= self.keyboard_timeout:
            cmd = self.keyboard_cmd
        else:
            cmd = Twist()
        self.output.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JaguarCmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.output.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
