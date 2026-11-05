#!/usr/bin/env python3
"""Small dependency-free velocity mux for the Jaguar bringup.

Xbox has priority while its heartbeat is fresh.  Keyboard is used after the
Xbox input times out.  A zero command is published when neither source is
available so the controller never retains a stale velocity.
"""

import time
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from motion_limits import declare_motion_limits


class JaguarCmdVelMux(Node):
    def __init__(self) -> None:
        super().__init__("jaguar_cmd_vel_mux")
        self.declare_parameter("xbox_timeout", 0.30)
        self.declare_parameter("keyboard_timeout", 0.30)
        self.motion_limits = declare_motion_limits(self)
        self.xbox_timeout = float(self.get_parameter("xbox_timeout").value)
        self.keyboard_timeout = float(self.get_parameter("keyboard_timeout").value)
        self.linear_accel = self.motion_limits["linear_accel_limit"]
        self.angular_accel = self.motion_limits["angular_accel_limit"]
        if not all(math.isfinite(v) and v > 0 for v in (self.linear_accel, self.angular_accel)):
            raise ValueError("Xbox acceleration limits must be finite and positive")

        self.xbox_cmd = Twist()
        self.keyboard_cmd = Twist()
        self.xbox_stamp = 0.0
        self.keyboard_stamp = 0.0
        self.smoothed_xbox = Twist()
        self.last_publish_time = time.monotonic()

        self.create_subscription(Twist, "/cmd_vel/xbox", self._xbox_cb, 10)
        self.create_subscription(Twist, "/cmd_vel/keyboard", self._keyboard_cb, 10)
        self.output = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.02, self._publish)

        self.get_logger().info(
            "Jaguar cmd_vel mux active: xbox(priority=100) -> keyboard(priority=10)"
        )

    def _xbox_cb(self, msg: Twist) -> None:
        self._clamp_command(msg)
        self.xbox_cmd = msg
        self.xbox_stamp = time.monotonic()

    def _keyboard_cb(self, msg: Twist) -> None:
        self._clamp_command(msg)
        self.keyboard_cmd = msg
        self.keyboard_stamp = time.monotonic()

    def _clamp_command(self, msg: Twist) -> None:
        for vector, axis, limit in ((msg.linear, "x", "max_vx"),
                                    (msg.linear, "y", "max_vy"),
                                    (msg.angular, "z", "max_wz")):
            value = getattr(vector, axis)
            bound = self.motion_limits[limit]
            setattr(vector, axis, max(-bound, min(bound, value)) if math.isfinite(value) else 0.0)

    def _publish(self) -> None:
        now = time.monotonic()
        # Cap elapsed time so a delayed timer cannot cause a large command jump.
        dt = min(max(now - self.last_publish_time, 0.0), 0.02)
        self.last_publish_time = now
        if self.xbox_stamp > 0.0 and now - self.xbox_stamp <= self.xbox_timeout:
            cmd = Twist()
            dx = self.xbox_cmd.linear.x - self.smoothed_xbox.linear.x
            dy = self.xbox_cmd.linear.y - self.smoothed_xbox.linear.y
            distance = math.hypot(dx, dy)
            scale = min(1.0, self.linear_accel * dt / distance) if distance else 0.0
            cmd.linear.x = self.smoothed_xbox.linear.x + dx * scale
            cmd.linear.y = self.smoothed_xbox.linear.y + dy * scale
            delta_yaw = self.xbox_cmd.angular.z - self.smoothed_xbox.angular.z
            yaw_step = self.angular_accel * dt
            cmd.angular.z = self.smoothed_xbox.angular.z + max(-yaw_step, min(yaw_step, delta_yaw))
            self.smoothed_xbox = cmd
        else:
            # Drop stale Xbox history immediately, including on source changes.
            self.smoothed_xbox = Twist()
            if self.keyboard_stamp > 0.0 and now - self.keyboard_stamp <= self.keyboard_timeout:
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
