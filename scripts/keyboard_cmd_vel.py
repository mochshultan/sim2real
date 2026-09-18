#!/usr/bin/env python3
"""Safe terminal keyboard teleop for Jaguar velocity commands.

The node publishes ``/cmd_vel`` and mode pulses on ``/joy``. Keyboard input is
disabled whenever a local Linux joystick device is present. Movement commands
are ramped and a missing key-repeat heartbeat ramps the command back to zero.
"""

import os
import select
import sys
import termios
import threading
import time
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class KeyboardCmdVel(Node):
    def __init__(self) -> None:
        super().__init__("jaguar_keyboard_cmd_vel")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)
        self.safe_pub = self.create_publisher(Bool, "/jaguar/safe_stop", 10)
        self.timer = self.create_timer(0.02, self._publish)
        self.lock = threading.Lock()
        self.current = np.zeros(3, dtype=np.float32)
        self.desired = np.zeros(3, dtype=np.float32)
        self.target_linear = 0.30
        self.target_yaw = 0.30
        self.last_motion = 0.0
        self.motion_key = None
        self.running = True
        self.old_termios = None

    @staticmethod
    def joystick_present() -> bool:
        try:
            return any(name.startswith("js") for name in os.listdir("/dev/input"))
        except OSError:
            return False

    def key(self, value: str) -> None:
        with self.lock:
            if self.joystick_present():
                self.get_logger().warning("Keyboard disabled: Xbox/Linux joystick detected.")
                return
            now = time.monotonic()
            key = value.lower()
            if key in ("w", "s", "a", "d", "q", "e"):
                self.motion_key = key
                self.last_motion = now
                self.desired[:] = 0.0
                if key == "w": self.desired[0] = self.target_linear
                if key == "s": self.desired[0] = -self.target_linear
                if key == "a": self.desired[1] = self.target_linear
                if key == "d": self.desired[1] = -self.target_linear
                if key == "q": self.desired[2] = self.target_yaw
                if key == "e": self.desired[2] = -self.target_yaw
            elif key == "p":
                self.target_linear = min(1.5, self.target_linear + 0.05)
                self.target_yaw = min(1.2, self.target_yaw + 0.05)
            elif key == "o":
                self.target_linear = max(0.05, self.target_linear - 0.05)
                self.target_yaw = max(0.05, self.target_yaw - 0.05)
            elif key in ("x", " "):
                self.desired[:] = 0.0
                self.motion_key = None
                if key == " ":
                    self.safe_pub.publish(Bool(data=True))
            elif key in ("1", "2", "3"):
                buttons = [0, 0, 0, 0, 0, 0, 0, 0]
                buttons[{"2": 0, "3": 1, "1": 2}[key]] = 1
                self.joy_pub.publish(Joy(buttons=buttons, axes=[]))

    def _publish(self) -> None:
        with self.lock:
            if self.motion_key is not None and time.monotonic() - self.last_motion > 0.30:
                self.desired[:] = 0.0
                self.motion_key = None
            delta = np.array([0.02, 0.02, 0.02], dtype=np.float32)
            self.current += np.clip(self.desired - self.current, -delta, delta)
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = map(float, self.current)
            self.cmd_pub.publish(msg)

    def run_keyboard(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().error("Keyboard teleop requires an interactive terminal.")
            return
        self.old_termios = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running and rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if ready:
                    value = sys.stdin.read(1)
                    if value == "\x03":
                        break
                    self.key(value)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_termios)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardCmdVel()
    thread = threading.Thread(target=node.run_keyboard, daemon=True)
    thread.start()
    print("Jaguar keyboard cmd_vel: hold W/A/S/D/Q/E | O/P speed | SPACE safe-park | Ctrl-C exit")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        with node.lock:
            node.desired[:] = 0.0
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
