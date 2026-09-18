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
from std_msgs.msg import Bool, String


class KeyboardCmdVel(Node):
    def __init__(self) -> None:
        super().__init__("jaguar_keyboard_cmd_vel")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)
        self.safe_pub = self.create_publisher(Bool, "/jaguar/safe_stop", 10)
        self.status_pub = self.create_publisher(String, "/jaguar/keyboard_teleop_status", 10)
        self.timer = self.create_timer(0.02, self._publish)
        self.lock = threading.Lock()
        self.current = np.zeros(3, dtype=np.float32)
        self.desired = np.zeros(3, dtype=np.float32)
        self.target_linear = 0.30
        self.target_yaw = 0.30
        self.last_motion = 0.0
        self.motion_key = None
        self.last_key = "-"
        self.last_event = "Menunggu input"
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
            self.last_key = "SPACE" if key == " " else key.upper()
            self.last_event = f"Key {self.last_key} diterima"
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
                self.last_event = "Target speed dinaikkan"
            elif key == "o":
                self.target_linear = max(0.05, self.target_linear - 0.05)
                self.target_yaw = max(0.05, self.target_yaw - 0.05)
                self.last_event = "Target speed diturunkan"
            elif key in ("x", " "):
                self.desired[:] = 0.0
                self.motion_key = None
                if key == " ":
                    self.safe_pub.publish(Bool(data=True))
                    self.last_event = "SAFE PARK dikirim"
            elif key in ("1", "2", "3"):
                buttons = [0, 0, 0, 0, 0, 0, 0, 0]
                buttons[{"2": 0, "3": 1, "1": 2}[key]] = 1
                self.joy_pub.publish(Joy(buttons=buttons, axes=[]))

    def render(self) -> None:
        with self.lock:
            current = self.current.copy()
            desired = self.desired.copy()
            target_linear = self.target_linear
            target_yaw = self.target_yaw
            last_key = self.last_key
            last_event = self.last_event
            motion_key = self.motion_key or "-"
        joystick = self.joystick_present()
        watchdog = "ACTIVE" if motion_key != "-" else "IDLE"
        deadlock = "KEY HEARTBEAT OK" if motion_key == "-" else (
            f"{max(0.0, 0.30 - (time.monotonic() - self.last_motion)):.2f}s remaining"
        )
        lines = [
            "\033[2J\033[H",
            "NXP JAGUAR - KEYBOARD CMD_VEL",
            "=" * 72,
            f"Input       : {'LOCKED (joystick detected)' if joystick else 'KEYBOARD ACTIVE'}",
            f"Last key    : {last_key:>5}    Event: {last_event}",
            f"Motion key  : {motion_key:>5}    Watchdog: {watchdog} | {deadlock}",
            "",
            "COMMAND STATUS",
            f"  Current    : Vx {current[0]:+6.2f} m/s | Vy {current[1]:+6.2f} m/s | Wz {current[2]:+6.2f} rad/s",
            f"  Desired    : Vx {desired[0]:+6.2f} m/s | Vy {desired[1]:+6.2f} m/s | Wz {desired[2]:+6.2f} rad/s",
            f"  Target     : linear {target_linear:.2f} m/s | yaw {target_yaw:.2f} rad/s",
            "",
            "KEYS",
            "  1  STANDBY       2  STANDUP        3  WALK",
            "  W  forward       S  backward       A  strafe left",
            "  D  strafe right  Q  turn left      E  turn right",
            "  O  slower        P  faster         SPACE  SAFE PARK",
            "  Ctrl-C  exit",
            "",
            "Safety: command ramps at 1.0 m/s^2 and yaw 1.0 rad/s^2.",
            "If key-repeat stops for 0.30 s, movement ramps automatically to zero.",
            "=" * 72,
        ]
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

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
            status = String()
            source = "LOCKED_JOYSTICK" if self.joystick_present() else "KEYBOARD"
            watchdog = "ACTIVE" if self.motion_key is not None else "IDLE"
            status.data = (
                f"alive=1 source={source} watchdog={watchdog} "
                f"cmd={self.current[0]:+.3f},{self.current[1]:+.3f},{self.current[2]:+.3f} "
                f"target={self.target_linear:.2f},{self.target_yaw:.2f}"
            )
            self.status_pub.publish(status)

    def run_keyboard(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().error("Keyboard teleop requires an interactive terminal.")
            return
        self.old_termios = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            last_render = 0.0
            while self.running and rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if ready:
                    value = sys.stdin.read(1)
                    if value == "\x03":
                        break
                    self.key(value)
                now = time.monotonic()
                if now - last_render >= 0.20:
                    self.render()
                    last_render = now
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_termios)

    def shutdown(self) -> None:
        """Publish a short zero-command tail before stopping the node."""
        zero = Twist()
        for _ in range(5):
            self.cmd_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)


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
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
