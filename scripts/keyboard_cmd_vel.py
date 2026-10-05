#!/usr/bin/env python3
"""Safe terminal keyboard teleop for Jaguar velocity commands.

The node publishes ``/cmd_vel/keyboard`` and mode pulses on ``/joy``. Keyboard
input is disabled whenever a local Linux joystick device is present. Movement
commands are ramped and a missing input heartbeat returns the command to zero.
Linear and angular inputs are kept independently, so combinations such as
``W + Q`` produce forward motion while turning left. The always-on terminal UI
also monitors the mux output on ``/cmd_vel``, which is what the controller and
policy receive.
"""

import os
import glob
import select
import struct
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
    TERMINAL_KEY_TIMEOUT = 0.75
    # Linux input-event key codes. Reading /dev/input/event* provides actual
    # key-down/key-up events, unlike a terminal which only provides characters.
    KEY_CODES = {
        2: "1", 3: "2", 4: "3",
        16: "q", 17: "w", 18: "e",
        24: "o", 25: "p",
        30: "a", 31: "s", 32: "d",
        45: "x", 57: " ",
    }
    MOTION_KEYS = frozenset(("w", "s", "a", "d", "q", "e"))
    INPUT_EVENT = struct.Struct("llHHI")
    EV_KEY = 0x01

    def __init__(self) -> None:
        super().__init__("jaguar_keyboard_cmd_vel")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel/keyboard", 10)
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)
        self.safe_pub = self.create_publisher(Bool, "/jaguar/safe_stop", 10)
        self.status_pub = self.create_publisher(String, "/jaguar/keyboard_teleop_status", 10)
        self.create_subscription(Twist, "/cmd_vel", self._model_cmd_cb, 10)
        self.timer = self.create_timer(0.02, self._publish)
        self.ui_timer = self.create_timer(0.20, self.render)
        self.lock = threading.Lock()
        self.current = np.zeros(3, dtype=np.float32)
        self.desired = np.zeros(3, dtype=np.float32)
        self.model_input = np.zeros(3, dtype=np.float32)
        self.model_input_stamp = 0.0
        self.target_linear = 0.30
        self.target_yaw = 0.30
        self.acceleration = 3.0
        self.deceleration = 10.0
        self.last_motion = 0.0
        self.pressed_keys = set()
        self.input_backend = "STARTING"
        self.input_heartbeat = time.monotonic()
        self.last_key = "-"
        self.last_event = "Menunggu input"
        self.running = True
        self.old_termios = None

    def _model_cmd_cb(self, msg: Twist) -> None:
        """Track the post-mux command consumed by the controller/policy."""
        with self.lock:
            self.model_input[:] = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.model_input_stamp = time.monotonic()

    @staticmethod
    def joystick_present() -> bool:
        try:
            return any(name.startswith("js") for name in os.listdir("/dev/input"))
        except OSError:
            return False

    def _update_desired_locked(self) -> None:
        """Compute all velocity axes from the keys currently held down."""
        previous = self.desired.copy()
        self.desired[0] = self.target_linear * (
            int("w" in self.pressed_keys) - int("s" in self.pressed_keys)
        )
        self.desired[1] = self.target_linear * (
            int("a" in self.pressed_keys) - int("d" in self.pressed_keys)
        )
        self.desired[2] = self.target_yaw * (
            int("q" in self.pressed_keys) - int("e" in self.pressed_keys)
        )
        # A released/cancelled axis must stop immediately. Other axes remain
        # untouched, so releasing Q from W+Q keeps forward motion active.
        released = (np.abs(previous) > 1e-6) & (np.abs(self.desired) <= 1e-6)
        self.current[released] = 0.0

    def key_event(self, value: str, pressed: bool, repeat: bool = False) -> None:
        """Apply one real key transition from the Linux input subsystem."""
        with self.lock:
            self.input_heartbeat = time.monotonic()
            key = value.lower()

            if self.joystick_present():
                self.pressed_keys.clear()
                self._update_desired_locked()
                self.last_event = "Keyboard dikunci: joystick terdeteksi"
                return

            self.last_key = "SPACE" if key == " " else key.upper()
            transition = "ditahan" if repeat else ("ditekan" if pressed else "dilepas")
            self.last_event = f"Key {self.last_key} {transition}"

            if key in self.MOTION_KEYS:
                if pressed:
                    self.pressed_keys.add(key)
                    self.last_motion = time.monotonic()
                else:
                    self.pressed_keys.discard(key)
                self._update_desired_locked()
            elif not pressed:
                return
            elif key == "p":
                self.target_linear = min(1.5, self.target_linear + 0.05)
                self.target_yaw = min(1.2, self.target_yaw + 0.05)
                self._update_desired_locked()
                self.last_event = "Target speed dinaikkan"
            elif key == "o":
                self.target_linear = max(0.05, self.target_linear - 0.05)
                self.target_yaw = max(0.05, self.target_yaw - 0.05)
                self._update_desired_locked()
                self.last_event = "Target speed diturunkan"
            elif key in ("x", " "):
                self.pressed_keys.clear()
                self._update_desired_locked()
                if key == " ":
                    self.safe_pub.publish(Bool(data=True))
                    self.last_event = "SAFE PARK dikirim"
            elif key in ("1", "2", "3"):
                buttons = [0, 0, 0, 0, 0, 0, 0, 0]
                buttons[{"2": 0, "3": 1, "1": 2}[key]] = 1
                self.joy_pub.publish(Joy(buttons=buttons, axes=[]))

    def key(self, value: str) -> None:
        """Compatibility handler for the single-key terminal fallback."""
        self.key_event(value, True)

    def render(self) -> None:
        with self.lock:
            current = self.current.copy()
            desired = self.desired.copy()
            target_linear = self.target_linear
            target_yaw = self.target_yaw
            last_key = self.last_key
            last_event = self.last_event
            input_backend = self.input_backend
            held_keys = "+".join(
                key.upper() for key in ("w", "s", "a", "d", "q", "e")
                if key in self.pressed_keys
            ) or "-"
            model_input = self.model_input.copy()
            model_input_stamp = self.model_input_stamp
            active_axes = []
            if abs(float(desired[0])) > 1e-6:
                active_axes.append("linear.x")
            if abs(float(desired[1])) > 1e-6:
                active_axes.append("linear.y")
            if abs(float(desired[2])) > 1e-6:
                active_axes.append("angular.z")
            motion_key = "+".join(active_axes) if active_axes else "-"
        joystick = self.joystick_present()
        watchdog = "ACTIVE" if motion_key != "-" else "IDLE"
        feedback_age = (
            time.monotonic() - model_input_stamp if model_input_stamp > 0.0 else float("inf")
        )
        if feedback_age > 0.50:
            feedback_state = "WAITING /cmd_vel"
        elif np.allclose(model_input, current, atol=0.03):
            feedback_state = f"LIVE, MATCH ({feedback_age * 1000.0:.0f} ms)"
        else:
            feedback_state = f"LIVE, MUX OVERRIDE ({feedback_age * 1000.0:.0f} ms)"
        policy_command = model_input * np.array([2.0, 2.0, 0.25], dtype=np.float32)
        if input_backend == "LINUX_EVDEV":
            deadlock = "KEY STATE OK"
        elif motion_key == "-":
            deadlock = "KEY HEARTBEAT OK"
        else:
            deadlock = f"{max(0.0, self.TERMINAL_KEY_TIMEOUT - (time.monotonic() - self.last_motion)):.2f}s remaining"
        lines = [
            "\033[2J\033[H",
            "NXP JAGUAR - KEYBOARD CMD_VEL",
            "=" * 72,
            f"Input       : {'LOCKED (joystick detected)' if joystick else 'KEYBOARD ACTIVE'}",
            f"Backend     : {input_backend}",
            f"Last key    : {last_key:>5}    Event: {last_event}",
            f"Held keys   : {held_keys:>5}    Axes: {motion_key}",
            f"Watchdog    : {watchdog} | {deadlock}",
            "",
            "COMMAND STATUS",
            f"  Current pub : Vx {current[0]:+6.2f} m/s | Vy {current[1]:+6.2f} m/s | Wz {current[2]:+6.2f} rad/s",
            f"  Key target  : Vx {desired[0]:+6.2f} m/s | Vy {desired[1]:+6.2f} m/s | Wz {desired[2]:+6.2f} rad/s",
            f"  Speed limit : linear {target_linear:.2f} m/s | yaw {target_yaw:.2f} rad/s",
            "",
            "MODEL INPUT (post-mux /cmd_vel)",
            f"  Received    : Vx {model_input[0]:+6.2f} m/s | Vy {model_input[1]:+6.2f} m/s | Wz {model_input[2]:+6.2f} rad/s",
            f"  Policy obs  : X  {policy_command[0]:+6.2f}     | Y  {policy_command[1]:+6.2f}     | Yaw {policy_command[2]:+6.2f}",
            f"  Feedback    : {feedback_state}",
            "",
            "KEYS",
            "  1  STANDBY       2  STANDUP        3  WALK",
            "  W  forward       S  backward       A  strafe left",
            "  D  strafe right  Q  turn left      E  turn right",
            "  Combine keys for hybrid motion, e.g. W+Q = forward-left",
            "  O  slower        P  faster         SPACE  SAFE PARK",
            "  Ctrl-C  exit",
            "",
            f"Ramp up {self.acceleration:.1f}/s; key release stops its axis immediately.",
            "Commands return to zero on key release or input-reader failure.",
            "=" * 72,
        ]
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

    def _publish(self) -> None:
        with self.lock:
            now = time.monotonic()
            if (self.input_backend == "TERMINAL" and self.pressed_keys
                    and now - self.last_motion > self.TERMINAL_KEY_TIMEOUT):
                self.pressed_keys.clear()
                self._update_desired_locked()
            elif self.input_backend == "LINUX_EVDEV" and now - self.input_heartbeat > 0.50:
                self.pressed_keys.clear()
                self._update_desired_locked()
            error = self.desired - self.current
            accelerating = (self.current * self.desired >= 0.0) & (
                np.abs(self.desired) > np.abs(self.current)
            )
            max_step = np.where(
                accelerating,
                self.acceleration * 0.02,
                self.deceleration * 0.02,
            ).astype(np.float32)
            self.current += np.clip(error, -max_step, max_step)
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = map(float, self.current)
            self.cmd_pub.publish(msg)
            status = String()
            source = "LOCKED_JOYSTICK" if self.joystick_present() else "KEYBOARD"
            watchdog = "ACTIVE" if self.pressed_keys else "IDLE"
            status.data = (
                f"alive=1 source={source} watchdog={watchdog} "
                f"cmd={self.current[0]:+.3f},{self.current[1]:+.3f},{self.current[2]:+.3f} "
                f"target={self.target_linear:.2f},{self.target_yaw:.2f}"
            )
            self.status_pub.publish(status)

    @staticmethod
    def _open_event_devices():
        devices = {}
        denied = []
        for path in sorted(glob.glob("/dev/input/event*")):
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                devices[fd] = path
            except PermissionError:
                denied.append(path)
            except OSError:
                continue
        return devices, denied

    def _run_linux_input(self, devices) -> None:
        with self.lock:
            self.input_backend = "LINUX_EVDEV"
            self.input_heartbeat = time.monotonic()
        self.get_logger().info(
            "Keyboard input-event backend active on: " + ", ".join(devices.values())
        )
        while self.running and rclpy.ok() and devices:
            with self.lock:
                self.input_heartbeat = time.monotonic()
            readable, _, _ = select.select(list(devices), [], [], 0.10)
            for fd in readable:
                try:
                    data = os.read(fd, self.INPUT_EVENT.size * 64)
                    if not data:
                        raise OSError("input device disconnected")
                except BlockingIOError:
                    continue
                except OSError as exc:
                    self.get_logger().warning(f"Input device {devices[fd]} stopped: {exc}")
                    os.close(fd)
                    del devices[fd]
                    with self.lock:
                        self.pressed_keys.clear()
                        self._update_desired_locked()
                    continue

                complete = len(data) - (len(data) % self.INPUT_EVENT.size)
                for offset in range(0, complete, self.INPUT_EVENT.size):
                    _, _, event_type, code, event_value = self.INPUT_EVENT.unpack_from(data, offset)
                    key = self.KEY_CODES.get(code)
                    if event_type == self.EV_KEY and key is not None and event_value in (0, 1, 2):
                        self.key_event(key, event_value != 0, repeat=event_value == 2)

        for fd in list(devices):
            os.close(fd)
        with self.lock:
            self.pressed_keys.clear()
            self._update_desired_locked()

    def _run_terminal_fallback(self) -> None:
        if not sys.stdin.isatty():
            with self.lock:
                self.input_backend = "UNAVAILABLE"
            self.get_logger().error(
                "No readable /dev/input/event* device and stdin is not interactive. "
                "Add the ROS user to the input group or grant access to keyboard event devices."
            )
            return
        with self.lock:
            self.input_backend = "TERMINAL"
        self.get_logger().info(
            "Reading keys from this terminal; simultaneous key-down/key-up "
            "is unavailable."
        )
        self.old_termios = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running and rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if ready:
                    value = os.read(sys.stdin.fileno(), 1).decode(errors="ignore")
                    if value == "\x03":
                        break
                    self.key(value)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_termios)

    def run_keyboard(self) -> None:
        # The operator types into this terminal (often over SSH). A readable
        # local event device may belong to another keyboard or peripheral, so
        # it must not take input away from an interactive terminal.
        if sys.stdin.isatty():
            self._run_terminal_fallback()
            return

        devices, denied = self._open_event_devices()
        if devices:
            self._run_linux_input(devices)
            return
        if denied:
            self.get_logger().warning(
                "Permission denied for input devices: " + ", ".join(denied)
            )
        self._run_terminal_fallback()

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
            node.pressed_keys.clear()
            node._update_desired_locked()
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
