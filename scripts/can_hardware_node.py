#!/usr/bin/env python3
"""
NXP Jaguar ROS 2 RobStride RS00 CAN Hardware Driver Node.
Interfaces 12 RobStride RS00 motors across can0 & can1 with ROS 2.
Subscribes: /joint_commands (sensor_msgs/JointState)
Publishes:  /joint_states (sensor_msgs/JointState)
            /jaguar/hardware_status (std_msgs/String)
"""

import sys
import os
import time
import math
import atexit
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32MultiArray

import parameters as P
from robstride_motor_lib import RobStrideMotorController

# Remapping dictionary: Isaac / ROS joint name to ROS CAN array index [0..11]
# ROS order: BL (0..2), BR (3..5), FL (6..8), FR (9..11)
NAME_TO_ROS_INDEX = {
    # ROS CAN Names
    'BL_collar_joint': 0, 'BL_hip_joint': 1, 'BL_knee_joint': 2,
    'BR_collar_joint': 3, 'BR_hip_joint': 4, 'BR_knee_joint': 5,
    'FL_collar_joint': 6, 'FL_hip_joint': 7, 'FL_knee_joint': 8,
    'FR_collar_joint': 9, 'FR_hip_joint': 10, 'FR_knee_joint': 11,
    # Isaac Lab Names
    'Bl_roll_joint': 0, 'Bl_hip_pitch_joint': 1, 'Bl_knee_joint': 2,
    'Br_roll_joint': 3, 'Br_hip_pitch_joint': 4, 'Br_knee_joint': 5,
    'Fl_roll_joint': 6, 'Fl_hip_pitch_joint': 7, 'Fl_knee_joint': 8,
    'Fr_roll_joint': 9, 'Fr_hip_pitch_joint': 10, 'Fr_knee_joint': 11,
}

class RobotHardwareState:
    def __init__(self):
        self.lock = threading.Lock()
        self.pos = P.STANDBY_ANGLE.copy()
        self.vel = [0.0] * P.N_JOINTS
        self.tau = [0.0] * P.N_JOINTS
        self.temp = [25.0] * P.N_JOINTS
        self.errors = [0] * P.N_JOINTS
        self.active = False

class RobotHardwareCommand:
    def __init__(self):
        self.lock = threading.Lock()
        self.pos = P.STANDBY_ANGLE.copy()
        self.vel = [0.0] * P.N_JOINTS
        self.tau = [0.0] * P.N_JOINTS
        self.kp = P.KP_GAIN.copy()
        self.kd = P.KD_GAIN.copy()
        self.enabled = False

class CanHardwareDriverNode(Node):
    def __init__(self):
        super().__init__("jaguar_can_hardware")

        self.get_logger().info("Initializing NXP Jaguar RobStride RS00 CAN Hardware Driver...")

        # Declare parameters dynamically sourced from parameters.py (config/sim2real.yaml)
        self.declare_parameter("rate_hz", P.CAN_HZ)
        self.declare_parameter("default_coxa_kp", float(P.KP_GAIN[0]))
        self.declare_parameter("default_coxa_kd", float(P.KD_GAIN[0]))
        self.declare_parameter("default_kp", float(P.KP_GAIN[1]))
        self.declare_parameter("default_kd", float(P.KD_GAIN[1]))

        self.rate_hz = self.get_parameter("rate_hz").as_int()
        self.default_coxa_kp = float(self.get_parameter("default_coxa_kp").value)
        self.default_coxa_kd = float(self.get_parameter("default_coxa_kd").value)
        self.default_kp = float(self.get_parameter("default_kp").value)
        self.default_kd = float(self.get_parameter("default_kd").value)

        self.state = RobotHardwareState()
        self.cmd = RobotHardwareCommand()
        self.motors = [None] * P.N_JOINTS
        self.threads = []
        self.running = True

        # Subscribers
        self.create_subscription(JointState, "/joint_commands", self._cmd_cb, 10)

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.diag_pub = self.create_publisher(Float32MultiArray, "/jaguar/motor_diagnostics", 10)
        self.status_pub = self.create_publisher(String, "/jaguar/hardware_status", 10)

        # Setup CAN Motors and start communication threads
        self._setup_can()

        # 200 Hz Publishing loop
        self.timer = self.create_timer(1.0 / P.CAN_HZ, self._publish_joint_states)
        self.diag_timer = self.create_timer(1.0, self._publish_diagnostics)

        atexit.register(self.disable_all_motors)
        self.get_logger().info("CAN Hardware Driver Ready. Publishing /joint_states @ 200 Hz.")

    def _cmd_cb(self, msg: JointState):
        with self.cmd.lock:
            has_custom_gains = (len(msg.effort) >= 24)
            if msg.name:
                for idx, name in enumerate(msg.name):
                    if name in NAME_TO_ROS_INDEX:
                        ros_idx = NAME_TO_ROS_INDEX[name]
                        if len(msg.position) > idx:
                            self.cmd.pos[ros_idx] = msg.position[idx]
                        if len(msg.velocity) > idx:
                            self.cmd.vel[ros_idx] = msg.velocity[idx]
                        if has_custom_gains:
                            self.cmd.kp[ros_idx] = msg.effort[idx]
                            self.cmd.kd[ros_idx] = msg.effort[12 + idx]
                            self.cmd.tau[ros_idx] = msg.effort[24 + idx] if len(msg.effort) >= 36 else 0.0
                        else:
                            is_coxa = (ros_idx % 3 == 0)
                            self.cmd.kp[ros_idx] = self.default_coxa_kp if is_coxa else self.default_kp
                            self.cmd.kd[ros_idx] = self.default_coxa_kd if is_coxa else self.default_kd
                            if len(msg.effort) > idx:
                                self.cmd.tau[ros_idx] = msg.effort[idx]
            elif len(msg.position) == 12:
                # Direct ROS order
                self.cmd.pos = list(msg.position)
                if len(msg.velocity) == 12:
                    self.cmd.vel = list(msg.velocity)
                if has_custom_gains:
                    for i in range(12):
                        self.cmd.kp[i] = msg.effort[i]
                        self.cmd.kd[i] = msg.effort[12 + i]
                        self.cmd.tau[i] = msg.effort[24 + i] if len(msg.effort) >= 36 else 0.0
                else:
                    for i in range(12):
                        is_coxa = (i % 3 == 0)
                        self.cmd.kp[i] = self.default_coxa_kp if is_coxa else self.default_kp
                        self.cmd.kd[i] = self.default_coxa_kd if is_coxa else self.default_kd
                        if len(msg.effort) > i:
                            self.cmd.tau[i] = msg.effort[i]
            self.cmd.enabled = True

    def _setup_can(self):
        bus_list = sorted(list(set(P.DEVICE)))  # ["can0", "can1"]

        for bus_name in bus_list:
            indices = [i for i, dev in enumerate(P.DEVICE) if dev == bus_name]
            t = threading.Thread(target=self._can_bus_worker, args=(bus_name, indices), daemon=True)
            t.start()
            self.threads.append(t)

    def _can_bus_worker(self, bus_name, indices):
        self.get_logger().info(f"[{bus_name}] Initializing {len(indices)} motors...")

        # 1. Instantiate Motor Controllers
        for i in indices:
            self.motors[i] = RobStrideMotorController(
                bus=P.DEVICE[i],
                motor_id=P.CAN_ID[i],
                motor_type=P.MOTOR_TYPE[i],
                motor_dir=P.MOTOR_DIR[i]
            )

        # 2. Enable Motors & set CONTROL_MODE
        self.get_logger().info(f"[{bus_name}] Enabling motors...")
        for i in indices:
            motor = self.motors[i]
            can_id, pos, vel, tau, tem = motor.enable_motor()
            self.get_logger().info(f"[{bus_name}] Motor #{P.CAN_ID[i]} ({P.JOINT_NAME[i]}) | Pos: {pos:.3f}, Temp: {tem:.1f}C")
            time.sleep(0.05)
            motor.set_run_mode("CONTROL_MODE")
            with self.state.lock:
                self.state.pos[i] = pos
                self.state.vel[i] = vel
                self.state.tau[i] = tau
                self.state.temp[i] = tem

        # 3. Apply Offset Angles
        self.get_logger().info(f"[{bus_name}] Applying angular calibration offsets...")
        for i in indices:
            motor = self.motors[i]
            offset = 0.0
            if P.MOTOR_OFFSET_ANGLE[i]:
                offset = P.MOTOR_OFFSET_ANGLE[i]
            motor.set_angle_offset(offset)

        # 4. Safe Passive Standby (Kp=0, Kd=0 - Zero Torque Sensing Mode)
        self.get_logger().info(f"[{bus_name}] Setting motors to PASSIVE ZERO-TORQUE mode (Kp=0, Kd=0)...")
        for i in indices:
            motor = self.motors[i]
            can_id, pos, vel, tau, tem = motor.send_control_command(
                p_ref=0.0, v_ref=0.0, kp=0.0, kd=0.0, tau_ff=0.0
            )
            with self.state.lock:
                if pos is not None: self.state.pos[i] = pos
                if vel is not None: self.state.vel[i] = vel
                if tau is not None: self.state.tau[i] = tau
                if tem is not None: self.state.temp[i] = tem

        self.get_logger().info(f"[{bus_name}] Motor initialization complete. Running 200 Hz loop in SAFE PASSIVE mode.")
        with self.state.lock:
            self.state.active = True

        # 5. Active 200 Hz Control Loop
        rate_dt = 1.0 / P.CAN_HZ
        while self.running:
            t0 = time.time()

            with self.cmd.lock:
                p_ref = self.cmd.pos.copy()
                v_ref = self.cmd.vel.copy()
                t_ref = self.cmd.tau.copy()
                kp_ref = self.cmd.kp.copy()
                kd_ref = self.cmd.kd.copy()
                enabled = self.cmd.enabled

            for i in indices:
                motor = self.motors[i]
                try:
                    if enabled:
                        can_id, pos, vel, tau, tem = motor.send_control_command(
                            p_ref=p_ref[i], v_ref=v_ref[i], kp=kp_ref[i], kd=kd_ref[i], tau_ff=t_ref[i]
                        )
                    else:
                        # SAFE PASSIVE: Zero torque, pure sensing when not commanded
                        can_id, pos, vel, tau, tem = motor.send_control_command(
                            p_ref=0.0, v_ref=0.0, kp=0.0, kd=0.0, tau_ff=0.0
                        )

                    if pos is not None:
                        with self.state.lock:
                            self.state.pos[i] = pos
                            self.state.vel[i] = vel
                            self.state.tau[i] = tau
                            self.state.temp[i] = tem

                    if tem is not None and tem > 75.0:
                        self.get_logger().error(f"OVERHEAT WARNING: Motor #{P.CAN_ID[i]} ({P.JOINT_NAME[i]}) Temp={tem:.1f}C!")

                except Exception as e:
                    with self.state.lock:
                        self.state.errors[i] += 1

            elapsed = time.time() - t0
            sleep_time = rate_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _publish_joint_states(self):
        with self.state.lock:
            pos = self.state.pos.copy()
            vel = self.state.vel.copy()
            tau = self.state.tau.copy()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = P.JOINT_NAME.copy()
        msg.position = pos
        msg.velocity = vel
        msg.effort = tau
        self.joint_state_pub.publish(msg)

    def _publish_diagnostics(self):
        with self.state.lock:
            temp = self.state.temp.copy()
            errs = sum(self.state.errors)

        diag_msg = Float32MultiArray()
        diag_msg.data = temp
        self.diag_pub.publish(diag_msg)

        status_msg = String()
        max_temp = max(temp) if temp else 0.0
        status_msg.data = f"CAN: Active | Max Temp: {max_temp:.1f}C | Total Packet Errors: {errs}"
        self.status_pub.publish(status_msg)

    def disable_all_motors(self):
        self.running = False
        self.get_logger().info("Disabling all 12 RobStride RS00 motors...")
        for motor in self.motors:
            if motor is not None:
                try:
                    motor.disable_motor()
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = CanHardwareDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CAN hardware node...")
    finally:
        node.disable_all_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
