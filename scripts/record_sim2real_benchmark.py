#!/usr/bin/env python3
"""
NXP Jaguar Sim2Real Automated Benchmark & 45D Observation Recorder.
Executes standard evaluation trajectory:
  1. Stand hold (3.0s)
  2. Transition to WALK, cmd=0 (5.0s)
  3. Forward cmd_vel ramp 0.0 to 0.5 m/s (2.0s)
  4. Forward cmd_vel ramp 0.5 to 0.0 m/s (2.0s)
  5. Return to Stand hold (3.0s)
Saves all data to compressed .npz archive and optional rosbag2.
Supports starting from sitting (auto-standup prep) or already standing!
"""

import argparse
import math
import os
import subprocess
import sys
import time
from datetime import datetime
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, JointState, Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

ROS_TO_ISAAC = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]
ROS_NAME_TO_ISAAC_IDX = {
    'FR_collar_joint': 0, 'Fr_roll_joint': 0,
    'FL_collar_joint': 1, 'Fl_roll_joint': 1,
    'BR_collar_joint': 2, 'Br_roll_joint': 2,
    'BL_collar_joint': 3, 'Bl_roll_joint': 3,
    'FR_hip_joint': 4,    'Fr_hip_pitch_joint': 4,
    'FL_hip_joint': 5,    'Fl_hip_pitch_joint': 5,
    'BR_hip_joint': 6,    'Br_hip_pitch_joint': 6,
    'BL_hip_joint': 7,    'Bl_hip_pitch_joint': 7,
    'FR_knee_joint': 8,   'Fr_knee_joint': 8,
    'FL_knee_joint': 9,   'Fl_knee_joint': 9,
    'BR_knee_joint': 10,  'Br_knee_joint': 10,
    'BL_knee_joint': 11,  'Bl_knee_joint': 11,
}

DEFAULT_JOINT_POS = np.array([
    0.0,   0.0,   0.0,   0.0,
   -1.61, -1.61, -1.55, -1.55,
    1.40,  1.40,  1.40,  1.40,
], dtype=np.float32)

class Sim2RealBenchmarkRecorder(Node):
    def __init__(self, out_dir: str, record_bag: bool = False):
        super().__init__("jaguar_benchmark_recorder")
        self.out_dir = out_dir
        os.makedirs(self.out_dir, exist_ok=True)
        self.max_vx = 0.5
        self.record_bag = record_bag
        self.bag_proc = None

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriptions
        self.create_subscription(Float32MultiArray, "/jaguar/state_debug", self._debug_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(Imu, "/Imu_data", self._imu_cb, sensor_qos)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, sensor_qos)
        self.create_subscription(String, "/jaguar/status", self._status_cb, 10)

        # Current live state
        self.latest_obs_45d = np.zeros(45, dtype=np.float32)
        self.latest_joint_pos = np.zeros(12, dtype=np.float32)
        self.latest_joint_vel = np.zeros(12, dtype=np.float32)
        self.latest_joint_tau = np.zeros(12, dtype=np.float32)
        self.latest_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.latest_ang_vel = np.zeros(3, dtype=np.float32)
        self.controller_state = "UNKNOWN"
        self.obs_received = False
        self.joints_received = False
        self.imu_received = False

        # Protocol timings
        self.t_stand_hold = 3.0
        self.t_walk_idle = 5.0
        self.t_ramp_up = 2.0
        self.t_ramp_down = 2.0
        self.t_return_stand = 3.0
        self.total_duration = (
            self.t_stand_hold + self.t_walk_idle +
            self.t_ramp_up + self.t_ramp_down + self.t_return_stand
        )

        # Data logs
        self.log_time = []
        self.log_phase = []
        self.log_cmd_vel = []
        self.log_obs_45d = []
        self.log_joint_pos = []
        self.log_joint_vel = []
        self.log_joint_tau = []
        self.log_ang_vel = []
        self.log_quat = []

        self.bench_start_time = None
        self.prep_start_time = None
        self.current_phase = "WAIT_READY"
        self.timer = self.create_timer(0.02, self._control_and_record_loop)  # 50 Hz loop

        self.get_logger().info(
            f"[RECORDER] Benchmark Node Started! Protocol: 3s Stand -> 5s Walk(0) -> 2s Accel(0->{self.max_vx}) -> 2s Decel({self.max_vx}->0) -> 3s Stand"
        )

    def _status_cb(self, msg: String):
        # Parses "State: <STATE> | ..."
        if "State: " in msg.data:
            state_part = msg.data.split("State: ")[1].split(" |")[0].strip()
            self.controller_state = state_part

    def _debug_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 45:
            self.latest_obs_45d = np.array(msg.data[:45], dtype=np.float32)
            self.obs_received = True

    def _joint_cb(self, msg: JointState):
        matched = False
        if msg.name:
            for idx, name in enumerate(msg.name):
                if name in ROS_NAME_TO_ISAAC_IDX:
                    isaac_idx = ROS_NAME_TO_ISAAC_IDX[name]
                    if len(msg.position) > idx:
                        self.latest_joint_pos[isaac_idx] = msg.position[idx]
                    if len(msg.velocity) > idx:
                        self.latest_joint_vel[isaac_idx] = msg.velocity[idx]
                    if len(msg.effort) > idx:
                        self.latest_joint_tau[isaac_idx] = msg.effort[idx]
                    matched = True
        if not matched and len(msg.position) == 12:
            for i in range(12):
                ros_idx = ROS_TO_ISAAC[i]
                self.latest_joint_pos[i] = msg.position[ros_idx]
                if len(msg.velocity) > ros_idx:
                    self.latest_joint_vel[i] = msg.velocity[ros_idx]
                if len(msg.effort) > ros_idx:
                    self.latest_joint_tau[i] = msg.effort[ros_idx]
        self.latest_joint_pos = (self.latest_joint_pos + np.pi) % (2.0 * np.pi) - np.pi
        self.joints_received = True

    def _imu_cb(self, msg: Imu):
        self.latest_quat = np.array([
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ], dtype=np.float32)
        self.latest_ang_vel = np.array([
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ], dtype=np.float32)
        self.imu_received = True

    def _send_joy_button(self, button_idx: int):
        msg = Joy()
        msg.buttons = [0] * 12
        msg.buttons[button_idx] = 1
        msg.axes = [0.0] * 8
        self.joy_pub.publish(msg)

    def _send_cmd_vel(self, vx: float, vy: float = 0.0, wz: float = 0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def _is_robot_standing(self) -> bool:
        # Check either via controller state or joint angles
        if self.controller_state == "STAND_HOLD":
            return True
        # If hips are near -1.6 rad and knees near 1.4 rad
        avg_hip = float(np.mean(self.latest_joint_pos[4:8]))
        avg_knee = float(np.mean(self.latest_joint_pos[8:12]))
        return (avg_hip < -1.0) and (avg_knee > 0.8)

    def _control_and_record_loop(self):
        now = time.monotonic()

        # Step 0: Check Sensors Readiness
        if self.current_phase == "WAIT_READY":
            if not self.joints_received or not self.imu_received:
                self.get_logger().info("[RECORDER] Waiting for /joint_states and /Imu_data streams...", throttle_duration_sec=2.0)
                return

            if self._is_robot_standing():
                self.get_logger().info("[RECORDER] Robot is ALREADY STANDING! Starting 15s Benchmark Protocol immediately...")
                self._start_benchmark(now)
            else:
                self.get_logger().info("[RECORDER] Robot is SITTING. Sending STANDUP command and waiting for robot to stand...")
                self.prep_start_time = now
                self.current_phase = "STANDUP_PREP"
                self._send_joy_button(0)
            return

        # Step 0B: Standup Preparation if started from sitting
        if self.current_phase == "STANDUP_PREP":
            elapsed_prep = now - self.prep_start_time
            # Keep commanding Button 0 briefly
            if elapsed_prep < 1.0 and int(elapsed_prep / 0.3) % 2 == 0:
                self._send_joy_button(0)

            # Wait for standup duration (approx 2.5 - 3.0s) and verify standing pose
            if elapsed_prep >= 3.0 and self._is_robot_standing():
                self.get_logger().info("✅ Robot has successfully stood up! Starting 15s Benchmark Protocol...")
                self._start_benchmark(now)
            else:
                sys.stdout.write(f"\r⏳ [STANDUP PREP: {elapsed_prep:4.1f}s / 3.0s] Waiting for robot to stand up firmly...")
                sys.stdout.flush()
            return

        elapsed = now - self.bench_start_time
        cmd_vx = 0.0

        # Phase timeline:
        # 1. 0.0 -> 3.0s: STAND_HOLD (cmd_vel = 0)
        if elapsed < self.t_stand_hold:
            self.current_phase = "STAND_HOLD"
            cmd_vx = 0.0
            if int(elapsed / 0.5) % 2 == 0:
                self._send_joy_button(0)

        # 2. 3.0 -> 8.0s: WALK mode idle (5.0s, cmd_vel = 0)
        elif elapsed < (self.t_stand_hold + self.t_walk_idle):
            if self.current_phase != "WALK_IDLE":
                self.get_logger().info(f"\n[{elapsed:5.2f}s] -> Phase: WALK_IDLE (cmd_vel = 0)")
                self._send_joy_button(1)
            self.current_phase = "WALK_IDLE"
            cmd_vx = 0.0

        # 3. 8.0 -> 10.0s: WALK mode accel ramp (0.0 to max_vx)
        elif elapsed < (self.t_stand_hold + self.t_walk_idle + self.t_ramp_up):
            phase_t = elapsed - (self.t_stand_hold + self.t_walk_idle)
            self.current_phase = "WALK_ACCEL"
            cmd_vx = self.max_vx * (phase_t / self.t_ramp_up)

        # 4. 10.0 -> 12.0s: WALK mode decel ramp (max_vx to 0.0)
        elif elapsed < (self.t_stand_hold + self.t_walk_idle + self.t_ramp_up + self.t_ramp_down):
            phase_t = elapsed - (self.t_stand_hold + self.t_walk_idle + self.t_ramp_up)
            self.current_phase = "WALK_DECEL"
            cmd_vx = self.max_vx * (1.0 - phase_t / self.t_ramp_down)

        # 5. 12.0 -> 15.0s: Return to STAND_HOLD
        elif elapsed <= self.total_duration:
            if self.current_phase != "RETURN_STAND":
                self.get_logger().info(f"\n[{elapsed:5.2f}s] -> Phase: RETURN_STAND (Switch to Stand Pose)")
                self._send_joy_button(0)
            self.current_phase = "RETURN_STAND"
            cmd_vx = 0.0

        # Completed: Save and Stop
        else:
            self._finish_and_save()
            return

        # Publish command velocity
        self._send_cmd_vel(cmd_vx)

        # Record timestep
        self.log_time.append(elapsed)
        self.log_phase.append(self.current_phase)
        self.log_cmd_vel.append([cmd_vx, 0.0, 0.0])
        self.log_obs_45d.append(self.latest_obs_45d.copy())
        self.log_joint_pos.append(self.latest_joint_pos.copy())
        self.log_joint_vel.append(self.latest_joint_vel.copy())
        self.log_joint_tau.append(self.latest_joint_tau.copy())
        self.log_ang_vel.append(self.latest_ang_vel.copy())
        self.log_quat.append(self.latest_quat.copy())

        # Progress display
        pct = min(100.0, 100.0 * elapsed / self.total_duration)
        sys.stdout.write(
            f"\r⏱️ [{elapsed:5.2f}s / {self.total_duration:5.2f}s] ({pct:4.1f}%) | "
            f"Phase: {self.current_phase:<12} | cmd_vx: {cmd_vx:+5.2f} m/s | "
            f"Samples: {len(self.log_time)}"
        )
        sys.stdout.flush()

    def _start_benchmark(self, now):
        self.bench_start_time = now
        self.current_phase = "STAND_HOLD"
        self._send_joy_button(0)
        if self.record_bag:
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_path = os.path.join(self.out_dir, f"bag_sim2real_{timestamp_str}")
            self.bag_proc = subprocess.Popen([
                "ros2", "bag", "record", "-o", bag_path,
                "/jaguar/state_debug", "/joint_states", "/Imu_data", "/cmd_vel", "/joy", "/jaguar/status"
            ])

    def _finish_and_save(self):
        print("\n\n✅ Benchmark Protocol Completed! Stopping robot and saving data...")
        self._send_cmd_vel(0.0)
        self._send_joy_button(0)

        if self.bag_proc is not None:
            self.bag_proc.terminate()
            self.bag_proc.wait()

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(self.out_dir, f"benchmark_sim2real_{timestamp_str}.npz")

        data_dict = {
            "source": "sim2real",
            "time": np.array(self.log_time, dtype=np.float32),
            "phase": np.array(self.log_phase),
            "cmd_vel": np.array(self.log_cmd_vel, dtype=np.float32),
            "obs_45d": np.array(self.log_obs_45d, dtype=np.float32),
            "joint_pos": np.array(self.log_joint_pos, dtype=np.float32),
            "joint_vel": np.array(self.log_joint_vel, dtype=np.float32),
            "joint_torque": np.array(self.log_joint_tau, dtype=np.float32),
            "base_ang_vel": np.array(self.log_ang_vel, dtype=np.float32),
            "body_quat": np.array(self.log_quat, dtype=np.float32),
            "total_duration": self.total_duration,
        }

        np.savez_compressed(filepath, **data_dict)
        print(f"📦 Successfully saved benchmark archive to: {filepath}")
        print(f"   Total timesteps: {len(self.log_time)} | Shape obs_45d: {data_dict['obs_45d'].shape}")

        rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description="Record 15s Benchmark Trajectory in Sim2Real")
    default_out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "benchmark_data")
    parser.add_argument("--out-dir", type=str, default=default_out_dir, help="Output directory")
    parser.add_argument("--bag", action="store_true", help="Also record ROS 2 bag")
    args = parser.parse_args()

    rclpy.init()
    node = Sim2RealBenchmarkRecorder(out_dir=args.out_dir, record_bag=args.bag)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nBenchmark cancelled by user.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
