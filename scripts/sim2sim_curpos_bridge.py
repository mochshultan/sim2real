#!/usr/bin/env python3
# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""MuJoCo feedback/impedance bridge for the unmodified real ROS policy loop."""

import json
import time
from pathlib import Path

import mujoco
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from joint_layout import ISAAC_JOINT_NAMES
from pion_mujoco_plant import PionMujocoPlant
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, Joy
from std_msgs.msg import String


class PionMujocoBridge(Node):
    def __init__(self):
        super().__init__("pion_mujoco_bridge")
        for name, value in dict(
            model_path="",
            config_file="",
            viz="viser",
            port=8080,
            duration=0.0,
            auto_walk=True,
            command_x=0.0,
            command_y=0.0,
            command_yaw=0.0,
            report="",
        ).items():
            self.declare_parameter(name, value)
        config = yaml.safe_load(Path(self.get_parameter("config_file").value).read_text())
        self.imu_upside_down = bool(config["nxp_jaguar_controller"]["ros__parameters"]["imu_upside_down"])
        self.plant = PionMujocoPlant(self.get_parameter("model_path").value)
        self.joints = self.create_publisher(JointState, "/joint_states", 10)
        self.imu = self.create_publisher(Imu, "/imu/data", qos_profile_sensor_data)
        self.joy = self.create_publisher(Joy, "/joy", 10)
        self.command = self.create_publisher(Twist, "/jaguar/benchmark_cmd_vel", 10)
        self.create_subscription(JointState, "/joint_commands", self.on_command, 10)
        self.create_subscription(String, "/jaguar/status", self.on_status, 10)
        self.start = time.monotonic()
        self.last_command = float("-inf")
        self.controller_state = "WAITING"
        self.states_seen = set()
        self.active = False
        self.auto_walk = self.get_parameter("auto_walk").value
        self.sent_stand = False
        self.sent_walk = False
        self.release_button = False
        self.received_commands = 0
        self.walk_commands = 0
        self.minimum_height = float(self.plant.data.qpos[2])
        self.maximum_torque = 0.0
        self.finished = False
        self.command_values = np.array(
            [self.get_parameter(name).value for name in ("command_x", "command_y", "command_yaw")]
        )
        self.viewer = None
        self.meshes = []
        self.sliders = []
        self.last_render_time = float("-inf")
        if self.get_parameter("viz").value == "viser":
            self.setup_viewer()
        self.get_logger().info(
            "MuJoCo bridge has no policy; nxp_jaguar_controller supplies all targets. No CAN opened."
        )
        self.timer = self.create_timer(float(self.plant.model.opt.timestep), self.tick)

    def setup_viewer(self):
        import viser

        port = self.get_parameter("port").value
        self.viewer = viser.ViserServer(host="0.0.0.0", port=port, label="Pion Current / real ROS controller")
        if self.viewer.get_port() != port:
            self.viewer.stop()
            raise RuntimeError(f"Viser port{port} is occupied")
        self.viewer.scene.set_up_direction("+z")
        self.viewer.scene.add_grid("/ground", plane="xy", width=30, height=30, cell_size=0.5)
        self.follow = self.viewer.gui.add_checkbox("Follow robot", initial_value=True)
        self.sliders = [
            self.viewer.gui.add_slider(name, min=-1.0, max=1.0, step=0.05, initial_value=float(value))
            for name, value in zip(("Forward m/s", "Lateral m/s", "Yaw rad/s"), self.command_values)
        ]
        self.state_text = self.viewer.gui.add_text("Controller", initial_value="WAITING", disabled=True)
        stand = self.viewer.gui.add_button("Stand (A)")
        walk = self.viewer.gui.add_button("Policy (B)")
        stop = self.viewer.gui.add_button("Zero velocity")
        stand.on_click(lambda _: self.press_button(0))
        walk.on_click(lambda _: self.press_button(1))

        @stop.on_click
        def zero_velocity(_):
            for slider in self.sliders:
                slider.value = 0.0

        model = self.plant.model
        for gid in range(model.ngeom):
            if model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH or model.geom_rgba[gid, 3] <= 0:
                continue
            mid = model.geom_dataid[gid]
            va, fa = model.mesh_vertadr[mid], model.mesh_faceadr[mid]
            handle = self.viewer.scene.add_mesh_simple(
                f"/pion/geom_{gid}",
                model.mesh_vert[va : va + model.mesh_vertnum[mid]].copy(),
                model.mesh_face[fa : fa + model.mesh_facenum[mid]].copy(),
                color=tuple(int(255 * value) for value in model.geom_rgba[gid, :3]),
            )
            self.meshes.append((gid, handle))

        @self.viewer.on_client_connect
        def connected(client):
            client.camera.position = (1.0, -1.2, 0.7)
            client.camera.look_at = (0.0, 0.0, 0.2)
            client.camera.up_direction = (0, 0, 1)

    def press_button(self, index):
        message = Joy()
        message.buttons = [int(i == index) for i in range(11)]
        self.joy.publish(message)
        self.release_button = True

    def on_status(self, message):
        if message.data.startswith("State: "):
            self.controller_state = message.data.split(" |", 1)[0].split(": ", 1)[1]
            self.states_seen.add(self.controller_state)

    def on_command(self, message):
        try:
            if len(message.name) != 12 or set(message.name) != set(ISAAC_JOINT_NAMES):
                raise ValueError("Expected12 unique controller joint names")
            if len(message.position) != 12 or len(message.effort) not in (24, 36):
                raise ValueError("Expected position12 and gains24 or gains/feedforward36")
            if len(message.velocity) not in (0, 12):
                raise ValueError("Expected velocity0 or12")
            order = [message.name.index(name) for name in ISAAC_JOINT_NAMES]
            gains = np.asarray(message.effort).reshape(-1, 12)
            velocity = np.asarray(message.velocity) if message.velocity else np.zeros(12)
            self.plant.set_command(
                np.asarray(message.position)[order],
                velocity[order],
                gains[0, order],
                gains[1, order],
                gains[2, order] if len(gains) == 3 else None,
            )
        except ValueError as error:
            self.get_logger().error(str(error))
            self.plant.kp[:] = 0
            self.plant.kd[:] = 0
            self.plant.feedforward[:] = 0
            return
        self.last_command = time.monotonic()
        self.active |= bool(self.plant.kp.any() or self.plant.kd.any())
        self.received_commands += 1
        self.walk_commands += int(self.controller_state == "WALK")

    def publish_feedback(self):
        position, velocity, torque = self.plant.feedback()
        stamp = self.get_clock().now().to_msg()
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ISAAC_JOINT_NAMES
        joints.position, joints.velocity, joints.effort = position.tolist(), velocity.tolist(), torque.tolist()
        self.joints.publish(joints)
        data, model = self.plant.data, self.plant.model
        q = data.qpos[3:7].copy()  # Body orientation wxyz.
        sensor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "body_gyro")
        address = model.sensor_adr[sensor]
        angular = data.sensordata[address : address + 3].copy()
        if self.imu_upside_down:
            # Inverse of the real controller's mounting correction, not a second body transform.
            w, x, y, z = q
            q = np.array([x, -w, -z, y])
            angular *= [1, -1, -1]
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = q.tolist()
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = angular.tolist()
        imu.orientation_covariance[0] = 0.001
        self.imu.publish(imu)

    def tick(self):
        elapsed = time.monotonic() - self.start
        if self.release_button:
            self.joy.publish(Joy(buttons=[0] * 11))
            self.release_button = False
        if self.active:
            self.plant.step(passive=time.monotonic() - self.last_command > 0.1)
            self.minimum_height = min(self.minimum_height, float(self.plant.data.qpos[2]))
            self.maximum_torque = max(self.maximum_torque, float(np.abs(self.plant.torque).max()))
        self.publish_feedback()
        if self.auto_walk and elapsed > 1 and self.controller_state == "STANDBY" and not self.sent_stand:
            self.press_button(0)
            self.sent_stand = True
        if self.auto_walk and self.controller_state == "STAND_HOLD" and not self.sent_walk:
            self.press_button(1)
            self.sent_walk = True
        if self.sliders:
            self.command_values[:] = [slider.value for slider in self.sliders]
        command = Twist()
        command.linear.x, command.linear.y = self.command_values[:2].tolist()
        command.angular.z = float(self.command_values[2])
        self.command.publish(command)
        if self.viewer is not None and elapsed - self.last_render_time >= 1 / 30:
            self.last_render_time = elapsed
            with self.viewer.atomic():
                for gid, handle in self.meshes:
                    quaternion = np.empty(4)
                    mujoco.mju_mat2Quat(quaternion, self.plant.data.geom_xmat[gid])
                    handle.position = self.plant.data.geom_xpos[gid].copy()
                    handle.wxyz = quaternion
                self.state_text.value = self.controller_state
                if self.follow.value:
                    for client in self.viewer.get_clients().values():
                        target = self.plant.data.qpos[:3].copy()
                        displacement = target - client.camera.look_at
                        client.camera.position = client.camera.position + displacement
                        client.camera.look_at = target
        duration = self.get_parameter("duration").value
        self.finished = bool(duration > 0 and elapsed >= duration)

    def close(self):
        if self.viewer is not None:
            self.viewer.stop()
        report = self.get_parameter("report").value
        if report:
            path = Path(report).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                json.dumps(
                    dict(
                        controller="nxp_jaguar_controller.py",
                        can_opened=False,
                        policy_in_bridge=False,
                        controller_states_seen=sorted(self.states_seen),
                        joint_commands_received=self.received_commands,
                        walk_commands_received=self.walk_commands,
                        minimum_base_height=self.minimum_height,
                        final_base_position=self.plant.data.qpos[:3].tolist(),
                        maximum_torque_nm=self.maximum_torque,
                        simulation_seconds=self.plant.data.time,
                    ),
                    indent=2,
                )
                + "\n"
            )


def main():
    rclpy.init()
    node = PionMujocoBridge()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.02)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
