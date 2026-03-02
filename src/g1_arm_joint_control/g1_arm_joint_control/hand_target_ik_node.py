from __future__ import annotations

import numpy as np
import mujoco
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from unitree_hg.msg import LowState

from .joint_map import ARM_JOINT_INDICES


class HandTargetIKNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_hand_target_ik')

        self.declare_parameter('target_topic', '/g1/hand_target_point')
        self.declare_parameter('cmd_topic', '/g1/arm_joint_position_cmd')
        self.declare_parameter('lowstate_topic', '/lowstate')
        self.declare_parameter('publish_rate_hz', 50.0)

        self.declare_parameter(
            'scene_path',
            '/home/yutong/scaledrive.ai/fincantieri/g1_control/g1_control_ws/src/'
            'g1_mujoco_nav/assets/unitree_robots/g1/scene_29dof.xml',
        )
        self.declare_parameter('controlled_motor_count', 29)

        self.declare_parameter('arm_side', 'left')
        self.declare_parameter('end_effector_body', 'left_wrist_yaw_link')
        self.declare_parameter('ee_local_offset', [0.0, 0.0, 0.0])

        self.declare_parameter('ik_iterations', 24)
        self.declare_parameter('ik_step_size', 0.7)
        self.declare_parameter('ik_damping', 0.08)
        self.declare_parameter('ik_tolerance', 0.01)
        self.declare_parameter('ik_max_step_per_iter', 0.15)
        self.declare_parameter('max_delta_per_cycle', 0.05)
        self.declare_parameter('cartesian_speed_mps', 0.04)
        self.declare_parameter('goal_reach_tolerance_m', 0.01)

        self.target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.lowstate_topic = self.get_parameter('lowstate_topic').get_parameter_value().string_value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.scene_path = str(self.get_parameter('scene_path').value)
        self.controlled_motor_count = int(self.get_parameter('controlled_motor_count').value)

        self.arm_side = str(self.get_parameter('arm_side').value).strip().lower()
        self.end_effector_body = str(self.get_parameter('end_effector_body').value).strip()
        self.ee_local_offset = np.asarray(self.get_parameter('ee_local_offset').value, dtype=np.float64)

        self.ik_iterations = int(self.get_parameter('ik_iterations').value)
        self.ik_step_size = float(self.get_parameter('ik_step_size').value)
        self.ik_damping = float(self.get_parameter('ik_damping').value)
        self.ik_tolerance = float(self.get_parameter('ik_tolerance').value)
        self.ik_max_step_per_iter = float(self.get_parameter('ik_max_step_per_iter').value)
        self.max_delta_per_cycle = float(self.get_parameter('max_delta_per_cycle').value)
        self.cartesian_speed_mps = float(self.get_parameter('cartesian_speed_mps').value)
        self.goal_reach_tolerance_m = float(self.get_parameter('goal_reach_tolerance_m').value)

        if self.arm_side not in ('left', 'right'):
            raise ValueError("arm_side must be 'left' or 'right'.")
        if self.ee_local_offset.size != 3:
            raise ValueError('ee_local_offset must be [x, y, z].')

        self.model = mujoco.MjModel.from_xml_path(self.scene_path)
        self.data = mujoco.MjData(self.model)
        self.base_qpos_dofs = 7 if self.model.nq >= 7 else 0

        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.end_effector_body)
        if self.body_id < 0:
            raise ValueError(f'end_effector_body not found in scene: {self.end_effector_body}')

        self.arm_motor_indices = [15, 16, 17, 18, 19, 20, 21] if self.arm_side == 'left' else [22, 23, 24, 25, 26, 27, 28]
        self.arm14_map = {motor_idx: i for i, motor_idx in enumerate(ARM_JOINT_INDICES)}

        self.active_joint_ids = [int(self.model.actuator_trnid[i, 0]) for i in self.arm_motor_indices]
        self.active_qpos_adrs = [int(self.model.jnt_qposadr[jid]) for jid in self.active_joint_ids]
        self.active_dof_adrs = [int(self.model.jnt_dofadr[jid]) for jid in self.active_joint_ids]
        self.active_q_min = np.asarray([self.model.jnt_range[jid, 0] for jid in self.active_joint_ids], dtype=np.float64)
        self.active_q_max = np.asarray([self.model.jnt_range[jid, 1] for jid in self.active_joint_ids], dtype=np.float64)

        self.latest_q = None
        self.latest_target = None
        self.active_target = None
        self.last_cmd = None
        self.last_seed_q = None
        self.warned_frame = False

        self.target_sub = self.create_subscription(PointStamped, self.target_topic, self._target_cb, 10)
        self.state_sub = self.create_subscription(LowState, self.lowstate_topic, self._state_cb, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)

        self.period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(self.period, self._timer_cb)

        self.get_logger().info(
            f'IK node started. arm_side={self.arm_side}, target_topic={self.target_topic}, cmd_topic={self.cmd_topic}'
        )
        self.get_logger().info(
            f'EE body={self.end_effector_body}, active motors={self.arm_motor_indices}'
        )
        self.get_logger().info(
            f'Cartesian speed mode: {self.cartesian_speed_mps:.3f} m/s, '
            f'goal_tolerance={self.goal_reach_tolerance_m:.3f} m'
        )

    def _target_cb(self, msg: PointStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id not in ('world', 'map', 'base', 'base_link') and not self.warned_frame:
            self.get_logger().warn(
                f'target frame_id={msg.header.frame_id} is assumed to already match simulation world frame.'
            )
            self.warned_frame = True
        self.latest_target = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)

    def _state_cb(self, msg: LowState) -> None:
        if len(msg.motor_state) < self.controlled_motor_count:
            return
        self.latest_q = np.asarray([msg.motor_state[i].q for i in range(self.controlled_motor_count)], dtype=np.float64)

    def _set_sim_q(self, motor_q: np.ndarray) -> None:
        self.data.qpos[self.base_qpos_dofs:self.base_qpos_dofs + self.controlled_motor_count] = motor_q
        mujoco.mj_forward(self.model, self.data)

    def _ee_world_pos(self) -> np.ndarray:
        pos = self.data.xpos[self.body_id].copy()
        if np.linalg.norm(self.ee_local_offset) > 0.0:
            rot = self.data.xmat[self.body_id].reshape(3, 3)
            pos = pos + rot @ self.ee_local_offset
        return pos

    def _solve_ik(self, seed_q: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, float]:
        motor_q = seed_q.copy()
        q_active = np.asarray([motor_q[i] for i in self.arm_motor_indices], dtype=np.float64)

        jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        jacr = np.zeros((3, self.model.nv), dtype=np.float64)

        err_norm = 1e9
        for _ in range(self.ik_iterations):
            for j, motor_idx in enumerate(self.arm_motor_indices):
                motor_q[motor_idx] = q_active[j]
            self._set_sim_q(motor_q)

            ee_pos = self._ee_world_pos()
            err = target - ee_pos
            err_norm = float(np.linalg.norm(err))
            if err_norm < self.ik_tolerance:
                break

            mujoco.mj_jac(self.model, self.data, jacp, jacr, ee_pos, self.body_id)
            j_active = jacp[:, self.active_dof_adrs]

            h = j_active.T @ j_active + (self.ik_damping ** 2) * np.eye(len(self.arm_motor_indices))
            g = j_active.T @ err
            try:
                dq = self.ik_step_size * np.linalg.solve(h, g)
            except np.linalg.LinAlgError:
                dq = self.ik_step_size * np.linalg.pinv(j_active) @ err

            dq = np.clip(dq, -self.ik_max_step_per_iter, self.ik_max_step_per_iter)
            q_active = np.clip(q_active + dq, self.active_q_min, self.active_q_max)

            for j, motor_idx in enumerate(self.arm_motor_indices):
                motor_q[motor_idx] = q_active[j]

        for j, motor_idx in enumerate(self.arm_motor_indices):
            motor_q[motor_idx] = q_active[j]
        return motor_q, err_norm

    def _timer_cb(self) -> None:
        if self.latest_q is None or self.latest_target is None:
            return

        seed_q = self.latest_q.copy()
        if self.last_seed_q is not None and self.last_seed_q.size == seed_q.size:
            # Keep IK continuity while still anchored to measured state.
            for idx in self.arm_motor_indices:
                seed_q[idx] = self.last_seed_q[idx]

        self._set_sim_q(seed_q)
        ee_now = self._ee_world_pos()

        if self.active_target is None:
            self.active_target = ee_now.copy()

        to_goal = self.latest_target - self.active_target
        dist_goal = float(np.linalg.norm(to_goal))
        max_step = self.cartesian_speed_mps * self.period
        if dist_goal <= max(self.goal_reach_tolerance_m, max_step):
            self.active_target = self.latest_target.copy()
        else:
            self.active_target = self.active_target + (to_goal / dist_goal) * max_step

        solved_q, err_norm = self._solve_ik(seed_q, self.active_target)
        arm14 = np.asarray([solved_q[i] for i in ARM_JOINT_INDICES], dtype=np.float64)

        if self.last_cmd is None:
            cmd = arm14
        else:
            delta = np.clip(arm14 - self.last_cmd, -self.max_delta_per_cycle, self.max_delta_per_cycle)
            cmd = self.last_cmd + delta

        self.last_cmd = cmd.copy()
        self.last_seed_q = solved_q.copy()

        msg = Float64MultiArray()
        msg.data = cmd.tolist()
        self.cmd_pub.publish(msg)

        if err_norm > 0.08:
            self.get_logger().debug(f'IK residual={err_norm:.3f} m (target may be out of reach).')


def main() -> None:
    rclpy.init()
    node = HandTargetIKNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
