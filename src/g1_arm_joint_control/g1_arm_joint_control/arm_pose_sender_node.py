import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from unitree_hg.msg import LowState

from .joint_map import ARM_DOF, ARM_JOINT_INDICES, ARM_JOINT_NAMES


class ArmPoseSenderNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_arm_pose_sender')

        self.declare_parameter('cmd_topic', '/g1/arm_joint_position_cmd')
        self.declare_parameter('lowstate_topic', '/lowstate')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('wait_for_initial_pose_sec', 2.0)
        self.declare_parameter('duration_sec', 0.0)
        self.declare_parameter('repetitive_motion', True)
        self.declare_parameter('repetitive_frequency_hz', 0.20)

        # Soft velocity mode parameters.
        self.declare_parameter('max_joint_velocity', 0.8)        # rad/s
        self.declare_parameter('velocity_gain', 2.0)             # 1/s
        self.declare_parameter('velocity_smoothing_alpha', 0.2)  # [0,1]

        # Lift target as offsets from current arm pose.
        self.declare_parameter('lift_side', 'left')
        self.declare_parameter('lift_shoulder_pitch_delta', 1.10)
        self.declare_parameter('lift_shoulder_roll_delta', 0.30)
        self.declare_parameter('lift_shoulder_yaw_delta', 0.05)
        self.declare_parameter('lift_elbow_delta', 1.15)

        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.lowstate_topic = self.get_parameter('lowstate_topic').get_parameter_value().string_value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.wait_for_initial_pose_sec = float(self.get_parameter('wait_for_initial_pose_sec').value)
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        self.repetitive_motion = bool(self.get_parameter('repetitive_motion').value)
        self.repetitive_frequency_hz = float(self.get_parameter('repetitive_frequency_hz').value)

        self.max_joint_velocity = float(self.get_parameter('max_joint_velocity').value)
        self.velocity_gain = float(self.get_parameter('velocity_gain').value)
        self.velocity_smoothing_alpha = float(self.get_parameter('velocity_smoothing_alpha').value)

        self.lift_side = str(self.get_parameter('lift_side').value).strip().lower()
        self.lift_shoulder_pitch_delta = float(self.get_parameter('lift_shoulder_pitch_delta').value)
        self.lift_shoulder_roll_delta = float(self.get_parameter('lift_shoulder_roll_delta').value)
        self.lift_shoulder_yaw_delta = float(self.get_parameter('lift_shoulder_yaw_delta').value)
        self.lift_elbow_delta = float(self.get_parameter('lift_elbow_delta').value)

        if self.lift_side not in ('left', 'right'):
            raise ValueError("lift_side must be 'left' or 'right'.")
        if not (0.0 <= self.velocity_smoothing_alpha <= 1.0):
            raise ValueError('velocity_smoothing_alpha must be in [0, 1].')

        self.pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        self.state_sub = self.create_subscription(LowState, self.lowstate_topic, self._state_cb, 10)

        self.initialized = False
        self.wait_logged = False
        self.fallback_logged = False
        self.done = False

        self.start_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.motion_start_time_sec = self.start_time_sec

        self.current_cmd = np.zeros(ARM_DOF, dtype=np.float64)
        self.base_cmd = np.zeros(ARM_DOF, dtype=np.float64)
        self.delta_cmd = np.zeros(ARM_DOF, dtype=np.float64)
        self.target_cmd = np.zeros(ARM_DOF, dtype=np.float64)
        self.filtered_velocity = np.zeros(ARM_DOF, dtype=np.float64)

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.dt = period
        self.timer = self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            f'Soft velocity lift sender started. cmd_topic={self.cmd_topic}, lowstate_topic={self.lowstate_topic}'
        )
        self.get_logger().info(f'Arm command order: {ARM_JOINT_NAMES}')
        if self.repetitive_motion:
            self.get_logger().info(
                f'Repetitive mode enabled: target swings between +delta and -delta at '
                f'{self.repetitive_frequency_hz:.2f} Hz.'
            )

    def _state_cb(self, msg: LowState) -> None:
        if self.initialized:
            return
        if len(msg.motor_state) <= max(ARM_JOINT_INDICES):
            return

        arm_q = np.asarray([msg.motor_state[i].q for i in ARM_JOINT_INDICES], dtype=np.float64)
        self.current_cmd = arm_q.copy()
        self.base_cmd = arm_q.copy()
        self.target_cmd = arm_q.copy()

        side_sign = 1.0 if self.lift_side == 'left' else -1.0
        base = 0 if self.lift_side == 'left' else 7
        self.delta_cmd.fill(0.0)
        self.delta_cmd[base + 0] = self.lift_shoulder_pitch_delta
        self.delta_cmd[base + 1] = side_sign * self.lift_shoulder_roll_delta
        self.delta_cmd[base + 2] = side_sign * self.lift_shoulder_yaw_delta
        self.delta_cmd[base + 3] = self.lift_elbow_delta
        if not self.repetitive_motion:
            self.target_cmd = self.base_cmd + self.delta_cmd

        self.initialized = True
        self.motion_start_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info('Captured initial arm pose from /lowstate and built lift target.')

    def _timer_cb(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9

        if not self.initialized:
            waited = now_sec - self.start_time_sec
            if not self.wait_logged:
                self.get_logger().info('Waiting for initial arm pose from /lowstate...')
                self.wait_logged = True
            if waited < self.wait_for_initial_pose_sec:
                return

            # Fallback if no lowstate received.
            self.initialized = True
            self.current_cmd = np.zeros(ARM_DOF, dtype=np.float64)
            self.base_cmd = np.zeros(ARM_DOF, dtype=np.float64)
            self.target_cmd = np.zeros(ARM_DOF, dtype=np.float64)
            side_sign = 1.0 if self.lift_side == 'left' else -1.0
            base = 0 if self.lift_side == 'left' else 7
            self.delta_cmd.fill(0.0)
            self.delta_cmd[base + 0] = self.lift_shoulder_pitch_delta
            self.delta_cmd[base + 1] = side_sign * self.lift_shoulder_roll_delta
            self.delta_cmd[base + 2] = side_sign * self.lift_shoulder_yaw_delta
            self.delta_cmd[base + 3] = self.lift_elbow_delta
            if not self.repetitive_motion:
                self.target_cmd = self.base_cmd + self.delta_cmd
            self.motion_start_time_sec = now_sec
            if not self.fallback_logged:
                self.get_logger().warn('No /lowstate received. Fallback to zero initial pose.')
                self.fallback_logged = True

        t_sec = now_sec - self.motion_start_time_sec
        if self.repetitive_motion:
            phase = 2.0 * np.pi * self.repetitive_frequency_hz * t_sec
            swing = np.sin(phase)  # [-1, 1] gives opposite-angle motion
            self.target_cmd = self.base_cmd + swing * self.delta_cmd

        error = self.target_cmd - self.current_cmd
        desired_velocity = np.clip(self.velocity_gain * error, -self.max_joint_velocity, self.max_joint_velocity)
        alpha = self.velocity_smoothing_alpha
        self.filtered_velocity = (1.0 - alpha) * self.filtered_velocity + alpha * desired_velocity
        self.current_cmd = self.current_cmd + self.filtered_velocity * self.dt

        msg = Float64MultiArray()
        msg.data = self.current_cmd.tolist()
        self.pub.publish(msg)

        if self.duration_sec > 0.0:
            if t_sec >= self.duration_sec:
                self.done = True
                self.timer.cancel()
                self.get_logger().info(f'Done after {self.duration_sec:.2f} sec.')


def main() -> None:
    rclpy.init()
    node = ArmPoseSenderNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
