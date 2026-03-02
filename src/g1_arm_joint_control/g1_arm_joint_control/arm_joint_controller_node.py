import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from unitree_hg.msg import LowCmd, LowState

from .joint_map import ARM_DOF, ARM_JOINT_INDICES, ARM_JOINT_NAMES


class ArmJointControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_arm_joint_controller')

        self.declare_parameter('cmd_topic', '/g1/arm_joint_position_cmd')
        self.declare_parameter('lowcmd_topic', '/lowcmd')
        self.declare_parameter('lowstate_topic', '/lowstate')
        self.declare_parameter('publish_rate_hz', 200.0)
        self.declare_parameter('mode_pr', 0)
        self.declare_parameter('mode_machine', 0)
        self.declare_parameter('control_mode', 1)
        self.declare_parameter('arm_joint_indices', ARM_JOINT_INDICES)
        self.declare_parameter('controlled_motor_count', 29)
        self.declare_parameter('kp', [80.0, 70.0, 60.0, 50.0, 20.0, 20.0, 20.0,
                                      80.0, 70.0, 60.0, 50.0, 20.0, 20.0, 20.0])
        self.declare_parameter('kd', [2.0] * ARM_DOF)
        self.declare_parameter('default_angles', [0.0] * ARM_DOF)
        self.declare_parameter('hold_non_arm_joints', True)
        self.declare_parameter('capture_standing_pose_from_lowstate', True)
        self.declare_parameter('non_arm_kp', 120.0)
        self.declare_parameter('non_arm_kd', 3.0)
        self.declare_parameter('non_arm_default_q', [0.0] * 29)
        self.declare_parameter('hold_last_command', True)
        self.declare_parameter('command_timeout_sec', 0.5)

        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.lowcmd_topic = self.get_parameter('lowcmd_topic').get_parameter_value().string_value
        self.lowstate_topic = self.get_parameter('lowstate_topic').get_parameter_value().string_value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.mode_pr = int(self.get_parameter('mode_pr').value)
        self.mode_machine = int(self.get_parameter('mode_machine').value)
        self.control_mode = int(self.get_parameter('control_mode').value)
        self.arm_joint_indices = [int(v) for v in self.get_parameter('arm_joint_indices').value]
        self.controlled_motor_count = int(self.get_parameter('controlled_motor_count').value)
        self.kp = np.asarray(self.get_parameter('kp').value, dtype=np.float64)
        self.kd = np.asarray(self.get_parameter('kd').value, dtype=np.float64)
        self.default_angles = np.asarray(self.get_parameter('default_angles').value, dtype=np.float64)
        self.hold_non_arm_joints = bool(self.get_parameter('hold_non_arm_joints').value)
        self.capture_standing_pose_from_lowstate = bool(
            self.get_parameter('capture_standing_pose_from_lowstate').value
        )
        self.non_arm_kp = float(self.get_parameter('non_arm_kp').value)
        self.non_arm_kd = float(self.get_parameter('non_arm_kd').value)
        self.non_arm_default_q = np.asarray(self.get_parameter('non_arm_default_q').value, dtype=np.float64)
        self.hold_last_command = bool(self.get_parameter('hold_last_command').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)

        self._validate_params()

        self.target_angles = self.default_angles.copy()
        self.last_cmd_time = self.get_clock().now()
        self.received_cmd = False
        self.warned_no_cmd = False
        self.standing_pose_q = self.non_arm_default_q.copy()
        self.standing_pose_captured = False
        self._printed_standing_fallback = False
        self.arm_index_set = set(self.arm_joint_indices)

        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            self.cmd_topic,
            self._cmd_cb,
            10,
        )
        self.state_sub = self.create_subscription(
            LowState,
            self.lowstate_topic,
            self._state_cb,
            10,
        )
        self.lowcmd_pub = self.create_publisher(LowCmd, self.lowcmd_topic, 10)

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._publish_lowcmd)

        self.get_logger().info(
            f'Controller started. cmd_topic={self.cmd_topic}, lowcmd_topic={self.lowcmd_topic}, '
            f'arm_dof={len(self.arm_joint_indices)}'
        )
        if self.hold_non_arm_joints:
            self.get_logger().info(
                f'Non-arm hold enabled (motors< {self.controlled_motor_count}, '
                f'non_arm_kp={self.non_arm_kp}, non_arm_kd={self.non_arm_kd}).'
            )
        self.get_logger().info(
            'Arm command order: ' + ', '.join(ARM_JOINT_NAMES)
        )

    def _validate_params(self) -> None:
        dof = len(self.arm_joint_indices)
        if dof != ARM_DOF:
            raise ValueError(f'arm_joint_indices must have {ARM_DOF} entries, got {dof}.')
        if self.kp.size != dof:
            raise ValueError(f'kp must have {dof} entries, got {self.kp.size}.')
        if self.kd.size != dof:
            raise ValueError(f'kd must have {dof} entries, got {self.kd.size}.')
        if self.default_angles.size != dof:
            raise ValueError(
                f'default_angles must have {dof} entries, got {self.default_angles.size}.'
            )
        if self.controlled_motor_count <= 0:
            raise ValueError('controlled_motor_count must be > 0.')
        if self.non_arm_default_q.size < self.controlled_motor_count:
            raise ValueError(
                f'non_arm_default_q must have at least {self.controlled_motor_count} entries, '
                f'got {self.non_arm_default_q.size}.'
            )
        for idx in self.arm_joint_indices:
            if idx >= self.controlled_motor_count:
                raise ValueError(
                    f'arm_joint_indices contains {idx}, outside controlled_motor_count={self.controlled_motor_count}.'
                )

    def _cmd_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != len(self.arm_joint_indices):
            self.get_logger().error(
                f'Invalid command size {len(msg.data)}. Expected {len(self.arm_joint_indices)} '
                f'for joints: {ARM_JOINT_NAMES}'
            )
            return

        self.target_angles = np.asarray(msg.data, dtype=np.float64)
        self.last_cmd_time = self.get_clock().now()
        self.received_cmd = True

    def _state_cb(self, msg: LowState) -> None:
        if not self.capture_standing_pose_from_lowstate or self.standing_pose_captured:
            return
        if len(msg.motor_state) < self.controlled_motor_count:
            return
        self.standing_pose_q = np.asarray(
            [msg.motor_state[i].q for i in range(self.controlled_motor_count)],
            dtype=np.float64,
        )
        self.standing_pose_captured = True
        self.get_logger().info('Captured standing pose from /lowstate for non-arm PD hold.')

    def _select_command(self) -> np.ndarray | None:
        if self.received_cmd:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if age <= self.command_timeout_sec or self.hold_last_command:
                return self.target_angles
            return self.default_angles

        if not self.warned_no_cmd:
            self.get_logger().warn('No arm command received yet; waiting for first command.')
            self.warned_no_cmd = True
        return None

    def _publish_lowcmd(self) -> None:
        active_cmd = self._select_command()
        if active_cmd is None:
            return

        msg = LowCmd()
        msg.mode_pr = self.mode_pr
        msg.mode_machine = self.mode_machine

        for motor in msg.motor_cmd:
            motor.mode = 0
            motor.q = 0.0
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = 0.0
            motor.kd = 0.0
            motor.reserve = 0

        if self.hold_non_arm_joints:
            if not self.standing_pose_captured and not self._printed_standing_fallback:
                self.get_logger().warn(
                    'Using non_arm_default_q until standing pose is captured from /lowstate.'
                )
                self._printed_standing_fallback = True
            for i in range(min(self.controlled_motor_count, len(msg.motor_cmd))):
                if i in self.arm_index_set:
                    continue
                motor = msg.motor_cmd[i]
                motor.mode = self.control_mode
                motor.q = float(self.standing_pose_q[i])
                motor.dq = 0.0
                motor.tau = 0.0
                motor.kp = float(self.non_arm_kp)
                motor.kd = float(self.non_arm_kd)
                motor.reserve = 0

        for i, motor_idx in enumerate(self.arm_joint_indices):
            if motor_idx >= len(msg.motor_cmd):
                self.get_logger().error(
                    f'arm_joint_indices includes {motor_idx}, but LowCmd only has '
                    f'{len(msg.motor_cmd)} motors.'
                )
                return

            motor = msg.motor_cmd[motor_idx]
            motor.mode = self.control_mode
            motor.q = float(active_cmd[i])
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = float(self.kp[i])
            motor.kd = float(self.kd[i])
            motor.reserve = 0

        msg.reserve[:] = [0, 0, 0, 0]
        msg.crc = 0
        self.lowcmd_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ArmJointControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
