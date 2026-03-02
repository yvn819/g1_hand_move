import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .joint_map import ARM_DOF, ARM_JOINT_NAMES


class ArmPoseSenderNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_arm_pose_sender')

        self.declare_parameter('cmd_topic', '/g1/arm_joint_position_cmd')
        self.declare_parameter('joint_angles', [0.0] * ARM_DOF)
        self.declare_parameter('repeat_count', 20)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.joint_angles = [float(x) for x in self.get_parameter('joint_angles').value]
        self.repeat_count = int(self.get_parameter('repeat_count').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        if len(self.joint_angles) != ARM_DOF:
            raise ValueError(f'joint_angles must contain {ARM_DOF} entries, got {len(self.joint_angles)}')

        self.pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        self.sent_count = 0
        self.done = False

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            f'Publishing arm target to {self.cmd_topic} for {self.repeat_count} cycles. '
            f'Order: {ARM_JOINT_NAMES}'
        )

    def _timer_cb(self) -> None:
        if self.sent_count >= self.repeat_count:
            self.done = True
            self.timer.cancel()
            self.get_logger().info('Done publishing arm target command.')
            return

        msg = Float64MultiArray()
        msg.data = self.joint_angles
        self.pub.publish(msg)
        self.sent_count += 1


def main() -> None:
    rclpy.init()
    node = ArmPoseSenderNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
