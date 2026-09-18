import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from .trajectory import ARM_JOINT_NAMES


WAYPOINTS = (
    (2.0, (0.0, -0.4, 0.0, -2.2, 0.0, 1.8, 0.7854)),
    (4.0, (0.45, -0.6, 0.15, -2.1, 0.0, 1.7, 0.7854)),
    (6.0, (-0.45, -0.6, -0.15, -2.1, 0.0, 1.7, 0.7854)),
    (8.0, (0.0, -0.2, 0.0, -1.9, 0.0, 1.7, 0.7854)),
    (10.0, (0.0, -0.4, 0.0, -2.2, 0.0, 1.8, 0.7854)),
)


def duration(seconds: float) -> Duration:
    message = Duration()
    message.sec = int(seconds)
    message.nanosec = int((seconds - int(seconds)) * 1_000_000_000)
    return message


class DemoSequence(Node):
    def __init__(self) -> None:
        super().__init__('demo_sequence')
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )
        self.last_feedback_log = 0.0

    def feedback_callback(self, feedback_message) -> None:
        now = time.monotonic()
        if now - self.last_feedback_log < 0.5:
            return
        self.last_feedback_log = now
        errors = feedback_message.feedback.error.positions
        if errors:
            self.get_logger().info(
                f'max tracking error: {max(abs(value) for value in errors):.3f}'
            )

    def run(self) -> bool:
        self.get_logger().info('waiting for trajectory action server')
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('trajectory action server is unavailable')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINT_NAMES)
        for time_from_start, positions in WAYPOINTS:
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start = duration(time_from_start)
            goal.trajectory.points.append(point)

        future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('demo trajectory was rejected')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(result.error_string)
        return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoSequence()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
