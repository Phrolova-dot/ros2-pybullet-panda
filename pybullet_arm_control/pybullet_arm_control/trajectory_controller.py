from __future__ import annotations

import threading
import time

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .trajectory import ARM_JOINT_NAMES, parse_trajectory, sample_trajectory, validate_trajectory


class TrajectoryController(Node):
    def __init__(self) -> None:
        super().__init__('trajectory_controller')
        self.declare_parameter('command_hz', 120.0)
        self.declare_parameter('interpolation', 'quintic')
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('settle_timeout', 2.0)
        self.command_hz = float(self.get_parameter('command_hz').value)
        self.interpolation = str(self.get_parameter('interpolation').value)
        if self.interpolation not in ('linear', 'quintic'):
            raise RuntimeError('interpolation must be linear or quintic')
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.settle_timeout = float(self.get_parameter('settle_timeout').value)
        if self.command_hz <= 0.0 or self.goal_tolerance <= 0.0:
            raise RuntimeError('command_hz and goal_tolerance must be positive')

        self.callback_group = ReentrantCallbackGroup()
        self.command_publisher = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 20
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            20,
            callback_group=self.callback_group,
        )
        self.current_positions: dict[str, float] = {}
        self.state_lock = threading.Lock()
        self.goal_lock = threading.Lock()
        self.goal_active = False
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info('FollowJointTrajectory action server ready')

    def _joint_state_callback(self, message: JointState) -> None:
        with self.state_lock:
            for name, position in zip(message.name, message.position):
                self.current_positions[name] = float(position)

    def _goal_callback(self, request: FollowJointTrajectory.Goal) -> GoalResponse:
        valid, reason = validate_trajectory(
            request.trajectory.joint_names, request.trajectory.points
        )
        if not valid:
            self.get_logger().warning(f'rejected trajectory: {reason}')
            return GoalResponse.REJECT
        with self.state_lock:
            ready = all(name in self.current_positions for name in ARM_JOINT_NAMES)
        if not ready:
            self.get_logger().warning('rejected trajectory: joint state is not ready')
            return GoalResponse.REJECT
        with self.goal_lock:
            if self.goal_active:
                self.get_logger().warning('rejected trajectory: another goal is active')
                return GoalResponse.REJECT
            self.goal_active = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _positions(self) -> tuple[float, ...]:
        with self.state_lock:
            return tuple(self.current_positions[name] for name in ARM_JOINT_NAMES)

    def _publish_command(self, positions: tuple[float, ...]) -> None:
        command = JointTrajectory()
        command.joint_names = list(ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        command.points = [point]
        self.command_publisher.publish(command)

    def _publish_feedback(self, goal_handle, desired: tuple[float, ...], elapsed: float):
        actual = self._positions()
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(ARM_JOINT_NAMES)
        feedback.desired.positions = list(desired)
        feedback.actual.positions = list(actual)
        feedback.error.positions = [
            target - measured for target, measured in zip(desired, actual, strict=True)
        ]
        seconds = max(0.0, elapsed)
        feedback.desired.time_from_start.sec = int(seconds)
        feedback.desired.time_from_start.nanosec = int(
            (seconds - int(seconds)) * 1_000_000_000
        )
        goal_handle.publish_feedback(feedback)

    def _result(self, code: int, message: str) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = message
        return result

    def _execute_callback(self, goal_handle):
        request = goal_handle.request
        trajectory = parse_trajectory(
            request.trajectory.joint_names, request.trajectory.points
        )
        start_positions = self._positions()
        start_time = time.monotonic()
        period = 1.0 / self.command_hz

        try:
            while True:
                elapsed = time.monotonic() - start_time
                if goal_handle.is_cancel_requested:
                    hold = self._positions()
                    self._publish_command(hold)
                    goal_handle.canceled()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        'trajectory canceled; current position is held',
                    )
                if elapsed >= trajectory.times[-1]:
                    break
                desired = sample_trajectory(
                    start_positions, trajectory, elapsed, self.interpolation)
                self._publish_command(desired)
                self._publish_feedback(goal_handle, desired, elapsed)
                time.sleep(period)

            final_positions = trajectory.positions[-1]
            self._publish_command(final_positions)
            settle_start = time.monotonic()
            while time.monotonic() - settle_start < self.settle_timeout:
                actual = self._positions()
                max_error = max(
                    abs(target - measured)
                    for target, measured in zip(final_positions, actual, strict=True)
                )
                self._publish_feedback(
                    goal_handle, final_positions, trajectory.times[-1]
                )
                if max_error <= self.goal_tolerance:
                    goal_handle.succeed()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        f'trajectory completed with max error {max_error:.4f}',
                    )
                if goal_handle.is_cancel_requested:
                    self._publish_command(actual)
                    goal_handle.canceled()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        'trajectory canceled while settling',
                    )
                time.sleep(period)

            actual = self._positions()
            max_error = max(
                abs(target - measured)
                for target, measured in zip(final_positions, actual, strict=True)
            )
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                f'goal tolerance violated: max error {max_error:.4f}',
            )
        finally:
            with self.goal_lock:
                self.goal_active = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
