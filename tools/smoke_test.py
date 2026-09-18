#!/usr/bin/env python3
from __future__ import annotations

import sys
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from pybullet_arm_interfaces.srv import SolveIK, SpawnBox
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINT_NAMES = tuple(f'panda_joint{index}' for index in range(1, 8))


class SmokeTest(Node):
    def __init__(self) -> None:
        super().__init__('pybullet_arm_smoke_test')
        self.clock_seen = False
        self.latest_positions: dict[str, float] = {}
        self.create_subscription(Clock, '/clock', self._clock_callback, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_callback, 20)
        self.reset_client = self.create_client(Trigger, '/simulation/reset')
        self.ik_client = self.create_client(SolveIK, '/arm/solve_ik')
        self.spawn_client = self.create_client(SpawnBox, '/simulation/spawn_box')
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )

    def _clock_callback(self, _message: Clock) -> None:
        self.clock_seen = True

    def _joint_callback(self, message: JointState) -> None:
        for name, position in zip(message.name, message.position):
            self.latest_positions[name] = float(position)

    def spin_until(self, predicate, timeout: float, description: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise TimeoutError(f'timed out waiting for {description}')

    def call(self, client, request, description: str):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'{description} service is unavailable')
        future = client.call_async(request)
        self.spin_until(future.done, 10.0, description)
        if future.exception() is not None:
            raise RuntimeError(f'{description} failed: {future.exception()}')
        return future.result()

    def run(self) -> None:
        self.spin_until(
            lambda: self.clock_seen
            and all(name in self.latest_positions for name in ARM_JOINT_NAMES),
            12.0,
            '/clock and /joint_states',
        )
        print('[PASS] simulation topics are active')

        reset = self.call(self.reset_client, Trigger.Request(), 'reset')
        if not reset.success:
            raise RuntimeError(reset.message)
        print('[PASS] reset service')

        spawn_request = SpawnBox.Request()
        spawn_request.name = 'smoke_box'
        spawn_request.pose.position.x = 0.35
        spawn_request.pose.position.y = -0.20
        spawn_request.pose.position.z = 0.025
        spawn_request.pose.orientation.w = 1.0
        spawn_request.size.x = 0.05
        spawn_request.size.y = 0.05
        spawn_request.size.z = 0.05
        spawn_request.mass = 0.05
        spawn_request.color.r = 0.1
        spawn_request.color.g = 0.8
        spawn_request.color.b = 0.2
        spawn_request.color.a = 1.0
        spawned = self.call(self.spawn_client, spawn_request, 'spawn box')
        if not spawned.success or spawned.body_id < 0:
            raise RuntimeError(spawned.message)
        print(f'[PASS] spawn service, body_id={spawned.body_id}')

        ik_request = SolveIK.Request()
        ik_request.target.orientation.x = 1.0
        ik_request.target.orientation.w = 0.0
        ik_request.target.position.x = 0.42
        ik_request.target.position.y = 0.10
        ik_request.target.position.z = 0.24
        ik_request.end_effector_link = 'tool_tip'
        ik_response = self.call(self.ik_client, ik_request, 'IK')
        if not ik_response.success or len(ik_response.joint_positions) != 7:
            raise RuntimeError(ik_response.message)
        print('[PASS] IK service')

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('trajectory action server is unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(ik_response.joint_positions)
        point.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [point]
        goal_future = self.action_client.send_goal_async(goal)
        self.spin_until(goal_future.done, 10.0, 'trajectory goal acceptance')
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('trajectory goal was rejected')
        result_future = goal_handle.get_result_async()
        self.spin_until(result_future.done, 10.0, 'trajectory result')
        wrapped_result = result_future.result()
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(wrapped_result.result.error_string)
        if wrapped_result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(wrapped_result.result.error_string)
        print('[PASS] FollowJointTrajectory action')


def main() -> int:
    rclpy.init()
    node = SmokeTest()
    try:
        node.run()
        print('[PASS] all integration checks completed')
        return 0
    except Exception as error:  # noqa: BLE001
        print(f'[FAIL] {error}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
