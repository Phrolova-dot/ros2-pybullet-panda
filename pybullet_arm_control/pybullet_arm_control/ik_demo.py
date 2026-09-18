from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from pybullet_arm_interfaces.srv import SolveIK
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from .trajectory import ARM_JOINT_NAMES


class IkDemo(Node):
    def __init__(self) -> None:
        super().__init__('ik_demo')
        self.declare_parameter('target_x', 0.42)
        self.declare_parameter('target_y', 0.12)
        self.declare_parameter('target_z', 0.24)
        self.ik_client = self.create_client(SolveIK, '/arm/solve_ik')
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )

    def run(self) -> bool:
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('IK service is unavailable')
            return False
        request = SolveIK.Request()
        request.target.orientation.x = 1.0
        request.target.orientation.w = 0.0
        request.target.position.x = float(self.get_parameter('target_x').value)
        request.target.position.y = float(self.get_parameter('target_y').value)
        request.target.position.z = float(self.get_parameter('target_z').value)
        request.end_effector_link = 'tool_tip'
        ik_future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, ik_future)
        response = ik_future.result()
        if response is None or not response.success:
            self.get_logger().error(response.message if response else 'IK request failed')
            return False

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('trajectory action server is unavailable')
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(response.joint_positions)
        point.time_from_start = Duration(sec=3, nanosec=0)
        goal.trajectory.points = [point]
        goal_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('IK trajectory was rejected')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(result.error_string)
        return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IkDemo()
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
