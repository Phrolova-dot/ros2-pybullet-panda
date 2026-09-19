"""Feedback-checked pick and place for the default Panda training cube."""

import math
import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from pybullet_arm_interfaces.msg import ContactStateArray
from pybullet_arm_interfaces.srv import SolveIK
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray

from .trajectory import ARM_JOINT_NAMES


class PickPlace(Node):
    def __init__(self, node_name='pick_place', use_object_feedback=True):
        super().__init__(node_name)
        self.declare_parameter('place_x', 0.43)
        self.declare_parameter('place_y', 0.25)
        self.box = None
        self.box_time = 0.0
        self.tip = None
        self.tip_time = 0.0
        self.fingers = set()
        self.contact_time = 0.0
        self.width = 0.0
        self.joint_time = 0.0
        self.joint_stamp = -1
        self.active_goal = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if use_object_feedback:
            self.create_subscription(MarkerArray, '/simulation/objects', self.objects, qos)
        self.create_subscription(ContactStateArray, '/simulation/contacts', self.contacts, 10)
        self.create_subscription(PoseStamped, '/arm/end_effector_pose', self.pose, 10)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.gripper = self.create_publisher(Float64, '/gripper/command', 10)
        self.ik = self.create_client(SolveIK, '/arm/solve_ik')
        self.action = ActionClient(self, FollowJointTrajectory,
                                   '/arm_controller/follow_joint_trajectory')

    def objects(self, message):
        self.box = next((m for m in message.markers
                         if m.ns == 'pybullet_objects' and m.text == 'training_cube'), None)
        self.box_time = time.monotonic()

    def contacts(self, message):
        self.fingers = {c.link_name for c in message.contacts
                        if c.object_name == 'training_cube' and c.normal_force > 0.05}
        self.contact_time = time.monotonic()

    def pose(self, message):
        p = message.pose.position
        self.tip = (p.x, p.y, p.z)
        self.tip_time = time.monotonic()

    def joints(self, message):
        positions = dict(zip(message.name, message.position))
        if all(n in positions for n in ('panda_finger_joint1', 'panda_finger_joint2')):
            self.width = positions['panda_finger_joint1'] + positions['panda_finger_joint2']
            self.joint_time = time.monotonic()
            self.joint_stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def wait(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError('Timed out: ' + label)

    def settle(self, seconds=1.0):
        end = time.monotonic() + seconds
        self.wait(lambda: time.monotonic() >= end, seconds + 2.0, 'settling')

    def box_position(self):
        if self.box is None or time.monotonic() - self.box_time > 1.0:
            raise RuntimeError('Fresh training_cube state is unavailable; restart the updated simulation')
        p = self.box.pose.position
        return (p.x, p.y, p.z)

    def gripping(self):
        return (time.monotonic() - self.contact_time < 0.5
                and {'panda_leftfinger', 'panda_rightfinger'}.issubset(self.fingers))

    def open_gripper(self):
        self.gripper.publish(Float64(data=0.08))
        self.wait(lambda: time.monotonic() - self.joint_time < 0.5 and self.width > 0.077,
                  4.0, 'gripper opening')
        self.settle(0.5)

    def move(self, target, label):
        self.get_logger().info(label)
        request = SolveIK.Request()
        request.target.position.x, request.target.position.y, request.target.position.z = target
        request.target.orientation.x = 1.0
        request.target.orientation.w = 0.0
        request.end_effector_link = 'tool_tip'
        future = self.ik.call_async(request)
        self.wait(future.done, 5.0, 'IK')
        solution = future.result()
        if solution is None or not solution.success:
            raise RuntimeError('IK failed')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(solution.joint_positions)
        point.time_from_start = Duration(sec=3)
        goal.trajectory.points = [point]
        future = self.action.send_goal_async(goal)
        self.wait(future.done, 5.0, 'goal acceptance')
        self.active_goal = future.result()
        if self.active_goal is None or not self.active_goal.accepted:
            raise RuntimeError('Trajectory rejected')
        future = self.active_goal.get_result_async()
        self.wait(future.done, 10.0, 'trajectory completion')
        result = future.result().result
        self.active_goal = None
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(result.error_string)
        self.wait(lambda: self.tip is not None and time.monotonic() - self.tip_time < 0.5
                  and math.dist(self.tip, target) < 0.015, 3.0, 'Cartesian target')
        self.settle(0.08)

    def run(self):
        destination = (float(self.get_parameter('place_x').value),
                       float(self.get_parameter('place_y').value))
        if not all(math.isfinite(v) for v in destination):
            raise RuntimeError('Place coordinates must be finite')
        if not (0.30 <= destination[0] <= 0.55 and -0.30 <= destination[1] <= 0.30):
            raise RuntimeError('Demo place area: x=[0.30,0.55], y=[-0.30,0.30]')
        if not self.ik.wait_for_service(timeout_sec=10) or not self.action.wait_for_server(timeout_sec=10):
            raise RuntimeError('Start simulation.launch.py before this program')
        self.wait(lambda: self.box is not None and self.tip is not None
                  and self.gripper.get_subscription_count() > 0, 10.0, 'simulation feedback')
        start = self.box_position()
        size = (self.box.scale.x, self.box.scale.y, self.box.scale.z)
        if (abs(start[2] - 0.035) > 0.015
                or math.dist(size, (0.06, 0.06, 0.07)) > 0.001
                or not (0.30 <= start[0] <= 0.55 and -0.30 <= start[1] <= 0.30)):
            raise RuntimeError('This demo requires the standard training cube resting on the ground')
        self.open_gripper()
        self.move((start[0], start[1], 0.20), 'APPROACH')
        self.move((start[0], start[1], 0.045), 'DESCEND')
        self.get_logger().info('GRASP')
        self.gripper.publish(Float64(data=0.0))
        self.wait(self.gripping, 5.0, 'two-finger contact')
        self.settle(1.0)
        if not self.gripping():
            raise RuntimeError('Grip was lost before lift')
        self.move((start[0], start[1], 0.25), 'LIFT')
        lifted = self.box_position()
        if lifted[2] < 0.18 or not self.gripping():
            raise RuntimeError('Lift failed: cube not held above the ground')
        self.get_logger().info(f'Lift verified: cube z={lifted[2]:.3f} m')
        self.move((*destination, 0.25), 'TRANSFER')
        if math.dist(self.box_position()[:2], destination) > 0.025 or not self.gripping():
            raise RuntimeError('Transfer failed: cube slipped')
        self.move((*destination, 0.045), 'LOWER')
        self.open_gripper()
        self.move((*destination, 0.25), 'RETREAT')
        self.settle()
        final = self.box_position()
        if math.dist(final, (*destination, 0.035)) > 0.025 or self.gripping():
            raise RuntimeError(f'Placement verification failed: {final}')
        self.get_logger().info(f'PASS: cube moved from {start} to {final}')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlace()
    success = False
    try:
        node.run()
        success = True
    except (RuntimeError, KeyboardInterrupt) as error:
        node.get_logger().error(f'Pick/place stopped: {error}')
    finally:
        if node.active_goal is not None and node.active_goal.accepted:
            future = node.active_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(1)
