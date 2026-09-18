from __future__ import annotations

from collections import deque

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from pybullet_arm_interfaces.msg import ContactState, ContactStateArray
from pybullet_arm_interfaces.srv import SolveIK, SpawnBox
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

from .bullet_world import ARM_JOINT_NAMES, BulletWorld
from .validation import validate_named_positions


class SimulationNode(Node):
    def __init__(self) -> None:
        super().__init__('pybullet_sim')
        self.declare_parameter('robot_description_path', '')
        self.declare_parameter('pybullet_gui', False)
        self.declare_parameter('physics_hz', 240.0)
        self.declare_parameter('publish_hz', 60.0)
        self.declare_parameter('seed', 42)
        self.declare_parameter('spawn_default_box', True)

        xacro_path = str(self.get_parameter('robot_description_path').value)
        physics_hz = float(self.get_parameter('physics_hz').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        if not xacro_path:
            raise RuntimeError('robot_description_path parameter is required')
        if physics_hz <= 0.0 or publish_hz <= 0.0 or publish_hz > physics_hz:
            raise RuntimeError('require 0 < publish_hz <= physics_hz')

        self.world = BulletWorld(
            xacro_path=xacro_path,
            gui=bool(self.get_parameter('pybullet_gui').value),
            fixed_time_step=1.0 / physics_hz,
            seed=int(self.get_parameter('seed').value),
        )
        self.physics_hz = physics_hz
        self.publish_every = max(1, int(round(physics_hz / publish_hz)))
        self.step_count = 0
        self.sim_time_nanoseconds = 0
        self.paused = False
        self.path_history: deque[PoseStamped] = deque(maxlen=500)

        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 20)
        self.clock_publisher = self.create_publisher(Clock, '/clock', 20)
        self.contact_publisher = self.create_publisher(
            ContactStateArray, '/simulation/contacts', 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/simulation/objects', marker_qos
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped, '/arm/end_effector_pose', 10
        )
        self.path_publisher = self.create_publisher(Path, '/arm/end_effector_path', 10)

        self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self._trajectory_callback,
            20,
        )
        self.create_subscription(
            Float64, '/gripper/command', self._gripper_callback, 10
        )
        self.create_service(Trigger, '/simulation/reset', self._reset_callback)
        self.create_service(SetBool, '/simulation/set_paused', self._pause_callback)
        self.create_service(SolveIK, '/arm/solve_ik', self._solve_ik_callback)
        self.create_service(SpawnBox, '/simulation/spawn_box', self._spawn_box_callback)

        if bool(self.get_parameter('spawn_default_box').value):
            self.world.spawn_box(
                name='training_cube',
                position=(0.43, 0.0, 0.035),
                orientation=(0.0, 0.0, 0.0, 1.0),
                size=(0.06, 0.06, 0.07),
                mass=0.10,
                color=(0.95, 0.25, 0.08, 1.0),
            )

        self.timer = self.create_timer(1.0 / physics_hz, self._step_callback)
        self.get_logger().info(
            f'PyBullet arm ready: physics={physics_hz:.0f} Hz, '
            f'publish={physics_hz / self.publish_every:.0f} Hz'
        )

    def _trajectory_callback(self, message: JointTrajectory) -> None:
        if not message.points:
            self.get_logger().warning('ignored empty joint trajectory command')
            return
        point = message.points[-1]
        valid, reason = validate_named_positions(
            message.joint_names, point.positions, set(ARM_JOINT_NAMES)
        )
        if not valid:
            self.get_logger().warning(f'ignored invalid joint command: {reason}')
            return
        self.world.set_joint_targets(dict(zip(message.joint_names, point.positions)))

    def _gripper_callback(self, message: Float64) -> None:
        try:
            self.world.set_gripper_width(float(message.data))
        except ValueError as error:
            self.get_logger().warning(str(error))

    def _reset_callback(self, _request: Trigger.Request, response: Trigger.Response):
        try:
            self.world.reset_world()
            self.path_history.clear()
            if bool(self.get_parameter('spawn_default_box').value):
                self.world.spawn_box(
                    name='training_cube',
                    position=(0.43, 0.0, 0.035),
                    orientation=(0.0, 0.0, 0.0, 1.0),
                    size=(0.06, 0.06, 0.07),
                    mass=0.10,
                    color=(0.95, 0.25, 0.08, 1.0),
                )
            response.success = True
            response.message = 'simulation reset to home state'
            self._publish_state()
        except Exception as error:  # noqa: BLE001
            response.success = False
            response.message = str(error)
        return response

    def _pause_callback(self, request: SetBool.Request, response: SetBool.Response):
        self.paused = bool(request.data)
        response.success = True
        response.message = 'simulation paused' if self.paused else 'simulation running'
        return response

    def _solve_ik_callback(self, request: SolveIK.Request, response: SolveIK.Response):
        try:
            orientation = (
                request.target.orientation.x,
                request.target.orientation.y,
                request.target.orientation.z,
                request.target.orientation.w,
            )
            solution = self.world.solve_ik(
                position=(
                    request.target.position.x,
                    request.target.position.y,
                    request.target.position.z,
                ),
                orientation=orientation,
                end_effector_link=request.end_effector_link or 'tool_tip',
                seed_positions=request.seed_positions,
            )
            response.success = True
            response.joint_positions = solution
            response.message = 'IK solution computed; collision checking is not included'
        except (ValueError, RuntimeError) as error:
            response.success = False
            response.message = str(error)
        return response

    def _spawn_box_callback(self, request: SpawnBox.Request, response: SpawnBox.Response):
        try:
            orientation = (
                request.pose.orientation.x,
                request.pose.orientation.y,
                request.pose.orientation.z,
                request.pose.orientation.w,
            )
            if sum(value * value for value in orientation) < 1e-8:
                orientation = (0.0, 0.0, 0.0, 1.0)
            body_id = self.world.spawn_box(
                name=request.name,
                position=(
                    request.pose.position.x,
                    request.pose.position.y,
                    request.pose.position.z,
                ),
                orientation=orientation,
                size=(request.size.x, request.size.y, request.size.z),
                mass=float(request.mass),
                color=(request.color.r, request.color.g, request.color.b, request.color.a),
            )
            response.success = True
            response.body_id = body_id
            response.message = f'spawned {request.name}'
            self._publish_markers()
        except ValueError as error:
            response.success = False
            response.body_id = -1
            response.message = str(error)
        return response

    def _step_callback(self) -> None:
        if not self.paused:
            self.world.step()
            self.sim_time_nanoseconds += int(round(1_000_000_000 / self.physics_hz))
        self.step_count += 1
        if self.step_count % self.publish_every == 0:
            self._publish_state()

    def _stamp(self):
        seconds, nanoseconds = divmod(self.sim_time_nanoseconds, 1_000_000_000)
        stamp = Clock().clock
        stamp.sec = int(seconds)
        stamp.nanosec = int(nanoseconds)
        return stamp

    def _publish_state(self) -> None:
        stamp = self._stamp()
        clock = Clock()
        clock.clock = stamp
        self.clock_publisher.publish(clock)

        names, positions, velocities, efforts = self.world.joint_state()
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = names
        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts
        self.joint_state_publisher.publish(joint_state)

        position, orientation = self.world.link_pose('tool_tip')
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'base_link'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = orientation
        self.pose_publisher.publish(pose)
        self.path_history.append(pose)
        path = Path()
        path.header = pose.header
        path.poses = list(self.path_history)
        self.path_publisher.publish(path)

        self._publish_markers()
        self._publish_contacts(stamp)

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        ground = Marker()
        ground.header.frame_id = 'base_link'
        ground.header.stamp = self._stamp()
        ground.ns = 'simulation_ground'
        ground.type = Marker.CUBE
        ground.action = Marker.ADD
        ground.pose.position.x = 0.35
        ground.pose.position.z = -0.005
        ground.pose.orientation.w = 1.0
        ground.scale.x, ground.scale.y, ground.scale.z = 2.0, 2.0, 0.01
        ground.color.r, ground.color.g, ground.color.b, ground.color.a = 0.35, 0.38, 0.42, 1.0
        markers.markers.append(ground)
        for body_id, box in self.world.objects.items():
            position, orientation = self.world.box_pose(body_id)
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self._stamp()
            marker.ns = 'pybullet_objects'
            marker.text = box.name
            marker.id = body_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
            (
                marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w,
            ) = orientation
            marker.scale.x, marker.scale.y, marker.scale.z = box.size
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = box.color
            markers.markers.append(marker)
        self.marker_publisher.publish(markers)

    def _publish_contacts(self, stamp) -> None:
        message = ContactStateArray()
        message.header.stamp = stamp
        message.header.frame_id = 'base_link'
        link_names = {index: name for name, index in self.world.link_indices.items()}
        for contact in self.world.contacts():
            other_body = int(contact[2])
            if other_body not in self.world.objects:
                continue
            state = ContactState()
            state.body_id = other_body
            state.object_name = self.world.objects[other_body].name
            state.link_name = link_names.get(int(contact[3]), 'base_link')
            state.position.x, state.position.y, state.position.z = contact[5]
            state.normal.x, state.normal.y, state.normal.z = contact[7]
            state.normal_force = float(contact[9])
            message.contacts.append(state)
        self.contact_publisher.publish(message)

    def destroy_node(self):
        self.world.disconnect()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
