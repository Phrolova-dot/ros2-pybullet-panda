from __future__ import annotations

from dataclasses import dataclass
import math
import os
import tempfile
from typing import Iterable

from ament_index_python.packages import get_package_share_directory
import pybullet as bullet
import pybullet_data
import xacro

from .validation import all_finite, clamp


ARM_JOINT_NAMES = tuple(f'panda_joint{index}' for index in range(1, 8))
GRIPPER_JOINT_NAMES = ('panda_finger_joint1', 'panda_finger_joint2')
DEFAULT_HOME = (0.0, -0.4, 0.0, -2.2, 0.0, 1.8, 0.7854)


@dataclass(frozen=True)
class JointSpec:
    name: str
    index: int
    link_name: str
    lower: float
    upper: float
    max_force: float
    max_velocity: float


@dataclass(frozen=True)
class BoxObject:
    body_id: int
    name: str
    size: tuple[float, float, float]
    color: tuple[float, float, float, float]


class BulletWorld:
    """Owns one PyBullet client and all mutable simulation state."""

    def __init__(
        self,
        xacro_path: str,
        gui: bool = False,
        fixed_time_step: float = 1.0 / 240.0,
        seed: int = 42,
    ) -> None:
        if fixed_time_step <= 0.0 or not math.isfinite(fixed_time_step):
            raise ValueError('fixed_time_step must be finite and greater than zero')
        if not os.path.isfile(xacro_path):
            raise FileNotFoundError(xacro_path)

        self.xacro_path = xacro_path
        self.gui = gui
        self.fixed_time_step = fixed_time_step
        self.seed = seed
        self.client_id = bullet.connect(bullet.GUI if gui else bullet.DIRECT)
        if self.client_id < 0:
            raise RuntimeError('failed to connect to PyBullet')
        if gui:
            # ROS camera previews live in their own window, not GUI debug buffers.
            for flag in (bullet.COV_ENABLE_GUI, bullet.COV_ENABLE_RGB_BUFFER_PREVIEW,
                         bullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                         bullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW):
                bullet.configureDebugVisualizer(flag, 0, physicsClientId=self.client_id)

        self.robot_id = -1
        self.urdf_path = ''
        self.joints: dict[str, JointSpec] = {}
        self.link_indices: dict[str, int] = {'base_link': -1}
        self.objects: dict[int, BoxObject] = {}
        self.object_names: dict[str, int] = {}
        self.targets: dict[str, float] = {}
        self.load()

    def disconnect(self) -> None:
        if self.client_id >= 0 and bullet.isConnected(self.client_id):
            bullet.disconnect(physicsClientId=self.client_id)
        self.client_id = -1
        self._remove_temporary_urdf()

    def _remove_temporary_urdf(self) -> None:
        if self.urdf_path and os.path.exists(self.urdf_path):
            os.unlink(self.urdf_path)
        self.urdf_path = ''

    def _create_urdf(self) -> str:
        document = xacro.process_file(self.xacro_path)
        for mesh in document.getElementsByTagName('mesh'):
            uri = mesh.getAttribute('filename')
            if uri.startswith('package://'):
                package, relative = uri[len('package://'):].split('/', 1)
                mesh.setAttribute('filename', os.path.join(
                    get_package_share_directory(package), relative))
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.urdf', prefix='pybullet_arm_', delete=False
        )
        with handle:
            handle.write(document.toprettyxml(indent='  '))
        return handle.name

    def load(self) -> None:
        self._remove_temporary_urdf()
        bullet.resetSimulation(physicsClientId=self.client_id)
        bullet.setAdditionalSearchPath(
            pybullet_data.getDataPath(), physicsClientId=self.client_id
        )
        bullet.setGravity(0.0, 0.0, -9.81, physicsClientId=self.client_id)
        bullet.setTimeStep(self.fixed_time_step, physicsClientId=self.client_id)
        bullet.setPhysicsEngineParameter(
            fixedTimeStep=self.fixed_time_step,
            numSolverIterations=100,
            deterministicOverlappingPairs=1,
            physicsClientId=self.client_id,
        )
        bullet.loadURDF('plane.urdf', physicsClientId=self.client_id)

        self.urdf_path = self._create_urdf()
        self.robot_id = bullet.loadURDF(
            self.urdf_path,
            basePosition=(0.0, 0.0, 0.0),
            useFixedBase=True,
            flags=bullet.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self.client_id,
        )
        self._read_model()
        self.objects.clear()
        self.object_names.clear()
        self.targets = dict(zip(ARM_JOINT_NAMES, DEFAULT_HOME, strict=True))
        self.targets.update({name: 0.015 for name in GRIPPER_JOINT_NAMES})
        self.reset_robot()

    def _read_model(self) -> None:
        self.joints.clear()
        self.link_indices = {'base_link': -1}
        for index in range(
            bullet.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        ):
            info = bullet.getJointInfo(
                self.robot_id, index, physicsClientId=self.client_id
            )
            name = info[1].decode('utf-8')
            link_name = info[12].decode('utf-8')
            self.link_indices[link_name] = index
            if info[2] == bullet.JOINT_FIXED:
                continue
            self.joints[name] = JointSpec(
                name=name,
                index=index,
                link_name=link_name,
                lower=float(info[8]),
                upper=float(info[9]),
                max_force=max(0.1, float(info[10])),
                max_velocity=max(0.01, float(info[11])),
            )

        expected = set(ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES)
        missing = sorted(expected - set(self.joints))
        if missing:
            raise RuntimeError(f'URDF is missing controlled joints: {missing}')
        if 'tool_tip' not in self.link_indices:
            raise RuntimeError('URDF is missing tool_tip link')

    def reset_robot(self) -> None:
        for name, target in self.targets.items():
            spec = self.joints[name]
            bullet.resetJointState(
                self.robot_id,
                spec.index,
                targetValue=target,
                targetVelocity=0.0,
                physicsClientId=self.client_id,
            )
        self._apply_motor_targets()

    def reset_world(self) -> None:
        self.load()

    def _apply_motor_targets(self) -> None:
        names = ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES
        bullet.setJointMotorControlArray(
            self.robot_id,
            [self.joints[name].index for name in names],
            bullet.POSITION_CONTROL,
            targetPositions=[self.targets[name] for name in names],
            forces=[self.joints[name].max_force for name in names],
            positionGains=[0.08] * len(names),
            velocityGains=[1.0] * len(names),
            physicsClientId=self.client_id,
        )

    def set_joint_targets(self, targets: dict[str, float]) -> None:
        unknown = sorted(set(targets) - set(self.joints))
        if unknown:
            raise ValueError(f'unknown joints: {unknown}')
        if not all_finite(targets.values()):
            raise ValueError('joint targets must be finite')
        for name, value in targets.items():
            spec = self.joints[name]
            self.targets[name] = clamp(float(value), spec.lower, spec.upper)
        self._apply_motor_targets()

    def set_gripper_width(self, width: float) -> None:
        if not math.isfinite(width):
            raise ValueError('gripper width must be finite')
        half_width = clamp(width / 2.0, 0.0, 0.04)
        self.set_joint_targets({name: half_width for name in GRIPPER_JOINT_NAMES})

    def step(self) -> None:
        self._apply_motor_targets()
        bullet.stepSimulation(physicsClientId=self.client_id)

    def joint_state(self) -> tuple[list[str], list[float], list[float], list[float]]:
        names = list(ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES)
        states = bullet.getJointStates(
            self.robot_id,
            [self.joints[name].index for name in names],
            physicsClientId=self.client_id,
        )
        return (
            names,
            [float(state[0]) for state in states],
            [float(state[1]) for state in states],
            [float(state[3]) for state in states],
        )

    def link_pose(
        self, link_name: str
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        if link_name == 'base_link':
            position, orientation = bullet.getBasePositionAndOrientation(
                self.robot_id, physicsClientId=self.client_id
            )
        else:
            if link_name not in self.link_indices:
                raise ValueError(f'unknown link: {link_name}')
            state = bullet.getLinkState(
                self.robot_id,
                self.link_indices[link_name],
                computeForwardKinematics=True,
                physicsClientId=self.client_id,
            )
            position, orientation = state[4], state[5]
        return tuple(position), tuple(orientation)

    def solve_ik(
        self,
        position: Iterable[float],
        orientation: Iterable[float] | None = None,
        end_effector_link: str = 'tool_tip',
        seed_positions: Iterable[float] | None = None,
    ) -> list[float]:
        target_position = tuple(float(value) for value in position)
        if len(target_position) != 3 or not all_finite(target_position):
            raise ValueError('target position must contain three finite values')
        if end_effector_link not in self.link_indices:
            raise ValueError(f'unknown end-effector link: {end_effector_link}')

        movable_names = list(ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES)
        lower = [self.joints[name].lower for name in movable_names]
        upper = [self.joints[name].upper for name in movable_names]
        ranges = [upper_value - lower_value for lower_value, upper_value in zip(lower, upper)]
        rest = [self.targets[name] for name in movable_names]
        if seed_positions is not None:
            seed = [float(value) for value in seed_positions]
            if seed:
                if len(seed) != len(ARM_JOINT_NAMES) or not all_finite(seed):
                    raise ValueError('seed_positions must contain seven finite arm positions')
                rest[: len(ARM_JOINT_NAMES)] = seed

        kwargs = {
            'bodyUniqueId': self.robot_id,
            'endEffectorLinkIndex': self.link_indices[end_effector_link],
            'targetPosition': target_position,
            'lowerLimits': lower,
            'upperLimits': upper,
            'jointRanges': ranges,
            'restPoses': rest,
            'jointDamping': [0.05] * len(movable_names),
            'maxNumIterations': 200,
            'residualThreshold': 1e-5,
            'physicsClientId': self.client_id,
        }
        if orientation is not None:
            target_orientation = tuple(float(value) for value in orientation)
            if len(target_orientation) != 4 or not all_finite(target_orientation):
                raise ValueError('target orientation must contain four finite values')
            if sum(value * value for value in target_orientation) > 1e-8:
                kwargs['targetOrientation'] = target_orientation

        solution = bullet.calculateInverseKinematics(**kwargs)
        arm_solution = []
        for index, name in enumerate(ARM_JOINT_NAMES):
            spec = self.joints[name]
            arm_solution.append(clamp(float(solution[index]), spec.lower, spec.upper))
        return arm_solution

    def spawn_box(
        self,
        name: str,
        position: Iterable[float],
        orientation: Iterable[float],
        size: Iterable[float],
        mass: float,
        color: Iterable[float],
    ) -> int:
        clean_name = name.strip()
        dimensions = tuple(float(value) for value in size)
        pose = tuple(float(value) for value in position)
        rotation = tuple(float(value) for value in orientation)
        rgba = tuple(clamp(float(value), 0.0, 1.0) for value in color)
        if not clean_name:
            raise ValueError('object name must not be empty')
        if clean_name in self.object_names:
            raise ValueError(f'object already exists: {clean_name}')
        if len(dimensions) != 3 or not all_finite(dimensions) or any(v <= 0.0 for v in dimensions):
            raise ValueError('box size must contain three positive finite values')
        if len(pose) != 3 or not all_finite(pose):
            raise ValueError('box position must contain three finite values')
        if len(rotation) != 4 or not all_finite(rotation):
            raise ValueError('box orientation must contain four finite values')
        if len(rgba) != 4 or not math.isfinite(mass) or mass < 0.0:
            raise ValueError('invalid color or mass')

        half_extents = [value / 2.0 for value in dimensions]
        collision = bullet.createCollisionShape(
            bullet.GEOM_BOX,
            halfExtents=half_extents,
            physicsClientId=self.client_id,
        )
        visual = bullet.createVisualShape(
            bullet.GEOM_BOX,
            halfExtents=half_extents,
            rgbaColor=rgba,
            physicsClientId=self.client_id,
        )
        body_id = bullet.createMultiBody(
            baseMass=float(mass),
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=pose,
            baseOrientation=rotation,
            physicsClientId=self.client_id,
        )
        self.objects[body_id] = BoxObject(body_id, clean_name, dimensions, rgba)
        self.object_names[clean_name] = body_id
        return body_id

    def box_pose(
        self, body_id: int
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        position, orientation = bullet.getBasePositionAndOrientation(
            body_id, physicsClientId=self.client_id
        )
        return tuple(position), tuple(orientation)

    def contacts(self) -> list[tuple]:
        return list(
            bullet.getContactPoints(
                bodyA=self.robot_id, physicsClientId=self.client_id
            )
        )
