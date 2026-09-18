import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from pybullet_arm_sim.bullet_world import ARM_JOINT_NAMES, BulletWorld


def model_path() -> str:
    share = Path(get_package_share_directory('pybullet_arm_description'))
    return str(share / 'urdf' / 'lab_arm.urdf.xacro')


def test_world_loads_steps_and_solves_ik():
    world = BulletWorld(model_path(), gui=False, fixed_time_step=1.0 / 240.0)
    try:
        names, positions, velocities, efforts = world.joint_state()
        assert set(ARM_JOINT_NAMES).issubset(names)
        assert len(names) == 9
        assert len(names) == len(positions) == len(velocities) == len(efforts)
        assert all(math.isfinite(value) for value in positions)

        world.set_joint_targets({'panda_joint1': 0.25})
        for _ in range(240):
            world.step()
        _, moved, _, _ = world.joint_state()
        assert abs(moved[names.index('panda_joint1')] - 0.25) < 0.03

        solution = world.solve_ik((0.45, 0.10, 0.25), orientation=(1.0, 0.0, 0.0, 0.0))
        assert len(solution) == len(ARM_JOINT_NAMES)
        assert all(math.isfinite(value) for value in solution)
        world.set_joint_targets(dict(zip(ARM_JOINT_NAMES, solution)))
        world.set_gripper_width(0.08)
        for _ in range(480):
            world.step()
        actual, _ = world.link_pose('tool_tip')
        assert math.dist(actual, (0.45, 0.10, 0.25)) < 0.015
        finger_names, finger_positions, _, _ = world.joint_state()
        for name in ('panda_finger_joint1', 'panda_finger_joint2'):
            assert abs(finger_positions[finger_names.index(name)] - 0.04) < 0.002
    finally:
        world.disconnect()


def test_spawn_box_rejects_duplicate_name():
    world = BulletWorld(model_path(), gui=False)
    try:
        arguments = {
            'name': 'box',
            'position': (0.4, 0.0, 0.05),
            'orientation': (0.0, 0.0, 0.0, 1.0),
            'size': (0.05, 0.05, 0.05),
            'mass': 0.1,
            'color': (1.0, 0.0, 0.0, 1.0),
        }
        world.spawn_box(**arguments)
        try:
            world.spawn_box(**arguments)
            assert False, 'duplicate object name should fail'
        except ValueError:
            pass
    finally:
        world.disconnect()
