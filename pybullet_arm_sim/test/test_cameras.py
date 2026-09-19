from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pybullet as bullet
import pytest

from pybullet_arm_sim.bullet_world import ARM_JOINT_NAMES, BulletWorld
from pybullet_arm_sim.cameras import CameraRig


@pytest.fixture
def world():
    share = Path(get_package_share_directory('pybullet_arm_description'))
    instance = BulletWorld(str(share / 'urdf' / 'lab_arm.urdf.xacro'))
    instance.spawn_box(
        name='camera_target', position=(0.43, 0.0, 0.035),
        orientation=(0, 0, 0, 1), size=(0.06, 0.06, 0.07),
        mass=0.1, color=(0.95, 0.12, 0.04, 1.0),
    )
    for _ in range(120):
        instance.step()
    try:
        yield instance
    finally:
        instance.disconnect()


def red_pixels(rgb):
    values = rgb.astype(float)
    return ((values[:, :, 0] > 70)
            & (values[:, :, 0] > 2.0 * values[:, :, 1])
            & (values[:, :, 0] > 2.0 * values[:, :, 2]))


def test_both_cameras_show_cube_without_changing_physics(world):
    rig = CameraRig(world)
    before = world.joint_state()
    body = world.object_names['camera_target']
    box_before = bullet.getBasePositionAndOrientation(body, physicsClientId=world.client_id)
    velocity_before = bullet.getBaseVelocity(body, physicsClientId=world.client_id)
    for name in rig.names:
        frame = rig.render(name)
        assert frame.rgb.shape == (224, 224, 3)
        assert frame.rgb.dtype == np.uint8
        assert frame.rgb.flags.c_contiguous
        mask = red_pixels(frame.rgb)
        assert np.count_nonzero(mask) > 20, f'{name} camera cannot see the red cube'

        # A rendered object must project to the ROS CameraInfo pixel coordinates.
        rotation = np.asarray(bullet.getMatrixFromQuaternion(frame.orientation)).reshape(3, 3)
        point = rotation.T @ (np.asarray(box_before[0]) - np.asarray(frame.position))
        projected = np.asarray(rig.intrinsics(name)).reshape(3, 3) @ point
        expected = projected[:2] / projected[2]
        row, column = np.nonzero(mask)
        actual = np.array([column.mean(), row.mean()])
        assert np.linalg.norm(expected - actual) < 4.0
    assert world.joint_state() == before
    assert bullet.getBasePositionAndOrientation(body, physicsClientId=world.client_id) == box_before
    assert bullet.getBaseVelocity(body, physicsClientId=world.client_id) == velocity_before


def test_intrinsics_match_rectangular_render_projection(world):
    rig = CameraRig(world, width=320, height=240)
    for name in rig.names:
        intrinsic = np.asarray(rig.intrinsics(name)).reshape(3, 3)
        projection = np.asarray(rig.projection(name)).reshape(4, 4, order='F')
        optical = np.array([0.11, -0.07, 0.70])
        clip = projection @ np.array([optical[0], -optical[1], -optical[2], 1.0])
        ndc = clip[:3] / clip[3]
        projected_pixel = np.array([
            (ndc[0] + 1.0) * rig.width / 2.0,
            (1.0 - ndc[1]) * rig.height / 2.0,
        ])
        ros_pixel = intrinsic @ optical
        np.testing.assert_allclose(ros_pixel[:2] / ros_pixel[2], projected_pixel)
        assert intrinsic[0, 0] == pytest.approx(intrinsic[1, 1])
        assert rig.render(name).rgb.shape == (240, 320, 3)


def test_wrist_mount_follows_hand_while_external_stays_fixed(world):
    rig = CameraRig(world)
    external_before = rig.optical_pose('external')
    wrist_before = rig.optical_pose('wrist')
    world.set_joint_targets({'panda_joint1': 0.35})
    for _ in range(240):
        world.step()
    np.testing.assert_allclose(rig.optical_pose('external')[0], external_before[0], atol=1e-7)
    np.testing.assert_allclose(rig.optical_pose('external')[1], external_before[1], atol=1e-7)
    wrist_after = rig.optical_pose('wrist')
    assert np.linalg.norm(np.array(wrist_after[0]) - wrist_before[0]) > 0.05

    hand_position, hand_orientation = world.link_pose('panda_hand')
    inverse_hand = bullet.invertTransform(hand_position, hand_orientation)
    local_position, local_orientation = bullet.multiplyTransforms(*inverse_hand, *wrist_after)
    np.testing.assert_allclose(local_position, rig.specs['wrist'].position, atol=1e-6)
    assert abs(np.dot(local_orientation, rig.specs['wrist'].orientation)) == pytest.approx(1.0, abs=1e-6)


def test_wrist_sees_cube_during_top_down_approach(world):
    rig = CameraRig(world)
    target = world.solve_ik((0.43, 0.0, 0.16), (1.0, 0.0, 0.0, 0.0))
    world.set_joint_targets(dict(zip(ARM_JOINT_NAMES, target, strict=True)))
    world.set_gripper_width(0.08)
    for _ in range(480):
        world.step()
    actual, _ = world.link_pose('tool_tip')
    assert np.linalg.norm(np.array(actual) - (0.43, 0.0, 0.16)) < 0.02
    assert np.count_nonzero(red_pixels(rig.render('wrist').rgb)) > 50
