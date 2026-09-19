import math

import numpy as np
import pytest

from test_bullet_world import model_path
from pybullet_arm_control.color_detection import detect_blocks, infeed
from pybullet_arm_sim.bullet_world import ARM_JOINT_NAMES, BulletWorld
from pybullet_arm_sim.cameras import CameraRig
from pybullet_arm_sim.sorting_scene import create_sorting_scene


def observe(world):
    rig = CameraRig(world, 448, 448)
    frame = rig.render('external')
    return detect_blocks(frame.rgb, rig.intrinsics('external'), frame.position, frame.orientation)


@pytest.mark.parametrize('seed', [0, 7, 42])
def test_color_localization_on_actual_render(seed):
    world = BulletWorld(model_path())
    try:
        create_sorting_scene(world, seed)
        for _ in range(120):
            world.step()
        detections = [d for d in observe(world) if infeed(d)]
        print('Detections:', detections)
        assert len(detections) == 4
        assert [d.color for d in detections].count('red') == 2
        for body_id, box in world.objects.items():
            position, _ = world.box_pose(body_id)
            color = 'red' if box.color[0] > box.color[2] else 'blue'
            error = min(math.dist((d.x, d.y), position[:2]) for d in detections if d.color == color)
            assert error < .012
    finally:
        world.disconnect()


def test_visual_sorting_physically_moves_all_four_parts():
    world = BulletWorld(model_path())
    try:
        create_sorting_scene(world, 42)
        counts = {'red': 0, 'blue': 0}

        def settle(steps=240):
            for _ in range(steps):
                world.step()

        def move(position):
            start = world.joint_state()[1][:7]
            goal = world.solve_ik(position, (1, 0, 0, 0))
            for index in range(720):
                targets = np.array(start) + (np.array(goal) - start) * ((index + 1) / 720)
                world.set_joint_targets(dict(zip(ARM_JOINT_NAMES, targets)))
                world.step()
            settle(100)
            assert math.dist(world.link_pose('tool_tip')[0], position) < .015

        settle()
        for _ in range(4):
            detections = [d for d in observe(world) if infeed(d)]
            assert detections
            target = detections[0]
            destination = ((.36, .50)[counts[target.color]], .28 if target.color == 'red' else -.28)
            world.set_gripper_width(.08)
            settle()
            move((target.x, target.y, .20))
            move((target.x, target.y, .045))
            world.set_gripper_width(0)
            settle(480)
            fingers = {world.joints[n].index for n in ('panda_finger_joint1', 'panda_finger_joint2')}
            held = [body for body in world.objects if fingers.issubset(
                {c[3] for c in world.contacts() if c[2] == body and c[9] > .05})]
            assert len(held) == 1
            move((target.x, target.y, .25))
            assert world.box_pose(held[0])[0][2] > .18
            move((*destination, .25))
            move((*destination, .045))
            world.set_gripper_width(.08)
            settle()
            move((*destination, .25))
            # Park outside the infeed view before the next visual detection.
            move((.40, 0, .42))
            position, _ = world.box_pose(held[0])
            print(target.color, position, 'destination', destination)
            assert math.dist(position, (*destination, .035)) < .025
            counts[target.color] += 1
        assert counts == {'red': 2, 'blue': 2}
        assert not [d for d in observe(world) if infeed(d)]
    finally:
        world.disconnect()
