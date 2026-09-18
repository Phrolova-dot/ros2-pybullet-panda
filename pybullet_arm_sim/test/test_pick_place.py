import math

from test_bullet_world import model_path
from pybullet_arm_sim.bullet_world import ARM_JOINT_NAMES, BulletWorld


def test_physical_pick_and_place():
    world = BulletWorld(model_path())
    try:
        box = world.spawn_box('training_cube', (.43, 0, .035), (0, 0, 0, 1),
                              (.06, .06, .07), .1, (1, .2, .1, 1))

        def settle(count=240):
            for _ in range(count):
                world.step()

        def move(position):
            start = world.joint_state()[1][:7]
            end = world.solve_ik(position, (1, 0, 0, 0))
            for step in range(720):
                ratio = (step + 1) / 720
                world.set_joint_targets(dict(zip(
                    ARM_JOINT_NAMES, [a + ratio * (b-a) for a, b in zip(start, end)])))
                world.step()
            settle()
            assert math.dist(world.link_pose('tool_tip')[0], position) < .015

        world.set_gripper_width(.08)
        settle()
        move((.43, 0, .20))
        move((.43, 0, .045))
        world.set_gripper_width(0)
        settle(480)
        fingers = {world.joints[name].index for name in
                   ('panda_finger_joint1', 'panda_finger_joint2')}
        touching = {c[3] for c in world.contacts() if c[2] == box and c[9] > .05}
        assert fingers.issubset(touching)
        move((.43, 0, .25))
        assert world.box_pose(box)[0][2] > .18
        move((.43, .25, .25))
        assert math.dist(world.box_pose(box)[0][:2], (.43, .25)) < .025
        move((.43, .25, .045))
        world.set_gripper_width(.08)
        settle()
        move((.43, .25, .25))
        settle()
        final = world.box_pose(box)[0]
        print('Final box position:', final)
        assert math.dist(final, (.43, .25, .035)) < .025
    finally:
        world.disconnect()
