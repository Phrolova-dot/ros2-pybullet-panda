"""Contact validation and failed-grasp classification."""

from types import SimpleNamespace
import time

import pytest

from pybullet_arm_control.auto_sort import AutoSort, GraspFailure, grasped_bodies


def contact(body, finger, force=1.0):
    return SimpleNamespace(body_id=body, link_name='panda_' + finger + 'finger',
                           normal_force=force)


def test_fingers_must_touch_the_same_body():
    assert grasped_bodies([contact(1, 'left'), contact(2, 'right')]) == set()
    assert grasped_bodies([contact(1, 'left'), contact(1, 'right')]) == {1}
    assert grasped_bodies([contact(1, 'left'), contact(1, 'right', .01)]) == set()


def test_missing_contact_is_a_retryable_grasp_failure():
    def timeout(*args):
        raise RuntimeError('missing contact')

    fake = SimpleNamespace(
        held_body=None, open_gripper=lambda: None, state=lambda *args: None,
        move=lambda *args: None, gripper=SimpleNamespace(publish=lambda msg: None),
        wait=timeout, gripping=lambda: False)
    target = SimpleNamespace(color='red', x=.4, y=0)
    with pytest.raises(GraspFailure, match='missing contact'):
        AutoSort.grasp(fake, target)


def test_scan_rejects_late_frames_captured_before_parking():
    fake = SimpleNamespace(joint_stamp=200, observation=(100, time.monotonic(), []))
    def wait(predicate, *args):
        fake.observation = (150, time.monotonic(), ['old'])
        assert not predicate()
        fake.observation = (201, time.monotonic(), ['fresh'])
        assert predicate()
    fake.wait = wait
    assert AutoSort.scan(fake) == ['fresh']
