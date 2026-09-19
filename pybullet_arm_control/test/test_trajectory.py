from builtin_interfaces.msg import Duration
from pybullet_arm_control.trajectory import (
    ARM_JOINT_NAMES,
    parse_trajectory,
    sample_trajectory,
    validate_trajectory,
)
import pytest
from trajectory_msgs.msg import JointTrajectoryPoint


def point(positions, seconds):
    message = JointTrajectoryPoint()
    message.positions = list(positions)
    message.time_from_start = Duration(sec=seconds, nanosec=0)
    return message


def test_validation_and_interpolation():
    points = [
        point((0.0, -0.4, 0.0, -2.2, 0.0, 1.8, 0.7854), 1),
        point((1.0, -0.8, 0.4, -2.0, 0.2, 1.6, 0.7854), 3),
    ]
    assert validate_trajectory(ARM_JOINT_NAMES, points) == (True, '')
    trajectory = parse_trajectory(ARM_JOINT_NAMES, points)
    assert sample_trajectory((0.0,) * 7, trajectory, 0.0) == (0.0,) * 7
    assert sample_trajectory((0.0,) * 7, trajectory, 1.0) == tuple(
        points[0].positions
    )
    middle = sample_trajectory((0.0,) * 7, trajectory, 2.0)
    assert middle == pytest.approx((0.5, -0.6, 0.2, -2.1, 0.1, 1.7, 0.7854))
    assert sample_trajectory((0.0,) * 7, trajectory, 5.0) == tuple(
        points[-1].positions
    )


def test_validation_rejects_invalid_trajectories():
    valid_point = point((0.0, -0.4, 0.0, -2.2, 0.0, 1.8, 0.7854), 1)
    assert not validate_trajectory([], [valid_point])[0]
    assert not validate_trajectory(ARM_JOINT_NAMES, [point((0.0,), 1)])[0]
    assert not validate_trajectory(
        ARM_JOINT_NAMES, [valid_point, point((0.0, -0.4, 0.0, -2.2, 0.0, 1.8, 0.7854), 1)]
    )[0]
    assert not validate_trajectory(
        ARM_JOINT_NAMES, [point((99.0, -0.4, 0.0, -2.2, 0.0, 1.8, 0.7854), 1)]
    )[0]


def test_quintic_smooth_monotone_endpoints():
    start = (0., -.4, 0., -2.2, 0., 1.8, .7854)
    end = (.5, -.8, .2, -2., .1, 1.6, .7854)
    trajectory = parse_trajectory(ARM_JOINT_NAMES, [point(end, 3)])
    def sample(t):
        return sample_trajectory(start, trajectory, t, 'quintic')
    assert sample(0) == start
    assert sample(3) == end
    assert sample(1.5) == pytest.approx([(a + b) / 2 for a, b in zip(start, end)])
    assert abs(sample(.001)[0] - start[0]) < 1e-8
    assert abs(sample(2.999)[0] - end[0]) < 1e-8
    positions = [sample(i * .03)[0] for i in range(101)]
    assert positions == sorted(positions)
