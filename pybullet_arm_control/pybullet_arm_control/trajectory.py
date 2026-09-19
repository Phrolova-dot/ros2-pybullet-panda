from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math


ARM_JOINT_NAMES = tuple(f'panda_joint{index}' for index in range(1, 8))

JOINT_LIMITS = dict(zip(ARM_JOINT_NAMES, (
    (-2.9671, 2.9671), (-1.8326, 1.8326), (-2.9671, 2.9671),
    (-3.1416, 0.0), (-2.9671, 2.9671), (-0.0873, 3.8223),
    (-2.9671, 2.9671),
), strict=True))


@dataclass(frozen=True)
class ParsedTrajectory:
    joint_names: tuple[str, ...]
    times: tuple[float, ...]
    positions: tuple[tuple[float, ...], ...]


def duration_to_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) / 1_000_000_000.0


def validate_trajectory(joint_names: Sequence[str], points: Sequence) -> tuple[bool, str]:
    if tuple(joint_names) != ARM_JOINT_NAMES:
        return False, f'joint_names must be exactly {list(ARM_JOINT_NAMES)}'
    if not points:
        return False, 'trajectory must contain at least one point'

    previous_time = 0.0
    for index, point in enumerate(points):
        if len(point.positions) != len(ARM_JOINT_NAMES):
            return False, f'point {index} has the wrong number of positions'
        if not all(math.isfinite(float(value)) for value in point.positions):
            return False, f'point {index} contains NaN or infinity'
        point_time = duration_to_seconds(point.time_from_start)
        if not math.isfinite(point_time) or point_time <= previous_time:
            return False, 'time_from_start values must be finite and strictly increasing'
        for name, value in zip(ARM_JOINT_NAMES, point.positions, strict=True):
            lower, upper = JOINT_LIMITS[name]
            if float(value) < lower or float(value) > upper:
                return False, f'{name} target {value} is outside [{lower}, {upper}]'
        previous_time = point_time
    return True, ''


def parse_trajectory(joint_names: Sequence[str], points: Sequence) -> ParsedTrajectory:
    valid, reason = validate_trajectory(joint_names, points)
    if not valid:
        raise ValueError(reason)
    return ParsedTrajectory(
        joint_names=tuple(joint_names),
        times=tuple(duration_to_seconds(point.time_from_start) for point in points),
        positions=tuple(tuple(float(value) for value in point.positions) for point in points),
    )


def sample_trajectory(
    start_positions: Sequence[float], trajectory: ParsedTrajectory, elapsed: float,
    interpolation: str = 'linear',
) -> tuple[float, ...]:
    if interpolation not in ('linear', 'quintic'):
        raise ValueError('interpolation must be linear or quintic')
    if len(start_positions) != len(trajectory.joint_names):
        raise ValueError('start position length does not match trajectory')
    if elapsed <= 0.0:
        return tuple(float(value) for value in start_positions)
    if elapsed >= trajectory.times[-1]:
        return trajectory.positions[-1]

    segment_start_time = 0.0
    segment_start = tuple(float(value) for value in start_positions)
    for segment_end_time, segment_end in zip(
        trajectory.times, trajectory.positions, strict=True
    ):
        if elapsed <= segment_end_time:
            ratio = (elapsed - segment_start_time) / (
                segment_end_time - segment_start_time
            )
            if interpolation == 'quintic':
                # Zero velocity and acceleration at each segment boundary.
                ratio = ratio ** 3 * (10.0 - 15.0 * ratio + 6.0 * ratio ** 2)
            return tuple(
                start + ratio * (end - start)
                for start, end in zip(segment_start, segment_end, strict=True)
            )
        segment_start_time = segment_end_time
        segment_start = segment_end
    return trajectory.positions[-1]
