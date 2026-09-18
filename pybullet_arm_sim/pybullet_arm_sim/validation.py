from collections.abc import Iterable, Sequence
import math


def all_finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def validate_named_positions(
    names: Sequence[str],
    positions: Sequence[float],
    allowed_names: set[str],
) -> tuple[bool, str]:
    if not names:
        return False, 'joint_names must not be empty'
    if len(names) != len(set(names)):
        return False, 'joint_names contains duplicates'
    if len(names) != len(positions):
        return False, 'joint_names and positions have different lengths'
    unknown = sorted(set(names) - allowed_names)
    if unknown:
        return False, f'unknown joints: {", ".join(unknown)}'
    if not all_finite(positions):
        return False, 'positions contains NaN or infinity'
    return True, ''


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
