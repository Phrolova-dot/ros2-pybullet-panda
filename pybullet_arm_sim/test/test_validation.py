import math

from pybullet_arm_sim.validation import all_finite, clamp, validate_named_positions


def test_validate_named_positions_accepts_well_formed_input():
    valid, reason = validate_named_positions(['a', 'b'], [0.1, 0.2], {'a', 'b'})
    assert valid
    assert reason == ''


def test_validate_named_positions_rejects_bad_input():
    assert not validate_named_positions([], [], {'a'})[0]
    assert not validate_named_positions(['a', 'a'], [0.0, 0.0], {'a'})[0]
    assert not validate_named_positions(['a'], [], {'a'})[0]
    assert not validate_named_positions(['b'], [0.0], {'a'})[0]
    assert not validate_named_positions(['a'], [math.nan], {'a'})[0]


def test_numeric_helpers():
    assert all_finite([0.0, 1.0, -2.0])
    assert not all_finite([math.inf])
    assert clamp(-1.0, 0.0, 2.0) == 0.0
    assert clamp(3.0, 0.0, 2.0) == 2.0
    assert clamp(1.0, 0.0, 2.0) == 1.0
