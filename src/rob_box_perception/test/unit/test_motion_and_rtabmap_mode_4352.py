"""Issue #4352: is_moving по окну скорости одометрии, mapping_mode из параметра rtabmap."""

from types import SimpleNamespace

from rob_box_perception import rtabmap_mode
from rob_box_perception.motion_state import MotionEstimator

#: Сэмплы /odom стоящего робота, замер 04.11.2026 (rtabmap/icp_odometry, робот на кирпичах).
STANDING_NOISE = (
    (-0.044, -0.015, 0.006),
    (0.020, 0.002, 0.019),
    (-0.006, 0.000, -0.007),
    (0.039, 0.005, -0.021),
)


def _fill(est, samples, t0=100.0, dt=0.2):
    for i, (vx, vy, wz) in enumerate(samples):
        est.add(t0 + i * dt, vx, vy, wz)
    return t0 + (len(samples) - 1) * dt


def test_standing_noise_is_not_motion():
    est = MotionEstimator()
    now = _fill(est, STANDING_NOISE)
    assert est.is_moving(now) is False


def test_old_threshold_would_call_noise_motion():
    """Прежнее правило |vx|>0.01 or |wz|>0.01 на последнем сэмпле давало True на этом шуме."""
    vx, _, wz = STANDING_NOISE[-1]
    assert abs(vx) > 0.01 or abs(wz) > 0.01


def test_slow_drive_is_motion():
    est = MotionEstimator()
    now = _fill(est, [(0.1, 0.0, 0.0)] * 5)
    assert est.is_moving(now) is True


def test_turn_in_place_is_motion():
    est = MotionEstimator()
    now = _fill(est, [(0.0, 0.0, 0.3)] * 5)
    assert est.is_moving(now) is True


def test_no_odometry_is_unknown():
    assert MotionEstimator().is_moving(100.0) is None


def test_stale_odometry_is_unknown():
    est = MotionEstimator()
    est.add(100.0, 0.3, 0.0, 0.0)
    assert est.is_moving(105.0) is None


def test_string_false_is_localization():
    value = SimpleNamespace(type=4, string_value='false', bool_value=False)
    assert rtabmap_mode.mode_from_parameter(value) == 'localization'


def test_string_true_is_mapping():
    value = SimpleNamespace(type=4, string_value='True', bool_value=False)
    assert rtabmap_mode.mode_from_parameter(value) == 'mapping'


def test_bool_parameter():
    assert rtabmap_mode.mode_from_parameter(SimpleNamespace(type=1, bool_value=True)) == 'mapping'


def test_unset_parameter_is_unknown():
    assert rtabmap_mode.mode_from_parameter(SimpleNamespace(type=0)) == 'unknown'
    assert rtabmap_mode.mode_from_parameter(None) == 'unknown'


def test_stale_answer_is_unknown():
    assert rtabmap_mode.current_mode('mapping', 100.0, 100.0 + rtabmap_mode.STALE_AFTER_S + 1) == 'unknown'
    assert rtabmap_mode.current_mode('mapping', 100.0, 101.0) == 'mapping'
    assert rtabmap_mode.current_mode('mapping', None, 101.0) == 'unknown'
