"""#5161: единица и 9999 в /rtabmap/localization_pose - не измеренная дисперсия.

Предикат по коду, не по замеру на проводе (живое значение топика не снималось):
маргиналы g2o не посчитаны -> 6x6 остаётся единичной, slam2d затирает z/крен/тангаж
эпсилонами; пустая ковариация -> CoreWrapper ставит 9999.
"""

from rob_box_perception.rtabmap_pose_covariance import (
    NOT_LOCALIZED,
    SLAM2D_ANGULAR_EPSILON,
    SLAM2D_LINEAR_EPSILON,
    covariance_unavailable_reason,
    health_line,
)


def _eye():
    cov = [0.0] * 36
    for index in (0, 7, 14, 21, 28, 35):
        cov[index] = 1.0
    return cov


def _slam2d_default():
    """Единица на x/y/yaw, эпсилоны slam2d на z/крен/тангаж (Rtabmap.cpp:4226-4231, тег 0.24.2)."""
    cov = _eye()
    cov[14] = SLAM2D_LINEAR_EPSILON
    cov[21] = SLAM2D_ANGULAR_EPSILON
    cov[28] = SLAM2D_ANGULAR_EPSILON
    return cov


def _not_localized():
    """CoreWrapper.cpp:2446-2452, ветка 2D: 9999 и эпсилоны."""
    cov = [0.0] * 36
    cov[0] = cov[7] = cov[35] = NOT_LOCALIZED
    cov[14] = SLAM2D_LINEAR_EPSILON
    cov[21] = cov[28] = SLAM2D_ANGULAR_EPSILON
    return cov


def test_identity_is_unpublished_marginal_not_a_variance():
    reason = covariance_unavailable_reason(_eye())
    assert reason is not None
    assert 'маргиналы' in reason
    assert 'ковариация позы недоступна' in health_line(reason)[0]


def test_slam2d_epsilon_identity_is_the_same_failure():
    reason = covariance_unavailable_reason(_slam2d_default())
    assert reason is not None and 'маргиналы' in reason


def test_9999_is_not_localized_and_not_the_marginal_failure():
    reason = covariance_unavailable_reason(_not_localized())
    assert reason is not None
    assert '9999' in reason
    assert 'маргиналы' not in reason


def test_real_covariance_is_left_alone():
    """Посчитанный блок маргиналов: малое xx и ненулевая вне диагонали. Число не выдумываем."""
    cov = _slam2d_default()
    cov[0] = 0.02
    cov[1] = 0.001
    assert covariance_unavailable_reason(cov) is None
    assert health_line(None) == []


def test_missing_or_short_covariance_is_unavailable_not_a_fake_number():
    assert covariance_unavailable_reason(None) is not None
    assert covariance_unavailable_reason([]) is not None
    assert covariance_unavailable_reason([1.0] * 35) is not None
    line = health_line(covariance_unavailable_reason(None))[0]
    assert '1.0' not in line and '9999' not in line


def test_off_diagonal_one_is_not_the_default():
    """Единица на (0, 1) - не след непрочитанной матрицы. Не называем это дефолтом."""
    cov = _slam2d_default()
    cov[1] = 1.0
    assert covariance_unavailable_reason(cov) is None
