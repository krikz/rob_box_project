"""#5161: единица и 9999 в /rtabmap/localization_pose - не измеренная дисперсия.

Предикат по коду, не по замеру на проводе (живое значение топика не снималось):
маргиналы g2o не посчитаны -> 6x6 остаётся единичной, slam2d затирает z/крен/тангаж
эпсилонами; пустая ковариация -> CoreWrapper ставит 9999.
"""

from rob_box_perception.rtabmap_pose_covariance import (
    NOT_LOCALIZED,
    REASON_MARGINAL,
    REASON_NOT_LOCALIZED,
    REASON_UNOBSERVABLE,
    SLAM2D_ANGULAR_EPSILON,
    SLAM2D_LINEAR_EPSILON,
    UNAVAILABLE,
    covariance_unavailable_reason,
    health_line,
)

#: Живая матрица /rtabmap/localization_pose, issue #5201, row-major 6x6.
#: Образ perception-humble-dev-f38fc04. Ранг 1, corr(x,y/yaw) = ±1, σ в сотни км.
#: Диагональ: x, y, z=1e-8, roll=3e-8, pitch=3e-8, yaw.
_LIVE_UNOBSERVABLE = [
    1.34935804e08, -5.99301356e09, 0.0, 0.0, 0.0, 4.79197337e09,
    -5.99301356e09, 2.66172585e11, 0.0, 0.0, 0.0, -2.12829810e11,
    0.0, 0.0, 1.00000000e-08, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 3.00000000e-08, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 3.00000000e-08, 0.0,
    4.79197337e09, -2.12829810e11, 0.0, 0.0, 0.0, 1.70177285e11,
]


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


def test_live_rank1_matrix_is_unobservable_not_a_measurement():
    """36 чисел с провода (#5201): σ сотни км, corr ±1. Это не дисперсия позы."""
    assert len(_LIVE_UNOBSERVABLE) == 36
    assert _LIVE_UNOBSERVABLE[0] == 1.34935804e08
    assert _LIVE_UNOBSERVABLE[7] == 2.66172585e11
    assert _LIVE_UNOBSERVABLE[14] == 1.00000000e-08
    assert _LIVE_UNOBSERVABLE[21] == 3.00000000e-08
    assert _LIVE_UNOBSERVABLE[28] == 3.00000000e-08
    assert _LIVE_UNOBSERVABLE[35] == 1.70177285e11
    assert covariance_unavailable_reason(_LIVE_UNOBSERVABLE) is REASON_UNOBSERVABLE


def test_room_scale_covariance_is_a_measurement():
    """Честная ошибка комнаты: diag 0.01/0.01/0.0001, вне диагонали нули. Не трогаем."""
    cov = [0.0] * 36
    cov[0] = 0.01
    cov[7] = 0.01
    cov[35] = 0.0001
    assert covariance_unavailable_reason(cov) is None


def test_identity_stays_marginal_and_9999_stays_not_localized():
    """Новая ветка не перехватывает две старые формы."""
    assert covariance_unavailable_reason(_eye()) is REASON_MARGINAL
    assert covariance_unavailable_reason(_not_localized()) is REASON_NOT_LOCALIZED


def test_unobservable_health_line_is_one_unavailable_line():
    line = health_line(REASON_UNOBSERVABLE)
    assert line == [f'{UNAVAILABLE}: {REASON_UNOBSERVABLE}']
    assert line[0].startswith(UNAVAILABLE)
