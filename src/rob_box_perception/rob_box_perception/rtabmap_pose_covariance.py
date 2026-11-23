"""rtabmap_pose_covariance.py - честный вердикт ковариации /rtabmap/localization_pose (#5161).

rtabmap в локализации считает маргиналы последней вершины графа в g2o (Optimizer/Strategy=1).
Когда маргиналы не считаются, апстрим пишет WARN ``has hessian not valid`` / ``negative hessian
index`` и оставляет ковариацию единичной матрицей, которой её инициализировали. Эта матрица
уходит в топик как число. Молча заменить её на 1.0 или заглушить WARN - ложь: 1.0 выглядит как
измеренная дисперсия 1 м².

Этот модуль не чинит граф и не глушит WARN. Он говорит одно: ковариация позы недоступна.

Что известно по коду (не по замеру на проводе; живое значение топика не снималось):

* ``OptimizerG2O::optimize`` инициализирует ``outputCovariance = cv::Mat::eye(6, 6)``
  (rtabmap 0.24.5, OptimizerG2O.cpp:233; номера строк WARN 1337/1341 совпали с логом робота).
  Блок маргиналов пишется только если ``hessianIndex`` внутри ``blockCols``. Иначе матрица
  остаётся единичной. ``Reg/Force3DoF=true`` делает ``isSlam2d()``, и Rtabmap.cpp (тег 0.24.2,
  строки 4226-4231) затирает z/крен/тангаж эпсилонами, а блок x/y/yaw не трогает.
* Пустая ковариация в статистике - другая ветка: CoreWrapper (rtabmap_ros 0.23.13-humble,
  CoreWrapper.cpp:2446-2452) публикует 9999 на диагонали и пишет «Not yet localized».
  ``LocalizationStatusTask`` считает ``cov(0,0) >= 9999`` признаком «не локализован», а единицу -
  обычной ошибкой. Поэтому 9999 и единицу нельзя сваливать в одну причину.
* Эпсилоны slam2d: ``Registration::COVARIANCE_LINEAR_EPSILON = 1e-8``,
  ``COVARIANCE_ANGULAR_EPSILON = 3e-8`` (Registration.cpp:36-37, тег 0.24.2).

Сдвиг версий честно: ядро читалось по тегу 0.24.2 (0.24.5 для Rtabmap.cpp отдал 404), обёртка -
по 0.23.13-humble, бинарь на роботе - 0.24.5 (замер ``rtabmap --version``).
"""

from __future__ import annotations

from typing import Optional, Sequence

#: Длина ``geometry_msgs/PoseWithCovariance.covariance`` (6x6, row-major).
COVARIANCE_LENGTH = 36

#: Порог «ещё не локализован»: CoreWrapper ставит 9999, статусная задача сравнивает ``>= 9999``.
NOT_LOCALIZED = 9999.0

#: Эпсилоны slam2d (Registration.cpp, тег 0.24.2). На диагонали z/крен/тангаж после Force3DoF.
SLAM2D_LINEAR_EPSILON = 1e-8
SLAM2D_ANGULAR_EPSILON = 3e-8

#: 2D-блок, который остаётся единичным, когда маргиналы не посчитаны: xx, yy, yaw.
#: xy и xθ не здесь: ненулевой внедиагональный элемент - след посчитанного блока, не дефолт.
_PLANAR_DIAGONAL = ((0, 1.0), (7, 1.0), (35, 1.0))

#: Диагональ «не локализован» в 2D: xx, yy, yaw = 9999; z/крен/тангаж = эпсилоны.
_NOT_LOCALIZED_DIAGONAL = (
    (0, NOT_LOCALIZED),
    (7, NOT_LOCALIZED),
    (14, SLAM2D_LINEAR_EPSILON),
    (21, SLAM2D_ANGULAR_EPSILON),
    (28, SLAM2D_ANGULAR_EPSILON),
    (35, NOT_LOCALIZED),
)

UNAVAILABLE = 'ковариация позы недоступна'
REASON_MARGINAL = 'маргиналы не посчитаны, в топике единица'
REASON_NOT_LOCALIZED = 'ещё не локализован, в топике 9999'
REASON_MISSING = 'ковариация в сообщении пуста или короче 36'

#: Допуск на сравнение double, пришедшего через ROS (memcpy из cv::Mat).
_ABS_TOL = 1e-12


def _close(value: float, expected: float) -> bool:
    """``value`` совпадает с ``expected`` с допуском на шум memcpy."""
    return abs(value - expected) <= _ABS_TOL


def _off_diagonal_clear(cov: Sequence[float]) -> bool:
    """Вне диагонали нули: ненулевой внедиагональный элемент - след посчитанного блока."""
    return all(_close(cov[index], 0.0) for index in range(COVARIANCE_LENGTH) if index % 7 != 0)


def _matches(cov: Sequence[float], pairs: Sequence[tuple]) -> bool:
    """Все пары ``(индекс, ожидаемое)`` совпали, и вне диагонали нет ненулевых элементов."""
    if not _off_diagonal_clear(cov):
        return False
    return all(_close(cov[index], expected) for index, expected in pairs)


def _unpublished_marginal(cov: Sequence[float]) -> bool:
    """2D-блок остался единичным: маргиналы не писали xx/yy/xθ/θθ.

    z/крен/тангаж при ``Force3DoF`` затираются эпсилонами (Rtabmap.cpp:4226-4231),
    но если ``isSlam2d()`` не сработал, там остаются единицы инициализации.
    Оба варианта - непрочитанная матрица, не дисперсия.
    """
    if not _off_diagonal_clear(cov):
        return False
    if not all(_close(cov[index], expected) for index, expected in _PLANAR_DIAGONAL):
        return False
    z_roll_pitch = (cov[14], cov[21], cov[28])
    untouched = all(_close(value, 1.0) for value in z_roll_pitch)
    slam2d = (
        _close(cov[14], SLAM2D_LINEAR_EPSILON)
        and _close(cov[21], SLAM2D_ANGULAR_EPSILON)
        and _close(cov[28], SLAM2D_ANGULAR_EPSILON)
    )
    return untouched or slam2d


def covariance_unavailable_reason(covariance: Optional[Sequence[float]]) -> Optional[str]:
    """Почему ковариация недоступна, или ``None``, если в ней есть посчитанное число.

    ``None``/короче 36 - поле не пришло. Единичный 2D-блок (z/крен/тангаж - эпсилоны или тоже
    единицы) - маргиналы не посчитаны, единицу нельзя выдавать за дисперсию. Диагональ 9999 -
    ещё не локализован. Любая другая конечная матрица - посчитанная ковариация, её не трогаем
    и число не выдумываем.
    """
    if covariance is None or len(covariance) < COVARIANCE_LENGTH:
        return REASON_MISSING
    cov = [float(item) for item in covariance[:COVARIANCE_LENGTH]]
    if _unpublished_marginal(cov):
        return REASON_MARGINAL
    if _matches(cov, _NOT_LOCALIZED_DIAGONAL):
        return REASON_NOT_LOCALIZED
    return None


def health_line(reason: Optional[str]) -> list:
    """Строки для ``health_issues``: пусто, если позы не было или ковариация посчитана.

    Причина в строке: 9999 и единица - разные ветки апстрима, сваливать их в одну фразу - снова ложь.
    Список, а не ``None``: нода добавляет через ``extend`` и не растит CC ``check_system_health``.
    """
    if reason is None:
        return []
    return [f'{UNAVAILABLE}: {reason}']
