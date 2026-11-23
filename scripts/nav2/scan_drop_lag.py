"""#5151: лаг дропа скана costmap — это таймаут MessageFilter, не возраст кэша TF.

Пин образа nav2 — navigation2 a0970867 (docker/main/nav2/Dockerfile).
ObstacleLayer (и VoxelLayer, тот же код) строит фильтр так:

    obstacle_layer.cpp:230-234
    MessageFilter(..., tf2::durationFromSec(transform_tolerance))

7-й аргумент — buffer_timeout (geometry2 humble, message_filter.hpp:191-204).
По таймеру Buffer::timerCallback бросает TimeoutException
(buffer.cpp), а transformReadyCallback любое исключение future.get()
помечает OutTheBack (message_filter.hpp:549-554). Строка у OutTheBack одна
(message_filter.hpp:105-107): «earlier than all the data in the transform
cache». Печать троттлится 2500 мс (message_filter.hpp:714), поэтому число
строк — нижняя граница, не число сканов.

Отсюда лаг (log_stamp − scan_stamp) дропа, который не успел дождаться TF,
равен transform_tolerance, а не «скан старше всего кэша». Живой замер
23.11: min 1.00027 с при transform_tolerance 1.0, скан отстаёт на 5–10 мс.

tf_buffer_duration в yaml узел не читает: Costmap2DROS объявляет только
transform_tolerance (costmap_2d_ros.cpp:98,150) и строит буфер без
cache_time (costmap_2d_ros.cpp:181). Дефолт tf2 humble — 10 с
(buffer_core.hpp, BUFFER_CORE_DEFAULT_CACHE_TIME). Поддержку мёртвого
ключа в код не добавлять (решение Шифу 23.11).
"""

from __future__ import annotations

import math

# geometry2 humble, buffer_core.hpp: BUFFER_CORE_DEFAULT_CACHE_TIME.
TF2_DEFAULT_CACHE_S = 10.0
# obstacle_layer.cpp:252 — setTolerance(0.05). Это time_tolerance_ фильтра
# (message_filter.hpp:330-334), не таймаут ожидания: фильтр просит TF ещё
# и на штамп скана + 0.05 с.
FILTER_TIME_TOLERANCE_S = 0.05
# message_filter.hpp:714 — RCLCPP_INFO_THROTTLE(..., 2500, ...).
DROP_LOG_THROTTLE_MS = 2500
# Живой замер карточки #5151, 23.11 06:30–07:30Z. Не пересчёт: сырьё
# гистограммы в репо нет, цифры — из тела issue.
MEASURED_DROP_LAG_MIN_S = 1.00027
MEASURED_DROP_LAG_MAX_S = 1.454
MEASURED_DROP_LAG_AVG_S = 1.016


def filter_timeout_s(transform_tolerance_s: float) -> float:
    """Сколько фильтр ждёт TF на штамп скана. Это и есть нижняя граница лага дропа."""
    if transform_tolerance_s < 0.0 or not math.isfinite(transform_tolerance_s):
        raise ValueError(f"transform_tolerance must be a finite non-negative number, got {transform_tolerance_s}")
    return transform_tolerance_s


def drop_lag_is_filter_timeout(lag_s: float, transform_tolerance_s: float) -> bool:
    """Лаг дропа сидит на таймауте фильтра, а не «скан старше кэша».

    Нижняя граница — сам таймаут: таймер не стреляет раньше. Верхняя —
    таймаут плюс setTolerance(0.05): фильтр ждёт TF и на штамп+0.05, и
    строка лога пишется по готовности future, не в момент прихода скана.
    """
    if lag_s < 0.0 or not math.isfinite(lag_s):
        raise ValueError(f"lag must be a finite non-negative number, got {lag_s}")
    timeout = filter_timeout_s(transform_tolerance_s)
    return timeout <= lag_s <= timeout + FILTER_TIME_TOLERANCE_S


def dead_yaml_key_does_not_set_cache(yaml_seconds: float, cache_seconds: float) -> bool:
    """Ключ, которого узел не объявляет, кэш не меняет: 30 в yaml, 10 в буфере."""
    return yaml_seconds != cache_seconds and cache_seconds == TF2_DEFAULT_CACHE_S
