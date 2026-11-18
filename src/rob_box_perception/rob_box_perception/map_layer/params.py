"""Пороги семантического слоя карты (ADR-0169 §2.3–2.4).

Все числа — начальная оценка ADR, не замер на роботе.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MapLayerParams:
    """Пороги слоя; неверная комбинация — ``ValueError`` сразу."""

    #: Подтверждений до confirmed.
    confirm_hits: int = 5
    #: Разных ракурсов до confirmed.
    confirm_views: int = 2
    #: Окно, за которое кандидат должен подтвердиться, с.
    confirm_window_sec: float = 600.0
    #: Минимальные ворота сопоставления, м.
    match_min_m: float = 0.6
    #: Добавка к половине extent в воротах, м.
    match_extent_extra_m: float = 0.3
    #: Ближняя граница вклада (там мерена глубина), м.
    min_distance_m: float = 0.6
    #: Дальняя граница вклада, м.
    max_distance_m: float = 4.0
    #: Смещение позы робота, с которого ракурс новый, м.
    view_min_move_m: float = 0.5
    #: Поворот позы робота, с которого ракурс новый, рад.
    view_min_yaw_rad: float = math.radians(20.0)
    #: Сколько поз ``seen_from`` помнить.
    max_seen_from: int = 8

    def __post_init__(self) -> None:
        if self.confirm_hits < 1:
            raise ValueError("MapLayerParams: confirm_hits >= 1")
        if self.confirm_views < 1:
            raise ValueError("MapLayerParams: confirm_views >= 1")
        if self.confirm_window_sec <= 0:
            raise ValueError("MapLayerParams: confirm_window_sec > 0")
        if self.match_min_m <= 0:
            raise ValueError("MapLayerParams: match_min_m > 0")
        if self.min_distance_m <= 0 or self.max_distance_m <= self.min_distance_m:
            raise ValueError("MapLayerParams: 0 < min_distance_m < max_distance_m")
