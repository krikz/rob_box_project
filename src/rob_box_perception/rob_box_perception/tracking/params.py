"""Параметры камерного трекера — единственное место (ADR-0130 этап 2, ADR-0148).

Все числа по умолчанию — **догадка**, не замер: калибруются на наборе
разметки ADR-0144 и записях с робота (план этапа 2, PR «замер»). ADR-0130
§2.6 прямо оставляет пороги открытыми; здесь — только пороги физического
трека, не имени.

Единицы: метры, секунды. Координаты — плоскость пола (x, y) того фрейма,
в котором трекер ведёт треки (``map`` по ADR-0130 §2.14 ``Track.header``).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple

#: Классы детекций → класс трека. ``face`` и ``person`` — один человек
#: (ADR-0130 §2.1: лицо и person-bbox сходятся в трек ``class=person``).
DEFAULT_CLASS_MAP: Dict[str, str] = {'person': 'person', 'face': 'person'}

#: Какие классы детекций рождают трек. Лицо без тела трек не рождает:
#: иначе каждое лицо, пришедшее отдельным пакетом, плодило бы дубль
#: трека рядом с person-bbox того же человека.
DEFAULT_BIRTH_CLASSES: Tuple[str, ...] = ('person',)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(f'TrackerParams: {message}')


@dataclass(frozen=True)
class TrackerParams:
    """Параметры трекера; неверная комбинация — ``ValueError`` сразу."""

    #: Ворота сопоставления активного трека: дальше — не тот объект, м.
    gate_m: float = 0.8
    #: Ворота повторного захвата потерянного трека (около места потери), м.
    reacquire_gate_m: float = 1.0
    #: Попаданий подряд (в пределах ``tentative_timeout_sec``) до confirmed.
    confirm_hits: int = 3
    #: Tentative без обновления дольше этого — удаляется как шум, с.
    tentative_timeout_sec: float = 0.5
    #: Confirmed без обновления дольше этого — становится lost, с.
    lost_after_sec: float = 1.0
    #: Lost без обновления дольше этого (от последнего обновления) — удаляется, с.
    delete_after_sec: float = 10.0
    #: Разрешение трекера: треки ближе — «сближение» (ADR-0130 §6), м.
    resolution_m: float = 0.4
    #: Потолок одновременных треков (ADR-0130 §6: «десяток треков»).
    max_tracks: int = 16
    #: Шум измерения положения (без учёта разброса глубины), СКО, м.
    meas_std_m: float = 0.15
    #: Спектральная плотность белого шума ускорения, м²/с³.
    accel_noise: float = 1.0
    #: Начальная СКО скорости нового трека, м/с.
    init_vel_std: float = 1.0
    #: Скидка к стоимости при совпадении ``candidate_id`` трека и детекции, м.
    candidate_match_bonus_m: float = 0.2
    #: Штраф к стоимости при разных непустых ``candidate_id``, м.
    candidate_mismatch_penalty_m: float = 0.5
    #: Префикс ``track_id``. Узел добавляет к нему метку запуска (план, PR-2).
    id_prefix: str = 'trk-'
    class_map: Dict[str, str] = field(
        default_factory=lambda: dict(DEFAULT_CLASS_MAP))
    birth_classes: Tuple[str, ...] = DEFAULT_BIRTH_CLASSES

    def __post_init__(self) -> None:
        """Проверить согласованность параметров."""
        _require(self.gate_m > 0 and self.reacquire_gate_m > 0,
                 'gate_m и reacquire_gate_m должны быть > 0')
        _require(self.confirm_hits >= 1, 'confirm_hits >= 1')
        _require(0 < self.tentative_timeout_sec,
                 'tentative_timeout_sec > 0')
        _require(0 < self.lost_after_sec < self.delete_after_sec,
                 '0 < lost_after_sec < delete_after_sec')
        _require(self.resolution_m >= 0, 'resolution_m >= 0')
        _require(self.max_tracks >= 1, 'max_tracks >= 1')
        _require(self.meas_std_m > 0 and self.init_vel_std > 0,
                 'meas_std_m и init_vel_std должны быть > 0')
        _require(self.accel_noise >= 0, 'accel_noise >= 0')
        _require(self.candidate_match_bonus_m >= 0
                 and self.candidate_mismatch_penalty_m >= 0,
                 'бонус и штраф candidate_id >= 0')
        unknown = [c for c in self.birth_classes if c not in self.class_map]
        _require(not unknown,
                 f'birth_classes вне class_map: {unknown}')
