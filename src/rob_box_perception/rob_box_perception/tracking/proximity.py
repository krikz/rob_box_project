"""Близость треков: стоимость пары, лицо к телу, сближение.

ADR-0161 §2 (лицо к телу того же момента), ADR-0130 §6 (сближение).

Чистые функции над треками трекера (утиная типизация: ``track_id``,
``class_name``, ``candidate_id``, ``last_body``, ``last_attached``,
``filt.position``). Вызывающий передаёт только активные треки
(не ``lost``).
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from rob_box_perception.tracking.params import TrackerParams


def distance(a: Any, b: Any) -> float:
    """Расстояние между оценками положения двух треков, м."""
    ax, ay = a.filt.position
    bx, by = b.filt.position
    return math.hypot(ax - bx, ay - by)


def match_cost(track: Any, det: Any, gate: float,
               params: TrackerParams) -> Optional[float]:
    """Стоимость пары трек–детекция, м; ``None`` — пара запрещена.

    Другой класс или дальше ``gate`` — запрещена. Совпавший
    ``candidate_id`` даёт скидку, разный непустой — штраф (улика без имени,
    ADR-0130 §5.1 п.1).
    """
    if params.class_map[det.class_name] != track.class_name:
        return None
    x, y = track.filt.position
    dist = math.hypot(det.x - x, det.y - y)
    if dist > gate:
        return None
    if track.candidate_id and det.candidate_id:
        if track.candidate_id == det.candidate_id:
            return max(0.0, dist - params.candidate_match_bonus_m)
        return dist + params.candidate_mismatch_penalty_m
    return dist


def attach_target(active: Sequence[Any], det: Any, stamp: float,
                  params: TrackerParams) -> Optional[Any]:
    """Ближайший трек тела того же момента, к которому прикрепить лицо.

    Трек годится, если: класс совпадает, тело было не дальше
    ``attach_window_sec`` от ``stamp``, лица в этот момент у трека ещё
    не было, до трека не дальше ``attach_m``, ``candidate_id`` не
    противоречит. ``None`` — такого трека нет, лицо рождает свой.
    """
    best: Optional[Tuple[float, Any]] = None
    for track in active:
        dist = _attach_dist(track, det, stamp, params)
        if dist is not None and (best is None or dist < best[0]):
            best = (dist, track)
    return None if best is None else best[1]


def _attach_dist(track: Any, det: Any, stamp: float,
                 params: TrackerParams) -> Optional[float]:
    """Расстояние до трека, если к нему можно прикрепить лицо."""
    if (track.last_body is None or track.last_attached == stamp
            or params.class_map[det.class_name] != track.class_name
            or abs(stamp - track.last_body) > params.attach_window_sec):
        return None
    if (track.candidate_id and det.candidate_id
            and track.candidate_id != det.candidate_id):
        return None
    x, y = track.filt.position
    dist = math.hypot(det.x - x, det.y - y)
    return dist if dist <= params.attach_m else None


def close_neighbours(active: Sequence[Any], resolution_m: float
                     ) -> Dict[str, Tuple[str, ...]]:
    """Для каждого трека — id треков того же класса ближе ``resolution_m``."""
    close: Dict[str, List[str]] = {t.track_id: [] for t in active}
    for i, a in enumerate(active):
        for b in active[i + 1:]:
            if a.class_name == b.class_name and \
                    distance(a, b) < resolution_m:
                close[a.track_id].append(b.track_id)
                close[b.track_id].append(a.track_id)
    return {k: tuple(v) for k, v in close.items()}
