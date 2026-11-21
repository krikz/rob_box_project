"""Направление на говорящего по фразе (ADR-0150 §7.3 В12, §6.2): DOA ReSpeaker в момент речи.

``/audio/direction`` (Int32, градусы) audio_node публикует постоянно, и у телевизора тоже есть направление —
поэтому в историю идут только отсчёты, пока ``/audio/vad`` говорит «речь». Фраза доезжает до STT уже записанной
целиком, так что направление берётся по окну её звучания (:meth:`DoaTrack.during`), а не «последнее на сейчас».
Чистый модуль: время передаёт вызывающий.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Optional, Tuple

from rob_box_dialog.knowledge import DOA_CONFIDENCE_MIN

#: Сколько секунд держать отсчёты DOA: дольше самой длинной фразы с запасом на STT.
HISTORY_S = 60.0
#: Ниже этого модуля векторы считаются взаимно погашенными (численный ноль, не порог уверенности).
_CANCELLED_R = 1e-9


def circular_mean_deg(values) -> Tuple[Optional[float], float]:
    """Круговое среднее: ``(угол [0, 360), R)``.

    ``R`` — длина результирующей, 0…1: согласованные отсчёты дают R≈1, разброс ~180° гасит вектор (R≈0).
    Угла нет (``None``, R=0) — отсчётов нет или они взаимно погашены. Порог «верить / не верить» — не здесь,
    а :data:`rob_box_dialog.knowledge.DOA_CONFIDENCE_MIN`.
    """
    sin = cos = 0.0
    n = 0
    for deg in values:
        sin += math.sin(math.radians(deg))
        cos += math.cos(math.radians(deg))
        n += 1
    if n == 0:
        return None, 0.0
    resultant = math.hypot(sin, cos) / n
    if resultant < _CANCELLED_R:
        return None, 0.0
    return round(math.degrees(math.atan2(sin, cos)), 6) % 360.0, resultant


def angle_diff_deg(a: float, b: float) -> float:
    """Наименьший угол между направлениями, 0…180."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


class DoaTrack:
    """Отсчёты DOA, снятые во время речи (VAD), и направление фразы по окну её звучания."""

    def __init__(self, history_s: float = HISTORY_S) -> None:
        self.history_s = history_s
        self._vad = False
        self._samples: Deque[Tuple[float, float]] = deque()

    def on_vad(self, active: bool, now: float) -> None:
        self._vad = bool(active)

    def on_direction(self, deg: float, now: float) -> None:
        if self._vad:
            self._samples.append((now, float(deg) % 360.0))
        while self._samples and now - self._samples[0][0] > self.history_s:
            self._samples.popleft()

    def during(self, start: float, end: float) -> Optional[float]:
        """Направление фразы в ``[start, end]``, если уверенность не ниже порога; иначе ``None``.

        ``None`` — в окне не было речи с направлением, либо отсчёты взаимно гасятся (R <
        :data:`rob_box_dialog.knowledge.DOA_CONFIDENCE_MIN`): потребитель не крутится на шум.
        """
        angle, confidence = circular_mean_deg(deg for t, deg in self._samples if start <= t <= end)
        if angle is None or confidence < DOA_CONFIDENCE_MIN:
            return None
        return angle
