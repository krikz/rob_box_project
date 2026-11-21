"""Направление на говорящего по фразе (ADR-0150 §7.3 В12, §6.2): DOA ReSpeaker в момент речи.

``/audio/direction`` (Int32, градусы) audio_node публикует постоянно, и у телевизора тоже есть направление —
поэтому в историю идут только отсчёты, пока ``/audio/vad`` говорит «речь». Фраза доезжает до STT уже записанной
целиком, так что направление берётся по окну её звучания (:meth:`DoaTrack.during`), а не «последнее на сейчас».
Чистый модуль: время передаёт вызывающий.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

from rob_box_dialog.knowledge import DOA_CONFIDENCE_MIN

#: Сколько секунд держать отсчёты DOA: дольше самой длинной фразы с запасом на STT.
HISTORY_S = 60.0
#: Ниже этого модуля векторы считаются взаимно погашенными (численный ноль, не порог уверенности).
_CANCELLED_R = 1e-9


@dataclass(frozen=True)
class DoaReading:
    """Направление фразы и его уверенность наружу (#3784).

    ``doa_deg`` — круговое среднее отсчётов окна, градусы [0, 360); ``None`` — отсчётов не было или они
    взаимно погашены (R≈0, угла нет). ``doa_conf`` — длина результирующей R, 0…1. ``reliable`` — верить ли
    углу: решает единственный порог :data:`rob_box_dialog.knowledge.DOA_CONFIDENCE_MIN`, второго числа нет.
    ``usable_deg`` — угол, который потребителю можно применять; ниже порога ``None``, и это видно по
    ``reliable is False``, а не по пропавшему углу.
    """

    doa_deg: Optional[float]
    doa_conf: float
    reliable: bool

    @property
    def usable_deg(self) -> Optional[float]:
        return self.doa_deg if self.reliable else None


def phrase_direction(angle: Optional[float], confidence: float) -> DoaReading:
    """Собрать чтение фразы: порог «верить / не верить» применяется здесь и только здесь."""
    conf = float(confidence)
    reliable = angle is not None and conf >= DOA_CONFIDENCE_MIN
    return DoaReading(doa_deg=angle, doa_conf=conf, reliable=reliable)


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

    def during(self, start: float, end: float) -> DoaReading:
        """Направление фразы в ``[start, end]``: угол, уверенность и признак «достоверно».

        Угол ниже порога не пропадает молча: ``doa_deg`` остаётся посчитанным, ``reliable`` — ``False``,
        ``usable_deg`` — ``None``. Потребитель применяет только ``usable_deg`` и видит, почему угла нет.
        """
        angle, confidence = circular_mean_deg(deg for t, deg in self._samples if start <= t <= end)
        return phrase_direction(angle, confidence)
