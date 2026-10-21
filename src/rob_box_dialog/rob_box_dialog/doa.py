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

#: Сколько секунд держать отсчёты DOA: дольше самой длинной фразы с запасом на STT.
HISTORY_S = 60.0


def circular_mean_deg(values) -> Optional[float]:
    """Среднее направление в градусах [0, 360); ``None`` — отсчётов нет или они взаимно гасятся."""
    sin = cos = 0.0
    n = 0
    for deg in values:
        sin += math.sin(math.radians(deg))
        cos += math.cos(math.radians(deg))
        n += 1
    if n == 0 or math.hypot(sin, cos) < 1e-9:
        return None
    return round(math.degrees(math.atan2(sin, cos)), 6) % 360.0


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
        """Направление фразы, звучавшей в ``[start, end]``; ``None`` — речи с направлением в окне не было."""
        return circular_mean_deg(deg for t, deg in self._samples if start <= t <= end)
