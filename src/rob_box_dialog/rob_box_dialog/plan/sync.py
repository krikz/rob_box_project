"""Старт речи на такте подложки (ADR-0150 §5.6.3 «Старт на такте», X2; Ableton Link quantized launch).

Сетка — из события плеера ``started{bpm, beat_at, bar_beats, start_beat}``: ``beat_at`` — момент, когда доля
старта трека **звучит** (часы ROS = стенные часы процесса; :func:`beat_heard_at` — единственная формула). Время
старта реплики ``SpeechRequest.at`` — тоже момент звучания: tts_node сам вычитает ``output_latency_ms`` (PR-3,
``speech_timing.at_delay_s``). Поэтому граница такта сравнивается с самым ранним моментом, когда реплика вообще
может прозвучать: ``now + output_latency + PERFORM_COMMIT_LEAD_S`` (доставка запроса до ожидания ``at``).

Подложки нет (или она кончилась) — сетки нет: реплика идёт сразу за предыдущей (``at = 0``), и это честно отражается
в ``PlanEvent.synced = false``. Чистый Python, без часов: время передаёт вызывающий.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn

#: Граница такта ближе этого к самому раннему старту ещё годится (погрешность арифметики с плавающей точкой).
_EPS_S = 1e-6


def beat_heard_at(ts: float, late_beats: float, bpm: float, latency_s: float) -> float:
    """Когда доля старта трека звучит: колбэк клока Renardo пришёл в ``ts`` на ``late_beats`` позже своей доли, а
    ноты Renardo звучат на ``Clock.latency`` (``latency_s``) позже доли клока (ADR-0149 §4.3, ``renardo_adapter``).
    Задержку scsynth → JACK → ReSpeaker формула не знает — её покажет замер A19 (спайк PR-S1)."""
    return float(ts) - float(late_beats) * 60.0 / float(bpm) + float(latency_s)


@dataclass(frozen=True)
class BeatGrid:
    """Сетка тактов подложки: темп, момент звучания доли старта, длина такта в долях, доля старта на клоке."""

    bpm: float
    beat_at: float
    bar_beats: float
    start_beat: float = 0.0

    def __post_init__(self) -> None:
        if self.bpm <= 0 or self.bar_beats <= 0:
            raise ValueError(f"BeatGrid: bpm={self.bpm}, bar_beats={self.bar_beats}")

    @property
    def bar_s(self) -> float:
        """Длина такта, секунды."""
        return self.bar_beats * 60.0 / self.bpm

    def beat_time(self, beat: float) -> float:
        """Когда звучит доля ``beat`` клока плеера (``form_end_beat`` из ``nearly_finished``)."""
        return self.beat_at + (float(beat) - self.start_beat) * 60.0 / self.bpm


@dataclass(frozen=True)
class Slot:
    """Момент старта реплики (``SpeechRequest.at``; 0 — сразу за предыдущей) и попал ли он на такт."""

    at: float
    synced: bool


def earliest(now: float, output_latency_ms: int) -> float:
    """Самый ранний момент, когда реплика, отправленная сейчас, может прозвучать."""
    return now + output_latency_ms / 1000.0 + kn.PERFORM_COMMIT_LEAD_S


def bar_at_or_after(grid: BeatGrid, t: float) -> float:
    """Граница такта в ``t`` или первая после него; раньше доли старта подложки тактов нет."""
    bars = max(0, math.ceil((t - grid.beat_at) / grid.bar_s - _EPS_S))
    return grid.beat_at + bars * grid.bar_s


def next_bar(grid: Optional[BeatGrid], now: float, output_latency_ms: int, after: float = 0.0) -> Slot:
    """Ближайшая граница такта не раньше самого раннего старта и не раньше ``after`` (конец предыдущего сегмента).
    Сетки нет — ``Slot(0, False)``: реплика идёт сразу за предыдущей."""
    if grid is None:
        return Slot(0.0, False)
    return Slot(bar_at_or_after(grid, max(earliest(now, output_latency_ms), after)), True)


def layout(durations: Sequence[float], grid: Optional[BeatGrid], first_at: float) -> Tuple[float, ...]:
    """Раскладка подряд идущих реплик: первая — в ``first_at``, каждая следующая — на ближайшем такте после конца
    предыдущей (без сетки — встык). Для ETA и длины подложки; при исполнении старт берётся по факту ``started``."""
    starts = []
    t = first_at
    for duration in durations:
        starts.append(t)
        t = t + duration if grid is None else bar_at_or_after(grid, t + duration)
    return tuple(starts)


__all__ = ["BeatGrid", "Slot", "bar_at_or_after", "beat_heard_at", "earliest", "layout", "next_bar"]
