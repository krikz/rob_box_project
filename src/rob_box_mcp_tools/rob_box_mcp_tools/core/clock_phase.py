"""Фаза клока Renardo относительно формы трека (issue #3112).

Renardo считает фазу ``var``/``linvar`` и индекс нот плеера от доли 0 клока
(``TimeVar.get_current_index``: ``time - self.start_time``, renardo_lib
0.9.13 ``TimeVar.py:150``; ``Player.count``: ``acc = now - (now % total_dur)``,
``Players.py:728``), а ``Clock.clear()`` долю не сбрасывает
(``TempoClock.py:686``). Плееры встают на ``Clock.next_bar()``
(``Players.py:892``). Поэтому трек, запущенный на давно идущем клоке,
начинается с позиции ``next_bar mod длина_формы``, а не с интро.

Модуль чистый: без окружения и без импорта Renardo.

* :func:`clock_phase_snapshot` — диагностика (включена всегда): по объекту
  клока (duck-typing: ``now()``, ``meter``) считает, с какой доли встанут
  плееры и какое это смещение внутри формы.
* :func:`clock_align_prelude` (определена в ``core/arranger`` — тот
  грузится ``gen_tool_catalog`` без пакета; здесь реэкспорт) — кандидат
  фикса (только за флагом): строка ``Clock.set_time(...)``, которую
  аранжировщики вставляют сразу после ``Clock.clear()``. Обоснование — ``docs/design/2026-09-28-music-clock-phase-3112.md``.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

from .arranger import ALIGN_LEAD_BEATS, clock_align_prelude


def _next_bar(beat: float, bar: float) -> float:
    """Та же арифметика, что ``TempoClock.next_bar`` (``TempoClock.py:648``)."""
    return beat + (bar - (beat % bar))


def clock_phase_snapshot(clock: Any, form_total_beats: int) -> Optional[Dict[str, float]]:
    """Где внутри формы встанут плееры, если их запланировать сейчас.

    Returns:
        ``{"clock_beat", "start_beat", "form_total_beats",
        "phase_offset_beats"}`` или ``None``, если клок недоступен/сломан —
        диагностика не имеет права ронять музыку.
    """
    try:
        total = float(form_total_beats)
        if clock is None or total <= 0:
            return None
        beat = float(clock.now())
        bar = float(clock.meter[0])
        if not (math.isfinite(beat) and bar > 0):
            return None
        start = _next_bar(beat, bar)
        return {
            "clock_beat": round(beat, 3),
            "start_beat": round(start, 3),
            "form_total_beats": total,
            "phase_offset_beats": round(start % total, 3),
        }
    except Exception:  # noqa: BLE001 — диагностика не ломает музыку
        return None


__all__ = [
    "ALIGN_LEAD_BEATS",
    "clock_align_prelude",
    "clock_phase_snapshot",
]
