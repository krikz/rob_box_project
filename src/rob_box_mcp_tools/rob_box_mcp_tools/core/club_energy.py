"""club_energy.py — уровень энергии club-трека в DJ-сете (issue #3311, ADR-0147).

Живой замер 01.10 (#3311): громкость DJ-сета по минутам почти не меняется —
размах 0.3–3.4 dB в 6-минутном сете против 8.9 dB у живого диджея. Каждый
трек калибруется к одной цели (:mod:`core.club_loudness`), а выравниватель
мастер-шины (``masterfilter.scd``, ``lvlRatio 3``) сжимает оставшуюся
разницу ещё втрое.

Энергия ``1..5`` (1 — интро/спад, 5 — пик сета) задаёт три оси трека:

* **громкость** — :data:`ENERGY_TRIM_DB`, смещение ПОСЛЕ радио-динамики
  мастер-шины (выравниватель его не съедает, лимитер его не видит). Пик
  сета (5) — 0 dB, то есть нынешний уровень; остальные только тише, поэтому
  пик выхода не выше нынешнего (−1 dBFS до фейдера, −7 dBFS при gain 0.5).
  В этом модуле — только таблица; проводка до ``masterfilter`` — следующий
  шаг ADR-0147 (§5, шаг 2);
* **плотность** — :data:`ENERGY_DENSITY`, множители 0..1 к откалиброванным
  уровням слоёв (та же ручка ``levels``, что у ``render_club_kit``: только
  тише, калибровка #3154 остаётся источником истины);
* **фильтр** — :data:`ENERGY_LPF_SCALE`, множитель частот ``lpf``-развёрток
  лида и баса (низкая энергия — темнее).

``energy=None`` — побайтно прежний трек (энергия не задана). Числа таблиц —
стартовая гипотеза дизайна, НЕ замер: приёмка — живой ``compare.py`` (ADR-0147 §6).

Модуль чистый, без ROS/Renardo.
"""

from __future__ import annotations

import re
from typing import Dict, Mapping, Optional, Tuple

from rob_box_music import knowledge as kn

__all__ = [
    "ENERGY_DENSITY",
    "ENERGY_LEVELS",
    "ENERGY_LPF_MIN_HZ",
    "ENERGY_LPF_SCALE",
    "ENERGY_TRIM_DB",
    "apply_energy",
    "energy_levels",
    "energy_lpf",
    "energy_note",
    "energy_trim_db",
    "validate_energy",
]

#: Уровни энергии (1 — интро/спад, 5 — пик) и смещение громкости ПОСЛЕ динамики мастер-шины, dB
#: (размах 9 dB — медиана 6-мин окон эталона, только ≤ 0) — одна таблица в ``rob_box_music.knowledge``
#: (ADR-0149 §4.6 в, PR-3b); здесь импорт до удаления старого пути (PR-7).
ENERGY_LEVELS = kn.ENERGY_LEVELS
ENERGY_TRIM_DB = kn.ENERGY_TRIM_DB

#: Плотность: множители 0..1 к откалиброванным уровням слоёв. Низкая
#: энергия — без клэпа, реже хэты, тише лид; бочка и бас держат ритм
#: (без бочки выравниватель вытянул бы пэд на +15 dB). 4 и 5 — полный микс.
ENERGY_DENSITY: Dict[int, Dict[str, float]] = {
    1: {"clap": 0.0, "hats": 0.5, "lead": 0.7},
    2: {"clap": 0.5, "hats": 0.7, "lead": 0.85},
    3: {"hats": 0.85},
    4: {},
    5: {},
}

#: Фильтр: множитель частот ``lpf``-развёрток лида и баса (``linvar([lo, hi], n)``).
ENERGY_LPF_SCALE: Dict[int, float] = {1: 0.45, 2: 0.6, 3: 0.8, 4: 1.0, 5: 1.0}

#: Нижний предел частоты среза после множителя (Гц): бас не глохнет совсем.
ENERGY_LPF_MIN_HZ = 250

_LINVAR_RANGE = re.compile(r"linvar\(\[([0-9]+), ([0-9]+)\]")


def validate_energy(energy: object) -> int:
    """Уровень энергии как ``int`` из :data:`ENERGY_LEVELS` или ``ValueError``."""
    if isinstance(energy, bool) or not isinstance(energy, int) or energy not in ENERGY_LEVELS:
        raise ValueError(f"energy={energy!r} вне 1..5 (1 — интро/спад, 5 — пик сета)")
    return energy


def energy_trim_db(energy: int) -> float:
    """Смещение громкости трека после динамики мастер-шины (dB, ≤ 0)."""
    return ENERGY_TRIM_DB[validate_energy(energy)]


def energy_levels(energy: int, levels: Optional[Mapping[str, float]] = None) -> Optional[Mapping[str, float]]:
    """Множители слоёв: плотность энергии × явные ``levels`` (0..1).

    Без плотности (энергия 4–5) возвращает ``levels`` как есть, в том числе ``None``.
    """
    density = ENERGY_DENSITY[validate_energy(energy)]
    if not density:
        return levels
    merged: Dict[str, float] = dict(density)
    for lane, factor in (levels or {}).items():
        merged[lane] = merged.get(lane, 1.0) * factor
    return merged


def energy_lpf(expr: str, energy: int) -> str:
    """``lpf``-развёртка с частотами × :data:`ENERGY_LPF_SCALE` (не ниже :data:`ENERGY_LPF_MIN_HZ`)."""
    scale = ENERGY_LPF_SCALE[validate_energy(energy)]
    if scale == 1.0:
        return expr

    def _scaled(match: "re.Match[str]") -> str:
        lo, hi = (max(ENERGY_LPF_MIN_HZ, round(int(v) * scale)) for v in match.groups())
        return f"linvar([{lo}, {hi}]"

    return _LINVAR_RANGE.sub(_scaled, expr)


def apply_energy(
    energy: Optional[int], lpf: Tuple[str, str], levels: Optional[Mapping[str, float]],
) -> Tuple[Tuple[str, str], Optional[Mapping[str, float]]]:
    """``(lpf лида и баса, множители слоёв)`` с учётом энергии; ``None`` — без изменений."""
    if energy is None:
        return lpf, levels
    return (energy_lpf(lpf[0], energy), energy_lpf(lpf[1], energy)), energy_levels(energy, levels)


def energy_note(energy: Optional[int]) -> str:
    """Хвост строки-заголовка трека: ``", энергия N"``; ``None`` — пусто (байты прежние)."""
    return "" if energy is None else f", энергия {validate_energy(energy)}"
