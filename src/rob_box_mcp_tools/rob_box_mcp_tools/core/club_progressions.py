"""club_progressions.py — банк прогрессий club по ладам и выбор имени.

Issue #3226 (карточка (в) umbrella #3223, ADR-0146). Банк был из 5
минорных прогрессий; теперь минор шире, плюс дорийский, фригийский и
мажор. Рендер честно умеет все четыре лада: лад club — это только набор
аккордов (сдвиг корня от тоники + качество ``m``/``M``); нота лида берётся
из пентатоники ТЕКУЩЕГО аккорда (chord-scale), бас — корень аккорда, пэд —
его трезвучие. Отдельных «шкал» рендер не использует, поэтому лад =
подходящая прогрессия, и ``scale`` обязан совпадать с ладом прогрессии
(:func:`progression_mode`).

Имя прогрессии несёт лад префиксом: ``dor:``, ``phr:``, ``maj:``; без
префикса — минор (первые 5 — как были, порядок и состав не менять: от
них зависит seed-выбор без истории, снимок seed=0 и sha-тесты #3224).

Модуль чистый, без ROS/Renardo.
"""

from __future__ import annotations

import random
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from .music_diversity import weighted_pick

__all__ = [
    "LEGACY_PROGRESSIONS",
    "PROGRESSIONS",
    "SUPPORTED_SCALES",
    "Chord",
    "pick_progression_name",
    "progression_mode",
    "progressions_for",
]

Chord = Tuple[int, str]  # (сдвиг корня от тоники в полутонах, "m"|"M")

#: Лады club. Первый — эталонный.
SUPPORTED_SCALES: Tuple[str, ...] = ("minor", "dorian", "phrygian", "major")

_PREFIX_MODES: Dict[str, str] = {"dor": "dorian", "phr": "phrygian", "maj": "major"}

#: Банк минорных прогрессий, зафиксированный ДО #3226 (4 аккорда по 2 такта).
#: Seed-выбор без истории — ``randrange`` по ЭТОМУ кортежу, менять нельзя.
LEGACY_PROGRESSIONS: Tuple[Tuple[str, Tuple[Chord, ...]], ...] = (
    ("VI-III-VII-i", ((8, "M"), (3, "M"), (10, "M"), (0, "m"))),  # By Design
    ("i-VI-III-VII", ((0, "m"), (8, "M"), (3, "M"), (10, "M"))),
    ("i-VII-VI-VII", ((0, "m"), (10, "M"), (8, "M"), (10, "M"))),
    ("i-iv-VI-v", ((0, "m"), (5, "m"), (8, "M"), (7, "m"))),
    ("VI-VII-i-i", ((8, "M"), (10, "M"), (0, "m"), (0, "m"))),
)

_EXTRA_PROGRESSIONS: Tuple[Tuple[str, Tuple[Chord, ...]], ...] = (
    # минор (натуральный)
    ("i-VII-VI-v", ((0, "m"), (10, "M"), (8, "M"), (7, "m"))),
    ("i-III-VII-VI", ((0, "m"), (3, "M"), (10, "M"), (8, "M"))),
    ("i-iv-VII-III", ((0, "m"), (5, "m"), (10, "M"), (3, "M"))),
    ("i-v-VI-iv", ((0, "m"), (7, "m"), (8, "M"), (5, "m"))),
    ("VI-VII-i-v", ((8, "M"), (10, "M"), (0, "m"), (7, "m"))),
    ("i-III-iv-VII", ((0, "m"), (3, "M"), (5, "m"), (10, "M"))),
    # дорийский: минор с мажорной IV
    ("dor:i-IV-i-VII", ((0, "m"), (5, "M"), (0, "m"), (10, "M"))),
    ("dor:i-ii-IV-i", ((0, "m"), (2, "m"), (5, "M"), (0, "m"))),
    ("dor:IV-VII-i-i", ((5, "M"), (10, "M"), (0, "m"), (0, "m"))),
    ("dor:i-VII-IV-VII", ((0, "m"), (10, "M"), (5, "M"), (10, "M"))),
    # фригийский: минор с мажорной II (b2)
    ("phr:i-II-i-VII", ((0, "m"), (1, "M"), (0, "m"), (10, "M"))),
    ("phr:i-II-VII-i", ((0, "m"), (1, "M"), (10, "M"), (0, "m"))),
    ("phr:i-VII-VI-II", ((0, "m"), (10, "M"), (8, "M"), (1, "M"))),
    ("phr:i-iv-II-i", ((0, "m"), (5, "m"), (1, "M"), (0, "m"))),
    # мажор (ионийский)
    ("maj:I-V-vi-IV", ((0, "M"), (7, "M"), (9, "m"), (5, "M"))),
    ("maj:vi-IV-I-V", ((9, "m"), (5, "M"), (0, "M"), (7, "M"))),
    ("maj:I-vi-IV-V", ((0, "M"), (9, "m"), (5, "M"), (7, "M"))),
    ("maj:I-IV-vi-V", ((0, "M"), (5, "M"), (9, "m"), (7, "M"))),
)

#: Весь банк: старые 5 первыми, дальше добавленные.
PROGRESSIONS: Tuple[Tuple[str, Tuple[Chord, ...]], ...] = LEGACY_PROGRESSIONS + _EXTRA_PROGRESSIONS


def progression_mode(name: str) -> str:
    """Лад прогрессии по имени (префикс ``dor:``/``phr:``/``maj:``, иначе minor)."""
    return _PREFIX_MODES.get(name.partition(":")[0], "minor") if ":" in name else "minor"


def progressions_for(scale: str) -> List[str]:
    """Имена прогрессий лада ``scale`` в порядке банка."""
    return [name for name, _ in PROGRESSIONS if progression_mode(name) == scale]


def pick_progression_name(
    seed: int = 0, recent: Optional[Sequence[Mapping[str, Any]]] = None, scale: str = "minor",
) -> str:
    """Имя прогрессии трека в ладе ``scale``.

    * minor, без истории или ``seed=0`` — прежний выбор ``random.Random(seed)``
      по :data:`LEGACY_PROGRESSIONS` (байты как до #3226);
    * с историей — :func:`weighted_pick` по всему пулу лада со штрафом за
      недавнее (отдельный ГСЧ ``club-prog:<seed>``, поток рифа не трогает);
    * не-minor без истории — детерминированно по сиду из пула лада
      (``seed=0`` — первая прогрессия лада).
    """
    if scale == "minor" and (seed == 0 or not recent):
        return LEGACY_PROGRESSIONS[random.Random(seed).randrange(len(LEGACY_PROGRESSIONS))][0]
    pool = progressions_for(scale)
    if not pool:
        raise ValueError(f"Лад {scale!r} не поддержан клубным режимом (допустимо: {', '.join(SUPPORTED_SCALES)})")
    if seed == 0:
        return pool[0]
    rng = random.Random(f"club-prog:{seed}")
    if not recent:
        return pool[rng.randrange(len(pool))]
    return weighted_pick(pool, [r.get("progression") for r in recent], rng)
