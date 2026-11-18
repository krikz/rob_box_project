"""Классы профиля «неживое неподвижное» (ADR-0169 §1.3, ADR-0148).

Список закрытый: только имена из ADR-0169 §1.3. Шкаф, полка, дверь, стена
сюда не входят — их нет в COCO, Шифу на В4 ещё не ответил.
"""

from __future__ import annotations

from typing import FrozenSet, Tuple

# Порядок как в ADR-0169 §1.3.
STATIC_INANIMATE_CLASSES: Tuple[str, ...] = (
    "couch",
    "bed",
    "dining table",
    "refrigerator",
    "oven",
    "sink",
    "toilet",
    "tv",
    "bench",
    "potted plant",
    "microwave",
    "chair",
)

STATIC_INANIMATE_CLASS_SET: FrozenSet[str] = frozenset(STATIC_INANIMATE_CLASSES)

# ADR-0169 §2.3: стул двигают чаще — стареет с одного промаха.
FAST_STALE_CLASSES: FrozenSet[str] = frozenset({"chair"})

_STALE_MISSES_DEFAULT = 3
_STALE_MISSES_FAST = 1


def is_static_inanimate(class_name: str) -> bool:
    """Класс профиля слоя карты, не выдуманный и не живой."""
    return class_name in STATIC_INANIMATE_CLASS_SET


def stale_misses_for(class_name: str) -> int:
    """Сколько промахов при Ожидаемой видимости до stale (ADR-0169 §2.3)."""
    if class_name in FAST_STALE_CLASSES:
        return _STALE_MISSES_FAST
    return _STALE_MISSES_DEFAULT
