"""Закрытый список классов слоя карты (ADR-0169 §1.3, #4369).

Не выдумывать мебель: шкаф/полка/дверь в список не входят.
"""

from __future__ import annotations

from rob_box_core.object_classes import (
    FAST_STALE_CLASSES,
    STATIC_INANIMATE_CLASSES,
    is_static_inanimate,
    stale_misses_for,
)

# ADR-0169 §1.3 дословно.
_NAMED = (
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


def test_named_classes_match_adr_0169_section_1_3():
    assert STATIC_INANIMATE_CLASSES == _NAMED


def test_living_and_carried_and_invented_are_out():
    for name in ("person", "face", "cat", "dog", "cup", "bottle", "laptop", "wardrobe", "shelf", "door", "wall"):
        assert not is_static_inanimate(name)


def test_each_named_class_is_accepted():
    assert all(is_static_inanimate(name) for name in STATIC_INANIMATE_CLASSES)


def test_chair_stales_faster_than_couch():
    assert FAST_STALE_CLASSES == frozenset({"chair"})
    assert stale_misses_for("chair") == 1
    assert stale_misses_for("couch") == 3
    assert stale_misses_for("bed") == 3
