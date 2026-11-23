"""Пороги стиля вайба живут в ``MUSIC_SCENE_BPM``, не своими числами (#5140).

``_music_style`` берёт первые границы таблицы (последняя — ``inf``, порогом не служит). Сдвиг таблицы
сдвигает и переключения: дубля 100/125 в функции больше нет.
"""

from __future__ import annotations

import math

import pytest
from rob_box_animations.face_director import _music_style

from rob_box_animations import face_director as fd
from rob_box_dialog.knowledge import MUSIC_DEFAULT_BPM, MUSIC_SCENE_BPM


def test_live_table_keeps_the_old_cuts():
    """Живая таблица не сдвинута: <100 plasma, [100,125) rainbow, >=125 fire. Середина 118 — rainbow."""
    assert [limit for limit, _ in MUSIC_SCENE_BPM[:-1]] == [100.0, 125.0]
    assert MUSIC_DEFAULT_BPM == 118.0
    assert _music_style(99.9) == "plasma"
    assert _music_style(100.0) == "rainbow"
    assert _music_style(124.9) == "rainbow"
    assert _music_style(125.0) == "fire"
    for empty in (None, 0, 0.0, -5, -0.1):
        assert _music_style(empty) == "rainbow"


def test_style_follows_the_table_not_the_old_numbers(monkeypatch):
    """Таблица на 80/90 — переключения там же. 100 и 125 больше ничего не режут."""
    monkeypatch.setattr(fd, "MUSIC_SCENE_BPM", ((80.0, "a"), (90.0, "b"), (math.inf, "c")))
    assert _music_style(79.9) == "plasma"
    assert _music_style(80.0) == "rainbow"
    assert _music_style(89.9) == "rainbow"
    assert _music_style(90.0) == "fire"
    # Старые числа лежат выше обеих границ: стиль уже fire, разреза на 100/125 нет.
    assert _music_style(99.9) == "fire"
    assert _music_style(100.0) == "fire"
    assert _music_style(125.0) == "fire"


@pytest.mark.parametrize("bpm", [None, 0, 0.0, -5])
def test_empty_tempo_uses_the_default_against_the_patched_table(bpm, monkeypatch):
    """Пустой темп — ``MUSIC_DEFAULT_BPM`` (118), против таблицы 80/90 это уже fire, не rainbow."""
    monkeypatch.setattr(fd, "MUSIC_SCENE_BPM", ((80.0, "a"), (90.0, "b"), (math.inf, "c")))
    assert _music_style(bpm) == "fire"
