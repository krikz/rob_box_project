"""Фрейм трекера: ``map``, если есть, иначе ``base_link`` (ADR-0161 §1).

Контракт:
  1. Первый пакет выбирает фрейм: ``map`` при ``map_ok``, иначе ``base_link``.
  2. ``base_link`` → ``map`` только после ``map_confirm_batches`` пакетов
     подряд с ``map``; разрыв серии начинает счёт заново.
  3. ``map`` → ``base_link`` только когда ``map`` нет дольше ``map_loss_sec``;
     короткая потеря фрейм не меняет.
  4. Смена объявляется ровно один раз — на пакете смены.

Запуск:
    python -m pytest src/rob_box_perception/test/unit/tracking -v --no-cov
"""

from __future__ import annotations

from rob_box_perception.tracking import FramePolicy, TrackerParams
from rob_box_perception.tracking.frame_policy import BASE_LINK, MAP


def feed(policy, seq, dt=0.1):
    return [policy.observe(k * dt, ok) for k, ok in enumerate(seq)]


def test_first_batch_picks_map_when_available():
    assert feed(FramePolicy(), [True]) == [MAP]


def test_first_batch_picks_base_link_without_map():
    """Шифу 04.11: нет map — «следить относительно себя»."""
    policy = FramePolicy()
    assert feed(policy, [False, False]) == [BASE_LINK, None]
    assert policy.frame == BASE_LINK


def test_switch_to_map_needs_consecutive_batches():
    policy = FramePolicy(TrackerParams(map_confirm_batches=3))
    out = feed(policy, [False, True, True, False, True, True, True, True])
    assert out == [BASE_LINK, None, None, None, None, None, MAP, None]


def test_short_map_loss_keeps_map():
    policy = FramePolicy(TrackerParams(map_loss_sec=2.0))
    out = feed(policy, [True] + [False] * 15 + [True], dt=0.1)
    assert out == [MAP] + [None] * 16
    assert policy.frame == MAP


def test_long_map_loss_falls_back_to_base_link_once():
    policy = FramePolicy(TrackerParams(map_loss_sec=1.0))
    out = feed(policy, [True] + [False] * 8, dt=0.25)
    # Потеря с t=0.25; на t=1.25 прошло ровно 1.0 с (ещё map), на t=1.5 — 1.25 с.
    assert out.index(BASE_LINK) == 6
    assert out.count(BASE_LINK) == 1
    assert policy.frame == BASE_LINK
