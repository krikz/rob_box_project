"""Unit-тесты ``OdomMotionTracker`` (чистая логика, без rclpy).

Импорт модуля под rclpy/audio_common_msgs заведомо падает на dev-env
(см. test_quest_bridge.py), поэтому весь тест держится на внутренних
классах из ``quest_node``. Если не вышло — пропускаем как на роботе
тестируется целиком.
"""

from __future__ import annotations

import pytest


def _try_import():
    """Ленивый импорт: внутри quest_node.py тянет rclpy + geometry_msgs.
    На dev-env падает с ImportError → тесты skip'аются."""
    try:
        from rob_box_quest.quest_node import OdomMotionTracker
    except ImportError as exc:  # noqa: BLE001
        pytest.skip(f"OdomMotionTracker требует rclpy/geometry_msgs: {exc}")
    return OdomMotionTracker


def test_initial_state_no_motion_yet():
    OdomMotionTracker = _try_import()
    t = OdomMotionTracker()
    assert t.seconds_since_last_motion(100.0) is None


def test_first_update_returns_zero_and_seeds_position():
    OdomMotionTracker = _try_import()
    t = OdomMotionTracker()
    out = t.update(0.0, 0.0, 1.0)
    assert out == 0.0
    assert t.seconds_since_last_motion(1.0) == 0.0


def test_static_position_accumulates_seconds():
    OdomMotionTracker = _try_import()
    t = OdomMotionTracker()
    t.update(1.0, 2.0, 0.0)
    # Никто не двигается; на тике t=5.0 прошло 5 секунд.
    assert t.seconds_since_last_motion(5.0) == pytest.approx(5.0, abs=1e-9)
    # update с той же позицией — счётчик продолжает.
    assert t.update(1.0, 2.0, 7.0) == pytest.approx(7.0, abs=1e-9)


def test_motion_resets_counter():
    OdomMotionTracker = _try_import()
    t = OdomMotionTracker()
    t.update(0.0, 0.0, 0.0)
    assert t.seconds_since_last_motion(10.0) == 10.0
    # Поехали на 1 м → счётчик сбросился.
    t.update(1.0, 0.0, 10.0)
    assert t.seconds_since_last_motion(10.5) == pytest.approx(0.5, abs=1e-9)


def test_motion_below_eps_does_not_reset():
    OdomMotionTracker = _try_import()
    t = OdomMotionTracker(eps_m=0.05)  # 5 см
    t.update(0.0, 0.0, 0.0)
    # Дрейф 1 см < eps → считаем статикой.
    assert t.update(0.01, 0.0, 5.0) == pytest.approx(5.0, abs=1e-9)
    # Дрейф 10 см > eps → движение, сброс.
    assert t.update(0.10, 0.0, 6.0) == pytest.approx(0.0, abs=1e-9)


def test_negative_elapsed_clamps_to_zero():
    OdomMotionTracker = _try_import()
    t = OdomMotionTracker()
    t.update(0.0, 0.0, 100.0)
    # now раньше last_motion → не может быть отрицательным.
    assert t.seconds_since_last_motion(99.999) == 0.0
