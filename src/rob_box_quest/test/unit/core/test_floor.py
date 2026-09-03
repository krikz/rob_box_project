"""Unit-тесты SupervisorFloorTracker (core/floor.py).

Покрывает:
- acquire free / занято чужим / идемпотентно свой;
- release — освобождает, чужой release no-op;
- rate-limit ERROR{FLOOR_HELD} с fake-clock (1 с интервал);
- force_release для reset/shutdown;
- is_held_by / holder.
"""

from __future__ import annotations

import pytest

from rob_box_quest.core.floor import (
    FLOOR_HELD_RATE_LIMIT_S,
    AcquireResult,
    SupervisorFloorTracker,
)


class _FakeClock:
    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def test_acquire_when_free_grants():
    clk = _FakeClock(0.0)
    tr = SupervisorFloorTracker(now_fn=clk)
    res = tr.acquire("client-a")
    assert res.granted is True
    assert res.held_by == "client-a"
    assert tr.holder == "client-a"
    assert tr.is_held_by("client-a") is True


def test_acquire_by_other_returns_held_by_other():
    clk = _FakeClock(0.0)
    tr = SupervisorFloorTracker(now_fn=clk)
    tr.acquire("client-a")
    res = tr.acquire("client-b")
    assert res.granted is False
    assert res.held_by == "client-a"
    assert tr.holder == "client-a"  # a не потерял


def test_acquire_is_idempotent_for_same_client():
    clk = _FakeClock(0.0)
    tr = SupervisorFloorTracker(now_fn=clk)
    tr.acquire("client-a")
    res = tr.acquire("client-a")
    assert res.granted is True
    assert res.held_by == "client-a"
    assert tr.holder == "client-a"


def test_acquire_empty_client_id_rejected():
    tr = SupervisorFloorTracker()
    res = tr.acquire("")
    assert res.granted is False
    assert "invalid_client_id" in res.reason
    assert tr.holder is None


def test_release_returns_true_for_holder():
    tr = SupervisorFloorTracker()
    tr.acquire("client-a")
    assert tr.release("client-a") is True
    assert tr.holder is None


def test_release_returns_false_for_non_holder():
    tr = SupervisorFloorTracker()
    tr.acquire("client-a")
    assert tr.release("client-b") is False
    assert tr.holder == "client-a"


def test_release_is_idempotent_for_non_holder():
    tr = SupervisorFloorTracker()
    assert tr.release("client-b") is False  # никогда не держал
    assert tr.release("client-b") is False


def test_after_release_other_can_acquire():
    tr = SupervisorFloorTracker()
    tr.acquire("client-a")
    tr.release("client-a")
    res = tr.acquire("client-b")
    assert res.granted is True
    assert tr.holder == "client-b"


def test_force_release_returns_prev_holder_and_clears():
    tr = SupervisorFloorTracker()
    tr.acquire("client-a")
    prev = tr.force_release()
    assert prev == "client-a"
    assert tr.holder is None
    # Повторный force без holder — None.
    assert tr.force_release() is None


def test_floor_held_error_rate_limit_first_call_allowed():
    clk = _FakeClock(100.0)
    tr = SupervisorFloorTracker(now_fn=clk)
    tr.acquire("client-a")
    # client-b пытается — получает ошибку; первая попытка ВСЕГДА проходит.
    assert tr.should_send_floor_held_error("client-b") is True


def test_floor_held_error_rate_limit_blocks_within_window():
    clk = _FakeClock(100.0)
    tr = SupervisorFloorTracker(now_fn=clk)
    tr.acquire("client-a")
    assert tr.should_send_floor_held_error("client-b") is True
    # +0.5 с — внутри окна, блокируем.
    clk.advance(0.5)
    assert tr.should_send_floor_held_error("client-b") is False
    # + ещё 0.4 с (всего 0.9 с) — всё ещё внутри окна.
    clk.advance(0.4)
    assert tr.should_send_floor_held_error("client-b") is False


def test_floor_held_error_rate_limit_resets_after_window():
    clk = _FakeClock(100.0)
    tr = SupervisorFloorTracker(now_fn=clk)
    tr.acquire("client-a")
    assert tr.should_send_floor_held_error("client-b") is True
    clk.advance(FLOOR_HELD_RATE_LIMIT_S + 0.1)  # > 1 с
    assert tr.should_send_floor_held_error("client-b") is True


def test_floor_held_error_rate_limit_is_per_client():
    """Два разных client_id не делят одно rate-limit окно."""
    clk = _FakeClock(100.0)
    tr = SupervisorFloorTracker(now_fn=clk)
    tr.acquire("client-x")  # holder
    # client-y шлёт → True (его первое окно).
    assert tr.should_send_floor_held_error("client-y") is True
    # client-z (тоже не держит) тут же → True (независимое окно).
    assert tr.should_send_floor_held_error("client-z") is True
    # client-y повторно через 0 с — False (его окно уже открыто).
    assert tr.should_send_floor_held_error("client-y") is False


def test_reset_rate_limit_clears_window():
    clk = _FakeClock(100.0)
    tr = SupervisorFloorTracker(now_fn=clk)
    tr.acquire("client-a")
    assert tr.should_send_floor_held_error("client-b") is True
    # Сразу повторно — False.
    assert tr.should_send_floor_held_error("client-b") is False
    tr.reset_rate_limit("client-b")
    # После сброса — снова True (окно очищено).
    assert tr.should_send_floor_held_error("client-b") is True


def test_holder_returns_none_initially():
    tr = SupervisorFloorTracker()
    assert tr.holder is None
    assert tr.is_held_by(None) is False
    assert tr.is_held_by("anyone") is False
