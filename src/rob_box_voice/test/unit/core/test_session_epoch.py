"""Issue #2835 — чистая политика поколений диалоговой сессии."""

from __future__ import annotations

import asyncio

from rob_box_voice.core.session_epoch import TURN_EPOCH, SessionEpoch


class TestStaleness:
    def test_fresh_epoch_is_current(self) -> None:
        e = SessionEpoch()
        assert e.current == 0
        assert not e.is_stale(0)

    def test_advance_makes_previous_epoch_stale(self) -> None:
        e = SessionEpoch()
        e.advance()
        assert e.is_stale(0)
        assert not e.is_stale(1)

    def test_unknown_epoch_is_never_stale(self) -> None:
        e = SessionEpoch()
        e.advance()
        assert not e.is_stale(None)


class TestRetriesAllowed:
    def test_normal_turn_may_retry(self) -> None:
        e = SessionEpoch()
        assert e.retries_allowed(turn_epoch=0, cancelled=False)

    def test_cancelled_turn_may_not_retry(self) -> None:
        e = SessionEpoch()
        assert not e.retries_allowed(turn_epoch=0, cancelled=True)

    def test_turn_that_outlived_reset_may_not_retry(self) -> None:
        e = SessionEpoch()
        e.advance()
        assert not e.retries_allowed(turn_epoch=0, cancelled=False)


class TestEpochForDispatch:
    def test_outside_turn_uses_current(self) -> None:
        e = SessionEpoch()
        e.advance()
        assert e.epoch_for_dispatch() == 1

    def test_inside_turn_inherits_parent_epoch(self) -> None:
        """Ретрай, задиспатченный изнутри хода старой сессии, несёт её
        поколение — даже если сброс успел пройти."""
        e = SessionEpoch()

        async def parent_turn() -> int:
            TURN_EPOCH.set(e.current)
            e.advance()  # сброс посреди хода (из ROS-потока)
            return e.epoch_for_dispatch()

        assert asyncio.run(parent_turn()) == 0
        assert e.is_stale(0)


