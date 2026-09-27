"""Issue #2835 — чистая политика поколений диалоговой сессии."""

from __future__ import annotations

import asyncio
import json
import logging
from unittest.mock import MagicMock

from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.core.music_guard import MusicGuard
from rob_box_voice.core.session_epoch import TURN_EPOCH, SessionEpoch


def _on() -> str:
    return json.dumps({"enabled": True, "persona": "Пёс"})


def _off() -> str:
    return json.dumps({"enabled": False})


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


class TestDjFence:
    def test_enable_admitted_before_any_reset(self) -> None:
        assert SessionEpoch().admits_dj_payload(_on())

    def test_enable_rejected_after_reset_until_new_turn(self) -> None:
        e = SessionEpoch()
        e.advance()
        assert not e.admits_dj_payload(_on())
        e.note_turn_started(0)  # ход старой сессии забор не снимает
        assert not e.admits_dj_payload(_on())
        e.note_turn_started(1)
        assert e.admits_dj_payload(_on())

    def test_disable_always_admitted(self) -> None:
        e = SessionEpoch()
        e.advance()
        assert e.admits_dj_payload(_off())

    def test_bad_payload_passes_to_controller(self) -> None:
        e = SessionEpoch()
        e.advance()
        assert e.admits_dj_payload("not json")


class TestDjSilentReset:
    def _dj(self) -> tuple[DJModeController, MagicMock]:
        on_stop = MagicMock()
        dj = DJModeController(
            hook=DJHook(
                dispatch=MagicMock(),
                is_active=lambda: False,
                is_dialogue_active=lambda: False,
                on_stop=on_stop,
            ),
            logger=logging.getLogger("test"),
        )
        return dj, on_stop

    def test_reset_silently_turns_dj_off_without_farewell(self) -> None:
        dj, on_stop = self._dj()
        dj.handle_message(_on())
        assert dj.state.enabled
        dj.reset_silently()
        assert not dj.state.enabled
        assert dj.state.next_transition_at == 0.0
        on_stop.assert_not_called()

    def test_disable_echo_on_already_off_dj_has_no_farewell(self) -> None:
        dj, on_stop = self._dj()
        dj.handle_message(_off())
        on_stop.assert_not_called()

    def test_disable_of_running_dj_still_says_farewell(self) -> None:
        dj, on_stop = self._dj()
        dj.handle_message(_on())
        dj.handle_message(_off())
        on_stop.assert_called_once()


class TestMusicGuardSessionReset:
    def test_reset_for_new_session_zeroes_both_budgets(self) -> None:
        g = MusicGuard()
        g._dj_retry_count = 2
        g._user_retry_count = 3
        g.reset_for_new_session()
        assert g.dj_retry_count == 0
        assert g.user_retry_count == 0
