"""Issue #3266 — Bug C не ретраит ход, в котором сет уже включён.

Живой прогон 01.10.2026, Vision Pi, ``docker logs voice-assistant``::

    11:12:03  «[TG] Ты Диджей Русс Иван и сегодня у нас чисто славянская
              вечеринка полька калинка …»
              execute_music_code(name='Калинка') → ❌ «не найдена в библиотеке»
              set_dj_mode(enabled=True, next_transition_sec=50) → ✅
              🎧 DJ Mode ON — next transition in 50s
    11:12:05  spoken='Калинку я не нашёл в нотах — сыграю её в духе народной
              плясовой, …' tools=['execute_music_code', 'set_dj_mode']
              [issue 2966] … falling through to Bug B/C
              [issue 992 Bug C] … synchronous retry 1/3
              [issue 2874] ответ хода не озвучиваю (retracted=True)
    11:12:07  spoken='DJ Русс Иван за пультом — гармонь и балалайка уже в
              эфире …' tools=[]   → retry 2/3, отозван
    11:12:15  spoken='… пилотный трек в эфире.' tools=[] → retry 3/3, бюджет
    11:12:54  🎧 DJ auto-transition #1 «СТАРТ ВЕЧЕРИНКИ» — музыка пошла.

Сет включил сам ``set_dj_mode``; ретраи только выманили у модели «уже в
эфире» и съели честный ответ.
"""

from __future__ import annotations

import logging
from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.dj_mode import DJModeController, DJState
from rob_box_voice.core.music_guard import (
    DJ_SET_TAKES_OVER,
    MusicGuard,
    MusicGuardVerdictKind,
    hurry_dj_set_start,
)
from rob_box_voice.dialogue_node import DialogueNode

LIVE_USER_INPUT = (
    "[TG] Ты Диджей Русс Иван и сегодня у нас чисто славянская вечеринка "
    "полька калинка молдованка польская корова бобр курва наше все!"
)
LIVE_SPOKEN = (
    "Калинку я не нашёл в нотах — сыграю её в духе народной плясовой,"
    "132 удара, гармошка, бас гудит."
)
LIVE_TOOLS = ("execute_music_code", "set_dj_mode")
MID_SET_ORDER = "сыграй калинку"


def _guard() -> MusicGuard:
    return MusicGuard(logger=logging.getLogger("test_3266"))


def _evaluate(guard: MusicGuard, **overrides):
    kwargs = dict(
        was_dj_auto=False,
        user_input=LIVE_USER_INPUT,
        tools_called=LIVE_TOOLS,
        dj_enabled=True,
        spoken=LIVE_SPOKEN,
        tool_error_occurred=True,
        succeeded_tools=("set_dj_mode",),
        music_playing=False,
    )
    kwargs.update(overrides)
    return guard.evaluate(**kwargs)


# --- MusicGuard: политика --------------------------------------------------


class TestGuardPolicy:
    def test_live_turn_is_not_retried(self) -> None:
        guard = _guard()
        verdict = _evaluate(guard)
        assert verdict.kind is MusicGuardVerdictKind.SKIP_NOT_APPLICABLE
        assert verdict.reason == DJ_SET_TAKES_OVER
        assert guard.user_retry_count == 0

    def test_set_dj_mode_alone_is_not_retried(self) -> None:
        """Сет без музыкального тула: трек поставит переход, как у роутера."""
        verdict = _evaluate(
            _guard(), tools_called=("set_dj_mode",), tool_error_occurred=False
        )
        assert verdict.reason == DJ_SET_TAKES_OVER

    @pytest.mark.parametrize(
        "overrides",
        [
            # DJ не включился (топик не дошёл / enabled=false) — ретрай как был.
            {"dj_enabled": False},
            # set_dj_mode сам упал.
            {"succeeded_tools": ()},
            # Харнесс не сообщил успехи — правило #2966 как раньше.
            {"succeeded_tools": None},
            # Заказ гостя посреди идущего сета: set_dj_mode в ходе нет.
            {
                "user_input": MID_SET_ORDER,
                "tools_called": ("execute_music_code",),
                "succeeded_tools": (),
            },
        ],
        ids=["dj_off", "set_dj_mode_failed", "no_succeeded_info", "guest_order"],
    )
    def test_bug_c_still_retries_otherwise(self, overrides) -> None:
        verdict = _evaluate(_guard(), **overrides)
        assert verdict.kind is MusicGuardVerdictKind.USER_RETRY
        assert verdict.reason == "bug_c"

    def test_dj_auto_transition_keeps_bug_b(self) -> None:
        """Ход DJ-перехода без трека — по-прежнему Bug B, не исключение."""
        verdict = _evaluate(_guard(), was_dj_auto=True)
        assert verdict.kind is MusicGuardVerdictKind.DJ_RETRY

    def test_successful_track_is_still_plain_skip(self) -> None:
        verdict = _evaluate(
            _guard(),
            tool_error_occurred=False,
            succeeded_tools=LIVE_TOOLS,
        )
        assert verdict.kind is MusicGuardVerdictKind.SKIP
        assert verdict.reason == "executed"


# --- hurry_dj_set_start: переход вместо таймера модели ----------------------

NOW = 1_790_842_325.0


def _state(next_in: float) -> DJState:
    state = DJState(enabled=True)
    state.next_transition_at = NOW + next_in
    return state


def _skip_verdict():
    return _evaluate(_guard())


class TestHurryTransition:
    def test_live_case_pulls_transition(self) -> None:
        state = _state(50.0)
        assert hurry_dj_set_start(
            _skip_verdict(), state, tools_called=LIVE_TOOLS,
            music_playing=False, now=NOW, delay_s=15.0,
        )
        assert state.next_transition_at == NOW + 15.0

    @pytest.mark.parametrize(
        "tools, playing, next_in",
        [
            (("set_dj_mode",), False, 50.0),  # модель сама отдала старт переходу
            (LIVE_TOOLS, True, 50.0),         # что-то играет — форма решает
            (LIVE_TOOLS, False, 5.0),         # переход и так скоро
        ],
        ids=["no_music_tool", "music_playing", "already_sooner"],
    )
    def test_timer_untouched(self, tools, playing, next_in) -> None:
        state = _state(next_in)
        assert not hurry_dj_set_start(
            _skip_verdict(), state, tools_called=tools,
            music_playing=playing, now=NOW, delay_s=15.0,
        )
        assert state.next_transition_at == NOW + next_in

    def test_other_verdicts_untouched(self) -> None:
        state = _state(50.0)
        other = _evaluate(_guard(), user_input="расскажи анекдот")
        assert other.reason == "not_music_request"
        assert not hurry_dj_set_start(
            other, state, tools_called=LIVE_TOOLS,
            music_playing=False, now=NOW, delay_s=15.0,
        )
        assert state.next_transition_at == NOW + 50.0


# --- DialogueNode: ход звучит, ретрая нет -----------------------------------


def _make_node():
    n = object.__new__(DialogueNode)
    n.get_logger = lambda: MagicMock()
    n._music_guard = MusicGuard()
    n._dj = MagicMock()
    n._dj.state = DJState(enabled=True)
    n._retry_dispatched_in_turn = False
    n._track_mode_music_active = False
    n._music_player_state = None
    n._generated_music_state = None
    n._consume_synthetic_retry = MagicMock(return_value=True)
    n._discard_last_music_reply = MagicMock()
    n._speak_direct = MagicMock()
    n._mark_retry_dispatched = MagicMock()
    n._reopen_dialogue_for_retry = MagicMock()
    n._dispatch_dj_turn = MagicMock()
    n._dispatch_turn = MagicMock()
    n._build_music_retry_prompt = MagicMock(return_value="[CRITICAL] ...")
    n._build_dj_retry_prompt = MagicMock(return_value="[CRITICAL] ...")
    n._publish_music_cleanup = MagicMock()
    n._classify_music_user_input_kind = MagicMock(return_value="other")
    n._should_force_dj_off_for_stop_command = MagicMock(return_value=False)
    return n


class TestNodeLiveTurn:
    def test_reply_is_voiced_and_transition_is_soon(self, monkeypatch) -> None:
        import rob_box_voice.dialogue_node as dn

        monkeypatch.setattr(dn.time, "time", lambda: NOW)
        n = _make_node()
        n._dj.state.next_transition_at = NOW + 50.0  # next_transition_sec=50

        dispatched = n._apply_music_guard(
            was_dj_auto=False,
            user_input=LIVE_USER_INPUT,
            tools_called=LIVE_TOOLS,
            spoken=LIVE_SPOKEN,
            tool_error_occurred=True,
            succeeded_tools=("set_dj_mode",),
        )

        assert dispatched is False
        n._dispatch_turn.assert_not_called()
        n._dispatch_dj_turn.assert_not_called()
        # Ответ хода не отзывается и не подменяется фразой гуарда.
        n._discard_last_music_reply.assert_not_called()
        n._speak_direct.assert_not_called()
        assert (
            n._dj.state.next_transition_at
            == NOW + DJModeController.POSTPONE_INTERVAL_S
        )
