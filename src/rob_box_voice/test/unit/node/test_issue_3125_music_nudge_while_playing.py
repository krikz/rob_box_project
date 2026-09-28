"""Issue #3125 — nudge «Я тут растерялся — бит не запустился» не звучит,
когда музыка играет; ``_apply_music_guard`` прокидывает ``succeeded_tools``
(#3004) и флаг «играет» в :class:`MusicGuard`.

Живой сет 28.09.2026 (``voice-assistant.log``): трек играл
(``[track-mode] TRACK играет с прошлого хода``), а робот трижды сказал «бит
не запустился». Строки ``user_input`` — дословно из лога (см.
``test/unit/core/test_issue_3125_music_volume_bug_c.py``).
"""

from __future__ import annotations

from unittest.mock import MagicMock

from rob_box_voice.core.music_guard import MusicGuard
from rob_box_voice.core.music_player_state import MusicPlayerState
from rob_box_voice.dialogue_node import DialogueNode

_DJ_WRAPPER = (
    '[TG] [🎧 Музыкальный режим активен — фоновая музыка играет, тема: '
    '"клубная вечеринка", диджей: диджей Дайв. Это ОБЫЧНАЯ команда юзера, '
    'не DJ-переход. Ответь на неё нормально. Не вызывай set_dj_mode и не '
    'меняй музыку, если юзер об этом не просит.] '
)
LIVE_B_RETRY_USER_INPUT = (
    "[Speaker:unknown] играй трей кромче а не говори громче\n\n[CRITICAL] "
    "Твой предыдущий ответ содержал ОБЕЩАНИЕ ДЕЙСТВИЯ (слова «сделал / "
    "запустил / перезапущу / установлю / остановлю / проверю / подложу / "
    "переключу / подкручу / обновлю / поменяю / изменю / доработаю» и т.п.), "
    "но ты НЕ вызвал НИ ОДНОГО инструмента"
)
LIVE_B_SPOKEN = (
    "Йо, понял — трек громче, не голос! Подкручиваю музыку на максимум, "
    "чтоб стены тряслись!"
)
LIVE_C_USER_INPUT = _DJ_WRAPPER + "сыграй в пещере гороного короля погромче"
LIVE_C_TOOLS = ("lookup_melody", "compose_music")


# --- nudge «бит не запустился» не звучит при играющей музыке ----------------


def _make_node(*, playing: bool, budget_left: bool = False):
    n = object.__new__(DialogueNode)
    n.get_logger = lambda: MagicMock()
    n._music_guard = MusicGuard()
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._retry_dispatched_in_turn = False
    # Issue #3133: «играет» — только снимок плеера (/voice/music/state).
    n._music_player_state = MusicPlayerState(state="playing" if playing else "idle")
    n._consume_synthetic_retry = MagicMock(return_value=budget_left)
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


def _spoken(n) -> list:
    return [c.args[0] for c in n._speak_direct.call_args_list]


class TestNudgeWhileMusicPlaying:
    def test_live_b_volume_request_no_nudge_at_all(self) -> None:
        # Бюджет исчерпан (как на 1790611319), но Bug C уже не ретраит.
        n = _make_node(playing=True, budget_left=False)
        dispatched = n._apply_music_guard(
            was_dj_auto=False,
            user_input=LIVE_B_RETRY_USER_INPUT,
            tools_called=(),
            spoken=LIVE_B_SPOKEN,
        )
        assert dispatched is False
        assert not any("растерялся" in t for t in _spoken(n))
        n._dispatch_turn.assert_not_called()

    def test_budget_exhausted_while_playing_is_honest(self) -> None:
        # Другая (не громкость) просьба, бюджет выгорел, музыка играет:
        # «бит не запустился» было бы враньём.
        n = _make_node(playing=True, budget_left=False)
        n._apply_music_guard(
            was_dj_auto=False,
            user_input=LIVE_C_USER_INPUT,
            tools_called=(),
        )
        said = _spoken(n)
        assert said == [DialogueNode.MUSIC_RETRY_NUDGE_WHILE_PLAYING_TEXT]
        assert not any("бит не запустился" in t for t in said)
        n._discard_last_music_reply.assert_called_once()

    def test_budget_exhausted_without_music_keeps_old_text(self) -> None:
        n = _make_node(playing=False, budget_left=False)
        n._apply_music_guard(
            was_dj_auto=False,
            user_input=LIVE_C_USER_INPUT,
            tools_called=(),
        )
        assert _spoken(n) == ["Я тут растерялся — бит не запустился, попробуй ещё раз."]

    def test_error_then_success_turn_threads_succeeded_tools(self) -> None:
        n = _make_node(playing=True, budget_left=True)
        dispatched = n._apply_music_guard(
            was_dj_auto=False,
            user_input=LIVE_C_USER_INPUT,
            tools_called=LIVE_C_TOOLS,
            tool_error_occurred=True,
            succeeded_tools=("lookup_melody", "compose_music"),
        )
        assert dispatched is False
        n._dispatch_turn.assert_not_called()
        assert _spoken(n) == []
