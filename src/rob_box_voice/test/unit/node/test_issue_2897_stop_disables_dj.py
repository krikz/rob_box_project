"""Issue #2897 — юзерская стоп-команда детерминированно выключает DJ.

Живой прогон 23.09.2026 19:27-19:29 UTC («Хопер»): «хватит диджеить» →
LLM вызвала только ``stop_music`` (звук встал), DJ-режим остался включён
(``set_dj_mode(enabled=false)`` LLM не вызвала), и 46с спустя тик
``DJModeController`` запустил финальный переход + перезапустил музыку.

Источник правды выбран в ``dialogue_node._apply_music_guard`` /
``_force_dj_off_for_stop_command``: ``is_music_stop_command(user_input)``
со стороны кода, а НЕ тул, который решила закрыть LLM (``stop_music``
глушит звук, но никогда не трогал DJ-флаг — см. ``MUSIC_HARD_STOP_TOOLS``
в ``core/dialogue_guards.py``). Это работает независимо от того, ушёл ли
ход через ``stop_music`` (MusicGuard → SKIP_NOT_APPLICABLE) или без
стоп-тула вовсе (MusicGuard → FORCE_STOP).

DialogueNode собирается через ``object.__new__`` (как в
test_new_session_resets_dj.py / test_issue_2874_silent_retries.py), rclpy
замокан в conftest.
"""

from __future__ import annotations

import json
import logging
from unittest.mock import MagicMock

from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.core.music_guard import MusicGuard
from rob_box_voice.dialogue_node import DialogueNode

_DJ_ON = json.dumps({"enabled": True, "persona": "ДиДжей Хопер"})


def _real_dj(on_stop: MagicMock) -> DJModeController:
    return DJModeController(
        hook=DJHook(
            dispatch=MagicMock(),
            is_active=lambda: False,
            is_dialogue_active=lambda: False,
            on_stop=on_stop,
        ),
        logger=logging.getLogger("test_2897"),
    )


def _make_node() -> DialogueNode:
    """Узел с реальными ``_dj`` / ``_music_guard`` — ``_apply_music_guard``
    вызывается НЕ замоканным, чтобы проверить настоящую побочку.
    """
    n = object.__new__(DialogueNode)
    n.get_logger = lambda: MagicMock()
    n._music_guard = MusicGuard()
    n._farewell = MagicMock()
    n._dj = _real_dj(n._farewell)
    n._dj_mode_pub = MagicMock()
    n._retry_dispatched_in_turn = False
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
    return n


def _dj_off_published(n) -> bool:
    return any(
        json.loads(c.args[0].data) == {"enabled": False}
        for c in n._dj_mode_pub.publish.call_args_list
    )


class TestStopMusicToolDisablesDj:
    """DJ включён, «хватит диджеить», tools=['stop_music']."""

    def test_dj_disabled(self):
        n = _make_node()
        n._dj.handle_message(_DJ_ON)
        assert n._dj.state.enabled

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="хватит диджеить",
            tools_called=("stop_music",),
            spoken="Готово, музыка выключена!",
        )

        assert n._dj.state.enabled is False

    def test_dj_off_published_for_mcp_watchdog(self):
        n = _make_node()
        n._dj.handle_message(_DJ_ON)

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="хватит диджеить",
            tools_called=("stop_music",),
            spoken="Готово, музыка выключена!",
        )

        assert _dj_off_published(n)

    def test_no_farewell_over_the_reply(self):
        n = _make_node()
        n._dj.handle_message(_DJ_ON)

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="хватит диджеить",
            tools_called=("stop_music",),
            spoken="Готово, музыка выключена!",
        )

        n._farewell.assert_not_called()

    def test_deferred_farewell_is_cancelled(self):
        """Отложенное прощание (#2875) — от прошлого финала — тоже гасится."""
        n = _make_node()
        n._dj.handle_message(_DJ_ON)
        # Симулируем уже взведённое отложенное прощание прошлого сета.
        n._dj.state.farewell_at = 999999999.0
        n._dj.state.farewell_persona = "ДиДжей Хопер"

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="хватит диджеить",
            tools_called=("stop_music",),
            spoken="Готово, музыка выключена!",
        )

        assert n._dj.state.farewell_at is None
        n._dj.tick()
        n._farewell.assert_not_called()

    def test_next_tick_does_not_dispatch_transition(self):
        n = _make_node()
        n._dj.handle_message(_DJ_ON)

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="хватит диджеить",
            tools_called=("stop_music",),
            spoken="Готово, музыка выключена!",
        )
        n._dj.tick()

        n._dj._hook.dispatch.assert_not_called()


class TestStopDjWithoutStopToolDisablesDj:
    """DJ включён, «стоп диджей», tools=[] (FORCE_STOP путь MusicGuard)."""

    def test_dj_disabled(self):
        n = _make_node()
        n._dj.handle_message(_DJ_ON)

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="стоп диджей",
            tools_called=(),
            spoken="Хорошо, стою.",
        )

        assert n._dj.state.enabled is False

    def test_dj_off_published(self):
        n = _make_node()
        n._dj.handle_message(_DJ_ON)

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="стоп диджей",
            tools_called=(),
            spoken="Хорошо, стою.",
        )

        assert _dj_off_published(n)

    def test_no_farewell(self):
        n = _make_node()
        n._dj.handle_message(_DJ_ON)

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="стоп диджей",
            tools_called=(),
            spoken="Хорошо, стою.",
        )

        n._farewell.assert_not_called()

    def test_next_tick_does_not_dispatch_transition(self):
        n = _make_node()
        n._dj.handle_message(_DJ_ON)

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="стоп диджей",
            tools_called=(),
            spoken="Хорошо, стою.",
        )
        n._dj.tick()

        n._dj._hook.dispatch.assert_not_called()

    def test_music_cleanup_still_force_published(self):
        """FORCE_STOP-ветка по-прежнему гасит звук (issue #992 Bug F) —
        фикс #2897 не должен убрать существующую защиту."""
        n = _make_node()
        n._dj.handle_message(_DJ_ON)

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="стоп диджей",
            tools_called=(),
            spoken="Хорошо, стою.",
        )

        n._publish_music_cleanup.assert_called_once()


class TestNonDjStopMusicUnchanged:
    """«стоп музыка» вне DJ-режима — поведение не меняется."""

    def test_dj_off_not_published_when_already_off(self):
        n = _make_node()
        assert n._dj.state.enabled is False

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="стоп музыка",
            tools_called=(),
            spoken="Музыка остановлена.",
        )

        n._dj_mode_pub.publish.assert_not_called()

    def test_force_stop_music_cleanup_unchanged(self):
        n = _make_node()

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="стоп музыка",
            tools_called=(),
            spoken="Музыка остановлена.",
        )

        n._publish_music_cleanup.assert_called_once()

    def test_dj_state_stays_disabled(self):
        n = _make_node()

        n._apply_music_guard(
            was_dj_auto=False,
            user_input="стоп музыка",
            tools_called=(),
            spoken="Музыка остановлена.",
        )

        assert n._dj.state.enabled is False
