"""Issue #2999 — адаптер ``DialogueNode._apply_music_guard`` на DJ-запросе.

Харнесс без ROS2 (``object.__new__``, как ``test_issue_2971``): реальные
``MusicGuard`` и ``DJModeController``, моки только на побочных эффектах.
Проверяем, что на «Ты диджей X…» уходит DJ-ретрай (не промпт Bug C),
что ``set_dj_mode`` закрывает ход без ретрая и что обёртка хода при
активном сете больше не запрещает ``set_dj_mode``.
"""

from __future__ import annotations

import json
import logging
from unittest.mock import MagicMock

from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.core.dj_request import is_dj_request
from rob_box_voice.core.music_guard import MusicGuard
from rob_box_voice.dialogue_node import DialogueNode

LIVE_A = "[TG] Ты диджей Снупдог и у нас сегодня вечеринка ганкста в чорном квартале"
LIVE_B = "[TG] Ты диджей Анакен скайвокер и у нас сегодня имперский слет в клубе"
_DIVE_ON = json.dumps({
    "enabled": True, "persona": "диджей Дайв", "theme": "клубная вечеринка",
})


def _make_node() -> DialogueNode:
    n = object.__new__(DialogueNode)
    n.get_logger = lambda: MagicMock()
    n._music_guard = MusicGuard()
    n._farewell = MagicMock()
    n._dj = DJModeController(
        hook=DJHook(
            dispatch=MagicMock(),
            is_active=lambda: False,
            is_dialogue_active=lambda: False,
            on_stop=n._farewell,
        ),
        logger=logging.getLogger("test_2999_node"),
    )
    n._dj_mode_pub = MagicMock()
    n._retry_dispatched_in_turn = False
    n._consume_synthetic_retry = MagicMock(return_value=True)
    n._discard_last_music_reply = MagicMock()
    n._speak_direct = MagicMock()
    n._mark_retry_dispatched = MagicMock()
    n._reopen_dialogue_for_retry = MagicMock()
    n._dispatch_dj_turn = MagicMock()
    n._dispatch_turn = MagicMock()
    n._build_music_retry_prompt = MagicMock(return_value="[CRITICAL] music")
    n._build_dj_retry_prompt = MagicMock(return_value="[CRITICAL] dj")
    n._publish_music_cleanup = MagicMock()
    n._classify_music_user_input_kind = MagicMock(return_value="other")
    return n


def _guard(n: DialogueNode, user_input: str, tools=()) -> bool:
    return n._apply_music_guard(
        was_dj_auto=False,
        user_input=user_input,
        tools_called=tuple(tools),
        spoken="Йо, бит качает, поехали!",
    )


def test_scenario_a_dispatches_dj_retry_not_bug_c() -> None:
    n = _make_node()
    assert _guard(n, LIVE_A, ()) is True
    n._build_music_retry_prompt.assert_not_called()
    n._discard_last_music_reply.assert_called_once()
    n._dispatch_turn.assert_called_once()
    args, kwargs = n._dispatch_turn.call_args
    assert "set_dj_mode(enabled=true" in args[0]
    assert "persona='диджей Снупдог'" in args[0]
    assert kwargs["is_synthetic"] is True
    assert kwargs["raw_user_command"] == LIVE_A


def test_scenario_a_set_dj_mode_closes_turn() -> None:
    n = _make_node()
    tools = ("load_skill", "set_dj_mode", "compose_music")
    assert _guard(n, LIVE_A, tools) is False
    n._dispatch_turn.assert_not_called()
    n._speak_direct.assert_not_called()


def test_scenario_a_exhausted_speaks_dj_fallback_once() -> None:
    n = _make_node()
    _guard(n, LIVE_A, ())
    _guard(n, LIVE_A, ())
    assert n._dispatch_turn.call_count == 2
    assert _guard(n, LIVE_A, ()) is False
    assert n._dispatch_turn.call_count == 2  # третьего ретрая нет
    n._speak_direct.assert_called_once()
    assert "Диджей-сет" in n._speak_direct.call_args[0][0]
    n._build_music_retry_prompt.assert_not_called()


def test_scenario_b_one_off_track_retries_with_active_set_prompt() -> None:
    n = _make_node()
    n._dj.handle_message(_DIVE_ON)
    assert _guard(n, LIVE_B, ("lookup_melody",)) is True
    prompt = n._dispatch_turn.call_args[0][0]
    assert "УЖЕ ИДЁТ" in prompt
    assert "bpm НЕ передавай" in prompt
    assert n._dj.state.enabled is True  # сет не выключен


def test_scenario_b_wrapper_no_longer_forbids_set_dj_mode() -> None:
    n = _make_node()
    n._dj.handle_message(_DIVE_ON)
    # Ровно как в DialogueNode._dispatch_cleaned.
    wrapped = n._dj.preamble(dj_request=is_dj_request(LIVE_B)) + LIVE_B
    assert "Не вызывай set_dj_mode" not in wrapped
    assert "set_dj_mode(enabled=true" in wrapped
    # Обычная команда посреди сета — старая обёртка.
    plain = n._dj.preamble(dj_request=is_dj_request("[TG] диджей, сделай громче"))
    assert "Не вызывай set_dj_mode" in plain


def test_dispatch_cleaned_passes_dj_request_flag() -> None:
    """Места вызова обёртки в ноде — через ``is_dj_request(clean)``."""
    import inspect

    src = inspect.getsource(DialogueNode._dispatch_cleaned)
    assert "self._dj.preamble(dj_request=is_dj_request(clean))" in src
    guard_src = inspect.getsource(DialogueNode._apply_music_guard)
    assert "self._music_guard.evaluate_turn(" in guard_src
