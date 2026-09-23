"""test_issue_2875_dj_track_counter.py — нода сообщает DJ о запущенных треках.

Issue #2875: номер трека плана = число реально запущенных треков, а не
номер DJ-перехода. Сигнал — ``MUSIC_STARTING_TOOLS`` в ``result.tools_called``
завершённого хода; нода передаёт его в ``DJModeController.note_turn_tools``
из ``_finalize_music_cleanup_policy`` (зовётся на каждый ход в ``finally``
``_run_turn``).
"""

from __future__ import annotations

import json
import logging
from types import SimpleNamespace
from unittest.mock import MagicMock

from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.dialogue_node import DialogueNode


def _node_with_dj(enabled: bool = True) -> DialogueNode:
    node = object.__new__(DialogueNode)
    node.get_logger = lambda: MagicMock()
    node._dj = DJModeController(
        hook=DJHook(
            dispatch=lambda *a, **k: None,
            is_active=lambda: False,
            is_dialogue_active=lambda: False,
        ),
        logger=logging.getLogger("t"),
    )
    if enabled:
        node._dj.handle_message(json.dumps({"enabled": True, "plan": "Трек 1: a"}))
    # Остальная cleanup-политика в этом тесте не проверяется.
    node._apply_stop_music_deferral = lambda result: False
    node._schedule_music_cleanup = lambda **kw: None
    node._flush_music_cleanup_if_idle = lambda was_dj_auto: None
    return node


def _finalize(
    node: DialogueNode,
    tools: list | None,
    *,
    was_dj_auto: bool = True,
    user_input: str = "x",
) -> None:
    result = None if tools is None else SimpleNamespace(tools_called=tools)
    node._finalize_music_cleanup_policy(
        result=result,
        was_dj_auto=was_dj_auto,
        raw_user_command=None,
        user_input=user_input,
    )


def test_user_request_mid_set_is_not_a_set_track() -> None:
    """Дополнение #2875: «сыграй тему марио» посреди сета — заказ, не трек."""
    node = _node_with_dj()

    _finalize(
        node, ["compose_music"], was_dj_auto=False,
        user_input="[TG] сыграй тему марио",
    )

    assert node._dj.state.tracks_started == 0


def test_retry_of_dj_transition_counts_via_turn_text() -> None:
    node = _node_with_dj()

    _finalize(
        node, ["compose_music"], was_dj_auto=False,
        user_input="[Speaker:unknown] [DJ_AUTO переход #2] ...",
    )

    assert node._dj.state.tracks_started == 1


def test_turn_that_started_music_counts_a_track() -> None:
    node = _node_with_dj()

    _finalize(node, ["speak_text", "compose_music"])

    assert node._dj.state.tracks_started == 1


def test_turn_without_music_start_does_not_count() -> None:
    node = _node_with_dj()

    _finalize(node, ["speak_text", "set_dj_mode"])
    _finalize(node, None)  # отменённый/упавший ход

    assert node._dj.state.tracks_started == 0


def test_music_outside_dj_set_is_not_counted() -> None:
    node = _node_with_dj(enabled=False)

    _finalize(node, ["compose_music"])

    assert node._dj.state.tracks_started == 0


def test_node_without_dj_controller_does_not_crash() -> None:
    node = _node_with_dj()
    del node._dj

    _finalize(node, ["compose_music"])
