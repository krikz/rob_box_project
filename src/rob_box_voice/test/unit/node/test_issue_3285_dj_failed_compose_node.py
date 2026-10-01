"""test_issue_3285_dj_failed_compose_node.py — нода передаёт DJ ``succeeded_tools``.

Issue #3285: ``_finalize_music_cleanup_policy`` отдавал в ``note_turn_tools``
только ``result.tools_called`` — упавший compose_music считался треком сета.
"""

from __future__ import annotations

import json
import logging
from types import SimpleNamespace
from unittest.mock import MagicMock

from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.dialogue_node import DialogueNode


def _controller() -> DJModeController:
    ctrl = DJModeController(
        hook=DJHook(
            dispatch=lambda *a, **k: None,
            is_active=lambda: False,
            is_dialogue_active=lambda: False,
        ),
        logger=logging.getLogger("t"),
    )
    ctrl.handle_message(json.dumps({"enabled": True, "plan": "Трек 1: a"}))
    return ctrl


def test_node_passes_succeeded_tools_to_dj() -> None:
    node = object.__new__(DialogueNode)
    node.get_logger = lambda: MagicMock()
    node._dj = _controller()
    node._apply_stop_music_deferral = lambda result: False
    node._schedule_music_cleanup = lambda **kw: None
    node._flush_music_cleanup_if_idle = lambda was_dj_auto: None

    node._finalize_music_cleanup_policy(
        result=SimpleNamespace(
            tools_called=["compose_music", "set_dj_mode"],
            succeeded_tools=["set_dj_mode"],
        ),
        was_dj_auto=True,
        raw_user_command=None,
        user_input="x",
    )

    assert node._dj.state.tracks_started == 0
