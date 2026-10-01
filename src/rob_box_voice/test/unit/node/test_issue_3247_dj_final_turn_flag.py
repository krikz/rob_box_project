"""Issue #3247 — нода выставляет ``TURN_DJ_SET_FINAL`` по DJ-контроллеру.

``_run_turn`` ставит флаг на время хода через
:meth:`DialogueNode._turn_is_dj_final`; гард исполнителя тулов по нему не
даёт финальному DJ_AUTO-ходу снова включить DJ (см.
``test/unit/core/test_issue_3247_dj_final_no_restart.py``).
"""

from __future__ import annotations

import ast
import json
import logging
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

from rob_box_voice.core.dj_mode import DJModeController
from rob_box_voice import dialogue_node as dialogue_node_module
from rob_box_voice.dialogue_node import DialogueNode


def _final_controller() -> DJModeController:
    hook = SimpleNamespace(
        persona_default="Роббокс", on_stop=lambda persona: None,
        dispatch=lambda prompt, is_auto=False: None,
        is_active=lambda: False, is_dialogue_active=lambda: False,
    )
    dj = DJModeController(hook=hook, logger=logging.getLogger("t3247"))
    dj.handle_message(json.dumps({
        "enabled": True, "theme": "море и чайки",
        "plan": "Трек 1: прибой\nТрек 2: чайки",
    }))
    dj.state.tracks_started = 1
    dj.state.transition_count = 2
    dj.build_auto_prompt(2)
    return dj


def test_turn_flag_reads_controller() -> None:
    node = SimpleNamespace(_dj=_final_controller())
    assert DialogueNode._turn_is_dj_final(node, True) is True
    assert DialogueNode._turn_is_dj_final(node, False) is False


def test_turn_flag_false_without_real_controller() -> None:
    # Стаб-нода без DJ-контроллера и с MagicMock-контроллером — флаг не
    # взводится (MagicMock truthy, но не ``True``).
    assert DialogueNode._turn_is_dj_final(SimpleNamespace(), True) is False
    node = SimpleNamespace(_dj=MagicMock())
    assert DialogueNode._turn_is_dj_final(node, True) is False


def test_run_turn_sets_and_resets_flag() -> None:
    source = Path(dialogue_node_module.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    run_turn = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.AsyncFunctionDef) and node.name == "_run_turn"
    )
    body = ast.unparse(run_turn)
    assert "TURN_DJ_SET_FINAL.set(self._turn_is_dj_final(is_dj_auto))" in body
    assert "TURN_DJ_SET_FINAL.reset(dj_final_token)" in body
