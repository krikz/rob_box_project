"""#3779 — move_direction в ходе «посмотри налево» не исполняется: отказ честной фразой кода (ADR-0148)."""

import logging

from rob_box_harness.core.agent_core import _refuse_unasked_motion
from rob_box_llm.provider import ToolCall


def _call(name="move_direction", **args):
    return ToolCall(id="c1", name=name, arguments=args or {"direction": "налево"})


def test_look_turn_refuses_move_and_logs(caplog):
    out = {}
    with caplog.at_level(logging.WARNING):
        left = _refuse_unasked_motion([_call()], "Робот, посмотри налево", out)
    result = out["c1"]
    assert left == [] and result.is_error and result.tool_call_id == "c1"
    assert "не просил ехать" in result.content
    assert "🚫 [move-guard]" in caplog.text


def test_motion_verb_lets_move_through():
    call = _call()
    assert _refuse_unasked_motion([call], "поезжай налево на метр", {}) == [call]


def test_other_tools_and_empty_input_untouched():
    anim, move = _call("play_animation", face={"gaze": "left"}), _call()
    assert _refuse_unasked_motion([anim], "посмотри налево", {}) == [anim]
    assert _refuse_unasked_motion([move], "", {}) == [move]
