"""ADR-0150 PR-3b (#3690): стадии хода в ``DialogueNode`` v1 - рефлексы приходят от кода, а не от LLM.

Настоящий ``_run_turn`` (как в test_issue_2874_silent_retries), ``TurnStages`` пишет события в списки; проверяем, что
нода открывает ход на допуске, зовёт ``thinking`` до LLM, закрывает ход по итогам тулов и LLM и что id хода у
рефлексов и у MCP-тулов один.
"""

from __future__ import annotations

import asyncio
import threading
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, patch

from rob_box_core import expression_client
from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
from rob_box_voice.core.stt_admission import SttOutcome, SttOutcomeKind
from rob_box_voice.core.turn_stages import TurnStages
from rob_box_voice.dialogue_node import DialogueNode


class _Result:
    def __init__(self, spoken="Готово.", tools=(), error=None):
        self.spoken_text = spoken
        self.tools_called = tuple(tools)
        self.error = error


def _make_node(replies):
    n = object.__new__(DialogueNode)
    n._task_lock = threading.Lock()
    n._run_cancelled = False
    n._speaker_id_enabled = False
    n._handle_speaker_turn = MagicMock()
    n._apply_speaker_identity = MagicMock()
    n._build_dynamic_system_context = MagicMock(return_value="<ctx/>")
    n._llm = MagicMock()
    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.DIALOGUE
    n._publish_state = MagicMock()
    n._apply_tool_skipped_guard = MagicMock(return_value=False)
    n._maybe_record_session_end = MagicMock()
    n.get_logger = lambda: MagicMock()
    n._core = MagicMock()
    n._core.process_input = AsyncMock(side_effect=list(replies))
    n._dispatch_turn = MagicMock()
    n._reopen_dialogue_for_retry = MagicMock()
    n._loop = MagicMock()
    n._handle_result = MagicMock()
    n._handle_llm_error = MagicMock()
    n.events, n.requests = [], []
    n._stages = TurnStages(
        lambda **kw: n.requests.append(kw), lambda **kw: n.events.append(kw),
        epoch=lambda: 0, normalize=lambda name: name)
    return n


def _run(n, text="который час", **kw):
    with patch("rob_box_voice.dialogue_node.asyncio.run_coroutine_threadsafe", return_value=MagicMock()):
        asyncio.run(n._run_turn(text, session_epoch=0, **kw))


def _stages(n):
    return [(e["stage"], e["status"]) for e in n.events]


def test_turn_walks_addressed_thinking_done_and_shares_its_id_with_the_mcp_turn_context():
    n = _make_node([_Result("Сейчас пятнадцать ноль семь.")])
    n._stages.addressed("u1")

    _run(n, utterance_id="u1")

    assert _stages(n) == [("addressed", "started"), ("thinking", "started"), ("done", "finished")]
    assert {e["turn_id"] for e in n.events} == {n._turn_id}          # скрытый turn_id тулов = id рефлексов
    assert [r["emotion"] for r in n.requests][:2] == ["listening", "thinking_teeth"]


def test_llm_exception_ends_the_turn_failed_with_the_reason():
    n = _make_node([RuntimeError("boom")])
    n._stages.addressed("u1")

    _run(n, utterance_id="u1")

    assert _stages(n)[-1] == ("failed", "failed")
    assert n.events[-1]["reason"] == "llm_error:RuntimeError"
    assert (n.requests[-1]["emotion"], n.requests[-1]["earcon"]) == ("error", "fail")


def test_llm_error_in_the_result_ends_the_turn_failed():
    n = _make_node([_Result("", error=ValueError("provider down"))])
    n._stages.addressed("u1")

    _run(n, utterance_id="u1")

    assert _stages(n)[-1] == ("failed", "failed") and "provider down" in n.events[-1]["reason"]


def test_guard_retry_continues_the_turn_instead_of_ending_it():
    n = _make_node([_Result("Сейчас около трёх.")])
    n._apply_post_turn_retry_guards = MagicMock(return_value=True)
    n._stages.addressed("u1")

    _run(n, utterance_id="u1")

    assert _stages(n) == [("addressed", "started"), ("thinking", "started")]


def test_cancelled_turn_is_closed_without_a_reflex():
    n = _make_node([asyncio.CancelledError()])
    n._stages.addressed("u1")
    requests = len(n.requests)

    _run(n, utterance_id="u1")

    assert (n.events[-1]["status"], n.events[-1]["reason"]) == ("skipped", "cancelled")
    assert len(n.requests) == requests + 1          # только thinking, терминального рефлекса нет


def test_node_without_stages_still_runs_the_turn():
    n = _make_node([_Result()])
    del n._stages

    _run(n, utterance_id="u1")

    n._core.process_input.assert_awaited_once()
    assert isinstance(n._turn_stages(), TurnStages)


def test_admission_closing_a_phrase_for_the_robot_calms_the_face_other_replies_settle():
    n = _make_node([])
    n._stages.addressed("u1")
    n._stage_admission_closed("u1", SttOutcome(kind=SttOutcomeKind.DROP, step_name="silenced", reason="silenced"))
    assert n.events[-1]["status"] == "skipped" and n.events[-1]["reason"] == "silenced"

    n._stages.addressed("u2")
    n._stage_admission_closed(
        "u2", SttOutcome(kind=SttOutcomeKind.HANDLED, step_name="address", reason="backlog_accumulated"))
    assert n.events[-1]["reason"] == "backlog_accumulated"

    n._stages.addressed("u3")
    count = len(n.events)
    n._stage_admission_closed("u3", SttOutcome(kind=SttOutcomeKind.HANDLED, step_name="bare_wake", reason="bare_wake"))
    assert len(n.events) == count                                   # голое «робот»: остаётся «слушаю»

    n._stages.addressed("u4")
    n._stage_admission_closed(
        "u4", SttOutcome(kind=SttOutcomeKind.HANDLED, step_name="media_command", reason="media_command"))
    assert n._stages._turn.body_done_at is not None                  # код ответил сам: ход дочитает речь и закроется


def test_tool_hook_reports_calls_and_errors_to_the_stages():
    n = _make_node([])
    n._stages.addressed("u1")

    n._on_tool_stage("navigate_to_waypoint", {"name": "x"}, None)
    assert _stages(n)[-1] == ("executing", "started")
    n._on_tool_stage("navigate_to_waypoint", {"name": "x"},
                     SimpleNamespace(is_error=True, content='{"error": "waypoint_not_found"}'))
    n._stages.finish(n._stages.turn_id_for("u1"))

    assert _stages(n)[-1] == ("failed", "failed") and n.events[-1]["reason"] == "waypoint_not_found"


def test_executor_and_sound_answers_reach_the_log():
    n = _make_node([])
    n.get_logger = lambda: SimpleNamespace(info=lambda text: n.logs.append(text), warning=lambda *a: None)
    n.logs = []
    n._stages = TurnStages(lambda **kw: None, lambda **kw: None, log=n.logs.append)
    n._on_expression_event(SimpleNamespace(executor="face", turn_id="abcdef012345", kind=1, reason="unknown_scene"))
    n._on_sound_event(SimpleNamespace(kind=2, sound="ui_confirm", reason="busy", request_id="r1"))
    n._on_sound_event(SimpleNamespace(kind=0, sound="ui_confirm", reason="", request_id="r2"))
    assert any("executor=face" in line and "skipped reason=unknown_scene" in line for line in n.logs)
    assert any("ui_confirm" in line and "busy" in line for line in n.logs)
    assert sum("ui_confirm" in line for line in n.logs) == 1        # started в лог диалога не пишется


def test_speech_event_moves_the_turn_to_speaking_and_done(monkeypatch):
    n = _make_node([])
    n._stages.addressed("u1")
    n._stages.tool_call("speak_text", {"text": "привет", "animation": "happy"})
    n._stages.finish(n._stages.turn_id_for("u1"))
    n._on_speech_event(SimpleNamespace(kind=0, agent="personality", reason="", speech_id="s1"))
    n._on_speech_event(SimpleNamespace(kind=2, agent="personality", reason="", speech_id="s1"))
    n._on_speech_event(SimpleNamespace(kind=4, agent="personality", reason="", speech_id="s1"))
    assert _stages(n)[-2:] == [("speaking", "started"), ("done", "finished")]


def test_greeting_earcons_go_through_the_expression_request(monkeypatch):
    n = _make_node([])
    sent = []
    monkeypatch.setattr(expression_client, "express", lambda node, **kw: sent.append(kw))
    n._express_earcon("heard")
    assert sent == [{"turn_id": "", "epoch": 0, "agent": "personality", "earcon": "heard"}]


def test_dialogue_node_has_no_sound_trigger_publisher():
    import inspect

    from rob_box_voice import dialogue_node

    source = inspect.getsource(dialogue_node)
    assert "/voice/sound/trigger" not in source and "_sound_trigger_pub" not in source
