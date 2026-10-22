"""ТАРС на общем движке (ADR-0150 §9.3, В7; PR-13 #3708): ход оператора — ``TurnEngine`` с ``AgentSpec(operator)``.

Проверяем поведение модуля, а не текст промпта (ADR-0148): ответ оператору строит код из хода, ``say`` — обещание
до ``SpeechEvent``, «Озвучил (N с)» — поздний итог тем же запросом; отказ без действия — не исполнен и не выдуман.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.execute import Executor, OwnerEvent
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_supervisor import operator_turns as ot

FINISHED, REJECTED = 4, 7


class FakeBase:
    """Общие порты v2 в тесте: LLM — заданные команды, владелец действий отвечает ``outcome`` (``pending`` у ``say``)."""

    def __init__(self, commands, outcome=("pending", None)):
        self._commands = list(commands)
        self._outcome = outcome
        self.on_late = None
        self.cancelled = []
        self.executor = Executor(self._send, on_late=lambda r: self.on_late and self.on_late(r))

    async def ask(self, messages, tools, timeout_s):
        self.tools = [t.name for t in tools]
        return Answer(self._commands.pop(0), "fake", 1, 10)

    async def execute(self, actions, snapshot, turn_id):
        return await self.executor.run(actions, snapshot, turn_id)

    async def _send(self, request):
        status, reason = self._outcome
        ref = f"speech-{request.turn_id}" if status == "pending" else None
        self.executor.on_event(OwnerEvent(request.turn_id, status, reason, {"text": request.args.get("text")},
                                          event_ref=ref))

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        self.cancelled.append(reason)

    def intent(self, turn_id, epoch, kind):
        pass


def _host(commands, outcome=("pending", None)):
    base = FakeBase(commands, outcome)
    ports = ot.OperatorPorts(base, log=lambda _line: None)
    base.on_late = ports.late
    session = Session(ot.OPERATOR, clock=time.time)
    engine = TurnEngine(session, ports, system_prompt="ТАРС")
    host = ot.OperatorHost(engine, ports, base.executor, log=lambda _line: None)
    late = []
    ports.on_late = lambda rid, result, text: late.append((rid, result.status, text))
    return host, base, late


@pytest.fixture
def stop_hosts():
    hosts = []
    yield hosts
    for host in hosts:
        host.shutdown()


def test_operator_sees_say_and_the_personality_does_not():
    snap = Session(ot.OPERATOR).snapshot
    assert "say" in {t.name for t in available_tools(snap, None, ot.OPERATOR, "скажи людям привет")}
    assert "say" not in {t.name for t in available_tools(Session("personality").snapshot, None, "personality", "")}
    assert "say_aloud" not in {t.name for t in available_tools(snap, None, ot.OPERATOR, "")}


def test_reply_is_built_by_code_from_the_turn(stop_hosts):
    host, _base, _late = _host([Command(speech="Батарея в норме.")])
    stop_hosts.append(host)
    out = host.run_payload({"text": "как батарея", "source": "telegram", "client_id": "c", "request_id": "r1"})
    assert out == {"ok": True, "summary": "Батарея в норме.", "tool_calls": [], "source": "telegram", "client_id": "c"}


def test_say_is_a_promise_then_late_result_by_speech_event(stop_hosts):
    """§9.3 / В7: ``say`` не «успех при публикации»: ход отдаёт ``pending``, «Озвучил (3 с)» — по SpeechEvent."""
    host, _base, late = _host([Command(actions=(ToolCall("say", {"text": "Внимание, люди"}),))])
    stop_hosts.append(host)
    out = host.run_payload({"text": "скажи людям внимание", "source": "quest", "request_id": "q:1"})
    assert out["ok"] is True
    assert out["tool_calls"] == [{"name": "say", "status": "pending", "reason": ""}]
    assert out["summary"] == ot.NO_REPLY  # речи модели при действии нет (§4.1), фразы о «pending» тоже
    ref = next(iter(host.ports._owners))
    speech_id = f"speech-{ref}"
    host.on_speech(SimpleNamespace(kind=FINISHED, speech_id=speech_id, agent=ot.OPERATOR, reason="", seconds=3.2,
                                   duration_ms=0))
    deadline = time.monotonic() + 2.0
    while not late and time.monotonic() < deadline:
        time.sleep(0.01)
    assert late == [("q:1", "done", "Озвучил (3 с).")]


def test_rejected_say_is_an_honest_failure_phrase_from_code(stop_hosts):
    host, _base, _late = _host([Command(actions=(ToolCall("say", {"text": "Привет"}),))], ("rejected", "busy"))
    stop_hosts.append(host)
    out = host.run_payload({"text": "скажи привет", "request_id": "r2"})
    assert out["tool_calls"] == [{"name": "say", "status": "rejected", "reason": "busy"}]
    assert out["summary"] == "Не озвучил: динамики заняты."


def test_empty_text_is_rejected_without_a_turn(stop_hosts):
    host, base, _late = _host([])
    stop_hosts.append(host)
    assert host.run_payload({"text": "  ", "request_id": "r3"})["summary"] == "empty_input"


def test_crashed_turn_is_loud_not_ok(stop_hosts):
    host, base, _late = _host([])  # LLM без ответов: ход падает
    stop_hosts.append(host)
    out = host.run_payload({"text": "что-нибудь", "request_id": "r4"})
    assert out["ok"] is False and out["summary"].startswith("llm_error")


def test_tier1_stop_cancels_operator_speech_without_llm(stop_hosts):
    host, base, _late = _host([])
    stop_hosts.append(host)
    out = host.run_payload({"text": "стоп", "request_id": "r5"})
    assert out["ok"] is True and out["summary"] == ot.NO_REPLY
    assert base.cancelled and not hasattr(base, "tools")  # LLM не звали


def test_agent_path_picks_engine_by_flag():
    node = SimpleNamespace(_build_agent_core_sync=lambda: ("core", "dsm"),
                           _run_agent_sync=lambda core, payload: {"core": core, **payload})
    build, run = ot.agent_path(node, "v1")
    assert build() == ("core", "dsm") and run("c", {"x": 1}) == {"core": "c", "x": 1}
    build, run = ot.agent_path(SimpleNamespace(), "v2")
    assert run(SimpleNamespace(run_payload=lambda p: {"v2": p}), {"y": 2}) == {"v2": {"y": 2}}


def test_operator_journal_is_a_known_snapshot_section():
    assert "operator_journal" in kn.CONTEXT_SECTIONS


def test_host_loop_survives_between_commands(stop_hosts):
    host, _base, _late = _host([Command(speech="Раз."), Command(speech="Два.")])
    stop_hosts.append(host)
    assert host.run_payload({"text": "раз", "request_id": "a"})["summary"] == "Раз."
    assert host.run_payload({"text": "два", "request_id": "b"})["summary"] == "Два."
