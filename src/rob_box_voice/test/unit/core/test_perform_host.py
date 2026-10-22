"""Хост выступления в ноде v1 (ADR-0150 §5.6.3, PR-7c ч.3б #3700): эффекты ``PerformBlock`` → ``SpeechRequest`` /
``SpeechCancel`` / тулы музыки, события владельцев → блок. Нода — двойник с публикаторами, тулы — корутина-двойник
в настоящем asyncio-loop (как loop ноды)."""

from __future__ import annotations

import asyncio
import re
import threading
import types
from typing import Any, Dict, List

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import Performance
from rob_box_llm.provider import ToolResult
from rob_box_voice.core.perform_host import PerformHost

M = install_dialog_msgs_stub()


def plain_text(ssml: str) -> str:
    """Текст запроса без SSML-обёртки ``Utterance``."""
    return re.sub(r"<[^>]+>", "", ssml).strip()


RAP = Performance(kind="rap", segments=("Я комар, звеню всю ночь. Ты не спишь.", "Прячусь за шторой. Жду темноты."),
                  backing="beat")


class _Pub:
    def __init__(self) -> None:
        self.sent: List[Any] = []

    def publish(self, msg: Any) -> None:
        self.sent.append(msg)


class _Node:
    def __init__(self) -> None:
        self.pubs: Dict[str, _Pub] = {}
        self.now_s = 1000.0
        self.logs: List[str] = []

    def create_publisher(self, msg_type: Any, topic: str, depth: int) -> _Pub:
        return self.pubs.setdefault(topic, _Pub())

    def get_clock(self) -> Any:
        node = self

        class _Now:
            nanoseconds = int(node.now_s * 1e9)

            @staticmethod
            def to_msg() -> Any:
                return types.SimpleNamespace(sec=int(node.now_s), nanosec=0)

        return types.SimpleNamespace(now=lambda: _Now())

    def get_logger(self) -> Any:
        return types.SimpleNamespace(info=self.logs.append)

    def requests(self) -> List[Any]:
        return list(self.pubs.get("/voice/speech/request", _Pub()).sent)

    def cancels(self) -> List[Any]:
        return list(self.pubs.get("/voice/speech/cancel", _Pub()).sent)


class _Tools:
    """Тулы музыки: ответ по сценарию, журнал вызовов."""

    def __init__(self, backing: Dict[str, Any]) -> None:
        self.backing = backing
        self.calls: List[Any] = []

    async def run(self, call: Any) -> ToolResult:
        self.calls.append(call)
        data = self.backing if call.name == "request_music" else {"ok": True}
        return ToolResult(call.id, "ok", data=data)


@pytest.fixture
def loop():
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    yield loop
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)


def _host(loop: Any, backing: Dict[str, Any]) -> tuple:
    node, tools, events = _Node(), _Tools(backing), []
    host = PerformHost(node, tools.run, lambda: loop, events.append, kn.SPEECH_OUTPUT_LATENCY_MS,
                       clock=lambda: node.now_s)
    return host, node, tools, events


def _settle(loop: Any) -> None:
    asyncio.run_coroutine_threadsafe(asyncio.sleep(0.05), loop).result(timeout=2)


STARTED = {"ok": True, "started": True, "track_id": "trk1", "bpm": 120.0, "beat_at": 1002.0, "bar_beats": 4.0,
           "start_beat": 32.0, "form_beats": 64.0}


def test_start_presynthesizes_segments_and_orders_the_backing(loop):
    host, node, tools, events = _host(loop, STARTED)
    assert host.start(RAP, "t1")["status"] == "pending"
    presynth = node.requests()
    assert [(plain_text(r.text), r.commit, r.boundary) for r in presynth[:2]] == [
        (RAP.segments[0], False, 1), (RAP.segments[1], False, 1)]  # граница предложения — для «хватит»
    assert all(host.owns(r.speech_id) for r in presynth)
    _settle(loop)
    (order,) = tools.calls
    assert order.name == "request_music" and order.arguments["intent"] == "backing"
    assert order.arguments["mood"] == kn.PERFORM_BACKING_MOODS["beat"][0] and order.arguments["min_seconds"] > 0
    lead = node.requests()[-1]  # бит встал — «Поехали!» уходит в пред-синтез
    assert plain_text(lead.text) == phrases.text("perform.backing_started") and lead.commit is False
    assert events[0].kind == "block_start" and host.owns_backing


def test_segment_goes_on_the_bar_after_synthesis(loop):
    host, node, _, _ = _host(loop, STARTED)
    host.start(RAP, "t1")
    _settle(loop)
    lead = node.requests()[-1]
    host.on_speech("synthesized", lead.speech_id, duration_ms=700)
    commit = node.requests()[-1]
    assert commit.speech_id == lead.speech_id and commit.commit is True
    assert commit.at == pytest.approx(1002.0)  # первая доля бита (самый ранний звук 1000 + 0.55 + 0.3 < 1002)


def test_busy_music_reads_without_beat_and_says_so_without_a_second_try(loop):
    host, node, tools, _ = _host(loop, {"ok": False, "reason": "busy"})
    host.start(RAP, "t1")
    _settle(loop)
    assert len(tools.calls) == 1  # вторая попытка бессмысленна: играет другая музыка
    said = [plain_text(r.text) for r in node.requests() if r.commit]
    assert said == [phrases.text("perform.no_backing.busy")]
    assert not host.owns_backing


def test_stop_finishes_the_sentence_halt_cuts_now(loop):
    host, node, _, _ = _host(loop, STARTED)
    host.start(RAP, "t1")
    _settle(loop)
    lead = node.requests()[-1]
    host.on_speech("synthesized", lead.speech_id, duration_ms=700)
    host.on_speech("started", lead.speech_id, duration_ms=700)
    assert host.stop()
    (cancel_active, *_rest) = node.cancels()
    assert cancel_active.speech_id == lead.speech_id and cancel_active.boundary == M.SpeechCancel.BOUNDARY_AS_REQUESTED
    host2, node2, tools2, _ = _host(loop, STARTED)
    host2.start(RAP, "t2")
    _settle(loop)
    lead2 = node2.requests()[-1]
    host2.on_speech("synthesized", lead2.speech_id, duration_ms=700)
    host2.on_speech("started", lead2.speech_id, duration_ms=700)
    assert host2.halt()
    assert node2.cancels()[0].boundary == M.SpeechCancel.BOUNDARY_NOW
    _settle(loop)
    assert [c.arguments for c in tools2.calls if c.name == "stop_music"] == [{"boundary": "now"}]
    assert host2.active_segment is None and not host2.halt()  # выступления больше нет


def test_nearly_finished_extends_the_beat(loop):
    host, node, tools, _ = _host(loop, STARTED)
    host.start(RAP, "t1")
    _settle(loop)
    host.on_music_event("nearly_finished", "trk1", {"form_end_beat": 34.0, "lead_beats": 8})
    _settle(loop)
    extend = [c for c in tools.calls if c.name == "request_music"][-1]
    assert len([c for c in tools.calls if c.name == "request_music"]) == 2 and extend.arguments["intent"] == "backing"


def test_rejected_comes_only_from_the_tool_result(loop):
    """Отказ подложки — из ответа тула; ``rejected`` из топика блок не получает (иначе второй экземпляр того же отказа
    съел бы вторую попытку)."""
    host, node, tools, _ = _host(loop, {"ok": False, "reason": "compose_error"})
    host.start(RAP, "t1")
    host.on_music_event("rejected", None, {"reason": "compose_error"})
    _settle(loop)
    assert len([c for c in tools.calls if c.name == "request_music"]) == 2  # вторая попытка — по ответу тула
    said = [plain_text(r.text) for r in node.requests() if r.commit]
    assert said == [phrases.text("perform.no_backing.error")]


def test_own_started_twice_is_one_start_and_foreign_started_ends_the_bars(loop):
    """Свой ``started`` из топика после ответа тула — повтор (вступление одно); чужой ``started`` — деку заняла другая
    музыка: тактов бита больше нет, дальше чтение без них (честный ``synced=false``)."""
    host, node, _, _ = _host(loop, STARTED)
    host.start(RAP, "t1")
    _settle(loop)
    host.on_music_event("started", "trk1", STARTED)
    leads = [r for r in node.requests() if plain_text(r.text) == phrases.text("perform.backing_started")]
    assert len(leads) == 1
    host.on_music_event("started", "someone_elses", {"bpm": 90.0, "beat_at": 1003.0, "bar_beats": 4.0})
    assert host.block.backing.grid is None and not host.owns_backing


def test_new_performance_over_a_running_one_merges_into_not_started_segments(loop):
    host, node, _, events = _host(loop, STARTED)
    host.start(RAP, "t1")
    again = host.start(Performance(kind="rap", segments=("Енот пришёл.", "Енот ушёл."), backing="beat"), "t2")
    assert again == {"status": "done"}
    assert [e.segment_no for e in events if e.kind == "segment_replaced"] == [1, 2]


def test_context_while_performing_and_none_after(loop):
    host, node, _, _ = _host(loop, STARTED)
    assert host.context() is None and host.active_segment is None
    host.start(RAP, "t1")
    ctx = host.context()
    assert ctx["kind"] == "rap" and ctx["first_pending"] == 1 and ctx["segments_total"] == 2
    assert host.active_segment == 0
