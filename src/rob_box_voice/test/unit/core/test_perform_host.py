"""Хост выступления в ноде v1 (ADR-0150 §5.6.3, PR-7c ч.3б #3700): эффекты ``PerformBlock`` → ``SpeechRequest`` /
``SpeechCancel`` / тулы музыки, события владельцев → блок. Нода — двойник с публикаторами, тулы — корутина-двойник
в настоящем asyncio-loop (как loop ноды)."""

from __future__ import annotations

import asyncio
import html
import re
import threading
import types
from typing import Any, Dict, List

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import Performance
from rob_box_dialog.plan.rap_pauses import markup
from rob_box_llm.provider import ToolResult
from rob_box_voice.core.perform_host import PerformHost

M = install_dialog_msgs_stub()
#: Тип ``builtin_interfaces/Time`` той сборки сообщений, что под рукой (rosidl в CI, заглушка без ROS).
TIME = type(M.SpeechEvent().stamp)


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
        ns = int(self.now_s * 1e9)
        stamp = TIME(sec=ns // 10**9, nanosec=ns % 10**9)
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=ns, to_msg=lambda: stamp))

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


def test_3700_performance_orders_the_beat_through_the_executor_owned_path(loop):
    """Бит под выступление проходит исполнитель тулов ноды своим путём (``execute_owned``), хотя тот же вызов из хода
    LLM он отклоняет: заказ кода выступления не ломается сужением схемы."""
    from rob_box_voice.core.plan_tools import PlanToolExecutor

    tools = _Tools(STARTED)

    class _Underlying:
        async def execute(self, call: Any) -> ToolResult:
            return await tools.run(call)

    executor, node, events = PlanToolExecutor(_Underlying()), _Node(), []
    host = PerformHost(node, executor.execute_owned, lambda: loop, events.append, kn.SPEECH_OUTPUT_LATENCY_MS,
                       clock=lambda: node.now_s)
    host.start(RAP, "t1")
    _settle(loop)
    (order,) = tools.calls
    assert order.name == "request_music" and order.arguments["intent"] == kn.BACKING_INTENT
    assert host.owns_backing  # ответ тула дошёл до блока: started с сеткой


def test_busy_music_reads_over_it_without_a_second_try(loop):
    """Музыка встала между вопросом и заказом (``busy`` от движка): второй попытки нет, чтение поверх неё без
    фразы об отказе бита (музыка звучит — это не тишина)."""
    host, node, tools, _ = _host(loop, {"ok": False, "reason": "busy"})
    host.start(RAP, "t1")
    _settle(loop)
    assert len(tools.calls) == 1
    assert [r for r in node.requests() if r.commit] == [] and not host.owns_backing


def _asking(loop: Any, backing: Dict[str, Any]) -> tuple:
    """Играет чужая музыка: выступление спрашивает «выключить и зачитать?» — бит не заказан, окно — после вопроса."""
    node, tools, events, opened = _Node(), _Tools(backing), [], []
    host = PerformHost(node, tools.run, lambda: loop, events.append, kn.SPEECH_OUTPUT_LATENCY_MS,
                       clock=lambda: node.now_s, playing=lambda: "Still Dre", on_ask=opened.append)
    host.start(RAP, "t1")
    (question,) = [r for r in node.requests() if r.commit]
    assert plain_text(question.text) == phrases.text("perform.ask_replace_music", title="Still Dre")
    _settle(loop)
    assert tools.calls == [] and opened == []
    host.on_speech("started", question.speech_id, duration_ms=2500)
    host.on_speech("finished", question.speech_id)
    assert opened == ["Still Dre"]  # вопрос прозвучал — окно ответа открыто
    return host, node, tools


def test_yes_stops_the_playing_music_then_orders_the_beat(loop):
    host, node, tools = _asking(loop, STARTED)
    assert host.answer(True)
    _settle(loop)
    assert [c.name for c in tools.calls] == ["stop_music", "request_music"]
    assert tools.calls[0].arguments == {"boundary": "now"} and host.owns_backing


def test_no_reads_over_the_playing_music(loop):
    host, node, tools = _asking(loop, STARTED)
    assert host.answer(False)
    _settle(loop)
    assert tools.calls == [] and not host.owns_backing
    assert not host.answer(True)  # вопрос уже закрыт


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


def test_edit_pending_starts_at_the_first_not_released_segment_whatever_the_model_said(loop):
    """X3: с какого сегмента править, решает код (#3700, A20 22.10: DeepSeek называл звучащий) — номер в правке
    игнорируется, правятся не отданные на звучание."""
    host, node, _, events = _host(loop, STARTED)
    host.start(RAP, "t1")
    edit = host.edit(Performance(kind="rap", segments=("Енот пришёл.", "Енот ушёл."), backing="beat",
                                 from_segment=7))
    assert edit == {"status": "done"}
    assert [e.segment_no for e in events if e.kind == "segment_replaced"] == [1, 2]


def test_perform_with_backing_and_no_new_segments_adds_the_beat(loop):
    """A22 22.10 (#3700): «Робот, добавь бит» → DeepSeek шлёт ``perform{backing: beat, segments: []}`` поверх рэпа
    без бита — это «добавь бит» (#993): бит кодом, куплеты прежние."""
    host, node, tools, events = _host(loop, STARTED)
    host.start(Performance(kind="rap", segments=RAP.segments, backing="none"), "t1")
    _settle(loop)
    assert tools.calls == []
    out = host.start(Performance(kind="rap", segments=(), backing="beat"), "t2")
    _settle(loop)
    assert out == {"status": "done", "backing": True}
    assert [c.arguments["intent"] for c in tools.calls] == ["backing"]
    assert [e for e in events if e.kind == "segment_replaced"] == []
    again = host.edit(Performance(kind="rap", segments=(), backing="beat"))  # бит уже есть — второго нет
    _settle(loop)
    assert again["status"] == "rejected" and len(tools.calls) == 1


def test_edit_with_new_segments_and_backing_adds_the_beat_too(loop):
    """A22 22.10 a22_3: на «добавь бит» правка с новыми куплетами и ``backing: beat`` — куплеты правятся и бит есть."""
    host, node, tools, events = _host(loop, STARTED)
    host.start(Performance(kind="rap", segments=RAP.segments, backing="none"), "t1")
    out = host.edit(Performance(kind="rap", segments=("Бит пошёл.", "Комар в такт."), backing="beat"))
    _settle(loop)
    assert out == {"status": "done"} and [c.arguments["intent"] for c in tools.calls] == ["backing"]
    assert [e.segment_no for e in events if e.kind == "segment_replaced"] == [1, 2]


def test_context_while_performing_and_none_after(loop):
    host, node, _, _ = _host(loop, STARTED)
    assert host.context() is None and host.active_segment is None
    host.start(RAP, "t1")
    ctx = host.context()
    assert ctx["kind"] == "rap" and ctx["first_pending"] == 1 and ctx["segments_total"] == 2
    assert host.active_segment == 0


LINED_RAP = Performance(kind="rap", segments=("Я комар, звеню всю ночь,\nТы не спишь, а мне не лень.",
                                              "Прячусь за шторой,\nЖду темноты."), backing="beat")


def test_3901_minimax_verses_go_to_presynth_with_pauses_on_the_beat_grid(loop):
    """Провайдер TTS MiniMax и бит: куплеты уходят в пред-синтез после ``started`` — с паузами ``<#x#>`` по такту
    сетки (120 BPM — 2 с), экранированными в SSML."""
    node, tools, events = _Node(), _Tools(STARTED), []
    host = PerformHost(node, tools.run, lambda: loop, events.append, kn.SPEECH_OUTPUT_LATENCY_MS,
                       clock=lambda: node.now_s, tts_provider=lambda: "minimax")
    host.start(LINED_RAP, "t1")
    assert node.requests() == []  # сетки ещё нет — пред-синтез ждёт бит
    _settle(loop)
    texts = [html.unescape(plain_text(r.text)) for r in node.requests() if r.commit is False]
    assert texts[0] == phrases.text("perform.backing_started")
    assert texts[1:] == [markup(seg, 2.0).text for seg in LINED_RAP.segments]
    assert all("&lt;#" in r.text for r in node.requests()[1:])
    assert any("разметка пауз MiniMax по сетке строк, такт 2.00 с" in line for line in node.logs)


def test_3901_not_minimax_reads_as_before_and_says_why(loop):
    host, node, _, _ = _host(loop, STARTED)
    host.start(LINED_RAP, "t1")
    assert [plain_text(r.text) for r in node.requests()] == list(LINED_RAP.segments)  # сразу и без пауз
    assert any("разметка пауз выключена (синтез неизвестен, не MiniMax)" in line for line in node.logs)


# --- #4085: выступление снимает только свою подложку ---------------------------------------------------------------

TALE = Performance(kind="tale", segments=("Жил-был енот у реки.", "Он мыл бельё по ночам.", "И всё у него сияло."),
                   backing="none")


def _read_to_the_end(host: PerformHost, node: _Node) -> None:
    """Владелец речи: пред-синтез готов, каждая отданная на звучание реплика звучит и кончается."""
    for request in [r for r in node.requests() if not r.commit]:
        host.on_speech("synthesized", request.speech_id, duration_ms=700)
    heard: set = set()
    for _ in range(50):
        host.tick()
        due = [r for r in node.requests() if r.commit and r.speech_id not in heard]
        if not due:
            return
        for request in due:
            heard.add(request.speech_id)
            host.on_speech("started", request.speech_id, duration_ms=700)
            host.on_speech("finished", request.speech_id)


def _wired(loop: Any, backing: Dict[str, Any]) -> tuple:
    """Как в ноде (``plan_host.make_tool_executor``): исполнитель тулов хода и хост выступления на нём."""
    from rob_box_voice.core.plan_tools import PlanToolExecutor

    tools = _Tools(backing)

    class _Underlying:
        async def execute(self, call: Any) -> ToolResult:
            return await tools.run(call)

    executor, node, events = PlanToolExecutor(_Underlying()), _Node(), []
    executor.performer = PerformHost(node, executor.execute_owned, lambda: loop, events.append,
                                     kn.SPEECH_OUTPUT_LATENCY_MS, clock=lambda: node.now_s)
    return executor, executor.performer, node, tools


def test_4085_music_ordered_during_a_beatless_tale_is_the_persons_and_plays_on_after_it(loop):
    """Прослушивание PR-14 27.10 (#4085): сказка без бита; посреди неё «Робот, поставь фоном спокойный бит на Ренардо»
    — LLM зовёт ``request_music(intent=track)``. Раньше исполнитель тулов подменял вызов битом выступления
    (``intent=backing``, ``mood=groove``, длина — до конца сказки), и конец сказки через секунду звал
    ``stop_music{bar}``. Теперь вызов уходит как просил человек, выступление трек не присваивает, конец сказки музыку
    не трогает."""
    from rob_box_llm.provider import ToolCall

    executor, host, node, tools = _wired(loop, {"ok": True, "started": True, "track_id": "req11344:02:A"})
    host.start(TALE, "t1")
    _settle(loop)
    assert tools.calls == [] and not host.owns_backing
    asked = ToolCall(id="c1", name="request_music",
                     arguments={"intent": "track", "text": "спокойный бит на Ренардо", "engine": "renardo",
                                "mood": "calm"})
    result = asyncio.run(executor.execute(asked))
    host.on_music_event("started", "req11344:02:A", {"bpm": 136.0, "beat_at": 1001.0, "bar_beats": 4.0})
    assert result.is_error is False and result.data == {"ok": True, "started": True, "track_id": "req11344:02:A"}
    assert [(c.name, c.arguments) for c in tools.calls] == [("request_music", asked.arguments)]
    assert not host.owns_backing and host.block.backing.state == "none"
    _read_to_the_end(host, node)
    _settle(loop)
    assert host.block.ended and host.block.end_reason == "done"
    assert [c.name for c in tools.calls] == ["request_music"]  # stop_music нет: трек человека играет дальше


def test_4085_own_beat_is_outroed_at_the_end_of_the_performance(loop):
    """Бит, который заказало само выступление (``Performance.backing``), снимается его концом — outro на такте."""
    executor, host, node, tools = _wired(loop, STARTED)
    host.start(RAP, "t1")
    _settle(loop)
    assert host.owns_backing
    _read_to_the_end(host, node)
    _settle(loop)
    assert host.block.ended and host.block.end_reason == "done"
    assert [(c.name, c.arguments.get("boundary")) for c in tools.calls][1:] == [("stop_music", "bar")]


def test_4085_foreign_started_before_the_tool_answer_is_not_the_beat(loop):
    """Плеер сообщил о чужом треке, пока бит выступления ещё не встал: исход своего запроса — только ответ тула
    (#4085: владение по ``track_id``), чужой трек своим не становится."""
    host, node, tools, _ = _host(loop, {"ok": False, "reason": "busy"})
    host.start(RAP, "t1")
    host.on_music_event("started", "person", {"bpm": 90.0, "beat_at": 1001.0, "bar_beats": 4.0})
    assert host.block.backing.track_id == "" and host.block.backing.grid is None
    _settle(loop)
    assert host.block.backing.state == "failed" and not host.owns_backing  # ответ тула: busy — читаем поверх
    _read_to_the_end(host, node)
    _settle(loop)
    assert host.block.ended and [c.name for c in tools.calls] == ["request_music"]
