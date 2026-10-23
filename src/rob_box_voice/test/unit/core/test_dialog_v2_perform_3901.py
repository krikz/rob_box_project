"""#3901: команда ``perform`` в dialog v2 — выступлению, а не репликами хода.

Живой робот 23.10 (dev-93343f9, dialog_engine v2): DeepSeek ответил ``command`` с ``perform{kind=rap, backing=beat}``
и слотами лица не на своих местах (``fields_moved``), мост сказал ``verdict=command``, а ``TurnEngine`` отдал сегменты
в ``say`` по одному: семь реплик ``voice=default``, без бита и без ``perform: разметка пауз …``. Здесь — та же
дорога целиком: провайдер-двойник с ответом того же вида → мост команд (``CommandModeLLM``) → ``V2Ports`` →
``TurnEngine`` → настоящий ``PerformHost`` (тулы музыки и речь — двойники)."""

from __future__ import annotations

import asyncio
import threading
import types
from typing import Any, Dict, List

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()

from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog.address import VOICE, Addressed  # noqa: E402
from rob_box_dialog.session import Session  # noqa: E402
from rob_box_dialog.turn import TurnEngine  # noqa: E402
from rob_box_llm.provider import LLMResponse, ToolCall, ToolResult  # noqa: E402
from rob_box_llm.providers.deepseek import DeepSeekProvider  # noqa: E402
from rob_box_voice.core import plan_host  # noqa: E402
from rob_box_voice.core.command_bridge import CommandModeLLM  # noqa: E402
from rob_box_voice.core.dialog_v2_ports import CallCounter, V2Ports  # noqa: E402
from rob_box_voice.core.perform_host import PerformHost  # noqa: E402

TIME = type(M.SpeechEvent().stamp)
SPEECH = "/voice/speech/request"

#: Рэп про енота — строки из лога робота 23.10 (сокращено до трёх), лицо — как его кладёт DeepSeek: слоты
#: ``mouth``/``tint``/``rear`` рядом с ``emotion`` (мост переносит их в ``face`` — ``fields_moved``).
SEGMENTS = ["Йо, микрофон включён, ночь во дворе, и главный тут — енот в своём домино.",
            "Он идёт по крыше, лапы как пружина, полосатый хвост — это его визитка-шина.",
            "Так что не злись, если пропал твой бутерброт — это не вор, это легендарный енот."]
RAP_COMMAND = {
    "perform": {
        "kind": "rap", "backing": "beat", "segments": SEGMENTS,
        "expression_per_segment": [{"emotion": "happy", "mouth": "smile", "tint": "warm", "rear": "pulse"},
                                   {"emotion": "happy", "mouth": "open", "tint": "warm"},
                                   {"emotion": "happy", "mouth": "smile", "tint": "cool"}],
    },
    "expression": {"emotion": "happy", "mouth": "smile", "tint": "warm", "rear": "pulse", "tempo": "fast"},
}
STARTED = {"ok": True, "started": True, "track_id": "beat1", "bpm": 120.0, "beat_at": 1002.0, "bar_beats": 4.0,
           "start_beat": 32.0, "form_beats": 64.0}


class _Pub:
    def __init__(self) -> None:
        self.sent: List[Any] = []

    def publish(self, msg: Any) -> None:
        self.sent.append(msg)


class _Node:
    """Нода-двойник: публикаторы, ROS-часы, логгер; ``_tool_executor.performer`` — как у ``dialog_node``."""

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
        return types.SimpleNamespace(info=self.logs.append, warning=self.logs.append, error=self.logs.append)

    def said(self) -> List[str]:
        return [m.text for m in self.pubs.get(SPEECH, _Pub()).sent]


class _Deepseek:
    """Провайдер с возможностями DeepSeek (режим ``tool_choice``) и одним ответом — вызовом ``command``."""

    name = "deepseek"

    def __init__(self, reply: LLMResponse) -> None:
        self._reply = reply
        self.capabilities = DeepSeekProvider(api_key="test").capabilities

    def capabilities_for(self, model: Any) -> Any:
        return self.capabilities

    async def complete(self, messages: Any, *, tools: Any = (), settings: Any = None) -> LLMResponse:
        return self._reply


class _Music:
    """Тулы музыки хоста выступления: ``request_music`` подложки — бит встал с сеткой."""

    def __init__(self) -> None:
        self.calls: List[Any] = []

    async def run(self, call: Any) -> ToolResult:
        self.calls.append(call)
        return ToolResult(call.id, "ok", data=STARTED if call.name == "request_music" else {"ok": True})


class _Tools:
    """Исполнитель тулов хода (действий у рэпа нет) с хостом выступления, как ``PlanToolExecutor``."""

    performer: Any = None

    async def execute(self, call: Any) -> ToolResult:
        raise AssertionError(f"у рэпа нет действий, а исполняется {call.name}")


@pytest.fixture
def loop():
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    yield loop
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)


def _turn(loop: Any, reply: LLMResponse, tts: str = "minimax") -> tuple:
    node, music, logs = _Node(), _Music(), []
    node._tool_executor = _Tools()
    node._tool_executor.performer = PerformHost(node, music.run, lambda: loop, lambda event: None,
                                                kn.SPEECH_OUTPUT_LATENCY_MS, clock=lambda: node.now_s,
                                                tts_provider=lambda: plan_host._tts_provider(node))
    plan_host.on_tts_provider_state(node, types.SimpleNamespace(data='{"provider": "%s"}' % tts))
    counter = CallCounter()
    llm = CommandModeLLM(_Deepseek(reply), log=logs.append)
    ports = V2Ports(node, llm, counter, node._tool_executor, performer=lambda: plan_host.performer(node))
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    agg = asyncio.run(engine.run(Addressed("зачитай рэп про енота под бит", VOICE)))
    asyncio.run_coroutine_threadsafe(asyncio.sleep(0.05), loop).result(timeout=2)  # исход заказа подложки
    return agg, node, music, logs


def _rap_reply() -> LLMResponse:
    return LLMResponse(content="", tool_calls=(ToolCall(id="call_0", name=kn.COMMAND_TOOL, arguments=RAP_COMMAND),),
                       finish_reason="tool_calls")


def test_3901_rap_with_beat_goes_to_performer_not_to_turn_speech(loop):
    """Живой ответ DeepSeek: мост — ``verdict=command`` с переносом слотов лица; ход отдаёт выступление хосту —
    бит заказан подложкой, куплеты MiniMax размечены паузами по такту сетки, репликами хода они не звучат."""
    agg, node, music, logs = _turn(loop, _rap_reply())
    bridge = [line for line in logs if "verdict=command" in line]
    assert bridge and "fields_moved=" in bridge[0] and "perform.expression_per_segment[0].mouth" in bridge[0]
    assert (agg.stage, agg.status) == ("done", "finished")
    (order,) = music.calls
    assert order.name == "request_music" and order.arguments["intent"] == kn.BACKING_INTENT
    assert any("разметка пауз MiniMax по сетке строк, такт 2.00 с" in line for line in node.logs)
    host = plan_host.performer(node)
    requests = node.pubs[SPEECH].sent
    verses = [r for r in requests if host.owns(r.speech_id)]
    assert len(verses) >= len(SEGMENTS), "куплеты — реплики выступления (пред-синтез хоста)"
    assert all(r.commit is False for r in verses), "куплеты ждут тактов бита, а не звучат сразу"
    assert not [r for r in requests if not host.owns(r.speech_id)], "ход не читает сегменты сам"


def test_3901_without_minimax_performer_still_owns_the_rap_and_says_why_no_pauses(loop):
    """Синтез не MiniMax — выступление то же (бит, такты), только без пауз — с причиной в логе."""
    _, node, music, _ = _turn(loop, _rap_reply(), tts="yandex")
    assert [c.name for c in music.calls] == ["request_music"]
    assert any("разметка пауз выключена" in line for line in node.logs)


def test_tts_provider_state_sets_actual_provider_and_ignores_garbage():
    node = types.SimpleNamespace(_actual_tts_provider=None)
    plan_host.on_tts_provider_state(node, types.SimpleNamespace(data="не json"))
    assert node._actual_tts_provider is None
    plan_host.on_tts_provider_state(node, types.SimpleNamespace(data='{"provider": "minimax", "voice": "x"}'))
    assert node._actual_tts_provider == "minimax" and plan_host._tts_provider(node) == "minimax"


def test_3973_block_end_of_owner_closes_the_request_in_the_engine_window(loop):
    """#3973: выступление стартовало ходом, владелец прислал ``block_end`` - просьба в окне LLM закрыта исходом
    (``stopped_by_user`` на «хватит»), а не висит неотвеченной."""
    from rob_box_dialog.plan.plan_events import PlanEvent  # noqa: PLC0415

    node, music = _Node(), _Music()
    node._tool_executor = _Tools()
    node._tool_executor.performer = PerformHost(node, music.run, lambda: loop, lambda event: None,
                                                kn.SPEECH_OUTPUT_LATENCY_MS, clock=lambda: node.now_s)
    llm = CommandModeLLM(_Deepseek(_rap_reply()), log=[].append)
    ports = V2Ports(node, llm, CallCounter(), node._tool_executor, performer=lambda: plan_host.performer(node))
    engine = node._engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    asyncio.run(engine.run(Addressed("зачитай рэп про енота под бит", VOICE)))
    (turn_id,) = [e for e in engine.log._turns]
    assert [dict(e.content)["status"] for e in engine.log.window() if e.role == "tool"] == ["started"]
    plan_host.note_perform_end(node, PlanEvent("segment_finished", "b1", turn_id=turn_id))  # не конец - не пишется
    plan_host.note_perform_end(node, PlanEvent("block_end", "b1", turn_id=turn_id, reason="stopped", segment_no=2))
    (done,) = [dict(e.content) for e in engine.log.window() if e.role == "tool"]
    assert done["status"] == "stopped_by_user" and done["data"] == {"kind": "rap", "segments_done": 2}


def test_3973_node_without_engine_ignores_block_end():
    from rob_box_dialog.plan.plan_events import PlanEvent  # noqa: PLC0415

    plan_host.note_perform_end(types.SimpleNamespace(), PlanEvent("block_end", "b1", reason="done"))  # v1: не падает
