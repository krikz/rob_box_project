"""#4151: хост выступления и очередь ``/voice/speech/request`` — длинное выступление не теряет запросов речи.

Сказка n412 (28.10): 13 ``SpeechRequest`` пред-синтеза за один вызов хоста, очередь KEEP_LAST 10 — сегмент 1 до
владельца речи не дошёл, выступление ждало его вечно. Здесь нода-двойник держит очередь той глубины, что просит
``speech_client`` у ``create_publisher`` (KEEP_LAST: из пачки одного вызова хоста доходят последние ``depth``), а
владелец речи-двойник отвечает на всё, что дошло: ``synthesized`` пред-синтезу, ``started``/``finished`` звучанию."""

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
TIME = type(M.SpeechEvent().stamp)
REQUEST = "/voice/speech/request"
STARTED = {"ok": True, "started": True, "track_id": "trk1", "bpm": 136.0, "beat_at": 1002.0, "bar_beats": 4.0,
           "start_beat": 32.0, "form_beats": 64.0}


def _segments(n: int) -> tuple:
    return tuple(f"Часть {i}: шла Красная Шапочка лесом к бабушке." for i in range(1, n + 1))


class _QueuePub:
    """Публикатор с очередью KEEP_LAST ``depth``: пачка одного вызова хоста — до подписчика доходят последние."""

    def __init__(self, depth: int) -> None:
        self.depth = depth
        self.pending: List[Any] = []
        self.lost: List[Any] = []

    def publish(self, msg: Any) -> None:
        self.pending.append(msg)

    def drain(self) -> List[Any]:
        burst, self.pending = self.pending, []
        self.lost += burst[:-self.depth]
        return burst[-self.depth:]


class _Node:
    def __init__(self) -> None:
        self.pubs: Dict[str, _QueuePub] = {}
        self.now_s = 1000.0
        self.logs: List[str] = []

    def create_publisher(self, msg_type: Any, topic: str, depth: int) -> _QueuePub:
        return self.pubs.setdefault(topic, _QueuePub(depth))

    def get_clock(self) -> Any:
        ns = int(self.now_s * 1e9)
        stamp = TIME(sec=ns // 10**9, nanosec=ns % 10**9)
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=ns, to_msg=lambda: stamp))

    def get_logger(self) -> Any:
        return types.SimpleNamespace(info=self.logs.append)

    def queue(self) -> _QueuePub:
        return self.pubs[REQUEST]


class _Tools:
    async def run(self, call: Any) -> ToolResult:
        return ToolResult(call.id, "ok", data=STARTED if call.name == "request_music" else {"ok": True})


@pytest.fixture
def loop():
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    yield loop
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)


def _settle(loop: Any) -> None:
    asyncio.run_coroutine_threadsafe(asyncio.sleep(0.05), loop).result(timeout=2)


def _text(msg: Any) -> str:
    return re.sub(r"<[^>]+>", "", msg.text).strip()


def _perform(loop: Any, n: int) -> tuple:
    """Сказка на ``n`` сегментов под calm, синтез MiniMax: что дошло до владельца речи и что прозвучало."""
    node, events = _Node(), []
    host = PerformHost(node, _Tools().run, lambda: loop, events.append, kn.SPEECH_OUTPUT_LATENCY_MS,
                       clock=lambda: node.now_s, tts_provider=lambda: "minimax")
    host.start(Performance(kind="tale", segments=_segments(n), backing="calm"), "t1")
    _settle(loop)  # подложка встала — «Поехали!» и сегменты уходят в пред-синтез
    delivered, heard = [], []
    inbox = node.queue().drain()
    while inbox:
        msg = inbox.pop(0)
        delivered.append(msg)
        node.now_s += 0.5
        if not msg.commit:
            host.on_speech("synthesized", msg.speech_id, duration_ms=3000)
        else:
            heard.append(_text(msg))
            host.on_speech("started", msg.speech_id, duration_ms=3000)
            node.now_s += 3.0
            host.tick()
            host.on_speech("finished", msg.speech_id, spoken_chars=len(_text(msg)))
        inbox += node.queue().drain()
    return node, events, delivered, heard


@pytest.mark.parametrize("n", [12, 11, 4])
def test_long_tale_reaches_the_speech_owner_whole_and_sounds_in_order(loop, n):
    node, events, delivered, heard = _perform(loop, n)
    assert node.queue().depth == 10  # очередь, которую просит speech_client (28.10)
    assert node.queue().lost == []
    assert sorted(_text(m) for m in delivered if not m.commit) == sorted(
        [phrases.text("perform.backing_started"), *_segments(n)])
    assert heard == [phrases.text("perform.backing_started"), *_segments(n)]
    assert [e.segment_no for e in events if e.kind == "segment_started"] == list(range(1, n + 1))
    assert [e.reason for e in events if e.kind == "block_end"] == ["done"]


def test_without_the_window_the_same_queue_loses_segments_as_on_the_robot(loop, monkeypatch):
    """Проверка модели очереди: окно шире очереди (как до #4151) — пачка из 13 теряет запросы, сказка не звучит."""
    monkeypatch.setattr(kn, "PERFORM_PRESYNTH_WINDOW", 100)
    node, events, _, heard = _perform(loop, 12)
    assert len(node.queue().lost) == 3  # pure KEEP_LAST: из 13 дошли последние 10
    assert heard == [] and not [e for e in events if e.kind == "segment_started"]
