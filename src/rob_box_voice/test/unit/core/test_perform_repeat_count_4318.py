"""#4318: повторный «посчитай от одного до сорока», когда первый счёт отдан на звучание целиком, — новое выступление.

Живой Sound Pi 04.11 02:23:51 UTC: второй запрос пришёл, пока звучал последний сегмент первого счёта («Тридцать один…»).
Хост отдал его в правку (``PerformHost.start`` → MERGE), правка целиком повторяла сказанное — отказ
``edit_repeats_said``, «Этот куплет уже пошёл — дочитаю как было.» и следом «Не вышло.». Править было нечего: все
сегменты уже отданы на звучание. Теперь правка — только пока есть не отданные сегменты; иначе прежнее досказывает
владелец речи, а новая просьба — новое выступление за ним. Отказ, который озвучило выступление, ход не дублирует."""

from __future__ import annotations

import asyncio
import re
import threading
import types
from typing import Any, Dict, List, Optional

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import Command, Performance
from rob_box_dialog.turn import hand_perform
from rob_box_llm.provider import ToolResult
from rob_box_voice.core.perform_host import PerformHost

M = install_dialog_msgs_stub()
TIME = type(M.SpeechEvent().stamp)
REQUEST = "/voice/speech/request"

# Счёт кодом (#4317) до сорока — четыре сегмента, как в живом логе.
COUNT = tuple(" ".join(f"{n}." for n in range(lo, lo + 10)) for lo in (1, 11, 21, 31))


class _Pub:
    def __init__(self) -> None:
        self.pending: List[Any] = []

    def publish(self, msg: Any) -> None:
        self.pending.append(msg)


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


async def _tool(call: Any) -> ToolResult:
    return ToolResult(call.id, "ok", data={"ok": True})


@pytest.fixture
def loop():
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    yield loop
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)


def _text(msg: Any) -> str:
    return re.sub(r"<[^>]+>", "", msg.text).strip()


class _Stand:
    """Хост выступления и владелец речи-двойник: пред-синтез отвечает ``synthesized``, звучание — ``started``, а
    ``finished`` — когда стенд доведёт реплику (:meth:`finish`)."""

    def __init__(self, loop: Any) -> None:
        self.node, self.events = _Node(), []
        self.host = PerformHost(self.node, _tool, lambda: loop, self.events.append, kn.SPEECH_OUTPUT_LATENCY_MS,
                                clock=lambda: self.node.now_s)
        self.heard: List[str] = []
        self.sounding: Optional[Any] = None

    def pump(self, stop_at: str = "") -> None:
        """Отвечать владельцем речи; реплика ``stop_at`` начинает звучать и остаётся звучать (без ``finished``)."""
        while True:
            inbox, self.node.pubs[REQUEST].pending = self.node.pubs[REQUEST].pending, []
            if not inbox:
                self.node.now_s += 0.2
                self.host.tick()
                if not self.node.pubs[REQUEST].pending:
                    return
                continue
            for msg in inbox:
                self.node.now_s += 0.1
                if not msg.commit:
                    self.host.on_speech("synthesized", msg.speech_id, duration_ms=3000)
                    continue
                self.heard.append(_text(msg))
                self.host.on_speech("started", msg.speech_id, duration_ms=3000)
                self.sounding = msg
                if _text(msg) == stop_at:
                    return
                self.finish()

    def finish(self) -> None:
        msg, self.sounding = self.sounding, None
        self.node.now_s += 3.0
        self.host.on_speech("finished", msg.speech_id, spoken_chars=len(_text(msg)))

    def kinds(self, kind: str) -> List[Any]:
        return [e for e in self.events if e.kind == kind]


def _count() -> Performance:
    return Performance("poem", COUNT, backing="none")


def test_same_count_while_the_last_segment_sounds_is_a_new_performance(loop):
    """Живой сценарий #4318: «посчитай до сорока» → звучит последний сегмент → тот же запрос — новое выступление."""
    stand = _Stand(loop)
    assert stand.host.start(_count(), "t1")["status"] == "pending"
    stand.pump(stop_at=COUNT[-1])  # «31. … 40.» звучит, несказанного нет
    first = stand.host.block
    assert not first.unsaid and not first.ended

    outcome = stand.host.start(_count(), "t2")

    assert outcome["status"] == "pending", outcome
    assert first.ended and first.end_reason == "done"
    assert stand.host.block is not first and stand.host.block.block_id == "perform-t2"
    assert stand.kinds("rejected") == []
    stand.finish()  # прежний последний сегмент дочитан владельцем речи
    stand.pump()
    assert stand.heard == [*COUNT, *COUNT]  # счёт целиком дважды, без «дочитаю как было»
    assert phrases.text("perform.edit_rejected") not in stand.heard
    assert [e.reason for e in stand.kinds("block_end")] == ["done", "done"]


def test_same_count_after_the_end_is_a_new_performance(loop):
    stand = _Stand(loop)
    stand.host.start(_count(), "t1")
    stand.pump()
    assert stand.host.block.ended
    assert stand.host.start(_count(), "t2")["status"] == "pending"
    stand.pump()
    assert stand.heard == [*COUNT, *COUNT]


def test_edit_flag_repeating_everything_said_with_nothing_unsaid_is_a_new_performance(loop):
    """«Посчитай ещё раз» модель прислала с ``edit_pending`` — несказанного нет, правка целиком повтор: новое."""
    stand = _Stand(loop)
    stand.host.start(_count(), "t1")
    stand.pump(stop_at=COUNT[-1])
    assert stand.host.edit(_count(), "t2", 0)["status"] == "pending"
    assert stand.host.block.block_id == "perform-t2"
    assert stand.kinds("rejected") == []


def test_continuation_with_edit_flag_while_the_last_segment_sounds_still_appends(loop):
    """Новое содержимое правкой, пока звучит последний сегмент, — продолжение в конец того же выступления."""
    stand = _Stand(loop)
    stand.host.start(_count(), "t1")
    stand.pump(stop_at=COUNT[-1])
    first = stand.host.block
    assert stand.host.edit(Performance("poem", ("Вот и всё, досчитали.",), backing="none"))["status"] == "done"
    assert stand.host.block is first
    stand.finish()
    stand.pump()
    assert stand.heard == [*COUNT, "Вот и всё, досчитали."]


def test_new_request_mid_count_still_edits_the_unsaid_segments(loop):
    """Пока есть несказанные сегменты — новая просьба по-прежнему правка (MERGE, #3700/#4207)."""
    stand = _Stand(loop)
    stand.host.start(_count(), "t1")
    stand.pump(stop_at=COUNT[1])  # звучит второй, третий и четвёртый — не отданы
    first = stand.host.block
    assert first.unsaid
    tail = ("21 — шаг. 22 — шаг.", "Все дошли.")
    assert stand.host.start(Performance("poem", tail, backing="none"), "t2")["status"] == "done"
    assert stand.host.block is first
    assert stand.kinds("segment_replaced")


def test_rejected_edit_mid_count_speaks_one_phrase_not_two(loop):
    """Правка посреди счёта целиком повторяет сказанное — отказ озвучивает выступление («дочитаю как было»), ход
    своё «Не вышло.» не добавляет: одна фраза кода по исходу."""
    stand = _Stand(loop)
    stand.host.start(_count(), "t1")
    stand.pump(stop_at=COUNT[1])
    perf = Performance("poem", COUNT[:2], backing="none")
    outcome = stand.host.edit(perf)
    assert outcome == {"status": "rejected", "reason": "edit_repeats_said", "phrased": True}

    ports = types.SimpleNamespace(perform=lambda turn_id, epoch, p, edit: outcome)
    log = types.SimpleNamespace(info=lambda text: None)
    assert hand_perform(ports, "t2", 0, Command(perform=perf, edit_pending=True), log) == (True, "")


def test_rejection_without_a_phrase_of_the_performance_still_says_not_done():
    ports = types.SimpleNamespace(perform=lambda turn_id, epoch, p, edit: {"status": "rejected", "reason": "x"})
    log = types.SimpleNamespace(info=lambda text: None)
    perf = Performance("poem", COUNT, backing="none")
    assert hand_perform(ports, "t", 0, Command(perform=perf), log) == (True, phrases.text("local.rejected"))
