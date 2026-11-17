"""#4320: та же просьба посреди идущего выступления (есть несказанные сегменты) — перезапуск с начала.

Живой Sound Pi 04.11: «посчитай до сорока», и тот же запрос, пока звучит третий сегмент. Счёт строит код (#4317), так
что второе выступление совпадает с идущим целиком. Правка (MERGE) тут ничего не меняет: сказанное правка не повторяет
(``repeats.drop_said``), а несказанный хвост тот же — старый счёт дочитал бы 31–40, и просьба «посчитай до сорока»
осталась бы без ответа. Код решает однозначно: повтор того же целиком — перезапуск (звучащий сегмент дочитывается до
конца предложения, остальное снимается, блок закрыт, новый счёт — с одного своим блоком). Всё уже отдано на звучание —
по-прежнему новое выступление за старым (#4318).

Здесь же ``speech_dropped`` строки ``[command]``: речь рядом с тихим действием (``play_animation``) звучит, и счётчик
не должен говорить, что она отброшена.
"""

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
from rob_box_dialog.command import Command, Performance, ToolCall
from rob_box_llm.provider import ToolResult
from rob_box_dialog_host import command_bridge as cb
from rob_box_voice.core.perform_host import PerformHost

M = install_dialog_msgs_stub()
TIME = type(M.SpeechEvent().stamp)
REQUEST = "/voice/speech/request"
CANCEL = "/voice/speech/cancel"

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
    """Хост выступления и владелец речи-двойник (как в #4318): пред-синтез — ``synthesized``, звучание — ``started``,
    ``finished`` — по :meth:`finish`. Отменённые реплики не звучат."""

    def __init__(self, loop: Any) -> None:
        self.node, self.events = _Node(), []
        self.host = PerformHost(self.node, _tool, lambda: loop, self.events.append, kn.SPEECH_OUTPUT_LATENCY_MS,
                                clock=lambda: self.node.now_s)
        self.heard: List[str] = []
        self.sounding: Optional[Any] = None

    def cancelled(self) -> set:
        return {m.speech_id for m in self.node.pubs.get(CANCEL, _Pub()).pending}

    def pump(self, stop_at: str = "") -> None:
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
                if msg.speech_id in self.cancelled():
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


def test_same_count_mid_count_restarts_from_the_start(loop):
    """Живой сценарий #4320: звучит третий сегмент, четвёртый не отдан — тот же счёт: перезапуск, счёт целиком."""
    stand = _Stand(loop)
    stand.host.start(_count(), "t1")
    stand.pump(stop_at=COUNT[2])
    first = stand.host.block
    assert first.unsaid

    outcome = stand.host.start(_count(), "t2")

    assert outcome["status"] == "pending", outcome
    assert first.ended and first.end_reason == "stopped"
    assert stand.host.block is not first and stand.host.block.block_id == "perform-t2"
    assert stand.kinds("rejected") == [] and stand.kinds("segment_replaced") == []
    stand.finish()  # звучащий третий дочитан до конца предложения
    stand.pump()
    assert stand.heard == [*COUNT[:3], *COUNT]  # 31–40 старого счёта не звучат, новый — с одного до сорока
    assert phrases.text("perform.edit_rejected") not in stand.heard
    assert [e.reason for e in stand.kinds("block_end")] == ["stopped", "done"]


def test_same_count_with_edit_flag_mid_count_restarts_too(loop):
    """Модель прислала тот же счёт целиком с ``edit_pending`` — исход тот же: перезапуск, а не пустая правка."""
    stand = _Stand(loop)
    stand.host.start(_count(), "t1")
    stand.pump(stop_at=COUNT[1])
    assert stand.host.edit(_count(), "t2", 0)["status"] == "pending"
    assert stand.host.block.block_id == "perform-t2"


def test_count_written_in_words_repeats_count_in_digits(loop):
    """Тот же счёт другими словами (цифры ↔ числительные) — тот же счёт: перезапуск."""
    stand = _Stand(loop)
    stand.host.start(_count(), "t1")
    stand.pump(stop_at=COUNT[1])
    words = ("Один. Два. Три. Четыре. Пять. Шесть. Семь. Восемь. Девять. Десять.",
             *COUNT[1:])
    assert stand.host.start(Performance("poem", words, backing="none"), "t2")["status"] == "pending"


def test_different_content_mid_count_still_edits(loop):
    """Другое содержимое посреди счёта — по-прежнему правка несказанного (MERGE, #3700/#4207)."""
    stand = _Stand(loop)
    stand.host.start(_count(), "t1")
    stand.pump(stop_at=COUNT[1])
    first = stand.host.block
    tail = ("21 — шаг. 22 — шаг.", "Все дошли.")
    assert stand.host.start(Performance("poem", tail, backing="none"), "t2")["status"] == "done"
    assert stand.host.block is first and stand.kinds("segment_replaced")


def test_speech_beside_quiet_action_is_not_logged_as_dropped():
    """Живой лог 04.11: ``actions=['play_animation'] speech_dropped=1``, а речь прозвучала — ``play_animation`` тихий
    (``QUIET_WITH_SPEECH_TOOLS``), речь при нём звучит. Отброшена речь только при громком действии."""
    quiet = Command(speech="Считаю.", actions=(ToolCall("play_animation", {"animation": "thinking"}),))
    line = cb.log_line("deepseek", "tool_choice", quiet, "command")
    assert "speech_dropped=0" in line and "speech=" not in line
    loud = Command(speech="Считаю.", actions=(ToolCall("get_current_time", {}),))
    assert "speech_dropped=1" in cb.log_line("deepseek", "tool_choice", loud, "command")
