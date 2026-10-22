"""Выступление в ноде v1 (ADR-0150 §5.6.3, PR-7c ч.3 #3700): автомат ``PerformBlock`` ↔ владельцы речи и музыки.

Решает ``rob_box_dialog.plan.perform``, хост только исполняет его эффекты и кормит его событиями:

* ``Say`` → ``SpeechRequest`` (``speech_client.say``: пред-синтез ``commit=false`` или звучание в ``at``;
  ``boundary=sentence`` — «хватит» дочитывает предложение, X5);
* ``Cancel`` → ``SpeechCancel`` реплики по ``speech_id``;
* ``Backing`` → ``request_music(intent=backing)``, ``MusicStop`` → ``stop_music(boundary)`` — исполнителем тулов
  напрямую (:meth:`PlanToolExecutor.execute_owned`): это код выступления, а не ход LLM — без допуска плана и без
  речи «музыки хода»; исход запуска подложки (``started``/``rejected``/``busy``) — из ответа тула, остальное
  (``nearly_finished``, ``finished``, ``idle``) — из ``/voice/music/event``;
* ``PlanEvent`` → планировщик хода (``/dialog/plan_event`` и строка лога).

Один блок на ноду. Колбэки ROS, таймер и loop ноды — разные потоки: блок трогается только под замком.
"""

from __future__ import annotations

import asyncio
import itertools
import logging
import threading
import time
from typing import Any, Callable, Dict, List, Mapping, Optional

from rob_box_core import speech_client
from rob_box_core.utterance import Utterance
from rob_box_dialog.command import Performance
from rob_box_dialog.plan.backing import Backing, MusicStop
from rob_box_dialog.plan.perform import Cancel, PerformBlock, Say, context_block
from rob_box_dialog.plan.plan_events import PlanEvent
from rob_box_llm.provider import ToolCall, ToolResult

_LOG = logging.getLogger(__name__)
AGENT = "personality"
#: Шаг таймера выступления: граница сегмента (за 2 с до конца) и срок подложки.
TICK_S = 0.1
REQUEST_MUSIC, STOP_MUSIC = "request_music", "stop_music"
#: События плеера, которые блок узнаёт из топика; исход запуска — из ответа тула (одна доставка на событие).
_TOPIC_EVENTS = ("nearly_finished", "finished", "idle")

RunTool = Callable[[ToolCall], Any]  # корутина → ToolResult


class PerformHost:
    """Одно выступление ноды v1: старт по команде LLM, правка, «хватит»/«стой», события владельцев, таймер."""

    def __init__(self, node: Any, run_tool: RunTool, loop: Callable[[], Optional[asyncio.AbstractEventLoop]],
                 emit: Callable[[PlanEvent], None], output_latency_ms: int,
                 clock: Callable[[], float] = time.time) -> None:
        self._node = node
        self._run_tool = run_tool
        self._loop = loop
        self._emit = emit
        self._latency_ms = int(output_latency_ms)
        self._clock = clock
        self._lock = threading.RLock()
        self._ids = itertools.count(1)
        self._epoch = 0
        self._turn_id = ""
        self.block: Optional[PerformBlock] = None

    # -- что видят валидатор, LLM и планировщик ---------------------------------------------------------------------

    @property
    def active_segment(self) -> Optional[int]:
        """``Busy.active_segment`` валидатора команды (``None`` — выступления нет)."""
        with self._lock:
            return None if self.block is None else self.block.active_segment

    @property
    def owns_backing(self) -> bool:
        """Музыка — бит идущего выступления (снимет его outro само), а не трек человека."""
        with self._lock:
            block = self.block
            return block is not None and not block.ended and block.backing.sounding

    def owns(self, speech_id: str) -> bool:
        """Реплика выступления — для колонки ``perform`` планировщика ресурсов."""
        with self._lock:
            return self.block is not None and self.block.lines.find(speech_id) is not None

    def context(self) -> Optional[Dict[str, Any]]:
        """Блок контекста LLM «что звучит / осталось N с» (риск §16); выступления нет — ``None``."""
        with self._lock:
            if self.block is None or self.block.ended:
                return None
            return context_block(self.block, self._clock())

    # -- команды -------------------------------------------------------------------------------------------------------

    def start(self, perf: Performance, turn_id: str, epoch: int = 0) -> Dict[str, Any]:
        """Новое выступление; идёт другое — правка его не начатых сегментов (MERGE, COMPOSITION perform×perform)."""
        with self._lock:
            if self.block is not None and not self.block.ended:
                return self._edit_locked(perf, merge=True)
            self._turn_id, self._epoch = turn_id, epoch
            self.block = PerformBlock(perf, f"perform-{turn_id or next(self._ids)}", turn_id=turn_id, epoch=epoch,
                                      output_latency_ms=self._latency_ms, log=self._log)
            self._apply(self.block.start(self._clock()))
            return {"status": "pending", "block_id": self.block.block_id}

    def edit(self, perf: Performance) -> Dict[str, Any]:
        """``edit_pending``: правка не начатых сегментов идущего выступления."""
        with self._lock:
            return self._edit_locked(perf, merge=False)

    def stop(self) -> bool:
        """«Хватит»: дочитать предложение, outro бита. Выступления нет — ``False`` (стоп решает общий путь)."""
        return self._command(lambda block, now: block.stop(now))

    def halt(self) -> bool:
        """«Стой»: речь и бит — сразу."""
        return self._command(lambda block, now: block.halt(now))

    def add_backing(self, backing: str = "beat") -> bool:
        """«Добавь бит» во время чтения (#993): ``True`` — бит заказан выступлению (его ещё не было)."""
        with self._lock:
            block = self.block
            if block is None or block.ended:
                return False
            effects = block.add_backing(backing, self._clock())
            self._apply(effects)
            return bool(effects)

    # -- события владельцев ---------------------------------------------------------------------------------------------

    def on_speech(self, kind: str, speech_id: str, duration_ms: int = 0, spoken_chars: int = 0) -> None:
        self._command(lambda block, now: block.on_speech(kind, speech_id, now, duration_ms=duration_ms,
                                                         spoken_chars=spoken_chars))

    def on_music_event(self, kind: str, track_id: Optional[str], fields: Mapping[str, Any]) -> None:
        if kind in _TOPIC_EVENTS:
            self._command(lambda block, now: block.on_music(kind, track_id, now, fields))

    def tick(self) -> None:
        self._command(lambda block, now: block.tick(now))

    # -- исполнение эффектов --------------------------------------------------------------------------------------------

    def _command(self, step: Callable[[PerformBlock, float], List[Any]]) -> bool:
        with self._lock:
            block = self.block
            if block is None or block.ended:
                return False
            self._apply(step(block, self._clock()))
            return True

    def _edit_locked(self, perf: Performance, merge: bool) -> Dict[str, Any]:
        block = self.block
        if block is None or block.ended:
            return {"status": "rejected", "reason": "edit_without_active_perform"}
        if merge:  # новое выступление поверх идущего: не начатые — с ACTIVE+1
            perf = Performance(perf.kind, perf.segments, perf.backing, perf.expression_per_segment,
                               (block.active_segment or 0) + 1, perf.voice_per_segment)
        effects = block.edit(perf, self._clock())
        self._apply(effects)
        rejected = [e for e in effects if isinstance(e, PlanEvent) and e.kind == "rejected"]
        return {"status": "rejected", "reason": rejected[0].reason} if rejected else {"status": "done"}

    def _apply(self, effects: List[Any]) -> None:
        for effect in effects:
            if isinstance(effect, Say):
                self._say(effect)
            elif isinstance(effect, Cancel):  # сегменты просят границу предложения; «стой» (boundary none) — сразу
                speech_client.cancel(self._node, agent=AGENT, speech_id=effect.speech_id, reason="perform",
                                     boundary="as_requested" if effect.boundary == "sentence" else "now")
            elif isinstance(effect, Backing):
                self._spawn(self._backing(effect))
            elif isinstance(effect, MusicStop):
                self._spawn(self._tool(STOP_MUSIC, {"boundary": "bar" if effect.boundary == "bar" else "now"}))
            elif isinstance(effect, PlanEvent):
                self._emit(effect)

    def _say(self, say: Say) -> None:
        utterance = Utterance(text=say.text, voice=say.voice or None, emotion=say.emotion or None,
                              speech_id=say.speech_id)
        speech_client.say(self._node, utterance, agent=AGENT, epoch=self._epoch, turn_id=self._turn_id,
                          at=say.at, commit=say.commit, boundary="sentence")

    async def _backing(self, backing: Backing) -> None:
        """Запуск/продление подложки; исход запуска — в блок (``started`` с сеткой или ``rejected`` с причиной)."""
        args = {"intent": "backing", "text": "", "mood": backing.mood, "min_seconds": backing.min_s}
        result = await self._tool(REQUEST_MUSIC, args)
        if backing.extend:
            return
        data = dict((result.data if result is not None else None) or {})
        if data.get("ok") and data.get("started"):
            self._command(lambda block, now: block.on_music("started", data.get("track_id"), now, data))
        else:
            reason = str(data.get("reason") or "error")
            self._command(lambda block, now: block.on_music("rejected", None, now, {"reason": reason}))

    async def _tool(self, name: str, args: Mapping[str, Any]) -> Optional[ToolResult]:
        call = ToolCall(id=f"perform-{next(self._ids)}", name=name, arguments=dict(args))
        try:
            return await self._run_tool(call)
        except Exception as exc:  # noqa: BLE001 — сбой транспорта: честный отказ подложки, а не падение ноды
            self._log(f"⚠️ [perform] {name} не исполнен: {type(exc).__name__}: {exc}")
            return None

    def _spawn(self, coro: Any) -> None:
        loop = self._loop()
        if loop is None or loop.is_closed():
            coro.close()
            self._log("⚠️ [perform] loop ноды не готов — тул выступления не исполнен")
            return
        asyncio.run_coroutine_threadsafe(coro, loop)

    def _log(self, text: str) -> None:
        self._node.get_logger().info(text)


__all__ = ["AGENT", "PerformHost", "TICK_S"]
