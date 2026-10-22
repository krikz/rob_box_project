"""Перебивание по эпохе и классу действия (ADR-0150 §6.2, §5.1; PR-9 #3703).

:func:`on_interrupt` — что гасить, когда человек с вейком сказал ясный стоп во время хода или речи робота. Решает код
по закрытому списку Tier-1 (``grammar.match`` + ``words.policy_word``) и по ``knowledge.ACTION_CLASSES``:

* «стой» (``halt``) — речь сразу (``SpeechCancel boundary=now``), выступление — ``halt``, ``stop_motion``, музыка по
  снимку (``dj_set(stop)``/``stop_music``);
* «хватит/стоп» (``stop``) — речь на границе предложения (≤ 3 с, tts_node), выступление дочитывает предложение и
  снимает бит на такте, музыка по снимку; режим тишины НЕ включается (В20, Шифу 22.10). ``stop_motion`` «хватит»
  шлёт по-прежнему: В20 оставил это открытым вопросом («не затронуто»), здесь поведение не меняется;
* «замолчи/помолчи» (``silence``) — то же, что «хватит», без музыки и движения; тишину кладёт ход (§6.3);
* не стоп (дополнение, новая просьба) и спорный стоп (В19) — :data:`NO_INTERRUPT`: речь не рвётся, новое решает
  ``plan.composition.compose()`` в ходе.

Любой стоп — REPLACE: ход LLM старой эпохи отменяется до HTTP-клиента (отмена задачи хода, :class:`TurnsInFlight`),
эпоха сессии +1, результаты старой эпохи исполнитель отбрасывает (``execute``: ``stale_epoch``). Классы
:func:`survives` (память, громкость, голос, знакомство, движение) не отменяются: начатое доживает до события
владельца, его исход в ход уже не попадает.

Модуль чистый: без ROS и без asyncio (будущие хода — любые объекты с ``done()``/``cancel()``).
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, FrozenSet, List, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import ToolCall
from rob_box_dialog.grammar import Tier1Command, match
from rob_box_dialog.session import SessionSnapshot
from rob_box_dialog.words import policy_word

#: Политики перебивания: ``TIER1_POLICY`` «стой» / «хватит», иначе не перебивание.
HALT, STOP, NONE = "halt", "stop", "none"
#: Виды Tier-1, которые перебивают (ясный стоп; спорный — нет).
_STOP_KINDS = ("stop_all", "silence")
_ALL = frozenset(kn.ACTION_CLASSES)


def survives(cls: str) -> bool:
    """Действие класса ``cls`` доживает до конца при отмене хода: непрерываемое (§5.1) и не звук (В20)."""
    return not kn.ACTION_CLASSES[cls].interruptible and cls not in kn.INTERRUPT_STOPS_CLASSES


def stop_music_calls(snapshot: SessionSnapshot) -> Tuple[ToolCall, ...]:
    """Стоп музыки по снимку плеера: идёт сет — ``dj_set(stop)`` и ``stop_music``, играет трек — ``stop_music``."""
    if snapshot.music.dj:
        return ToolCall("dj_set", {"action": "stop"}), ToolCall("stop_music", {})
    return (ToolCall("stop_music", {}),) if snapshot.music.state == "playing" else ()


@dataclass(frozen=True)
class Interrupt:
    """Что гасит перебивание. ``speech_boundary`` — ключ ``speech_client.CANCEL_BOUNDARIES``; ``perform`` — метод
    хоста выступления (``halt``/``stop``); ``music`` — тулы стопа музыки по снимку; ``cancel_actions``/``keep`` —
    классы действий, что отменяются и что доживают."""

    policy: str = NONE
    kind: str = ""
    speech_boundary: str = "as_requested"
    perform: Optional[str] = None
    stop_motion: bool = False
    music: Tuple[ToolCall, ...] = ()
    cancel_actions: FrozenSet[str] = frozenset()
    keep: FrozenSet[str] = field(default_factory=lambda: _ALL)

    @property
    def interrupts(self) -> bool:
        """REPLACE: речь старой эпохи отменяется, ход LLM в полёте — тоже, эпоха +1."""
        return self.policy != NONE


NO_INTERRUPT = Interrupt()


def interrupt_for(command: Optional[Tier1Command], text: str, snapshot: SessionSnapshot,
                  wake_words: Optional[Sequence[str]] = None) -> Interrupt:
    """Перебивание по уже разобранной Tier-1 команде (ход) — то же решение, что :func:`on_interrupt`."""
    if command is None or command.disputed or command.kind not in _STOP_KINDS:
        return NO_INTERRUPT
    halt = policy_word(text, wake_words) == HALT
    cancel = frozenset(c for c in _ALL if not survives(c)) | (frozenset({"action.motion"}) if halt else frozenset())
    sounds = command.kind == "stop_all"
    return Interrupt(policy=HALT if halt else STOP, kind=command.kind,
                     speech_boundary="now" if halt else "as_requested", perform=HALT if halt else STOP,
                     stop_motion=sounds,
                     music=stop_music_calls(snapshot) if sounds else (), cancel_actions=cancel, keep=_ALL - cancel)


def on_interrupt(text: str, snapshot: SessionSnapshot, wake_words: Optional[Sequence[str]] = None) -> Interrupt:
    """Фраза (с вейком или без) → :class:`Interrupt`. Не ясный стоп — :data:`NO_INTERRUPT`."""
    return interrupt_for(match(text or "", snapshot), text or "", snapshot, wake_words)


class TurnsInFlight:
    """Хода агента в полёте (будущие ``run_coroutine_threadsafe``): стоп отменяет все — и идущий, и ждущий очереди.

    Отмена будущего отменяет задачу хода в её цикле: ``CancelledError`` приходит в ``await`` LLM-провайдера, и
    HTTP-запрос закрывает его клиент (#1280) — ответ старой эпохи не приходит вовсе. ``key`` — фраза хода
    (``utterance_id``): запрос перебивания той же фразы, пришедший позже её реплики (порядок топиков ROS не
    гарантирован), свой ход стопа не отменяет."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._items: List[Tuple[Any, str]] = []

    def add(self, future: Any, key: str = "") -> Any:
        with self._lock:
            self._items = [(f, k) for f, k in self._items if not f.done()] + [(future, key)]
        return future

    def cancel(self, keep_key: str = "") -> int:
        """Отменить незавершённые хода (кроме хода фразы ``keep_key``); вернуть, сколько отменено."""
        with self._lock:
            keep = [(f, k) for f, k in self._items if keep_key and k == keep_key and not f.done()]
            gone = [f for f, k in self._items if not (keep_key and k == keep_key)]
            self._items = keep
        return sum(1 for f in gone if not f.done() and f.cancel())

    def __len__(self) -> int:
        with self._lock:
            return sum(1 for f, _ in self._items if not f.done())


__all__ = ["HALT", "Interrupt", "NO_INTERRUPT", "NONE", "STOP", "TurnsInFlight", "interrupt_for", "on_interrupt",
           "stop_music_calls", "survives"]
