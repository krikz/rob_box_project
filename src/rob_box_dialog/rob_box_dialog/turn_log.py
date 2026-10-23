"""Журнал ходов сессии и окно истории для LLM из того, что прозвучало (ADR-0150 §2.1 п.8, §7.3, §7.4; PR-8 #3702).

В историю модели попадает только то, что человек услышал, и структурные результаты действий:

* ``user`` — текст хода как его взял код (без вейка и без маркеров ``[Spkr:]``/``[TG]``/``[URGENT_BACKLOG]``: кто
  говорил и откуда — в снимке сессии, не в тексте);
* ``assistant`` — ``text[:spoken_chars]`` по событию владельца речи (``SpeechEvent`` ``finished/cancelled``): не
  прозвучало — нет и в истории; у текстового канала (телеграм) — весь текст, он доставлен;
* ``tool`` — :class:`~rob_box_dialog.respond.ActionResult` структурой (тул, статус, причина, данные).

Отвергнутые команды и невалидный ответ модели в окно не попадают (#3145) — только в агрегат хода. Окно — не больше
``knowledge.TURN_WINDOW_MAX_TURNS`` ходов и ``TURN_WINDOW_MAX_TOKENS`` токенов, старые ходы выпадают целиком;
:meth:`TurnLog.reset` (новая сессия) очищает всё, и поздние события речи старой эпохи ничего не добавляют.

Журнал живёт в памяти процесса (директива Шифу 02.09, решение В8): на диск в проде уходят только агрегаты без
текста — :class:`TurnAggregate` (строка лога :data:`AGGREGATE_MARK`); текст хода пишет только e2e-БД (PR-8 ч.8c).
"""

from __future__ import annotations

import json
from collections import OrderedDict
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple

from rob_box_core.token_estimate import estimate_tokens
from rob_box_dialog import knowledge as kn

#: Строка лога агрегата хода (читает ``scripts/dialog`` для метрик §14 без текста турнов).
AGGREGATE_MARK = "📊 [turn]"
USER, ASSISTANT, TOOL = "user", "assistant", "tool"


@dataclass(frozen=True)
class WindowEntry:
    """Запись окна LLM: ``content`` — строка у ``user``/``assistant``, данные результата у ``tool``."""

    role: str
    content: Any

    def text(self) -> str:
        """Как запись уходит модели (и как считается её размер): данные ``tool`` — компактным JSON."""
        if isinstance(self.content, str):
            return self.content
        return json.dumps(self.content, ensure_ascii=False, separators=(",", ":"), default=_plain)


def _plain(value: Any) -> Any:
    """JSON для неизменяемых данных результата: ``MappingProxyType`` — словарём, остальное — строкой."""
    return dict(value) if isinstance(value, Mapping) else str(value)


@dataclass(frozen=True)
class TurnAggregate:
    """Агрегат хода без текста — единственное, что о ходе пишется на диск в проде (§7.4)."""

    turn_id: str
    epoch: int
    agent: str
    stage: str
    status: str
    latency_ms: int = 0
    llm_calls: int = 0
    llm_provider: str = ""
    tokens: int = 0

    def log_line(self) -> str:
        return (f"{AGGREGATE_MARK} turn_id={self.turn_id} epoch={self.epoch} agent={self.agent} stage={self.stage} "
                f"status={self.status} latency_ms={self.latency_ms} llm_calls={self.llm_calls} "
                f"llm_provider={self.llm_provider or '-'} tokens={self.tokens}")


@dataclass
class _Turn:
    turn_id: str
    epoch: int
    user: str
    entries: List[WindowEntry] = field(default_factory=list)


def _tool_content(result: Any) -> Mapping[str, Any]:
    data = dict(getattr(result, "data", None) or {})
    out: Dict[str, Any] = {"tool": result.tool, "status": result.status}
    if getattr(result, "reason", None):
        out["reason"] = result.reason
        note = kn.REJECT_NOTES.get(result.reason) if result.status == "rejected" else None
        if note:  # отказ памяти - исход словами, чтобы модель не несла факт в следующий ход (#3881)
            out["note"] = note
    if data:
        out["data"] = data
    return MappingProxyType(out)


class TurnLog:
    """Ходы одной сессии агента: что сказал человек, что прозвучало, чем кончились действия."""

    def __init__(self, max_turns: int = kn.TURN_WINDOW_MAX_TURNS, max_tokens: int = kn.TURN_WINDOW_MAX_TOKENS,
                 estimate: Callable[[str], int] = estimate_tokens) -> None:
        self._max_turns = max_turns
        self._max_tokens = max_tokens
        self._estimate = estimate
        self._turns: "OrderedDict[str, _Turn]" = OrderedDict()
        self._speech: Dict[str, Tuple[str, str]] = {}  # speech_id → (turn_id, текст)

    def begin(self, turn_id: str, epoch: int, text: str) -> None:
        """Ход начался: ``text`` — реплика человека, как её взял код (пустая — ход без слов, в окно не идёт)."""
        self._turns[turn_id] = _Turn(turn_id, int(epoch), (text or "").strip())
        self._trim()

    def said(self, turn_id: str, speech_id: str, text: str, delivered: bool = False) -> None:
        """Агент отдал реплику владельцу речи. В окно она попадёт по :meth:`spoken`; ``delivered`` — текстовый
        канал (телеграм): текст доставлен целиком сразу."""
        if turn_id not in self._turns or not text:
            return
        if delivered:
            self._add(turn_id, WindowEntry(ASSISTANT, text))
        else:
            self._speech[speech_id] = (turn_id, text)

    def spoken(self, speech_id: str, spoken_chars: int) -> Optional[str]:
        """Реплика отзвучала (``finished``/``cancelled``/``rejected``): в окно — озвученная часть. Чужая или
        старой сессии реплика — ``None``, ничего не меняется."""
        turn_id, text = self._speech.pop(speech_id, (None, ""))
        part = text[:max(0, int(spoken_chars or 0))].strip()
        if turn_id is None or not part:
            return None
        self._add(turn_id, WindowEntry(ASSISTANT, part))
        return part

    def speech(self, speech_id: str) -> Optional[Tuple[str, str]]:
        """``(turn_id, текст)`` реплики, которая ещё не отзвучала; чужая или уже отзвучавшая — ``None``."""
        return self._speech.get(speech_id)

    def result(self, turn_id: str, result: Any) -> None:
        """Результат действия хода (``ActionResult``) — структурой."""
        self._add(turn_id, WindowEntry(TOOL, _tool_content(result)))

    def window(self) -> Tuple[WindowEntry, ...]:
        """Окно для LLM: ходы по порядку, новые важнее; лишние по числу и токенам выпадают целиком с начала."""
        turns = [self._entries(t) for t in self._turns.values()]
        turns = [t for t in turns if t][-self._max_turns:]
        while len(turns) > 1 and self._tokens(turns) > self._max_tokens:
            turns.pop(0)
        return tuple(e for t in turns for e in t)

    def tokens(self) -> int:
        """Оценка размера окна в токенах (то, что уйдёт модели)."""
        return sum(self._estimate(e.text()) for e in self.window())

    def reset(self) -> None:
        """Новая сессия: окно и ожидающие реплики очищены."""
        self._turns.clear()
        self._speech.clear()

    def _entries(self, turn: _Turn) -> List[WindowEntry]:
        lead = [WindowEntry(USER, turn.user)] if turn.user else []
        return lead + list(turn.entries)

    def _tokens(self, turns: List[List[WindowEntry]]) -> int:
        return sum(self._estimate(e.text()) for t in turns for e in t)

    def _add(self, turn_id: str, entry: WindowEntry) -> None:
        turn = self._turns.get(turn_id)
        if turn is not None:
            turn.entries.append(entry)

    def _trim(self) -> None:
        """Память журнала ограничена окном: старше ``max_turns`` ходов не хранится (и их реплики — тоже)."""
        while len(self._turns) > self._max_turns:
            gone, _ = self._turns.popitem(last=False)
            self._speech = {sid: v for sid, v in self._speech.items() if v[0] != gone}


__all__ = ["AGGREGATE_MARK", "ASSISTANT", "TOOL", "TurnAggregate", "TurnLog", "USER", "WindowEntry"]
