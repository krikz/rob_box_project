"""Фраза об исполнении — из события владельца, а не из слов LLM (ADR-0150 §2.1 п.3, §5.2, §5.4; ADR-0148).

Исполнитель отвечает :class:`ActionResult`; :func:`phrase_from_result` строит по нему реплику из каталога
``phrases/ru.yaml`` по ключу ``{класс}.{статус}[.{причина}]`` (или особому ключу тула, ``knowledge.TOOL_PHRASES``),
:func:`join` собирает фразы нескольких действий хода в одну реплику. ``query`` с ``done`` фразы не даёт: данные
уходят обратно LLM.
"""

from __future__ import annotations

import zlib
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping, Optional, Sequence

from rob_box_core.utterance import Sink, Utterance
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases

#: Статусы, при которых рефлекс побеждает эмоцию команды (§5.5.1: нельзя «радостно» о провале).
_FAILED = frozenset({"rejected", "timeout"})


@dataclass(frozen=True)
class ActionResult:
    """Ответ исполнителя (§5.2): статус по событию владельца ресурса, причина — ключ фраз, данные — слоты."""

    turn_id: str
    epoch: int
    agent: str
    tool: str
    cls: str
    status: str
    reason: Optional[str] = None
    data: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))
    event_ref: Optional[str] = None


def phrase_key(result: ActionResult) -> Optional[str]:
    """Ключ фразы результата или ``None``, если о таком исходе шаблон не говорит (``query.done`` — речь LLM)."""
    base = kn.TOOL_PHRASES.get(result.tool, {}).get(result.status) \
        or kn.ACTION_CLASSES[result.cls].phrases.get(result.status)
    if base is None:
        return None
    return phrases.first_key(f"{base}.{result.reason}" if result.reason else None, base)


def phrase_from_result(result: ActionResult, snapshot: Any = None) -> Optional[Utterance]:
    """Реплика по результату исполнителя. Вариант фразы выбирается по ``turn_id`` (повторяемо в тестах), адресат —
    наушник для агента оператора, иначе динамики (телеграм-агент берёт из реплики только текст)."""
    key = phrase_key(result)
    if key is None:
        return None
    text = phrases.text(key, zlib.crc32(result.turn_id.encode("utf-8")), **dict(result.data))
    agent = getattr(snapshot, "agent", None) or result.agent
    return Utterance(text=text, sink=Sink.HEADSET if agent == "operator" else Sink.SPEAKERS,
                     emotion="sad" if result.status in _FAILED else None)


def join(results: Sequence[ActionResult], snapshot: Any = None) -> Optional[Utterance]:
    """Фразы всех действий хода — одной репликой в порядке результатов; провал хоть одного — грустная эмоция."""
    said = [u for u in (phrase_from_result(r, snapshot) for r in results) if u is not None]
    if not said:
        return None
    sad = any(u.emotion == "sad" for u in said)
    return Utterance(text=" ".join(u.text for u in said), sink=said[0].sink, emotion="sad" if sad else None)
