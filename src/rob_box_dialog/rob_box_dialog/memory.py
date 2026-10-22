"""Память о людях: факт привязан к собеседнику из снимка, а не к словам LLM (ADR-0150 §7.4-7.5, PR-11 #3705).

Два правила кода (ADR-0148), оба чистые функции без ROS:

* :func:`bind_person` - тулы ``knowledge.PERSON_SCOPED_TOOLS`` получают ``SpeakerRef.person_ref`` снимка. Что бы ни
  написала модель в аргументе собеседника, подставляется он; в снимке его нет - привязать не к кому, исполнитель
  отвечает ``rejected{no_person}`` и в память не идёт (раньше такой факт становился общим для всех, #1770);
* :func:`has_evidence` - ``done`` тула из ``knowledge.DONE_EVIDENCE`` засчитывается только с полем доказательства в
  данных события владельца (``saved{fact_id}``): фраза «Запомнил» не бывает без записи.

Имя собеседника не нужно: у ``person_ref`` может быть лишь кличка (#3858). Ключ памяти - ``person_ref`` как есть; после
ADR-0130 этапа 3 это ``Acquaintance.id``, смена - в адаптере собеседника, а не здесь.
"""

from __future__ import annotations

from types import MappingProxyType
from typing import Any, Mapping, Optional

from rob_box_dialog import knowledge as kn


def bind_person(call: Any, snapshot: Any) -> Optional[Mapping[str, Any]]:
    """Аргументы вызова с собеседником снимка; ``None`` - тул личный, а собеседника (``person_ref``) в снимке нет."""
    if call.tool not in kn.PERSON_SCOPED_TOOLS or snapshot.agent not in kn.PERSON_AGENTS:
        return call.args
    if not hasattr(snapshot, "speaker"):  # заглушка снимка режима команд v1 (``command_turn``): собеседник там - у MCP
        return call.args
    speaker = snapshot.speaker
    person = getattr(speaker, "person_ref", None)
    if not person:
        return None
    return MappingProxyType({**call.args, kn.PERSON_ARG: person})


def has_evidence(tool: str, data: Mapping[str, Any]) -> bool:
    """Есть ли в данных события владельца то, без чего ``done`` этого тула не верится (``knowledge.DONE_EVIDENCE``)."""
    key = kn.DONE_EVIDENCE.get(tool)
    return key is None or data.get(key) is not None


__all__ = ["bind_person", "has_evidence"]
