"""Память о людях: факт привязан к собеседнику из снимка, а не к словам LLM (ADR-0150 §7.4-7.5, PR-11 #3705).

Два правила кода (ADR-0148), оба чистые функции без ROS:

* :func:`bind_person` - тулы ``knowledge.PERSON_SCOPED_TOOLS`` получают ``SpeakerRef.person_ref`` снимка. Что бы ни
  написала модель в аргументе собеседника, подставляется он; в снимке его нет - привязать не к кому, исполнитель
  отвечает ``rejected{no_person}`` и в память не идёт (раньше такой факт становился общим для всех, #1770). Голос
  узнан, а чей он - не подтверждено (спор, гипотеза: ``knowledge.VOICE_UNCONFIRMED_STATES``) - тоже не к кому, но
  причина своя, :func:`unbound_reason` (#3993): «не узнаю голос» тут неправда;
* :func:`has_evidence` - ``done`` тула из ``knowledge.DONE_EVIDENCE`` засчитывается только с полем доказательства в
  данных события владельца (``saved{fact_id}``): фраза «Запомнил» не бывает без записи.

* :func:`profiled_person` / :func:`speaker_memory_block` - «робот помнит, с кем говорит» (#3919): профиль ведётся только
  о собеседнике с подтверждённым именем (``name_state == confirmed``: биометрия уверена или голос подтвердило лицо,
  ADR-0130 §2.6), а раздел снимка ``speaker_memory`` - ``dialog_count`` и его факты, без правил для модели.

Имя собеседника не нужно: у ``person_ref`` может быть лишь кличка (#3858). Ключ памяти - ``person_ref`` как есть; после
ADR-0130 этапа 3 это ``Acquaintance.id``, смена - в адаптере собеседника, а не здесь.
"""

from __future__ import annotations

from types import MappingProxyType
from typing import Any, Dict, Iterable, Mapping, Optional

from rob_box_dialog import knowledge as kn


def bind_person(call: Any, snapshot: Any) -> Optional[Mapping[str, Any]]:
    """Аргументы вызова с собеседником снимка; ``None`` - тул личный, а собеседника, чьи это факты, в снимке нет:
    ``person_ref`` пуст или голос не подтверждён (спор, гипотеза - для памяти гипотеза как спор, #3993: вопроса «ты
    Саша?» нет, а факт кандидата чужому хуже пустого). Почему - :func:`unbound_reason`."""
    if call.tool not in kn.PERSON_SCOPED_TOOLS or snapshot.agent not in kn.PERSON_AGENTS:
        return call.args
    if not hasattr(snapshot, "speaker"):  # заглушка снимка режима команд v1 (``command_turn``): собеседник там - у MCP
        return call.args
    speaker = snapshot.speaker
    person = getattr(speaker, "person_ref", None)
    if not person or getattr(speaker, "name_state", None) in kn.VOICE_UNCONFIRMED_STATES:
        return None
    return MappingProxyType({**call.args, kn.PERSON_ARG: person})


def unbound_reason(snapshot: Any) -> str:
    """Причина отказа тула памяти без собеседника (:func:`bind_person` вернул ``None``): голос узнан, а чей - не
    подтверждено (``knowledge.VOICE_UNCONFIRMED_STATES``) - ``voice_unconfirmed`` («Голос знакомый, но не уверен
    чей»); иначе собеседника нет - ``no_person``."""
    state = getattr(getattr(snapshot, "speaker", None), "name_state", None)
    return kn.VOICE_UNCONFIRMED_REASON if state in kn.VOICE_UNCONFIRMED_STATES else kn.NO_PERSON_REASON


def speaker_changed(call: Any, turn_snapshot: Any, now_snapshot: Any) -> bool:
    """Собеседник сменился между началом хода и исполнением: факт мог быть сказан другим человеком, привязывать его к
    ``person_ref`` начала хода нельзя (#3881). Неузнанное начало хода - уже ``no_person``, здесь не считается."""
    if call.tool != "memory_save" or turn_snapshot.agent not in kn.PERSON_AGENTS:
        return False
    before = getattr(getattr(turn_snapshot, "speaker", None), "person_ref", None)
    now = getattr(getattr(now_snapshot, "speaker", None), "person_ref", None)
    return bool(before) and before != now


def has_evidence(tool: str, data: Mapping[str, Any]) -> bool:
    """Есть ли в данных события владельца то, без чего ``done`` этого тула не верится (``knowledge.DONE_EVIDENCE``)."""
    key = kn.DONE_EVIDENCE.get(tool)
    return key is None or data.get(key) is not None


def profiled_person(speaker: Any) -> Optional[str]:
    """``person_ref``, о котором код ведёт профиль и кладёт факты в снимок; ``None`` - собеседник не уверен/неизвестен
    (догадку по голосу в профиль не пишем: счётчик и факты чужого человека - хуже пустого раздела)."""
    person = getattr(speaker, "person_ref", None)
    return person if person and getattr(speaker, "name_state", None) == "confirmed" else None


def speaker_memory_block(profile: Optional[Mapping[str, Any]], facts: Iterable[Any]) -> Optional[Dict[str, Any]]:
    """Раздел снимка ``speaker_memory``: ``{dialog_count, facts}`` (ключ без данных пропущен), нечего - ``None``.
    Факты - тексты, свежие первыми: не больше ``SPEAKER_FACTS_MAX`` по ``SPEAKER_FACT_MAX_CHARS`` знаков."""
    texts = [t[:kn.SPEAKER_FACT_MAX_CHARS] for t in (str(f).strip() for f in facts) if t][:kn.SPEAKER_FACTS_MAX]
    block: Dict[str, Any] = {}
    if (profile or {}).get("dialog_count") is not None:
        block["dialog_count"] = int(profile["dialog_count"])  # type: ignore[index]
    if texts:
        block["facts"] = texts
    return block or None


__all__ = ["bind_person", "has_evidence", "profiled_person", "speaker_changed", "speaker_memory_block",
           "unbound_reason"]
