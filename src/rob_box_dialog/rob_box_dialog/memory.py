"""Память о людях: факт привязан к собеседнику из снимка, а не к словам LLM (ADR-0150 §7.4-7.5, PR-11 #3705).

Два правила кода (ADR-0148), оба чистые функции без ROS:

* :func:`bind_person` - тулы ``knowledge.PERSON_SCOPED_TOOLS`` получают ``SpeakerRef.person_ref`` снимка. Что бы ни
  написала модель в аргументе собеседника, подставляется он; в снимке его нет - привязать не к кому, исполнитель
  отвечает ``rejected{no_person}`` и в память не идёт (раньше такой факт становился общим для всех, #1770). Голос
  узнан, а чей он - не подтверждено (спор, гипотеза: ``knowledge.VOICE_UNCONFIRMED_STATES``) - тоже не к кому, но
  причина своя, :func:`unbound_reason` (#3993): «не узнаю голос» тут неправда; о таком отказе чтения памяти
  (:func:`refused_read`) на вопрос о памяти о самом человеке говорит код (:func:`voiced_refusals`, ревью #4053),
  на остальные реплики он - данные модели (ревью PR #4055);
* :func:`has_evidence` - ``done`` тула из ``knowledge.DONE_EVIDENCE`` засчитывается только с полем доказательства в
  данных события владельца (``saved{fact_id}``): фраза «Запомнил» не бывает без записи.
* :func:`code_outcome` - исход, который код решает без владельца памяти: факт - только имя собеседника
  (:func:`name_fact`, #4054: имя живёт в профиле голоса) - ``done{name_in_profile}`` без записи и без фразы; собеседник
  сменился с начала хода - ``rejected{speaker_changed}`` (#3881).

* :func:`profiled_person` / :func:`speaker_memory_block` - «робот помнит, с кем говорит» (#3919): профиль ведётся только
  о собеседнике с подтверждённым именем (``name_state == confirmed``: биометрия уверена или голос подтвердило лицо,
  ADR-0130 §2.6), а раздел снимка ``speaker_memory`` - ``dialog_count`` и его факты, без правил для модели.

Имя собеседника не нужно: у ``person_ref`` может быть лишь кличка (#3858). Ключ памяти - ``person_ref`` как есть; после
ADR-0130 этапа 3 это ``Acquaintance.id``, смена - в адаптере собеседника, а не здесь.
"""

from __future__ import annotations

from types import MappingProxyType
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.words import asks_own_memory, name_tokens


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


def refused_read(result: Any) -> bool:
    """Чтение памяти (``memory_search``/``memory_context``) отклонено кодом до владельца - собеседника нет или голос
    не подтверждён (``knowledge.UNBOUND_REASONS``). Кто о нём говорит - :func:`voiced_refusals`."""
    return (result.tool in kn.PERSON_SCOPED_TOOLS and result.cls == "query" and result.status == "rejected"
            and result.reason in kn.UNBOUND_REASONS)


def voiced_refusals(results: Sequence[Any], text: str) -> Tuple[Any, ...]:
    """Отказы чтения памяти (:func:`refused_read`), о которых говорит КОД фразой ``query.rejected.<причина>``, а не
    модель: только когда реплика - вопрос о памяти о самом человеке (``words.asks_own_memory``: «что ты обо мне
    помнишь?» - ответ и есть отказ; ревью #4053: не «запоминать не буду» записи). Иначе ``()``: отказ - данные модели,
    как до #4053, и она отвечает на сам вопрос (ревью PR #4055: неузнанный голос спросил «кто такой Гагарин?», модель
    позвала ``memory_search("Гагарин")`` - ход закрывался фразой об отказе, вопрос оставался без ответа)."""
    refused = tuple(r for r in results if refused_read(r))
    return refused if refused and asks_own_memory(text) else ()


def speaker_changed(call: Any, turn_snapshot: Any, now_snapshot: Any) -> bool:
    """Собеседник сменился между началом хода и исполнением: факт мог быть сказан другим человеком, привязывать его к
    ``person_ref`` начала хода нельзя (#3881). Неузнанное начало хода - уже ``no_person``, здесь не считается."""
    if call.tool != "memory_save" or turn_snapshot.agent not in kn.PERSON_AGENTS:
        return False
    before = getattr(getattr(turn_snapshot, "speaker", None), "person_ref", None)
    now = getattr(getattr(now_snapshot, "speaker", None), "person_ref", None)
    return bool(before) and before != now


def name_fact(call: Any, snapshot: Any) -> bool:
    """Факт ``memory_save`` - только имя собеседника (таблица ``knowledge.NAME_FACT_WORDS``, #4054, решение Шифу 27.10):
    «Пользователя зовут Тамара». Имя живёт в профиле голоса (``register_speaker``), а не фактом памяти. Только у агента с
    собеседником-голосом (``PERSON_AGENTS``): у чата Telegram профиля голоса нет, его факт об имени - обычный факт."""
    if call.tool != "memory_save" or snapshot.agent not in kn.PERSON_AGENTS:
        return False
    tokens = name_tokens(str(call.args.get("fact") or ""))
    words = kn.NAME_FACT_WORDS
    rest = [t for t in tokens if t not in words["marker"] and t not in words["subject"]]
    return len(rest) == 1 and len(rest[0]) >= 2 and any(t in words["marker"] for t in tokens)


def code_outcome(call: Any, turn_snapshot: Any, now_snapshot: Any) -> Optional[Tuple[str, str]]:
    """Исход вызова памяти, который решает код без владельца, - ``(статус, причина)``, или ``None`` (вызов идёт
    владельцу): факт - только имя (:func:`name_fact`) - ``done{name_in_profile}`` без записи и без фразы; собеседник
    сменился с начала хода (:func:`speaker_changed`) - ``rejected{speaker_changed}``."""
    if name_fact(call, turn_snapshot):
        return "done", kn.NAME_IN_PROFILE_REASON
    if speaker_changed(call, turn_snapshot, now_snapshot):
        return "rejected", kn.SPEAKER_CHANGED_REASON
    return None


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


__all__ = ["bind_person", "code_outcome", "has_evidence", "name_fact", "profiled_person", "refused_read",
           "speaker_changed", "speaker_memory_block", "unbound_reason", "voiced_refusals"]
