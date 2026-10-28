"""Типизированный ``SpeakerResult`` <-> словарь потребителей (ADR-0150 §10.1, ADR-0131).

До ADR-0150 PR-2 ``/voice/speaker/result`` был ``std_msgs/String`` с JSON, и
пять потребителей (``dialogue_node``, ``mcp_server``, quest, harness, лицо)
читали один и тот же словарь. Сообщение теперь типизировано, а словарь
остаётся внутренним языком потребителей: ЕДИНСТВЕННОЕ место, где знание о
соответствии «поле сообщения <-> ключ словаря» живёт, - этот модуль.
Писатель (``speaker_id_node``) строит словарь так же, как раньше, и кладёт его
в сообщение через :func:`fill_speaker_result`; читатель получает тот же словарь
из :func:`speaker_result_to_payload`.

Модуль не импортирует ROS: сообщение - любой объект с полями ``SpeakerResult``
(настоящий тип или ``rob_box_core.msg_stub``).

:func:`name_state` - что результат фразы говорит об имени собеседника
(``confirmed``/``tentative``/``contested``/``nameless``/``unknown``): одно
правило для диалога и MCP (#4023), выводится из полей сообщения. Значения
``NAME_*`` - единственный словарь состояний имени: ``SpeakerRef.name_state``
диалога и исход ответа на «ты меня узнал?» берут их отсюда (#4141).
:func:`first_heard` - профиль без имени заведён из этой же фразы (#3858):
голос слышен впервые, узнан он быть не мог (#3993).
"""

from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Tuple

EVENT_MATCH = 0
EVENT_REGISTERED = 1
EVENT_REGISTER_ERROR = 2
EVENT_RENAMED = 3
EVENT_MERGED = 4

_EVENT_NAMES = {
    EVENT_REGISTERED: "registered",
    EVENT_REGISTER_ERROR: "register_error",
    EVENT_RENAMED: "renamed",
    EVENT_MERGED: "merged",
}
_EVENT_CODES = {name: code for code, name in _EVENT_NAMES.items()}

# ``tentative_kind`` узнанного голоса, чьё имя не опубликовано (#2809): одна
# гипотеза имени в полосе сомнения или конкурент с ДРУГИМ именем рядом.
TENTATIVE_SINGLE = "single"
TENTATIVE_CONTESTED = "contested"

# Состояние имени результата фразы (#4023, #3993) - что биометрия знает о
# том, КТО говорит. Одна таблица для потребителей (диалог, MCP), без нового
# поля в сообщении: всё нужное уже лежит в ``name``/``tentative_*``.
NAME_CONFIRMED = "confirmed"  # имя опубликовано: биометрия уверена или голос только что зарегистрирован
NAME_TENTATIVE = "tentative"  # одна гипотеза имени, вслух не называть
NAME_CONTESTED = "contested"  # голос похож на знакомого, но кто из похожих - не решено: не приписан никому
NAME_NAMELESS = "nameless"  # профиль без имени, с кличкой (#3858): голос свой, имени нет
NAME_UNKNOWN = "unknown"  # голос не узнан или не оценён
#: Голос узнан, а чей - не подтверждено: одна гипотеза без второго канала или конкурент с другим именем (#2809).
#: Память о людях с таким собеседником не работает (#3993: вопроса «ты Саша?» нет, чужие факты хуже пустых), на «ты
#: меня узнал?» - «голос знакомый, не уверен чей».
NAME_UNCONFIRMED: Tuple[str, ...] = (NAME_TENTATIVE, NAME_CONTESTED)

# ``source`` результата фразы, которым ``speaker_id_node`` сообщает «профиль без имени заведён из ЭТОЙ фразы» (#3858):
# сходства ни с кем не было, голос слышен впервые. Писатель и читатели берут значение отсюда.
SOURCE_ENROLL = "enroll"


def name_state(payload: Mapping[str, Any]) -> str:
    """Состояние имени результата биометрии фразы (словарь :func:`speaker_result_to_payload`).

    ``contested`` - не «незнаком» и не «узнан»: ``speaker_id`` в нём лишь
    ближайший кандидат, которому конкурирует профиль с другим именем, - ни
    имя, ни профиль (а значит, и его факты) этому голосу приписывать нельзя.
    Служебные события (``registered`` и т.п.) - не результат фразы: ``unknown``.
    """
    if payload.get("event") or not payload.get("is_known"):
        return NAME_UNKNOWN
    if payload.get("name"):
        return NAME_CONFIRMED
    if payload.get("tentative_kind") == TENTATIVE_CONTESTED:
        return NAME_CONTESTED
    if payload.get("tentative_name"):
        return NAME_TENTATIVE
    return NAME_NAMELESS


def first_heard(payload: Mapping[str, Any]) -> bool:
    """Профиль без имени заведён из этой же фразы (#3858, ``source`` = :data:`SOURCE_ENROLL`): голос слышен впервые.

    Состояние имени у такого результата - :data:`NAME_NAMELESS` (профиль свой, факты пишутся в него), но узнавать
    голос было не по чему: «ты меня узнал?» на нём - «не узнаю», а не «голос помню» (#3993, ревью PR #4121).
    """
    return name_state(payload) == NAME_NAMELESS and payload.get("source") == SOURCE_ENROLL


# Ключ словаря -> поле сообщения для скалярных значений, одинаковых по смыслу
# во всех событиях. Строки "" в сообщении читаются как отсутствие значения.
_STR_KEYS = (
    "speaker_id", "name", "epithet", "source", "reason", "error",
    "tentative_name", "tentative_kind", "src_speaker_id", "dst_speaker_id",
    "new_name", "candidate_name",
)


def _clean_float(value: float, digits: int = 4) -> float:
    return round(float(value), digits)


def _match_payload(msg: Any) -> Dict[str, Any]:
    if msg.inconclusive:
        payload: Dict[str, Any] = {"is_known": False, "inconclusive": True, "reason": msg.reason}
    elif msg.is_known:
        payload = {
            "is_known": True,
            "speaker_id": msg.speaker_id,
            "name": msg.name or None,
            "confidence": _clean_float(msg.confidence),
            "epithet": msg.epithet or None,
        }
        if msg.source:
            payload["source"] = msg.source
        if msg.tentative_kind:
            payload["tentative_conf"] = _clean_float(msg.tentative_conf)
            payload["tentative_kind"] = msg.tentative_kind
            if msg.tentative_name:
                payload["tentative_name"] = msg.tentative_name
    else:
        payload = {"is_known": False}
    if not msg.is_known and msg.candidate_name:  # гипотеза неузнанного голоса (#3767), не имя
        payload["candidate_name"] = msg.candidate_name
        payload["candidate_score"] = _clean_float(msg.candidate_score)
    if msg.utterance_id:
        payload["utterance_id"] = msg.utterance_id
    return payload


def _registered_payload(msg: Any) -> Dict[str, Any]:
    payload: Dict[str, Any] = {
        "event": "registered",
        "name": msg.name,
        "speaker_id": msg.speaker_id,
        "reused_profile": bool(msg.reused_profile),
        "utterance_id": msg.utterance_id or None,
    }
    if msg.already_known:
        payload["already_known"] = True
    if msg.new_name:  # регистрация переименовала уверенно узнанный профиль голоса фразы (#4054)
        payload["new_name"] = msg.new_name
    if msg.conflict_name:
        payload["voice_conflict"] = {
            "name": msg.conflict_name,
            "speaker_id": msg.conflict_speaker_id,
            "score": _clean_float(msg.conflict_score),
        }
    if msg.twin_name:
        score = float(msg.twin_score)
        payload["name_twin"] = {
            "name": msg.twin_name,
            "speaker_id": msg.twin_speaker_id,
            "score": None if math.isnan(score) else _clean_float(score),
        }
    return payload


def _register_error_payload(msg: Any) -> Dict[str, Any]:
    payload: Dict[str, Any] = {
        "event": "register_error",
        "error": msg.error,
        "name": msg.name,
        "utterance_id": msg.utterance_id or None,
    }
    if msg.error == "too_short":
        payload["duration_s"] = round(float(msg.duration_s), 2)
        payload["min_required_s"] = round(float(msg.min_required_s), 2)
    return payload


def speaker_result_to_payload(msg: Any) -> Dict[str, Any]:
    """Сообщение ``SpeakerResult`` -> словарь, который читают потребители."""
    payload = _event_payload(msg)
    if bool(getattr(msg, "e2e_db", False)):  # результат из E2E-БД дикторов (#3876)
        payload["e2e_db"] = True
    return payload


def _event_payload(msg: Any) -> Dict[str, Any]:
    event = int(msg.event)
    if event == EVENT_MATCH:
        return _match_payload(msg)
    if event == EVENT_REGISTERED:
        return _registered_payload(msg)
    if event == EVENT_REGISTER_ERROR:
        return _register_error_payload(msg)
    if event == EVENT_RENAMED:
        return {"event": "renamed", "ok": bool(msg.ok), "speaker_id": msg.speaker_id,
                "new_name": msg.new_name}
    if event == EVENT_MERGED:
        return {"event": "merged", "ok": bool(msg.ok), "src_speaker_id": msg.src_speaker_id,
                "dst_speaker_id": msg.dst_speaker_id,
                "embeddings_moved": int(msg.embeddings_moved),
                "facts_moved": int(msg.facts_moved)}
    raise ValueError(f"неизвестный SpeakerResult.event={event}")


def _fill_nested(msg: Any, payload: Mapping[str, Any]) -> None:
    conflict = payload.get("voice_conflict")
    if conflict:
        msg.conflict_name = str(conflict.get("name") or "")
        msg.conflict_speaker_id = str(conflict.get("speaker_id") or "")
        msg.conflict_score = float(conflict.get("score") or 0.0)
    twin = payload.get("name_twin")
    if twin:
        msg.twin_name = str(twin.get("name") or "")
        msg.twin_speaker_id = str(twin.get("speaker_id") or "")
        score = twin.get("score")
        msg.twin_score = math.nan if score is None else float(score)


def fill_speaker_result(msg: Any, payload: Mapping[str, Any]) -> Any:
    """Записать словарь писателя в сообщение ``SpeakerResult`` и вернуть его.

    Ключ ``event`` выбирает вид сообщения; неизвестное событие - ошибка
    (писатель обязан знать контракт), а не тихо потерянный ack.
    """
    name = payload.get("event")
    if name is None:
        msg.event = EVENT_MATCH
    elif name in _EVENT_CODES:
        msg.event = _EVENT_CODES[name]
    else:
        raise ValueError(f"неизвестное событие SpeakerResult: {name!r}")
    msg.utterance_id = str(payload.get("utterance_id") or "")
    for key in _STR_KEYS:
        if payload.get(key) is not None:
            setattr(msg, key, str(payload[key]))
    msg.is_known = bool(payload.get("is_known", False))
    msg.inconclusive = bool(payload.get("inconclusive", False))
    msg.ok = bool(payload.get("ok", False))
    msg.reused_profile = bool(payload.get("reused_profile", False))
    msg.already_known = bool(payload.get("already_known", False))
    msg.e2e_db = bool(payload.get("e2e_db", False))
    for key, field in (("confidence", "confidence"), ("tentative_conf", "tentative_conf"),
                       ("duration_s", "duration_s"), ("min_required_s", "min_required_s"),
                       ("candidate_score", "candidate_score")):
        if payload.get(key) is not None:
            setattr(msg, field, float(payload[key]))
    for key in ("embeddings_moved", "facts_moved"):
        if payload.get(key) is not None:
            setattr(msg, key, int(payload[key]))
    _fill_nested(msg, payload)
    return msg
