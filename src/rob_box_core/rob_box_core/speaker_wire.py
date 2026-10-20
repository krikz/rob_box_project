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
"""

from __future__ import annotations

import math
from typing import Any, Dict, Mapping

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

# Ключ словаря -> поле сообщения для скалярных значений, одинаковых по смыслу
# во всех событиях. Строки "" в сообщении читаются как отсутствие значения.
_STR_KEYS = (
    "speaker_id", "name", "epithet", "source", "reason", "error",
    "tentative_name", "tentative_kind", "src_speaker_id", "dst_speaker_id",
    "new_name",
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
    for key, field in (("confidence", "confidence"), ("tentative_conf", "tentative_conf"),
                       ("duration_s", "duration_s"), ("min_required_s", "min_required_s")):
        if payload.get(key) is not None:
            setattr(msg, field, float(payload[key]))
    for key in ("embeddings_moved", "facts_moved"):
        if payload.get(key) is not None:
            setattr(msg, key, int(payload[key]))
    _fill_nested(msg, payload)
    return msg
