"""Инструмент ТАРС ``list_faces`` — список лиц у владельца галереи (issue #3655).

Оператор просит «выведи список распознанных лиц». Тул зовёт уже существующий
сервис ``/perception/people`` (``People.srv`` ``OP_LIST``, issue #4469): галерею
держит ``vision-face`` на Vision Pi. Локальный диск ``/data/faces`` на Sound Pi
не читается — это копия-снимок, не источник.

Почему локальный инструмент ТАРС, а не MCP ``people_list``
----------------------------------------------------------
* Список лиц — персональные данные (ADR-0123): он нужен только оператору
  за PIN, а не личности. Локальный инструмент не попадает в MCP-каталог.
  MCP ``people_list`` — срез ``telegram`` (решение Шифу, #4469).
* Образец — ``show_metrics`` (:mod:`rob_box_supervisor.tars_panel`).

Только чтение. В ответе нет эмбеддингов, путей к файлам и снимков.
Новый протокол не вводим: те же ``op``/``filter``, что у ``People.srv``.
Vision не ответил — честный ``unavailable``, не «лиц нет» (ADR-0148).
"""

from __future__ import annotations

import datetime as _dt
import json
import logging
import threading
import time
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

_LOG = logging.getLogger(__name__)

TOOL_NAME = "list_faces"
DEFAULT_LIMIT = 10
MAX_LIMIT = 30

#: Контракт владельца (``People.srv`` / ``people_service.py``). Не новый протокол.
PEOPLE_SERVICE = "/perception/people"
OP_LIST = 0
OWNER_UNAVAILABLE = "owner_unavailable"
CALL_TIMEOUT_S = 6.0

PeopleListCall = Callable[[], Tuple[bool, str, Any]]


def _ago_ru(ts: Optional[float], now: float) -> str:
    if not ts or ts <= 0:
        return "время неизвестно"
    delta = max(0, int(now - ts))
    if delta < 90:
        return "только что"
    if delta < 3600:
        return f"{delta // 60} мин назад"
    if delta < 86400:
        return f"{delta // 3600} ч назад"
    return f"{delta // 86400} дн назад"


def _iso_utc(ts: Optional[float]) -> Optional[str]:
    if not ts or ts <= 0:
        return None
    try:
        return _dt.datetime.fromtimestamp(ts, tz=_dt.timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    except (OverflowError, OSError, ValueError):
        return None


def _ts(row: Mapping[str, Any]) -> Optional[float]:
    raw = row.get("last_encounter_ts")
    try:
        ts = float(raw)
    except (TypeError, ValueError):
        return None
    return ts if ts > 0 else None


def _row(person: Mapping[str, Any], now: float) -> Dict[str, Any]:
    ts = _ts(person)
    return {
        "id": str(person.get("id") or "")[:8],
        "name": person.get("name") or "незнакомец",
        "encounters": int(person.get("encounters") or 0),
        "last_seen": _iso_utc(ts),
        "last_seen_ago": _ago_ru(ts, now),
    }


def unavailable_listing(reason: str = OWNER_UNAVAILABLE) -> Dict[str, Any]:
    """Vision не ответил: не «лиц нет» (ADR-0148, ADR-0159 В8)."""
    return {
        "status": "unavailable",
        "reason": reason or OWNER_UNAVAILABLE,
        "message": "Владелец лиц (Vision Pi) не ответил; список лиц получить не удалось.",
    }


def listing_from_owner(
    data: Mapping[str, Any],
    *,
    limit: int = DEFAULT_LIMIT,
    named_only: bool = False,
    now: Optional[float] = None,
) -> Dict[str, Any]:
    """Короткий список из ответа ``OP_LIST``: новые встречи — первыми у владельца."""
    now_ts = time.time() if now is None else now
    limit = max(1, min(int(limit), MAX_LIMIT))
    raw_people = data.get("people")
    people: Sequence[Mapping[str, Any]] = raw_people if isinstance(raw_people, list) else []
    try:
        total = int(data.get("total") if data.get("total") is not None else len(people))
    except (TypeError, ValueError):
        total = len(people)
    named = sum(1 for p in people if isinstance(p, Mapping) and p.get("name"))
    rows = [p for p in people if isinstance(p, Mapping)]
    if named_only:
        rows = [p for p in rows if p.get("name")]
    shown = rows[:limit]
    if total == 0:
        message = "Лицевая база пуста."
    else:
        message = (
            f"В базе {total} лиц (с именем {named} среди ответа владельца), "
            f"показано {len(shown)}, новые встречи первыми."
        )
    return {
        "status": "ok",
        "total_in_base": total,
        "named_in_base": named,
        "shown": len(shown),
        "faces": [_row(p, now_ts) for p in shown],
        "message": message,
    }


def ros_people_list(node: Any) -> PeopleListCall:
    """Вызов ``/perception/people`` ``OP_LIST``; нет сервиса или ответа — ``owner_unavailable``."""

    def call() -> Tuple[bool, str, Any]:
        from rob_box_perception_msgs.srv import People  # noqa: PLC0415

        clients = node.__dict__.setdefault("_tars_people_clients", {})
        if "client" not in clients:
            clients["client"] = node.create_client(People, PEOPLE_SERVICE)
        client = clients["client"]
        if not client.service_is_ready():
            return False, OWNER_UNAVAILABLE, {}
        request = People.Request()
        request.op, request.filter, request.person_id, request.name = OP_LIST, "all", "", ""
        future, done = client.call_async(request), threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(CALL_TIMEOUT_S):
            return False, OWNER_UNAVAILABLE, {}
        try:
            response = future.result()
            payload = json.loads(response.data_json or "{}")
            if not isinstance(payload, dict):
                return False, OWNER_UNAVAILABLE, {}
            return bool(response.ok), str(response.reason), payload
        except Exception:  # noqa: BLE001 - битый ответ владельца = нет ответа, не пустой список
            return False, OWNER_UNAVAILABLE, {}

    return call


def _spec() -> Any:
    from rob_box_harness.tools import ToolSpec  # noqa: PLC0415

    return ToolSpec(
        name=TOOL_NAME,
        description=(
            "List the faces the robot has recognised (read-only, from the "
            "Vision Pi owner service /perception/people). Returns a short "
            "list sorted by most recent encounter: id, name "
            "('незнакомец' = not yet named), number of encounters and when "
            "the person was last seen. No photos or embeddings. Use for "
            "'выведи список распознанных лиц', 'кого ты видел', 'кто в базе "
            "лиц'. Answer the operator with the names and last-seen times "
            "that came back; if status is not ok, say the face owner on "
            "Vision Pi did not answer instead of guessing."
        ),
        parameters={
            "type": "object",
            "properties": {
                "limit": {
                    "type": "integer",
                    "description": (f"How many faces to return, default {DEFAULT_LIMIT}, " f"max {MAX_LIMIT}."),
                    "default": DEFAULT_LIMIT,
                },
                "named_only": {
                    "type": "boolean",
                    "description": "Only people who already have a name.",
                    "default": False,
                },
            },
            "required": [],
            "additionalProperties": False,
        },
    )


def register_list_faces_tool(
    registry: Any,
    *,
    call: Optional[PeopleListCall] = None,
    node: Any = None,
) -> None:
    """Зарегистрировать ``list_faces`` в :class:`ToolRegistry` оператора."""
    people_call = call if call is not None else (ros_people_list(node) if node is not None else None)

    def _handler(args: Mapping[str, Any]) -> Dict[str, Any]:
        try:
            limit = int(args.get("limit") or DEFAULT_LIMIT)
        except (TypeError, ValueError):
            limit = DEFAULT_LIMIT
        if people_call is None:
            return unavailable_listing()
        ok, reason, data = people_call()
        if not ok:
            return unavailable_listing(reason or OWNER_UNAVAILABLE)
        return listing_from_owner(data, limit=limit, named_only=bool(args.get("named_only")))

    registry.register(_spec(), _handler, override=True)
    _LOG.info("[face_roster_tool] %s registered (service=%s)", TOOL_NAME, PEOPLE_SERVICE)


__all__: List[str] = [
    "CALL_TIMEOUT_S",
    "OP_LIST",
    "OWNER_UNAVAILABLE",
    "PEOPLE_SERVICE",
    "TOOL_NAME",
    "listing_from_owner",
    "register_list_faces_tool",
    "ros_people_list",
    "unavailable_listing",
]
