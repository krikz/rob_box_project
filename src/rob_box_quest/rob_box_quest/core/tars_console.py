"""События консоли ТАРС 1 (issue #3253, Ш4): фразы оператора и короткие события.

Чистая логика без ROS: из полезной нагрузки двух уже существующих топиков
собирает JSON-события ``tars_console`` для шлема.

* ``/avatar/stt/result``     -> ``kind="operator"``: распознанная фраза оператора;
* ``/avatar/command_result`` -> ``kind="event"``: имена вызванных тулов и
  признак неудачного хода.

Правила ADR-0018 и безопасности: в событие идёт только имя тула (параметры
могут нести подписи/секреты), текст ошибки не пересылается вообще — только
факт ``ok=false``. Нет источника — нет события.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Optional

EVENT_TYPE = "tars_console"
# Имя тула в консоли: обрезаем, чтобы чужой мусор не растягивал строку.
MAX_TOOL_NAME = 48
MAX_OPERATOR_TEXT = 400


def _event(kind: str, text: str, request_id: str, ts_ms: Optional[int] = None) -> dict:
    return {
        "type": EVENT_TYPE,
        "kind": kind,
        "text": text,
        "request_id": request_id,
        "ts_ms": int(time.time() * 1000) if ts_ms is None else ts_ms,
    }


def operator_event(payload: Any) -> Optional[dict]:
    """Фраза оператора из /avatar/stt/result. Пустой текст — события нет."""
    if not isinstance(payload, dict):
        return None
    text = str(payload.get("text", "") or "").strip()
    if not text:
        return None
    client_id = str(payload.get("client_id", "") or "")
    ts_raw = payload.get("ts_ms")
    ts_ms = int(ts_raw) if isinstance(ts_raw, (int, float)) else None
    request_id = f"{client_id}:{ts_ms}" if client_id and ts_ms is not None else ""
    return _event("operator", text[:MAX_OPERATOR_TEXT], request_id, ts_ms)


def operator_events(payload: Any) -> list[dict]:
    """То же списком (0 или 1 событие) — без ветвления в обработчике ноды."""
    ev = operator_event(payload)
    return [] if ev is None else [ev]


def _tool_name(item: Any) -> Optional[str]:
    raw = item.get("name") if isinstance(item, dict) else item
    if not isinstance(raw, str):
        return None
    name = raw.strip()[:MAX_TOOL_NAME]
    return name or None


def turn_events(payload: Any) -> list[dict]:
    """События хода из /avatar/command_result: tool-вызовы и ошибка."""
    if not isinstance(payload, dict):
        return []
    request_id = str(payload.get("request_id", "") or "")
    out: list[dict] = []
    calls = payload.get("tool_calls")
    for item in calls if isinstance(calls, (list, tuple)) else []:
        name = _tool_name(item)
        if name:
            out.append(_event("event", f"tool: {name}", request_id))
    if payload.get("ok") is False:
        out.append(_event("event", "ошибка хода", request_id))
    return out


def relay_events(
    events: list[dict], broadcast: Callable[[dict], Any], log_debug: Callable[[str], Any]
) -> None:
    """Разослать события; сбой WS не должен ронять ноду."""
    for ev in events:
        try:
            broadcast(ev)
        except Exception as e:  # noqa: BLE001
            log_debug(f"tars_console broadcast failed: {e}")
