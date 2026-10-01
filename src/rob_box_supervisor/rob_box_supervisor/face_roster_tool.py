"""Инструмент ТАРС ``list_faces`` — список лиц из лицевой базы (issue #3299).

Оператор просит «выведи список распознанных лиц», а ТАРС отвечал, что
доступа к камере у него нет: лицевая база была только в Telegram
(``/faces``, #3025). Инструмент читает ту же базу тем же общим кодом
(:mod:`rob_box_core.face_roster`), что и Telegram-бот.

Почему локальный инструмент ТАРС, а не MCP-инструмент в voice-assistant
-----------------------------------------------------------------------
* Список лиц — персональные данные (ADR-0123): он нужен только оператору
  за PIN, а не личности, которая болтает с гостями. Локальный инструмент
  не попадает в MCP-каталог вообще — личность его физически не видит, без
  правки среза ``slice_policy.yaml`` и каталога ``_tool_catalog_data``.
* Том ``/data/faces`` монтируется read-only в avatar-supervisor
  (docker/vision/docker-compose.yaml) — тот же приём, что для telegram-bot.
* Образец — ``show_metrics`` (:mod:`rob_box_supervisor.tars_panel`).

Только чтение. В ответе нет эмбеддингов, путей к файлам и снимков.
Режим приватности (workshop/exhibition/strict) решает ``FaceStore`` при
записи: инструмент показывает ровно то, что реально лежит на диске, и
честно говорит «база пуста / недоступна», а не выдумывает.
"""

from __future__ import annotations

import datetime as _dt
import logging
import os
import time
from typing import Any, Dict, List, Mapping, Optional

from rob_box_core.face_roster import FaceSummary, list_people

_LOG = logging.getLogger(__name__)

#: Куда в контейнере смонтирована лицевая база (``:ro``).
DEFAULT_FACES_ROOT = "/data/faces"
TOOL_NAME = "list_faces"
DEFAULT_LIMIT = 10
MAX_LIMIT = 30


def _last_seen_ts(s: FaceSummary) -> Optional[float]:
    # Последняя встреча, иначе создание записи.
    return s.last_encounter_ts or s.created_at


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
        return _dt.datetime.fromtimestamp(ts, tz=_dt.timezone.utc).strftime(
            "%Y-%m-%d %H:%M UTC"
        )
    except (OverflowError, OSError, ValueError):
        return None


def _row(s: FaceSummary, now: float) -> Dict[str, Any]:
    ts = _last_seen_ts(s)
    return {
        "id": s.person_id_short,
        "name": s.name or "незнакомец",
        "encounters": s.encounter_count,
        "last_seen": _iso_utc(ts),
        "last_seen_ago": _ago_ru(ts, now),
    }


def build_faces_listing(
    root: str,
    *,
    limit: int = DEFAULT_LIMIT,
    named_only: bool = False,
    now: Optional[float] = None,
) -> Dict[str, Any]:
    """Короткий список лиц для ответа LLM: новые встречи — первыми."""
    now_ts = time.time() if now is None else now
    limit = max(1, min(int(limit), MAX_LIMIT))
    if not os.path.isdir(root):
        return {
            "status": "unavailable",
            "message": "Лицевая база недоступна: каталог лиц не примонтирован.",
        }
    people = list_people(root, load_gallery=False)
    total = len(people)
    named = sum(1 for p in people if p.name)
    if named_only:
        people = [p for p in people if p.name]
    people.sort(key=lambda p: _last_seen_ts(p) or 0.0, reverse=True)
    shown = people[:limit]
    if total == 0:
        message = "Лицевая база пуста."
    else:
        message = (
            f"В базе {total} лиц (с именем {named}), показано {len(shown)}, "
            "новые встречи первыми."
        )
    return {
        "status": "ok",
        "total_in_base": total,
        "named_in_base": named,
        "shown": len(shown),
        "faces": [_row(p, now_ts) for p in shown],
        "message": message,
    }


def _spec() -> Any:
    from rob_box_harness.tools import ToolSpec  # noqa: PLC0415

    return ToolSpec(
        name=TOOL_NAME,
        description=(
            "List the faces the robot has recognised (read-only face base). "
            "Returns a short list sorted by most recent encounter: id, name "
            "('незнакомец' = not yet named), number of encounters and when "
            "the person was last seen. No photos or embeddings. Use for "
            "'выведи список распознанных лиц', 'кого ты видел', 'кто в базе "
            "лиц'. Answer the operator with the names and last-seen times "
            "that came back; if status is not ok, say the face base is "
            "unavailable instead of guessing."
        ),
        parameters={
            "type": "object",
            "properties": {
                "limit": {
                    "type": "integer",
                    "description": (
                        f"How many faces to return, default {DEFAULT_LIMIT}, "
                        f"max {MAX_LIMIT}."
                    ),
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


def register_list_faces_tool(registry: Any, *, root: Optional[str] = None) -> None:
    """Зарегистрировать ``list_faces`` в :class:`ToolRegistry` оператора."""
    faces_root = root or os.environ.get("FACE_STORE_ROOT", DEFAULT_FACES_ROOT)

    def _handler(args: Mapping[str, Any]) -> Dict[str, Any]:
        try:
            limit = int(args.get("limit") or DEFAULT_LIMIT)
        except (TypeError, ValueError):
            limit = DEFAULT_LIMIT
        return build_faces_listing(
            faces_root, limit=limit, named_only=bool(args.get("named_only"))
        )

    registry.register(_spec(), _handler, override=True)
    _LOG.info("[face_roster_tool] %s registered (root=%s)", TOOL_NAME, faces_root)


__all__: List[str] = ["TOOL_NAME", "build_faces_listing", "register_list_faces_tool"]
