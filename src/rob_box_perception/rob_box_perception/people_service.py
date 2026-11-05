#!/usr/bin/env python3
"""people_service.py - лица для пульта Telegram: список, снимок, имя (issue #4469, ADR-0148).

Владелец данных - ``FaceStore`` на Vision Pi; здесь решает код, а не модель: id проверяется по существующим записям,
имя - узкой схемой, отказ несёт причину из словаря :data:`REASONS`. Тулы ``mcp_server`` (Sound Pi) только зовут сервис
``/perception/people`` (``rob_box_perception_msgs/srv/People``) и строят фразу об исходе из его ответа.

Модуль не знает про ROS: ``vision_face_node`` оборачивает :meth:`PeopleService.handle` в сервис, тесты идут на
``FaceStore`` во временной папке.
"""

from __future__ import annotations

import json
import re
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional

from .face_store import MODE_EXHIBITION, FaceStore, best_snapshot_bytes

#: Потолок снимка в ответе: байты идут через zenoh между Pi, большое пережимается на Vision.
PHOTO_MAX_BYTES = 200 * 1024

#: Значения ``People.srv`` ``op`` (зеркало констант .srv: модуль не импортирует ROS-типы).
OP_LIST, OP_PHOTO, OP_SET_NAME = 0, 1, 2

#: Фильтр списка - перечисление, а не свободный текст.
FILTERS = ("all", "unnamed", "today")
#: Сколько лиц в одном ответе: сообщение Telegram ограничено 4096 знаками.
LIST_MAX = 30
#: Длина имени и допустимые знаки: буквы любого алфавита, цифры, пробел, точка, дефис, апостроф.
NAME_MAX = 40
_NAME_RE = re.compile(r"[^\W_](?:[\w .'\-]*[^\W_.'\- ])?", re.UNICODE)
#: Id: 8 и больше шестнадцатеричных знаков (дефисы полного uuid допускаются).
_ID_RE = re.compile(r"[0-9a-f][0-9a-f\-]{7,35}")

REASONS = (
    "bad_filter", "bad_id", "bad_name", "person_not_found", "ambiguous_id", "exhibition_mode", "no_snapshot",
    "bad_op", "store_error",
)


@dataclass(frozen=True)
class Reply:
    """Ответ сервиса: ``ok``, причина отказа и данные (JSON-совместимые)."""

    ok: bool
    reason: str = ""
    data: Any = field(default_factory=dict)
    jpeg: bytes = b""


def _refuse(reason: str) -> Reply:
    return Reply(False, reason, {})


def fit_jpeg(raw: bytes, limit: int = PHOTO_MAX_BYTES) -> Optional[bytes]:
    """Снимок не больше ``limit`` байт: влезает - как есть; иначе уменьшаем и пережимаем (cv2). Не вышло - ``None``."""
    if len(raw) <= limit:
        return raw
    try:
        import cv2  # noqa: PLC0415 - тяжёлая зависимость только для редкого большого снимка
        import numpy as np  # noqa: PLC0415

        img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        for _ in range(4):
            for quality in (85, 70, 55):
                ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
                if ok and len(buf) <= limit:
                    return bytes(buf)
            img = cv2.resize(img, None, fx=0.75, fy=0.75, interpolation=cv2.INTER_AREA)
    except Exception:  # noqa: BLE001 - битый снимок или нет cv2: честный отказ, не обрезанный файл
        return None
    return None


def day_start(now: float) -> float:
    """Начало локальных суток ``now`` (секунды epoch)."""
    local = time.localtime(now)
    return time.mktime((local.tm_year, local.tm_mon, local.tm_mday, 0, 0, 0, 0, 0, -1))


def valid_name(name: str) -> bool:
    """Имя годится: 1..:data:`NAME_MAX` знаков из узкого набора, без переводов строки и управляющих знаков."""
    return 0 < len(name) <= NAME_MAX and bool(_NAME_RE.fullmatch(name))


def _iso(ts: float) -> Optional[str]:
    return datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ") if ts else None


def _row(person: Dict[str, Any]) -> Dict[str, Any]:
    ts = float(person.get("last_encounter_ts") or 0.0)
    return {
        "id": str(person["person_id"])[:8],
        "name": person.get("name"),
        "epithet": person.get("epithet"),
        # «последняя встреча по лицу»: не last_seen шва «Знакомый» (ADR-0106), счётчик самого хранилища
        "last_encounter": _iso(ts),
        "last_encounter_ts": ts,
        "encounters": int(person.get("encounter_count") or 0),
        "voice_linked": bool(person.get("speaker_id")),
        "has_snapshot": bool(person.get("has_snapshot")),
    }


class PeopleService:
    """Операции пульта над ``FaceStore``; ``clock`` подменяется в тестах."""

    def __init__(self, store: FaceStore, clock: Callable[[], float] = time.time) -> None:
        self._store = store
        self._clock = clock

    def handle(self, op: int, filter: str = "", person_id: str = "", name: str = "") -> Reply:
        """Выполнить операцию ``op`` (``OP_*``); неизвестная - отказ ``bad_op``."""
        if op == OP_LIST:
            return self._list(filter or "all")
        if op == OP_PHOTO:
            return self._photo(person_id)
        if op == OP_SET_NAME:
            return self._set_name(person_id, name)
        return _refuse("bad_op")

    def _list(self, flt: str) -> Reply:
        if flt not in FILTERS:
            return _refuse("bad_filter")
        people = self._store.people()
        if flt == "unnamed":
            people = [p for p in people if p.get("name") is None]
        elif flt == "today":
            since = day_start(self._clock())
            people = [p for p in people if float(p.get("last_encounter_ts") or 0.0) >= since]
        people.sort(key=lambda p: float(p.get("last_encounter_ts") or 0.0), reverse=True)
        rows: List[Dict[str, Any]] = [_row(p) for p in people[:LIST_MAX]]
        return Reply(True, "", {"people": rows, "total": len(people), "shown": len(rows), "filter": flt})

    def _resolve(self, query: str) -> "str | Reply":
        """Полный id записи по префиксу или отказ: id проверяется по существующим записям (ADR-0148)."""
        q = (query or "").strip().lower()
        if not _ID_RE.fullmatch(q):
            return _refuse("bad_id")
        found = [p["person_id"] for p in self._store.people() if str(p["person_id"]).startswith(q)]
        if not found:
            return _refuse("person_not_found")
        if len(found) > 1:
            return _refuse("ambiguous_id")
        return str(found[0])

    def _photo(self, query: str) -> Reply:
        if self._store.mode == MODE_EXHIBITION:  # снимки лиц режим выставки наружу не отдаёт (ADR-0123)
            return _refuse("exhibition_mode")
        full = self._resolve(query)
        if isinstance(full, Reply):
            return full
        raw = best_snapshot_bytes(self._store, full)
        jpeg = fit_jpeg(raw) if raw else None
        if not jpeg:
            return _refuse("no_snapshot")
        return Reply(True, "", {"id": full[:8], "size": len(jpeg)}, jpeg)

    def _set_name(self, query: str, name: str) -> Reply:
        clean = " ".join((name or "").split())
        if not valid_name(clean):
            return _refuse("bad_name")
        full = self._resolve(query)
        if isinstance(full, Reply):
            return full
        before = next((p.get("name") for p in self._store.people() if p["person_id"] == full), None)
        if not self._store.attach_name(full, clean):
            return _refuse("person_not_found")  # запись исчезла между проверкой и записью
        stored = next((p.get("name") for p in self._store.people() if p["person_id"] == full), None)
        return Reply(True, "", {"id": full[:8], "name": stored, "previous_name": before})


def serve(service: PeopleService, request: Any, response: Any) -> Any:
    """Колбэк ROS-сервиса ``People``: запрос -> :meth:`PeopleService.handle` -> поля ответа. Исключение хранилища -
    отказ ``store_error``, а не тишина клиента."""
    try:
        reply = service.handle(int(request.op), str(request.filter), str(request.person_id), str(request.name))
    except Exception as exc:  # noqa: BLE001 - клиент получает честную причину, нода не падает
        reply = Reply(False, "store_error", {"error": f"{type(exc).__name__}: {exc}"[:200]})
    response.ok, response.reason = reply.ok, reply.reason
    response.jpeg = list(reply.jpeg)
    response.data_json = json.dumps(reply.data, ensure_ascii=False)
    return response


__all__ = ["FILTERS", "LIST_MAX", "NAME_MAX", "OP_LIST", "OP_PHOTO", "OP_SET_NAME", "PeopleService", "REASONS", "Reply",
           "PHOTO_MAX_BYTES", "day_start", "fit_jpeg", "serve", "valid_name"]
