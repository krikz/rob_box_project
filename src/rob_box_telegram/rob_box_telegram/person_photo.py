"""person_photo.py - снимок лица в чат (issue #4469).

Снимок бот берёт у владельца лиц (``vision_face_node`` на Vision Pi) сервисом ``/perception/people`` (``OP_PHOTO``):
сервис отдаёт JPEG байтами, а id приходит в ``DialogOutput(kind=photo)`` (байты по zenoh через диалог не гоняем).
Каталог ``/data/faces`` на Sound Pi для фото не читается: копия там отстаёт от Vision (40 лиц против 107).
"""

from __future__ import annotations

import json
import re
import threading
from typing import Any, Callable, Optional, Tuple

PEOPLE_SERVICE = "/perception/people"
OP_PHOTO = 1
#: Сколько бот ждёт снимок от владельца: JPEG до 200 КБ через сеть между Pi.
CALL_TIMEOUT_S = 8.0
_ID_RE = re.compile(r"[0-9a-f]{8}")
#: Подпись снимка в Telegram - до 1024 знаков; наша короче, это страховка.
CAPTION_MAX = 200

#: ``fetch(id)`` -> ``(ok, reason, jpeg)``; блокирует поток, звать из executor. В тестах подменяется.
Fetch = Callable[[str], Tuple[bool, str, bytes]]

#: Причина отказа владельца -> строка в чат (по-русски, без выдумок).
REFUSALS = {
    "exhibition_mode": "В режиме выставки снимки лиц не отдаю.",
    "no_snapshot": "У этого лица нет снимка.",
    "person_not_found": "Такого лица уже нет в базе.",
    "owner_unavailable": "Владелец лиц (Vision Pi) не ответил, снимок не получил.",
}
REFUSAL_DEFAULT = "Снимок получить не удалось."


def parse_payload(text: str) -> Optional[Tuple[str, str]]:
    """``(id, подпись)`` из текста ``DialogOutput(kind=photo)`` или ``None``, если это не наш JSON или id не 8 hex."""
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    pid = data.get("id")
    if not isinstance(pid, str) or not _ID_RE.fullmatch(pid):
        return None
    return pid, str(data.get("caption") or "")[:CAPTION_MAX]


def ros_fetch(node: Any) -> Fetch:
    """Запрос снимка у владельца по ROS: клиент один на ноду; нет сервиса или ответа - ``owner_unavailable``."""

    def fetch(person_id: str) -> Tuple[bool, str, bytes]:
        from rob_box_perception_msgs.srv import People  # noqa: PLC0415 - ROS-типы только в рантайме ноды

        clients = node.__dict__.setdefault("_people_photo_clients", {})
        if "client" not in clients:
            clients["client"] = node.create_client(People, PEOPLE_SERVICE)
        client = clients["client"]
        if not client.service_is_ready():
            return False, "owner_unavailable", b""
        request = People.Request()
        request.op, request.person_id = OP_PHOTO, person_id
        future, done = client.call_async(request), threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(CALL_TIMEOUT_S):
            return False, "owner_unavailable", b""
        try:
            response = future.result()
            return bool(response.ok), str(response.reason), bytes(response.jpeg)
        except Exception:  # noqa: BLE001 - битый ответ = нет ответа, а не снимок
            return False, "owner_unavailable", b""

    return fetch
