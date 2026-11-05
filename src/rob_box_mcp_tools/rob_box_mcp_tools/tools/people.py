#!/usr/bin/env python3
"""
people.py - тулы пульта Telegram по лицам (issue #4469, ADR-0148 «код решает, LLM говорит»).

Инструменты (срез ``telegram``: видит только агент чата Telegram, голосу и ТАРС не даны):

* :class:`PeopleListTool` - ``people_list(filter)``: лица с id, именем, кличкой, последней встречей, числом встреч и
  связью с голосом.
* :class:`PersonPhotoTool` - ``person_photo(id)``: лучший снимок лица из галереи. Тул отдаёт путь внутри корня
  ``/data/faces`` (``photo_path``), отправляет файл в чат бот Telegram (он монтирует корень на чтение).
* :class:`NamePersonTool` - ``name_person(id, name)``: имя лицу. Пишет владелец (``FaceStore`` в ``vision_face``);
  фразу «Готово» строит код по ответу владельца (диалог берёт её из данных исхода).

Решения принимает владелец данных (сервис ``/perception/people``, ``rob_box_perception.people_service``): проверка id
по существующим записям, допустимое имя, отказ режима ``exhibition``. Тул только зовёт сервис и переводит ответ в
``MCPToolResult``: статус и причина - из ответа владельца, фраза успеха - из его данных.
"""

from __future__ import annotations

import json
from typing import Any, Callable, Dict, List, Optional, Tuple

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType, wait_future

#: Сервис владельца (``vision_face_node``) и значения ``People.srv`` ``op``.
PEOPLE_SERVICE = "/perception/people"
OP_LIST, OP_PHOTO, OP_SET_NAME = 0, 1, 2
#: Сколько тул ждёт ответа владельца: чтение/запись каталога на Vision Pi - миллисекунды, потолок на сеть.
CALL_TIMEOUT_S = 6.0
#: Отказ тула, когда сервис владельца не отвечает (Vision Pi не в сети, нода не поднята).
OWNER_UNAVAILABLE = "owner_unavailable"

#: ``call(op, filter, person_id, name)`` -> ``(ok, reason, data)``; в тестах подменяется.
PeopleCall = Callable[[int, str, str, str], Tuple[bool, str, Any]]


def ros_people_call(node: Any) -> PeopleCall:
    """Вызов сервиса владельца по ROS: клиент один на ноду; не готов или не ответил - ``owner_unavailable``."""

    def call(op: int, flt: str, person_id: str, name: str) -> Tuple[bool, str, Any]:
        from rob_box_perception_msgs.srv import People  # noqa: PLC0415 - ROS-типы только в рантайме ноды

        clients = node.__dict__.setdefault("_people_clients", {})
        if "client" not in clients:
            clients["client"] = node.create_client(People, PEOPLE_SERVICE)
        client = clients["client"]
        if not client.service_is_ready():
            return False, OWNER_UNAVAILABLE, {}
        request = People.Request()
        request.op, request.filter, request.person_id, request.name = op, flt, person_id, name
        future = client.call_async(request)
        if not wait_future(future, CALL_TIMEOUT_S):
            return False, OWNER_UNAVAILABLE, {}
        try:
            response = future.result()
            return bool(response.ok), str(response.reason), json.loads(response.data_json or "{}")
        except Exception:  # noqa: BLE001 - битый ответ владельца = нет ответа, а не успех
            return False, OWNER_UNAVAILABLE, {}

    return call


def _rejected(reason: str) -> MCPToolResult:
    """Отказ: статус и причина - данные результата (``execute.owner_event`` берёт их оттуда), фраза - код по причине."""
    return MCPToolResult(success=False, error=reason, message=f"Отказ: {reason}",
                         data={"status": "rejected", "reason": reason})


def _who(row: Dict[str, Any]) -> str:
    return row.get("name") or row.get("epithet") or "без имени"


def _row_line(row: Dict[str, Any]) -> str:
    seen = row.get("last_encounter") or "нет данных"
    voice = "да" if row.get("voice_linked") else "нет"
    epithet = f" ({row['epithet']})" if row.get("name") and row.get("epithet") else ""
    return f"{row['id']} - {_who(row)}{epithet}; встреч {row.get('encounters', 0)}; последняя {seen}; голос: {voice}"


def list_message(data: Dict[str, Any]) -> str:
    """Готовый текст списка (по данным владельца): модель пересказывает его, а не пересчитывает."""
    rows: List[Dict[str, Any]] = data.get("people", [])
    if not rows:
        return "Лиц по этому фильтру нет."
    head = f"Лиц: {data.get('total', len(rows))}" + (f", показано {len(rows)}" if data.get("total") != len(rows) else "")
    return head + ".\n" + "\n".join(_row_line(r) for r in rows)


class _PeopleCalls:
    """Общее: вызов сервиса владельца. Видимость и срез объявляет каждый тул сам (каталог читает их из класса, AST):
    ``telegram_visible`` без ``llm_visible`` - пульт по людям видит только агент чата Telegram (решение Шифу, #4469)."""

    def __init__(self, node: Any, call: Optional[PeopleCall] = None) -> None:
        super().__init__(node)
        self._call = call if call is not None else ros_people_call(node)


class PeopleListTool(_PeopleCalls, MCPTool):
    """Список лиц, которых робот видел."""

    @property
    def llm_visible(self) -> bool:
        return False  # голосу и ТАРС не даётся: отдельное решение Шифу

    @property
    def telegram_visible(self) -> bool:
        return True

    @property
    def slice(self) -> str:
        return "telegram"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.MEDIUM

    @property
    def name(self) -> str:
        return "people_list"

    @property
    def read_only(self) -> bool:
        return True

    @property
    def description(self) -> str:
        return (
            "Список лиц, которых робот видел камерой: id (8 знаков), имя, кличка, когда видел последний раз, "
            "сколько встреч, связано ли лицо с голосом. Звать на «покажи лица», «кто без имени», «кого видел "
            "сегодня». id из списка передавай в person_photo и name_person как есть. Пересказывай данные результата, "
            "ничего не добавляй."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="filter", type="string", required=False, enum=["all", "unnamed", "today"],
                description="all - все; unnamed - только без имени; today - встречались сегодня. По умолчанию all.",
            ),
        ]

    def execute(self, filter: str = "all") -> MCPToolResult:
        """Список от владельца; отказ - его причина."""
        ok, reason, data = self._call(OP_LIST, filter or "all", "", "")
        if not ok:
            return _rejected(reason or OWNER_UNAVAILABLE)
        return MCPToolResult(success=True, data={"status": "done", **data}, message=list_message(data))


class PersonPhotoTool(_PeopleCalls, MCPTool):
    """Лучший снимок лица - в чат."""

    @property
    def llm_visible(self) -> bool:
        return False  # голосу и ТАРС не даётся: отдельное решение Шифу

    @property
    def telegram_visible(self) -> bool:
        return True

    @property
    def slice(self) -> str:
        return "telegram"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.MEDIUM

    @property
    def name(self) -> str:
        return "person_photo"

    @property
    def read_only(self) -> bool:
        return True

    @property
    def description(self) -> str:
        return (
            "Прислать в чат лучший снимок лица из галереи робота. Звать на «покажи fff371fe», «как выглядит "
            "Кирилл». id - ровно 8 знаков 0-9a-f из people_list; имени вместо id тул не принимает: найди id в "
            "people_list. Картинку в чат отправляет код после вызова, ты её не описываешь и не обещаешь без вызова. "
            "Снимка нет или режим выставки - тул откажет с причиной."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [MCPToolParameter(name="id", type="string", required=True,
                                 description="id лица: 8 знаков 0-9a-f из people_list.")]

    def execute(self, id: str) -> MCPToolResult:  # noqa: A002 - имя параметра в схеме тула
        """Путь к снимку от владельца; отправку в чат делает бот."""
        ok, reason, data = self._call(OP_PHOTO, "", id, "")
        if not ok:
            return _rejected(reason or OWNER_UNAVAILABLE)
        who = data.get("name") or "без имени"
        return MCPToolResult(success=True, message=f"Снимок {data['id']} ({who}) отправляю в чат.",
                             data={"status": "done", "id": data["id"], "photo_path": data["photo_path"],
                                   "caption": f"{data['id']} - {who}"})


class NamePersonTool(_PeopleCalls, MCPTool):
    """Дать или сменить имя лица."""

    #: Событие владельца - ответ его сервиса: имя читается из базы после записи (``stored_name``).
    event_source = "srv:/perception/people"

    @property
    def llm_visible(self) -> bool:
        return False  # голосу и ТАРС не даётся: отдельное решение Шифу

    @property
    def telegram_visible(self) -> bool:
        return True

    @property
    def slice(self) -> str:
        return "telegram"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.MEDIUM

    @property
    def name(self) -> str:
        return "name_person"

    @property
    def idempotent(self) -> bool:
        return True

    @property
    def description(self) -> str:
        return (
            "Дать или сменить имя лицу в базе лиц робота. Звать на «назови fff371fe Кириллом», «это Кирилл» про "
            "лицо из списка. id - ровно 8 знаков 0-9a-f из people_list; имя - до 40 знаков: буквы, цифры, пробел, "
            "точка, дефис, апостроф. Имя записывает владелец базы, а фразу «Готово» говорит код по его ответу: "
            "не пиши «записал» сам. В память (memory_save) id лица не клади."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(name="id", type="string", required=True,
                             description="id лица: 8 знаков 0-9a-f из people_list."),
            MCPToolParameter(name="name", type="string", required=True,
                             description="Имя, как его назвал человек в чате, до 40 знаков."),
        ]

    def execute(self, id: str, name: str) -> MCPToolResult:  # noqa: A002 - имя параметра в схеме тула
        """Запись имени владельцем; ``stored_name`` - имя, прочитанное из базы после записи (доказательство)."""
        ok, reason, data = self._call(OP_SET_NAME, "", id, name)
        if not ok:
            return _rejected(reason or OWNER_UNAVAILABLE)
        stored, previous = data.get("name"), data.get("previous_name")
        if not stored:  # владелец принял, но имя из базы не прочиталось: успеха не говорим
            return _rejected("no_evidence")
        previous = previous if previous and previous != stored else None  # то же имя - не «было X, стало X»
        before = f" (было «{previous}»)" if previous else ""
        return MCPToolResult(success=True, message=f"Готово: лицо {data['id']}{before} теперь {stored}.",
                             data={"status": "done", "id": data["id"], "stored_name": stored,
                                   "previous_name": previous})


__all__ = ["NamePersonTool", "PeopleListTool", "PersonPhotoTool", "list_message", "ros_people_call"]
