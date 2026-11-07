#!/usr/bin/env python3
"""
people.py - тулы пульта Telegram по лицам (issue #4469, ADR-0148 «код решает, LLM говорит»).

Инструменты (срез ``telegram``: видит только агент чата Telegram, голосу и ТАРС не даны):

* :class:`PeopleListTool` - ``people_list(filter)``: лица с id, именем, кличкой, последней встречей, числом встреч и
  связью с голосом.
* :class:`PersonPhotoTool` - ``person_photo(id)``: лучший снимок лица из галереи. Тул проверяет у владельца, что
  снимок есть (сервис отдаёт JPEG байтами), и отдаёт id (``photo_id``); картинку бот Telegram берёт у владельца сам по
  этому id (байты в ``DialogOutput`` не гоняем), диск лиц на Sound Pi не читается.
* :class:`NamePersonTool` - ``name_person(id, name)``: имя лицу. Пишет владелец (``FaceStore`` в ``vision_face``);
  фразу «Готово» строит код по ответу владельца (диалог берёт её из данных исхода).
* :class:`FindSimilarTool` - ``find_similar(id)``: код владельца считает, кто похож на лицо; в чат уходит один коллаж
  с номерами 1..k, а номера -> id запоминает :class:`CollageMemory` (модель номера не угадывает).
* :class:`MergePeopleTool` - ``merge_people(target, numbers | all_except)``: код переводит номера последнего коллажа
  в id (нет коллажа или номер вне диапазона - честный отказ), спрашивает подтверждение одной строкой (окно ответа
  диалога, ``knowledge.TOOL_QUESTIONS``) и только после «да» зовёт владельца; фразу «склеено N, теперь у <имя> M
  снимков» строит код из его ответа.

Решения принимает владелец данных (сервис ``/perception/people``, ``rob_box_perception.people_service``): проверка id
по существующим записям, допустимое имя, отказ режима ``exhibition``. Тул только зовёт сервис и переводит ответ в
``MCPToolResult``: статус и причина - из ответа владельца, фраза успеха - из его данных.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType, wait_future

#: Сервис владельца (``vision_face_node``) и значения ``People.srv`` ``op``.
PEOPLE_SERVICE = "/perception/people"
OP_LIST, OP_PHOTO, OP_SET_NAME, OP_SIMILAR, OP_MERGE = 0, 1, 2, 3, 4
#: Сколько тул ждёт ответа владельца: чтение/запись каталога на Vision Pi - миллисекунды, потолок на сеть.
CALL_TIMEOUT_S = 6.0
#: Отказ тула, когда сервис владельца не отвечает (Vision Pi не в сети, нода не поднята).
OWNER_UNAVAILABLE = "owner_unavailable"

#: Причины отказа самих тулов склейки (до владельца): нет коллажа / коллаж про другое лицо / номер вне диапазона /
#: не выбрано ни одного номера. Фразы - ``phrases/ru.yaml``, ключи ``local.rejected.*``.
NO_COLLAGE, COLLAGE_MISMATCH, BAD_NUMBER, BAD_SELECTION = "no_collage", "collage_mismatch", "bad_number", "bad_selection"
#: Подтверждение склейки - вопрос человеку (``knowledge.TOOL_QUESTIONS``): тул без ``confirm`` только спрашивает.
NEEDS_MERGE_CONFIRMATION = "needs_merge_confirmation"
#: Сколько живёт коллаж в памяти тула: «1, 3, 5 - да» через час уже про другой разговор.
COLLAGE_TTL_S = 1800.0

#: Причины отказа самих тулов склейки (до владельца): нет коллажа / коллаж про другое лицо / номер вне диапазона /
#: не выбрано ни одного номера. Фразы - ``phrases/ru.yaml``, ключи ``local.rejected.*``.
NO_COLLAGE, COLLAGE_MISMATCH, BAD_NUMBER, BAD_SELECTION = "no_collage", "collage_mismatch", "bad_number", "bad_selection"
#: Подтверждение склейки - вопрос человеку (``knowledge.TOOL_QUESTIONS``): тул без ``confirm`` только спрашивает.
NEEDS_MERGE_CONFIRMATION = "needs_merge_confirmation"
#: Сколько живёт коллаж в памяти тула: «1, 3, 5 - да» через час уже про другой разговор.
COLLAGE_TTL_S = 1800.0

#: ``call(op, filter, person_id, name)`` -> ``(ok, reason, data)``; в тестах подменяется. Для ``OP_MERGE`` в
#: ``filter`` идут id склеиваемых записей через запятую, в ``person_id`` - цель (поле переиспользовано, .srv не меняем).
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
        """Владелец подтвердил снимок (байты тул не несёт); id уходит боту, он заберёт снимок у владельца сам."""
        ok, reason, data = self._call(OP_PHOTO, "", id, "")
        if not ok:
            return _rejected(reason or OWNER_UNAVAILABLE)
        who = _who(data)
        return MCPToolResult(success=True, message=f"Снимок {data['id']} ({who}) отправляю в чат.",
                             data={"status": "done", "id": data["id"], "photo_id": data["id"],
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


@dataclass(frozen=True)
class Collage:
    """Последний коллаж «найди похожих»: искомое лицо, номера -> id (8 знаков) и когда он собран."""

    target: str
    target_name: Optional[str]
    ids: Tuple[str, ...]  # ids[n - 1] - id плитки с номером n
    made_at: float


class CollageMemory:
    """Номера коллажа -> id хранит код, а не модель (#4469, ADR-0148): модель говорит «1, 3, 5», перевод в id - здесь.

    Помнит только последний коллаж и недолго (:data:`COLLAGE_TTL_S`): номера без коллажа или со старого коллажа
    ничего не значат. Состояние живёт в процессе тулов (``mcp_server``), потому что перевод номеров идёт в тулах до
    вызова владельца; окно ответа на подтверждение - отдельно, в сессии диалога (``tool_ask``)."""

    def __init__(self, clock: Callable[[], float] = time.time, ttl_s: float = COLLAGE_TTL_S) -> None:
        self._clock, self._ttl = clock, ttl_s
        self._last: Optional[Collage] = None

    def remember(self, target: str, target_name: Optional[str], ids: Sequence[str]) -> None:
        self._last = Collage(target, target_name, tuple(ids), self._clock())

    def latest(self) -> Optional[Collage]:
        last = self._last
        return last if last is not None and self._clock() - last.made_at <= self._ttl else None

    def forget(self) -> None:
        self._last = None


#: Память коллажа по умолчанию - одна на процесс: ``find_similar`` пишет, ``merge_people`` читает.
COLLAGES = CollageMemory()


def _whole(values: Any) -> Optional[List[int]]:
    """Список целых из аргумента тула или ``None``, если это не список целых (``True``, дробь, строка - не номер)."""
    if values in (None, ""):
        return []
    if not isinstance(values, (list, tuple)):
        values = [values]
    if any(isinstance(v, bool) or not isinstance(v, int) for v in values):
        return None
    return list(values)


def choose_ids(collage: Collage, numbers: Any, all_except: Any) -> Tuple[List[str], str]:
    """Номера выбора -> id по коллажу. Возвращает ``(id, причина отказа)``: причина пуста, когда выбор годится.

    ``numbers`` - выбранные номера; ``all_except`` - «все, кроме». Нужно ровно одно из двух. Номер вне ``1..k`` или не
    целое - ``bad_number``; ничего не выбрано (или оба поля, или исключили всё) - ``bad_selection``."""
    chosen, excluded = _whole(numbers), _whole(all_except)
    if chosen is None or excluded is None:
        return [], BAD_NUMBER
    if bool(chosen) == bool(excluded):  # оба или ни одного
        return [], BAD_SELECTION
    if any(not 1 <= n <= len(collage.ids) for n in chosen + excluded):
        return [], BAD_NUMBER
    picked = chosen if chosen else [n for n in range(1, len(collage.ids) + 1) if n not in set(excluded)]
    ordered = sorted(set(picked))
    return [collage.ids[n - 1] for n in ordered], "" if ordered else BAD_SELECTION


def _plural(n: int, one: str, few: str, many: str) -> str:
    if n % 10 == 1 and n % 100 != 11:
        return one
    return few if 2 <= n % 10 <= 4 and not 12 <= n % 100 <= 14 else many


def similar_message(data: Dict[str, Any]) -> str:
    """Текст к коллажу по данным владельца: номер, id, имя и сходство каждого кандидата."""
    rows: List[Dict[str, Any]] = data.get("candidates", [])
    who = data.get("name") or "без имени"
    if not rows:
        return f"Похожих на {data['id']} ({who}) не нашёл."
    lines = [f"{r['n']}. {r['id']} - {r.get('name') or 'без имени'}, сходство {round(float(r['similarity']) * 100)}%"
             + (" (тёзка)" if r.get("same_name") else "") for r in rows]
    return f"Похожие на {data['id']} ({who}): {len(rows)}. Коллаж в чате.\n" + "\n".join(lines)


def merge_question(count: int, name: Optional[str], target: str) -> str:
    """Подтверждение одной строкой перед слиянием: «Склеиваю 3 записи в «Имя» (71cda108)? Скажи «да, склей».»"""
    noun = _plural(count, "запись", "записи", "записей")
    return f"Склеиваю {count} {noun} в «{name or 'без имени'}» ({target})? Скажи «да, склей»."


def merge_summary(data: Dict[str, Any]) -> str:
    """Фраза исхода по факту владельца: «Склеено N, теперь у <имя> M снимков.» (имя и число - из базы после записи)."""
    who = data.get("name") or f"лица {data['id']}"
    gallery = int(data.get("gallery") or 0)
    shots = _plural(gallery, "снимок", "снимка", "снимков")
    return f"Склеено {data['merged']}, теперь у {who} {gallery} {shots}."


class _CollageCalls(_PeopleCalls):
    """Тулы коллажа делят память номеров (:class:`CollageMemory`)."""

    def __init__(self, node: Any, call: Optional[PeopleCall] = None, memory: Optional[CollageMemory] = None) -> None:
        super().__init__(node, call)
        self._memory = memory if memory is not None else COLLAGES


class FindSimilarTool(_CollageCalls, MCPTool):
    """Найти записи, похожие на лицо: код считает сходство, коллаж уходит в чат."""

    #: Событие владельца - ответ его сервиса: кандидатов и коллаж собирает FaceStore на Vision Pi.
    event_source = "srv:/perception/people"

    @property
    def llm_visible(self) -> bool:
        return False  # голосу и ТАРС не даётся: пульт по лицам только в Telegram (#4469)

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
        return "find_similar"

    @property
    def read_only(self) -> bool:
        return True

    @property
    def description(self) -> str:
        return (
            "Найти в базе лиц записи, похожие на данное (дубли одного человека): код сравнивает лица, ты не решаешь, "
            "кто похож. В чат уходит ОДИН коллаж: снимки с крупными номерами 1, 2, 3…, под каждым id и сходство; "
            "дубли с тем же именем первыми. Звать на «найди похожих», «найди дубли 71cda108». id - 8 знаков 0-9a-f "
            "из people_list. Дальше человек пишет номера («1, 3, 5 - да», «все, кроме 2»): склей их тулом "
            "merge_people, номера не переводи в id сам. Коллаж отправляет код после вызова."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [MCPToolParameter(name="id", type="string", required=True,
                                 description="id лица, для которого ищем дубли: 8 знаков 0-9a-f.")]

    def execute(self, id: str) -> MCPToolResult:  # noqa: A002 - имя параметра в схеме тула
        """Кандидаты от владельца; номера -> id запоминает код (``CollageMemory``), картинку коллажа несёт бот."""
        ok, reason, data = self._call(OP_SIMILAR, "", id, "")
        if not ok:
            return _rejected(reason or OWNER_UNAVAILABLE)
        rows = data.get("candidates", [])
        if not rows:
            self._memory.forget()  # старые номера про другое лицо: не оставляем
            return MCPToolResult(success=True, message=similar_message(data),
                                 data={"status": "done", "id": data["id"], "candidates": [],
                                       "summary": similar_message(data)})
        self._memory.remember(data["id"], data.get("name"), [r["id"] for r in rows])
        return MCPToolResult(success=True, message=similar_message(data),
                             data={"status": "done", "id": data["id"], "candidates": rows, "photo_id": data["id"],
                                   "photo_kind": "collage", "summary": f"Похожих на {data['id']}: {len(rows)}, коллаж выше. Назови номера.",
                                   "caption": f"Похожие на {data['id']} ({data.get('name') or 'без имени'})"})


class MergePeopleTool(_CollageCalls, MCPTool):
    """Склеить дубли лица по номерам последнего коллажа."""

    #: Событие владельца - ответ его сервиса: число склеенных и имя читаются из базы после записи.
    event_source = "srv:/perception/people"

    @property
    def llm_visible(self) -> bool:
        return False  # необратимое: только пульту Telegram, голосу и ТАРС не даётся (#4469)

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
        return "merge_people"

    @property
    def description(self) -> str:
        return (
            "Склеить дубли лица: записи с номерами последнего коллажа find_similar вливаются в target. Звать, когда "
            "человек ответил на коллаж номерами: «1, 3, 5 - да» -> numbers=[1,3,5]; «все, кроме 2» -> "
            "all_except=[2]. Передай ровно одно из двух. Номера - с коллажа, id ты не подставляешь: перевод в id "
            "делает код и откажет, если коллажа нет или номер не из него. Без confirm тул только спрашивает "
            "(needs_merge_confirmation), confirm=true - после «да» человека. Слияние делает владелец базы с бэкапом, "
            "а фразу «склеено N, теперь у … M снимков» говорит код по его ответу: не пиши «склеил» сам."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(name="target", type="string", required=True,
                             description="id лица, в которое склеиваем (то, для которого делали коллаж): 8 знаков."),
            MCPToolParameter(name="numbers", type="array", required=False,
                             items=MCPToolParameter(name="n", type="integer", description="Номер с коллажа."),
                             description="Номера коллажа, которые склеить (человек сказал «1, 3, 5 - да»)."),
            MCPToolParameter(name="all_except", type="array", required=False,
                             items=MCPToolParameter(name="n", type="integer", description="Номер с коллажа."),
                             description="Склеить все номера коллажа, кроме этих («все, кроме 2»)."),
            MCPToolParameter(name="confirm", type="boolean", required=False, default=False,
                             description="true - только после того, как человек ответил «да» на вопрос «Склеиваю …?» "
                                         "(его задаёт сам тул ответом needs_merge_confirmation)."),
        ]

    def execute(self, target: str, numbers: Any = None, all_except: Any = None, confirm: Any = False,
                **_: Any) -> MCPToolResult:
        """Номера -> id по коллажу, вопрос, затем слияние у владельца; отказ - причина, а не тихое «готово»."""
        collage = self._memory.latest()
        if collage is None:
            return _rejected(NO_COLLAGE)
        if (target or "").strip().lower()[:8] != collage.target:
            return _rejected(COLLAGE_MISMATCH)
        ids, refusal = choose_ids(collage, numbers, all_except)
        if refusal:
            return _rejected(refusal)
        if not (confirm is True or str(confirm).lower() == "true"):
            question = merge_question(len(ids), collage.target_name, collage.target)
            return MCPToolResult(success=False, error=question, message=question,
                                 data={"status": "rejected", "reason": NEEDS_MERGE_CONFIRMATION, "question": question,
                                       "count": len(ids)})
        ok, reason, data = self._call(OP_MERGE, ",".join(ids), collage.target, "")
        if not ok:
            return _rejected(reason or OWNER_UNAVAILABLE)
        if not data.get("merged"):  # владелец принял, но склеенных нет: успеха не говорим
            return _rejected("merge_unconfirmed")
        self._memory.forget()  # слитых записей нет: номера этого коллажа больше не про них
        return MCPToolResult(success=True, message=merge_summary(data),
                             data={"status": "done", "id": data["id"], "merged": data["merged"],
                                   "stored_name": data.get("name"), "gallery": data.get("gallery"),
                                   "summary": merge_summary(data), "backup": data.get("backup")})


__all__ = ["COLLAGES", "CollageMemory", "FindSimilarTool", "MergePeopleTool", "NamePersonTool", "PeopleListTool",
           "PersonPhotoTool", "choose_ids", "list_message", "merge_question", "merge_summary", "ros_people_call",
           "similar_message"]
