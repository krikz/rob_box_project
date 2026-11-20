"""«Пришли их» без id - снимки последней группы лиц зовёт код, не модель (ADR-0148, #4505).

Живой ход 06.11: после ``person_photo(71cda108)`` реплика «ну пришли их» не содержит id. Группа ``faces``
уже открыта в срезе, но модель ответила одним ``say_aloud`` «Сейчас пришлю снимки…» - вслух обещание, снимка нет.

Решает код по словам человека и по id, которые прошлый ход уже получил от владельца (``people_list`` /
``person_photo`` / ``name_person`` / коллаж). Реплика без своего id просит прислать «их» / «их снимки» - код
кладёт ``person_photo`` на каждый id последней группы, не больше ``ACTIONS_ACCEPT_MAX``. Своих id в реплике нет,
а сохранённой группы тоже нет - честная фраза, не обещание. «Отправил» по-прежнему говорит бот после
``send_photo``, не этот модуль.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Mapping, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.words import has_form, name_tokens

#: Местоимение «их» и просьба прислать - целые формы, не основы («них» и «пришлите» сюда не входят).
THEM = ("их",)
SEND = ("пришли", "пришлите", "отправь", "отправьте", "скинь", "скиньте", "вышли", "вышлите", "покажи", "покажите")
#: «Снимки» рядом с «их»: «их снимки», «фото их» - та же просьба, даже без глагола отправки.
PHOTO = ("сним", "фот", "фото", "изображ")
#: Тулы, чей успешный исход называет id группы лиц (владелец, не модель).
GROUP_TOOLS = kn.TOOL_GROUPS[kn.FACES_GROUP].tools
_ID_KEYS = ("id", "photo_id", "person_id")
_ROW_KEYS = ("people", "candidates", "persons")


@dataclass(frozen=True)
class FaceGroup:
    """Последняя группа лиц одного источника: id в порядке появления, без повторов."""

    source: str
    ids: Tuple[str, ...]

    def for_source(self, source: str) -> Tuple[str, ...]:
        """Id группы, если прошлый ход был того же источника; иначе пусто (чужой чат не получает чужие лица)."""
        return self.ids if self.source == source else ()


def asks_for_them(text: str) -> bool:
    """Реплика просит прислать «их» и своего id лица не называет.

    «Ну пришли их», «их снимки», «пришли их снимки». «Пришли 71cda108» - свой id, модель (или срез по id) решает
    сама. «Пришли ссылку» местоимения группы не содержит.
    """
    if kn.FACE_ID_RE.search(text or ""):
        return False
    tokens = name_tokens(text)
    if not has_form(text, THEM):
        return False
    return has_form(text, SEND) or any(word.startswith(PHOTO) for word in tokens)


def photo_calls(ids: Sequence[str]) -> Tuple[ToolCall, ...]:
    """``person_photo`` на каждый id группы, не больше потолка действий хода."""
    unique = tuple(dict.fromkeys(ids))[: kn.ACTIONS_ACCEPT_MAX]
    return tuple(ToolCall(kn.PHOTO_TOOL, {"id": person_id}) for person_id in unique)


def with_their_photos(
    text: str, cmd: Command, ids: Sequence[str], acted: Sequence[ToolCall] = ()
) -> Tuple[Command, str]:
    """Команда, что шлёт снимки «их», и что сделал код.

    ``"sent"`` - добавил ``person_photo`` по сохранённым id, слова модели не звучат (обещание «пришлю» без тула
    иначе уходит в чат или в голос). ``"missing"`` - «их» попросили, а группы нет: честная фраза, тулов нет.
    ``""`` - не про «их», либо модель уже позвала ``person_photo`` (её id не подменяем).
    """
    if not asks_for_them(text) or any(call.tool == kn.PHOTO_TOOL for call in (*cmd.actions, *acted)):
        return cmd, ""
    if not ids:
        said = phrases.text(kn.PHOTO_GROUP_MISSING_PHRASE)
        return replace(cmd, actions=(), speech=said, ask=None), "missing"
    return replace(cmd, actions=photo_calls(ids), speech=None, ask=None), "sent"


def note(group: FaceGroup, source: str, actions: Sequence[ToolCall], results: Sequence[Any]) -> FaceGroup:
    """Группа после хода: id из успешных тулов лиц того же разговора; хода без таких id - группа как была."""
    found = ids_of(actions, results)
    return FaceGroup(source, found) if found else group


def returns_to_model(text: str, last: bool, results: Sequence[Any]) -> bool:
    """``query`` уходит модели за словами, кроме «их»: там отправку уже решил код, второго вызова нет."""
    querying = not last and any(getattr(result, "cls", None) == "query" for result in results)
    return querying and not asks_for_them(text)


def _noted(engine: Any, turn: Any, actions: Sequence[Any], results: Sequence[Any]) -> None:
    """Запомнить id лиц хода для «их» следующего хода того же источника."""
    engine._faces = note(engine._faces, turn.source, actions, results)


def _kept(engine: Any, turn: Any) -> None:
    """Ход лиц не звал, а журнал их ещё помнит: группа не пустеет из-за пустого ``_faces``."""
    if engine._faces.ids:
        return
    remembered = ids_from_entries(engine.log.window())
    if remembered:
        engine._faces = FaceGroup(turn.source, remembered)


def dropped(engine: Any) -> None:
    """Окно сброшено: id лиц прошлого разговора больше не «их» (#4694).

    Журнал ходов уже пуст (``reset_window`` / «новая сессия»). Группа жила рядом с ним и
    переживала сброс: «пришли их» слало вчерашние снимки. Перенос среза (``_carry``) — тот же
    разговор, его тоже нет.
    """
    engine._faces = FaceGroup("", ())
    engine._carry = ("", ())


def ids_of(actions: Sequence[ToolCall], results: Sequence[Any]) -> Tuple[str, ...]:
    """Id лиц из успешных исходов тулов группы, в порядке появления. Провал и чужой тул id не добавляют."""
    found: list = []
    for call, result in zip(actions, results):
        if call.tool not in GROUP_TOOLS or getattr(result, "status", None) != "done":
            continue
        data = getattr(result, "data", None)
        found.extend(_ids(data, call.args.get("id")))
    return tuple(dict.fromkeys(found))


def ids_from_entries(entries: Sequence[Any]) -> Tuple[str, ...]:
    """Id из записей окна (``recent_actions``): исход тула лиц со статусом ``done``."""
    found: list = []
    for entry in entries:
        content = getattr(entry, "content", entry)
        if not isinstance(content, Mapping) or content.get("status") != "done":
            continue
        if content.get("tool") not in GROUP_TOOLS:
            continue
        found.extend(_ids(content.get("data"), None))
    return tuple(dict.fromkeys(found))


def _ids(data: Optional[Mapping[str, Any]], called: Any) -> Tuple[str, ...]:
    """Id из данных владельца; нет структуры списка - id вызова, если он id лица."""
    rows = _rows(data)
    if rows:
        return tuple(row_id for row in rows if (row_id := _row_id(row)))
    own = _one(data)
    if own:
        return (own,)
    text = str(called or "")
    return (text,) if kn.FACE_ID_RE.fullmatch(text) else ()


def _rows(data: Optional[Mapping[str, Any]]) -> Tuple[Mapping[str, Any], ...]:
    if not isinstance(data, Mapping):
        return ()
    for key in _ROW_KEYS:
        rows = data.get(key)
        if isinstance(rows, (list, tuple)) and rows and all(isinstance(row, Mapping) for row in rows):
            return tuple(rows)
    return ()


def _row_id(row: Mapping[str, Any]) -> str:
    for key in _ID_KEYS:
        text = str(row.get(key) or "")
        if kn.FACE_ID_RE.fullmatch(text):
            return text
    return ""


def _one(data: Optional[Mapping[str, Any]]) -> str:
    if not isinstance(data, Mapping):
        return ""
    for key in _ID_KEYS:
        text = str(data.get(key) or "")
        if kn.FACE_ID_RE.fullmatch(text):
            return text
    return ""


__all__ = [
    "FaceGroup",
    "asks_for_them",
    "dropped",
    "ids_from_entries",
    "ids_of",
    "note",
    "photo_calls",
    "with_their_photos",
]
