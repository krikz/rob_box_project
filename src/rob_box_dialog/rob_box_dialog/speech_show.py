"""Составная просьба «расскажи и покажи анимацию» исполняет код, не модель (#5094, ADR-0148).

Живой n906 (марафон №3 акт 9, run 44999727503): «расскажи анекдот и покажи под него весёлую
анимацию» — модель прислала одну ``play_animation``. ``play_animation`` входит в
``SILENT_DONE_TOOLS`` и ``INAUDIBLE_TOOLS``, и ход закрывался без речи. Отказ одной мигалки
(``Form.needs_speech``, ``VIOLATION_SPEECH_ASKED``) уже на ``develop``: он толкает модель за
словами и не выдумывает анекдот. Составную просьбу он не собирает: если модель дала речь или
выступление и забыла анимацию, сцена не зажигается.

Выбрать одну из двух просьб код не имеет права. Состав — глагол говорения
(``knowledge.SPEECH_ASK_VERBS``) и показ анимации (``SHOW_TOOL`` + «анимац*», либо имя сцены
из словаря). Имя сцены читает ``knowledge.animation_name``; «весёлую» в словаре не было — её
даёт строка той же таблицы алиасов, не догадка по свободному тексту. Голое «покажи
анимацию» анимацию не добавляет (её и так просили одну), голое «расскажи» — тоже. Модуль
чистый: без ROS и I/O.
"""

from __future__ import annotations

from dataclasses import replace
from types import MappingProxyType
from typing import Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.tools_view import speech_asked
from rob_box_dialog.words import has_word, name_tokens

#: Что код сделал с командой: добавил сцену показа; пусто — команда как есть.
ADDED_SHOW = "show"
#: Сцена, когда показ просили, а имя сцены не назвали («покажи анимацию»). Не догадка: строка
#: таблицы, та же, что у «весёлую» — n906 просит весёлую, а голый показ без имени её и берёт.
DEFAULT_SHOW = "happy"


def compound(text: str) -> bool:
    """Реплика — составная просьба: говорить И показать анимацию.

    Говорить — :func:`tools_view.speech_asked` (глагол из ``SPEECH_ASK_VERBS``, «не рассказывай»
    не считается). Показать — «покажи» рядом с «анимац*» или с именем сцены словаря. Одно без
    другого составной просьбой не является.
    """
    return speech_asked(text) and _show_named(text)


def scene_named(text: str) -> Optional[str]:
    """Имя сцены, которое человек назвал в реплике, или ``None``.

    Первое слово, которое ``knowledge.animation_name`` знает: имя из ``KNOWN_ANIMATIONS`` или
    алиас из ``ANIMATION_ALIASES`` («радостную» → ``happy``, «весёлую» → ``happy``). Несколько
    имён — первое: код не выбирает, какую из названных сцен оставить.
    """
    for token in name_tokens(text):
        name = kn.animation_name(token)
        if name is not None:
            return name
    return None


def _show_named(text: str) -> bool:
    """Человек просит показать анимацию: «покажи» и «анимац*», либо «покажи» и имя сцены."""
    if not has_word(text, kn.FACE_SHOW_STEMS):
        return False
    return has_word(text, ("анимац",)) or scene_named(text) is not None


def owed(text: str, actions: Sequence[ToolCall], acted: Sequence[ToolCall]) -> Optional[str]:
    """Какую сцену добавить, или ``None``.

    Составной просьбы нет — ничего. Сцена уже в команде или исполнена в ходе — ничего: код не
    ставит вторую. Имя взято из реплики; не названо — :data:`DEFAULT_SHOW`.
    """
    if not compound(text):
        return None
    calls = (*actions, *acted)
    if any(call.tool == kn.SHOW_TOOL for call in calls):
        return None
    return scene_named(text) or DEFAULT_SHOW


def show_call(animation: str) -> ToolCall:
    """Вызов показа: аргумент ``animation`` — тот, что ждёт схема ``play_animation``."""
    return ToolCall(kn.SHOW_TOOL, MappingProxyType({"animation": animation}))


def with_show(text: str, cmd: Command, acted: Sequence[ToolCall] = ()) -> Tuple[Command, str]:
    """Команда, в которой есть названная сцена, и что добавил код.

    Пустая строка — добавлять нечего: составной просьбы нет, сцена уже есть, модель читает,
    переспрашивает или правит идущее. Выступление (``perform``) — законный носитель речи
    («расскажи анекдотом»), его код не пропускает: в отличие от сирены, речь здесь и есть
    просьба. Слова модели не подменяются.
    """
    reading = any(kn.call_class(call.tool, call.args) == "query" for call in cmd.actions)
    if reading or cmd.ask is not None or cmd.edit_pending:
        return cmd, ""
    animation = owed(text, cmd.actions, acted)
    if animation is None:
        return cmd, ""
    return replace(cmd, actions=(*cmd.actions, show_call(animation))), ADDED_SHOW


def salvage_with_show(text: str, salvage: Sequence[ToolCall], acted: Sequence[ToolCall] = ()) -> Tuple[ToolCall, ...]:
    """Допустимые действия невалидной команды плюс названная сцена.

    Повтор за словами не помог — показ, который просили, всё равно исполняет код. Речь код не
    выдумывает: спасённые действия — только то, что модель уже прислала.
    """
    animation = owed(text, salvage, acted)
    if animation is None:
        return tuple(salvage)
    return (*salvage, show_call(animation))
