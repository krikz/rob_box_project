"""Смену темы играющего диджей-сета строит код из слов человека (#4347, ADR-0148: решает код, LLM говорит).

М10 n604 («Робот, уводи в сторону рока…» под играющим сетом): гард ``music_promise`` отбил первую попытку модели, а
на повторе она снова пообещала («заявку принял: рок будет»), только словами без глагола из таблицы обещаний, и без
``dj_set`` - ответ принят, пустое обещание прозвучало, тема не сменилась. Ловить каждую форму обещания в тексте
модели - бесконечная гонка; что человек просил, код знает сам из ЕГО слов (``words.theme_switch_ask``), а исполнила ли
команда просьбу - из её структуры (есть ли тул ``knowledge.THEME_SWITCH_TOOLS``).

Под играющим сетом (кадр ``DJ``) просьба со стилем, а тула смены в команде нет и в ходе не было - код кладёт
``dj_set start`` с темой словом человека (:func:`with_switch`), слова модели не звучат: о новом сете скажет код по
исходу тула. Стоп сета (``dj_set stop``) сменой не считается и вместо смены не исполняется (живой n604 04.11).
Стиля в словах нет - тему код не угадывает, слова модели заменяет честная фраза
``knowledge.THEME_SWITCH_FAILED_PHRASE``. Модель читает данные (``query``) или переспрашивает (``ask``) - её ход:
команда остаётся. Модуль чистый: без ROS и I/O.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.words import theme_switch_ask

#: Тул, которым код меняет тему сета.
SWITCH_TOOL = "dj_set"


def switch_call(theme: str) -> ToolCall:
    """Вызов смены темы сета: новый сет на тему словом человека (тул сверит её с репликой хода, ``heard_text``)."""
    return ToolCall(SWITCH_TOOL, {"action": "start", "theme": theme})


def _switches(call: ToolCall) -> bool:
    """Вызов исполняет смену темы: тул смены (``knowledge.THEME_SWITCH_TOOLS``), но не стоп музыки
    (``knowledge.stops_music``): ``dj_set stop`` гасит сет, а не меняет тему (#4347, живой n604 04.11: повтор вернул
    один стоп - «Выключил музыку.», рока нет)."""
    return call.tool in kn.THEME_SWITCH_TOOLS and not kn.stops_music(call.tool, call.args)


def _without_stops(actions: Sequence[ToolCall]) -> Tuple[ToolCall, ...]:
    """Действия без стопов музыки: человек просил сменить тему, а не выключить - новый сет сам заменит идущий."""
    return tuple(a for a in actions if not kn.stops_music(a.tool, a.args))


def owed_switch(dj_playing: bool, text: str, actions: Sequence[ToolCall], acted: Sequence[ToolCall]) -> Optional[str]:
    """Тема смены, которую команда не исполнила: стиль словом человека, ``""`` - просьба без стиля, ``None`` - смены
    не ждут (сет не играет, просьбы нет, вызов смены в команде или уже исполнен в ходе). Стоп музыки сменой не
    считается (:func:`_switches`); без просьбы сменить («выключи музыку») стоп остаётся стопом."""
    if not dj_playing or any(_switches(a) for a in (*actions, *acted)):
        return None
    return theme_switch_ask(text)


def with_switch(dj_playing: bool, text: str, cmd: Command, acted: Sequence[ToolCall] = ()) -> Tuple[Command, str]:
    """Команда, что исполняет просьбу сменить тему сета, и что сделал код: ``"built"`` - добавил ``dj_set`` со стилем
    человека, ``"vague"`` - стиля нет, говорит честная фраза, ``""`` - команда как есть (смены не ждут, модель читает
    данные или переспрашивает, выступает)."""
    reading = any(kn.call_class(a.tool, a.args) == "query" for a in cmd.actions)
    if reading or cmd.ask is not None or cmd.perform is not None:
        return cmd, ""
    theme = owed_switch(dj_playing, text, cmd.actions, acted)
    if theme is None:
        return cmd, ""
    kept = _without_stops(cmd.actions)  # стоп вместо смены не исполняется: сет играет, пока не сменится
    if not theme:
        return replace(cmd, actions=kept, speech=phrases.text(kn.THEME_SWITCH_FAILED_PHRASE)), "vague"
    return replace(cmd, actions=(*kept, switch_call(theme)), speech=None), "built"


def salvage_with_switch(dj_playing: bool, text: str, salvage: Sequence[ToolCall],
                        acted: Sequence[ToolCall] = ()) -> Tuple[ToolCall, ...]:
    """Допустимые действия невалидной команды (``Invalid.salvage``) и смена темы, если человек назвал стиль, а тула
    смены нет: обещание модели не прозвучит, а просьбу код исполнит; стоп музыки среди спасённых уступает смене. Стиля
    нет - как было (говорит фраза нарушения)."""
    theme = owed_switch(dj_playing, text, salvage, acted)
    return (*_without_stops(salvage), switch_call(theme)) if theme else tuple(salvage)
