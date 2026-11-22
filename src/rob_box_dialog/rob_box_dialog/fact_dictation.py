"""Диктовку факта сохраняет код, не модель (#5128, ADR-0148).

Живой n206 (марафон №3, 22.11): «робот запомни про меня борис болеет за спартак…» —
``actual_tool_calls: []``, в ``voice_facts`` пусто, а робот сказал «Запомнил». Правило
промпта модель не исполнила; ловить форму её ответа регексом — гонка и ложные срабатывания
(«Запомнил, тебя зовут Саша», «запомнил двоих»). Что человек продиктовал, код знает из его слов.

Код кладёт в команду обычный ``ToolCall`` тула ``knowledge.MEMORY_SAVE_TOOL``. Дальше — тот же
путь, что у вызова модели: исполнитель привяжет спикера снимка (``memory.bind_person``),
MCP позовёт ``VoiceMemory.save_fact``. Второй писатель в ``voice_facts`` не заводится.

Тул уже в команде или исполнен в ходе — код молчит: ноль дополнительных записей.
Модуль чистый: без ROS и I/O.
"""

from __future__ import annotations

from dataclasses import replace
from types import MappingProxyType
from typing import Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.words import has_form, name_tokens, strip_wake_word

#: Ведущее слово диктовки. Прошедшее «запомнил» — ответ, не команда: его нет в таблице.
_LEAD = "запомни"
#: «Запомни меня как Саша» — самопредставление (``self_intro``), не факт.
_INTRO_MARK = "как"
#: «Запомни точку/место» — точка навигации (``save_waypoint``), не факт о человеке.
_PLACE_WORDS = frozenset({"точка", "точку", "точки", "место", "места"})
#: «Запомни, что я сказал» / «запомни это» — мета, «держи в уме»: сохранять нечего.
_META_TAILS = frozenset({"это", "то"})
_META_SAID = frozenset({"я сказал", "что я сказал", "я говорила", "что я говорила"})
#: После этих оборотов хвост — факт дословно. Более длинный оборот проверяется первым.
_PREFIXES = (("про", "меня"), ("что",), ())
#: Вейк, которым STT начинает диктовку. Таблица хода по умолчанию пуста (ADR-0167 §2.4).
_WAKE = ("робот",)


def _saves(call: ToolCall) -> bool:
    return call.tool == kn.MEMORY_SAVE_TOOL


def _tail(tokens: Tuple[str, ...]) -> Optional[str]:
    """Хвост после «запомни», если оборот — диктовка, иначе ``None``.

    «про меня» и «что» снимаются. «меня как» — представление, не факт: проверяется
    до голого хвоста, иначе «как» останется в факте. Голый хвост из «это»/«то»
    или «я сказал» — мета, не факт.
    """
    if not tokens or tokens[0] in _PLACE_WORDS or (len(tokens) >= 2 and tokens[1] == _INTRO_MARK):
        return None
    for prefix in _PREFIXES:
        if tokens[: len(prefix)] != prefix:
            continue
        rest = tokens[len(prefix) :]
        if not rest:
            return None
        said = " ".join(rest)
        if said in _META_TAILS or said in _META_SAID:
            return None
        return said
    return None


def dictated_fact(text: str) -> Optional[str]:
    """Продиктованный факт дословно (нижний регистр, ё → е) или ``None``.

    Ловит «запомни про меня: …», «запомни, что …», «запомни: …» и те же обороты без
    двоеточия: живой n206 пришёл без него. Не ловит вопрос («что ты про меня помнишь»),
    самопредставление («запомни меня как …»), прошедшее («запомнил…») и мету
    («запомни, что я сказал», «запомни это»).
    """
    heard = strip_wake_word(text or "", _WAKE)
    if not has_form(heard, (_LEAD,)):
        return None
    tokens = name_tokens(heard)
    at = tokens.index(_LEAD)
    if at != 0:
        return None
    return _tail(tokens[1:])


def owed(text: str, actions: Sequence[ToolCall], acted: Sequence[ToolCall]) -> Optional[str]:
    """Факт, который код должен сохранить, или ``None``.

    Диктовки нет — нечего. ``memory_save`` уже в команде или исполнен в ходе — ноль
    дополнительных записей, даже если текст факта у модели другой.
    """
    fact = dictated_fact(text)
    if fact is None or any(_saves(call) for call in (*actions, *acted)):
        return None
    return fact


def save_call(fact: str) -> ToolCall:
    """Вызов записи факта: аргументы те, что ждёт схема ``memory_save``.

    Категория — умолчание тула (``general``): код не классифицирует. Спикера подставит
    исполнитель из снимка, не из текста факта.
    """
    return ToolCall(kn.MEMORY_SAVE_TOOL, MappingProxyType({"fact": fact, "category": "general"}))


def with_dictated(text: str, cmd: Command, acted: Sequence[ToolCall] = ()) -> Tuple[Command, str]:
    """Команда с записью продиктованного факта и имя добавленного тула.

    Пустая строка — добавлять нечего: диктовки нет, либо тул уже в ходе. Слова модели
    не подменяются: ``memory_save`` тихий, речь рядом с ним звучит.
    """
    fact = owed(text, cmd.actions, acted)
    if fact is None:
        return cmd, ""
    return replace(cmd, actions=(*cmd.actions, save_call(fact))), kn.MEMORY_SAVE_TOOL


def salvage_with_dictated(
    text: str, salvage: Sequence[ToolCall], acted: Sequence[ToolCall] = ()
) -> Tuple[ToolCall, ...]:
    """Допустимые действия невалидной команды плюс запись продиктованного факта.

    Повтор моста не помог — диктовку всё равно сохраняет код.
    """
    fact = owed(text, salvage, acted)
    return (*salvage, save_call(fact)) if fact is not None else tuple(salvage)
