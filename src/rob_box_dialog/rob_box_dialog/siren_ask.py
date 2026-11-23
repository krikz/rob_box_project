"""Просьба о сирене и мигалке исполняет код, не модель (#5020, ADR-0148).

Живой n903 (21.11 и 22.11, марафон №3): «сирену и мигалку покажи» — модель прислала одну
``play_animation``, ``play_sound`` не вызван. Переписывание ``ambulance`` → ``police_lights``
(#4694) меняет имя сцены и звук не добавляет. Ловить форму ответа модели — гонка; что человек
просил, код знает из его слов и из одной таблицы ``knowledge.SOUND_ALIASES_RU``.

Сирену называет основа ``сирен*`` (триггер каталога ``robot_alert``). «Мигалка» — сцена лица
``police_lights``, не звук: «покажи мигалку» звука не заказывает. Явная скорая помощь
(``скорая``/``амбуланс``) остаётся ``ambulance`` — туда сирену код не подкладывает.

Живой n903 (21.11 run 44916929470 и 22.11 run 44999727503): STT слышит «сирену» как «сериал».
Основа ``сирен*`` такое слово не ловит, ``heard.repair`` чинит только начало команды. Оговорка
целым словом — :data:`knowledge.SOUND_MISHEARS_RU`. Читается она только рядом с «мигалку»: голое «включи сериал» звук не
заказывает, «сериал и мигалку» — заказывает.
Модуль чистый: без ROS и I/O.
"""

from __future__ import annotations

from dataclasses import replace
from types import MappingProxyType
from typing import Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.words import has_word, name_tokens

#: Что код сделал с командой: добавил звук, сцену или оба; пусто — команда как есть.
ADDED_SOUND = "sound"
ADDED_SCENE = "scene"


def misheard_stems(text: str) -> Tuple[str, ...]:
    """Основы из ``SOUND_MISHEARS_RU``, которые назвали целые слова реплики, или пусто.

    Живой промах n903 — «сериал» рядом с «мигалку», не голое «сериал». Поэтому оговорка
    читается только при ``мигалк*`` и не при скорой помощи: «включи сериал» и «звонок и сериал»
    звук сирены не заказывают, названный звонок не подменяется. Сверка — форма целиком
    (``name_tokens``), не начало слова: «сериалы» в таблице, «серия» — нет.
    """
    if not has_word(text, ("мигалк",)) or has_word(text, kn.AMBULANCE_EXCLUDE_STEMS):
        return ()
    heard = set(name_tokens(text))
    return tuple(dict.fromkeys(stem for word, stem in kn.SOUND_MISHEARS_RU if word in heard))


def named_sound(text: str) -> Optional[str]:
    """Триггер каталога, который человек назвал словом из ``SOUND_ALIASES_RU``, или ``None``.

    Первая совпавшая строка таблицы. Несколько триггеров в строке («клик» → четыре ui-звука) —
    первый: код не угадывает, какой из синонимов каталога человек имел в виду.
    Оговорка STT (:func:`misheard_stems`) читается тем же списком основ.
    """
    extra = misheard_stems(text)
    for triggers, stems in kn.SOUND_ALIASES_RU:
        if has_word(text, stems) or any(stem in extra for stem in stems):
            return triggers[0]
    return None


def asks_flasher(text: str) -> bool:
    """Человек просит мигалку (``мигалк*``) и не называет скорую помощь."""
    return has_word(text, ("мигалк",)) and not has_word(text, kn.AMBULANCE_EXCLUDE_STEMS)


def asks_police_lights(text: str) -> bool:
    """Реплика про сирену или мигалку: основа ``POLICE_LIGHTS_STEMS`` либо оговорка STT той же основы.

    «Сериал и мигалку» без этого остаётся ``ambulance``: #4694 смотрит только ``сирен*``/``мигалк*``.
    Скорая помощь (``AMBULANCE_EXCLUDE_STEMS``) не переписывается.
    """
    if has_word(text, kn.AMBULANCE_EXCLUDE_STEMS):
        return False
    return has_word(text, kn.POLICE_LIGHTS_STEMS) or bool(misheard_stems(text))


def _sound_arg(call: ToolCall) -> str:
    return str(call.args.get("sound") or call.args.get("name") or "")


def _plays(call: ToolCall, sound: str) -> bool:
    return call.tool == kn.SOUND_TOOL and _sound_arg(call) == sound


def _shows(call: ToolCall, animation: str) -> bool:
    return call.tool == kn.SHOW_TOOL and call.args.get("animation") == animation


def owed(text: str, actions: Sequence[ToolCall], acted: Sequence[ToolCall]) -> Tuple[Optional[str], bool]:
    """Чего не хватает команде: триггер звука (или ``None``) и нужна ли сцена мигалки.

    Звук уже в команде или исполнен в ходе — не добавляем. Сцена ``police_lights`` уже есть —
    тоже. Скорая помощь звук сирены и сцену мигалки не получает.
    """
    sound = named_sound(text)
    if sound is not None and has_word(text, kn.AMBULANCE_EXCLUDE_STEMS) and sound == "robot_alert":
        sound = None
    calls = (*actions, *acted)
    if sound is not None and any(_plays(call, sound) for call in calls):
        sound = None
    scene = asks_flasher(text) and not any(_shows(call, kn.POLICE_LIGHTS_ANIMATION) for call in calls)
    return sound, scene


def sound_call(sound: str) -> ToolCall:
    """Вызов звука каталога: аргумент ``sound`` — тот, что ждёт схема ``play_sound``."""
    return ToolCall(kn.SOUND_TOOL, MappingProxyType({"sound": sound}))


def scene_call() -> ToolCall:
    """Сцена мигалки: то же имя, на которое #4694 переписывает ``ambulance``."""
    return ToolCall(kn.SHOW_TOOL, MappingProxyType({"animation": kn.POLICE_LIGHTS_ANIMATION}))


def with_asked(text: str, cmd: Command, acted: Sequence[ToolCall] = ()) -> Tuple[Command, str]:
    """Команда, в которой есть названный звук и сцена мигалки, и что добавил код.

    Пустая строка — добавлять нечего (звука не просили, либо он уже в команде, либо модель
    читает/переспрашивает/выступает — её ход). Слова модели не подменяются: звук звучит сам,
    сцена красится итогом хода.
    """
    reading = any(kn.call_class(call.tool, call.args) == "query" for call in cmd.actions)
    if reading or cmd.ask is not None or cmd.perform is not None:
        return cmd, ""
    sound, scene = owed(text, cmd.actions, acted)
    if sound is None and not scene:
        return cmd, ""
    extra = []
    done = []
    if scene:
        extra.append(scene_call())
        done.append(ADDED_SCENE)
    if sound is not None:
        extra.append(sound_call(sound))
        done.append(ADDED_SOUND)
    return replace(cmd, actions=(*cmd.actions, *extra)), "+".join(done)


def salvage_with_asked(text: str, salvage: Sequence[ToolCall], acted: Sequence[ToolCall] = ()) -> Tuple[ToolCall, ...]:
    """Допустимые действия невалидной команды плюс названный звук и сцена мигалки.

    Повтор моста не помог — просьбу всё равно исполняет код. Читать и переспрашивать тут уже
    нечего: ``salvage`` — только действия.
    """
    sound, scene = owed(text, salvage, acted)
    extra = []
    if scene:
        extra.append(scene_call())
    if sound is not None:
        extra.append(sound_call(sound))
    return (*salvage, *extra) if extra else tuple(salvage)
