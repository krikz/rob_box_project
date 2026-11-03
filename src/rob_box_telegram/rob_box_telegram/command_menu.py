"""Меню команд бота (``setMyCommands``): короткие русские описания в одном месте (#4243, решение Шифу 03.11).

Бот регистрирует ``CommandHandler`` на каждую ``*_handler`` из ``handlers/commands.py``; меню в клиенте строится из этой
таблицы. Тест сверяет её с обработчиками: у каждой команды есть строка, лишних строк нет, имена и длины - по правилам
Bot API (имя ``a-z0-9_`` до 32 знаков, описание 1-256).

``in_menu=False`` - рабочие, но не для меню: служебные (``myid``) и операторские (``repl`` исполняет код на роботе,
``cmd``/``operator`` ведут в супервизор-агента). Команда продолжает работать, просто клиент её не подсказывает.
"""

from __future__ import annotations

import re
from types import MappingProxyType
from typing import List, Mapping, NamedTuple, Tuple


class Entry(NamedTuple):
    description: str
    in_menu: bool = True


NAME_RE = re.compile(r"^[a-z0-9_]{1,32}$")
DESCRIPTION_MAX = 256

#: имя команды (без ``/``, без ``_handler``) -> описание
MENU: Mapping[str, Entry] = MappingProxyType({
    "start": Entry("Приветствие и главное меню"),
    "help": Entry("Список всех команд"),
    "menu": Entry("Быстрое меню кнопками"),
    "avatar": Entry("Кто сейчас за рулём и голосом робота"),
    "photo": Entry("Фото с фронтальной камеры"),
    "photo_up": Entry("Фото с потолочной камеры"),
    "photo_depth": Entry("Карта глубины (OAK-D)"),
    "photo_map": Entry("Карта помещения"),
    "say": Entry("Робот произнесёт текст вслух: /say текст"),
    "playvoice": Entry("Следующее голосовое робот проиграет вслух"),
    "radio": Entry("Рация: голос в динамик робота (on, off, status)"),
    "status": Entry("Состояние робота"),
    "waypoints": Entry("Список точек маршрута"),
    "goto": Entry("Ехать к точке: /goto название"),
    "stop": Entry("Остановить движение"),
    "pose": Entry("Где робот сейчас"),
    "control": Entry("Пульт движения"),
    "volume": Entry("Громкость голоса: /volume 0-100"),
    "animation": Entry("Анимация на лице: /animation название"),
    "sound": Entry("Звуковой эффект: /sound название"),
    "map": Entry("Картографирование: /map start или /map stop"),
    "music": Entry("Музыка: /music код или /music stop"),
    "stopmusic": Entry("Остановить музыку"),
    "clear": Entry("Очистить историю разговора"),
    "faces": Entry("Список лиц в базе"),
    "face": Entry("Карточка лица: /face номер или имя"),
    "myid": Entry("Показать chat ID", in_menu=False),
    "repl": Entry("Код Renardo на робота (операторская)", in_menu=False),
    "cmd": Entry("Команда супервизор-агенту (операторская)", in_menu=False),
    "operator": Entry("Режим оператора чата (операторская)", in_menu=False),
})


def menu_commands() -> List[Tuple[str, str]]:
    """Список для ``bot.set_my_commands``: ``(команда, описание)`` в порядке таблицы, только ``in_menu``."""
    return [(name, entry.description) for name, entry in MENU.items() if entry.in_menu]


__all__ = ["DESCRIPTION_MAX", "Entry", "MENU", "NAME_RE", "menu_commands"]
