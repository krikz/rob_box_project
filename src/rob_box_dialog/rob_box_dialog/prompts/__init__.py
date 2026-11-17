"""Системные промпты агентов ядра: основа ядра и персона продукта (ADR-0167 §2.4, шаги Ш1б-3, Ш1б-4).

Основа (:data:`BASE_FILE`, данные пакета) - правила хода, текст речи, формат команды; имени продукта в ней нет. В ней
два места для персоны: ``{identity}`` - первое предложение промпта (кто ты), ``{character}`` - пункты раздела
``# 3. CHARACTER`` (характер, как себя называть, выход из роли). Персону даёт профиль продукта (``persona`` в
``assistant.yaml``, :mod:`rob_box_dialog.profile`); без профиля - нейтральная персона ядра без имени
(:data:`NEUTRAL_FILE`).

Файл персоны - два блока через пустую строку: первый абзац - ``identity``, всё после него - ``character``::

    You are РОББОКС — a friendly autonomous rover companion.

    - Friendly and brief, call yourself «РОББОКС». Emotions — `expression`.

Промпт собирает код (:func:`personality_prompt`): правила модели из ``knowledge.PROMPT_RULES`` дописывает к нему мост
(``dialog_v2_ports.system_prompt``). Данных нет в пакете или блок пуст - ошибка, а не молча пустой промпт.

Терминалы (Ш1б-4) - так же: основа ядра без имени продукта, имя и характер - из профиля.

* Чат Telegram (:data:`TELEGRAM_FILE`): одно место ``{name}`` (дважды) - имя продукта (``name`` профиля),
  :func:`telegram_prompt`. Имени нет - ``ValueError``: «You are 's assistant» хуже, чем честная ошибка.
* Агент-оператор (ТАРС у робота, :data:`OPERATOR_FILE`): места ``{identity}`` (кто ты и откуда оператор управляет) и
  ``{character}`` (заключительный абзац) - персона оператора (``operator_persona`` профиля, тот же формат, что у
  персоны Личности), :func:`operator_rules`. Без профиля - :data:`NEUTRAL_OPERATOR_FILE`. Раздел инструментов к ним
  дописывает хост оператора (``rob_box_supervisor.operator_prompt``).
"""

from __future__ import annotations

import functools
from dataclasses import dataclass
from importlib import resources

#: Основа промпта Личности (данные пакета).
BASE_FILE = "personality.txt"
#: Нейтральная персона ядра - без имени продукта (данные пакета, тот же формат, что у персоны профиля).
NEUTRAL_FILE = "neutral_persona.txt"
#: Места персоны в основе: каждое ровно один раз.
SLOTS = ("{identity}", "{character}")
#: Основа промпта чата Telegram (данные пакета) с местом имени продукта :data:`NAME_SLOT`.
TELEGRAM_FILE = "telegram.txt"
NAME_SLOT = "{name}"
#: Основа правил агента-оператора (данные пакета) с местами :data:`SLOTS`.
OPERATOR_FILE = "operator.txt"
#: Нейтральная персона оператора - без имени продукта и агента.
NEUTRAL_OPERATOR_FILE = "neutral_operator_persona.txt"


@dataclass(frozen=True)
class Persona:
    """Персона продукта: ``identity`` - кто ты (начало промпта), ``character`` - пункты раздела CHARACTER."""

    identity: str
    character: str


def _data(name: str) -> str:
    try:
        return resources.files(__name__).joinpath(name).read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"в пакете {__name__} нет {name}: ядро поставлено без данных (package_data)") from exc


def parse_persona(text: str, source: str = "персона") -> Persona:
    """Персона из текста файла: первый абзац - ``identity``, остальное - ``character``; нет блока - ``ValueError``."""
    head, sep, rest = text.replace("\r\n", "\n").strip().partition("\n\n")
    identity, character = head.strip(), rest.strip()
    if not sep or not identity or not character:
        raise ValueError(f"{source}: ждали два блока через пустую строку - кто ты (identity) и характер (character)")
    return Persona(identity=identity, character=character)


def _slotted(name: str) -> str:
    """Основа ``name`` с местами персоны :data:`SLOTS`; места нет или оно не одно - ``ValueError``."""
    text = _data(name).strip()
    wrong = [slot for slot in SLOTS if text.count(slot) != 1]
    if wrong:
        raise ValueError(f"{name}: каждое место персоны {SLOTS} - ровно один раз, а не так: {wrong}")
    return text


@functools.lru_cache(maxsize=1)
def base() -> str:
    """Основа промпта Личности с местами :data:`SLOTS`; места нет или оно не одно - ``ValueError``."""
    return _slotted(BASE_FILE)


@functools.lru_cache(maxsize=1)
def operator_base() -> str:
    """Основа правил агента-оператора с местами :data:`SLOTS`."""
    return _slotted(OPERATOR_FILE)


@functools.lru_cache(maxsize=1)
def telegram_base() -> str:
    """Основа промпта чата Telegram с местом :data:`NAME_SLOT`; места нет - ``ValueError``."""
    text = _data(TELEGRAM_FILE).strip()
    if NAME_SLOT not in text:
        raise ValueError(f"{TELEGRAM_FILE}: нет места имени продукта {NAME_SLOT}")
    return text


@functools.lru_cache(maxsize=1)
def neutral_persona() -> Persona:
    """Персона ядра без профиля: без имени продукта."""
    return parse_persona(_data(NEUTRAL_FILE), NEUTRAL_FILE)


@functools.lru_cache(maxsize=1)
def neutral_operator_persona() -> Persona:
    """Персона оператора без профиля: без имени продукта и агента."""
    return parse_persona(_data(NEUTRAL_OPERATOR_FILE), NEUTRAL_OPERATOR_FILE)


def _fill(text: str, persona: Persona) -> str:
    return text.replace(SLOTS[0], persona.identity).replace(SLOTS[1], persona.character)


def personality_prompt(persona: Persona) -> str:
    """Системный промпт Личности (без правил знания): основа с персоной на своих местах."""
    return _fill(base(), persona)


def operator_rules(persona: Persona) -> str:
    """Правила агента-оператора (без раздела инструментов хоста): основа с персоной оператора на своих местах."""
    return _fill(operator_base(), persona)


def telegram_prompt(name: str) -> str:
    """Системный промпт чата Telegram (без правил знания): основа с именем продукта; имени нет - ``ValueError``."""
    if not name or not name.strip():
        raise ValueError(f"{TELEGRAM_FILE}: чату Telegram нужно имя продукта (name профиля), а его нет")
    return telegram_base().replace(NAME_SLOT, name.strip())


__all__ = [
    "BASE_FILE",
    "NAME_SLOT",
    "NEUTRAL_FILE",
    "NEUTRAL_OPERATOR_FILE",
    "OPERATOR_FILE",
    "Persona",
    "SLOTS",
    "TELEGRAM_FILE",
    "base",
    "neutral_operator_persona",
    "neutral_persona",
    "operator_base",
    "operator_rules",
    "parse_persona",
    "personality_prompt",
    "telegram_base",
    "telegram_prompt",
]
