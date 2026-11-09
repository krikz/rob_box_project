"""Системный промпт Личности: основа ядра и персона продукта (ADR-0167 §2.4, шаг Ш1б-3).

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


@functools.lru_cache(maxsize=1)
def base() -> str:
    """Основа промпта с местами :data:`SLOTS`; места нет или оно не одно - ``ValueError``."""
    text = _data(BASE_FILE).strip()
    wrong = [slot for slot in SLOTS if text.count(slot) != 1]
    if wrong:
        raise ValueError(f"{BASE_FILE}: каждое место персоны {SLOTS} - ровно один раз, а не так: {wrong}")
    return text


@functools.lru_cache(maxsize=1)
def neutral_persona() -> Persona:
    """Персона ядра без профиля: без имени продукта."""
    return parse_persona(_data(NEUTRAL_FILE), NEUTRAL_FILE)


def personality_prompt(persona: Persona) -> str:
    """Системный промпт Личности (без правил знания): основа с персоной на своих местах."""
    return base().replace(SLOTS[0], persona.identity).replace(SLOTS[1], persona.character)


__all__ = [
    "BASE_FILE",
    "NEUTRAL_FILE",
    "Persona",
    "SLOTS",
    "base",
    "neutral_persona",
    "parse_persona",
    "personality_prompt",
]
