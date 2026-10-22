"""Каталог фраз робота (ADR-0150 §5.4): тексты — ``phrases/ru.yaml``, одна таблица на весь проект.

Ключ ``{класс}.{статус}[.{причина}]`` или особый (``understand.timeout``, ``music.*``); 1–8 вариантов со слотами
``{name}``. Нет ключа или слота — ``KeyError`` (громко): фраза об исполнении не может молча стать пустой.
"""

from __future__ import annotations

import functools
import pathlib
import random
from typing import Any, Mapping, Optional, Tuple

import yaml

CATALOG_PATH = pathlib.Path(__file__).with_name("ru.yaml")
MAX_VARIANTS = 8


@functools.lru_cache(maxsize=1)
def catalog() -> Mapping[str, Tuple[str, ...]]:
    """Ключ → варианты текста. Битый каталог — ``ValueError`` при первом обращении (а не тишина в эфире)."""
    data = yaml.safe_load(CATALOG_PATH.read_text(encoding="utf-8")) or {}
    out = {}
    for key, value in data.items():
        variants = (value,) if isinstance(value, str) else tuple(value or ())
        if not 1 <= len(variants) <= MAX_VARIANTS or not all(isinstance(v, str) and v for v in variants):
            raise ValueError(f"phrases/ru.yaml: у {key!r} должно быть 1–{MAX_VARIANTS} непустых строк")
        out[str(key)] = variants
    return out


def text(key: str, variant: int = 0, **slots: Any) -> str:
    """Вариант ``variant`` (по модулю числа вариантов) фразы ``key`` со слотами."""
    variants = catalog()[key]
    return variants[variant % len(variants)].format(**slots)


def pick(key: str, last: Optional[int] = None, rng: Optional[random.Random] = None, **slots: Any) -> Tuple[str, int]:
    """Случайный вариант ``key`` без повтора предыдущего (``last`` — индекс прошлого выбора); ``(текст, индекс)``.
    Выбор кодом (ADR-0148): модель фразу не выбирает. У единственного варианта повтор неизбежен."""
    variants = catalog()[key]
    choices = [i for i in range(len(variants)) if i != last] or [0]
    index = (rng or random).choice(choices)
    return variants[index].format(**slots), index


class Picker:
    """Выбор вариантов кодом с памятью: один и тот же вариант ключа дважды подряд не звучит."""

    def __init__(self, rng: Optional[random.Random] = None) -> None:
        self._rng = rng
        self._last: dict = {}

    def __call__(self, key: str) -> str:
        text, self._last[key] = pick(key, self._last.get(key), self._rng)
        return text


def first_key(*keys: Optional[str]) -> Optional[str]:
    """Первый ключ, который есть в каталоге (``motion.rejected.no_motors`` → ``motion.rejected``), или ``None``."""
    return next((k for k in keys if k and k in catalog()), None)
