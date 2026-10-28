"""Стартовая фраза робота: перенос v1 ``rob_box_voice/startup_greeting.py`` как есть.

Тексты — ``phrases/ru.yaml`` (``startup.greeting``, 9 вариантов), выбор кодом (ADR-0148); явный текст оператора
(параметр ``startup_greeting_text``) важнее. Тайминги — ``knowledge.STARTUP_GREETING_*``. v1 импортирует отсюда.
Новое поведение («готов к работе» по готовности всех узлов) — отдельная карточка, не здесь.
"""

from __future__ import annotations

import random
from typing import Optional, Tuple

from rob_box_dialog import phrases

#: Варианты стартовой фразы (из каталога фраз; кортеж — как в v1).
GREETINGS: Tuple[str, ...] = tuple(phrases.catalog()["startup.greeting"])


def pick_greeting(override: Optional[str] = None) -> str:
    """Фраза приветствия: непустой ``override`` (без пробелов по краям), иначе — случайная из :data:`GREETINGS`."""
    if override and override.strip():
        return override.strip()
    return random.choice(GREETINGS)


__all__ = ["GREETINGS", "pick_greeting"]
