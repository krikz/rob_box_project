"""Обращение к координатору («Клод …») в сообщении из Telegram — Issue #3518.

Товарищ Шифу пишет в Telegram-бот робота сообщения координатору (Claude
Code), начинающиеся с «Клод …». Координатор читает их из лога
voice-assistant (строка ``🎤 STT: [TG:<id>] Клод …`` в command_node). Диалог
робота такие реплики получать не должен: LLM принимала их за команды
(«не попадает в ритм» → ``dj_set stop``).

Решает код (ADR-0148): единственное место знания об обращении — здесь.
"""

from __future__ import annotations

import re

# Обращение — первое слово: «Клод», «клод,», «Claude», «Клод:» и т.п.
# Границы слова не даём ни «Клодия», ни «клод» в середине фразы.
COORDINATOR_NAMES = ("клод", "claude")

_ADDRESS_RE = re.compile(
    r"^\s*(?:" + "|".join(COORDINATOR_NAMES) + r")(?![\w])",
    re.IGNORECASE,
)


def is_coordinator_address(text: str) -> bool:
    """True, если реплика начинается с обращения к координатору."""
    if not isinstance(text, str):
        return False
    return _ADDRESS_RE.match(text) is not None
