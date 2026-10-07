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
from typing import Any, Callable

from rob_box_voice.core.stt_admission import parse_tg_prefix

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


def skip_coordinator_tg(handler: Callable[[Any], None], get_logger: Callable[[], Any]):
    """Обернуть обработчик ``/voice/stt/result``: TG-реплику «Клод …» не пускать дальше.

    Строка ``🎤 STT: [TG:<id>] Клод …`` в логе command_node остаётся (это
    отдельный подписчик). Голосовые реплики и обычные TG-сообщения идут в
    ``handler`` без изменений. Обёртка, а не ветка в ``_on_stt``: класс
    DialogueNode под бюджетом размера (ADR-0145).
    """
    def _wrapped(msg):
        text, tg_chat_id = parse_tg_prefix((getattr(msg, "data", "") or "").strip())
        if tg_chat_id is not None and is_coordinator_address(text):
            get_logger().info("📨 TG-реплика координатору «Клод» — диалог пропускает")
            return
        handler(msg)

    return _wrapped
