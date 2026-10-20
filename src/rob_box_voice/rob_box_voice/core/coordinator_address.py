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

# Источник реплики из Telegram - значение ``Utterance.SOURCE_TELEGRAM``
# (rob_box_dialog_msgs/msg/Utterance.msg). Модуль остаётся чистым Python без
# импорта сообщений; тест сверяет константу с ``.msg``.
TELEGRAM_SOURCE = "telegram"

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


def stt_log_label(msg: Any) -> str:
    """Текст реплики для лога command_node: ``[TG:<id>] текст`` у Telegram.

    Формат - контракт с координатором: он читает «Клод …» из строки
    ``🎤 STT: [TG:<id>] …`` лога voice-assistant. До ADR-0150 PR-2 префикс
    приходил в самом тексте топика, теперь его строит код из полей
    ``source`` и ``chat_id``.
    """
    text = (getattr(msg, "text", "") or "").strip()
    chat_id = getattr(msg, "chat_id", 0)
    if text and chat_id and getattr(msg, "source", "") == TELEGRAM_SOURCE:
        return f"[TG:{chat_id}] {text}"
    return text


def skip_coordinator_tg(handler: Callable[[Any], None], get_logger: Callable[[], Any]):
    """Обернуть обработчик ``/voice/stt/utterance``: TG-реплику «Клод …» не пускать дальше.

    Строка ``🎤 STT: [TG:<id>] Клод …`` в логе command_node остаётся (это
    отдельный подписчик). Голосовые реплики и обычные TG-сообщения идут в
    ``handler`` без изменений. Обёртка, а не ветка в ``_on_stt``: класс
    DialogueNode под бюджетом размера (ADR-0145).
    """
    def _wrapped(msg):
        if (
            getattr(msg, "source", "") == TELEGRAM_SOURCE
            and getattr(msg, "chat_id", 0)
            and is_coordinator_address((getattr(msg, "text", "") or "").strip())
        ):
            get_logger().info("📨 TG-реплика координатору «Клод» — диалог пропускает")
            return
        handler(msg)

    return _wrapped
