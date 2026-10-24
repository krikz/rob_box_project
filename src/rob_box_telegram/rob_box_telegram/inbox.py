"""Входящие сообщения чата: одна стабильная строка лога на каждое (канал координатора, ADR-0150 PR-12a #3706).

Координатор читает сообщения Шифу «Клод …» из лога ``telegram-bot``
(``docker logs -f telegram-bot | grep 'tg-in'``, ``docs/process/claude-coordinator-robot-flow.md`` §5): формат строки —
контракт, его сверяет тест. «Клод …» решает та же таблица, что адресацию (``rob_box_dialog.address``).
"""

from __future__ import annotations

import logging
from typing import Any

from rob_box_dialog.address import is_coordinator_call

#: Метка строки — по ней grep координатора.
TG_IN_MARK = "📨 [tg-in]"

_LOG = logging.getLogger(__name__)


def tg_in_line(chat_id: int, text: str) -> str:
    """``📨 [tg-in] chat_id=<id> coordinator=<true|false> text=<текст>``; переводы строк — `` ⏎ `` (одна строка)."""
    body = " ⏎ ".join(part.strip() for part in (text or "").strip().splitlines())
    flag = "true" if is_coordinator_call(body) else "false"
    return f"{TG_IN_MARK} chat_id={int(chat_id)} coordinator={flag} text={body}"


def log_incoming(node: Any, chat_id: int, text: str) -> None:
    """Строка в лог ноды (ROS-логгер, его видит ``docker logs``); без ноды — в логгер модуля."""
    line = tg_in_line(chat_id, text)
    get_logger = getattr(node, "get_logger", None)
    if callable(get_logger):
        get_logger().info(line)
    else:
        _LOG.info(line)
