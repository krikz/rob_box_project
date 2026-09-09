#!/usr/bin/env python3
"""
auth.py — Telegram bot authorization by chat_id whitelist.

Checks incoming updates against TELEGRAM_ALLOWED_USERS env variable.
Unauthorized users receive their chat_id so they can request access.
"""

import logging
import os
from functools import wraps
from typing import Callable, List, Optional, Set

from telegram import Update
from telegram.ext import ContextTypes

logger = logging.getLogger(__name__)


def get_allowed_users() -> Set[int]:
    """Parse TELEGRAM_ALLOWED_USERS env var (comma-separated chat IDs).

    Returns:
        Set of authorized Telegram chat IDs.
    """
    raw = os.getenv("TELEGRAM_ALLOWED_USERS", "")
    if not raw.strip():
        logger.warning("TELEGRAM_ALLOWED_USERS is empty — bot will reject everyone except /myid")
        return set()

    result: Set[int] = set()
    for token in raw.split(","):
        token = token.strip()
        if token:
            try:
                result.add(int(token))
            except ValueError:
                logger.warning("Invalid chat_id in TELEGRAM_ALLOWED_USERS: %s", token)
    logger.info("Authorized users: %s", result)
    return result


# Module-level cache — populated on first call
_allowed_users: Optional[Set[int]] = None


def _get_cached_allowed_users() -> Set[int]:
    global _allowed_users
    if _allowed_users is None:
        _allowed_users = get_allowed_users()
    return _allowed_users


def reload_allowed_users() -> Set[int]:
    """Force reload of whitelist from env (for runtime reconfiguration)."""
    global _allowed_users
    _allowed_users = get_allowed_users()
    return _allowed_users


def is_authorized(chat_id: int) -> bool:
    """Check if a chat_id is in the whitelist."""
    return chat_id in _get_cached_allowed_users()


def authorized(func: Callable) -> Callable:
    """Decorator for Telegram handlers — rejects unauthorized users.

    Usage::

        @authorized
        async def photo_handler(update: Update, context: ContextTypes.DEFAULT_TYPE):
            ...
    """

    @wraps(func)
    async def wrapper(update: Update, context: ContextTypes.DEFAULT_TYPE, *args, **kwargs):
        chat_id = update.effective_chat.id
        if not is_authorized(chat_id):
            logger.warning("Unauthorized access attempt from chat_id=%d", chat_id)
            await update.message.reply_text(
                f"⛔ Access denied.\n\nYour chat ID: `{chat_id}`\n"
                "Send this ID to the robot owner to request access.",
                parse_mode="Markdown",
            )
            return
        return await func(update, context, *args, **kwargs)

    return wrapper
