#!/usr/bin/env python3
"""
handlers/messages.py — Handlers for text messages and voice messages.

After Phase 6 v2 / W7 this module is a *thin transport*: every text and
voice message is forwarded to ``/voice/stt/result`` so the unified
DialogCore/harness pipeline (in ``dialogue_node``) can decide what to
do with it. There is no LLM call here.

Features:
- 👀 reaction on message receive (processing indicator)
- Message debouncing: split messages are merged before forwarding
"""

import asyncio
import logging

from telegram import Update
from telegram.error import TimedOut, NetworkError
from telegram.ext import ContextTypes

from ..auth import authorized
from ..voice_processor import transcribe_voice

logger = logging.getLogger(__name__)

# Delay (seconds) to wait for additional message parts before processing
_DEBOUNCE_DELAY = 2.0


def _node(context: ContextTypes.DEFAULT_TYPE):
    """Shortcut to get TelegramNode from bot_data."""
    return context.bot_data["node"]


async def _react_eyes(update: Update) -> None:
    """Add 👀 reaction to message (best-effort, ignore failures)."""
    try:
        await update.message.set_reaction("👀")
    except Exception as e:
        logger.debug("set_reaction(👀) not available or failed: %s", e)


# ─── Text messages → forward to /voice/stt/result ─────────────────────────


@authorized
async def text_message_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle plain text messages — forward to the unified dialogue pipeline.

    Implements message debouncing: if user sends multiple messages quickly
    (e.g. Telegram splits a long message), they are merged before
    forwarding. The dialogue_node (downstream) decides what to do with
    the text — chat with LLM, run a tool, or ignore.
    """
    chat_id = update.effective_chat.id
    user_text = update.message.text.strip()

    if not user_text:
        return

    # React with 👀 to indicate we received the message
    await _react_eyes(update)

    # ── Debounce: buffer split messages ──
    # 🔴 FIX (issue #1195): раньше таймер создавался через
    # ``loop.call_later`` и хранился в buf["task"] — это
    # ``asyncio.TimerHandle``, у которого НЕТ ``.done()``. При 2+
    # сообщениях подряд ``buf["task"].done()`` падал с AttributeError
    # (traceback в логах telegram-bot). Теперь используем
    # ``asyncio.create_task`` — Task имеет ``.done()``/``.cancel()``.
    async def _flush_after_delay(chat_id: int, context: ContextTypes.DEFAULT_TYPE) -> None:
        await asyncio.sleep(_DEBOUNCE_DELAY)
        await _flush_buffer(chat_id, context)

    buf = context.user_data.get("msg_buffer")
    if buf is not None:
        # Already buffering — append text and reset timer
        buf["texts"].append(user_text)
        # Cancel previous scheduled task
        if buf.get("task") and not buf["task"].done():
            buf["task"].cancel()
        # Schedule forward after delay
        buf["task"] = asyncio.create_task(_flush_after_delay(chat_id, context))
        logger.debug("Buffered message part %d for chat %d", len(buf["texts"]), chat_id)
        return

    # First message — start buffering
    new_buf = {
        "texts": [user_text],
        "task": None,
    }
    context.user_data["msg_buffer"] = new_buf
    new_buf["task"] = asyncio.create_task(_flush_after_delay(chat_id, context))
    logger.debug("Started message buffer for chat %d", chat_id)


async def _flush_buffer(chat_id: int, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Forward buffered messages to the dialogue pipeline after debounce."""
    buf = context.user_data.pop("msg_buffer", None)
    if not buf or not buf["texts"]:
        return

    node = _node(context)
    combined_text = "\n".join(buf["texts"])

    if len(buf["texts"]) > 1:
        logger.info("Merged %d message parts for chat %d", len(buf["texts"]), chat_id)

    # Forward to the unified dialogue pipeline with the source marker
    # (issue #1195): [TG:chat_id] lets dialogue_node skip the wake-word
    # gate (chat messages are explicit address) and route the LLM reply
    # back into this chat. The chat_id is also recorded as "active chat"
    # so voice-initiated replies echo here too.
    node.forward_to_stt(combined_text, chat_id=chat_id)


# ─── Voice messages ──────────────────────────────────────────────────────


@authorized
async def voice_message_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle voice messages — transcribe and forward.

    Two modes:
    1. Normal: transcribe → forward to /voice/stt/result (same as text)
    2. Playvoice: transcribe → robot speaks the text (TTS)
    """
    node = _node(context)
    voice = update.message.voice
    chat_id = update.effective_chat.id

    if not voice:
        return

    # Issue #1160 — Prometheus metrics: входящее голосовое сообщение.
    from ..observability import is_metrics_enabled, record_telegram_message

    if is_metrics_enabled():
        record_telegram_message("in", message_type="voice")

    # React with 👀 to indicate we received the message
    await _react_eyes(update)

    # Show typing indicator (best-effort — ignore network errors)
    try:
        await update.message.chat.send_action("typing")
    except (TimedOut, NetworkError) as e:
        logger.warning("send_action typing failed (ignored): %s", e)

    # Download voice file
    try:
        voice_file = await context.bot.get_file(voice.file_id)
        ogg_bytes = await voice_file.download_as_bytearray()
    except Exception as e:
        logger.error("Failed to download voice message: %s", e)
        await update.message.reply_text("⚠️ Не удалось загрузить голосовое сообщение.")
        return

    # Transcribe
    text = await transcribe_voice(
        bytes(ogg_bytes),
        method=node.voice_stt_method,
        language=node.voice_stt_language,
    )

    if not text:
        await update.message.reply_text("⚠️ Не удалось распознать голосовое сообщение.")
        return

    # Check if playvoice mode is active
    playvoice_mode = context.user_data.pop("playvoice_mode", False)

    if playvoice_mode:
        # Mode B: Robot speaks the transcribed text
        node.publish_tts(text)
        await update.message.reply_text(
            f"🎤 Распознано: _{text}_\n\n🗣 Робот произносит текст.",
            parse_mode="Markdown",
        )
    else:
        # Mode A: Forward to the unified dialogue pipeline with the
        # source marker (issue #1195) — same as text messages, so the
        # LLM reply is routed back into this chat.
        await update.message.reply_text(f"🎤 Распознано: _{text}_", parse_mode="Markdown")
        node.forward_to_stt(text, chat_id=chat_id)
