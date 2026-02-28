#!/usr/bin/env python3
"""
handlers/messages.py — Handlers for text messages (LLM chat) and voice messages.

Text messages without "/" are routed to the LLM chat.
Voice messages are transcribed and then processed as text or spoken by the robot.
"""

import logging

from telegram import Update
from telegram.ext import ContextTypes

from ..auth import authorized
from ..voice_processor import transcribe_voice

logger = logging.getLogger(__name__)


def _node(context: ContextTypes.DEFAULT_TYPE):
    """Shortcut to get TelegramNode from bot_data."""
    return context.bot_data["node"]


# ─── Text messages → LLM Chat ───────────────────────────────────────────────


@authorized
async def text_message_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle plain text messages — route to LLM chat with tool support."""
    node = _node(context)
    chat_id = update.effective_chat.id
    user_text = update.message.text.strip()

    if not user_text:
        return

    # Show typing indicator
    await update.message.chat.send_action("typing")

    # Execute LLM chat with MCP tool support
    async def tool_executor(tool_name: str, args: dict) -> str:
        return await node.mcp_bridge.execute_simple(tool_name, args)

    response = await node.llm_chat.chat_with_tools(chat_id, user_text, tool_executor)

    # Send response (split if too long for Telegram's 4096 char limit)
    if len(response) <= 4096:
        await update.message.reply_text(response)
    else:
        for i in range(0, len(response), 4096):
            await update.message.reply_text(response[i : i + 4096])


# ─── Voice messages ──────────────────────────────────────────────────────────


@authorized
async def voice_message_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle voice messages — transcribe and process.

    Two modes:
    1. Normal: transcribe → LLM chat (same as text)
    2. Playvoice: transcribe → robot speaks the text (TTS)
    """
    node = _node(context)
    chat_id = update.effective_chat.id
    voice = update.message.voice

    if not voice:
        return

    await update.message.chat.send_action("typing")

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
        # Mode A: Route to LLM chat
        await update.message.reply_text(f"🎤 Распознано: _{text}_", parse_mode="Markdown")
        await update.message.chat.send_action("typing")

        async def tool_executor(tool_name: str, args: dict) -> str:
            return await node.mcp_bridge.execute_simple(tool_name, args)

        response = await node.llm_chat.chat_with_tools(chat_id, text, tool_executor)
        await update.message.reply_text(f"💬 {response}")
