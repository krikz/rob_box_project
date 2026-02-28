#!/usr/bin/env python3
"""
handlers/commands.py — Telegram command handlers (/start, /photo, /say, /status, etc.)

All handlers receive `context.bot_data["node"]` — our TelegramNode instance
which provides access to ROS 2 publishers, MCP bridge, camera cache, and LLM chat.
"""

import io
import logging

from telegram import Update
from telegram.ext import ContextTypes

from ..auth import authorized
from ..keyboard_layouts import MAIN_MENU_KEYBOARD, MOVEMENT_KEYBOARD

logger = logging.getLogger(__name__)


def _node(context: ContextTypes.DEFAULT_TYPE):
    """Shortcut to get TelegramNode from bot_data."""
    return context.bot_data["node"]


# ─── /start ──────────────────────────────────────────────────────────────────


async def start_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /start — greeting + main menu (no auth required for greeting)."""
    chat_id = update.effective_chat.id
    from ..auth import is_authorized

    if is_authorized(chat_id):
        await update.message.reply_text(
            "🤖 *Rob Box Operator Panel*\n\n"
            "Я — Rob Box, автономный ровер.\n"
            "Используй команды или просто пиши мне сообщения.\n\n"
            "Основные команды:\n"
            "/photo — фото с камеры\n"
            "/say <текст> — произнести текст\n"
            "/status — статус робота\n"
            "/control — пульт управления\n"
            "/menu — быстрое меню\n"
            "/help — все команды\n",
            parse_mode="Markdown",
            reply_markup=MAIN_MENU_KEYBOARD,
        )
    else:
        await update.message.reply_text(
            f"👋 Привет! Ваш chat ID: `{chat_id}`\n\n"
            "Отправьте этот ID администратору для получения доступа.",
            parse_mode="Markdown",
        )


# ─── /myid ───────────────────────────────────────────────────────────────────


async def myid_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /myid — show chat_id (works for everyone)."""
    chat_id = update.effective_chat.id
    await update.message.reply_text(f"🆔 Ваш chat ID: `{chat_id}`", parse_mode="Markdown")


# ─── /help ───────────────────────────────────────────────────────────────────


@authorized
async def help_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /help — list all available commands."""
    await update.message.reply_text(
        "📋 *Доступные команды:*\n\n"
        "*Камера:*\n"
        "/photo — фото с фронтальной камеры\n"
        "/photo\\_up — фото с потолочной камеры\n\n"
        "*Голос:*\n"
        "/say <текст> — произнести текст\n"
        "/playvoice — режим: следующее голосовое проиграть\n\n"
        "*Навигация:*\n"
        "/goto <вейпоинт> — ехать к точке\n"
        "/waypoints — список вейпоинтов\n"
        "/stop — остановить навигацию\n"
        "/pose — текущая позиция\n\n"
        "*Управление:*\n"
        "/control — пульт движения\n"
        "/menu — быстрое меню\n"
        "/volume <0-100> — громкость\n\n"
        "*Эффекты:*\n"
        "/animation <имя> — LED анимация\n"
        "/sound <имя> — звуковой эффект\n\n"
        "*Маппинг:*\n"
        "/map start — начать картографирование\n"
        "/map stop — завершить\n\n"
        "*Музыка:*\n"
        "/music <код> — воспроизвести музыку\n"
        "/music stop — остановить\n\n"
        "*Система:*\n"
        "/status — статус робота\n"
        "/clear — очистить историю чата\n"
        "/myid — показать chat ID\n",
        parse_mode="Markdown",
    )


# ─── /menu ───────────────────────────────────────────────────────────────────


@authorized
async def menu_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /menu — show quick action menu."""
    await update.message.reply_text("📱 Быстрое меню:", reply_markup=MAIN_MENU_KEYBOARD)


# ─── /photo ──────────────────────────────────────────────────────────────────


@authorized
async def photo_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /photo — send latest front camera frame."""
    node = _node(context)
    topic = node.camera_topic

    jpeg_data = node.camera_cache.get(topic)
    if jpeg_data is None:
        age = node.camera_cache.get_age(topic)
        if age is not None:
            await update.message.reply_text(f"⚠️ Кадр устарел ({age:.0f}с). Камера может быть отключена.")
        else:
            await update.message.reply_text("⚠️ Нет кадров с камеры. Проверьте подключение OAK-D.")
        return

    await update.message.reply_photo(
        photo=io.BytesIO(jpeg_data),
        caption="📸 Фронтальная камера",
    )


@authorized
async def photo_up_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /photo_up — send latest ceiling camera frame."""
    node = _node(context)
    topic = node.camera_up_topic

    jpeg_data = node.camera_cache.get(topic)
    if jpeg_data is None:
        await update.message.reply_text("⚠️ Нет кадров с потолочной камеры.")
        return

    await update.message.reply_photo(
        photo=io.BytesIO(jpeg_data),
        caption="📸 Потолочная камера",
    )


# ─── /say ────────────────────────────────────────────────────────────────────


@authorized
async def say_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /say <text> — make the robot speak."""
    text = " ".join(context.args) if context.args else ""
    if not text:
        await update.message.reply_text("Использование: /say <текст для озвучки>")
        return

    node = _node(context)
    node.publish_tts(text)
    await update.message.reply_text(f"🗣 Озвучиваю: _{text}_", parse_mode="Markdown")


# ─── /playvoice ──────────────────────────────────────────────────────────────


@authorized
async def playvoice_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /playvoice — next voice message will be transcribed and spoken by the robot."""
    context.user_data["playvoice_mode"] = True
    await update.message.reply_text(
        "🎤 Режим озвучки активирован.\n"
        "Отправьте голосовое сообщение — робот произнесёт его текст.",
    )


# ─── /status ─────────────────────────────────────────────────────────────────


@authorized
async def status_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /status — show robot telemetry."""
    node = _node(context)
    await update.message.reply_text("⏳ Запрашиваю статус...")

    result = await node.mcp_bridge.execute_simple("get_robot_status")
    await update.message.reply_text(f"📊 *Статус робота:*\n\n{result}", parse_mode="Markdown")


# ─── /waypoints ──────────────────────────────────────────────────────────────


@authorized
async def waypoints_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /waypoints — list saved waypoints."""
    node = _node(context)
    result = await node.mcp_bridge.execute_simple("list_waypoints")
    await update.message.reply_text(f"📍 *Вейпоинты:*\n\n{result}", parse_mode="Markdown")


# ─── /goto ───────────────────────────────────────────────────────────────────


@authorized
async def goto_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /goto <waypoint> — navigate to a waypoint."""
    target = " ".join(context.args) if context.args else ""
    if not target:
        await update.message.reply_text("Использование: /goto <имя вейпоинта>")
        return

    node = _node(context)
    await update.message.reply_text(f"🚗 Еду к: _{target}_...", parse_mode="Markdown")
    result = await node.mcp_bridge.execute_simple("navigate_to_waypoint", {"waypoint": target})
    await update.message.reply_text(result)


# ─── /stop ───────────────────────────────────────────────────────────────────


@authorized
async def stop_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /stop — stop navigation and movement."""
    node = _node(context)
    result = await node.mcp_bridge.execute_simple("stop_navigation")
    await update.message.reply_text(f"⏹ {result}")


# ─── /pose ───────────────────────────────────────────────────────────────────


@authorized
async def pose_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /pose — get current robot position."""
    node = _node(context)
    result = await node.mcp_bridge.execute_simple("get_current_pose")
    await update.message.reply_text(f"📍 {result}")


# ─── /control ────────────────────────────────────────────────────────────────


@authorized
async def control_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /control — show movement control pad."""
    await update.message.reply_text("🎮 Пульт управления:", reply_markup=MOVEMENT_KEYBOARD)


# ─── /volume ─────────────────────────────────────────────────────────────────


@authorized
async def volume_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /volume <0-100> — set robot volume."""
    if not context.args:
        await update.message.reply_text("Использование: /volume <0-100>")
        return
    try:
        level = int(context.args[0])
        if not 0 <= level <= 100:
            raise ValueError
    except ValueError:
        await update.message.reply_text("⚠️ Уровень громкости должен быть от 0 до 100")
        return

    node = _node(context)
    result = await node.mcp_bridge.execute_simple("set_volume", {"volume": level})
    await update.message.reply_text(f"🔊 {result}")


# ─── /animation ──────────────────────────────────────────────────────────────


@authorized
async def animation_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /animation <name> — play LED animation."""
    name = " ".join(context.args) if context.args else ""
    if not name:
        await update.message.reply_text("Использование: /animation <имя анимации>")
        return

    node = _node(context)
    result = await node.mcp_bridge.execute_simple("play_animation", {"animation": name})
    await update.message.reply_text(f"💡 {result}")


# ─── /sound ──────────────────────────────────────────────────────────────────


@authorized
async def sound_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /sound <name> — play sound effect."""
    name = " ".join(context.args) if context.args else ""
    if not name:
        await update.message.reply_text("Использование: /sound <имя звука>")
        return

    node = _node(context)
    result = await node.mcp_bridge.execute_simple("play_sound", {"sound": name})
    await update.message.reply_text(f"🔔 {result}")


# ─── /map ────────────────────────────────────────────────────────────────────


@authorized
async def map_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /map start|stop — control SLAM mapping."""
    action = context.args[0] if context.args else ""
    node = _node(context)

    if action == "start":
        result = await node.mcp_bridge.execute_simple("start_mapping")
        await update.message.reply_text(f"🗺 {result}")
    elif action in ("stop", "finish"):
        result = await node.mcp_bridge.execute_simple("finish_mapping")
        await update.message.reply_text(f"🗺 {result}")
    else:
        await update.message.reply_text("Использование: /map start | /map stop")


# ─── /music ──────────────────────────────────────────────────────────────────


@authorized
async def music_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /music <code> or /music stop."""
    args_text = " ".join(context.args) if context.args else ""
    if not args_text:
        await update.message.reply_text("Использование:\n/music <код>\n/music stop")
        return

    node = _node(context)
    if args_text.strip().lower() == "stop":
        result = await node.mcp_bridge.execute_simple("stop_music")
    else:
        result = await node.mcp_bridge.execute_simple("execute_music_code", {"code": args_text})
    await update.message.reply_text(f"🎵 {result}")


# ─── /clear ──────────────────────────────────────────────────────────────────


@authorized
async def clear_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /clear — clear LLM chat history."""
    node = _node(context)
    chat_id = update.effective_chat.id
    node.llm_chat.clear_session(chat_id)
    await update.message.reply_text("🧹 История чата очищена.")
