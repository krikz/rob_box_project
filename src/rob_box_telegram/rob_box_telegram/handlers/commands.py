#!/usr/bin/env python3
"""
handlers/commands.py — Telegram command handlers (/start, /photo, /say, /status, etc.)

All handlers receive `context.bot_data["node"]` — our TelegramNode instance
which provides access to ROS 2 publishers, MCP bridge, camera cache, and LLM chat.
"""

import io
import logging
import struct

import numpy as np
from PIL import Image

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
        "/photo\\_up — фото с потолочной камеры\n"
        "/photo\\_depth — карта глубины (OAK-D)\n"
        "/photo\\_map — 2D карта помещения (RTAB-Map)\n\n"
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


# ─── вспомогательные функции ──────────────────────────────────────────────────────────────────────────────


def _depth_compressed_to_jpeg(data: bytes) -> bytes:
    """Convert compressedDepth bytes to a colorized JPEG.

    compressedDepth format (ROS 2 Humble): 12-byte header
    (int32 format + float depthQuantA + float depthQuantB) + PNG (16-bit grayscale).
    Result: colorized JPEG (blue=near, red=far).
    """
    # Find PNG magic bytes to skip the variable-length header robustly
    PNG_SIG = b"\x89PNG\r\n\x1a\n"
    offset = data.find(PNG_SIG)
    if offset == -1:
        raise ValueError(f"No PNG signature in compressedDepth data (len={len(data)})")
    png_bytes = data[offset:]
    img = Image.open(io.BytesIO(png_bytes))
    arr = np.array(img, dtype=np.float32)

    # Normalize valid (non-zero) pixels to 0–255
    valid_mask = arr > 0
    if valid_mask.any():
        lo = float(np.percentile(arr[valid_mask], 2))
        hi = float(np.percentile(arr[valid_mask], 98))
        arr = np.clip(arr, lo, hi)
        arr = (arr - lo) / (hi - lo + 1e-6) * 255.0
    arr8 = arr.astype(np.uint8)

    # Pseudo-colormap: синий (близко) → зелёный → красный (далеко)
    r = np.clip(arr8.astype(np.int16) * 2 - 255, 0, 255).astype(np.uint8)
    g = np.clip(255 - np.abs(arr8.astype(np.int16) * 2 - 255), 0, 255).astype(np.uint8)
    b = np.clip(255 - arr8.astype(np.int16) * 2, 0, 255).astype(np.uint8)
    rgb = np.stack([r, g, b], axis=-1)

    out = Image.fromarray(rgb, mode="RGB")
    buf = io.BytesIO()
    out.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def _occupancy_grid_to_png(grid) -> bytes:
    """Render nav_msgs/OccupancyGrid as a PNG image.

    Color scheme:
        gray  — unknown (-1)
        white — free (0)
        black — occupied (100)
    """
    h = grid.info.height
    w = grid.info.width
    data = np.array(grid.data, dtype=np.int8).reshape(h, w)

    rgb = np.full((h, w, 3), 128, dtype=np.uint8)   # unknown = gray
    rgb[data == 0] = [235, 235, 235]                 # free = light
    rgb[data == 100] = [20, 20, 20]                  # occupied = dark
    # Partial occupancy gradient
    mask = (data > 0) & (data < 100)
    if mask.any():
        vals = data[mask].astype(np.float32)
        col = (235 - vals * 2.1).clip(0, 235).astype(np.uint8)
        rgb[mask] = np.stack([col, col, col], axis=-1)

    # ROS maps are y-up, flip for correct display
    rgb = np.flipud(rgb)

    img = Image.fromarray(rgb, mode="RGB")
    # Scale: min 400px on the short side, max 1200px on the long side
    min_dim, max_dim = 400, 1200
    scale = max(min_dim / max(w, h, 1), 1.0)
    scale = min(scale, max_dim / max(w, h, 1))
    if abs(scale - 1.0) > 0.05:
        img = img.resize((max(1, int(w * scale)), max(1, int(h * scale))), Image.NEAREST)

    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


# ─── /photo_depth ──────────────────────────────────────────────────────────────────────────────


@authorized
async def photo_depth_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /photo_depth — send colorized depth image from OAK-D."""
    node = _node(context)
    topic = node.camera_depth_topic

    raw = node.camera_cache.get(topic)
    if raw is None:
        age = node.camera_cache.get_age(topic)
        if age is not None:
            await update.message.reply_text(f"⚠️ Кадр глубины устарел ({age:.0f}с).")
        else:
            await update.message.reply_text("⚠️ Нет кадров глубины. Проверьте OAK-D.")
        return

    try:
        jpeg = _depth_compressed_to_jpeg(raw)
    except Exception as e:
        logger.exception("Depth decode error")
        await update.message.reply_text(f"⚠️ Ошибка декодирования глубины: {e}")
        return

    await update.message.reply_photo(
        photo=io.BytesIO(jpeg),
        caption="🔵 Глубина (синий=близко, красный=далеко)",
    )


# ─── /photo_map ───────────────────────────────────────────────────────────────────────────────


@authorized
async def photo_map_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /photo_map — send current 2D occupancy map from RTAB-Map."""
    node = _node(context)
    grid = getattr(node, "latest_map_grid", None)
    if grid is None:
        await update.message.reply_text(
            "⚠️ Карта ещё не построена. Подождите пока rtabmap создаст карту."
        )
        return

    try:
        png = _occupancy_grid_to_png(grid)
    except Exception as e:
        logger.exception("Map render error")
        await update.message.reply_text(f"⚠️ Ошибка рендеринга карты: {e}")
        return

    res = grid.info.resolution
    size_m = f"{grid.info.width * res:.1f}×{grid.info.height * res:.1f}"
    await update.message.reply_photo(
        photo=io.BytesIO(png),
        caption=f"🗺 Карта RTAB-Map ({grid.info.width}×{grid.info.height} пикс, {res:.2f}м/пикс, {size_m}м)",
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
