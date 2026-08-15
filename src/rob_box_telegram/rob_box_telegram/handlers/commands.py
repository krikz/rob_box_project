#!/usr/bin/env python3
"""
handlers/commands.py — Telegram command handlers (/start, /photo, /say, /status, etc.)

After Phase 6 v2 / W7 this module is a *thin transport*:

* Photo / camera handlers (read from the cached frames in TelegramNode) stay
  here because they are direct ROS callbacks — no LLM, no tool bridge.
* Status / navigation / volume / music / mapping handlers used to call the
  ToolProvider through ``_invoke_tool``. They now forward the command
  intent to ``/voice/stt/result`` so the unified DialogCore/harness
  pipeline (``dialogue_node``) can decide what to do with it.

All handlers receive ``context.bot_data["node"]`` — our TelegramNode
instance which exposes ROS 2 publishers and the camera cache.
"""

import io
import logging

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


def _forward(node, text: str, chat_id=None) -> None:
    """Forward a command intent to the unified dialogue pipeline.

    ``chat_id`` is passed through to ``forward_to_stt`` which adds the
    ``[TG:chat_id]`` source marker (issue #1195) — commands are explicit
    intents, so the dialogue_node wake-word gate must not drop them.
    """
    node.forward_to_stt(text, chat_id=chat_id)


# ─── /start ──────────────────────────────────────────────────────────────


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
            "/repl <код> — запустить Renardo/FoxDot код\n"
            "/stopmusic — остановить музыку\n"
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


# ─── /myid ───────────────────────────────────────────────────────────────


async def myid_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /myid — show chat_id (works for everyone)."""
    chat_id = update.effective_chat.id
    await update.message.reply_text(f"🆔 Ваш chat ID: `{chat_id}`", parse_mode="Markdown")


# ─── /help ───────────────────────────────────────────────────────────────


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
        "/repl <код> — отправить Renardo/FoxDot код в робота\n"
        "/stopmusic — остановить музыку\n"
        "/music <код> — совместимый алиас для старого режима\n\n"
        "*Система:*\n"
        "/status — статус робота\n"
        "/clear — очистить историю чата\n"
        "/myid — показать chat ID\n",
        parse_mode="Markdown",
    )


# ─── /menu ───────────────────────────────────────────────────────────────


@authorized
async def menu_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /menu — show quick action menu."""
    await update.message.reply_text("📱 Быстрое меню:", reply_markup=MAIN_MENU_KEYBOARD)


# ─── /photo ──────────────────────────────────────────────────────────────


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


# ─── вспомогательные функции ──────────────────────────────────────────────


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


# ─── /photo_depth ────────────────────────────────────────────────────────


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


# ─── /photo_map ──────────────────────────────────────────────────────────


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


# ─── /say ────────────────────────────────────────────────────────────────


@authorized
async def say_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /say <text> — make the robot speak."""
    text = " ".join(context.args) if context.args else ""
    if not text:
        await update.message.reply_text("Использование: /say <текст для озвучки>")
        return

    node = _node(context)
    # Issue #1195 — echo path: remember the chat so the TTS output of
    # /say is also echoed into the right chat.
    node.set_active_chat(update.effective_chat.id)
    node.publish_tts(text)
    await update.message.reply_text(f"🗣 Озвучиваю: _{text}_", parse_mode="Markdown")


# ─── /playvoice ──────────────────────────────────────────────────────────


@authorized
async def playvoice_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /playvoice — next voice message will be transcribed and spoken by the robot."""
    # Issue #1195 — echo path: remember the chat so the played voice
    # text is echoed into the right chat.
    node = _node(context)
    node.set_active_chat(update.effective_chat.id)
    context.user_data["playvoice_mode"] = True
    await update.message.reply_text(
        "🎤 Режим озвучки активирован.\n"
        "Отправьте голосовое сообщение — робот произнесёт его текст.",
    )


# ─── Tool-bridged commands (W7: forward intents to /voice/stt/result) ──
#
# These handlers used to invoke the ToolProvider (status, waypoints, goto,
# pose, volume, etc.). After W7 the Telegram node is a thin transport, so
# the slash-command text is forwarded to /voice/stt/result and the unified
# dialogue pipeline picks it up. The user gets an immediate ACK so the
# conversation does not feel broken while the harness reacts.


async def _forward_and_ack(update: Update, node, intent: str, ack: str) -> None:
    """Forward ``intent`` to the dialogue pipeline and reply with ``ack``.

    The ACK keeps the operator informed while the harness reacts on the
    other side of ``/voice/stt/result``.
    """
    _forward(node, intent, chat_id=update.effective_chat.id)
    await update.message.reply_text(ack)


# ─── /status ─────────────────────────────────────────────────────────────


@authorized
async def status_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /status — forward the intent to the dialogue pipeline."""
    node = _node(context)
    await _forward_and_ack(update, node, "/status", "📤 Запрос статуса отправлен...")


# ─── /waypoints ──────────────────────────────────────────────────────────


@authorized
async def waypoints_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /waypoints — forward the intent to the dialogue pipeline."""
    node = _node(context)
    await _forward_and_ack(update, node, "/waypoints", "📤 Запрос списка вейпоинтов отправлен...")


# ─── /goto ───────────────────────────────────────────────────────────────


@authorized
async def goto_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /goto <waypoint> — forward the intent to the dialogue pipeline."""
    target = " ".join(context.args) if context.args else ""
    if not target:
        await update.message.reply_text("Использование: /goto <имя вейпоинта>")
        return

    node = _node(context)
    await _forward_and_ack(
        update,
        node,
        f"/goto {target}",
        f"🚗 Еду к: _{target}_... (intent forwarded)",
    )


# ─── /stop ───────────────────────────────────────────────────────────────


@authorized
async def stop_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /stop — forward the intent to the dialogue pipeline."""
    node = _node(context)
    await _forward_and_ack(update, node, "/stop", "⏹ Команда остановки отправлена...")


# ─── /pose ───────────────────────────────────────────────────────────────


@authorized
async def pose_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /pose — forward the intent to the dialogue pipeline."""
    node = _node(context)
    await _forward_and_ack(update, node, "/pose", "📤 Запрос позиции отправлен...")


# ─── /control ────────────────────────────────────────────────────────────


@authorized
async def control_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /control — show movement control pad."""
    await update.message.reply_text("🎮 Пульт управления:", reply_markup=MOVEMENT_KEYBOARD)


# ─── /volume ─────────────────────────────────────────────────────────────


@authorized
async def volume_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /volume <0-100> — forward the intent to the dialogue pipeline."""
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
    await _forward_and_ack(
        update,
        node,
        f"/volume {level}",
        f"🔊 Запрошено: громкость {level}",
    )


# ─── /animation ──────────────────────────────────────────────────────────


@authorized
async def animation_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /animation <name> — forward the intent to the dialogue pipeline."""
    name = " ".join(context.args) if context.args else ""
    if not name:
        await update.message.reply_text("Использование: /animation <имя анимации>")
        return

    node = _node(context)
    await _forward_and_ack(
        update,
        node,
        f"/animation {name}",
        f"💡 Анимация: {name}",
    )


# ─── /sound ──────────────────────────────────────────────────────────────


@authorized
async def sound_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /sound <name> — forward the intent to the dialogue pipeline."""
    name = " ".join(context.args) if context.args else ""
    if not name:
        await update.message.reply_text("Использование: /sound <имя звука>")
        return

    node = _node(context)
    await _forward_and_ack(
        update,
        node,
        f"/sound {name}",
        f"🔔 Звук: {name}",
    )


# ─── /map ────────────────────────────────────────────────────────────────


@authorized
async def map_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /map start|stop — forward the intent to the dialogue pipeline."""
    action = context.args[0] if context.args else ""
    node = _node(context)

    if action == "start":
        await _forward_and_ack(update, node, "/map start", "🗺 Картографирование запускается...")
    elif action in ("stop", "finish"):
        await _forward_and_ack(update, node, "/map stop", "🗺 Картографирование завершается...")
    else:
        await update.message.reply_text("Использование: /map start | /map stop")


# ─── /music ──────────────────────────────────────────────────────────────


@authorized
async def music_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /music <code> or /music stop — forward the intent to the dialogue pipeline."""
    args_text = " ".join(context.args) if context.args else ""
    if not args_text:
        await update.message.reply_text("Использование:\n/music <код>\n/music stop")
        return

    node = _node(context)
    if args_text.strip().lower() == "stop":
        await _forward_and_ack(update, node, "/music stop", "⏹ Останавливаю музыку...")
    else:
        await _forward_and_ack(
            update,
            node,
            f"/music {args_text}",
            "🎵 Запрос на воспроизведение отправлен...",
        )


@authorized
async def repl_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /repl <code> — forward the intent to the dialogue pipeline.

    Uses raw message text to preserve newlines, since context.args splits
    by whitespace and would collapse multiline code into a single line,
    breaking Python comments (#).
    """
    import re

    raw_text = update.message.text or ""
    # Strip the command prefix (/repl or /repl@botname) preserving all newlines
    args_text = re.sub(r"^/repl\S*\s*", "", raw_text, flags=re.IGNORECASE).strip()
    if not args_text:
        await update.message.reply_text("Использование: /repl <Renardo/FoxDot код>")
        return

    node = _node(context)
    await _forward_and_ack(update, node, f"/repl {args_text}", "🎵 Код отправлен...")


@authorized
async def stopmusic_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /stopmusic — forward the intent to the dialogue pipeline."""
    node = _node(context)
    await _forward_and_ack(update, node, "/stopmusic", "⏹ Останавливаю музыку...")


# ─── /clear ──────────────────────────────────────────────────────────────


@authorized
async def clear_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /clear — clear LLM chat history.

    After W7 the chat history lives in the dialogue pipeline (DialogCore),
    not in the Telegram node. Forward the intent as plain text so the
    dialogue manager can drop the session.
    """
    node = _node(context)
    # Issue #1195 — pass chat_id so /clear carries the [TG:] source marker
    # (command intents must not be dropped by the wake-word gate).
    _forward(node, "/clear", chat_id=update.effective_chat.id)
    await update.message.reply_text("🧹 Запрос на очистку истории отправлен.")
