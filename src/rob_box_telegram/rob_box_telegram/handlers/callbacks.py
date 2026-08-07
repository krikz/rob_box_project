#!/usr/bin/env python3
"""
handlers/callbacks.py — Inline keyboard callback handlers (movement, quick actions, volume).

After Phase 6 v2 / W7 the Telegram node is a thin transport: every
callback intent that needs to invoke a tool (status / pose / waypoints /
stop / volume) is forwarded as plain text to ``/voice/stt/result`` so
the unified dialogue pipeline can run it. Movement and camera callback
actions stay here because they talk directly to ROS topics.
"""

import asyncio
import logging

from geometry_msgs.msg import Twist
from telegram import Update
from telegram.ext import ContextTypes

from ..auth import is_authorized
from ..keyboard_layouts import MAIN_MENU_KEYBOARD, MOVE_VELOCITIES, MOVEMENT_KEYBOARD

logger = logging.getLogger(__name__)


def _node(context: ContextTypes.DEFAULT_TYPE):
    """Shortcut to get TelegramNode from bot_data."""
    return context.bot_data["node"]


async def callback_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Route inline keyboard callbacks to appropriate handlers."""
    query = update.callback_query
    await query.answer()

    chat_id = query.message.chat_id
    if not is_authorized(chat_id):
        await query.edit_message_text("⛔ Access denied.")
        return

    data = query.data
    if data.startswith("move:"):
        await _handle_move(query, context, data.split(":", 1)[1])
    elif data.startswith("quick:"):
        await _handle_quick(query, context, data.split(":", 1)[1])
    elif data.startswith("vol:"):
        await _handle_volume(query, context, data.split(":", 1)[1])
    elif data.startswith("confirm:"):
        await query.edit_message_text(f"{'✅ Confirmed' if data == 'confirm:yes' else '❌ Cancelled'}")
    else:
        logger.warning("Unknown callback_data: %s", data)


async def _handle_move(query, context, direction: str) -> None:
    """Publish velocity command for movement button press."""
    node = _node(context)
    vel = MOVE_VELOCITIES.get(direction, (0.0, 0.0))

    twist = Twist()
    twist.linear.x = float(vel[0])
    twist.angular.z = float(vel[1])

    # Clamp to safety limits
    max_lin = node.max_linear_speed
    max_ang = node.max_angular_speed
    twist.linear.x = max(-max_lin, min(max_lin, twist.linear.x))
    twist.angular.z = max(-max_ang, min(max_ang, twist.angular.z))

    node.cmd_vel_pub.publish(twist)

    if direction == "stop":
        await query.edit_message_text("⏹ Остановлен", reply_markup=MOVEMENT_KEYBOARD)
    else:
        # Schedule stop after move_duration
        asyncio.get_event_loop().call_later(
            node.move_duration,
            lambda: _publish_stop(node),
        )
        symbol = {"forward": "⬆️", "backward": "⬇️", "left": "⬅️", "right": "➡️"}.get(direction, "🔄")
        await query.edit_message_text(
            f"{symbol} {direction} (lin={vel[0]}, ang={vel[1]})",
            reply_markup=MOVEMENT_KEYBOARD,
        )


def _publish_stop(node) -> None:
    """Publish zero velocity (called after move_duration timeout)."""
    twist = Twist()
    node.cmd_vel_pub.publish(twist)


async def _handle_quick(query, context, action: str) -> None:
    """Handle quick action buttons (photo, status, etc.).

    Photo / map actions read straight from the cached frames (no tool
    bridge). Status / pose / waypoints / stop_nav forward an intent to
    the unified dialogue pipeline.
    """
    node = _node(context)
    import io

    if action == "photo":
        jpeg_data = node.camera_cache.get(node.camera_topic)
        if jpeg_data:
            await query.message.reply_photo(photo=io.BytesIO(jpeg_data), caption="📸 Фронтальная камера")
        else:
            await query.message.reply_text("⚠️ Нет кадров с камеры")

    elif action == "photo_up":
        jpeg_data = node.camera_cache.get(node.camera_up_topic)
        if jpeg_data:
            await query.message.reply_photo(photo=io.BytesIO(jpeg_data), caption="📸 Потолочная камера")
        else:
            await query.message.reply_text("⚠️ Нет кадров с потолочной камеры")

    elif action == "photo_depth":
        from .commands import _depth_compressed_to_jpeg

        raw = node.camera_cache.get(node.camera_depth_topic)
        if raw:
            try:
                jpeg = _depth_compressed_to_jpeg(raw)
                await query.message.reply_photo(photo=io.BytesIO(jpeg), caption="🔵 Глубина")
            except Exception as e:
                await query.message.reply_text(f"⚠️ Ошибка: {e}")
        else:
            await query.message.reply_text("⚠️ Нет кадров глубины")

    elif action == "photo_map":
        from .commands import _occupancy_grid_to_png

        grid = getattr(node, "latest_map_grid", None)
        if grid:
            try:
                png = _occupancy_grid_to_png(grid)
                res = grid.info.resolution
                size_m = f"{grid.info.width * res:.1f}×{grid.info.height * res:.1f}"
                await query.message.reply_photo(
                    photo=io.BytesIO(png),
                    caption=f"🗺 Карта ({grid.info.width}×{grid.info.height} пикс, {size_m}м)",
                )
            except Exception as e:
                await query.message.reply_text(f"⚠️ Ошибка рендеринга: {e}")
        else:
            await query.message.reply_text("⚠️ Карта ещё не построена")

    elif action == "status":
        node.forward_to_stt("/status")
        await query.message.reply_text("📤 Запрос статуса отправлен...")

    elif action == "pose":
        node.forward_to_stt("/pose")
        await query.message.reply_text("📤 Запрос позиции отправлен...")

    elif action == "waypoints":
        node.forward_to_stt("/waypoints")
        await query.message.reply_text("📤 Запрос списка вейпоинтов отправлен...")

    elif action == "control":
        await query.message.reply_text("🎮 Пульт управления:", reply_markup=MOVEMENT_KEYBOARD)

    elif action == "stop_nav":
        node.forward_to_stt("/stop")
        await query.message.reply_text("⏹ Команда остановки отправлена...")


async def _handle_volume(query, context, direction: str) -> None:
    """Handle volume up/down buttons — forward the intent to the dialogue pipeline."""
    node = _node(context)

    delta = 10 if direction == "up" else -10
    node.forward_to_stt(f"/volume {direction}")
    await query.message.reply_text(f"🔊 Volume {direction} (Δ={delta:+d}) — forwarded")
