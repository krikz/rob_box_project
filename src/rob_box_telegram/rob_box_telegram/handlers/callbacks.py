#!/usr/bin/env python3
"""
handlers/callbacks.py — Inline keyboard callback handlers (movement, quick actions, volume).

Processes callback_data from inline buttons defined in keyboard_layouts.py.
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
    """Handle quick action buttons (photo, status, etc.)."""
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

    elif action == "status":
        await query.message.reply_text("⏳ Запрашиваю статус...")
        result = await node.mcp_bridge.execute_simple("get_robot_status")
        await query.message.reply_text(f"📊 {result}")

    elif action == "pose":
        result = await node.mcp_bridge.execute_simple("get_current_pose")
        await query.message.reply_text(f"📍 {result}")

    elif action == "waypoints":
        result = await node.mcp_bridge.execute_simple("list_waypoints")
        await query.message.reply_text(f"📍 Вейпоинты:\n{result}")

    elif action == "control":
        await query.message.reply_text("🎮 Пульт управления:", reply_markup=MOVEMENT_KEYBOARD)

    elif action == "stop_nav":
        result = await node.mcp_bridge.execute_simple("stop_navigation")
        await query.message.reply_text(f"⏹ {result}")


async def _handle_volume(query, context, direction: str) -> None:
    """Handle volume up/down buttons."""
    node = _node(context)

    # Get current volume, adjust by 10
    delta = 10 if direction == "up" else -10
    result = await node.mcp_bridge.execute_simple("set_volume", {"volume_delta": delta})
    await query.message.reply_text(f"🔊 {result}")
