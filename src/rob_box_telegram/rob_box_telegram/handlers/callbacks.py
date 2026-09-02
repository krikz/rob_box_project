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
    elif data.startswith("floor:"):
        await _handle_floor(query, context, data)
    elif data == "avatar:refresh":
        await _handle_avatar_refresh(query, context)
    elif data.startswith("confirm:"):
        await query.edit_message_text(f"{'✅ Confirmed' if data == 'confirm:yes' else '❌ Cancelled'}")
    else:
        logger.warning("Unknown callback_data: %s", data)


async def _handle_move(query, context, direction: str) -> None:
    """Publish velocity command for movement button press.

    AV-10 (ADR-0028 §4.4): телеграм больше не публикует ``cmd_vel``
    напрямую. Перед каждой командой движения бот просит у
    ``avatar_supervisor`` floor ``teleop``. Если супервизор
    отказывает (например, Quest уже держит ``teleop_floor``) —
    кнопки гасятся и оператор видит «удерживается другим
    оператором» (UX из ADR-0028 §6 Q3).

    Дополнительно (UX ADR-0028 §6 Q3): если ``/avatar/state``
    сообщает, что ``teleop_floor != "telegram"``, кнопки блокируются
    сразу, без попытки acquire.
    """
    node = _node(context)
    vel = MOVE_VELOCITIES.get(direction, (0.0, 0.0))

    twist = Twist()
    twist.linear.x = float(vel[0])
    twist.angular.z = float(vel[1])

    # Clamp to safety limits
    max_lin = getattr(node, "max_linear_speed", 0.5)
    max_ang = getattr(node, "max_angular_speed", 1.0)
    twist.linear.x = max(-max_lin, min(max_lin, twist.linear.x))
    twist.angular.z = max(-max_ang, min(max_ang, twist.angular.z))

    # AV-10 — UI gate: если по данным супервизора floor сейчас не у
    # нас, даже не пытаемся acquire (экономим service-call и даём
    # мгновенный UX-фидбек).
    current_state = node.supervisor.state
    teleop_floor = current_state.teleop_floor
    if teleop_floor is not None and teleop_floor != "telegram":
        await query.edit_message_text(
            f"🚫 Управление удерживает {teleop_floor}. "
            "Дождитесь, пока оператор в очках отпустит руль, "
            "или используйте текстовые команды.",
            reply_markup=MOVEMENT_KEYBOARD,
        )
        return

    # AV-10 — acquire teleop_floor через супервизор.
    result = node.publish_move_with_floor(twist)
    if not result.granted:
        held = result.held_by or "другим оператором"
        await query.edit_message_text(
            f"🚫 Управление удерживает {held}. "
            "Дождитесь, пока оператор в очках отпустит руль, "
            "или используйте текстовые команды.",
            reply_markup=MOVEMENT_KEYBOARD,
        )
        return

    if direction == "stop":
        await query.edit_message_text("⏹ Остановлен", reply_markup=MOVEMENT_KEYBOARD)
    else:
        # Schedule stop after move_duration (только если floor всё ещё
        # у нас — иначе _publish_stop тоже пройдёт через супервизор).
        move_duration = getattr(node, "move_duration", 0.5)
        asyncio.get_event_loop().call_later(
            move_duration,
            lambda: _publish_stop(node),
        )
        symbol = {"forward": "⬆️", "backward": "⬇️", "left": "⬅️", "right": "➡️"}.get(direction, "🔄")
        await query.edit_message_text(
            f"{symbol} {direction} (lin={vel[0]}, ang={vel[1]})",
            reply_markup=MOVEMENT_KEYBOARD,
        )


def _publish_stop(node) -> None:
    """Publish zero velocity (called after move_duration timeout).

    AV-10: через ``publish_move_with_floor`` — если floor уже не у
    нас, останавливаться не нужно (другой клиент рулит).

    AV-24: результат проверяем — если floor у другого клиента,
    логируем (этот случай уже отфильтрован UI-gate'ом выше, но
    race между gate'ом и таймером возможен).
    """
    twist = Twist()
    result = node.publish_move_with_floor(twist)
    if not result.granted:
        # Это нормальный race: пока таймер ждал, Quest забрал floor.
        # Ничего не делаем — другой клиент уже рулит.
        logger.info(
            "_publish_stop: floor удерживает %s (race с другим клиентом), "
            "stop не публикуем",
            result.held_by or "unknown",
        )


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


# ─── AV-24: floor-кнопки из /avatar ────────────────────────────────────────


async def _handle_floor(query, context, data: str) -> None:
    """Обработать нажатие ``floor:take:teleop|voice`` / ``floor:release:*``.

    callback_data формат: ``floor:{take|release}:{teleop|voice}``.

    На отказ показываем «кто держит» (held_by из ``AcquireResult``).
    На успех — обновляем карточку с новым состоянием.
    """
    from ..avatar_card import build_floor_keyboard, format_avatar_card
    from ..supervisor_client import Floor

    node = _node(context)
    bot_data = context.application.bot_data if hasattr(context, "application") else context.bot_data
    store = bot_data.get("avatar_card_store")

    parts = data.split(":")
    if len(parts) != 3:
        logger.warning("Bad floor callback_data: %s", data)
        return
    op, floor_name = parts[1], parts[2]
    if op not in ("take", "release") or floor_name not in ("teleop", "voice"):
        logger.warning("Unknown floor op: %s", data)
        return

    floor = Floor.TELEOP if floor_name == "teleop" else Floor.VOICE

    if op == "take":
        result = node.supervisor.acquire_floor(floor)
    else:
        node.supervisor.release_floor(floor)
        result = node.supervisor.acquire_floor(floor) if False else _released_result()

    state = node.supervisor.state
    text = format_avatar_card(state, now_s=store.now() if store else 0.0)
    keyboard_rows = build_floor_keyboard(
        state.teleop_floor, state.voice_floor, client_id=node.supervisor.client_id
    )["rows"]
    markup_rows = [
        [
            _button(btn["text"], btn["callback_data"])
            for btn in row
        ]
        for row in keyboard_rows
    ]
    from telegram import InlineKeyboardMarkup
    reply_markup = InlineKeyboardMarkup(markup_rows)

    if op == "take" and not result.granted:
        held = result.held_by or "другим оператором"
        # Сначала отвечаем на callback (иначе крутилка в Telegram).
        await query.answer(f"🚫 Руль/голос удерживает {held}", show_alert=False)
        # Показываем «отказ» в самой карточке: пишем объяснение вверху.
        denied_text = (
            f"🚫 Не удалось взять {floor_name}: удерживает {held}.\n\n" + text
        )
        await _safe_edit(query, denied_text, reply_markup)
        return

    if op == "release":
        await query.answer(f"✅ {floor_name} отдан", show_alert=False)
    else:
        await query.answer(f"✅ {floor_name} взят", show_alert=False)

    await _safe_edit(query, text, reply_markup)

    if store is not None:
        store.register(query.message.chat_id, query.message.message_id, text, state)


async def _handle_avatar_refresh(query, context) -> None:
    """«Обновить» — форсированно перерисовать карточку из текущего state.

    Полезно, когда ``AvatarCardStore`` подавил edit из-за throttling'а или
    одинакового текста.
    """
    from ..avatar_card import build_floor_keyboard, format_avatar_card
    from telegram import InlineKeyboardMarkup

    node = _node(context)
    bot_data = context.application.bot_data if hasattr(context, "application") else context.bot_data
    store = bot_data.get("avatar_card_store")

    state = node.supervisor.state
    text = format_avatar_card(state, now_s=store.now() if store else 0.0)
    keyboard_rows = build_floor_keyboard(
        state.teleop_floor, state.voice_floor, client_id=node.supervisor.client_id
    )["rows"]
    markup_rows = [
        [_button(btn["text"], btn["callback_data"]) for btn in row]
        for row in keyboard_rows
    ]
    await query.answer("🔄 Обновлено")
    await _safe_edit(query, text, InlineKeyboardMarkup(markup_rows))

    if store is not None:
        store.register(query.message.chat_id, query.message.message_id, text, state)


def _button(text: str, callback_data: str):
    """Ленивая обёртка — python-telegram-bot импортируется в handler'е."""
    from telegram import InlineKeyboardButton
    return InlineKeyboardButton(text=text, callback_data=callback_data)


async def _safe_edit(query, text: str, reply_markup) -> None:
    """edit_message_text, игнорирующий ``message is not modified``."""
    try:
        await query.edit_message_text(text=text, reply_markup=reply_markup)
    except Exception as exc:  # noqa: BLE001
        name = type(exc).__name__
        msg = str(exc).lower()
        if name == "BadRequest" and "not modified" in msg:
            return
        logger.warning("Floor button edit failed: %r", exc)


def _released_result() -> "AcquireResult":
    """Фиктивный «успешный» результат для release-ветки (только чтобы
    не дублировать AcquireResult-создание). Release не возвращает
    AcquireResult по API ``SupervisorClient.release_floor``."""
    from ..supervisor_client import AcquireResult
    return AcquireResult(granted=True, contacted_service=False)
