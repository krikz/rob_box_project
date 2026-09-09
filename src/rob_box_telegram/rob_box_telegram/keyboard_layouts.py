#!/usr/bin/env python3
"""
keyboard_layouts.py — Telegram inline keyboard layouts for robot control.

Provides pre-built keyboard markups for movement control,
quick actions, and navigation menus.
"""

from telegram import InlineKeyboardButton, InlineKeyboardMarkup

# ─── Movement Control Pad ────────────────────────────────────────────────────

MOVEMENT_KEYBOARD = InlineKeyboardMarkup(
    [
        [
            InlineKeyboardButton("↗️", callback_data="move:forward_left"),
            InlineKeyboardButton("⬆️ Вперёд", callback_data="move:forward"),
            InlineKeyboardButton("↘️", callback_data="move:forward_right"),
        ],
        [
            InlineKeyboardButton("⬅️ Влево", callback_data="move:left"),
            InlineKeyboardButton("⏹ Стоп", callback_data="move:stop"),
            InlineKeyboardButton("➡️ Вправо", callback_data="move:right"),
        ],
        [
            InlineKeyboardButton("↙️", callback_data="move:backward_left"),
            InlineKeyboardButton("⬇️ Назад", callback_data="move:backward"),
            InlineKeyboardButton("↘️", callback_data="move:backward_right"),
        ],
        [
            InlineKeyboardButton("📸 Фото", callback_data="quick:photo"),
            InlineKeyboardButton("📊 Статус", callback_data="quick:status"),
            InlineKeyboardButton("🔇 Стоп навиг.", callback_data="quick:stop_nav"),
        ],
    ]
)

# ─── Main Menu ────────────────────────────────────────────────────────────────

MAIN_MENU_KEYBOARD = InlineKeyboardMarkup(
    [
        [
            InlineKeyboardButton("📸 Фото", callback_data="quick:photo"),
            InlineKeyboardButton("📸 Потолок", callback_data="quick:photo_up"),
        ],
        [
            InlineKeyboardButton("� Глубина", callback_data="quick:photo_depth"),
            InlineKeyboardButton("🗺 Карта", callback_data="quick:photo_map"),
        ],
        [
            InlineKeyboardButton("�📊 Статус", callback_data="quick:status"),
            InlineKeyboardButton("📍 Позиция", callback_data="quick:pose"),
        ],
        [
            InlineKeyboardButton("🗺 Вейпоинты", callback_data="quick:waypoints"),
            InlineKeyboardButton("🎮 Управление", callback_data="quick:control"),
        ],
        [
            InlineKeyboardButton("🔊 Громкость +", callback_data="vol:up"),
            InlineKeyboardButton("🔉 Громкость -", callback_data="vol:down"),
        ],
    ]
)

# ─── Confirm dangerous action ────────────────────────────────────────────────

CONFIRM_KEYBOARD = InlineKeyboardMarkup(
    [
        [
            InlineKeyboardButton("✅ Да", callback_data="confirm:yes"),
            InlineKeyboardButton("❌ Нет", callback_data="confirm:no"),
        ],
    ]
)


# ─── Movement velocity presets ────────────────────────────────────────────────

# callback_data "move:*" → (linear_x, angular_z)
MOVE_VELOCITIES = {
    "forward": (0.2, 0.0),
    "backward": (-0.2, 0.0),
    "left": (0.0, 0.4),
    "right": (0.0, -0.4),
    "forward_left": (0.15, 0.3),
    "forward_right": (0.15, -0.3),
    "backward_left": (-0.15, 0.3),
    "backward_right": (-0.15, -0.3),
    "stop": (0.0, 0.0),
}
