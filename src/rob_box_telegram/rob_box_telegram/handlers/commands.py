#!/usr/bin/env python3
"""
handlers/commands.py — Telegram command handlers (/start, /photo, /say, /status, etc.)

After Phase 6 v2 / W7 this module is a *thin transport*:

* Photo / camera handlers (read from the cached frames in TelegramNode) stay
  here because they are direct ROS callbacks — no LLM, no tool bridge.
* Status / navigation / volume / music / mapping handlers used to call the
  ToolProvider through ``_invoke_tool``. They now forward the command
  intent to ``/voice/stt/result`` so the unified AgentCore/harness
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
from ..radio import get_radio_mode, set_radio_mode

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
            f"👋 Привет! Ваш chat ID: `{chat_id}`\n\n" "Отправьте этот ID администратору для получения доступа.",
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
        "/playvoice — режим: следующее голосовое проиграть\n"
        "/radio on|off|status — рация (голос → динамик робота)\n\n"
        "*Навигация:*\n"
        "/goto <вейпоинт> — ехать к точке\n"
        "/waypoints — список вейпоинтов\n"
        "/stop — остановить навигацию\n"
        "/pose — текущая позиция\n\n"
        "*Управление:*\n"
        "/control — пульт движения\n"
        "/menu — быстрое меню\n"
        "/volume <0-100> — громкость\n"
        "/avatar — состояние аватара, кто за рулём/голосом, кнопки floor\n\n"
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


# ─── /avatar ──────────────────────────────────────────────────────────────


@authorized
async def avatar_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /avatar — карточка состояния аватара.

    AV-24 (issue #1916): оператор в Telegram хочет видеть, кто сейчас рулит
    (Quest-оператор в очках или сам Telegram), как долго держится floor, и
    почему получает отказ на команду движения/TTS. Эта команда показывает
    inline-карточку с состоянием и кнопками floor. Дальнейшие обновления
    карточки приходят через ``AvatarCardStore.on_state_update`` — listener
    подписан на ``/avatar/state`` (один на процесс, не на каждый ``/avatar``).
    """
    from ..avatar_card import (
        AvatarCardStore,
        build_floor_keyboard,
        format_avatar_card,
    )

    node = _node(context)
    chat_id = update.effective_chat.id

    # AV-24: ленивая инициализация store + единый state-listener. Каждый
    # вызов /avatar НЕ создаёт новый listener — иначе через 100 нажатий
    # в чате жили бы 100 callback'ов на каждый state-update (это и есть
    # «утячка listener'ов» из acceptance criteria).
    bot_data = context.application.bot_data if hasattr(context, "application") else context.bot_data
    store: AvatarCardStore = bot_data.setdefault(
        "avatar_card_store",
        AvatarCardStore(),
    )

    # Первый store в процессе → подписываемся на /avatar/state один раз.
    if not bot_data.get("avatar_state_listener_installed"):
        _install_state_listener(node, store, bot_data)

    state = node.supervisor.state
    text = format_avatar_card(state, now_s=store.now(), last_seen_s=store.now())
    keyboard_rows = build_floor_keyboard(
        state.teleop_floor,
        state.voice_floor,
        client_id=node.supervisor.client_id,
    )["rows"]
    reply_markup = _build_inline_markup(keyboard_rows)

    sent = await update.message.reply_text(text, reply_markup=reply_markup)
    store.register(chat_id, sent.message_id, text, state)


def _install_state_listener(node, store: "AvatarCardStore", bot_data: dict) -> None:
    """Подписаться на ``/avatar/state`` (один раз на процесс).

    Listener живёт в ``bot_data["avatar_state_listener_unsubscribe"]`` —
    handler'ы могут его дёрнуть (например, при shutdown). Listener-функция
    **сама** не делает Telegram-вызовов: она делегирует это
    ``AvatarCardDispatcher`` (см. ``avatar_card_dispatcher.py``), который
    вызывается из ROS-потока через ``call_soon_threadsafe``.

    AV-24 / ADR-0028 §4.4: ``supervisor.subscribe_state`` уже имеет
    механизм отписки — используем его, а не пишем свой.
    """
    from .avatar_card_dispatcher import make_dispatcher

    loop = bot_data.get("telegram_loop")
    dispatcher = make_dispatcher(loop=loop, store=store, bot_data=bot_data)
    unsubscribe = node.supervisor.subscribe_state(dispatcher)
    bot_data["avatar_state_listener_unsubscribe"] = unsubscribe
    bot_data["avatar_state_listener_installed"] = True
    bot_data["avatar_state_dispatcher"] = dispatcher


def _build_inline_markup(rows: list) -> "InlineKeyboardMarkup":
    """Собрать ``InlineKeyboardMarkup`` из словарного описания.

    ``rows`` — список списков ``{"text": "...", "callback_data": "..."}``.
    Тестируем без ``python-telegram-bot`` (словарный формат), а в handler'е
    разворачиваем в настоящую разметку.
    """
    from telegram import InlineKeyboardButton, InlineKeyboardMarkup

    markup_rows = []
    for row in rows:
        markup_rows.append([InlineKeyboardButton(text=btn["text"], callback_data=btn["callback_data"]) for btn in row])
    return InlineKeyboardMarkup(markup_rows)


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

    rgb = np.full((h, w, 3), 128, dtype=np.uint8)  # unknown = gray
    rgb[data == 0] = [235, 235, 235]  # free = light
    rgb[data == 100] = [20, 20, 20]  # occupied = dark
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
        await update.message.reply_text("⚠️ Карта ещё не построена. Подождите пока rtabmap создаст карту.")
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
    """Handle /say <text> — make the robot speak.

    AV-10 (ADR-0028 §4.4): TTS публикуется только после успешного
    AcquireFloor(voice). Если супервизор отказывает (другой клиент
    держит voice) — пользователь видит уведомление, а не молчаливый
    «произношу».
    """
    text = " ".join(context.args) if context.args else ""
    if not text:
        await update.message.reply_text("Использование: /say <текст для озвучки>")
        return

    node = _node(context)
    # Issue #1195 — echo path: remember the chat so the TTS output of
    # /say is also echoed into the right chat.
    node.set_active_chat(update.effective_chat.id)
    result = node.publish_tts_with_floor(text)
    if not result.granted:
        held = result.held_by or "другим оператором"
        await update.message.reply_text(
            f"🚫 Голос удерживает {held}. " "Дождитесь окончания текущего ответа или используйте текст."
        )
        return
    await update.message.reply_text(f"🗣 Озвучиваю: _{text}_", parse_mode="Markdown")


# ─── /playvoice ──────────────────────────────────────────────────────────


@authorized
async def playvoice_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle /playvoice — robot speaks the transcribed text of the NEXT voice.

    AV-23 / ADR-0021: это **отдельный** режим от ``/radio``. Разница:

    * ``/playvoice`` (one-shot, 1 сообщение) — STT → текст → TTS (робот
      произносит РАСПОЗНАННЫЙ текст голосом TTS). Используется, чтобы
      дать роботу голосовое сообщение «без клавиатуры».
    * ``/radio on|off`` (per-chat, sticky) — голос оператора БЕЗ обработки
      идёт в динамик робота как рация (см. ``docs/plans/2026-08-27-
      quest-voice-passthrough-design.md`` §1.1, режим «рация»).

    Плодить второй механизм для одной задачи запрещено (ADR-0021),
    но это РАЗНЫЕ задачи: playvoice = STT→TTS echo, radio = raw passthrough.
    """
    # Issue #1195 — echo path: remember the chat so the played voice
    # text is echoed into the right chat.
    node = _node(context)
    node.set_active_chat(update.effective_chat.id)
    context.user_data["playvoice_mode"] = True
    await update.message.reply_text(
        "🎤 Режим озвучки активирован.\n"
        "Отправьте голосовое сообщение — робот произнесёт его текст.\n\n"
        "Если хотите, чтобы голос шёл в динамик как рация — "
        "используйте /radio on.",
    )


# ─── /radio ──────────────────────────────────────────────────────────────


@authorized
async def radio_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle ``/radio on|off|status`` — per-chat режим рации (AV-23).

    Семантика:
      * default — ``off`` (как до фичи, никаких побочных эффектов).
      * ``on``  — следующие голосовые из этого чата идут прямо в
        ``/avatar/voice_in`` (рация), STT НЕ запускается.
      * ``off`` — вернуться к STT-режиму.
      * ``status`` — текущее состояние.
    """
    user_data = context.user_data or {}
    if context.user_data is None:
        # python-telegram-bot хранит per-chat state в Context.user_data,
        # но в новой версии (21.x) он может быть None до первого
        # обращения. Нам mutate-нужно, поэтому прокинем dict через
        # ``__setattr__`` (Context — не обычный dataclass).
        try:
            context.user_data = user_data
        except AttributeError:
            pass
    arg = context.args[0].lower() if context.args else "status"
    if arg == "on":
        set_radio_mode(user_data, True)
        await update.message.reply_text(
            "📻 Рация включена.\n"
            "Голосовые сообщения будут звучать в динамике робота "
            "(как рация). Для возврата — /radio off."
        )
    elif arg == "off":
        set_radio_mode(user_data, False)
        await update.message.reply_text("📴 Рация выключена. Голосовые снова идут через STT.")
    elif arg == "status":
        on = get_radio_mode(user_data)
        await update.message.reply_text("📻 Рация: " + ("включена ✅" if on else "выключена ❌"))
    else:
        await update.message.reply_text("Использование: /radio on | off | status")


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

    After W7 the chat history lives in the dialogue pipeline (AgentCore),
    not in the Telegram node. Forward the intent as plain text so the
    dialogue manager can drop the session.
    """
    node = _node(context)
    # Issue #1195 — pass chat_id so /clear carries the [TG:] source marker
    # (command intents must not be dropped by the wake-word gate).
    _forward(node, "/clear", chat_id=update.effective_chat.id)
    await update.message.reply_text("🧹 Запрос на очистку истории отправлен.")


# ─── AV-22 (Issue #1914) — /cmd и /operator ──────────────────────────────


@authorized
async def cmd_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle ``/cmd <text>`` — операторская команда для супервизор-агента.

    ВСЕГДА идёт в ``/avatar/command`` (worker-brief §3.3, ADR-0027 §3.4 +
    ADR-0028), независимо от того, включён ли в чате «режим оператора»
    (``/operator on|off``). Это явная команда — пользователь сознательно
    нажал ``/cmd``, как нажимают ``/cmd`` в CLI.
    """
    node = _node(context)
    chat_id = update.effective_chat.id
    # PTB ``CommandHandler`` срезает ``/cmd`` и кладёт остаток в
    # ``context.args`` (как у /say, /goto, /volume и др. — см. этот же
    # файл). ``update.message.text`` содержит ПОЛНЫЙ текст ``/cmd ...``
    # — для команды он бесполезен. Если шлют ``/cmd@botname <text>`` —
    # ``@botname`` уже отрезан PTB (он не попадает в args).
    text = " ".join(context.args).strip() if context.args else ""
    if not text:
        await update.message.reply_text(
            "ℹ️ Использование: `/cmd <текст команды для агента>`",
            parse_mode="Markdown",
        )
        return
    request_id = node.publish_avatar_command(text=text, chat_id=chat_id)
    if request_id is None:
        await update.message.reply_text(
            "⚠️ Не удалось отправить команду: пустой текст после ``/cmd``."
        )
        return
    await update.message.reply_text(
        f"🎮 Команда отправлена агенту.\n"
        f"`request_id`: `{request_id}`\n"
        f"Ждите ответа в этом чате.",
        parse_mode="Markdown",
    )


@authorized
async def operator_handler(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    """Handle ``/operator on|off`` — per-chat переключатель «режима оператора».

    * ``on`` — каждый СВОБОДНЫЙ текст (без команды) в этом чате идёт
      прямиком в ``/avatar/command`` (а не в личность через ``forward_to_stt``).
      Это «консоль оператора», когда оператор знает, что бот сейчас —
      его инструмент.
    * ``off`` (default) — стандартное поведение: свободный текст идёт в
      личность (как было до AV-22).

    Состояние per-chat: хранится в ``context.user_data["operator_mode"]``
    (PTB user_data изолирован по user_id). При перезапуске бота —
    сбрасывается в ``off``.
    """
    chat_id = update.effective_chat.id
    raw = (update.message.text or "").strip()
    # ``/operator on`` / ``/operator off``
    parts = raw.split(maxsplit=1)
    arg = parts[1].lower() if len(parts) > 1 else ""

    if arg not in ("on", "off", "вкл", "выкл"):
        await update.message.reply_text(
            "ℹ️ Использование: `/operator on` или `/operator off`",
            parse_mode="Markdown",
        )
        return

    on = arg in ("on", "вкл")
    context.user_data["operator_mode"] = on
    label = "✅ Режим оператора **включён** — свободный текст идёт в агента." \
        if on else \
        "🟢 Режим оператора **выключен** — свободный текст идёт в личность (как раньше)."
    await update.message.reply_text(label, parse_mode="Markdown")
    logger.info("AV-22 /operator %s chat_id=%s", arg, chat_id)
