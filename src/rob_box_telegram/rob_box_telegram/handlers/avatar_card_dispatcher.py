#!/usr/bin/env python3
"""avatar_card_dispatcher.py — связь между ``/avatar/state`` и Telegram-карточками.

Контекст (AV-24)
----------------

Когда оператор жмёт ``/avatar``, ``AvatarCardStore`` регистрирует одну
«карточку» в чате. Дальнейшие обновления приходят из супервизора
через ``SupervisorClient.subscribe_state(listener)``.

listener вызывается из **ROS-потока** (тот же, что и ``_on_response`` в
``telegram_node.py`` — issue #1195 учил нас не звать Telegram API прямо
отсюда). Мы пробрасываем работу в asyncio-loop telegram-бота через
``loop.call_soon_threadsafe`` — проверенный паттерн из
``AvatarState`` в ``telegram_node._on_avatar_state``.

Структура
---------

``make_dispatcher`` собирает объект с одним методом ``__call__(state)``:

* listener — вызывается ROS-потоком при каждом ``/avatar/state``;
* внутри — формирует ``CardUpdate`` через ``AvatarCardStore.on_state_update``
  и планирует ``dispatch_actions`` в asyncio-loop;
* ``dispatch_actions`` уже ходит в Telegram API и обновляет ``store``.

Listener можно подменить в тестах — это просто callable(AvatarState)→None.

Зачем отдельный модуль
----------------------

Хотелось держать ``AvatarCardStore`` чистым от Telegram API
(форматирование + throttling + dedup тестируются без сети). Только этот
дисpatcher знает про ``bot.edit_message_text`` / ``bot.send_message`` /
``BadRequest`` / asyncio-loop.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Awaitable, Callable, Dict, Optional

if TYPE_CHECKING:
    import asyncio

    from telegram import Bot
    from telegram.error import BadRequest

    from ..avatar_card import AvatarCardStore, CardUpdate
    from ..supervisor_client import AvatarState

logger = logging.getLogger(__name__)


DispatchFn = Callable[["AvatarState"], None]


def make_dispatcher(
    loop: Optional["asyncio.AbstractEventLoop"],
    store: "AvatarCardStore",
    bot_data: Dict[str, Any],
) -> DispatchFn:
    """Собрать listener для ``SupervisorClient.subscribe_state``.

    Аргументы:
        ``loop``: asyncio-loop telegram-бота (получаем из
            ``bot_data["telegram_loop"]``, который ставит ``TelegramNode._run_telegram``
            при старте loop'а — issue #1195). Если ``None`` — listener
            работает в «тестовом режиме»: только обновляет ``store``,
            Telegram-вызовы не делает.
        ``store``: ``AvatarCardStore`` (один на процесс).
        ``bot_data``: ``context.application.bot_data``; используется
            ``bot_data["telegram_app"]`` для доступа к ``Bot``.

    Возвращает:
        callable, пригодный для передачи в ``SupervisorClient.subscribe_state``.
        Этот callable **синхронный** — он не возвращает coroutine, потому что
        ``SupervisorClient.subscribe_state`` ожидает sync listener.
    """

    def _listener(state: "AvatarState") -> None:
        last_seen_s = store.now()
        actions = store.on_state_update(state, last_seen_s=last_seen_s)
        if not actions:
            return

        if loop is None:
            # Тестовый режим: записей нет, Telegram-вызовы не делаем.
            return

        try:
            loop.call_soon_threadsafe(_dispatch_actions, actions, state, bot_data)
        except RuntimeError as exc:
            # Loop закрыт (бот крахнулся). Не роняем ROS-поток — store
            # останется в согласованном состоянии, и при следующем старте
            # бота оператор увидит «состояние неизвестно» в карточке.
            logger.warning("Avatar card dispatch skipped (loop closed?): %r", exc)

    return _listener


async def _dispatch_actions(
    actions: Dict[int, "CardUpdate"],
    state: "AvatarState",
    bot_data: Dict[str, Any],
) -> None:
    """Выполнить CardUpdate'ы в Telegram asyncio-loop'е.

    Внутри корутины (уже в asyncio-loop, не в ROS-потоке), поэтому
    ``await bot.edit_message_text`` / ``bot.send_message`` — нормально.
    """
    app = bot_data.get("telegram_app")
    if app is None:
        logger.debug("Avatar dispatch: telegram_app not ready, dropping %d actions", len(actions))
        return
    bot: "Bot" = app.bot

    store: "AvatarCardStore" = bot_data.get("avatar_card_store")
    if store is None:
        return

    for chat_id, action in actions.items():
        if action.edit_text is not None:
            await _safe_edit(bot, chat_id, action.message_id, action.edit_text, store, state)
        if action.notify_floor_lost is not None:
            await _safe_notify_floor_lost(bot, chat_id, action.notify_floor_lost, store)


async def _safe_edit(
    bot: "Bot",
    chat_id: int,
    message_id: int,
    text: str,
    store: "AvatarCardStore",
    state: "AvatarState",
) -> None:
    """edit_message_text с обработкой ``400 message is not modified``."""
    try:
        await bot.edit_message_text(chat_id=chat_id, message_id=message_id, text=text)
    except Exception as exc:  # noqa: BLE001
        # ``telegram.error.BadRequest: message is not modified`` — норма.
        # Это значит Telegram уже хранит ровно тот текст, что мы шлём.
        # Не двигаем last_text (он уже совпадает), но last_edit_s тоже
        # не двигаем — throttling остаётся консервативным.
        if _is_message_not_modified(exc):
            store.apply_edit_not_modified(chat_id)
            return
        logger.warning(
            "Avatar card edit failed (chat=%d, msg=%d): %r",
            chat_id,
            message_id,
            exc,
        )
        return
    store.apply_edit(chat_id, text, state)


async def _safe_notify_floor_lost(
    bot: "Bot",
    chat_id: int,
    held_by: str,
    store: "AvatarCardStore",
) -> None:
    """Отправить «Руль забрал …» — ровно один раз на потерю."""
    name = _held_by_display(held_by)
    try:
        await bot.send_message(
            chat_id=chat_id,
            text=f"🛞 **Руль забрал {name}.**\n\n"
            f"Ваши команды движения больше не исполняются. "
            f"Нажмите «Взять руль» в /avatar, когда {name} отпустит.",
        )
    except Exception as exc:  # noqa: BLE001
        logger.warning("Avatar floor-lost notify failed (chat=%d): %r", chat_id, exc)
        return
    store.ack_loss_notified(chat_id, held_by)


def _held_by_display(client_id: str) -> str:
    """client_id держателя → человеко-читаемая строка для уведомления."""
    mapping = {
        "quest": "оператор в очках",
        "admin": "админ-панель",
        "telegram": "Telegram",
    }
    return mapping.get(client_id, client_id)


def _is_message_not_modified(exc: BaseException) -> bool:
    """Telegram вернул ``400 message is not modified``?"""
    name = type(exc).__name__
    if name == "BadRequest":
        msg = str(exc).lower()
        return "message is not modified" in msg or "not modified" in msg
    # Fallback: строковое сопоставление (на случай если python-telegram-bot
    # не импортирован в среде теста).
    return "message is not modified" in str(exc).lower()
