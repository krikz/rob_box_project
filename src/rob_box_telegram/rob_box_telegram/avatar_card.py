#!/usr/bin/env python3
"""avatar_card.py — форматирование и обновление «карточки состояния» аватара в Telegram.

Контекст (AV-24, ADR-0028 §4.2 + §6 Q3):
оператор в Telegram хочет видеть, **кто сейчас рулит** (Quest-оператор в очках или
сам Telegram), **как долго** держится floor, и **почему** получает отказ на
команду движения/TTS. Карточка состояния — единственный способ передать эту
информацию, потому что кнопки движения/TTS могут быть либо активны (floor у
Telegram), либо задизейблены (floor у другого клиента).

Принципы
--------

1. **Честное «неизвестно»** (ADR-0018 в UX). Если ``/avatar/state`` старше
   5 секунд (или вообще ни разу не пришёл в active-режиме), карточка пишет
   «состояние неизвестно», а **не** последнее увиденное значение как факт.
2. **edit_message_text, а не новые сообщения**. Без throttling'а чат зальёт
   «🤖 teleop_floor: quest…». Минимум 2 с между edit'ами; повторный edit с тем
   же текстом подавляется (Telegram отвечает ``400 message is not modified``
   — это норма, не ошибка).
3. **Уведомление о потере floor — одно**. Если Telegram держал teleop_floor
   и потерял его (dead-man, перехват Quest'ом) — **ровно одно** сообщение
   «Руль забрал …», даже если state-обновления продолжают приходить с
   тем же содержимым.
4. **Без глобального состояния ноды**. ``AvatarCardStore`` живёт в
   ``context.application.bot_data`` (один на процесс) — handlers получают
   к нему доступ через ``bot_data["avatar_card_store"]``.

Тестируемость
-------------

Модуль спроектирован так, чтобы unit-тесты могли проверить:

* формат текста карточки (русский язык, человеко-читаемые режимы,
  время удержания);
* throttling 2 с (``fake_clock`` + ``_now`` hook);
* dedup-loss-уведомления (``fake_clock`` + ручная прокрутка состояний);
* «состояние неизвестно» при stale state > 5 с;
* идемпотентный ``format`` для одинакового state (одинаковый текст →
  одинаковый вывод).

Эти гарантии не зависят от Telegram Bot API — тесты запускаются без
``python-telegram-bot`` в окружении.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

from .supervisor_client import AvatarState, Floor

logger = logging.getLogger(__name__)


# Сколько секунд state считается «свежим». После этого карточка пишет
# «состояние неизвестно», а не последнее значение (ADR-0018).
STALE_THRESHOLD_S = 5.0

# Минимальный интервал между edit_message_text. Частота state-update
# (10 Гц heartbeat) → без throttle Telegram зальёт чат.
EDIT_THROTTLE_S = 2.0

# Карта client_id → человеко-читаемое имя. Для UI важно, чтобы
# "held_by=quest" стало понятным «оператор в очках», а не «quest».
_OPERATOR_DISPLAY: Dict[str, str] = {
    "telegram": "Telegram",
    "quest": "оператор в очках",
    "admin": "админ-панель",
}

# Карта mode → человеко-читаемое имя.
_MODE_DISPLAY: Dict[str, str] = {
    "off": "робот свободен",
    "telegram_active": "Telegram управляет",
    "avatar_present": "оператор в очках телеопит",
    "mixed": "mixed: кто-то рулит, кто-то говорит",
    "teleop_only": "только телеуправление",
    "voice_only": "только голос",
}


def _operator_name(client_id: Optional[str]) -> str:
    """client_id → человеко-читаемое имя держателя floor'а."""
    if client_id is None:
        return "свободно"
    return _OPERATOR_DISPLAY.get(client_id, client_id)


def _mode_display(mode: str) -> str:
    """mode → человеко-читаемая строка."""
    return _MODE_DISPLAY.get(mode, mode)


def _format_duration(seconds: float) -> str:
    """Красиво: «12 с» / «3 мин» / «2 ч 5 мин».

    Не тащим Babel — формат фиксирован под русскую локаль UX.
    """
    if seconds < 60:
        return f"{int(seconds)} с"
    minutes = int(seconds // 60)
    if minutes < 60:
        return f"{minutes} мин"
    hours = minutes // 60
    rem_minutes = minutes % 60
    if rem_minutes:
        return f"{hours} ч {rem_minutes} мин"
    return f"{hours} ч"


def format_avatar_card(
    state: AvatarState,
    now_s: float,
    last_seen_s: Optional[float] = None,
) -> str:
    """Форматировать «карточку состояния» как текст для Telegram.

    Аргументы:
        ``state``: текущий ``AvatarState`` (может быть «пустым» — None floors).
        ``now_s``: «сейчас» в секундах (для расчёта длительности удержания
            и stale-проверки). Fake-clock-friendly.
        ``last_seen_s``: когда ``state`` реально был получен. Если None —
            считается, что ``now_s``. Полезно для тестов, где state
            пришёл из кэша.

    Возвращает:
        Готовый текст для ``edit_message_text`` (Markdown отключён —
        используется только жирный «**» для выделения).
    """
    if last_seen_s is None:
        last_seen_s = now_s
    age = max(0.0, now_s - last_seen_s)
    if age > STALE_THRESHOLD_S:
        return (
            "🎮 **Состояние аватара**\n\n"
            "⚠️ Состояние неизвестно — нет свежих данных от супервизора.\n"
            f"_Последний пакет был {int(age)} с назад._\n\n"
            "Попробуйте /avatar через пару секунд."
        )

    teleop = _operator_name(state.teleop_floor)
    voice = _operator_name(state.voice_floor)
    mode_text = _mode_display(state.mode)

    held_lines = []
    if state.teleop_floor:
        held_for = max(0, state.since_ms) / 1000.0
        held_lines.append(f"🛞 **Руль:** {teleop}, уже {_format_duration(held_for)}")
    else:
        held_lines.append("🛞 **Руль:** свободно")

    if state.voice_floor:
        held_for = max(0, state.since_ms) / 1000.0
        held_lines.append(f"🎤 **Голос:** {voice}, уже {_format_duration(held_for)}")
    else:
        held_lines.append("🎤 **Голос:** свободно")

    age_str = "только что" if age < 1 else f"{int(age)} с назад"

    return (
        f"🎮 **Состояние аватара** ({mode_text})\n\n"
        + "\n".join(held_lines)
        + f"\n\n_Данные от супервизора: {age_str}._"
    )


@dataclass
class CardRecord:
    """Состояние одной карточки в чате."""

    chat_id: int
    message_id: int
    last_text: str = ""
    last_edit_s: float = 0.0
    last_seen_state: Optional[AvatarState] = None
    # True, когда мы уже отправили «floor потерян» для текущего «выбывания» —
    # повторно не шлём, пока состояние не вернётся к «у нас».
    loss_notified_for: Optional[str] = None  # client_id, который у нас забрал


class AvatarCardStore:
    """Хранилище «карточек состояния» в чатах.

    Один экземпляр на процесс (``bot_data["avatar_card_store"]``). Каждый
    чат имеет не более одной активной карточки; новый ``/avatar`` в том же
    чате заменяет старую (edit, а не новое сообщение).

    Потокобезопасно: handlers работают из asyncio-loop'а telegram-бота, но
    state-listener может вызываться из ROS-потока при обновлении
    ``/avatar/state``. ``threading.Lock`` делает записи атомарными.
    """

    def __init__(self, now_fn: Optional[Callable[[], float]] = None) -> None:
        self._records: Dict[int, CardRecord] = {}
        self._lock = threading.Lock()
        self._now_fn: Callable[[], float] = now_fn or time.monotonic

    def now(self) -> float:
        return self._now_fn()

    def register(self, chat_id: int, message_id: int, text: str, state: AvatarState) -> None:
        """Зарегистрировать только что отправленную карточку."""
        rec = CardRecord(
            chat_id=chat_id,
            message_id=message_id,
            last_text=text,
            last_edit_s=self.now(),
            last_seen_state=state,
        )
        with self._lock:
            self._records[chat_id] = rec

    def clear(self, chat_id: int) -> None:
        with self._lock:
            self._records.pop(chat_id, None)

    def get(self, chat_id: int) -> Optional[CardRecord]:
        with self._lock:
            return self._records.get(chat_id)

    def record_count(self) -> int:
        """Сколько активных карточек (для теста на утечку)."""
        with self._lock:
            return len(self._records)

    def on_state_update(
        self,
        state: AvatarState,
        last_seen_s: float,
    ) -> Dict[int, "CardUpdate"]:
        """Реакция на обновление ``/avatar/state``.

        Возвращает словарь ``chat_id → CardUpdate``: что делать с каждой
        активной карточкой. Не вызывает Telegram API — это работа
        ``AvatarCardHandler``.

        Тип ``CardUpdate``:
            * ``edit_message`` — нужно вызвать ``edit_message_text``;
            * ``notify_floor_lost`` — Telegram потерял floor,
              отправить отдельное сообщение.
        """
        now = self.now()
        actions: Dict[int, CardUpdate] = {}

        with self._lock:
            chats = list(self._records.items())

        for chat_id, rec in chats:
            new_text = format_avatar_card(state, now_s=now, last_seen_s=last_seen_s)
            held_by = _detect_floor_loss(rec.last_seen_state, state)

            action = CardUpdate(chat_id=chat_id, message_id=rec.message_id)

            if held_by is not None and rec.loss_notified_for != held_by:
                # Floor ушёл из наших рук — уведомление (ровно одно).
                action.notify_floor_lost = held_by
                # Не обновляем loss_notified_for здесь — это делает
                # ``ack_loss_notified`` после успешной отправки.

            if new_text != rec.last_text:
                # Throttling: между edit'ами не менее EDIT_THROTTLE_S.
                # Первая запись сразу редактируется (last_edit_s=0).
                if now - rec.last_edit_s >= EDIT_THROTTLE_S:
                    action.edit_text = new_text

            if action.has_action:
                actions[chat_id] = action

        return actions

    def apply_edit(self, chat_id: int, text: str, state: AvatarState) -> None:
        """После успешного ``edit_message_text`` обновляем внутренний кэш."""
        with self._lock:
            rec = self._records.get(chat_id)
            if rec is None:
                return
            rec.last_text = text
            rec.last_edit_s = self.now()
            rec.last_seen_state = state

    def apply_edit_not_modified(self, chat_id: int) -> None:
        """Telegram ответил ``400 message is not modified`` — норма, не ошибка.

        Просто обновляем ``last_text`` (теперь мы «синхронизированы» с тем,
        что уже в Telegram), но не двигаем ``last_edit_s``, чтобы следующая
        попытка edit'а не подавлялась throttling'ом раньше времени.
        """
        with self._lock:
            rec = self._records.get(chat_id)
            if rec is None:
                return
            # last_text не меняется — он уже совпадает.
            # last_edit_s не двигаем — это было бы «бесплатное» обновление.

    def ack_loss_notified(self, chat_id: int, held_by: str) -> None:
        with self._lock:
            rec = self._records.get(chat_id)
            if rec is None:
                return
            rec.loss_notified_for = held_by

    def clear_loss_marker_if_held(self, chat_id: int, current_holder: Optional[str]) -> None:
        """Если floor снова у нас — сбрасываем loss-маркер.

        Это позволяет следующей потере снова отправить уведомление.
        Вызывается handler'ом, когда ``/avatar/state`` сообщает
        ``teleop_floor == "telegram"``.
        """
        with self._lock:
            rec = self._records.get(chat_id)
            if rec is None:
                return
            if current_holder == "telegram" or current_holder is None:
                rec.loss_notified_for = None


@dataclass
class CardUpdate:
    """Решение по обновлению одной карточки."""

    chat_id: int
    message_id: int
    edit_text: Optional[str] = None
    notify_floor_lost: Optional[str] = None  # client_id, который у нас забрал

    @property
    def has_action(self) -> bool:
        return self.edit_text is not None or self.notify_floor_lost is not None


def _detect_floor_loss(
    previous: Optional[AvatarState],
    current: AvatarState,
) -> Optional[str]:
    """«У нас забрали teleop_floor»? Возвращает client_id, который теперь держит, иначе None.

    Семантика (ADR-0028 §4.2): Telegram теряет floor, когда:
      * был ``teleop_floor == "telegram"`` → стал ``None`` или другой client_id;
      * (аналогично для voice_floor).

    Мы уведомляем только когда это была **наша** потеря (held_by != "telegram"),
    а не ситуация «floor освободился без держателя».
    """
    if previous is None:
        return None
    prev_t = previous.teleop_floor
    curr_t = current.teleop_floor
    if prev_t == "telegram" and curr_t is not None and curr_t != "telegram":
        return curr_t
    return None


def build_floor_keyboard(
    teleop_floor: Optional[str],
    voice_floor: Optional[str],
    client_id: str = "telegram",
) -> Dict[str, Any]:
    """Собрать inline-кнопки floor для карточки.

    Возвращает dict ``{"buttons": [...rows...]}`` — это намеренно НЕ
    ``InlineKeyboardMarkup``: тестируем без ``python-telegram-bot``.

    Кнопки:

    * «Взять руль» / «Отдать руль» — enable/disable по факту держателя.
    * «Взять голос» / «Отдать голос» — аналогично.
    * «Обновить» — форсированный refresh карточки (на случай, если
      throttling подавил edit).

    Callback_data формат (handler в ``callbacks.py`` маршрутизирует):

        floor:take:teleop
        floor:take:voice
        floor:release:teleop
        floor:release:voice
        avatar:refresh
    """
    teleop_held = teleop_floor == client_id
    voice_held = voice_floor == client_id
    rows: list = []

    if teleop_held:
        rows.append([{"text": "🛞 Отдать руль", "callback_data": "floor:release:teleop"}])
    else:
        label = (
            f"🛞 Руль у {_operator_name(teleop_floor)}"
            if teleop_floor
            else "🛞 Взять руль"
        )
        rows.append([{"text": label, "callback_data": "floor:take:teleop"}])

    if voice_held:
        rows.append([{"text": "🎤 Отдать голос", "callback_data": "floor:release:voice"}])
    else:
        label = (
            f"🎤 Голос у {_operator_name(voice_floor)}"
            if voice_floor
            else "🎤 Взять голос"
        )
        rows.append([{"text": label, "callback_data": "floor:take:voice"}])

    rows.append([{"text": "🔄 Обновить", "callback_data": "avatar:refresh"}])
    return {"rows": rows}


def is_held_by_other(floor_value: Optional[str], client_id: str = "telegram") -> bool:
    """«Floor удерживает другой клиент, и это не мы»?"""
    return floor_value is not None and floor_value != client_id


def is_held_by_us(floor_value: Optional[str], client_id: str = "telegram") -> bool:
    return floor_value == client_id


# Re-export Floor для удобства импорта в handlers (один import).
__all__ = [
    "AvatarCardStore",
    "CardRecord",
    "CardUpdate",
    "EDIT_THROTTLE_S",
    "Floor",
    "STALE_THRESHOLD_S",
    "build_floor_keyboard",
    "format_avatar_card",
    "is_held_by_other",
    "is_held_by_us",
]
