"""Отправка ответов в чат Telegram с показом процесса (ADR-0160 §2.4, #4243).

Пока идёт ход, человек видит, что робот работает, а не тишину:

1. сразу - действие «печатает» (``sendChatAction typing``; Telegram гасит его через 5 с, поэтому повтор каждые 4 с);
2. если ход длится дольше ``PROGRESS_AFTER_S`` и код хода прислал подпись шага («Думаю…», «Делаю…») - ОДИН индикатор,
   который правится, а не поток новых сообщений: в личном чате это черновик ``sendMessageDraft`` (Bot API 9.3+,
   доступен всем ботам с 9.5; временный, исчезает сам, когда уходит настоящее сообщение), в группе или если
   черновик отвергнут - одно сообщение, которое правится ``editMessageText`` и удаляется перед ответом;
3. ответ - сообщения :func:`chat_render.render`; если Telegram отверг разметку, та же реплика уходит голым текстом.

Транспорт - любой объект с методами ``python-telegram-bot`` (``send_message``, ``edit_message_text``,
``delete_message``, ``send_chat_action``, ``do_api_request``): в тестах это заглушка, часы и сон тоже подменяются.
Ход ограничен ``TURN_MAX_S``: если ответа нет, индикатор гаснет сам (честнее вечного «печатает»).
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Dict, Optional

from . import chat_render

_LOG = logging.getLogger(__name__)

#: Действие Bot API: «печатает».
ACTION_TYPING = "typing"
#: Повтор «печатает»: клиент гасит статус через 5 с.
TYPING_EVERY_S = 4.0
#: Сколько ход идёт молча, прежде чем показать подпись шага (быстрый ответ индикатора не получает).
PROGRESS_AFTER_S = 3.0
#: Как часто обновлять индикатор (счётчик секунд); правка одного сообщения чаще раза в 3 с - риск лимита Bot API.
PROGRESS_EVERY_S = 3.0
#: Потолок хода: дольше индикатор не держим.
TURN_MAX_S = 120.0
#: Шаг цикла индикатора.
TICK_S = 1.0
#: Идентификатор черновика: один и тот же - Telegram анимирует изменения вместо замены.
DRAFT_ID = 1

#: ``kind`` исходящего элемента очереди сверх ``DialogOutput.kind``: «ход начался» (внутренний, не с провода).
KIND_BEGIN = "begin"

Sleep = Callable[[float], Awaitable[Any]]


@dataclass
class _Turn:
    started: float
    label: str = ""
    msg_id: Optional[int] = None
    shown: str = ""
    last_typing: float = -1e9
    last_progress: float = -1e9
    draft_ok: bool = True
    done: bool = False
    task: Optional["asyncio.Task[None]"] = None


def _is_private(chat_id: int) -> bool:
    """Личный чат - положительный id; у групп и каналов он отрицательный. Черновик Bot API - только для личного."""
    return chat_id > 0


def _is_markup_error(exc: Exception) -> bool:
    return "parse entities" in str(exc).lower() or "can't parse" in str(exc).lower()


class ChatSender:
    """Ответы и ход процесса по чатам; один ход на чат."""

    def __init__(self, bot: Any, clock: Callable[[], float] = time.monotonic, sleep: Sleep = asyncio.sleep,
                 log: logging.Logger = _LOG) -> None:
        self.bot = bot
        self._clock = clock
        self._sleep = sleep
        self._log = log
        self._turns: Dict[int, _Turn] = {}

    def begin(self, chat_id: int) -> None:
        """Ход начался (сообщение человека ушло в диалог): «печатает» сразу, подпись шага - позже."""
        turn = self._turns.get(chat_id)
        if turn is not None and not turn.done:
            turn.started = self._clock()  # новое сообщение подряд - срок хода считаем заново
            return
        turn = _Turn(started=self._clock())
        self._turns[chat_id] = turn
        turn.task = asyncio.ensure_future(self._pulse(chat_id, turn))

    def set_progress(self, chat_id: int, label: str) -> None:
        if chat_id not in self._turns or self._turns[chat_id].done:
            self.begin(chat_id)
        self._turns[chat_id].label = label

    async def deliver(self, chat_id: int, text: str, role: str = "") -> None:
        """Ответ в чат: индикатор хода снят, реплика оформлена."""
        await self._end(chat_id)
        for item in chat_render.render(text, role):
            await self._send(chat_id, item)

    async def close(self) -> None:
        for chat_id in list(self._turns):
            await self._end(chat_id)

    async def _pulse(self, chat_id: int, turn: _Turn) -> None:
        try:
            while not turn.done and self._clock() - turn.started < TURN_MAX_S:
                await self._tick(chat_id, turn)
                await self._sleep(TICK_S)
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 - индикатор не должен ронять ответ
            self._log.warning("индикатор хода chat=%s упал: %r", chat_id, exc)
        await self._finish(chat_id, turn)

    async def _tick(self, chat_id: int, turn: _Turn) -> None:
        now = self._clock()
        if now - turn.last_typing >= TYPING_EVERY_S:
            turn.last_typing = now
            await self._typing(chat_id)
        elapsed = now - turn.started
        if turn.label and elapsed >= PROGRESS_AFTER_S and now - turn.last_progress >= PROGRESS_EVERY_S:
            turn.last_progress = now
            await self._show_progress(chat_id, turn, chat_render.progress(turn.label, elapsed))

    async def _typing(self, chat_id: int) -> None:
        try:
            await self.bot.send_chat_action(chat_id=chat_id, action=ACTION_TYPING)
        except Exception as exc:  # noqa: BLE001
            self._log.debug("send_chat_action chat=%s: %r", chat_id, exc)

    async def _show_progress(self, chat_id: int, turn: _Turn, item: chat_render.Rendered) -> None:
        if _is_private(chat_id) and turn.draft_ok and turn.msg_id is None:
            try:
                await self.bot.do_api_request(
                    "sendMessageDraft",
                    api_kwargs={"chat_id": chat_id, "draft_id": DRAFT_ID, "text": item.text,
                                "parse_mode": item.parse_mode})
                return
            except Exception as exc:  # noqa: BLE001 - черновик недоступен: один правимый индикатор сообщением
                turn.draft_ok = False
                self._log.info("sendMessageDraft chat=%s отвергнут (%r): индикатор - правимое сообщение", chat_id, exc)
        await self._edit_progress_message(chat_id, turn, item)

    async def _edit_progress_message(self, chat_id: int, turn: _Turn, item: chat_render.Rendered) -> None:
        if item.text == turn.shown:
            return
        turn.shown = item.text
        if turn.msg_id is None:
            sent = await self.bot.send_message(chat_id=chat_id, text=item.text, parse_mode=item.parse_mode,
                                               disable_notification=True)
            turn.msg_id = int(sent.message_id)
        else:
            await self.bot.edit_message_text(chat_id=chat_id, message_id=turn.msg_id, text=item.text,
                                             parse_mode=item.parse_mode)

    async def _end(self, chat_id: int) -> None:
        turn = self._turns.get(chat_id)
        if turn is None or turn.done:
            return
        turn.done = True
        task = turn.task
        if task is not None and task is not asyncio.current_task():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        await self._finish(chat_id, turn)

    async def _finish(self, chat_id: int, turn: _Turn) -> None:
        turn.done = True
        if self._turns.get(chat_id) is turn:
            del self._turns[chat_id]
        if turn.msg_id is not None:
            msg_id, turn.msg_id = turn.msg_id, None
            try:
                await self.bot.delete_message(chat_id=chat_id, message_id=msg_id)
            except Exception as exc:  # noqa: BLE001
                self._log.debug("delete_message chat=%s: %r", chat_id, exc)

    async def _send(self, chat_id: int, item: chat_render.Rendered) -> None:
        options = {"link_preview_options": {"is_disabled": True}}  # у робота ссылка - не карточка сайта
        try:
            await self.bot.send_message(chat_id=chat_id, text=item.text, parse_mode=item.parse_mode, **options)
        except Exception as exc:  # noqa: BLE001
            if not _is_markup_error(exc):
                raise
            self._log.warning("разметка отвергнута (%r): реплика уходит голым текстом, chat=%s", exc, chat_id)
            await self.bot.send_message(chat_id=chat_id, text=item.plain, **options)


__all__ = ["ACTION_TYPING", "ChatSender", "DRAFT_ID", "KIND_BEGIN", "PROGRESS_AFTER_S", "TURN_MAX_S"]
