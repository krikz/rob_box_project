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
import os
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable, Dict, List, Optional

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
#: Новый шаг хода показываем не позже чем через столько после прошлого обновления (лимит правок Bot API).
PROGRESS_STEP_EVERY_S = 1.0
#: Потолок хода: дольше индикатор не держим.
TURN_MAX_S = 120.0
#: Шаг цикла индикатора.
TICK_S = 1.0
#: Повтор при сетевом сбое (``TimedOut``/``NetworkError``; не ``BadRequest``): всего попыток и пауза перед n-й (n * пауза).
#: Политика «дубль лучше потери»: ``TimedOut`` у ``sendMessage`` может значить «доставлено, но ответ не дошёл», и повтор
#: тогда даст второе такое же сообщение; потерянный ответ робота хуже (ответ Шифу 17:26 UTC пропал целиком).
RETRY_ATTEMPTS = 3
RETRY_PAUSE_S = 1.0
#: Идентификатор черновика: один и тот же - Telegram анимирует изменения вместо замены.
DRAFT_ID = 1

#: ``kind`` исходящего элемента очереди сверх ``DialogOutput.kind``: «ход начался» (внутренний, не с провода).
KIND_BEGIN = "begin"

#: Переменная окружения: по умолчанию ответы уходят «богатыми сообщениями» (Bot API 10.1 ``sendRichMessage``), при отказе - HTML.
#: Выключить: ``0`` / ``false`` / ``no`` / ``off`` (решение Шифу 03.11, #4243).
RICH_ENV = "TELEGRAM_RICH_MESSAGES"
_RICH_OFF = frozenset({"0", "false", "no", "off"})


def rich_enabled_from_env() -> bool:
    """Rich включены, если переменная не задана или не из списка «выключено»."""
    return os.environ.get(RICH_ENV, "").strip().lower() not in _RICH_OFF


Sleep = Callable[[float], Awaitable[Any]]


@dataclass
class _Turn:
    started: float
    steps: List[str] = field(default_factory=list)
    changed: bool = False
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


def _is_transient(exc: Exception) -> bool:
    """Сетевой сбой (таймаут, обрыв связи), а не отказ Telegram по существу: повтор имеет смысл. ``telegram.error``
    не импортируем (тесты идут без библиотеки): смотрим имена в MRO - ``BadRequest`` наследует ``NetworkError`` в
    python-telegram-bot, но это отказ сервера (битая разметка, чужое сообщение), его повторять нечего."""
    names = {cls.__name__ for cls in type(exc).__mro__}
    if "BadRequest" in names:
        return False
    return bool(names & {"TimedOut", "NetworkError", "TimeoutError", "ConnectionError"})


def _is_markup_error(exc: Exception) -> bool:
    return "parse entities" in str(exc).lower() or "can't parse" in str(exc).lower()


class ChatSender:
    """Ответы и ход процесса по чатам; один ход на чат."""

    def __init__(self, bot: Any, clock: Callable[[], float] = time.monotonic, sleep: Sleep = asyncio.sleep,
                 log: logging.Logger = _LOG, rich: Optional[bool] = None) -> None:
        self.bot = bot
        # Rich messages - по умолчанию вкл.: PTB 21.11.1 метода не знает, шлём сырым do_api_request; отказ - откат на HTML навсегда
        self._rich_ok = rich_enabled_from_env() if rich is None else rich
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
        steps = self._turns[chat_id].steps
        if not steps or steps[-1] != label:  # тот же шаг подряд - не новая строка
            steps.append(label)
            self._turns[chat_id].changed = True

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
        due = PROGRESS_STEP_EVERY_S if turn.changed else PROGRESS_EVERY_S  # новый шаг видно раньше счётчика секунд
        if turn.steps and elapsed >= PROGRESS_AFTER_S and now - turn.last_progress >= due:
            turn.last_progress, turn.changed = now, False
            await self._show_progress(chat_id, turn, chat_render.progress(turn.steps, elapsed))

    async def _retry(self, what: str, call: Callable[[], Awaitable[Any]]) -> Any:
        """Вызов Bot API с повтором при сетевом сбое; отказ по существу и последняя неудача уходят наверх."""
        for attempt in range(1, RETRY_ATTEMPTS + 1):
            try:
                return await call()
            except Exception as exc:  # noqa: BLE001
                if attempt == RETRY_ATTEMPTS or not _is_transient(exc):
                    raise
                self._log.warning("%s: сеть (%s), повтор %s из %s", what, type(exc).__name__, attempt + 1,
                                  RETRY_ATTEMPTS)
                await self._sleep(RETRY_PAUSE_S * attempt)

    async def _typing(self, chat_id: int) -> None:
        try:
            await self.bot.send_chat_action(chat_id=chat_id, action=ACTION_TYPING)
        except Exception as exc:  # noqa: BLE001
            self._log.debug("send_chat_action chat=%s: %r", chat_id, exc)

    async def _show_progress(self, chat_id: int, turn: _Turn, item: chat_render.Rendered) -> None:
        if _is_private(chat_id) and turn.draft_ok and turn.msg_id is None:
            kwargs = {"chat_id": chat_id, "draft_id": DRAFT_ID, "text": item.text, "parse_mode": item.parse_mode}
            try:
                await self._retry("sendMessageDraft",
                                  lambda: self.bot.do_api_request("sendMessageDraft", api_kwargs=kwargs))
                return
            except Exception as exc:  # noqa: BLE001 - черновик недоступен: один правимый индикатор сообщением
                if _is_transient(exc):  # сеть моргнула: этот шаг пропущен, на следующем тике попробуем снова
                    return
                turn.draft_ok = False
                self._log.info("sendMessageDraft chat=%s отвергнут (%r): индикатор - правимое сообщение", chat_id, exc)
        await self._edit_progress_message(chat_id, turn, item)

    async def _edit_progress_message(self, chat_id: int, turn: _Turn, item: chat_render.Rendered) -> None:
        if item.text == turn.shown:
            return
        turn.shown = item.text
        if turn.msg_id is None:
            sent = await self._retry("send_message(индикатор)", lambda: self.bot.send_message(
                chat_id=chat_id, text=item.text, parse_mode=item.parse_mode, disable_notification=True))
            turn.msg_id = int(sent.message_id)
        else:
            msg_id = turn.msg_id
            await self._retry("edit_message_text", lambda: self.bot.edit_message_text(
                chat_id=chat_id, message_id=msg_id, text=item.text, parse_mode=item.parse_mode))

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
                await self._retry("delete_message", lambda: self.bot.delete_message(chat_id=chat_id, message_id=msg_id))
            except Exception as exc:  # noqa: BLE001
                self._log.debug("delete_message chat=%s: %r", chat_id, exc)

    async def _send_rich(self, chat_id: int, item: chat_render.Rendered) -> bool:
        """Реплика богатым сообщением (``rich_message.html`` - тот же HTML, что у :func:`chat_render.render`); ``False`` -
        Telegram отказал, дальше идёт обычный HTML (и до перезапуска rich больше не пробуем)."""
        kwargs = {"chat_id": chat_id, "rich_message": {"html": item.text}}
        try:
            await self._retry("sendRichMessage", lambda: self.bot.do_api_request("sendRichMessage", api_kwargs=kwargs))
            return True
        except Exception as exc:  # noqa: BLE001
            self._rich_ok = _is_transient(exc)  # сеть моргнула - rich остаётся; отказ сервера - выключаем
            self._log.warning("sendRichMessage отвергнут (%r): ответы идут обычным HTML, chat=%s", exc, chat_id)
            return False

    async def _send(self, chat_id: int, item: chat_render.Rendered) -> None:
        if self._rich_ok and await self._send_rich(chat_id, item):
            return
        options = {"link_preview_options": {"is_disabled": True}}  # у робота ссылка - не карточка сайта
        try:
            await self._retry("send_message", lambda: self.bot.send_message(
                chat_id=chat_id, text=item.text, parse_mode=item.parse_mode, **options))
        except Exception as exc:  # noqa: BLE001
            if not _is_markup_error(exc):
                raise
            self._log.warning("разметка отвергнута (%r): реплика уходит голым текстом, chat=%s", exc, chat_id)
            await self._retry("send_message(plain)", lambda: self.bot.send_message(
                chat_id=chat_id, text=item.plain, **options))


__all__ = ["ACTION_TYPING", "ChatSender", "DRAFT_ID", "KIND_BEGIN", "PROGRESS_AFTER_S", "RETRY_ATTEMPTS", "RICH_ENV", "TURN_MAX_S"]
