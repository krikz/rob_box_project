"""Индикатор работы студии: одно сообщение, которое правится на месте.

Просьба Шифу (06.11): «пока студия занята — показывай шаги». Шаги строит КОД из
реальных этапов (ADR-0148): модель их не придумывает. Пример ленты:

    ✓ Беру пост и факты сессии: 3 факта
    ✓ Пишу текст (MiniMax-M3.1-Flash)
    ✓ Проверяю цифры: 5/5, имена: 2/2
    ⏳ Рисую инфографику…

Правила Bot API 10.3, на которые опирается модуль (core.telegram.org/bots/api, проверено 06.11.2026):
  * editMessageText — правка того же сообщения; ``<blockquote expandable>`` — раскрывающаяся цитата
    (сюда идёт поле модели ``why`` — короткое «почему такая подача», проверенное кодом);
  * sendChatAction — статус держится ≤ 5 с, поэтому его повторяет фоновый поток каждые 4 с;
  * FAQ ботов: в один чат не чаще сообщения в секунду, поэтому правки индикатора — не чаще
    ``MIN_INTERVAL`` (1,5 с): промежуточный шаг ждёт, а не пропадает.
"""

from __future__ import annotations

import html
import threading
import time
from typing import Callable, Optional

MIN_INTERVAL = 1.5  # с между правками одного сообщения (FAQ ботов: ≤ 1 сообщения в секунду на чат)
ACTION_REPEAT = 4.0  # с: sendChatAction живёт до 5 с — повторяем раньше
WHY_LIMIT = 300  # знаков: поле why модели (узкая схема, проверка в studio_text.check_why)


def esc(text: str) -> str:
    return html.escape(text or "", quote=False)


class ChatAction:
    """Фоновый повтор sendChatAction, пока идёт работа. Действие меняется по шагам."""

    def __init__(self, bot, chat_id: int, interval: float = ACTION_REPEAT):
        self.bot, self.chat_id, self.interval = bot, chat_id, interval
        self.action = ""
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def set(self, action: str) -> None:
        if not action or action == self.action:
            return
        self.action = action
        self._send()
        if self._thread is None:
            self._thread = threading.Thread(target=self._loop, name="studio-chat-action", daemon=True)
            self._thread.start()

    def _send(self) -> None:
        sender = getattr(self.bot, "send_chat_action", None)
        if sender and self.action:
            try:
                sender(self.chat_id, self.action)
            except Exception:  # noqa: BLE001 — статус не важнее работы
                pass

    def _loop(self) -> None:
        while not self._stop.wait(self.interval):
            self._send()

    def stop(self) -> None:
        self._stop.set()


class Progress:
    """Одно сообщение-индикатор + chat action. Все тексты строит код."""

    def __init__(
        self,
        bot,
        chat_id: int,
        title: str,
        reply_to: Optional[int] = None,
        min_interval: float = MIN_INTERVAL,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ):
        self.bot, self.chat_id, self.title = bot, chat_id, title
        self.reply_to = reply_to
        self.min_interval, self.clock, self.sleep = min_interval, clock, sleep
        self.steps: list = []  # [статус, текст]: статус ✓ / ⏳ / ✗
        self.why = ""
        self.footer = ""
        self.message_id: Optional[int] = None
        self._last = 0.0
        self.action = ChatAction(bot, chat_id)
        self.edits = 0

    # ------------------------------------------------------------------ шаги

    def step(self, text: str, action: str = "typing") -> None:
        """Новый шаг: прошлый текущий становится выполненным."""
        self._close_current()
        self.steps.append(["⏳", text])
        self.action.set(action)
        self._render()

    def note(self, text: str) -> None:
        """Уточнить текст текущего шага результатом («Проверяю цифры: 5/5»)."""
        if self.steps and self.steps[-1][0] == "⏳":
            self.steps[-1][1] = text
        else:
            self.steps.append(["✓", text])
        self._render()

    def set_why(self, why: str) -> None:
        self.why = (why or "").strip()[:WHY_LIMIT]

    def finish(self, text: str = "Готово") -> None:
        self._close_current()
        self.footer = f"<b>{esc(text)}</b>"
        self.action.stop()
        self._render()

    def fail(self, reason: str) -> None:
        """Ошибка: индикатор показывает, на каком шаге упало и почему."""
        failed = "старт"
        if self.steps and self.steps[-1][0] == "⏳":
            self.steps[-1][0] = "✗"
            failed = self.steps[-1][1]
        else:
            self.steps.append(["✗", "работа"])
        self.footer = f"<b>Не вышло</b> на шаге «{esc(failed)}»: {esc(reason)}"
        self.action.stop()
        self._render()

    # ------------------------------------------------------------------ вывод

    def _close_current(self) -> None:
        if self.steps and self.steps[-1][0] == "⏳":
            self.steps[-1][0] = "✓"

    def text(self) -> str:
        lines = [f"<b>{esc(self.title)}</b>"]
        for status, text in self.steps:
            suffix = "…" if status == "⏳" else ""
            lines.append(f"{status} {esc(text)}{suffix}")
        if self.why:
            lines.append(f"<blockquote expandable>Почему так: {esc(self.why)}</blockquote>")
        if self.footer:
            lines.append(self.footer)
        return "\n".join(lines)

    def _render(self) -> None:
        # Промежуточный шаг не пропускаем, а ждём: Шифу должен увидеть каждый этап,
        # а правки одного сообщения — не чаще min_interval (лимит частоты Telegram).
        wait = self.min_interval - (self.clock() - self._last)
        if self.message_id is not None and wait > 0:
            self.sleep(wait)
        text = self.text()
        if self.message_id is None:
            if self.reply_to:
                result = self.bot.send_message(self.chat_id, text, reply_to=self.reply_to)
            else:
                result = self.bot.send_message(self.chat_id, text)
            if result.get("ok"):
                self.message_id = (result.get("result") or {}).get("message_id")
        else:
            self.bot.edit_message_text(self.chat_id, self.message_id, text)
            self.edits += 1
        self._last = self.clock()
