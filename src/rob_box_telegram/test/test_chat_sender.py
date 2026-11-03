"""#4243: показ хода процесса и отправка ответа - заглушка бота, часы и сон подменены (без сети и без Telegram)."""

from __future__ import annotations

import asyncio
import logging
import unittest
from types import SimpleNamespace

import pytest

from rob_box_telegram import chat_sender as cs

PRIVATE, GROUP = 495039871, -100123


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class Bot:
    """Записывает вызовы Bot API; ``draft_error`` / ``markup_error`` - отказ сервера."""

    def __init__(self, draft_error=None, markup_error=False):
        self.calls, self.draft_error, self.markup_error, self._id = [], draft_error, markup_error, 100

    async def send_chat_action(self, **kw):
        self.calls.append(("action", kw["chat_id"], kw["action"]))

    async def do_api_request(self, endpoint, api_kwargs=None):
        self.calls.append(("api", endpoint, api_kwargs))
        if self.draft_error:
            raise self.draft_error
        return True

    async def send_message(self, **kw):
        if self.markup_error and kw.get("parse_mode"):
            raise ValueError("Bad Request: can't parse entities: Unsupported start tag")
        self._id += 1
        self.calls.append(("send", kw["chat_id"], kw["text"], kw.get("parse_mode"), kw.get("disable_notification")))
        return SimpleNamespace(message_id=self._id)

    async def edit_message_text(self, **kw):
        self.calls.append(("edit", kw["chat_id"], kw["message_id"], kw["text"]))

    async def delete_message(self, **kw):
        self.calls.append(("delete", kw["chat_id"], kw["message_id"]))


def make(bot=None):
    clock = Clock()

    async def sleep(seconds):
        clock.now += seconds
        await asyncio.sleep(0)

    return cs.ChatSender(bot or Bot(), clock=clock, sleep=sleep, log=logging.getLogger("t")), clock


async def spin(n=60):
    for _ in range(n):
        await asyncio.sleep(0)


class ChatSenderTest(unittest.IsolatedAsyncioTestCase):
    """Свои сценарии без pytest-asyncio: CI гоняет тесты пакета по одному файлу из его каталога."""

    async def test_typing_starts_at_once_and_repeats_while_the_turn_runs(self):
        sender, _ = make()
        sender.begin(PRIVATE)
        await spin()
        actions = [c for c in sender.bot.calls if c[0] == "action"]
        assert actions and actions[0] == ("action", PRIVATE, "typing") and len(actions) >= 2
        await sender.close()


    async def test_fast_answer_shows_no_progress_and_clears_the_turn(self):
        sender, _ = make()
        sender.begin(PRIVATE)
        sender.set_progress(PRIVATE, "Думаю…")
        await sender.deliver(PRIVATE, "Привет!")
        kinds = [c[0] for c in sender.bot.calls]
        assert "api" not in kinds and "edit" not in kinds
        assert sender.bot.calls[-1] == ("send", PRIVATE, "Привет!", "HTML", None)
        assert PRIVATE not in sender._turns


    async def test_slow_private_turn_streams_a_draft_not_messages(self):
        sender, _ = make()
        sender.begin(PRIVATE)
        sender.set_progress(PRIVATE, "Думаю…")
        await spin(200)
        drafts = [c for c in sender.bot.calls if c[0] == "api"]
        assert drafts and drafts[0][1] == "sendMessageDraft"
        assert drafts[0][2]["chat_id"] == PRIVATE and drafts[0][2]["draft_id"] == cs.DRAFT_ID
        assert "Думаю…" in drafts[0][2]["text"]
        assert not [c for c in sender.bot.calls if c[0] == "send"]  # ни одного нового сообщения
        await sender.close()


    async def test_group_turn_edits_one_progress_message_and_deletes_it_before_the_answer(self):
        sender, _ = make()
        sender.begin(GROUP)
        sender.set_progress(GROUP, "Думаю…")
        await spin(30)
        sender.set_progress(GROUP, "Делаю…")
        await spin(30)
        sends = [c for c in sender.bot.calls if c[0] == "send"]
        edits = [c for c in sender.bot.calls if c[0] == "edit"]
        assert len(sends) == 1 and sends[0][4] is True  # индикатор - один и тихий
        assert edits and all(e[2] == 101 for e in edits) and "Делаю…" in edits[-1][3]
        assert not [c for c in sender.bot.calls if c[0] == "api"]  # в группе черновика нет
        await sender.deliver(GROUP, "Готово.")
        assert sender.bot.calls[-2:] == [("delete", GROUP, 101), ("send", GROUP, "Готово.", "HTML", None)]


    async def test_rejected_draft_falls_back_to_one_editable_message_once(self):
        sender, _ = make(Bot(draft_error=RuntimeError("method not found")))
        sender.begin(PRIVATE)
        sender.set_progress(PRIVATE, "Думаю…")
        await spin(200)
        assert len([c for c in sender.bot.calls if c[0] == "api"]) == 1  # больше не пробуем
        assert len([c for c in sender.bot.calls if c[0] == "send"]) == 1
        assert [c for c in sender.bot.calls if c[0] == "edit"]
        await sender.close()


    async def test_turn_without_an_answer_expires_and_removes_the_progress_message(self):
        sender, clock = make()
        sender.begin(GROUP)
        sender.set_progress(GROUP, "Думаю…")
        for _ in range(400):
            await spin(5)
            if GROUP not in sender._turns:
                break
        assert GROUP not in sender._turns and clock.now >= cs.TURN_MAX_S
        assert sender.bot.calls[-1][0] == "delete"


    async def test_markup_rejected_by_telegram_resends_the_same_reply_as_plain_text(self):
        sender, _ = make(Bot(markup_error=True))
        await sender.deliver(PRIVATE, "Ток < 5", "Ермил")
        assert sender.bot.calls == [("send", PRIVATE, "Ермил\nТок < 5", None, None)]


    async def test_other_send_errors_are_not_swallowed(self):
        class Down(Bot):
            async def send_message(self, **kw):
                raise ConnectionError("net down")

        sender, _ = make(Down())
        with pytest.raises(ConnectionError):
            await sender.deliver(PRIVATE, "x")


    async def test_long_reply_goes_as_several_messages_with_link_previews_off(self):
        sent = []

        class Spy(Bot):
            async def send_message(self, **kw):
                sent.append(kw)
                return SimpleNamespace(message_id=1)

        sender, _ = make(Spy())
        await sender.deliver(PRIVATE, "слово " * 2000)
        assert len(sent) >= 3 and all(kw["link_preview_options"] == {"is_disabled": True} for kw in sent)
