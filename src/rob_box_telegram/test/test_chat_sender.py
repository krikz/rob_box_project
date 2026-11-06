"""#4243: показ хода процесса и отправка ответа - заглушка бота, часы и сон подменены (без сети и без Telegram)."""

from __future__ import annotations

import asyncio
import logging
import os
import unittest
from unittest import mock
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


def make(bot=None, rich=False):
    clock = Clock()

    async def sleep(seconds):
        clock.now += seconds
        await asyncio.sleep(0)

    return cs.ChatSender(bot or Bot(), clock=clock, sleep=sleep, log=logging.getLogger("t"), rich=rich), clock


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

    async def test_steps_pile_up_in_one_indicator_and_repeat_is_not_a_new_step(self):
        sender, _ = make()
        sender.begin(GROUP)
        for label in ("Думаю…", "Думаю…", "Ищу в сети…", "Ищу в сети - готово"):
            sender.set_progress(GROUP, label)
        await spin(30)
        shown = [c for c in sender.bot.calls if c[0] == "send"][0][2]
        assert shown == "<i>• Думаю…\n• Ищу в сети…\n⏳ Ищу в сети - готово 3 с</i>"
        await sender.close()

    async def test_rich_messages_go_by_raw_call_when_enabled(self):
        sender, _ = make()
        sender._rich_ok = True
        await sender.deliver(PRIVATE, "Ток 5 < 7", "Ермил")
        assert sender.bot.calls == [("api", "sendRichMessage", {
            "chat_id": PRIVATE, "rich_message": {"html": "<b>Ермил</b>\n<blockquote>Ток 5 &lt; 7</blockquote>"}})]

    async def test_rich_refused_falls_back_to_html_and_is_not_retried(self):
        sender, _ = make(Bot(draft_error=RuntimeError("method not found")))
        sender._rich_ok = True
        await sender.deliver(PRIVATE, "Раз")
        await sender.deliver(PRIVATE, "Два")
        assert [c[0] for c in sender.bot.calls] == ["api", "send", "send"] and sender._rich_ok is False

    async def test_timeout_on_send_is_retried_and_the_reply_is_not_lost(self):
        class TimedOut(Exception):  # имя как у telegram.error.TimedOut
            pass

        class Flaky(Bot):
            fails = 1

            async def send_message(self, **kw):
                if self.fails:
                    self.fails -= 1
                    raise TimedOut("Timed out")
                return await super().send_message(**kw)

        sender, clock = make(Flaky())
        await sender.deliver(PRIVATE, "Ответ")
        assert [c[0] for c in sender.bot.calls] == ["send"] and clock.now >= cs.RETRY_PAUSE_S

    async def test_network_error_gives_up_after_the_attempts_but_bad_request_is_not_retried(self):
        class NetworkError(Exception):
            pass

        class BadRequest(NetworkError):  # как в python-telegram-bot: BadRequest - потомок NetworkError
            pass

        class Down(Bot):
            tries = 0

            async def send_message(self, **kw):
                Down.tries += 1
                raise self.err

        down = Down()
        down.err = NetworkError("no route")
        sender, _ = make(down)
        with pytest.raises(NetworkError):
            await sender.deliver(PRIVATE, "Ответ")
        assert Down.tries == cs.RETRY_ATTEMPTS

        Down.tries = 0
        down.err = BadRequest("Chat not found")
        with pytest.raises(BadRequest):
            await sender.deliver(PRIVATE, "Ответ")
        assert Down.tries == 1

    async def test_rich_timeout_keeps_rich_on_and_the_reply_still_goes_as_html(self):
        class TimedOut(Exception):
            pass

        class Flaky(Bot):
            async def do_api_request(self, endpoint, api_kwargs=None):
                raise TimedOut("Timed out")

        sender, _ = make(Flaky())
        sender._rich_ok = True
        await sender.deliver(PRIVATE, "Ответ")
        assert [c[0] for c in sender.bot.calls] == ["send"] and sender._rich_ok is True

    async def test_rich_is_on_by_default(self):
        with mock.patch.dict(os.environ):
            os.environ.pop(cs.RICH_ENV, None)
            sender, _ = make(rich=None)
        assert sender._rich_ok is True
        await sender.deliver(PRIVATE, "Раз")
        assert [c[0] for c in sender.bot.calls] == ["api"]

    async def test_rich_can_be_switched_off_by_env(self):
        for value in ("0", "false", "No", "OFF"):
            with mock.patch.dict(os.environ, {cs.RICH_ENV: value}):
                sender, _ = make(rich=None)
            assert sender._rich_ok is False, value
            await sender.deliver(PRIVATE, "Раз")
            assert [c[0] for c in sender.bot.calls] == ["send"], value

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
