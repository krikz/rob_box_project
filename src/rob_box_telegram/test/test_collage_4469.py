"""Коллаж «найди похожих» в чат (issue #4469): бот забирает у владельца последний коллаж (``OP_COLLAGE``), а не снимок
лица; обычный снимок идёт как прежде. Сеть и ROS не нужны: ``fetch`` и бот подставные."""

from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace

import pytest

from rob_box_telegram import person_photo
from rob_box_telegram.chat_sender import ChatSender

PID = "71cda108"


class Bot:
    def __init__(self):
        self.photos, self.messages = [], []

    async def send_photo(self, **kw):
        self.photos.append(kw)
        return SimpleNamespace(message_id=1)

    async def send_message(self, **kw):
        self.messages.append(kw)
        return SimpleNamespace(message_id=2)


class Owner:
    """``fetch`` бота: фиксирует, с каким ``op`` его позвали (по умолчанию - снимок лица)."""

    def __init__(self, reply):
        self.reply, self.asked = reply, []

    def __call__(self, person_id, op=person_photo.OP_PHOTO):
        self.asked.append((person_id, op))
        return self.reply


def _send(bot, owner, payload):
    asyncio.run(ChatSender(bot, rich=False).send_person_photo(7, payload, owner))


def test_payload_kind_picks_the_owner_operation():
    assert person_photo.payload_op(json.dumps({"id": PID, "kind": "collage"})) == person_photo.OP_COLLAGE == 5
    assert person_photo.payload_op(json.dumps({"id": PID})) == person_photo.OP_PHOTO == 1
    assert person_photo.payload_op("не json") == person_photo.OP_PHOTO
    assert person_photo.parse_payload(json.dumps({"id": PID, "kind": "collage", "caption": "c"})) == (PID, "c")


def test_collage_is_fetched_with_the_collage_op_and_sent_as_one_photo():
    bot, owner = Bot(), Owner((True, "", b"collage-bytes"))

    _send(bot, owner, json.dumps({"id": PID, "kind": "collage", "caption": "Похожие на 71cda108"}, ensure_ascii=False))

    assert owner.asked == [(PID, person_photo.OP_COLLAGE)]
    assert bot.photos == [{"chat_id": 7, "photo": b"collage-bytes", "caption": "Похожие на 71cda108"}]


def test_plain_photo_is_still_fetched_by_id_only():
    bot, owner = Bot(), Owner((True, "", b"jpeg"))

    _send(bot, owner, json.dumps({"id": PID, "caption": "c"}))

    assert owner.asked == [(PID, person_photo.OP_PHOTO)] and len(bot.photos) == 1


@pytest.mark.parametrize("reason, fragment", [("no_collage", "Коллажа"), ("owner_unavailable", "не ответил")])
def test_missing_collage_is_said_honestly_and_nothing_is_sent_as_a_photo(reason, fragment):
    bot = Bot()

    _send(bot, Owner((False, reason, b"")), json.dumps({"id": PID, "kind": "collage"}))

    assert bot.photos == [] and fragment in bot.messages[0]["text"]
