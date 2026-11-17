"""Снимок лица в чат (issue #4469): id из ``DialogOutput``, байты берёт у владельца ``fetch``, отправка - ``ChatSender``.

Сеть и ROS не нужны: ``fetch`` и бот подставные. Каталог ``/data/faces`` на Sound Pi не читается.
"""

from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace

import pytest

from rob_box_telegram import person_photo
from rob_box_telegram.chat_sender import ChatSender

PID = "fff371fe"


def test_payload_parsing():
    text = json.dumps({"id": PID, "caption": "x" * 500}, ensure_ascii=False)
    pid, caption = person_photo.parse_payload(text)
    assert pid == PID and len(caption) == person_photo.CAPTION_MAX


@pytest.mark.parametrize("bad", [
    "не json", "[]", '{"caption": "x"}', '{"id": 5}', '{"id": "../secret"}', '{"id": "fff371fe-1111"}',
    '{"id": "FFF371FE"}', '{"path": "fff371fe-1111/reference.jpg"}',
])
def test_payload_without_a_valid_id_is_not_ours(bad):
    assert person_photo.parse_payload(bad) is None


def test_no_disk_reader_left():
    assert not hasattr(person_photo, "read_photo") and not hasattr(person_photo, "faces_root")


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
    def __init__(self, reply):
        self.reply, self.asked = reply, []

    def __call__(self, person_id):
        self.asked.append(person_id)
        return self.reply


def _send(bot, owner, payload):
    asyncio.run(ChatSender(bot, rich=False).send_person_photo(7, payload, owner))


def test_sender_sends_the_bytes_from_the_owner_with_caption():
    bot, owner = Bot(), Owner((True, "", b"jpeg-bytes"))

    _send(bot, owner, json.dumps({"id": PID, "caption": "fff371fe - Кирилл"}, ensure_ascii=False))

    assert owner.asked == [PID]
    assert bot.photos == [{"chat_id": 7, "photo": b"jpeg-bytes", "caption": "fff371fe - Кирилл"}] and bot.messages == []


@pytest.mark.parametrize("reason, fragment", [
    ("exhibition_mode", "выставки"), ("no_snapshot", "нет снимка"), ("person_not_found", "нет в базе"),
    ("owner_unavailable", "не ответил"), ("store_error", "не удалось"),
])
def test_owner_refusal_is_said_honestly_and_no_photo_is_sent(reason, fragment):
    bot = Bot()

    _send(bot, Owner((False, reason, b"")), json.dumps({"id": PID, "caption": "x"}))

    assert bot.photos == [] and fragment in bot.messages[0]["text"]


def test_ok_without_bytes_is_not_a_photo():
    bot = Bot()

    _send(bot, Owner((True, "", b"")), json.dumps({"id": PID}))

    assert bot.photos == [] and len(bot.messages) == 1


def test_bad_payload_never_reaches_the_owner():
    bot, owner = Bot(), Owner((True, "", b"x"))

    _send(bot, owner, '{"id": "../etc"}')

    assert owner.asked == [] and bot.photos == [] and len(bot.messages) == 1


def test_ros_fetch_without_a_ready_service_is_owner_unavailable(monkeypatch):
    import sys
    import types

    srv = types.ModuleType("rob_box_perception_msgs.srv")
    srv.People = SimpleNamespace(Request=lambda: SimpleNamespace())
    monkeypatch.setitem(sys.modules, "rob_box_perception_msgs", types.ModuleType("rob_box_perception_msgs"))
    monkeypatch.setitem(sys.modules, "rob_box_perception_msgs.srv", srv)
    node = SimpleNamespace(create_client=lambda *a: SimpleNamespace(service_is_ready=lambda: False))

    assert person_photo.ros_fetch(node)(PID) == (False, "owner_unavailable", b"")
