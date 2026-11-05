"""Снимок лица в чат (issue #4469): путь от владельца проверяется по шаблону, файл читает бот, отправка - ``ChatSender``."""

from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace

import pytest

from rob_box_telegram import person_photo
from rob_box_telegram.chat_sender import ChatSender

PID = "fff371fe-1111-2222-3333-444444444444"


def _faces(tmp_path):
    (tmp_path / PID / "encounters").mkdir(parents=True)
    (tmp_path / PID / "reference.jpg").write_bytes(b"ref")
    (tmp_path / PID / "encounters" / "000003_face.jpg").write_bytes(b"enc")
    (tmp_path / "secret.jpg").write_bytes(b"secret")
    return tmp_path


def test_reads_reference_and_encounter_files(tmp_path):
    root = _faces(tmp_path)
    assert person_photo.read_photo(f"{PID}/reference.jpg", root) == b"ref"
    assert person_photo.read_photo(f"{PID}/encounters/000003_face.jpg", root) == b"enc"


@pytest.mark.parametrize("bad", [
    "../secret.jpg", f"{PID}/../secret.jpg", "secret.jpg", f"{PID}/meta.json", "/etc/passwd",
    f"{PID}/encounters/../../secret.jpg", f"{PID}/reference.jpg/", "", f"{PID}/missing.jpg",
])
def test_paths_outside_the_template_or_missing_files_are_not_read(tmp_path, bad):
    assert person_photo.read_photo(bad, _faces(tmp_path)) is None


def test_payload_parsing():
    text = json.dumps({"path": "a/reference.jpg", "caption": "x" * 500}, ensure_ascii=False)
    path, caption = person_photo.parse_payload(text)
    assert path == "a/reference.jpg" and len(caption) == person_photo.CAPTION_MAX
    assert person_photo.parse_payload("не json") is None and person_photo.parse_payload('{"caption": "x"}') is None


class Bot:
    def __init__(self):
        self.photos, self.messages = [], []

    async def send_photo(self, **kw):
        self.photos.append(kw)
        return SimpleNamespace(message_id=1)

    async def send_message(self, **kw):
        self.messages.append(kw)
        return SimpleNamespace(message_id=2)


def test_sender_sends_the_photo_with_caption(tmp_path):
    bot = Bot()
    payload = json.dumps({"path": f"{PID}/reference.jpg", "caption": "fff371fe - Кирилл"}, ensure_ascii=False)

    asyncio.run(ChatSender(bot, rich=False).send_person_photo(7, payload, _faces(tmp_path)))

    assert bot.photos == [{"chat_id": 7, "photo": b"ref", "caption": "fff371fe - Кирилл"}] and bot.messages == []


def test_sender_says_honestly_when_the_file_is_not_there(tmp_path):
    bot = Bot()
    payload = json.dumps({"path": f"{PID}/encounters/000009_face.jpg", "caption": "x"})

    asyncio.run(ChatSender(bot, rich=False).send_person_photo(7, payload, _faces(tmp_path)))

    assert bot.photos == [] and "не нашёл" in bot.messages[0]["text"]
