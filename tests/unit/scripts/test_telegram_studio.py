"""Студия канала: прямой транспорт, кнопки, состояние. Сеть замокана."""

import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_lib as lib  # noqa: E402
import channel_studio as studio  # noqa: E402


# --------------------------------------------------------------------------- DirectTransport


class FakeResponse:
    def __init__(self, payload):
        self._payload = payload

    def json(self):
        return self._payload


class FakeSession:
    """Заглушка requests.Session: помнит вызовы, отдаёт заготовленный ответ."""

    def __init__(self, payload):
        self.payload = payload
        self.calls = []

    def post(self, url, json=None, data=None, files=None, timeout=None):
        self.calls.append({"url": url, "json": json, "data": data, "files": files})
        return FakeResponse(self.payload)


def test_direct_transport_requires_token(monkeypatch):
    monkeypatch.delenv("TG_STUDIO_BOT_TOKEN", raising=False)
    with pytest.raises(RuntimeError, match="TG_STUDIO_BOT_TOKEN"):
        lib.DirectTransport()


def test_direct_transport_reads_token_from_env(monkeypatch):
    monkeypatch.setenv("TG_STUDIO_BOT_TOKEN", "123:ABC")
    assert lib.DirectTransport().token == "123:ABC"


def test_direct_transport_sends_text(monkeypatch):
    monkeypatch.setenv("TG_STUDIO_BOT_TOKEN", "123:ABC")
    session = FakeSession({"ok": True, "result": {"message_id": 77}})
    transport = lib.DirectTransport(session=session)
    result = transport("sendMessage", {"chat_id": -100, "text": "привет"})
    assert result["ok"] is True
    assert result["message_id"] == 77
    assert result["message_ids"] == [77]
    # токен уходит в URL и не должен попасть в параметры
    assert "123:ABC" in session.calls[0]["url"]
    assert session.calls[0]["json"]["text"] == "привет"


def test_direct_transport_uploads_photo(tmp_path, monkeypatch):
    monkeypatch.setenv("TG_STUDIO_BOT_TOKEN", "123:ABC")
    photo = tmp_path / "pic.png"
    photo.write_bytes(b"\x89PNG")
    session = FakeSession({"ok": True, "result": {"message_id": 5}})
    transport = lib.DirectTransport(session=session)
    result = transport("sendPhoto", {"chat_id": -100, "caption": "подпись"}, photo)
    assert result["ok"] is True
    call = session.calls[0]
    # files — это dict {поле: файловый дескриптор}
    assert "photo" in call["files"]
    assert call["files"]["photo"].name.endswith("pic.png")
    assert call["data"]["caption"] == "подпись"


def test_direct_transport_album_uses_f0_f1(tmp_path, monkeypatch):
    monkeypatch.setenv("TG_STUDIO_BOT_TOKEN", "123:ABC")
    a, b = tmp_path / "a.png", tmp_path / "b.png"
    a.write_bytes(b"a")
    b.write_bytes(b"b")
    session = FakeSession({"ok": True, "result": [{"message_id": 1}, {"message_id": 2}]})
    transport = lib.DirectTransport(session=session)
    result = transport("sendMediaGroup", {"chat_id": -100, "media": []}, [a, b])
    assert result["message_ids"] == [1, 2]
    assert set(session.calls[0]["files"]) == {"f0", "f1"}


def test_direct_transport_reports_error_without_token(tmp_path, monkeypatch):
    monkeypatch.setenv("TG_STUDIO_BOT_TOKEN", "123:SECRET")
    session = FakeSession({"ok": False, "description": "chat not found"})
    transport = lib.DirectTransport(session=session)
    result = transport("sendMessage", {"chat_id": 1, "text": "x"})
    assert result["ok"] is False
    assert result["description"] == "chat not found"


def test_direct_transport_survives_network_error(monkeypatch):
    monkeypatch.setenv("TG_STUDIO_BOT_TOKEN", "123:ABC")

    class Broken:
        def post(self, *args, **kwargs):
            raise OSError("сеть упала")

    transport = lib.DirectTransport(session=Broken())
    result = transport("sendMessage", {"chat_id": 1, "text": "x"})
    assert result["ok"] is False
    assert "OSError" in result["description"]


def test_make_transport_picks_implementation(monkeypatch):
    monkeypatch.setenv("TG_STUDIO_BOT_TOKEN", "123:ABC")
    assert isinstance(lib.make_transport("direct"), lib.DirectTransport)
    assert isinstance(lib.make_transport("robot"), lib.RobotTransport)
    with pytest.raises(ValueError, match="неизвестный транспорт"):
        lib.make_transport("carrier-pigeon")


# --------------------------------------------------------------------------- состояние


def test_state_roundtrip(tmp_path):
    path = tmp_path / "state.json"
    state = studio.load_state(path)
    assert state == {"offset": 0, "ideas": [], "posts": {}}
    state["ideas"].append({"id": "i1", "title": "тест", "status": "proposed"})
    state["offset"] = 42
    studio.save_state(state, path)
    again = studio.load_state(path)
    assert again["offset"] == 42
    assert again["ideas"][0]["title"] == "тест"


def test_broken_state_file_does_not_crash(tmp_path):
    path = tmp_path / "state.json"
    path.write_text("{это не json", encoding="utf-8")
    assert studio.load_state(path)["offset"] == 0


# --------------------------------------------------------------------------- кнопки


def test_idea_keyboard_has_three_buttons():
    kb = studio.idea_keyboard("i1")
    row = kb["inline_keyboard"][0]
    assert len(row) == 3
    assert [b["callback_data"] for b in row] == [
        "studio:idea:i1:take",
        "studio:idea:i1:discuss",
        "studio:idea:i1:skip",
    ]


def test_post_keyboard_has_publish_and_reject():
    kb = studio.post_keyboard("001")
    row = kb["inline_keyboard"][0]
    assert [b["callback_data"] for b in row] == ["studio:post:001:publish", "studio:post:001:reject"]


def test_callback_data_fits_telegram_limit():
    # Telegram ограничивает callback_data 64 байтами
    for data in [
        b["callback_data"]
        for b in studio.idea_keyboard("x" * 32)["inline_keyboard"][0]
    ]:
        assert len(data.encode()) <= 64


@pytest.mark.parametrize(
    "action,expected",
    [("take", "agreed"), ("discuss", "discuss"), ("skip", "rejected")],
)
def test_idea_actions_change_status(action, expected):
    state = {"ideas": [{"id": "i1", "title": "идея", "status": "proposed"}]}
    answer = studio.handle_callback(state, {"data": f"studio:idea:i1:{action}"})
    assert state["ideas"][0]["status"] == expected
    assert answer


def test_unknown_idea_is_reported():
    state = {"ideas": []}
    answer = studio.handle_callback(state, {"data": "studio:idea:i9:take"})
    assert "не найдена" in answer


def test_post_publish_marks_approved():
    state = {"posts": {}}
    studio.handle_callback(state, {"data": "studio:post:001:publish"})
    assert state["posts"]["001"]["status"] == "approved"


def test_post_reject_marks_rejected():
    state = {"posts": {}}
    studio.handle_callback(state, {"data": "studio:post:001:reject"})
    assert state["posts"]["001"]["status"] == "rejected"


def test_garbage_callback_is_ignored():
    state = {"ideas": [], "posts": {}}
    assert studio.handle_callback(state, {"data": "что-то"}) == "Непонятная кнопка."


def test_callback_with_wrong_kind_is_rejected():
    state = {"ideas": [], "posts": {}}
    # take — это действие идеи, посту оно не полагается
    answer = studio.handle_callback(state, {"data": "studio:post:001:take"})
    assert "Непонятное действие" in answer
    assert "posts" not in state or state["posts"] == {}


# --------------------------------------------------------------------------- опрос апдейтов


class FakeBot:
    def __init__(self, updates):
        self.updates = updates
        self.answered = []
        self.sent = []

    def get_updates(self, offset, timeout=30):
        return self.updates

    def answer_callback(self, callback_id, text=""):
        self.answered.append((callback_id, text))
        return {"ok": True}

    def send_message(self, chat_id, text, keyboard=None, parse_mode="HTML"):
        self.sent.append({"chat_id": chat_id, "text": text, "keyboard": keyboard})
        return {"ok": True}


def test_poll_once_handles_callback_and_advances_offset(tmp_path):
    path = tmp_path / "state.json"
    state = studio.load_state(path)
    state["ideas"] = [{"id": "i1", "title": "идея", "status": "proposed"}]
    bot = FakeBot(
        [
            {
                "update_id": 100,
                "callback_query": {
                    "id": "cb1",
                    "data": "studio:idea:i1:take",
                    "message": {"chat": {"id": 495039871}},
                },
            }
        ]
    )
    handled = studio.poll_once(bot, state, path, timeout=0)
    assert handled == 1
    assert state["offset"] == 101
    assert state["ideas"][0]["status"] == "agreed"
    assert bot.answered[0][0] == "cb1"
    assert bot.sent[0]["chat_id"] == 495039871
    # состояние сохранено на диск
    assert studio.load_state(path)["offset"] == 101


def test_poll_once_ignores_non_callback_updates(tmp_path):
    path = tmp_path / "state.json"
    state = studio.load_state(path)
    bot = FakeBot([{"update_id": 5, "message": {"text": "привет"}}])
    assert studio.poll_once(bot, state, path, timeout=0) == 0
    assert state["offset"] == 6


# --------------------------------------------------------------------------- отправка идей


class RecordingBot:
    def __init__(self):
        self.sent = []

    def send_message(self, chat_id, text, keyboard=None, parse_mode="HTML"):
        self.sent.append({"chat_id": chat_id, "text": text, "keyboard": keyboard})
        return {"ok": True}


def test_send_ideas_gives_each_its_buttons():
    bot = RecordingBot()
    ideas = [{"id": "i1", "title": "про навигацию", "facts": ["PR #123"], "show": "гиф"}]
    assert studio.send_ideas(bot, ideas, 495039871) == 1
    message = bot.sent[0]
    assert message["chat_id"] == 495039871
    assert "про навигацию" in message["text"]
    assert "PR #123" in message["text"]
    assert "гиф" in message["text"]
    assert message["keyboard"]["inline_keyboard"][0][0]["callback_data"] == "studio:idea:i1:take"


def test_send_nothing_when_no_ideas():
    bot = RecordingBot()
    assert studio.send_ideas(bot, [], 495039871) == 0
    assert bot.sent == []


def test_send_post_with_publish_button():
    bot = RecordingBot()
    post = {"id": "001", "title": "черновик", "body_html": "<b>текст</b> #роббокс"}
    assert studio.send_post(bot, post, 495039871) == 1
    assert "studio:post:001:publish" == bot.sent[0]["keyboard"]["inline_keyboard"][0][0]["callback_data"]


# --------------------------------------------------------------------------- секреты не в git


def test_studio_secrets_are_not_tracked():
    """Токен бота и ключи не должны попадать в репозиторий (ADR-0166 §8)."""
    ignore = (REPO / "docker" / "studio" / ".gitignore").read_text(encoding="utf-8")
    assert ".env.secrets" in ignore
    assert ".env" in ignore
    # шаблон в репо — только плейсхолдеры
    example = (REPO / "docker" / "studio" / ".env.example").read_text(encoding="utf-8")
    assert "TG_STUDIO_BOT_TOKEN=" in example
    for line in example.splitlines():
        if line.startswith("TG_STUDIO_BOT_TOKEN="):
            assert line.split("=", 1)[1] == "", "в шаблоне не должно быть значения токена"


def test_state_file_not_in_docs_channel():
    """state.json — локальное состояние, в git ему не место."""
    assert studio.STATE_PATH.name == "state.json"
    assert "docs" not in studio.STATE_PATH.parts
