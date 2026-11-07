"""Кнопка «Видео» должна нести просьбу Шифу, а не собирать по умолчанию.

Шифу (07.11): «Так ты чот накосячил надо было их разместить плохой сверху
после снизу в вертикальном формате и с музыкой» → кнопка собрала горизонтальный
ролик рядом. Причина: кнопка-предложение не помнила просьбу, ради которой её
предложили, и кнопка на черновике брала пустую инструкцию.
"""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_studio as cs  # noqa: E402
import studio_intent as intent  # noqa: E402
import studio_jobs as jobs  # noqa: E402
import studio_messages as messages  # noqa: E402
import studio_state as st  # noqa: E402


class Bot:
    """Подставка StudioBot: без сети."""

    def __init__(self):
        self.calls = []

    def send_message(self, chat_id, text, keyboard=None, parse_mode="HTML", reply_to=None):
        self.calls.append(("sendMessage", text, keyboard))
        return {"ok": True, "result": {"message_id": 501}}

    def edit_message_text(self, chat_id, message_id, text, keyboard=None, parse_mode="HTML"):
        self.calls.append(("editMessageText", text, keyboard))
        return {"ok": True}

    def edit_message_caption(self, chat_id, message_id, caption, keyboard=None):
        return {"ok": True}

    def edit_message_media(self, chat_id, message_id, path, caption="", keyboard=None):
        return {"ok": True}

    def answer_callback(self, callback_id, text=""):
        self.calls.append(("answerCallbackQuery", text))
        return {"ok": True}

    def send_chat_action(self, chat_id, action):
        return {"ok": True}

    def send_media(self, chat_id, path, caption="", keyboard=None, above=False):
        self.calls.append(("sendMedia", str(path), caption))
        return {"ok": True, "result": {"message_id": 777}}


class Progress:
    def __init__(self):
        self.steps = []
        self.notes = []
        self.finished = None
        self.failed = None

    def step(self, text, action="typing"):
        self.steps.append(text)

    def note(self, text):
        self.notes.append(text)

    def set_why(self, why):
        pass

    def finish(self, text="Готово"):
        self.finished = text

    def fail(self, reason):
        self.failed = reason


class Runner:
    """Подставка JobRunner: выполняет сразу, в том же потоке."""

    def __init__(self):
        self.calls = []

    def claim(self, key):
        return True

    def submit(self, key, func, *args):
        self.calls.append((key, func.__name__))
        return func(*args)


def test_offer_button_keeps_the_request(tmp_path):
    """Кнопка-предложение помнит просьбу, ради которой её предложили."""
    state_path = tmp_path / "state.json"
    st.save_state(state_path, {})

    class Bot:
        def __init__(self):
            self.calls = []

        def send_message(self, chat_id, text, keyboard=None, parse_mode="HTML", reply_to=None):
            self.calls.append(("sendMessage", text, keyboard))
            return {"ok": True, "result": {"message_id": 501}}

        def edit_message_text(self, chat_id, message_id, text, keyboard=None, parse_mode="HTML"):
            self.calls.append(("editMessageText", text, keyboard))
            return {"ok": True}

    monkey = cs.st  # состояние сохраняется тем же кодом
    request = "сделай видео вертикальным чтобы до было сверху а после снизу"
    bot = Bot()
    cs.reply_to_shifu(bot, state_path, request)

    state = st.load_state(state_path)
    pending = state.get("pending_request") or {}
    # Просьба сохранена, если модель предложила действие (в тесте модель не
    # отвечает — поэтому проверяем и путь «желания»).
    wish = state.get("wish") or {}
    assert wish.get("text") == request or pending.get("text") == request


def test_offer_button_remembers_request_through_the_real_router(tmp_path, monkeypatch):
    """Его слова → ответ с кнопкой «Видео» → просьба лежит в состоянии."""
    state_path = tmp_path / "state.json"
    draft = {
        "id": "p1",
        "status": "proposed",
        "title": "Локализация",
        "body_html": "<b>Локализация</b> (#4495)",
        "message_id": 77,
        "message_kind": "text",
    }
    st.save_state(state_path, {"posts": {"p1": draft}})
    monkeypatch.setattr(
        cs, "run_reply_agent", lambda text, idea: '{"answer": "Сделаю вертикально", "offer": "video"}'
    )

    # Без слов «видео/ролик»: сообщение уходит в разговор, модель предлагает
    # кнопку «Видео» — именно этот путь терял просьбу (Шифу, 23:26).
    request = "надо было их разместить плохой сверху после снизу в вертикальном формате и с музыкой"
    message = {"chat": {"id": cs.SHIFU_CHAT_ID}, "message_id": 91, "text": request}
    bot = Bot()
    messages.route(bot, state_path, None, 5, message, Runner(), lambda *a: None)

    state = st.load_state(state_path)
    pending = state.get("pending_request") or {}
    assert pending.get("action") == "video"
    assert pending.get("text") == request
    wish = state.get("wish") or {}
    assert wish.get("text") == request
    # Кнопка «Видео» — в ответе бота (сообщение или правка «Думаю…»)
    keyboard = [c[2] for c in bot.calls if c[2]]
    assert keyboard, "бот не предложил кнопку"
    labels = [b["text"] for row in keyboard[0]["inline_keyboard"] for b in row]
    assert "Видео" in labels


def test_words_ask_for_the_post_are_recognised():
    """«Пришли мне собранный пост» — это действие «прислать пост», а не «ролик»."""
    assert intent.classify("Так теперь пришли мне собранный пост") == intent.ACTION_POST
    assert intent.classify("покажи пост") == intent.ACTION_POST
    assert intent.classify("пришли черновик") == intent.ACTION_POST


def test_post_action_sends_built_post_with_media(tmp_path):
    """«Пришли пост» — студия присылает собранный черновик с его медиа."""
    video = tmp_path / "p1.mp4"
    video.write_bytes(b"x" * 128)
    record = {
        "id": "p1",
        "body_html": "<b>Локализация</b> (#4495)",
        "media_path": str(video),
        "message_id": 77,
    }
    bot = Bot()
    progress = Progress()
    assert jobs._refine_post(bot, tmp_path / "state.json", record, progress, 495039871) is True
    media = [c for c in bot.calls if c[0] == "sendMedia"]
    assert media, "пост не прислали"
    assert media[0][1] == str(video)
    assert "Локализация" in media[0][2]
    assert progress.finished and "Готово" in progress.finished


def test_post_action_sends_text_when_no_media(tmp_path):
    """Без медиа пост присылается текстом."""
    record = {"id": "p1", "body_html": "<b>Локализация</b> (#4495)", "message_id": 77}
    bot = Bot()
    progress = Progress()
    assert jobs._refine_post(bot, tmp_path / "state.json", record, progress, 495039871) is True
    texts = [c for c in bot.calls if c[0] == "sendMessage"]
    assert texts and "Локализация" in texts[0][1]


def test_wish_is_used_when_button_pressed_without_request(tmp_path):
    """Кнопка на черновике без кнопки-предложения берёт свежую просьбу."""
    state = {"wish": {"text": "вертикальным, до сверху, после снизу", "at": 9999999999}}
    got = cs._take_wish(state)
    assert got["text"].startswith("вертикальным")
    assert "wish" not in state, "пожелание одноразовое"


def test_expired_wish_is_dropped():
    """Старое пожелание не подмешивается под новую сборку."""
    state = {"wish": {"text": "ролик", "at": 1.0}}
    assert cs._take_wish(state) == {}
    assert "wish" not in state


def test_video_wish_markers():
    """Слова про ролик узнаются, а обычные реплики — нет."""
    assert cs.VIDEO_WISH_RE.search("сделай видео вертикальным")
    assert cs.VIDEO_WISH_RE.search("наложи на ролик этот звук")
    assert cs.VIDEO_WISH_RE.search("до сверху, после снизу")
    assert not cs.VIDEO_WISH_RE.search("привет, как дела")


@pytest.mark.parametrize("action", ["video", "animate"])
def test_take_pending_ignores_other_action(action):
    """Чужая просьба не подставляется под другое действие."""
    state = {"pending_request": {"action": "shorter", "text": "короче", "at": 1.0}}
    assert cs._take_pending(state, action) == {}
    assert state["pending_request"] == {"action": "shorter", "text": "короче", "at": 1.0}
