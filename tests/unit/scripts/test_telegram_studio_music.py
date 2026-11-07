"""Студия принимает трек и кладёт его под ролик.

Замечания Шифу (07.11):
- «нуб музычку надо наложить на водосик» — нужен звук в ролике;
- «прислал его как мп3 и он мне сказал что не может» — MP3, отправленный
  как музыка, студия отвергала: в списке файлов был только document.

Сеть, ssh/scp и ffmpeg — подставки: тест проверяет решения модулей.
"""

import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import studio_inbox as inbox  # noqa: E402
import studio_motion as motion  # noqa: E402
import studio_progress as sp  # noqa: E402
import studio_render as render  # noqa: E402
import studio_state as st  # noqa: E402
import studio_video as video  # noqa: E402

AUDIO = {
    "file_id": "f1",
    "file_size": 3 * 1024 * 1024,
    "file_name": "Werewolf Asso.mp3",
    "mime_type": "audio/mpeg",
}


# ------------------------------------------------------------------ приём файла


def test_audio_message_is_recognised_as_media():
    """MP3, отправленный как музыка, — это файл, а не «неподдерживаемый тип»."""
    media = inbox.media_of({"audio": AUDIO})
    assert media is not None
    assert media["kind"] == "audio" and media["size"] == 3 * 1024 * 1024
    assert inbox.unsupported_kind({"audio": AUDIO}) == ""


def test_audio_file_gets_mp3_extension():
    media = inbox.media_of({"audio": AUDIO})
    name = inbox.local_name(media, 90)
    assert name.endswith(".mp3")


def test_voice_still_unsupported():
    """Голосовое — по-прежнему не материал (это не трек)."""
    assert inbox.unsupported_kind({"voice": {"file_id": "v"}}) == "голосовое"


# ------------------------------------------------------------------ трек в состоянии


class Bot:
    def __init__(self):
        self.calls = []

    def get_file(self, file_id):
        self.calls.append(("getFile", file_id))
        return {"ok": True, "result": {"file_path": "music/file_1.mp3"}}

    def download_file(self, file_path, dest):
        self.calls.append(("downloadFile", file_path))
        Path(dest).parent.mkdir(parents=True, exist_ok=True)
        Path(dest).write_bytes(b"ID3" + b"x" * 512)
        return ""

    def send_message(self, chat_id, text, keyboard=None, parse_mode="HTML", reply_to=None):
        self.calls.append(("sendMessage", text))
        return {"ok": True}

    def edit_message_text(self, chat_id, message_id, text, keyboard=None, parse_mode="HTML"):
        self.calls.append(("editMessageText", text))
        return {"ok": True}

    def edit_message_caption(self, chat_id, message_id, caption, keyboard=None):
        self.calls.append(("editMessageCaption", caption))
        return {"ok": True}

    def edit_message_media(self, chat_id, message_id, path, caption="", keyboard=None):
        self.calls.append(("editMessageMedia", str(path)))
        return {"ok": True}

    def send_chat_action(self, chat_id, action):
        return {"ok": True}

    def send_media(self, chat_id, path, caption="", keyboard=None, above=False):
        self.calls.append(("sendMedia", str(path), caption))
        return {"ok": True, "result": {"message_id": 777}}


@pytest.fixture
def fast_progress(monkeypatch):
    monkeypatch.setattr(sp, "MIN_INTERVAL", 0.0)
    original = sp.Progress.__init__

    def fast(self, *args, **kwargs):
        kwargs.setdefault("min_interval", 0.0)
        kwargs.setdefault("sleep", lambda _s: None)
        original(self, *args, **kwargs)

    monkeypatch.setattr(sp.Progress, "__init__", fast)


def test_received_track_is_remembered(tmp_path, monkeypatch, fast_progress):
    """Принятый MP3 запоминается — монтаж возьмёт его под ролик."""
    run = tmp_path / "run"
    run.mkdir()
    monkeypatch.setenv("STUDIO_TMP", str(run))
    state = tmp_path / "state.json"
    st.save_state(state, {})

    media = inbox.media_of({"audio": AUDIO})
    assert inbox.attach_job(Bot(), state, "p1", media, "", 90, out=lambda *a: None) is True

    music = st.load_state(state).get("music") or {}
    assert music.get("path")
    assert Path(music["path"]).exists()
    assert music["name"].endswith(".mp3")


# ------------------------------------------------------------------ звук в монтаже


def test_ffmpeg_command_gets_audio_track(tmp_path):
    """С треком в команде ffmpeg есть вторая дорожка и aac."""
    commands = []

    class FakeStdin:
        def write(self, _data):
            return None

        def close(self):
            return None

    class FakeProc:
        def __init__(self, cmd, stdin=None, stderr=None):
            commands.append(cmd)
            self.stdin = FakeStdin()
            self.stderr = None
            self.returncode = 0
            Path(cmd[-1]).write_bytes(b"x")  # ffmpeg «записал» файл

        def wait(self):
            return 0

    frames = [__import__("PIL.Image", fromlist=["Image"]).new("RGB", (8, 8), "black")]
    out = tmp_path / "v.mp4"
    reason = render.encode_mp4(frames, (8, 8), 5, out, popen=FakeProc, music=Path("/tmp/t.mp3"))
    assert reason == ""
    cmd = commands[0]
    assert "-i" in cmd and "/tmp/t.mp3" in cmd
    assert "-c:a" in cmd and "aac" in cmd
    assert "-shortest" in cmd


def test_fit_mp4_passes_music_through(tmp_path, monkeypatch):
    seen = {}

    def fake_encode(frames, size, fps, out, crf=23, popen=None, music=None):
        seen["music"] = music
        out.write_bytes(b"x")
        return ""

    monkeypatch.setattr(render, "encode_mp4", fake_encode)
    render.fit_mp4(lambda: [], (8, 8), 5, tmp_path / "v.mp4", music=Path("/tmp/t.mp3"))
    assert seen["music"] == Path("/tmp/t.mp3")


# ------------------------------------------------------------------ трек в ролике


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


CATALOG = [
    {"id": "shisyun_s1", "label": "до", "kind": "poses"},
    {"id": "shisyun_s2", "label": "после", "kind": "poses"},
]
PLAN = {"template": "compare", "runs": ["shisyun_s1", "shisyun_s2"], "layout": "stacked", "orientation": "portrait"}


@pytest.fixture
def wire(monkeypatch, tmp_path):
    """Подставки: каталог, выбор плана, монтаж, запись в состояние."""
    calls = {"build": [], "saved": None}
    built = tmp_path / "v.mp4"
    built.write_bytes(b"x" * 512)
    track = tmp_path / "track.mp3"
    track.write_bytes(b"ID3" + b"x" * 256)

    monkeypatch.setattr(motion.material, "runs_catalog", lambda gate: (list(CATALOG), ""))
    monkeypatch.setattr(motion.video, "choose_plan", lambda record, request, facts, catalog, fmt: (dict(PLAN), "план"))

    def fake_build(plan, fmt, ctx, catalog):
        calls["build"].append(dict(plan))
        built.write_bytes(b"x" * 512)
        return built, ""

    monkeypatch.setattr(motion.video, "build", fake_build)
    monkeypatch.setattr(motion.jobs, "update_draft", lambda bot, record, chat_id, new_media=None: (True, ""))
    monkeypatch.setattr(motion.jobs, "run_dir", lambda: tmp_path)

    def fake_save(state_path, post_id, fields):
        calls["saved"] = (post_id, fields)

    monkeypatch.setattr(motion.jobs, "save_fields", fake_save)
    return calls, built, track


def test_track_from_state_goes_into_the_video(wire, tmp_path):
    """Трек, который прислал Шифу, ложится под ролик."""
    calls, built, track = wire
    state = tmp_path / "state.json"
    st.save_state(state, {"music": {"path": str(track), "name": "track.mp3"}})

    progress = Progress()
    assert motion.refine_motion(Bot(), state, {"id": "p1", "message_id": 40}, [], "video", progress, 495039871) is True
    assert calls["build"], "монтаж не вызван"
    assert calls["build"][0].get("music") == str(track)


def test_request_without_track_fails_honestly(wire, tmp_path):
    """Просят наложить трек, а трека нет — честная причина, а не тишина."""
    calls, built, track = wire
    state = tmp_path / "state.json"
    st.save_state(state, {})

    progress = Progress()
    ok = motion.refine_motion(
        Bot(), state, {"id": "p1", "message_id": 40}, [], "video", progress, 495039871, request="наложи трек"
    )
    assert ok is False
    assert progress.failed and "трек" in progress.failed
    assert not calls["build"], "монтаж пошёл без трека"


def test_no_music_when_asked_without(wire, tmp_path):
    """«Без музыки» — ролик без звука, даже если трек есть."""
    calls, built, track = wire
    state = tmp_path / "state.json"
    st.save_state(state, {"music": {"path": str(track), "name": "track.mp3"}})

    progress = Progress()
    assert motion.refine_motion(
        Bot(), state, {"id": "p1", "message_id": 40}, [], "video", progress, 495039871, request="без музыки"
    ) is True
    assert calls["build"][0].get("music") is None


def test_new_track_changes_fingerprint():
    """Новый трек — другой ролик: отпечаток меняется."""
    plan = dict(PLAN)
    a = video.fingerprint(dict(plan, music="/tmp/a.mp3"), "mp4", CATALOG)
    b = video.fingerprint(dict(plan, music="/tmp/b.mp3"), "mp4", CATALOG)
    c = video.fingerprint(plan, "mp4", CATALOG)
    assert a != b and a != c
