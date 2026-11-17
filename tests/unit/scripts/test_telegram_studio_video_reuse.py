"""Студия: ролик не пересобирается заново и приходит отдельным сообщением.

Замечания Шифу (07.11):
- «каждый раз собирает видео заново» — сборка занимает ~50 с, а входы те же;
- «не может мне видео прислать» — правка старой картинки прячет ролик:
  Telegram кэширует превью сообщения, поэтому видео приходит новым сообщением.

Сеть, ssh/scp и ffmpeg — подставки: тест проверяет решения модулей, а не сеть.
"""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import studio_motion as motion  # noqa: E402
import studio_video as video  # noqa: E402

CATALOG = [
    {"id": "shisyun_s1", "label": "до", "kind": "poses"},
    {"id": "shisyun_s2", "label": "после", "kind": "poses"},
]
PLAN = {"template": "compare", "runs": ["shisyun_s1", "shisyun_s2"], "layout": "side_by_side", "orientation": "landscape"}


class Bot:
    """Подставка StudioBot: пишет вызовы Bot API по порядку."""

    def __init__(self, media_ok=True):
        self.calls = []
        self.media_ok = media_ok

    def send_media(self, chat_id, path, caption="", keyboard=None, above=False):
        self.calls.append(("sendMedia", str(path), caption))
        if not self.media_ok:
            return {"ok": False, "description": "Bad Request: failed to get HTTP URL content"}
        return {"ok": True, "result": {"message_id": 777}}

    def edit_message_media(self, chat_id, message_id, path, caption="", keyboard=None):
        self.calls.append(("editMessageMedia", str(path), message_id))
        return {"ok": True, "result": {"message_id": message_id}}


class Progress:
    """Подставка индикатора: только запоминает, что сказали."""

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


@pytest.fixture
def wire(monkeypatch, tmp_path):
    """Подставки вместо гейта, выбора шаблона, монтажа и записи в состояние."""
    calls = {"build": 0, "saved": None}
    built = tmp_path / "p1-side-by-side.mp4"
    built.write_bytes(b"x" * 1024)

    monkeypatch.setattr(motion.material, "runs_catalog", lambda gate: (list(CATALOG), ""))
    monkeypatch.setattr(motion.video, "choose_plan", lambda record, request, facts, catalog, fmt: (dict(PLAN), "план"))

    def fake_build(plan, fmt, ctx, catalog):
        calls["build"] += 1
        built.write_bytes(b"x" * 1024)
        return built, ""

    monkeypatch.setattr(motion.video, "build", fake_build)
    monkeypatch.setattr(motion.jobs, "update_draft", lambda bot, record, chat_id, new_media=None: (True, ""))
    monkeypatch.setattr(motion.jobs, "run_dir", lambda: tmp_path)

    def fake_save(state_path, post_id, fields):
        calls["saved"] = (post_id, fields)

    monkeypatch.setattr(motion.jobs, "save_fields", fake_save)
    return calls, built


def record_with_motion(built: Path, fingerprint: str) -> dict:
    """Черновик, у которого ролик уже собран."""
    return {
        "id": "p1",
        "message_id": 40,
        "message_kind": "media",
        "last_motion": {
            "template": "compare",
            "layout": "side_by_side",
            "orientation": "landscape",
            "runs": ["shisyun_s1", "shisyun_s2"],
            "format": "mp4",
            "bytes": built.stat().st_size,
            "build_seconds": 54.3,
            "path": str(built),
            "fingerprint": fingerprint,
        },
    }


# ------------------------------------------------------------------ отпечаток входов


def test_fingerprint_same_plan_is_same():
    a = video.fingerprint(PLAN, "mp4", CATALOG)
    b = video.fingerprint(dict(PLAN), "mp4", list(CATALOG))
    assert a == b


@pytest.mark.parametrize(
    "plan,fmt,catalog",
    [
        ({"template": "top_view", "runs": ["shisyun_s1", "shisyun_s2"]}, "mp4", CATALOG),
        ({"template": "compare", "runs": ["shisyun_s1"]}, "mp4", CATALOG),
        (PLAN, "gif", CATALOG),
        (PLAN, "mp4", [{"id": "shisyun_s1", "kind": "poses"}, {"id": "shisyun_s2", "kind": "frames"}]),
    ],
)
def test_fingerprint_changes_with_inputs(plan, fmt, catalog):
    assert video.fingerprint(plan, fmt, catalog) != video.fingerprint(PLAN, "mp4", CATALOG)


# ------------------------------------------------------------------ переиспользование


def test_second_press_reuses_built_video(wire):
    """Второе нажатие «Видео» не монтирует заново."""
    calls, built = wire
    fingerprint = video.fingerprint(PLAN, "mp4", CATALOG)
    record = record_with_motion(built, fingerprint)
    bot, progress = Bot(), Progress()

    assert motion.refine_motion(bot, Path("state.json"), record, [], "video", progress, 495039871) is True
    assert calls["build"] == 0, "ролик пересобирался заново"
    assert any("переиспольз" in note for note in progress.notes)
    assert ("sendMedia", str(built), ) in [(c[0], c[1]) for c in bot.calls]


def test_changed_material_rebuilds(wire):
    """Материал или план изменились — ролик собирается заново."""
    calls, built = wire
    other = video.fingerprint(PLAN, "mp4", [{"id": "shisyun_s1", "kind": "poses"}])
    record = record_with_motion(built, other)
    bot, progress = Bot(), Progress()

    assert motion.refine_motion(bot, Path("state.json"), record, [], "video", progress, 495039871) is True
    assert calls["build"] == 1


def test_request_pere_soberi_forces_rebuild(wire):
    """«пересобери» в просьбе — явная команда пересобрать."""
    calls, built = wire
    fingerprint = video.fingerprint(PLAN, "mp4", CATALOG)
    record = record_with_motion(built, fingerprint)
    bot, progress = Bot(), Progress()

    assert motion.refine_motion(bot, Path("state.json"), record, [], "video", progress, 495039871, request="пересобери") is True
    assert calls["build"] == 1


def test_missing_file_rebuilds(wire, tmp_path):
    """Файл ролика пропал — собираем заново."""
    calls, built = wire
    fingerprint = video.fingerprint(PLAN, "mp4", CATALOG)
    record = record_with_motion(built, fingerprint)
    built.unlink()
    bot, progress = Bot(), Progress()

    assert motion.refine_motion(bot, Path("state.json"), record, [], "video", progress, 495039871) is True
    assert calls["build"] == 1


# ------------------------------------------------------------------ доставка


def test_video_goes_as_new_message(wire):
    """Ролик приходит отдельным сообщением, а не только правкой черновика."""
    calls, built = wire
    bot, progress = Bot(), Progress()

    assert motion.refine_motion(bot, Path("state.json"), {"id": "p1", "message_id": 40}, [], "video", progress, 495039871) is True
    media_calls = [c for c in bot.calls if c[0] == "sendMedia"]
    assert media_calls, "ролик не отправлен отдельным сообщением"
    assert media_calls[0][1] == str(built)
    assert "видео" in media_calls[0][2]


def test_failed_upload_is_not_reported_as_done(wire):
    """Telegram не принял файл — это не «готово», и в состояние ничего не пишем."""
    calls, built = wire
    bot, progress = Bot(media_ok=False), Progress()

    assert motion.refine_motion(bot, Path("state.json"), {"id": "p1", "message_id": 40}, [], "video", progress, 495039871) is False
    assert progress.failed
    assert calls["saved"] is None


def test_state_remembers_path_and_fingerprint(wire):
    """В состоянии — путь и отпечаток: следующий раз ролик не пересобирается."""
    calls, built = wire
    bot, progress = Bot(), Progress()

    assert motion.refine_motion(bot, Path("state.json"), {"id": "p1", "message_id": 40}, [], "video", progress, 495039871) is True
    post_id, fields = calls["saved"]
    assert post_id == "p1"
    assert fields["last_motion"]["path"] == str(built)
    assert fields["last_motion"]["fingerprint"] == video.fingerprint(PLAN, "mp4", CATALOG)
    assert fields["media_path"] == str(built)
