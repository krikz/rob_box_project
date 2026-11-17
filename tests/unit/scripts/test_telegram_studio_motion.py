"""Студия #4513: просьба без reply → действие, анимация и видео из готовых шаблонов, файлы от Шифу,
честный ответ на любое сообщение, причина неудачи в журнале и в state.posts[id].last_error.

Сеть, ssh/scp и ffmpeg — подставки: тест проверяет поведение модулей, а не живые хосты.
"""

import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))
sys.path.insert(0, str(REPO / "scripts" / "hil" / "video"))

import channel_studio as studio  # noqa: E402
import studio_inbox as inbox  # noqa: E402
import studio_intent as intent  # noqa: E402
import studio_jobs as jobs  # noqa: E402
import studio_material as material  # noqa: E402
import studio_motion as motion  # noqa: E402
import studio_progress as sp  # noqa: E402
import studio_render as render  # noqa: E402
import studio_reply as reply  # noqa: E402
import studio_state as st  # noqa: E402
import studio_video as video  # noqa: E402

SHIFU = studio.SHIFU_CHAT_ID


class Bot:
    """Подставка StudioBot: пишет вызовы Bot API по порядку."""

    def __init__(self, updates=None, files=None):
        self.updates = list(updates or [])
        self.files = files or {}
        self.calls = []
        self._next_id = 500

    def _ok(self):
        self._next_id += 1
        return {"ok": True, "result": {"message_id": self._next_id}}

    def get_updates(self, offset, timeout=30):
        items, self.updates = self.updates, []
        return items

    def answer_callback(self, callback_id, text=""):
        self.calls.append(("answerCallbackQuery", text))
        return {"ok": True}

    def send_message(self, chat_id, text, keyboard=None, parse_mode="HTML", reply_to=None):
        self.calls.append(("sendMessage", text, keyboard))
        return self._ok()

    def edit_message_text(self, chat_id, message_id, text, keyboard=None, parse_mode="HTML"):
        self.calls.append(("editMessageText", text, message_id, keyboard))
        return {"ok": True}

    def edit_message_caption(self, chat_id, message_id, caption, keyboard=None):
        self.calls.append(("editMessageCaption", caption, message_id))
        return {"ok": True}

    def edit_message_media(self, chat_id, message_id, path, caption="", keyboard=None):
        self.calls.append(("editMessageMedia", str(path), message_id, keyboard))
        return {"ok": True}

    def send_media(self, chat_id, path, caption="", keyboard=None, above=False):
        self.calls.append(("sendMedia", str(path), caption))
        return {"ok": True, "result": {"message_id": 601}}

    def send_chat_action(self, chat_id, action):
        self.calls.append(("sendChatAction", action))
        return {"ok": True}

    def get_file(self, file_id):
        self.calls.append(("getFile", file_id))
        if file_id not in self.files:
            return {"ok": False, "description": "Bad Request: file is too big"}
        return {"ok": True, "result": {"file_path": self.files[file_id]}}

    def download_file(self, file_path, dest):
        self.calls.append(("download", file_path))
        Path(dest).write_bytes(b"\x00" * 64)
        return ""

    def names(self):
        return [c[0] for c in self.calls]

    def texts(self):
        return [c[1] for c in self.calls if c[0] in ("sendMessage", "editMessageText")]


@pytest.fixture
def env(tmp_path, monkeypatch):
    run = tmp_path / "run"
    run.mkdir()
    (run / "facts.json").write_text(json.dumps({"visible_facts": []}), encoding="utf-8")
    monkeypatch.setenv("STUDIO_TMP", str(run))
    monkeypatch.setenv("STUDIO_FACTS", str(run / "facts.json"))
    monkeypatch.setattr(sp, "MIN_INTERVAL", 0.0)
    original = sp.Progress.__init__

    def fast(self, *args, **kwargs):
        kwargs.setdefault("min_interval", 0.0)
        kwargs.setdefault("sleep", lambda _s: None)
        original(self, *args, **kwargs)

    monkeypatch.setattr(sp.Progress, "__init__", fast)
    return tmp_path


def _draft(path, post_id="p1", message_id=77, **extra):
    record = {
        "id": post_id,
        "status": "proposed",
        # Пост про замер дрейфа локализации: ролик «до/после» — его тема.
        # Пост про конфиг параметров ролика не получает (Шифу, 07.11).
        "title": "локализация: сброс ICP-одометрии после потери",
        "body_html": "<b>Потеря опоры и дрейф локализации</b> (#4503) #роббокс #дневник",
        "tags": ["#роббокс", "#дневник"],
        "fact_keys": ["pr:4507"],
        "message_id": message_id,
        "message_kind": "media",
    }
    record.update(extra)
    studio.register_post(path, record)
    return record


def _message(text=None, update_id=1, reply_to=None, chat=SHIFU, **extra):
    message = {"chat": {"id": chat}, "message_id": 90 + update_id}
    if text is not None:
        message["text"] = text
    if reply_to:
        message["reply_to_message"] = {"message_id": reply_to}
    message.update(extra)
    return {"update_id": update_id, "message": message}


def _quiet(_m):
    return None


# --------------------------------------------------------------------------- 1. просьба без reply → действие


@pytest.mark.parametrize(
    "text, action",
    [
        ("Может сделать из него анимацию?", "animate"),
        ("сделай ролик до/после по дрейфу", "video"),
        # Живое сообщение Шифу 06.11 19:48 (journalctl studio-watch): ушло в правку текста, а не в ролик.
        ("Давай сделаем видос до после из сима", "video"),
        ("другая картинка", "media"),
        ("короче", "shorter"),
        ("Сделай попроще", "simpler"),
        ("Убери номера PR", "edit"),
        ("Короче, что там с дрейфом?", ""),
        ("Ок, ищи данные", ""),
        ("Как дела у робота?", ""),
    ],
)
def test_classify_is_code_not_model(text, action):
    assert intent.classify(text) == action


def test_target_single_draft_reply_named_and_ambiguous():
    now = 1_000_000.0
    state = {
        "posts": {
            "a": {"status": "proposed", "message_id": 10, "body_html": "x", "active_at": now - 60},
            "b": {"status": "proposed", "message_id": 20, "body_html": "y", "active_at": now - 120},
            "z": {"status": "approved", "message_id": 30, "body_html": "z"},
        }
    }
    assert intent.target(state, "", reply_to=20, now=now) == ("b", [])
    assert intent.target(state, "сделай b короче", now=now)[0] in ("b",)
    post_id, candidates = intent.target(state, "анимацию", now=now)
    assert post_id == "" and candidates == ["a", "b"], "два свежих черновика — переспросить"
    state["posts"]["b"]["active_at"] = now - 10 * 3600
    assert intent.target(state, "анимацию", now=now) == ("a", []), "свежий один — он"
    assert intent.target({"posts": {}}, "анимацию") == ("", [])


def test_animation_request_without_reply_runs_animate_on_active_draft(env, monkeypatch):
    """Разбор 06.11: «Может сделать из него анимацию?» без reply ушло в разговор — теперь это работа."""
    path = env / "state.json"
    _draft(path)
    seen = {}

    def fake_motion(bot, state_path, record, facts, action, progress, chat_id, request="", gate=None):
        seen.update(post=record["id"], action=action, request=request)
        progress.finish("Готово")
        return True

    monkeypatch.setattr(motion, "refine_motion", fake_motion)
    monkeypatch.setattr(studio, "reply_to_shifu", lambda *a, **k: pytest.fail("это не разговор"))
    bot = Bot([_message("Может сделать из него анимацию?")])
    assert studio.poll_once(bot, path, timeout=0, out=_quiet) == 1
    assert seen == {"post": "p1", "action": "animate", "request": "Может сделать из него анимацию?"}


def test_ambiguous_request_asks_with_buttons_then_press_runs_it(env, monkeypatch):
    path = env / "state.json"
    _draft(path, "p1", 77)
    _draft(path, "p2", 78)
    seen = {}

    def fake_motion(bot, state_path, record, facts, action, progress, chat_id, request="", gate=None):
        seen.update(post=record["id"], action=action, request=request)
        progress.finish("Готово")
        return True

    monkeypatch.setattr(motion, "refine_motion", fake_motion)
    bot = Bot([_message("сделай ролик до/после")])
    studio.poll_once(bot, path, timeout=0, out=_quiet)
    ask = [c for c in bot.calls if c[0] == "sendMessage"][-1]
    assert "К какому черновику" in ask[1]
    datas = [row[0]["callback_data"] for row in ask[2]["inline_keyboard"]]
    assert sorted(datas) == ["studio:post:p1:video", "studio:post:p2:video"]
    assert st.load_state(path)["pending_request"]["text"] == "сделай ролик до/после"

    press = {
        "update_id": 2,
        "callback_query": {
            "id": "cb",
            "data": "studio:post:p2:video",
            "from": {"id": SHIFU},
            "message": {"chat": {"id": SHIFU}, "message_id": 600},
        },
    }
    bot.updates = [press]
    studio.poll_once(bot, path, timeout=0, out=_quiet)
    assert seen == {"post": "p2", "action": "video", "request": "сделай ролик до/после"}
    assert "pending_request" not in st.load_state(path)


def test_action_without_any_draft_gets_honest_answer(env):
    path = env / "state.json"
    bot = Bot([_message("сделай анимацию")])
    studio.poll_once(bot, path, timeout=0, out=_quiet)
    assert any("черновика на обсуждении нет" in t for t in bot.texts())


def test_post_keyboard_has_animation_next_to_media():
    rows = studio.post_keyboard("p1")["inline_keyboard"]
    second = [b["text"] for b in rows[1]]
    assert second == ["Подача", "Медиа", "Анимация"]
    assert rows[1][2]["callback_data"] == "studio:post:p1:animate"
    assert studio.CALLBACK_RE.match("studio:post:p1:animate") and studio.CALLBACK_RE.match("studio:post:p1:video")


# --------------------------------------------------------------------------- мост: предложение кнопкой, не словами


def test_bridge_offer_becomes_a_button_built_by_code(env, monkeypatch):
    path = env / "state.json"
    _draft(path)
    answer = json.dumps({"answer": "Можно показать дрейф на записях прогонов.", "offer": "video"}, ensure_ascii=False)
    monkeypatch.setattr(studio, "run_reply_agent", lambda text, idea: answer)
    bot = Bot([_message("А что есть по дрейфу?")])
    studio.poll_once(bot, path, timeout=0, out=_quiet)
    final = [c for c in bot.calls if c[0] == "editMessageText"][-1]
    assert final[1].startswith("Можно показать дрейф")
    assert final[3]["inline_keyboard"][0][0]["callback_data"] == "studio:post:p1:video"


def test_bridge_drops_offer_outside_schema():
    assert studio.split_reply(json.dumps({"answer": "ок", "offer": "publish"})) == ("ок", "")
    assert reply.ask_model_full("p", call=lambda *_a, **_k: {"answer": "да", "offer": "rm -rf"}) == ("да", "")
    assert reply.ask_model_full("p", call=lambda *_a, **_k: {"answer": "да", "offer": "animate"}) == ("да", "animate")


# --------------------------------------------------------------------------- 4–5. файлы и любое сообщение


def test_video_with_caption_goes_into_draft(env):
    """Разбор 06.11: видео с подписью студия молча пропускала — подпись лежит в caption, не в text."""
    path = env / "state.json"
    _draft(path)
    video_msg = {"file_id": "F1", "file_size": 3 * 1024 * 1024, "file_name": "test clip.mp4", "mime_type": "video/mp4"}
    bot = Bot([_message(caption="вот ролик с полигона", video=video_msg)], files={"F1": "videos/file_1.mp4"})
    assert studio.poll_once(bot, path, timeout=0, out=_quiet) == 1
    assert "getFile" in bot.names()
    media = [c for c in bot.calls if c[0] == "editMessageMedia"]
    assert media and media[0][2] == 77, "файл встал медиа того же черновика"
    saved = Path(media[0][1])
    assert saved.parent == env / "run" / "media" / "inbox" and saved.suffix == ".mp4"
    progress = bot.texts()[-1]
    assert "использую в черновике p1" in progress and "Готово" in progress
    record = st.load_state(path)["posts"]["p1"]
    assert record["media_choice"].startswith("inbox:") and record["privacy_check"]
    assert st.load_state(path)["messages"][-1]["text"] == "вот ролик с полигона"


def test_file_over_bot_limit_gets_honest_answer(env):
    path = env / "state.json"
    _draft(path)
    big = {"file_id": "F2", "file_size": 25 * 1024 * 1024, "file_name": "big.mp4"}
    bot = Bot([_message(video=big)])
    studio.poll_once(bot, path, timeout=0, out=_quiet)
    assert "getFile" not in bot.names()
    assert "до 20.0 МБ" in bot.texts()[-1]
    assert "20.0 МБ" in st.load_state(path)["posts"]["p1"]["last_error"]["reason"]


def test_photo_takes_largest_size_and_document_kept():
    message = {
        "photo": [{"file_id": "s", "file_size": 10, "width": 90}, {"file_id": "L", "file_size": 900, "width": 1280}]
    }
    assert inbox.media_of(message)["file_id"] == "L"
    doc = inbox.media_of({"document": {"file_id": "d", "file_name": "план.pdf", "file_size": 5}})
    assert doc["kind"] == "document" and inbox.local_name(doc, 7, "documents/file.pdf") == "7-план.pdf"


def test_sticker_gets_honest_answer_not_silence(env):
    path = env / "state.json"
    bot = Bot([_message(sticker={"file_id": "st"})])
    assert studio.poll_once(bot, path, timeout=0, out=_quiet) == 1
    assert "стикер" in bot.texts()[-1] and "пока не умею" in bot.texts()[-1]


def test_files_from_strangers_are_not_taken(env):
    path = env / "state.json"
    _draft(path)
    bot = Bot([_message(chat=12345, video={"file_id": "F1", "file_size": 10})], files={"F1": "v.mp4"})
    assert studio.poll_once(bot, path, timeout=0, out=_quiet) == 0
    assert bot.calls == []


# --------------------------------------------------------------------------- 6. причина неудачи


def test_failure_reason_goes_to_journal_and_last_error(env, monkeypatch):
    path = env / "state.json"
    _draft(path)

    def failing(bot, state_path, record, facts, action, progress, chat_id, request="", gate=None):
        progress.step("Беру прогон shisyun_s1 с песочницы")
        progress.fail("scp: Permission denied")
        return False

    monkeypatch.setattr(motion, "refine_motion", failing)
    lines = []
    jobs.refine_job(Bot(), path, "p1", studio.ACTION_ANIMATE, out=lines.append)
    assert "не вышло" in lines[-1] and "scp: Permission denied" in lines[-1]
    error = st.load_state(path)["posts"]["p1"]["last_error"]
    assert error["reason"] == "scp: Permission denied" and error["action"] == "animate"
    assert error["step"].startswith("Беру прогон")


def test_crash_inside_job_is_reported_not_swallowed(env, monkeypatch):
    path = env / "state.json"
    _draft(path)
    monkeypatch.setattr(motion, "refine_motion", lambda *a, **k: 1 / 0)
    lines = []
    assert jobs.refine_job(Bot(), path, "p1", studio.ACTION_VIDEO, out=lines.append) is False
    assert "ZeroDivisionError" in st.load_state(path)["posts"]["p1"]["last_error"]["reason"]


# --------------------------------------------------------------------------- 2–3. план и монтаж

CATALOG = [
    {"id": "depot", "label": "склад Depot", "kind": "frames", "topic": ""},
    {"id": "shisyun_s1", "label": "ДО: прогон s1", "kind": "poses", "topic": "drift", "role": "before"},
    {"id": "shisyun_s2", "label": "ПОСЛЕ: прогон s2", "kind": "poses", "topic": "drift", "role": "after"},
]


def test_drift_post_gets_compare_by_code_rule():
    """Пост про сам замер дрейфа локализации — ролик «до/после»."""
    record = {"id": "p", "title": "локализация", "body_html": "сброс ICP-одометрии после потери (#4503)"}
    plan, why = video.choose_plan(record, "сделай анимацию", [], CATALOG, "gif", ask=lambda *a, **k: None)
    assert plan is not None
    assert plan["template"] == "compare" and plan["runs"] == ["shisyun_s1", "shisyun_s2"]
    assert "код" in why


def test_config_post_does_not_get_drift_video():
    """Пост про конфиг параметров — не про замер: ролик «до/после» ему не подходит.

    Шифу (07.11): «текст не соответствует видео» — к посту про конфиг висело
    видео с замером дрейфа.
    """
    record = {"id": "p", "title": "rtabmap", "body_html": "один источник параметров (#4495)"}
    plan, _why = video.choose_plan(record, "сделай анимацию", [], CATALOG, "gif", ask=lambda *a, **k: None)
    assert plan is None, "ролик про дрейф не должен подходить к посту про конфиг"


def test_vertical_request_picks_stacked_portrait():
    """Просьба «вертикальным, до сверху, после снизу» — стопка в вертикальном кадре."""
    record = {"id": "p", "title": "rtabmap", "body_html": "локализация (#4495)"}
    request = "сделай видео вертикальным чтобы до было сверху а после снизу"
    plan, _why = video.choose_plan(record, request, [], CATALOG, "mp4", ask=lambda *a, **k: None)
    assert plan["template"] == "compare"
    assert plan["layout"] == "stacked" and plan["orientation"] == "portrait"
    assert plan["runs"] == ["shisyun_s1", "shisyun_s2"]


def test_top_view_request_still_goes_to_top_view():
    """«Сверху» без «снизу» — по-прежнему вид сверху, а не стопка."""
    record = {"id": "p", "title": "rtabmap", "body_html": "локализация (#4495)"}
    plan, _why = video.choose_plan(record, "вид сверху на карту", [], CATALOG, "mp4", ask=lambda *a, **k: None)
    assert plan["template"] == "top_view"


def test_legacy_template_name_is_understood():
    """Старое имя шаблона из старых планов — это compare с боковой раскладкой."""
    plan = video.normalize_plan({"template": "side_by_side", "runs": ["a", "b"], "title": "t"})
    assert plan["template"] == "compare"
    assert plan["layout"] == "side_by_side" and plan["orientation"] == "landscape"


def test_model_plan_is_validated_against_catalog():
    record = {"id": "p", "title": "Светодиоды", "body_html": "матрица"}
    invented = {"template": "side_by_side", "runs": ["shisyun_s1", "s9"], "title": "x"}
    plan, why = video.choose_plan(record, "", [], CATALOG, "mp4", ask=lambda *a, **k: invented)
    assert plan is None and "не из каталога" in why
    good = {"template": "animated_card", "runs": ["shisyun_s1"], "title": "Ошибка позы"}
    plan, _why = video.choose_plan(record, "", [], CATALOG, "mp4", ask=lambda *a, **k: good)
    assert plan["template"] == "animated_card"
    bad = {"template": "fireworks", "runs": [], "title": "x"}
    assert "не из возможных" in video.choose_plan(record, "", [], CATALOG, "mp4", ask=lambda *a, **k: bad)[1]


def test_no_material_is_said_honestly():
    plan, why = video.choose_plan({"id": "p"}, "", [], [], "mp4")
    assert plan is None and "материала нет" in why


def _poses_csv(path: Path, drift: bool) -> None:
    lines = ["t,tx,ty,tyaw,ex,ey,eyaw", "10.0,0.0,0.0,0.0,,,"]
    for i in range(1, 60):
        x = i * 0.2
        est = x + (i * 0.1 if drift else 0.01)
        lines.append(f"{10 + i:.1f},{x:.3f},0.0,0.0,{est:.3f},0.0,0.0")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


class FakeGate:
    """Гейт без сети: ls по словарю, fetch пишет синтетический poses.csv."""

    def __init__(self):
        self.calls = 0
        self.fetched = []

    def ls(self, path):
        self.calls += 1
        if path == material.HIL_OUT:
            return ["route.log", "frames", "shisyun_s1", "shisyun_s2", "world.sdf"], ""
        if path.endswith("/maprec"):
            return ["poses.csv", "maps", "meta.json"], ""
        return [], ""

    def fetch(self, remote, local):
        self.calls += 1
        self.fetched.append(remote)
        Path(local).parent.mkdir(parents=True, exist_ok=True)
        _poses_csv(Path(local), drift="shisyun_s1" in remote)
        return ""


def test_animate_builds_gif_and_edits_same_draft(env, monkeypatch):
    pytest.importorskip("PIL")
    path = env / "state.json"
    _draft(path)
    gate = FakeGate()
    monkeypatch.setattr(material, "Gate", lambda: gate)
    bot = Bot()
    assert jobs.refine_job(bot, path, "p1", studio.ACTION_ANIMATE, out=_quiet, instruction="сделай анимацию")
    media = [c for c in bot.calls if c[0] == "editMessageMedia"]
    assert media and media[0][2] == 77 and media[0][1].endswith(".gif")
    assert Path(media[0][1]).stat().st_size <= render.GIF_LIMIT
    record = st.load_state(path)["posts"]["p1"]
    assert record["last_motion"]["template"] == "compare" and record["last_motion"]["format"] == "gif"
    assert any(r.endswith("shisyun_s1/maprec/poses.csv") for r in gate.fetched)
    progress = bot.texts()[-1]
    assert "Прогон shisyun_s1: 60 строк позы" in progress and "Готово: анимация в чате" in progress


class FakeProc:
    """ffmpeg-подставка: принимает кадры из stdin и пишет файл заданного размера."""

    sizes = []

    def __init__(self, cmd, stdin=None, stderr=None):
        self.out = Path(cmd[-1])
        self.frames = 0
        self.stdin = self
        self.stderr = None

    def write(self, data):
        self.frames += 1

    def close(self):
        size = FakeProc.sizes.pop(0) if FakeProc.sizes else 1000
        self.out.write_bytes(b"\x00" * size)

    def wait(self):
        return 0

    def kill(self):
        pass


def test_mp4_ladder_raises_crf_until_it_fits(tmp_path):
    pytest.importorskip("PIL")
    from PIL import Image

    FakeProc.sizes = [3000, 900]
    out = tmp_path / "a.mp4"

    def frames():
        return (Image.new("RGB", (64, 36)) for _ in range(5))

    path, reason, note = render.fit_mp4(frames, (64, 36), 10, out, limit=1000, popen=FakeProc)
    assert path == out and reason == "" and "CRF 28" in note
    FakeProc.sizes = [5000, 5000, 5000]
    path, reason, _note = render.fit_mp4(frames, (64, 36), 10, out, limit=1000, popen=FakeProc)
    assert path is None and "больше лимита" in reason


def test_video_action_builds_mp4_with_fake_ffmpeg(env, monkeypatch):
    pytest.importorskip("PIL")
    path = env / "state.json"
    _draft(path)
    monkeypatch.setattr(material, "Gate", lambda: FakeGate())
    monkeypatch.setattr(video, "MP4_SECONDS", 1)
    monkeypatch.setattr(video, "HOLD_SECONDS", 0)
    FakeProc.sizes = [2048]
    monkeypatch.setattr(video.Context, "__init__", _ctx_with_popen(video.Context.__init__, FakeProc))
    bot = Bot()
    assert jobs.refine_job(bot, path, "p1", studio.ACTION_VIDEO, out=_quiet, instruction="ролик до/после")
    media = [c for c in bot.calls if c[0] == "editMessageMedia"]
    assert media[0][1].endswith(".mp4")
    assert st.load_state(path)["posts"]["p1"]["last_motion"]["bytes"] == 2048


def _ffprobe_ok(cmd, **_kw):
    out = json.dumps({"streams": [{"width": 1280, "height": 720}], "format": {"duration": "33.0"}})
    return subprocess.CompletedProcess(cmd, 0, stdout=out, stderr="")


def _ctx_with_popen(original, popen):
    def init(self, *args, **kwargs):
        kwargs["popen"] = popen
        kwargs["run"] = _ffprobe_ok
        original(self, *args, **kwargs)

    return init


def test_mp4_self_check_rejects_empty_video(tmp_path):
    def broken(cmd, **_kw):
        return subprocess.CompletedProcess(cmd, 1, stdout="{}", stderr="moov atom not found")

    note, reason = render.check_mp4(tmp_path / "a.mp4", run=broken)
    assert note == "" and "moov atom not found" in reason
    note, reason = render.check_mp4(tmp_path / "a.mp4", run=_ffprobe_ok)
    assert reason == "" and "1280×720, 33.0 с" in note


# --------------------------------------------------------------------------- гейт и материал


def test_gate_uses_legacy_scp_and_studio_key(tmp_path):
    seen = []

    def runner(cmd, **_kw):
        seen.append(cmd)
        if cmd[0] == "scp":
            Path(cmd[-1]).write_text("t,tx\n", encoding="utf-8")
        return subprocess.CompletedProcess(cmd, 0, stdout="a\nb\n", stderr="")

    gate = material.Gate(key=Path("/k/studio_ro"), runner=runner)
    assert gate.ls("/x") == (["a", "b"], "")
    assert gate.fetch("/x/poses.csv", tmp_path / "p.csv") == ""
    scp = seen[-1]
    assert scp[:2] == ["scp", "-O"] and "/k/studio_ro" in scp and scp[-2].endswith(":/x/poses.csv")
    assert gate.fetch("/x/poses.csv", tmp_path / "p.csv") == "" and len(seen) == 2, "второй раз — из кэша"


def test_gate_refusal_is_a_reason():
    def runner(cmd, **_kw):
        return subprocess.CompletedProcess(cmd, 126, stdout="", stderr="studio_ro_gate: deny: path")

    names, reason = material.Gate(runner=runner).ls("/etc")
    assert names == [] and "deny" in reason


def test_catalog_marks_known_before_after_pair():
    catalog, reason = material.runs_catalog(FakeGate())
    assert reason == ""
    assert material.pair_for("drift", catalog) == ["shisyun_s1", "shisyun_s2"]
    assert any(item["kind"] == "frames" for item in catalog)
    assert material.sample(list(range(100)), 5) == [0, 25, 50, 74, 99]


# --------------------------------------------------------------------------- pose_anim (перенос hil_anim.py)


def test_pose_rows_and_summary(tmp_path):
    pytest.importorskip("PIL")
    import pose_anim

    csv_path = tmp_path / "poses.csv"
    _poses_csv(csv_path, drift=True)
    rows = pose_anim.load_poses_csv(csv_path)
    assert rows[0].error is None and rows[-1].error == pytest.approx(5.9, abs=0.01)
    stats = pose_anim.summary(rows)
    assert stats["max_error"] == pytest.approx(5.9, abs=0.01) and stats["with_estimate"] == 59
    assert pose_anim.meters(0.0774) == "0.08 м" and pose_anim.meters(16.54) == "16.5 м"


def test_map_layer_stays_inside_its_panel():
    """Уплывшая карта «до» шире мира прогона: клетки вне панели не рисуются (живая сборка 06.11)."""
    pytest.importorskip("PIL")
    import pose_anim

    view = pose_anim.Viewport((0.0, 0.0, 10.0, 10.0), (100, 100, 200, 200))
    data = [[100] * 40 for _ in range(40)]  # занято всё поле 40×40 м с началом в (-15, -15)
    snap = {"resolution": 1.0, "origin_x": -15.0, "origin_y": -15.0, "data": data}
    layer = pose_anim.map_layer(snap, view, (300, 300))
    x0, y0, x1, y1 = layer.getbbox()
    assert 100 <= x0 and 100 <= y0 and x1 <= 201 and y1 <= 201


def test_track_log_port_parses_hp2_lines(tmp_path):
    pytest.importorskip("PIL")
    import pose_anim

    log = tmp_path / "run.log"
    log.write_text(
        "10:00:00.0 TRACK gt=(0.10,0.20,90°) v=0 em=0\n"
        "10:00:01.5 TRACK gt=(0.50,0.20,45°) v=0 em=1\n"
        "RESULT STATUS_SUCCEEDED wall=12.5s\n",
        encoding="utf-8",
    )
    rows, result = pose_anim.load_track_log(log)
    assert [round(r.t, 1) for r in rows] == [0.0, 1.5] and rows[1].stop and result == "SUCCEEDED за 12.5 с"
    frames = list(pose_anim.track_frames(rows, "проём", result, size=(200, 160)))
    assert len(frames) == 2 and frames[0].size == (200, 160)


def test_top_view_picks_only_needed_frames():
    route = (
        "GOAL_START M1 target=(1,0,0) sim=100.0 frame=1000\n"
        "GOAL_END M1 result=STATUS_SUCCEEDED sim=200.0 frame=2000\n"
        "GOAL_START R1 target=(1,0,0) sim=210.0 frame=2100\n"
    )
    need = video.top_view_frames(route, fps=20, limit=50)
    assert need["t0"] == 100.0 and need["t1"] == 200.0
    assert len(need["frames"]) <= 50 and need["frames"][0] == 1000
