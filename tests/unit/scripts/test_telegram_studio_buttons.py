"""Кнопки студии: «Медиа» не уходит в себя, нажатие подтверждается сразу, шаги видны.

Разбор 06.11 (кнопка «Медиа» на черновике p20261106-129 молчала, воспроизведено на моках):
  1. опрос держал state_transaction весь long-poll и в finally писал старое состояние —
     черновик, записанный сессией за эти секунды, пропадал из state.json;
  2. apply_post_action создавал пустую запись {}, и переделка видела «поста нет»;
  3. результат refine_post никуда не уходил — Шифу видел только «Смотрю медиа.»;
  4. «Медиа» правила только текст (editMessageText) — картинка не появлялась никогда;
  5. переделка текста звала studio_text.py без --facts — argparse падал, правки не было.
"""

import json
import sys
import threading
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_studio as studio  # noqa: E402
import studio_jobs as jobs  # noqa: E402
import studio_media as smedia  # noqa: E402
import studio_progress as sp  # noqa: E402
import studio_state as st  # noqa: E402
import studio_text as text_mod  # noqa: E402

SHIFU = studio.SHIFU_CHAT_ID

FACTS = [
    {
        "kind": "merged_pr",
        "claim": "PR #4502 влит в develop: fix(nav): сторож локализации — не «приехал», если поза ушла (#4495)",
        "source": "PR #4502 (https://github.com/krikz/rob_box_project/pull/4502)",
        "merged_at": "2026-11-06T12:41:04Z",
        "key": "pr:4502",
    },
    {
        "kind": "merged_pr",
        "claim": "PR #4503 влит в develop: fix(rtabmap): сброс ICP-одометрии после потери (#4495)",
        "source": "PR #4503 (https://github.com/krikz/rob_box_project/pull/4503)",
        "merged_at": "2026-11-06T12:39:20Z",
        "key": "pr:4503",
    },
]


class Bot:
    """Подставка StudioBot: пишет вызовы Bot API по порядку."""

    def __init__(self, updates=None, poll_delay=0.0):
        self.updates = list(updates or [])
        self.poll_delay = poll_delay
        self.calls = []
        self._next_id = 100

    def _ok(self):
        self._next_id += 1
        return {"ok": True, "result": {"message_id": self._next_id}}

    def get_updates(self, offset, timeout=30):
        time.sleep(self.poll_delay)
        items, self.updates = self.updates, []
        return items

    def answer_callback(self, callback_id, text=""):
        self.calls.append(("answerCallbackQuery", text, time.monotonic()))
        return {"ok": True}

    def send_message(self, chat_id, text, keyboard=None, parse_mode="HTML", reply_to=None):
        self.calls.append(("sendMessage", text, keyboard))
        return self._ok()

    def edit_message_text(self, chat_id, message_id, text, keyboard=None, parse_mode="HTML"):
        self.calls.append(("editMessageText", text, message_id))
        return {"ok": True}

    def edit_message_caption(self, chat_id, message_id, caption, keyboard=None):
        self.calls.append(("editMessageCaption", caption, message_id))
        return {"ok": True}

    def edit_message_media(self, chat_id, message_id, path, caption="", keyboard=None):
        self.calls.append(("editMessageMedia", str(path), message_id, caption))
        return {"ok": True}

    def send_media(self, chat_id, path, caption="", keyboard=None, above=False):
        self.calls.append(("sendMedia", str(path), caption, keyboard))
        return self._ok()

    def send_chat_action(self, chat_id, action):
        self.calls.append(("sendChatAction", action))
        return {"ok": True}

    def names(self):
        return [c[0] for c in self.calls]


def _press(post_id, action, update_id=1):
    return {
        "update_id": update_id,
        "callback_query": {
            "id": f"cb{update_id}",
            "data": f"studio:post:{post_id}:{action}",
            "from": {"id": SHIFU},
            "message": {"chat": {"id": SHIFU}, "message_id": 77},
        },
    }


@pytest.fixture
def run_env(tmp_path, monkeypatch):
    run = tmp_path / "run"
    run.mkdir()
    (run / "facts.json").write_text(json.dumps({"visible_facts": FACTS}, ensure_ascii=False), encoding="utf-8")
    monkeypatch.setenv("STUDIO_TMP", str(run))
    monkeypatch.setenv("STUDIO_FACTS", str(run / "facts.json"))
    monkeypatch.setattr(sp, "MIN_INTERVAL", 0.0)
    monkeypatch.setattr(sp.Progress, "__init__", _fast_progress_init(sp.Progress.__init__))

    # Рендер инфографики требует шрифтов с кириллицей; в CI их может не быть.
    # Здесь проверяем поток кнопок, а сам рендер — отдельным тестом ниже.
    def fake_render(post, facts, out_dir):
        out_dir.mkdir(parents=True, exist_ok=True)
        out = out_dir / f"{post.get('id')}-timeline.png"
        out.write_bytes(b"png")
        return out, ""

    monkeypatch.setattr(smedia, "render_timeline", fake_render)
    return tmp_path


def _fast_progress_init(original):
    def init(self, *args, **kwargs):
        kwargs.setdefault("min_interval", 0.0)
        kwargs.setdefault("sleep", lambda _s: None)
        original(self, *args, **kwargs)

    return init


def _draft(path, **extra):
    record = {
        "id": "p1",
        "status": "proposed",
        "title": "Сторож",
        "body_html": "<b>Сторож</b> не врёт. #роббокс #навигация",
        "tags": ["#роббокс", "#навигация"],
        "fact_keys": ["pr:4502"],
        "message_id": 77,
        "message_kind": "text",
    }
    record.update(extra)
    studio.register_post(path, record)
    return record


# --------------------------------------------------------------------------- 1. потеря черновика


def test_draft_written_during_long_poll_is_not_lost(tmp_path):
    """Сессия пишет черновик, пока опрос ждёт getUpdates: запись обязана выжить."""
    path = tmp_path / "state.json"
    bot = Bot(poll_delay=0.5)
    poll = threading.Thread(target=studio.poll_once, args=(bot, path), kwargs={"timeout": 1})
    poll.start()
    time.sleep(0.1)
    studio.register_post(path, {"id": "p20261106-129", "title": "t", "body_html": "текст #роббокс #навигация"})
    poll.join()
    assert st.load_state(path)["posts"]["p20261106-129"]["body_html"].startswith("текст")


def test_refine_press_does_not_create_empty_record(tmp_path):
    state = {"posts": {}}
    studio.apply_post_action(state, "nope", studio.ACTION_MEDIA)
    assert "nope" not in state["posts"]


# --------------------------------------------------------------------------- 2. «Медиа»


def test_media_press_is_answered_then_media_replaced_in_place(run_env):
    path = run_env / "state.json"
    _draft(path)
    bot = Bot([_press("p1", "media")])
    studio.poll_once(bot, path, timeout=0, out=lambda _m: None)
    names = bot.names()
    assert names[0] == "answerCallbackQuery" and bot.calls[0][1] == "Делаю…"
    assert "sendChatAction" in names
    media_calls = [c for c in bot.calls if c[0] == "editMessageMedia"]
    assert media_calls and media_calls[0][2] == 77, "медиа меняется в том же сообщении"
    assert media_calls[0][1].endswith(".png")
    record = st.load_state(path)["posts"]["p1"]
    assert record["message_kind"] == "media" and record["media_choice"] == "infographic:timeline"
    progress_texts = [c[1] for c in bot.calls if c[0] in ("sendMessage", "editMessageText")]
    assert "Готово" in progress_texts[-1]
    assert "Рисую инфографику" in progress_texts[-1]


def test_media_press_without_draft_says_why(run_env):
    """Черновика нет — индикатор говорит это словами, а не молчит."""
    path = run_env / "state.json"
    bot = Bot([_press("ghost", "media")])
    studio.poll_once(bot, path, timeout=0, out=lambda _m: None)
    last = [c[1] for c in bot.calls if c[0] in ("sendMessage", "editMessageText")][-1]
    assert "Не вышло" in last and "черновика нет" in last


def test_media_failure_from_telegram_is_reported(run_env):
    path = run_env / "state.json"
    _draft(path)

    class Refusing(Bot):
        def edit_message_media(self, *a, **k):
            return {"ok": False, "description": "Bad Request: wrong file"}

    bot = Refusing([_press("p1", "media")])
    studio.poll_once(bot, path, timeout=0, out=lambda _m: None)
    last = [c[1] for c in bot.calls if c[0] in ("sendMessage", "editMessageText")][-1]
    assert "Меняю медиа" in last and "wrong file" in last


# --------------------------------------------------------------------------- 3. ответ без ожидания


def test_press_is_acknowledged_before_heavy_work_finishes(tmp_path, monkeypatch):
    path = tmp_path / "state.json"
    finished = threading.Event()

    def slow_job(*_args):
        time.sleep(1.0)
        finished.set()

    monkeypatch.setattr(jobs, "refine_job", slow_job)
    bot = Bot([_press("p1", "shorter")])
    runner = studio.JobRunner(threaded=True, out=lambda _m: None)
    started = time.monotonic()
    studio.poll_once(bot, path, timeout=0, out=lambda _m: None, runner=runner)
    elapsed = time.monotonic() - started
    assert bot.calls[0][0] == "answerCallbackQuery"
    assert bot.calls[0][2] - started < 0.5, "«Делаю…» — сразу, без ожидания работы"
    assert elapsed < 0.5, "опрос не ждёт тяжёлую работу"
    assert not finished.is_set()
    runner.join()
    assert finished.is_set()


def test_second_press_while_busy_gets_honest_answer(tmp_path, monkeypatch):
    path = tmp_path / "state.json"
    gate = threading.Event()
    monkeypatch.setattr(jobs, "refine_job", lambda *_a: gate.wait(2))
    runner = studio.JobRunner(threaded=True, out=lambda _m: None)
    bot = Bot([_press("p1", "shorter", 1), _press("p1", "simpler", 2)])
    studio.poll_once(bot, path, timeout=0, out=lambda _m: None, runner=runner)
    answers = [c[1] for c in bot.calls if c[0] == "answerCallbackQuery"]
    gate.set()
    runner.join()
    assert answers[0] == "Делаю…"
    assert "Уже делаю" in answers[1]
    assert st.load_state(path)["offset"] == 3


def test_watch_is_single_instance(tmp_path):
    lock = tmp_path / "poll.lock"
    first = studio._single_instance(lock)
    assert first is not None
    assert studio._single_instance(lock) is None, "второй получатель апдейтов дал бы 409"
    first.close()


# --------------------------------------------------------------------------- 4. индикатор


def test_progress_edits_one_message_and_throttles():
    bot = Bot()
    slept = []
    clock = iter([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]).__next__
    progress = sp.Progress(bot, SHIFU, "Тест", min_interval=1.5, clock=clock, sleep=slept.append)
    progress.step("Собираю факты")
    progress.step("Пишу текст")
    progress.finish()
    assert bot.names().count("sendMessage") == 1, "индикатор — одно сообщение"
    assert bot.names().count("editMessageText") == 2
    assert slept and all(0 < s <= 1.5 for s in slept), "правки не чаще раза в 1,5 с"
    assert "✓ Собираю факты" in bot.calls[-1][1] and "Готово" in bot.calls[-1][1]


def test_progress_failure_names_step_and_reason():
    bot = Bot()
    progress = sp.Progress(bot, SHIFU, "Тест", min_interval=0)
    progress.step("Пишу текст (MiniMax)")
    progress.fail("HTTP 500 <oops>")
    last = bot.calls[-1][1]
    assert "✗ Пишу текст (MiniMax)" in last
    assert "«Пишу текст (MiniMax)»" in last and "HTTP 500 &lt;oops&gt;" in last


def test_progress_why_goes_into_expandable_quote():
    bot = Bot()
    progress = sp.Progress(bot, SHIFU, "Тест", min_interval=0)
    progress.set_why("Взял PR #4502 <главный>")
    progress.finish()
    assert "<blockquote expandable>Почему так: Взял PR #4502 &lt;главный&gt;</blockquote>" in bot.calls[-1][1]


def test_why_with_invented_number_or_key_is_dropped():
    facts = [{"claim": "PR #4502 влит", "source": "PR #4502"}]
    assert text_mod.check_why("Подача через PR #4502", facts) == []
    assert any("12" in p for p in text_mod.check_why("Робот проехал 12 м", facts))
    assert any("секрет" in p or "ключ" in p for p in text_mod.check_why("eyJhbGciOiJIUzI1NiIsInR5cCI6", facts))
    assert text_mod.check_why("x" * 301, facts)


def test_fact_checks_count_numbers():
    checks = text_mod.fact_checks("<b>48 часов</b> и 3 попытки", ["число «3» не найдено среди фактов"])
    assert checks["numbers"] == 2 and checks["numbers_ok"] == 1


# --------------------------------------------------------------------------- 5. черновик с графикой


def test_send_post_with_media_uses_caption_and_keyboard(tmp_path):
    image = tmp_path / "a.png"
    image.write_bytes(b"png")
    bot = Bot()
    post = {"id": "p1", "body_html": "<b>Тезис</b> #роббокс #дневник", "media_path": str(image), "note": "тест"}
    assert studio.send_post(bot, post, SHIFU)
    name, path, caption, keyboard = bot.calls[0]
    assert name == "sendMedia" and path == str(image)
    assert caption.startswith("<i>Черновик p1 · тест</i>")
    assert keyboard["inline_keyboard"][0][0]["callback_data"] == "studio:post:p1:shorter"
    assert post["message_kind"] == "media"


def test_caption_header_dropped_when_it_would_overflow(tmp_path):
    image = tmp_path / "a.png"
    image.write_bytes(b"png")
    body = "я" * 1020
    caption = studio.draft_caption({"id": "p1", "body_html": body, "media_path": str(image)})
    assert caption == body


def test_update_draft_reports_not_modified_honestly():
    class Same(Bot):
        def edit_message_caption(self, *a, **k):
            return {"ok": False, "description": "Bad Request: message is not modified"}

    ok, reason = jobs.update_draft(Same(), {"id": "p1", "message_id": 5, "message_kind": "media"}, SHIFU)
    assert not ok and "совпала" in reason


def test_draft_job_sends_progress_then_draft_with_media(run_env, monkeypatch):
    import studio_draft

    monkeypatch.setattr(
        studio_draft,
        "build_post",
        lambda facts, **_kw: {
            "title": "Сторож",
            "body_html": "<b>Сторож</b> не врёт.\n<blockquote expandable>PR #4502</blockquote>\n#роббокс #навигация",
            "tags": ["#роббокс", "#навигация"],
            "media_ref": "",
            "fact_keys": ["pr:4502"],
            "source": "PR #4502",
            "why": "Взял PR #4502",
            "checks": {"numbers": 1, "numbers_ok": 1, "length": 60},
            "attempts": 1,
        },
    )
    path = run_env / "state.json"
    bot = Bot()
    assert jobs.draft_job(bot, path, run_env / "run" / "facts.json", chat_id=SHIFU, note="тест")
    media = [c for c in bot.calls if c[0] == "sendMedia"]
    assert media, "черновик уходит с картинкой"
    assert "<blockquote expandable>" in media[0][2]
    progress = [c[1] for c in bot.calls if c[0] in ("sendMessage", "editMessageText")][-1]
    for step in ("Читаю факты сессии", "Пишу текст", "Проверяю цифры: 1/1 ✓", "Рисую инфографику", "Готово"):
        assert step in progress, step
    record = next(iter(st.load_state(path)["posts"].values()))
    assert record["message_kind"] == "media" and record["message_id"]


# --------------------------------------------------------------------------- 6. медиа и Bot API


def test_timeline_events_come_from_merged_prs():
    events = smedia.timeline_events(FACTS, focus_keys=["pr:4503"])
    assert events[0]["source"] == "PR #4503"
    assert events[0]["date"] == "06.11"
    assert "(#4495)" not in events[0]["text"]


def test_render_without_font_is_a_reason_not_a_crash(tmp_path, monkeypatch):
    """Нет шрифта — make_channel_media делает SystemExit; работа студии падать не должна."""
    pytest.importorskip("PIL")
    import infographic

    def no_font(*_a, **_k):
        raise SystemExit("нет шрифта с кириллицей")

    monkeypatch.setattr(infographic, "render", no_font)
    path, reason = smedia.render_timeline({"id": "p1", "fact_keys": ["pr:4502"]}, FACTS, tmp_path)
    assert path is None and "шрифта" in reason


def test_media_options_cycle(tmp_path):
    post = {"id": "p1", "fact_keys": ["pr:4502"]}
    assert smedia.options(post, FACTS) == ["infographic:timeline"]
    assert smedia.next_option(dict(post, media_choice="infographic:timeline"), FACTS) is None


class _Recorder:
    def __init__(self):
        self.requests = []

    def post(self, url, json=None, data=None, files=None, timeout=None):
        self.requests.append({"url": url, "json": json, "data": data, "files": files})

        class _R:
            @staticmethod
            def json():
                return {"ok": True, "result": {"message_id": 5}}

        return _R()


def test_edit_message_media_uploads_with_attach(tmp_path):
    image = tmp_path / "a.png"
    image.write_bytes(b"png")
    session = _Recorder()
    bot = studio.StudioBot(token="x", session=session)
    assert bot.edit_message_media(SHIFU, 5, image, caption="<b>x</b>", keyboard={"inline_keyboard": []})["ok"]
    request = session.requests[0]
    assert request["url"].endswith("/editMessageMedia")
    media = json.loads(request["data"]["media"])
    assert media == {"type": "photo", "media": "attach://file", "caption": "<b>x</b>", "parse_mode": "HTML"}
    assert "file" in request["files"]


def test_retry_after_is_honoured(monkeypatch):
    class Flood:
        calls = 0

        def post(self, url, json=None, timeout=None, **_k):
            Flood.calls += 1
            payload = (
                {"ok": False, "error_code": 429, "parameters": {"retry_after": 7}}
                if Flood.calls == 1
                else {"ok": True, "result": True}
            )

            class _R:
                @staticmethod
                def json():
                    return payload

            return _R()

    waits = []
    monkeypatch.setattr("time.sleep", waits.append)
    studio.StudioBot(token="x", session=Flood()).send_chat_action(SHIFU, "typing")
    assert waits == [7]


# --------------------------------------------------------------------------- 7. развёртывание


def test_poll_cron_replaced_by_service():
    spec = json.loads((REPO / "docker/studio/cron.json").read_text(encoding="utf-8"))
    assert all(job["script"] != "studio_poll.sh" for job in spec["jobs"]), "кнопки — не крон"
    unit = (REPO / "docker/studio/studio-watch.service").read_text(encoding="utf-8")
    assert "Restart=always" in unit and " watch " in unit


def test_session_heredoc_gets_module_dir():
    script = (REPO / "scripts/telegram/studio_session.sh").read_text(encoding="utf-8")
    assert '"$STUDIO_REPO/docker/studio/state.json" "$STUDIO_REPO/scripts/telegram" <<' in script
    assert "draft --facts" in script


def test_develop_worktree_refuses_main_clone(tmp_path, monkeypatch):
    import studio_develop

    monkeypatch.setenv("STUDIO_DEVELOP_WORKTREE", str(tmp_path))
    (tmp_path / ".git").mkdir()
    calls = []
    monkeypatch.setattr(
        studio_develop, "_git", lambda args, cwd, timeout=180: calls.append(args) or type("D", (), {"returncode": 0})()
    )
    assert studio_develop.ensure_develop_worktree(tmp_path, out=lambda _m: None) is None
    assert not any(a[0] in ("reset", "clean", "checkout") for a in calls)


def test_post_id_for_is_stable():
    import studio_draft

    post = {"source": "PR #4502", "title": "Сторож"}
    assert studio_draft.post_id_for(post, "20261106") == studio_draft.post_id_for(dict(post), "20261106")


# --------------------------------------------------------------------------- 8. свободный текст Шифу


def test_active_draft_prefers_reply_then_freshest_and_does_not_guess():
    """Правило одно с studio_intent.target. 09.11: «самый большой message_id» выбрал не тот черновик."""
    now = time.time()
    state = {
        "posts": {
            "old": {"status": "proposed", "message_id": 10, "body_html": "a", "active_at": now - 60},
            "new": {"status": "proposed", "message_id": 20, "body_html": "b", "active_at": now - 5 * 3600},
            "done": {"status": "approved", "message_id": 30, "body_html": "c"},
        }
    }
    assert studio.active_draft(state, reply_to=20) == "new"
    assert studio.active_draft(state) == "old"  # только что тронутый, а не с бо́льшим message_id
    for record in state["posts"].values():
        record.pop("active_at", None)
    assert studio.active_draft(state) == ""  # неясно какой — не угадываем
    assert studio.active_draft({"posts": {}}) == ""


def test_free_text_edits_latest_draft_in_place(run_env, monkeypatch):
    """«Убери номера PR» в личке — правка черновика на месте, а не болтовня."""
    path = run_env / "state.json"
    _draft(path, message_kind="media", media_path=str(run_env / "a.png"))
    seen = {}

    def fake_produce(record, facts, instruction, shifu_rules=None):
        seen["instruction"] = instruction
        return {
            "body_html": "<b>Сторож</b> честный. #роббокс #навигация",
            "tags": ["#роббокс", "#навигация"],
            "problems": [],
            "ok": True,
            "why": "",
            "why_problems": [],
            "checks": {"numbers": 0, "numbers_ok": 0, "length": 30},
        }

    monkeypatch.setattr(jobs, "_produce", fake_produce)
    message = {
        "chat": {"id": SHIFU},
        "text": "Убери номера PR",
        "message_id": 90,
        "reply_to_message": {"message_id": 77},
    }
    bot = Bot([{"update_id": 5, "message": message}])
    studio.poll_once(bot, path, timeout=0, out=lambda _m: None)
    assert "Убери номера PR" in seen["instruction"]
    captions = [c for c in bot.calls if c[0] == "editMessageCaption"]
    assert captions and captions[0][2] == 77
    assert st.load_state(path)["posts"]["p1"]["body_html"].startswith("<b>Сторож</b> честный")


def test_free_text_without_reply_is_conversation_not_edit(run_env, monkeypatch):
    """«Ок, ищи данные» без reply не переписывает пост; ответ модели + подсказка от кода."""
    path = run_env / "state.json"
    _draft(path)
    monkeypatch.setattr(jobs, "_produce", lambda *a: pytest.fail("без reply правки быть не должно"))
    monkeypatch.setattr(studio, "run_reply_agent", lambda text, idea: "Нужен файл с замером.")
    bot = Bot([{"update_id": 6, "message": {"chat": {"id": SHIFU}, "text": "Ок, ищи данные", "message_id": 91}}])
    studio.poll_once(bot, path, timeout=0, out=lambda _m: None)
    final = [c[1] for c in bot.calls if c[0] in ("sendMessage", "editMessageText")][-1]
    assert final.startswith("Нужен файл с замером.")
    assert "пиши прямо сюда" in final and "p1" in final
