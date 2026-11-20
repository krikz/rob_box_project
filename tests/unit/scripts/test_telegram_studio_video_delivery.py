"""Студия: кнопка «Видео» присылает ролик, индикатор не врёт, ответчик видит факты.

Разбор 14.11 (issue #4713). На боевом состоянии (`docker/studio/state.json` +
`docker/studio/run/facts.json`) кнопка «Видео» не отправляла ролик: текстом
просьбы становился произвольный вопрос Шифу («Так и где?»), `wants_resend()` давал
False, `delivered` совпадал с fingerprint — отправка пропускалась, а индикатор
писал «Готово: видео в чате». Ответчик при этом отвечал «нигде нет записей
заездов», хотя ролик по прогону `depot4682_20261111T191857Z` был отправлен
(message_id=162).

Три проверки:
  1) явная просьба после вопроса → ролик уходит в чат (файл переиспользуется,
     заново не монтируется);
  2) повтор той же просьбы без слов «пришли ещё раз» → отправки нет, и индикатор
     об этом говорит честно, а не «Готово: видео в чате»;
  3) промпт ответчика видит числа факта и run_id/message_id отправленного ролика.

Сеть, ssh/scp и ffmpeg — подставки: тест проверяет решения модулей, а не сеть.
"""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import studio_motion as motion  # noqa: E402
import studio_reply as reply  # noqa: E402
import studio_video as video  # noqa: E402

RUN = "depot4682_20261111T191857Z"
LONG = {"goals": 10, "sim_s": 1561, "map_snapshots": 1193, "pose_rows": 15340}

CATALOG = [{"id": RUN, "label": f"прогон {RUN}", "kind": "poses"}]
PLAN = {"template": "run_view", "layout": "side_by_side", "orientation": "landscape", "runs": [RUN]}


class Bot:
    """Подставка StudioBot: пишет вызовы Bot API по порядку."""

    def __init__(self):
        self.calls = []

    def send_media(self, chat_id, path, caption="", keyboard=None, above=False):
        self.calls.append(("sendMedia", str(path), caption))
        return {"ok": True, "result": {"message_id": 900}}

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
    built = tmp_path / "p20261114-559-run.mp4"
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


def delivered_record(built: Path, fingerprint: str, request: str) -> dict:
    """Черновик, которому ролик уже отправлен (как в боевом состоянии 14.11)."""
    return {
        "id": "p20261114-559",
        "message_id": 161,
        "message_kind": "media",
        "title": "Робот прошёл склад",
        "fact_keys": [f"robot_run:{RUN}"],
        "last_motion": {
            "template": "run_view",
            "layout": "side_by_side",
            "orientation": "landscape",
            "runs": [RUN],
            "format": "mp4",
            "bytes": built.stat().st_size,
            "build_seconds": 53.6,
            "path": str(built),
            "fingerprint": fingerprint,
            "delivered": fingerprint,
            "message_id": 162,
            "request": request,
        },
    }


# ------------------------------------------------------------------ доставка


def test_explicit_request_after_question_delivers_again(wire):
    """Просьба после вопроса — другой intent: ролик уходит в чат, даже если файл тот же."""
    calls, built = wire
    fingerprint = video.fingerprint(PLAN, "mp4", CATALOG)
    record = delivered_record(built, fingerprint, "Для видео возьми самый длинный заезд")
    bot, progress = Bot(), Progress()

    assert motion.refine_motion(bot, Path("state.json"), record, [], "video", progress, 495039871, request="Так и где?")
    assert ("sendMedia", str(built)) in [(c[0], c[1]) for c in bot.calls], "ролик не ушёл в чат"
    assert calls["build"] == 0, "ролик пересобирался заново"


def test_same_request_without_resend_words_does_not_send_again(wire):
    """Та же просьба без слов «пришли ещё раз» — второй раз молчим (жалоба 07.11)."""
    calls, built = wire
    fingerprint = video.fingerprint(PLAN, "mp4", CATALOG)
    record = delivered_record(built, fingerprint, "Так и где?")
    bot, progress = Bot(), Progress()

    assert motion.refine_motion(bot, Path("state.json"), record, [], "video", progress, 495039871, request="Так и где?")
    assert not [c for c in bot.calls if c[0] == "sendMedia"], "ролик ушёл без просьбы"
    assert "Готово: видео в чате" not in (progress.finished or ""), "индикатор врёт про доставку"
    assert "уже присылан" in (progress.finished or ""), "индикатор не сказал, что ролик уже присылан"


def test_resend_words_always_deliver(wire):
    """«Пришли ещё раз» — явная просьба: ролик уходит, даже если просьба та же самая."""
    calls, built = wire
    fingerprint = video.fingerprint(PLAN, "mp4", CATALOG)
    record = delivered_record(built, fingerprint, "Так и где?")
    bot, progress = Bot(), Progress()

    assert motion.refine_motion(
        bot, Path("state.json"), record, [], "video", progress, 495039871, request="пришли ещё раз"
    )
    assert ("sendMedia", str(built)) in [(c[0], c[1]) for c in bot.calls], "ролик не ушёл в чат"


def test_request_is_remembered_with_motion(wire):
    """Просьба, под которую ролик ушёл, запоминается рядом с роликом."""
    calls, built = wire
    fingerprint = video.fingerprint(PLAN, "mp4", CATALOG)
    record = delivered_record(built, fingerprint, "Для видео возьми самый длинный заезд")
    bot, progress = Bot(), Progress()

    motion.refine_motion(bot, Path("state.json"), record, [], "video", progress, 495039871, request="Так и где?")
    _, fields = calls["saved"]
    assert fields["last_motion"]["request"] == "Так и где?"


# ------------------------------------------------------------------ ответчик


def run_fact() -> dict:
    """Факт о прогоне — как его собирает studio_facts."""
    return {
        "key": f"robot_run:{RUN}",
        "kind": "robot_run",
        "run_id": RUN,
        "run_label": f"прогон {RUN}",
        "claim": "Робот прошёл склад: 10 целей маршрута, 1561 с симуляции (26 минут), 1193 снимка карты",
        "source": "журнал прогонов РОББОКС",
        "numbers": dict(LONG),
        "visible": True,
    }


def draft_state() -> dict:
    """Состояние студии с черновиком, которому ролик уже отправлен."""
    return {
        "offset": 0,
        "ideas": [],
        "posts": {
            "p20261114-559": {
                "id": "p20261114-559",
                "status": "proposed",
                "title": "Робот прошёл склад",
                "body_html": "<b>Робот прошёл склад</b>",
                "fact_keys": [f"robot_run:{RUN}"],
                "fact_dicts": [run_fact()],
                "last_motion": {
                    "template": "run_view",
                    "runs": [RUN],
                    "format": "mp4",
                    "bytes": 1067435,
                    "message_id": 162,
                },
            }
        },
        "messages": [{"role": "shifu", "text": "Так и где?"}],
    }


def test_reply_prompt_contains_facts_and_delivered_media(monkeypatch):
    """Промпт ответчика: числа факта и run_id/message_id уже отправленного ролика."""
    monkeypatch.setattr(reply, "session_facts", lambda: [run_fact()])
    state = draft_state()
    idea = reply.discussed_idea(state, "p20261114-559")
    prompt = reply.build_prompt("Так и где?", idea, state)

    assert "10 целей" in prompt, "в промпте нет чисел факта"
    assert RUN in prompt, "в промпте нет прогона отправленного ролика"
    assert "162" in prompt, "в промпте нет message_id отправленного ролика"


def test_reply_finds_draft_post_by_id():
    """Черновик лежит в posts, а не в ideas: ответчик должен найти его по id."""
    state = draft_state()
    idea = reply.discussed_idea(state, "p20261114-559")
    assert idea.get("id") == "p20261114-559"
    assert idea.get("body_html"), "черновик не найден по id"
    assert idea.get("last_motion"), "ответчик не видит, что ролик уже отправлен"
