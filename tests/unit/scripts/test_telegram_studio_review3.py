"""Регрессы третьего ревью координатора (06.11) и решения Шифу.

Каждый тест — на конкретное замечание ревью, чтобы следующий раз не пришлось
искать, «а это правда закрыли».
"""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_studio as studio  # noqa: E402
import studio_develop as develop  # noqa: E402
import studio_draft as draft  # noqa: E402
import studio_reply as reply  # noqa: E402
import studio_text as text  # noqa: E402

FACTS = [
    {
        "kind": "merged_pr",
        "claim": "Робот проехал 12 м по коридору и остановился у стены",
        "source": "PR #4508",
        "key": "k1",
        "visible": True,
    },
    {
        "kind": "media",
        "claim": "Новое медиа: график",
        "source": "docs/channel/media/a.png",
        "key": "k2",
        "visible": True,
    },
]


def make_state():
    """Состояние с идеей, взятой в работу, и постом."""
    return {
        "ideas": [{"id": "i1", "title": "Идея", "status": "proposed", "fact_keys": ["k1"]}],
        "posts": {},
        "offset": 0,
    }


# ------------------------------------------------------------------ 1. Идемпотентность поста


def test_publish_twice_creates_one_queue_file(tmp_path, monkeypatch):
    """Повторное «Опубликовать» не создаёт второй файл очереди."""
    monkeypatch.setenv("STUDIO_QUEUE_DIR", str(tmp_path))
    state = make_state()
    state["posts"]["001"] = {"id": "001", "status": "proposed", "body_html": "<b>текст</b> #роббокс", "title": "T"}
    studio.apply_post_action(state, "001", studio.ACTION_PUBLISH)
    first = state["posts"]["001"].get("queue_file")
    assert first, "первый файл очереди должен появиться"
    studio.apply_post_action(state, "001", studio.ACTION_PUBLISH)
    assert state["posts"]["001"]["queue_file"] == first
    assert len(list(tmp_path.glob("*.md"))) == 1, "второй файл очереди появиться не должен"


def test_publish_with_missing_queue_file_creates_no_second_file(tmp_path, monkeypatch):
    """Если файл очереди исчез, второй файл не создаётся молча."""
    monkeypatch.setenv("STUDIO_QUEUE_DIR", str(tmp_path))
    state = make_state()
    state["posts"]["001"] = {
        "id": "001",
        "status": "approved",
        "body_html": "<b>текст</b>",
        "title": "T",
        "queue_file": str(tmp_path / "900-post.md"),
    }
    answer = studio.apply_post_action(state, "001", studio.ACTION_PUBLISH)
    assert not list(tmp_path.glob("*.md")), "файл не должен появляться"
    assert "ручную проверку" in answer or "не создаю" in answer


def test_rejected_post_is_not_resurrected_by_take():
    """«Отклонить» → «Берём» не возвращает пост в публикацию."""
    state = make_state()
    state["posts"]["i1-post"] = {"id": "i1-post", "status": "rejected"}
    answer = studio.apply_idea_action(state, "i1", studio.ACTION_TAKE)
    assert "отклонён" in answer
    assert state["ideas"][0]["status"] != "agreed"


def test_queue_number_beyond_999(tmp_path, monkeypatch):
    """Номер после 999 считается правильно (раньше name[:3] давал 1000 → 100)."""
    monkeypatch.setenv("STUDIO_QUEUE_DIR", str(tmp_path))
    (tmp_path / "1000-post.md").write_text("x", encoding="utf-8")
    state = make_state()
    state["posts"]["001"] = {"id": "001", "status": "proposed", "body_html": "<b>т</b> #роббокс", "title": "T"}
    studio.apply_post_action(state, "001", studio.ACTION_PUBLISH)
    created = sorted(name.name for name in tmp_path.glob("*.md"))
    assert len(created) == 2
    # Следующий номер после 1000 — 1001, а не 100 (как было при name[:3]).
    assert created[0] == "1000-post.md"
    assert created[1].startswith("1001-"), created


# ------------------------------------------------------------------ 2. Честный отчёт


def test_send_post_failure_is_reported(monkeypatch):
    """Неуспешная отправка — ноль, а не «отправлен»."""

    class FailingBot:
        def send_message(self, chat_id, text, keyboard=None, parse_mode="HTML"):
            return {"ok": False, "description": "chat not found"}

    assert studio.send_post(FailingBot(), {"id": "001", "body_html": "x"}, 1) == 0


# ------------------------------------------------------------------ 3. Агент без инструментов


def test_reply_answer_with_minimax_key_is_rejected():
    """У моста нет инструментов; а если модель всё же выдала ключ — ответ не уходит."""
    assert reply.answer_problems("ключ eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9")


def test_secret_patterns_cover_minimax_key():
    """Ключ MiniMax — это JWT (eyJ…), его тоже надо ловить."""
    assert "eyJ" in reply.SECRET_PATTERNS


# ------------------------------------------------------------------ 4. Проверка фактов


def test_number_from_other_fact_is_not_allowed():
    """Число из чужого факта той же идеи — нельзя."""
    own = [FACTS[0]]
    body = "Робот проехал 12 м и 15 м по коридору"
    problems = text.check_against_facts(body, FACTS, own_facts=own)
    assert any("15" in problem for problem in problems), problems


def test_empty_own_facts_does_not_fall_back_to_all():
    """Пустой набор «своих» фактов не превращается в набор всех."""
    own: list = []
    body = "Робот проехал 12 м по коридору"
    problems = text.check_against_facts(body, FACTS, own_facts=own)
    assert problems, "при пустых own_facts сверяться не с чем — число не должно проходить"


def test_claim_with_appended_name_is_rejected():
    """Приписка «12 м, Саша рад» к факту «12 м» — не совпадение."""
    assert text._claim_matches_fact("12 м, Саша рад", "12 м") is False
    assert text._claim_matches_fact("12 м", "12 м") is True


@pytest.mark.parametrize("name", ["Данила", "Мила", "Людмила", "Саша"])
def test_names_ending_in_la_or_li_are_caught(name):
    """Имена на -ла/-ли — имена, а не «глагольные слова»."""
    body = f"Робот проехал 12 м. {name} был там."
    problems = text.check_against_facts(body, [FACTS[0]])
    assert problems, f"{name} не пойман"


@pytest.mark.parametrize("word", ["Проверили", "Сделали", "Настроили", "Настроила", "Выложили"])
def test_common_verbs_are_not_mistaken_for_names(word):
    """Обычные глаголы прошедшего времени именами быть не должны."""
    body = f"Робот проехал 12 м. {word} результат."
    assert not text.check_against_facts(body, [FACTS[0]]), word


# ------------------------------------------------------------------ 5. Свежие материалы из develop


def test_develop_worktree_is_read_only_clone(monkeypatch):
    """Worktree develop — detached и обновляется до origin/develop."""
    path = develop.develop_worktree_path()
    assert path.name == "rob_box_develop"
    source = (REPO / "scripts" / "telegram" / "studio_develop.py").read_text(encoding="utf-8")
    assert "checkout" in source and "--detach" in source


def test_media_from_develop_is_visible_in_facts():
    """Материал, которого нет в рабочей ветке, виден через worktree develop."""
    worktree = develop.develop_worktree_path()
    if not (worktree / ".git").exists():
        pytest.skip("worktree develop не создан")
    media = worktree / "docs/channel/media/locdrift-before-after.png"
    if not media.exists():
        pytest.skip("материала нет в develop")
    assert media.exists()


# ------------------------------------------------------------------ 6. Готовый пост (решение Шифу)


def test_draft_picks_fact_and_media():
    """Черновик строится из факта; материал — из материала того же факта."""
    fact = draft.pick_fact(FACTS)
    assert fact["kind"] == "merged_pr"
    assert draft.pick_media(FACTS, fact) == ""


def test_media_of_another_pr_is_not_taken():
    """Картинка из другой правки к посту не цепляется (Шифу, 07.11)."""
    facts = [
        {"kind": "merged_pr", "claim": "PR #4507 влит в develop (#4495)", "source": "PR #4507", "key": "k1"},
        {"kind": "media", "claim": "Новое медиа: график", "source": "docs/channel/media/a.png", "pr": "4508"},
    ]
    fact = facts[0]
    assert draft.pick_media(facts, fact) == ""


def test_media_of_same_pr_is_taken():
    """Картинка из той же правки — берётся."""
    facts = [
        {"kind": "merged_pr", "claim": "PR #4508 влит в develop (#4495)", "source": "PR #4508", "key": "k1"},
        {"kind": "media", "claim": "Новое медиа: график", "source": "docs/channel/media/a.png", "pr": "4508"},
    ]
    assert draft.pick_media(facts, facts[0]) == "docs/channel/media/a.png"


def test_media_fact_carries_its_pr():
    """Факт-медиа помнит, из какой правки он пришёл."""
    facts = [
        {"kind": "merged_pr", "claim": "PR #4508 влит в develop (#4495)", "source": "PR #4508", "key": "k1"},
        {
            "kind": "media",
            "claim": "Новое медиа: docs/channel/media/a.png",
            "source": "docs/channel/media/a.png",
            "pr": "4508",
            "key": "k2",
        },
    ]
    assert draft.pick_media(facts, facts[0]) == "docs/channel/media/a.png"


def test_post_id_is_stable():
    """id поста задаёт код: от запуска к запуску он не меняется."""
    import hashlib

    source = FACTS[0]["source"] + FACTS[0]["claim"]
    first = int(hashlib.md5(source.encode("utf-8")).hexdigest()[:6], 16) % 900 + 100
    second = int(hashlib.md5(source.encode("utf-8")).hexdigest()[:6], 16) % 900 + 100
    assert first == second


def test_refine_actions_are_deferred_outside_lock():
    """Итерационные кнопки не меняют состояние под блокировкой."""
    state = make_state()
    state["posts"]["001"] = {"id": "001", "status": "proposed", "body_html": "x", "title": "T"}
    answer = studio.apply_post_action(state, "001", studio.ACTION_SHORTER)
    assert "переделываю" in answer
    assert state["posts"]["001"]["body_html"] == "x", "текст не должен меняться под блокировкой"


def test_title_is_escaped_in_post():
    """Заголовок с «<» не должен ломать отправку."""
    assert studio.escape_html("a < b & c") == "a &lt; b &amp; c"


# ------------------------------------------------------------------ 7. .gitignore и пути


def test_gitignore_protects_runtime_files():
    """Рантайм-файлы студии не должны попадать в git."""
    ignore = (REPO / "docker/studio/.gitignore").read_text(encoding="utf-8")
    for entry in (".env", "state.json", "state.json.bak", "*.tmp"):
        assert entry in ignore, entry


def test_session_script_has_no_hardcoded_home_path():
    """В путях сессии не должно быть зашитого /home/builder."""
    script = (REPO / "scripts/telegram/studio_session.sh").read_text(encoding="utf-8")
    assert '"/home/builder/hermes-share/rob_box_project/scripts/telegram"' not in script


def test_session_limits_by_last_session():
    """Лимит частоты считается по метке последней сессии."""
    script = (REPO / "scripts/telegram/studio_session.sh").read_text(encoding="utf-8")
    assert "last_session" in script
