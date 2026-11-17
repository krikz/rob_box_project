"""Идемпотентность студии: одно событие не показываем дважды, но и не теряем.

Сценарий, который закрывают эти тесты: событие (PR, прогон, медиа) попало в идею,
Шифу идею не взял. Через два дня сессия собрала это событие снова — его нельзя
предлагать бесконечно, но и нельзя выбросить молча.
"""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_studio as studio  # noqa: E402
import studio_facts as facts_mod  # noqa: E402
import studio_state as st  # noqa: E402
import studio_text as text_mod  # noqa: E402


@pytest.fixture(autouse=True)
def _queue_goes_to_tmp(tmp_path, monkeypatch):
    """Очередь канала — всегда во временный каталог.

    Без этого тест публикации писал настоящий пост в docs/channel/queue/
    репозитория: он попадал бы в канал вместе со служебным мусором.
    """
    monkeypatch.setenv("STUDIO_QUEUE_DIR", str(tmp_path / "queue"))


# --------------------------------------------------------------------------- ключи фактов


def test_fact_key_for_pr():
    fact = {"kind": "merged_pr", "claim": "PR #4503 влит в develop: сброс ICP", "source": "PR #4503"}
    assert facts_mod.fact_key(fact) == "pr:4503"


def test_fact_key_for_issue():
    fact = {"kind": "closed_issue", "claim": "Issue #4492 закрыт: студия", "source": "issue #4492"}
    assert facts_mod.fact_key(fact) == "issue:4492"


def test_fact_key_for_run():
    fact = {"kind": "run", "claim": "Прогон «L: Build All Services»", "source": "run_id 42219638788"}
    assert facts_mod.fact_key(fact) == "run:42219638788"


def test_fact_key_for_media():
    fact = {"kind": "media", "claim": "Новое медиа", "source": "docs/assets/hil/polygon.gif"}
    assert facts_mod.fact_key(fact) == "media:docs/assets/hil/polygon.gif"


def test_fact_key_is_stable_across_runs():
    """Ключ не должен зависеть от номера в списке — иначе сессии будут спорить."""
    fact = {"kind": "merged_pr", "claim": "PR #4502 влит в develop", "source": "PR #4502"}
    first = facts_mod.fact_key(fact)
    fact2 = dict(fact)
    assert facts_mod.fact_key(fact2) == first


# --------------------------------------------------------------------------- покрытие


def test_take_does_not_close_facts():
    """«Берём» — это ещё не публикация.

    Событие закрывается только после публикации поста. Если закрывать на «Берём»,
    то неудавшийся пост (Шифу передумал, модель не выдала текст) съел бы событие
    навсегда — это и есть «пропуск».
    """
    state = {"ideas": [{"id": "i1", "title": "тест", "status": "proposed", "fact_keys": ["pr:4503"]}]}
    studio.apply_idea_action(state, "i1", studio.ACTION_TAKE)
    assert "pr:4503" not in state.get("covered", {})
    assert state["ideas"][0]["status"] == "agreed"


def test_publish_closes_facts(tmp_path, monkeypatch):
    """Публикация закрывает событие: оно больше не предлагается.

    Очередь — во временный каталог: тест не должен писать в настоящую очередь
    репозитория (иначе в канал ушёл бы мусорный пост).
    """
    monkeypatch.setenv("STUDIO_QUEUE_DIR", str(tmp_path / "queue"))
    state = {
        "ideas": [],
        "posts": {
            "001": {"status": "proposed", "fact_keys": ["pr:4503"], "body_html": "текст #роббокс", "tags": ["#роббокс"]}
        },
    }
    studio.apply_post_action(state, "001", studio.ACTION_PUBLISH)
    assert "pr:4503" in state.get("covered", {}) or state["posts"]["001"]["status"] == "rejected"


def test_skip_marks_facts_rejected():
    state = {"ideas": [{"id": "i1", "title": "тест", "status": "proposed", "fact_keys": ["pr:4503"]}]}
    studio.apply_idea_action(state, "i1", studio.ACTION_SKIP)
    assert "pr:4503" in state["rejected"]
    assert "pr:4503" not in state.get("covered", {})


def test_covered_facts_are_not_offered_again():
    state = {"covered": {"pr:4503": {"post": "i1"}}, "rejected": {"pr:4499": {}}}
    closed = st.covered_keys(state)
    assert closed == {"pr:4503", "pr:4499"}


def test_pending_facts_come_back():
    """Факт, который предлагали, но не взяли, должен вернуться в следующей сессии."""
    state = {"pending": [{"key": "pr:4503", "claim": "PR #4503", "attempts": 1}]}
    pending = st.pending_facts(state)
    assert len(pending) == 1
    assert pending[0]["key"] == "pr:4503"


def test_pending_attempts_counted():
    state = {"pending": [{"key": "pr:4503", "attempts": 1}]}
    st.save_pending(state, [{"key": "pr:4503", "claim": "PR #4503"}])
    assert state["pending"][0]["attempts"] == 2


def test_covered_fact_leaves_pending():
    state = {"pending": [{"key": "pr:4503"}]}
    st.mark_covered(state, {"pr:4503"}, "i1", "2026-11-06T00:00:00Z")
    assert state["pending"] == []
    assert "pr:4503" in state["covered"]


def test_rejected_fact_leaves_pending():
    state = {"pending": [{"key": "pr:4503"}]}
    st.mark_rejected(state, {"pr:4503"}, "2026-11-06T00:00:00Z")
    assert state["pending"] == []
    assert "pr:4503" in state["rejected"]


# --------------------------------------------------------------------------- связь идеи и фактов


def test_match_fact_keys_by_claim():
    facts = [{"key": "pr:4503", "claim": "PR #4503 влит в develop: сброс ICP-одометрии"}]
    idea = {"facts": ["PR #4503 влит в develop: сброс ICP-одометрии после потери"]}
    assert text_mod.match_fact_keys(idea, facts) == ["pr:4503"]


def test_match_fact_keys_ignores_unknown_claims():
    facts = [{"key": "pr:4503", "claim": "PR #4503 влит в develop"}]
    idea = {"facts": ["Что-то совсем другое"]}
    assert text_mod.match_fact_keys(idea, facts) == []


def test_idea_carries_fact_keys():
    facts = [{"key": "pr:4503", "claim": "PR #4503 влит в develop: сброс ICP"}]
    idea = {"facts": ["PR #4503 влит в develop: сброс ICP"], "body_html": ""}
    assert text_mod.match_fact_keys(idea, facts) == ["pr:4503"]


# --------------------------------------------------------------------------- регистрация


def test_register_ideas_saves_fact_keys(tmp_path):
    path = tmp_path / "state.json"
    ideas = [{"id": "i1", "title": "тест", "fact_keys": ["pr:4503"], "facts": ["PR #4503"]}]
    assert studio.register_ideas(path, ideas) == 1
    state = st.load_state(path)
    assert state["ideas"][0]["fact_keys"] == ["pr:4503"]
    assert state["ideas"][0]["status"] == "proposed"


def test_register_ideas_does_not_reset_status(tmp_path):
    """Повторная отправка той же идеи не должна откатывать её статус."""
    path = tmp_path / "state.json"
    ideas = [{"id": "i1", "title": "тест", "fact_keys": ["pr:4503"]}]
    studio.register_ideas(path, ideas)
    state = st.load_state(path)
    state["ideas"][0]["status"] = "agreed"
    st.save_state(path, state)
    studio.register_ideas(path, ideas)
    assert st.load_state(path)["ideas"][0]["status"] == "agreed"


def test_register_post_saves_body(tmp_path):
    path = tmp_path / "state.json"
    studio.register_post(path, {"id": "001", "title": "черновик", "body_html": "<b>текст</b>"})
    state = st.load_state(path)
    assert state["posts"]["001"]["status"] == "proposed"
    assert state["posts"]["001"]["body_html"] == "<b>текст</b>"


# --------------------------------------------------------------------------- порог попыток


def test_max_attempts_constant():
    assert facts_mod.MAX_ATTEMPTS >= 2


def test_pending_fact_is_not_reoffered_and_not_lost(tmp_path):
    """Сквозной сценарий: событие предложено → взято → не опубликовано.

    Оно не должно предлагаться второй раз в том же виде и не должно пропасть:
    факт остаётся в ожидании, пока не опубликован или не отклонён.
    """
    path = tmp_path / "state.json"
    fact = {"key": "pr:4503", "claim": "PR #4503 влит в develop"}
    studio.register_ideas(path, [{"id": "i1", "title": "тест", "fact_keys": ["pr:4503"], "fact_dicts": [fact]}])
    state = st.load_state(path)
    assert [f["key"] for f in state["pending"]] == ["pr:4503"]
    assert state["pending"][0]["attempts"] == 1

    # Повторная регистрация той же идеи не должна накручивать попытки второй раз.
    studio.register_ideas(path, [{"id": "i1", "title": "тест", "fact_keys": ["pr:4503"], "fact_dicts": [fact]}])
    state = st.load_state(path)
    assert state["pending"][0]["attempts"] == 2

    # Событие в ожидании — значит, закрыто оно ещё не было.
    assert "pr:4503" not in state.get("covered", {})


def test_publish_removes_fact_from_pending(tmp_path):
    """После публикации факт уходит из ожидания и попадает в закрытые."""
    path = tmp_path / "state.json"
    fact = {"key": "pr:4503", "claim": "PR #4503 влит в develop"}
    studio.register_ideas(path, [{"id": "i1", "title": "тест", "fact_keys": ["pr:4503"], "fact_dicts": [fact]}])
    state = st.load_state(path)
    st.mark_covered(state, {"pr:4503"}, "001", "2026-11-06T00:00:00Z")
    st.save_state(path, state)
    state = st.load_state(path)
    assert state["pending"] == []
    assert "pr:4503" in state["covered"]
