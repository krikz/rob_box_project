"""Один пост — одна тема; правило Шифу запоминает код (разбор 07–09.11).

Что было по журналу studio-watch и состоянию builder:
  1. черновик p20261109-810 про диалог (#4555) получил график дрейфа из #4508 —
     черновик сессии брал любую картинку сессии (``studio_media.options``);
  2. «в постах не нужно писать про пиары, ветки и детали из репозитория и исью» —
     модель ответила «Приняли», не сохранилось ничего; следующий текст снова с ``fix(dialog)``;
  3. «наложи на это вертикальное видео звук» после «до сверху, после снизу» дал «рядом»;
  4. reply на ролик (он приходит отдельным сообщением) не считался reply на черновик.
"""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import channel_lib as lib  # noqa: E402
import channel_studio as studio  # noqa: E402
import studio_draft  # noqa: E402
import studio_intent as intent  # noqa: E402
import studio_jobs as jobs  # noqa: E402
import studio_media as smedia  # noqa: E402
import studio_messages as messages  # noqa: E402
import studio_reply  # noqa: E402
import studio_rules as rules  # noqa: E402
import studio_state as st  # noqa: E402
import studio_text as text_mod  # noqa: E402
import studio_video as video  # noqa: E402
from test_telegram_studio_buttons import Bot, _draft, run_env  # noqa: E402,F401

SHIFU = studio.SHIFU_CHAT_ID
RULE = "Так ещё момент в постах не нужно писать про пиары ветки и детали из репозитория и исью пока мы это не пишем"


def _session_facts(tmp_path):
    drift = tmp_path / "locdrift-before-after.png"
    drift.write_bytes(b"png")
    own = tmp_path / "dialog-window.png"
    own.write_bytes(b"png")
    return [
        {
            "kind": "merged_pr",
            "claim": "PR #4555 влит в develop: fix(dialog): ответ тула не вытесняет историю из окна модели",
            "source": "PR #4555 (https://github.com/krikz/rob_box_project/pull/4555)",
            "key": "pr:4555",
        },
        {"kind": "media", "claim": f"Новое медиа: {drift}", "source": str(drift), "pr": "4508", "key": "media:drift"},
        {"kind": "media", "claim": f"Новое медиа: {own}", "source": str(own), "pr": "4555", "key": "media:own"},
    ]


# --------------------------------------------------------------------------- 1. медиа поста — только своё


def test_session_draft_does_not_take_media_of_another_pr(tmp_path):
    """p20261109-810: пост про #4555 не получает график дрейфа из #4508."""
    facts = _session_facts(tmp_path)
    variants = smedia.options({"fact_keys": ["pr:4555"]}, facts)
    assert str(tmp_path / "locdrift-before-after.png") not in variants
    assert variants[0] == str(tmp_path / "dialog-window.png")


def test_stale_foreign_media_ref_of_old_draft_is_dropped(tmp_path):
    """Запись p20261109-810 уже хранит чужой media_ref (дрейф #4508) — кнопка «Медиа» его не вернёт."""
    facts = _session_facts(tmp_path)
    post = {"fact_keys": ["pr:4555"], "media_ref": str(tmp_path / "locdrift-before-after.png")}
    assert str(tmp_path / "locdrift-before-after.png") not in smedia.options(post, facts)


def test_post_without_own_media_gets_no_foreign_picture(tmp_path):
    """Своего медиа нет — чужое не подставляется (дальше только инфографика или без картинки)."""
    facts = [f for f in _session_facts(tmp_path) if f["key"] != "media:own"]
    assert [v for v in smedia.options({"fact_keys": ["pr:4555"]}, facts) if not v.startswith("infographic:")] == []


def test_pick_media_and_options_follow_one_rule(tmp_path):
    """Правило одно (studio_media.own_media): выбор темы сессии и кнопка «Медиа» не расходятся."""
    facts = _session_facts(tmp_path)
    assert studio_draft.pick_media(facts, facts[0]) == str(tmp_path / "dialog-window.png")
    drift_post = {"fact_keys": ["pr:4508"]}
    assert smedia.options(drift_post, facts)[0] == str(tmp_path / "locdrift-before-after.png")


# --------------------------------------------------------------------------- 2. внутренняя кухня


def test_reader_title_drops_pr_branch_and_commit_type():
    claim = "PR #4507 влит в develop: fix(rtabmap): один источник параметров — patch_rtabmap_launch.py"
    title = smedia.reader_title(claim)
    assert title.startswith("Один источник параметров")
    assert "PR" not in title and "develop" not in title and "fix(" not in title


def test_commit_type_and_develop_are_rejected_in_post():
    """p20261109-810 вышел с «<code>fix(dialog)</code>» — такой текст не проходит."""
    for body in ("Исправление относится к работе диалога: <code>fix(dialog)</code>.", "Изменение влито в develop."):
        problems = text_mod.validate_post(body + " #роббокс #дневник", ["#роббокс", "#дневник"], [])
        assert any("в тексте" in p for p in problems), body


def test_chat_reply_and_why_may_name_pr():
    """Правило — про текст поста: в личке и в «почему так» номер PR нужен, чтобы понять, о чём речь."""
    assert all(item not in studio_reply.FORBIDDEN for item in lib.FORBIDDEN_INTERNAL)
    problems = text_mod.check_why("Подача по сторожу локализации из PR #4502", [])
    assert all("номер PR" not in p for p in problems), problems


def test_post_prompt_does_not_ask_for_pr_link():
    """Задание модели больше не требует ссылку на PR (раньше требовало — и проверка её же отбивала)."""
    prompt = text_mod.build_post_prompt({"title": "t", "show": "s"}, [], "правила")
    assert "pull/N" not in prompt
    assert "PR #N" not in prompt


def test_channel_rules_doc_does_not_allow_pr_numbers():
    """Правила канала идут модели целиком: в них не должно быть «номера PR можно»."""
    doc = (REPO / "docs" / "process" / "telegram-channel.md").read_text(encoding="utf-8")
    assert "Номера карточек и PR можно" not in doc
    assert "ссылка на PR | промпт поста" not in doc


# --------------------------------------------------------------------------- 3. правило Шифу


def test_rule_message_is_stored_by_code_and_answered_by_code():
    state = {"posts": {}}
    decision = messages.decide(state, 1, {"chat": {"id": SHIFU}, "text": RULE, "message_id": 5})
    assert decision["kind"] == "rule"
    assert rules.active(state) == [" ".join(RULE.split())]
    assert decision["answer"].startswith("Записал правило к постам")
    assert "нет" in decision["answer"]  # черновика нет — так и сказано


def test_same_rule_twice_is_not_duplicated():
    state = {"posts": {}}
    messages.decide(state, 1, {"chat": {"id": SHIFU}, "text": RULE, "message_id": 5})
    again = messages.decide(state, 2, {"chat": {"id": SHIFU}, "text": RULE, "message_id": 6})
    assert len(rules.active(state)) == 1
    assert again["answer"].startswith("Это правило уже записано")


def test_rule_goes_into_post_prompt():
    prompt = text_mod.build_post_prompt({"title": "t", "show": "s", "shifu_rules": ["не пиши про PR"]}, [], "")
    assert "ПРАВИЛА ШИФУ К ПОСТАМ" in prompt and "- не пиши про PR" in prompt


def test_rule_rewrites_the_draft_with_rule_in_the_prompt(run_env, monkeypatch):  # noqa: F811
    """Правило сохранено, черновик на обсуждении переписан, правило дошло до задания модели."""
    path = run_env / "state.json"
    _draft(path, message_kind="media", media_path=str(run_env / "a.png"))
    seen = {}

    def fake_produce(record, facts, instruction, shifu_rules=None):
        seen["instruction"], seen["rules"] = instruction, list(shifu_rules or [])
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
    bot = Bot([{"update_id": 5, "message": {"chat": {"id": SHIFU}, "text": RULE, "message_id": 90}}])
    studio.poll_once(bot, path, timeout=0, out=lambda _m: None)
    said = [c[1] for c in bot.calls if c[0] == "sendMessage"]
    assert any(text.startswith("Записал правило к постам") and "p1" in text for text in said)
    assert seen["rules"] == [" ".join(RULE.split())]
    assert "пиары" in seen["instruction"]
    assert [c for c in bot.calls if c[0] == "editMessageCaption" and c[2] == 77]
    assert st.load_state(path)["rules"][0]["text"] == " ".join(RULE.split())


# --------------------------------------------------------------------------- 4. раскладка и reply на ролик


def test_layout_is_kept_until_shifu_says_otherwise():
    record = {"last_motion": {"layout": "stacked", "orientation": "portrait"}}
    assert video.remembered_layout(record, "наложи на это видео звук красиво", False, False) == ("stacked", "portrait")
    assert video.remembered_layout(record, "давай рядом и горизонтально", False, False) == (
        "side_by_side",
        "landscape",
    )
    assert video.remembered_layout({}, "видео", False, False) == ("side_by_side", "landscape")


def test_reply_to_the_video_is_reply_to_the_draft():
    record = {"status": "proposed", "message_id": 77, "body_html": "x", "last_motion": {"message_id": 106}}
    state = {"posts": {"p1": record, "p2": dict(record, message_id=78, last_motion={})}}
    assert intent.target(state, "оформи текст", reply_to=106) == ("p1", [])
