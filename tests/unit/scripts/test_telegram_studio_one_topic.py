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


def test_text_draft_recorded_as_media_is_edited_as_it_really_is(tmp_path):
    """Живая проверка 09.11: запись «media», а в Telegram текст — правка не падает, сообщение получает ролик."""
    clip = tmp_path / "clip.mp4"
    clip.write_bytes(b"mp4")

    class RealTextMessage(Bot):
        def edit_message_caption(self, chat_id, message_id, caption, keyboard=None):
            self.calls.append(("editMessageCaption", caption, message_id))
            return {"ok": False, "description": "Bad Request: there is no caption in the message to edit"}

    bot = RealTextMessage()
    record = {"id": "p1", "message_id": 77, "message_kind": "media", "media_path": str(clip), "body_html": "x"}
    assert jobs.update_draft(bot, record, SHIFU) == (True, "")
    assert [c for c in bot.calls if c[0] == "editMessageMedia" and c[1] == str(clip) and c[2] == 77]
    assert record["message_kind"] == "media"


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


# --------------------------------------------------------------------------- 5. «публикуем» словами (09.11)


def _two_drafts(now):
    """Как на builder 09.11: пост про навигацию только что показан, черновик про диалог — с ночи."""
    return {
        "posts": {
            "i20261106-1-post": {
                "status": "proposed",
                "title": "Навигация перестала считать потерю опоры обычным прибытием",
                "message_id": 116,
                "post_message_id": 128,
                "body_html": "x",
                "active_at": now - 5 * 60,
            },
            "p20261109-810": {
                "status": "proposed",
                "title": "Ответ тула больше не вытесняет историю из окна модели",
                "message_id": 124,
                "body_html": "y",
                "active_at": now - 5 * 3600,
            },
        }
    }


def test_publish_words_are_an_action_not_a_talk():
    assert intent.classify("Отлично! публикуем!") == intent.ACTION_PUBLISH
    assert intent.classify("опубликуй его") == intent.ACTION_PUBLISH
    assert intent.classify("пока не публикуй") == ""


def test_publish_after_shown_post_targets_that_post_and_asks_by_button():
    """«Отлично! публикуем!» после поста про навигацию — кнопка к нему, а не к черновику про диалог."""
    import time

    state = _two_drafts(time.time())
    decision = messages.decide(state, 1, {"chat": {"id": SHIFU}, "text": "Отлично! публикуем!", "message_id": 130})
    assert decision["kind"] == "confirm"
    assert decision["post_id"] == "i20261106-1-post"
    assert state["posts"]["i20261106-1-post"]["status"] == "proposed"  # словами не публикуем — только кнопкой


def test_talk_and_actions_pick_the_same_draft():
    """Одно правило «какой черновик»: разговор (active_draft) и действия (target) не расходятся."""
    import time

    state = _two_drafts(time.time())
    assert studio.active_draft(state) == intent.target(state)[0] == "i20261106-1-post"
    assert intent.target(state, reply_to=128) == ("i20261106-1-post", [])  # reply на присланный пост


def test_publish_confirmation_names_the_draft_and_carries_publish_button():
    bot = Bot()
    messages._confirm(bot, {"post_id": "i20261106-1-post", "title": "Навигация"})
    text, keyboard = bot.calls[0][1], bot.calls[0][2]
    assert "i20261106-1-post" in text and "Навигация" in text
    assert keyboard["inline_keyboard"][0][0]["callback_data"] == "studio:post:i20261106-1-post:publish"


# --------------------------------------------------------------------------- 6. таймлайн без кухни (09.11)


def _merged(number, title, day="06"):
    return {
        "kind": "merged_pr",
        "claim": f"PR #{number} влит в develop: {title}",
        "source": f"PR #{number} (https://github.com/krikz/rob_box_project/pull/{number})",
        "merged_at": f"2026-11-{day}T12:00:00Z",
        "key": f"pr:{number}",
    }


def test_timeline_picture_has_no_pr_numbers_or_develop():
    """Картинка идёт в канал: ни в заголовке, ни в строках, ни в подвале нет PR и develop."""
    facts = [
        _merged(4502, "fix(nav): сторож локализации — не «приехал», если поза ушла (#4495)"),
        _merged(4503, "fix(rtabmap): сброс ICP-одометрии после потери (#4495)"),
    ]
    spec = smedia.timeline_spec({"fact_keys": ["pr:4502"]}, facts)
    assert spec is not None
    shown = smedia.visible_text(spec)
    assert all(smedia.is_public(text) for text in shown), shown
    assert not any("PR" in text or "develop" in text for text in shown), shown
    assert [ev["pr"] for ev in spec["events"]] == ["#4502", "#4503"]  # сверка с источником остаётся


def test_timeline_line_with_repo_file_is_left_out():
    """«…— patch_rtabmap_launch.py» — имя файла, на картинку такая строка не идёт."""
    facts = [
        _merged(4507, "fix(rtabmap): один источник параметров — patch_rtabmap_launch.py"),
        _merged(4502, "fix(nav): сторож локализации — не «приехал», если поза ушла"),
    ]
    texts = [ev["text"] for ev in smedia.timeline_events(facts)]
    assert texts == ["Сторож локализации — не «приехал», если поза ушла"]


# --------------------------------------------------------------------------- 7. «где этот пост?» (09.11, 09:17 UTC)

WHERE_IS_POST = (
    "блин где этот пост?\nНавигация перестала считать потерю опоры обычным прибытием\n\n"
    "После потери опоры робот теперь сбрасывает ICP-одометрию с параметром Odom/ResetCountdown=1.\n\n"
    "ты же мне прислал его для просмотре с видосом уже где он????"
)


def test_where_is_the_post_is_send_post_not_new_video():
    """«…прислал его с видосом, где он?» — прислать пост, а не собирать новый ролик."""
    assert intent.classify(WHERE_IS_POST) == intent.ACTION_POST
    assert intent.classify("сделай видео до/после") == intent.ACTION_VIDEO


def test_quoted_post_text_beats_freshness():
    """Вставленный текст поста про навигацию — это он, хотя черновик про диалог тронут позже."""
    import time

    now = time.time()
    state = _two_drafts(now)
    state["posts"]["i20261106-1-post"]["active_at"] = now - 40 * 60
    state["posts"]["p20261109-810"]["active_at"] = now - 60
    assert intent.target(state, WHERE_IS_POST) == ("i20261106-1-post", [])
    assert intent.target(state, "так что с этим материалом он готов?")[0] == "p20261109-810"


def test_sent_post_carries_draft_buttons_and_becomes_the_draft_message(run_env):  # noqa: F811
    """Пост приходит с кнопками черновика; правки и reply идут к нему, старое сообщение тоже узнаётся."""
    path = run_env / "state.json"
    clip = run_env / "clip.mp4"
    clip.write_bytes(b"mp4")
    record = _draft(path, message_kind="media", media_path=str(clip))
    bot = Bot()
    progress = type("P", (), {"step": lambda *a, **k: None, "finish": lambda *a, **k: None, "fail": lambda *a: None})()
    assert jobs._refine_post(bot, path, record, progress, SHIFU)
    sent = [c for c in bot.calls if c[0] == "sendMedia"][0]
    assert any(
        b["callback_data"] == "studio:post:p1:publish" for row in sent[3]["inline_keyboard"] for b in row
    ), "под присланным постом есть «Опубликовать»"
    saved = st.load_state(path)["posts"]["p1"]
    assert saved["message_id"] != 77 and 77 in saved["old_message_ids"]
    assert intent.message_ids(saved) >= {77, saved["message_id"]}


# --------------------------------------------------------------------------- 8. время слота для Шифу (09.11)


def test_slot_is_told_in_gmt_plus_1_not_utc_hour():
    """09.11: «выйдет в слот 10.11 в 10:00 (по GMT+1)» — час по UTC с подписью GMT+1; верно 11:00."""
    from datetime import datetime, timezone

    assert lib.slot_text(datetime(2026, 11, 10, 10, 0, tzinfo=timezone.utc)) == "10.11 в 11:00 по GMT+1"


def test_publish_answer_names_the_real_slot_and_repeat_reads_it_from_queue(tmp_path, monkeypatch):
    """Ответ «одобрен» — по сроку файла очереди; повторное нажатие называет тот же срок, а не «ближайший сейчас»."""
    monkeypatch.setenv("STUDIO_QUEUE_DIR", str(tmp_path / "queue"))
    record = {
        "id": "p1",
        "status": "proposed",
        "title": "Навигация",
        "body_html": "<b>Навигация</b> честная. #роббокс #навигация",
        "tags": ["#роббокс", "#навигация"],
        "message_id": 77,
    }
    state = {"posts": {"p1": record}}
    first = studio.apply_post_action(state, "p1", studio.ACTION_PUBLISH)
    slot = studio.queued_slot(__import__("pathlib").Path(record["queue_file"]))
    assert slot.hour == 10 and lib.slot_text(slot).endswith("в 11:00 по GMT+1")
    assert lib.slot_text(slot) in first
    again = studio.apply_post_action(state, "p1", studio.ACTION_PUBLISH)
    assert "уже одобрен" in again and lib.slot_text(slot) in again
