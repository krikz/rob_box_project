"""Словарь хэштегов и разовое наполнение канала (--backfill)."""

import sys
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_lib as lib  # noqa: E402
import channel_post  # noqa: E402

NOW = datetime(2026, 11, 6, 12, 0, tzinfo=timezone.utc)


def write_post(
    queue, num, status="approved", tags="#роббокс #дневник", body_tags=None, not_before="2030-01-01T00:00:00Z"
):
    body_tags = tags if body_tags is None else body_tags
    head = f"status: {status}\nmedia:\nnot_before: {not_before}\nmessage_id:\nhashtags: {tags}"
    text = f"---\n{head}\n---\nПост {num}\n\n{body_tags}\n"
    path = queue / f"{num:03d}-p.md"
    path.write_text(text, encoding="utf-8")
    return path


def test_tags_must_be_in_dictionary_2_to_4_and_match_body(tmp_path):
    def problems(tags, body_tags=None):
        return lib.validate(lib.parse_post(write_post(tmp_path, 1, tags=tags, body_tags=body_tags)), tmp_path)

    assert problems("#роббокс #дневник") == []
    assert any("нет в словаре" in p for p in problems("#роббокс #выдуманный"))
    assert any("нужно 2–4" in p for p in problems("#роббокс"))
    assert any("нужно 2–4" in p for p in problems("#роббокс #дневник #цифры #ros2 #люди"))
    assert any("нет в hashtags" in p for p in problems("#роббокс #дневник", "#роббокс #дневник #лишний"))


def test_every_real_post_uses_dictionary_tags():
    for post in lib.load_queue(REPO / "docs" / "channel" / "queue"):
        assert 2 <= len(post.hashtags) <= 4, post.name
        assert set(post.hashtags) <= lib.TAG_DICTIONARY, post.name


def test_backfill_posts_all_approved_in_order_with_pause_ignoring_48h_and_not_before(tmp_path):
    for n in (1, 2, 3):
        write_post(tmp_path, n)
    write_post(tmp_path, 4, status="draft")
    sent, pauses = [], []

    def transport(method, params, media):
        sent.append(params["text"].split("\n")[0])
        return {"ok": True, "message_id": 100 + len(sent)}

    rc = channel_post.backfill(
        tmp_path, tmp_path, transport, False, sleep=pauses.append, clock=lambda: NOW, out=lambda s: None
    )
    assert rc == 0
    assert sent == ["Пост 1", "Пост 2", "Пост 3"]  # хронология, черновик не тронут
    assert pauses == [20.0, 20.0]  # пауза между постами, после последнего её нет
    posts = {p.name: p for p in lib.load_queue(tmp_path)}
    assert [posts[f"00{n}-p.md"].message_id for n in (1, 2, 3)] == [101, 102, 103]
    assert posts["004-p.md"].status == "draft"


def test_backfill_checks_everything_before_sending_anything(tmp_path):
    write_post(tmp_path, 1)
    write_post(tmp_path, 2, tags="#роббокс #выдуманный")
    out = []
    rc = channel_post.backfill(
        tmp_path, tmp_path, lambda *a: (_ for _ in ()).throw(AssertionError("сеть")), False, out=out.append
    )
    assert rc == 1 and any("002-p.md" in line for line in out)
    assert lib.parse_post(tmp_path / "001-p.md").status == "approved"


def test_backfill_stops_on_first_telegram_error_and_keeps_sent(tmp_path):
    for n in (1, 2, 3):
        write_post(tmp_path, n)
    answers = iter([{"ok": True, "message_id": 1}, {"ok": False, "description": "boom"}])
    rc = channel_post.backfill(
        tmp_path, tmp_path, lambda *a: next(answers), False, sleep=lambda s: None, out=lambda s: None
    )
    assert rc == 1
    statuses = [p.status for p in lib.load_queue(tmp_path)]
    assert statuses == ["posted", "approved", "approved"]


def test_backfill_dry_run_sends_and_sleeps_nothing(tmp_path):
    write_post(tmp_path, 1)
    write_post(tmp_path, 2)
    rc = channel_post.backfill(
        tmp_path,
        tmp_path,
        None,
        True,
        sleep=lambda s: (_ for _ in ()).throw(AssertionError("пауза")),
        out=lambda s: None,
    )
    assert rc == 0
    assert [p.status for p in lib.load_queue(tmp_path)] == ["approved", "approved"]
