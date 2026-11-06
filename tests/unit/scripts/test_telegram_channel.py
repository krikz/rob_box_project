"""Планировщик и оформление канала @robbox_project: выбор поста, интервал 48 ч, запись результата. Сеть замокана."""

import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_lib as lib  # noqa: E402
import channel_post  # noqa: E402
import channel_setup  # noqa: E402

UTC = timezone.utc
T0 = datetime(2026, 11, 10, 10, 0, tzinfo=UTC)


def make_post(queue: Path, num: int, status="approved", not_before="2026-11-10T10:00:00Z", media="", **extra):
    meta = {
        "status": status,
        "media": media,
        "not_before": not_before,
        "message_id": "",
        "hashtags": "#роббокс #дневник",
    }
    meta.update(extra)
    head = "\n".join(f"{k}: {v}" for k, v in meta.items())
    path = queue / f"{num:03d}-p.md"
    path.write_text(f"---\n{head}\n---\n<b>Пост {num}</b> #роббокс #дневник\n", encoding="utf-8")
    return path


@pytest.fixture
def queue(tmp_path):
    q = tmp_path / "queue"
    q.mkdir()
    return q


def test_picks_first_approved_whose_time_has_come(queue):
    make_post(queue, 1, status="draft")
    make_post(queue, 2, not_before="2026-11-20T10:00:00Z")
    make_post(queue, 3)
    make_post(queue, 4)
    assert lib.pick_next(lib.load_queue(queue), T0).name == "003-p.md"


def test_nothing_before_not_before(queue):
    make_post(queue, 1)
    assert lib.pick_next(lib.load_queue(queue), T0 - timedelta(minutes=1)) is None


def test_drafts_are_never_picked(queue):
    make_post(queue, 1, status="draft")
    assert lib.pick_next(lib.load_queue(queue), T0 + timedelta(days=30)) is None


def test_interval_48h_since_last_post(queue):
    make_post(queue, 1, status="posted", message_id="7", posted_at=T0.isoformat())
    make_post(queue, 2)
    posts = lib.load_queue(queue)
    assert lib.pick_next(posts, T0 + timedelta(hours=47, minutes=59)) is None
    assert lib.pick_next(posts, T0 + timedelta(hours=48)).name == "002-p.md"


def test_interval_uses_latest_posted(queue):
    make_post(queue, 1, status="posted", message_id="1", posted_at=(T0 - timedelta(days=5)).isoformat())
    make_post(queue, 2, status="posted", message_id="2", posted_at=(T0 - timedelta(hours=3)).isoformat())
    make_post(queue, 3)
    assert lib.pick_next(lib.load_queue(queue), T0) is None


def test_bad_status_is_an_error(queue):
    make_post(queue, 1, status="maybe")
    with pytest.raises(ValueError):
        lib.load_queue(queue)


def test_publish_text_marks_posted(queue, tmp_path):
    path = make_post(queue, 1)
    calls = []

    def transport(method, params, media):
        calls.append((method, params, media))
        return {"ok": True, "message_id": 42}

    out = []
    assert channel_post.run(queue, tmp_path, T0, transport, dry_run=False, out=out.append) == 0
    assert calls[0][0] == "sendMessage" and calls[0][1]["chat_id"] == lib.CHANNEL_ID
    post = lib.parse_post(path)
    assert (post.status, post.message_id, post.posted_at) == ("posted", 42, T0)


def test_publish_gif_uses_send_animation_and_photo_uses_send_photo(queue, tmp_path):
    (tmp_path / "a.gif").write_bytes(b"GIF89a")
    (tmp_path / "b.png").write_bytes(b"\x89PNG")
    make_post(queue, 1, media="a.gif")
    make_post(queue, 2, media="b.png")
    posts = lib.load_queue(queue)
    assert lib.build_request(posts[0], tmp_path)[0] == "sendAnimation"
    assert lib.build_request(posts[1], tmp_path)[0] == "sendPhoto"
    assert lib.build_request(posts[0], tmp_path)[1]["caption"].startswith("<b>Пост 1")


def test_dry_run_sends_nothing_and_writes_nothing(queue, tmp_path):
    path = make_post(queue, 1)
    before = path.read_text(encoding="utf-8")

    def boom(*a):
        raise AssertionError("сеть в dry-run")

    out = []
    assert channel_post.run(queue, tmp_path, T0, boom, dry_run=True, out=out.append) == 0
    assert path.read_text(encoding="utf-8") == before
    assert any("DRY-RUN" in line for line in out)


def test_telegram_refusal_leaves_post_approved(queue, tmp_path):
    path = make_post(queue, 1)
    rc = channel_post.run(
        queue, tmp_path, T0, lambda *a: {"ok": False, "description": "nope"}, False, out=lambda s: None
    )
    assert rc == 1 and lib.parse_post(path).status == "approved"


def test_validate_blocks_internal_ip_and_missing_media(queue, tmp_path):
    path = make_post(queue, 1, media="nope.gif")
    path.write_text(path.read_text(encoding="utf-8") + "робот на 10.1.1.22\n", encoding="utf-8")
    problems = lib.validate(lib.parse_post(path), tmp_path)
    assert any("IP" in p for p in problems) and any("нет файла" in p for p in problems)
    rc = channel_post.run(queue, tmp_path, T0, lambda *a: {"ok": True, "message_id": 1}, False, out=lambda s: None)
    assert rc == 1 and lib.parse_post(path).status == "approved"


def test_validate_caption_limit_and_html(queue, tmp_path):
    (tmp_path / "a.gif").write_bytes(b"GIF89a")
    path = make_post(queue, 1, media="a.gif")
    path.write_text(path.read_text(encoding="utf-8") + "x" * 1100 + " <div>\n", encoding="utf-8")
    problems = lib.validate(lib.parse_post(path), tmp_path)
    assert any("лимит" in p for p in problems) and any("div" in p for p in problems)


def test_real_queue_is_valid_and_unfinished_posts_stay_draft():
    posts = lib.load_queue(REPO / "docs" / "channel" / "queue")
    assert 8 <= len(posts) <= 11
    for post in posts:
        if post.pending:  # пост с заглушкой медиа нельзя одобрять
            assert post.status == "draft", post.name
        problems = lib.validate(post, REPO)
        if post.pending:  # заглушка медиа: единственное, что допустимо — «не готово» и отсутствие файла
            assert all("не готово" in p or "нет файла" in p for p in problems), (post.name, problems)
            assert any("не готово" in p for p in problems), post.name
        else:
            assert problems == [], post.name
    gaps = [b.not_before - a.not_before for a, b in zip(posts, posts[1:])]
    assert all(g >= lib.MIN_GAP for g in gaps)
    assert all(p.not_before.weekday() in (1, 3, 5) and p.not_before.hour == 10 for p in posts)  # вт/чт/сб 11:00 GMT+1


def test_setup_dry_run_steps_and_limits(capsys):
    assert channel_setup.run(REPO, None, dry_run=True) == 0
    out = capsys.readouterr().out
    for step in ("setChatPhoto", "setChatTitle", "setChatDescription", "sendAnimation", "pinChatMessage"):
        assert step in out
    profile = channel_setup.read_profile(REPO)
    assert len(profile["description"]) <= 255


def test_setup_refuses_live_without_approved_pin(tmp_path):
    import shutil

    shutil.copytree(REPO / "docs" / "channel", tmp_path / "docs" / "channel")
    shutil.copytree(REPO / "docs" / "assets" / "animations", tmp_path / "docs" / "assets" / "animations")
    pin = tmp_path / "docs" / "channel" / "queue" / "001-hello-pin.md"
    lib.set_front_matter(pin, status="draft")
    rc = channel_setup.run(tmp_path, lambda *a: pytest.fail("сеть без одобрения"), dry_run=False, out=lambda s: None)
    assert rc == 1


def test_setup_live_runs_all_steps_and_marks_pin_posted(tmp_path):
    import shutil

    shutil.copytree(REPO / "docs" / "channel", tmp_path / "docs" / "channel")
    shutil.copytree(REPO / "docs" / "assets" / "animations", tmp_path / "docs" / "assets" / "animations")
    pin = tmp_path / "docs" / "channel" / "queue" / "001-hello-pin.md"
    lib.set_front_matter(pin, status="approved", message_id="")
    calls = []

    def transport(method, params, media):
        calls.append((method, params))
        return {"ok": True, "message_id": 77}

    assert channel_setup.run(tmp_path, transport, dry_run=False, out=lambda s: None) == 0
    assert [c[0] for c in calls] == [
        "setChatPhoto",
        "setChatTitle",
        "setChatDescription",
        "sendAnimation",
        "pinChatMessage",
    ]
    assert calls[-1][1]["message_id"] == 77
    pin = lib.parse_post(tmp_path / "docs" / "channel" / "queue" / "001-hello-pin.md")
    assert (pin.status, pin.message_id) == ("posted", 77)


def test_avatar_is_640_square():
    Image = pytest.importorskip("PIL.Image")

    assert Image.open(REPO / "docs" / "channel" / "avatar.png").size == (640, 640)


def test_post_with_pending_media_is_never_published(queue, tmp_path):
    path = make_post(queue, 1, media="", media_pending="ждём ролик")
    out = []
    rc = channel_post.run(queue, tmp_path, T0, lambda *a: pytest.fail("сеть"), dry_run=False, out=out.append)
    assert rc == 1 and any("не готово" in line for line in out)
    assert lib.parse_post(path).status == "approved"


def _fake_convert(gif):
    out = gif.with_suffix(".mp4")
    out.write_bytes(b"mp4")
    return out


def _album_files(tmp_path):
    (tmp_path / "a.gif").write_bytes(b"GIF89a")
    Image = pytest.importorskip("PIL.Image")

    Image.new("RGB", (100, 60)).save(tmp_path / "i.png")


def test_extra_media_goes_as_one_album_with_caption_on_first(queue, tmp_path, monkeypatch):
    _album_files(tmp_path)
    monkeypatch.setattr(lib, "gif_to_mp4", _fake_convert)
    monkeypatch.setattr(lib.build_album_request, "__defaults__", (lib.CHANNEL_ID, _fake_convert))
    path = make_post(queue, 1, media="a.gif", extra_media="i.png")
    calls = []

    def transport(method, params, media):
        calls.append((method, params, media))
        return {"ok": True, "message_ids": [5, 6]}

    assert channel_post.run(queue, tmp_path, T0, transport, dry_run=False, out=lambda s: None) == 0
    assert len(calls) == 1 and calls[0][0] == "sendMediaGroup"
    items = calls[0][1]["media"]
    assert [i["type"] for i in items] == ["video", "photo"]  # GIF стал video, animation в альбоме нельзя
    assert "caption" in items[0] and items[0]["parse_mode"] == "HTML" and "caption" not in items[1]
    assert [m.name for m in calls[0][2]] == ["a.mp4", "i.png"]
    post = lib.parse_post(path)
    assert (post.status, post.message_id, post.extra_message_ids) == ("posted", 5, [6])


def test_gif_to_mp4_runs_ffmpeg_with_safe_flags(tmp_path, monkeypatch):
    gif = tmp_path / "x.gif"
    gif.write_bytes(b"GIF89a")
    seen = {}

    def fake_run(cmd, **kw):
        seen["cmd"] = cmd
        return None

    monkeypatch.setattr(lib.subprocess, "run", fake_run)
    out = lib.gif_to_mp4(gif, tmp_path / "w")
    cmd = seen["cmd"]
    assert out == tmp_path / "w" / "x.mp4" and cmd[0] == "ffmpeg" and "-an" in cmd
    assert cmd[cmd.index("-c:v") + 1] == "libx264" and cmd[cmd.index("-pix_fmt") + 1] == "yuv420p"
    assert "trunc(iw/2)*2" in cmd[cmd.index("-vf") + 1]


def test_post_without_extra_media_is_sent_as_before(queue, tmp_path):
    (tmp_path / "a.gif").write_bytes(b"GIF89a")
    (tmp_path / "v.mp4").write_bytes(b"mp4")
    make_post(queue, 1, media="a.gif")
    make_post(queue, 2, media="v.mp4")
    posts = lib.load_queue(queue)
    assert lib.build_publish_request(posts[0], tmp_path)[0] == "sendAnimation"
    assert lib.build_publish_request(posts[1], tmp_path)[0] == "sendVideo"


def test_album_limits_and_exec_whitelist():
    assert lib.ALBUM_MIN == 2 and lib.ALBUM_MAX == 10 and lib.CAPTION_LIMIT == 1024
    exec_src = (REPO / "scripts" / "telegram" / "channel_exec.py").read_text(encoding="utf-8")
    for method in ("sendMediaGroup", "sendVideo", "deleteMessage"):
        assert f'"{method}"' in exec_src


def _posted(queue, num, mid, extra=""):
    return make_post(
        queue, num, status="posted", message_id=str(mid), posted_at="2026-11-06T08:00:00Z", extra_message_ids=extra
    )


def test_republish_deletes_ids_incl_extra_then_republishes_in_order(queue, tmp_path):
    (tmp_path / "a.gif").write_bytes(b"GIF89a")
    _posted(queue, 1, 5)
    _posted(queue, 2, 7)
    _posted(queue, 3, 9)
    calls = []

    def transport(method, params, media):
        calls.append((method, params.get("message_id")))
        return {"ok": True, "message_id": 100 + len(calls)}

    rc = channel_post.republish_from(
        queue, tmp_path, 2, {2: [8]}, transport, False, pause_s=0, sleep=lambda s: None, out=lambda s: None
    )
    assert rc == 0
    assert calls[:3] == [("deleteMessage", 7), ("deleteMessage", 8), ("deleteMessage", 9)]
    assert [c[0] for c in calls[3:]] == ["sendMessage", "sendMessage"]
    posts = {p.name[:3]: p for p in lib.load_queue(queue)}
    assert posts["001"].message_id == 5  # ниже N не трогаем
    assert (posts["002"].status, posts["002"].message_id) == ("posted", 104)


def test_republish_stops_when_delete_fails_and_publishes_nothing(queue, tmp_path):
    _posted(queue, 2, 7)
    _posted(queue, 3, 9)
    calls = []
    out = []

    def transport(method, params, media):
        calls.append(method)
        return {"ok": False, "description": "message can't be deleted"}

    rc = channel_post.republish_from(queue, tmp_path, 2, {}, transport, False, out=out.append)
    assert rc == 1 and calls == ["deleteMessage"]
    assert any("не вышел" in line for line in out)
    assert lib.parse_post(queue / "002-p.md").message_id == 7  # ничего не сброшено


def test_republish_dry_run_sends_nothing_and_changes_nothing(queue, tmp_path):
    _posted(queue, 2, 7)
    before = (queue / "002-p.md").read_text(encoding="utf-8")
    out = []
    rc = channel_post.republish_from(queue, tmp_path, 2, {2: [8]}, lambda *a: pytest.fail("сеть"), True, out=out.append)
    assert rc == 0 and (queue / "002-p.md").read_text(encoding="utf-8") == before
    assert any("deleteMessage 8" in line for line in out) and any("sendMessage" in line for line in out)


def test_parse_extra_ids():
    assert channel_post.parse_extra_ids(["3:8", "4:1,2"]) == {3: [8], 4: [1, 2]}
    with pytest.raises(ValueError):
        channel_post.parse_extra_ids(["x"])
