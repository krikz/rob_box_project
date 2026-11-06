"""Лимиты Bot API (проверены по документации 06.11.2026, версия 10.3) в коде скриптов канала."""

import sys
from datetime import datetime, timezone
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_lib as lib  # noqa: E402
import channel_post  # noqa: E402


def check(post, root):
    """validate без проверок хэштегов: здесь проверяются длина и разметка, теги — в test_telegram_backfill_tags."""
    return [p for p in lib.validate(post, root) if "хэштег" not in p and "hashtags" not in p]


def post_with(tmp_path, body, media=""):
    path = tmp_path / "001-x.md"
    path.write_text(
        f"---\nstatus: draft\nmedia: {media}\nnot_before: 2026-11-10T10:00:00Z\nmessage_id:\n---\n{body}\n", "utf-8"
    )
    return lib.parse_post(path)


def test_length_counts_text_after_entity_parsing(tmp_path):
    # теги не считаются, &amp; считается как один знак: 1024 видимых знака — ровно предел подписи
    (tmp_path / "a.gif").write_bytes(b"GIF89a")
    body = "<b>" + "а" * 1000 + "</b>" + "&amp;" * 24
    assert lib.visible_length(body) == 1024
    assert check(post_with(tmp_path, body, "a.gif"), tmp_path) == []
    over = check(post_with(tmp_path, body + "я", "a.gif"), tmp_path)
    assert any("1024" in p for p in over)


def test_text_without_media_allows_4096(tmp_path):
    assert check(post_with(tmp_path, "я" * 4096), tmp_path) == []
    assert any("4096" in p for p in check(post_with(tmp_path, "я" * 4097), tmp_path))


@pytest.mark.parametrize(
    "body,fragment",
    [
        ("a < b", "голые"),
        ("a & b", "голые"),
        ("<b>открыт", "не закрыт"),
        ("<b><i>x</b></i>", "неверном порядке"),
        ("<blockquote>a<blockquote>b</blockquote></blockquote>", "вкладывать"),
        ("<marquee>x</marquee>", "не поддерживается"),
    ],
)
def test_html_errors_are_explained(tmp_path, body, fragment):
    assert any(fragment in p for p in check(post_with(tmp_path, body), tmp_path))


def test_new_html_features_are_allowed(tmp_path):
    body = (
        '<blockquote expandable>цитата</blockquote> <tg-spoiler>спойлер</tg-spoiler> <a href="https://x.y">ссылка</a>'
    )
    body += " &lt;"
    assert check(post_with(tmp_path, body), tmp_path) == []


def test_photo_limits(tmp_path):
    Image = pytest.importorskip("PIL.Image", reason="нужен Pillow")
    Image.new("RGB", (4000, 100)).save(tmp_path / "wide.png")  # 40:1
    Image.new("RGB", (6000, 5000)).save(tmp_path / "huge.png")  # ширина+высота 11000
    ratio = check(post_with(tmp_path, "x", "wide.png"), tmp_path)
    huge = check(post_with(tmp_path, "x", "huge.png"), tmp_path)
    assert any("соотношение" in p for p in ratio)
    assert any("10000" in p for p in huge)


def test_gif_over_2mb_and_missing_file(tmp_path):
    (tmp_path / "big.gif").write_bytes(b"0" * (2 * 1024 * 1024 + 1))
    assert any("2 МБ" in p for p in check(post_with(tmp_path, "x", "big.gif"), tmp_path))
    assert any("нет файла" in p for p in check(post_with(tmp_path, "x", "nope.gif"), tmp_path))


def test_text_request_disables_link_preview_with_new_field(tmp_path):
    method, params, _ = lib.build_request(post_with(tmp_path, "x"), tmp_path)
    assert method == "sendMessage"
    assert params["link_preview_options"] == {"is_disabled": True}
    assert "disable_web_page_preview" not in params


def test_send_draft_goes_to_private_chat_not_channel(tmp_path):
    queue = tmp_path / "queue"
    queue.mkdir()
    (queue / "003-p.md").write_text(
        "---\nstatus: draft\nmedia:\nnot_before: 2026-11-12T10:00:00Z\nmessage_id:\n"
        "hashtags: #роббокс #дневник\n---\nТекст\n\n#роббокс #дневник\n",
        encoding="utf-8",
    )
    sent = []

    def transport(method, params, media):
        sent.append((method, params))
        return {"ok": True, "message_id": 1}

    out = []
    assert channel_post.send_draft(queue, tmp_path, 3, transport, False, out=out.append) == 0
    assert [m for m, _ in sent] == ["sendMessage", "sendMessage"]
    assert all(p["chat_id"] == lib.DRAFT_CHAT_ID for _, p in sent)
    assert "12.11.2026 11:00 (GMT+1)" in sent[1][1]["text"]
    assert (queue / "003-p.md").read_text(encoding="utf-8").count("status: draft") == 1  # статус не менялся
    assert channel_post.send_draft(queue, tmp_path, 9, transport, True, out=out.append) == 1
    assert datetime(2026, 1, 1, tzinfo=timezone.utc)


def test_preview_all_survives_pending_media_placeholder(tmp_path):
    queue = tmp_path / "queue"
    queue.mkdir()
    (queue / "002-x.md").write_text(
        "---\nstatus: draft\nmedia: docs/channel/media/none.mp4\nmedia_pending: ждём MP4\n"
        "not_before: 2026-11-10T10:00:00Z\nmessage_id:\nhashtags: #роббокс #дневник\n---\n"
        "<b>x</b> #роббокс #дневник\n",
        "utf-8",
    )
    out = []
    rc = channel_post.preview_all(queue, tmp_path, out=out.append)
    assert rc == 1  # заглушка медиа честно считается проблемой
    assert any("НЕТ ФАЙЛА" in line for line in out)
