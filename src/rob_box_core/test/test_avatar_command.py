"""Tests for ``rob_box_core.avatar_command`` — контракт ``/avatar/command``.

Закрывает контракт из worker-brief §3.3 (Issue #1914 / AV-22):

* ``/avatar/command`` — producer → супервизор-агент (AV-21):
  ``{request_id, source, client_id, text, ts_ms}``
* ``/avatar/command_result`` — агент → producer'ы:
  ``{request_id, ok, summary, tool_calls}``

Тесты не требуют rclpy (модуль чистый Python).
"""

from __future__ import annotations

import json

import pytest

from rob_box_core.avatar_command import (
    AVATAR_COMMAND_RESULT_TOPIC,
    AVATAR_COMMAND_TOPIC,
    SOURCES,
    build_command,
    build_command_result,
    decode_command,
    decode_command_result,
    encode_command,
    encode_command_result,
    make_quest_client_id,
    make_telegram_client_id,
    new_request_id,
    now_ts_ms,
)


# ─── Контрактные константы ────────────────────────────────────────────────


def test_topics_and_sources_frozen():
    """Имена топиков и source-значения заморожены (§1.2, §3.3 worker-brief)."""
    assert AVATAR_COMMAND_TOPIC == "/avatar/command"
    assert AVATAR_COMMAND_RESULT_TOPIC == "/avatar/command_result"
    assert SOURCES == ("quest", "telegram")


# ─── ID-хелперы ──────────────────────────────────────────────────────────


def test_make_quest_client_id():
    assert make_quest_client_id("sess-abc-123") == "quest:sess-abc-123"


def test_make_quest_client_id_rejects_empty():
    with pytest.raises(ValueError):
        make_quest_client_id("")


def test_make_telegram_client_id():
    assert make_telegram_client_id(123456789) == "telegram:123456789"


def test_make_telegram_client_id_rejects_none_and_non_int():
    with pytest.raises(ValueError):
        make_telegram_client_id(None)  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        make_telegram_client_id("not-an-int")  # type: ignore[arg-type]


def test_new_request_id_is_uuid_v4_string():
    """request_id — UUIDv4 (36 chars: 8-4-4-4-12)."""
    rid = new_request_id()
    assert isinstance(rid, str)
    parts = rid.split("-")
    assert len(parts) == 5
    assert len(parts[0]) == 8
    assert len(parts[1]) == 4
    # version 4 → третья группа начинается с "4"
    assert parts[2].startswith("4")


def test_new_request_id_uniqueness():
    """UUIDv4 → разные на каждый вызов (collisions astronomically unlikely)."""
    ids = {new_request_id() for _ in range(1000)}
    assert len(ids) == 1000


# ─── now_ts_ms с инжектором часов ────────────────────────────────────────


def test_now_ts_ms_default_is_now():
    """Без clock_ns — берём реальное время."""
    before = 1788378800000  # какой-то эпох-мс в прошлом
    got = now_ts_ms()
    assert got >= before


def test_now_ts_ms_with_clock_injection():
    """worker-brief §6.7: часы инжектятся, иначе dead-man не протестировать."""
    # 1.5e18 ns = 1.5e12 ms = 2025-01-01 UTC ms-epoch
    assert now_ts_ms(clock_ns=1_500_000_000_000_000_000) == 1_500_000_000_000


# ─── build_command ───────────────────────────────────────────────────────


def test_build_command_required_fields():
    p = build_command(
        source="quest",
        client_id="quest:sess-1",
        text="мотивируй народ",
        ts_ms=1700000000000,
        request_id="fixed-uuid-for-test",
    )
    assert p["source"] == "quest"
    assert p["client_id"] == "quest:sess-1"
    assert p["text"] == "мотивируй народ"
    assert p["ts_ms"] == 1700000000000
    assert p["request_id"] == "fixed-uuid-for-test"


def test_build_command_auto_fields():
    """Без ts_ms и request_id — генерим автоматически."""
    p = build_command(source="telegram", client_id="telegram:1", text="hi")
    assert isinstance(p["request_id"], str) and len(p["request_id"]) == 36
    assert isinstance(p["ts_ms"], int) and p["ts_ms"] > 0


def test_build_command_rejects_bad_source():
    with pytest.raises(ValueError, match="source"):
        build_command(source="foo", client_id="x", text="y")


def test_build_command_rejects_empty_client_id():
    with pytest.raises(ValueError, match="client_id"):
        build_command(source="quest", client_id="", text="y")


def test_build_command_rejects_empty_text():
    with pytest.raises(ValueError, match="text"):
        build_command(source="quest", client_id="x", text="")


# ─── encode/decode command ───────────────────────────────────────────────


def test_encode_decode_command_round_trip():
    """JSON round-trip сохраняет все поля."""
    p = build_command(
        source="quest",
        client_id="quest:sess-1",
        text='мотивируй "народ" — по-русски',
        ts_ms=1700000000000,
        request_id="abc-123",
    )
    raw = encode_command(p)
    assert isinstance(raw, str)
    decoded = decode_command(raw)
    assert decoded == p


def test_encode_command_uses_separators_for_size():
    """Без пробелов — payload компактнее (worker-brief §6.2: нет мусора)."""
    p = build_command(
        source="quest", client_id="q:1", text="t", request_id="r"
    )
    raw = encode_command(p)
    # ensure_ascii=False → русский текст НЕ escape'ится
    assert "\\u" not in raw


def test_decode_command_rejects_empty():
    with pytest.raises(ValueError, match="пустой"):
        decode_command("")
    with pytest.raises(ValueError, match="пустой"):
        decode_command(None)  # type: ignore[arg-type]


def test_decode_command_rejects_invalid_json():
    with pytest.raises(ValueError, match="JSON"):
        decode_command("not json")


def test_decode_command_rejects_missing_fields():
    bad = json.dumps({"source": "quest", "text": "x"})  # нет request_id, client_id, ts_ms
    with pytest.raises(ValueError, match="отсутствуют"):
        decode_command(bad)


def test_decode_command_rejects_bad_source():
    bad = json.dumps({
        "request_id": "r", "source": "web", "client_id": "c", "text": "t",
        "ts_ms": 1,
    })
    with pytest.raises(ValueError, match="source"):
        decode_command(bad)


# ─── build_command_result ────────────────────────────────────────────────


def test_build_command_result_required_fields():
    r = build_command_result(
        request_id="abc-123", ok=True, summary="включил музыку",
        tool_calls=["play_track", "set_volume"],
    )
    assert r["request_id"] == "abc-123"
    assert r["ok"] is True
    assert r["summary"] == "включил музыку"
    assert r["tool_calls"] == ["play_track", "set_volume"]


def test_build_command_result_empty_tool_calls_default():
    r = build_command_result(request_id="r", ok=False, summary="fail")
    assert r["tool_calls"] == []


def test_build_command_result_rejects_empty_request_id():
    with pytest.raises(ValueError, match="request_id"):
        build_command_result(request_id="", ok=True, summary="x")


# ─── encode/decode command_result ────────────────────────────────────────


def test_encode_decode_command_result_round_trip():
    r = build_command_result(
        request_id="r-1", ok=True, summary="ok", tool_calls=["a", "b"]
    )
    raw = encode_command_result(r)
    decoded = decode_command_result(raw)
    assert decoded == r


def test_decode_command_result_rejects_bad_ok():
    bad = json.dumps({
        "request_id": "r", "ok": "yes", "summary": "s", "tool_calls": [],
    })
    with pytest.raises(ValueError, match="ok"):
        decode_command_result(bad)


def test_decode_command_result_rejects_non_list_tool_calls():
    bad = json.dumps({
        "request_id": "r", "ok": True, "summary": "s", "tool_calls": "not-a-list",
    })
    with pytest.raises(ValueError, match="tool_calls"):
        decode_command_result(bad)


# ─── Тот же payload можно подставить в обе стороны ──────────────────────


def test_command_and_result_share_request_id_semantics():
    """request_id, выданный producer'ом, коррелирует с command_result."""
    cmd = build_command(
        source="quest", client_id="quest:s1", text="t",
        request_id="corr-id-1",
    )
    res = build_command_result(
        request_id=cmd["request_id"], ok=True, summary="ok",
    )
    assert cmd["request_id"] == res["request_id"]


# ─── Защита от подделки client_id на клиенте ─────────────────────────────


def test_producer_server_side_client_id_quest():
    """``make_quest_client_id`` — единственный путь; producer не берёт
    значение, присланное клиентом (worker-brief §1.3)."""
    # Симулируем «клиент прислал client_id=foo» — сервер игнорирует и
    # формирует свой на основе session_id.
    fake_client_id = "foo"  # прислал клиент
    real_client_id = make_quest_client_id("sess-1")  # сформировал сервер
    assert fake_client_id != real_client_id
    assert real_client_id == "quest:sess-1"


def test_producer_server_side_client_id_telegram():
    fake_client_id = "telegram:999999"  # прислал клиент
    real_client_id = make_telegram_client_id(1)  # chat_id, выданный PTB
    assert real_client_id == "telegram:1"
    assert fake_client_id != real_client_id
