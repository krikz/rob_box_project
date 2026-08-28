#!/usr/bin/env python3
"""Тесты аутентификации отправителя /mcp/execute (rob_box_mcp_tools.mcp_auth).

Топик ``/mcp/execute`` открыт всему ROS2/Zenoh-графу, а за ним сразу
``registry.execute()``. Эти тесты фиксируют инвариант: без подписи общим
секретом запрос до реестра не доходит.
"""

import os
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rob_box_mcp_tools.mcp_auth import (  # noqa: E402
    ENV_ALLOW_UNAUTH,
    ENV_TOKEN,
    ENV_TOKEN_FILE,
    RequestAuthenticator,
    _read_or_create_token_file,
)


TOKEN = "0123456789abcdef" * 4


def _request(tool_name="get_battery_level", parameters=None, request_id="req-1"):
    return {
        "tool_name": tool_name,
        "parameters": parameters if parameters is not None else {},
        "request_id": request_id,
    }


def _sender(token=TOKEN, name="dialogue_node"):
    return RequestAuthenticator(token, sender=name)


def _server(token=TOKEN):
    return RequestAuthenticator(token)


# ---------------------------------------------------------------------------
# Happy path
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSignAndVerify:
    def test_signed_request_is_accepted(self):
        signed = _sender().sign(_request())
        ok, err = _server().verify(signed)
        assert ok is True, err

    def test_signature_travels_but_token_does_not(self):
        """Любой пир может подписаться на топик — секрет туда попасть не должен."""
        signed = _sender().sign(_request())
        assert TOKEN not in str(signed)
        assert set(signed["auth"]) == {"sender", "ts", "sig"}

    def test_unicode_parameters_round_trip(self):
        signed = _sender().sign(_request(parameters={"waypoint": "кухня"}))
        ok, err = _server().verify(signed)
        assert ok is True, err


# ---------------------------------------------------------------------------
# Rejection paths — the actual vulnerability
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestRejection:
    def test_unsigned_request_is_rejected(self):
        """Ровно то, что раньше исполнялось: голый JSON от любого пира."""
        ok, err = _server().verify(_request(tool_name="move_direction"))
        assert ok is False
        assert "подпис" in err

    def test_request_signed_with_wrong_token_is_rejected(self):
        signed = RequestAuthenticator("attacker-token", sender="dialogue_node").sign(
            _request()
        )
        ok, err = _server().verify(signed)
        assert ok is False
        assert "не совпадает" in err

    def test_tampered_parameters_are_rejected(self):
        """Подпись накрывает параметры, а не только имя тула."""
        signed = _sender().sign(_request(parameters={"distance": 1.0}))
        signed["parameters"] = {"distance": 99999.0}
        ok, err = _server().verify(signed)
        assert ok is False

    def test_tampered_tool_name_is_rejected(self):
        signed = _sender().sign(_request(tool_name="get_battery_level"))
        signed["tool_name"] = "execute_music_code"
        ok, err = _server().verify(signed)
        assert ok is False

    def test_unknown_sender_is_rejected(self):
        signed = _sender(name="some_other_node").sign(_request())
        ok, err = _server().verify(signed)
        assert ok is False
        assert "не в списке разрешённых" in err

    def test_stale_request_is_rejected(self):
        signed = _sender().sign(_request())
        signed["auth"]["ts"] -= 120.0
        signed["auth"]["sig"] = _sender()._signature(
            signed, sender="dialogue_node", ts=signed["auth"]["ts"]
        )
        ok, err = _server().verify(signed)
        assert ok is False
        assert "устарел" in err

    def test_replayed_request_is_rejected(self):
        """Подпись валидна, но тот же request_id второй раз не проходит."""
        server = _server()
        signed = _sender().sign(_request(request_id="replay-me"))
        assert server.verify(signed)[0] is True
        ok, err = server.verify(signed)
        assert ok is False
        assert "овторный" in err

    def test_malformed_auth_block_is_rejected(self):
        for auth in ("not-a-dict", None, {}, {"sender": "dialogue_node"}):
            request = _request()
            request["auth"] = auth
            ok, _ = _server().verify(request)
            assert ok is False, auth

    def test_non_numeric_timestamp_is_rejected(self):
        signed = _sender().sign(_request())
        signed["auth"]["ts"] = "now"
        ok, err = _server().verify(signed)
        assert ok is False
        assert "метка времени" in err


# ---------------------------------------------------------------------------
# Missing-secret behaviour
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestNoToken:
    def test_no_token_rejects_everything_by_default(self):
        """Отсутствие секрета — это отказ, а не открытая дверь."""
        server = RequestAuthenticator(None)
        assert server.enabled is False
        ok, err = server.verify(_sender().sign(_request()))
        assert ok is False
        assert "не сконфигурирован" in err

    def test_explicit_opt_out_passes_everything_through(self):
        server = RequestAuthenticator(None, allow_unauthenticated=True)
        ok, _ = server.verify(_request())
        assert ok is True

    def test_sign_without_token_adds_nothing(self):
        request = RequestAuthenticator(None, sender="dialogue_node").sign(_request())
        assert "auth" not in request


# ---------------------------------------------------------------------------
# Shared-secret file
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestTokenFile:
    def test_token_file_is_created_once_and_reused(self, tmp_path):
        path = str(tmp_path / "sub" / ".mcp_token")
        first = _read_or_create_token_file(path)
        assert first and len(first) == 64
        assert _read_or_create_token_file(path) == first

    def test_token_file_is_not_world_readable(self, tmp_path):
        path = str(tmp_path / ".mcp_token")
        _read_or_create_token_file(path)
        if os.name != "nt":  # POSIX-биты на Windows не значат ничего
            assert os.stat(path).st_mode & 0o077 == 0

    def test_no_leftover_temp_files(self, tmp_path):
        _read_or_create_token_file(str(tmp_path / ".mcp_token"))
        assert [p.name for p in tmp_path.iterdir()] == [".mcp_token"]

    def test_unwritable_path_yields_no_token(self, tmp_path):
        # Файл вместо каталога — makedirs/open обязаны упасть, а мы —
        # вернуть None, а не бросить исключение в конструктор ноды.
        blocker = tmp_path / "blocker"
        blocker.write_text("x", encoding="utf-8")
        assert _read_or_create_token_file(str(blocker / "deep" / ".tok")) is None


@pytest.mark.unit
class TestFromEnv:
    def test_env_token_wins_over_file(self, tmp_path, monkeypatch):
        monkeypatch.setenv(ENV_TOKEN, TOKEN)
        monkeypatch.setenv(ENV_TOKEN_FILE, str(tmp_path / ".mcp_token"))
        monkeypatch.delenv(ENV_ALLOW_UNAUTH, raising=False)
        auth = RequestAuthenticator.from_env(sender="dialogue_node")
        assert auth.enabled is True
        assert not (tmp_path / ".mcp_token").exists()
        assert _server().verify(auth.sign(_request()))[0] is True

    def test_two_nodes_share_the_file_secret(self, tmp_path, monkeypatch):
        """dialogue_node и mcp_server живут в одном контейнере (см. compose)."""
        monkeypatch.delenv(ENV_TOKEN, raising=False)
        monkeypatch.delenv(ENV_ALLOW_UNAUTH, raising=False)
        monkeypatch.setenv(ENV_TOKEN_FILE, str(tmp_path / ".mcp_token"))
        node = RequestAuthenticator.from_env(sender="dialogue_node")
        server = RequestAuthenticator.from_env()
        assert node.enabled and server.enabled
        ok, err = server.verify(node.sign(_request()))
        assert ok is True, err

    def test_opt_out_env_disables_verification(self, tmp_path, monkeypatch):
        monkeypatch.delenv(ENV_TOKEN, raising=False)
        monkeypatch.setenv(ENV_ALLOW_UNAUTH, "1")
        monkeypatch.setenv(ENV_TOKEN_FILE, str(tmp_path / "unwritable" / "x" / ".tok"))
        (tmp_path / "unwritable").write_text("x", encoding="utf-8")
        server = RequestAuthenticator.from_env()
        assert server.enabled is False
        assert server.verify(_request())[0] is True
