"""Unit tests for :mod:`rob_box_voice.core.voice_memory_init`."""

from __future__ import annotations

import logging
from unittest.mock import MagicMock, patch

import pytest

from rob_box_voice.core.voice_memory_init import init_voice_memory, safe_save_turn


# ---------------------------------------------------------------------------
# init_voice_memory
# ---------------------------------------------------------------------------


class TestInitVoiceMemory:
    def test_returns_none_when_module_unavailable(self, caplog: pytest.LogCaptureFixture) -> None:
        with patch.dict("sys.modules", {"rob_box_voice.core.voice_memory": None}):
            with caplog.at_level(logging.WARNING):
                result = init_voice_memory()
        assert result is None
        # Warning is logged about the unavailability
        assert any("VoiceMemory unavailable" in r.message for r in caplog.records)

    def test_constructs_memory_with_env_defaults(self) -> None:
        fake_memory = MagicMock()
        fake_memory.get_stats.return_value = {"turn_count": 7, "session_count": 2}

        fake_class = MagicMock(return_value=fake_memory)
        fake_module = MagicMock()
        fake_module.VoiceMemory = fake_class

        with patch.dict("os.environ", {}, clear=False):
            with patch.dict(
                "sys.modules",
                {"rob_box_voice.core.voice_memory": fake_module},
            ):
                result = init_voice_memory()

        assert result is fake_memory
        # Default DB path used
        _, kwargs = fake_class.call_args
        assert kwargs["db_path"] == "/data/voice_memory.db"
        assert kwargs["ollama_base_url"] == "http://localhost:11434"

    def test_explicit_db_path_and_ollama_url(self) -> None:
        fake_memory = MagicMock()
        fake_memory.get_stats.return_value = {"turn_count": 0, "session_count": 0}

        fake_class = MagicMock(return_value=fake_memory)
        fake_module = MagicMock()
        fake_module.VoiceMemory = fake_class

        with patch.dict(
            "sys.modules",
            {"rob_box_voice.core.voice_memory": fake_module},
        ):
            init_voice_memory(db_path="/tmp/m.db", ollama_base_url="http://x:1234")

        _, kwargs = fake_class.call_args
        assert kwargs["db_path"] == "/tmp/m.db"
        assert kwargs["ollama_base_url"] == "http://x:1234"

    def test_env_var_overrides_only_when_arg_is_none(self) -> None:
        fake_class = MagicMock(return_value=MagicMock(get_stats=MagicMock(return_value={})))
        fake_module = MagicMock()
        fake_module.VoiceMemory = fake_class

        with patch.dict(
            "os.environ",
            {"VOICE_MEMORY_DB_PATH": "/from/env.db", "OLLAMA_BASE_URL": "http://env"},
        ):
            with patch.dict(
                "sys.modules",
                {"rob_box_voice.core.voice_memory": fake_module},
            ):
                # Both args None → use env
                init_voice_memory()

        _, kwargs = fake_class.call_args
        assert kwargs["db_path"] == "/from/env.db"
        assert kwargs["ollama_base_url"] == "http://env"

    def test_returns_none_on_construction_error(self, caplog: pytest.LogCaptureFixture) -> None:
        fake_class = MagicMock(side_effect=RuntimeError("db corrupt"))
        fake_module = MagicMock()
        fake_module.VoiceMemory = fake_class

        with caplog.at_level(logging.ERROR):
            with patch.dict(
                "sys.modules",
                {"rob_box_voice.core.voice_memory": fake_module},
            ):
                result = init_voice_memory()
        assert result is None
        assert any("VoiceMemory init failed" in r.message for r in caplog.records)


# ---------------------------------------------------------------------------
# safe_save_turn
# ---------------------------------------------------------------------------


class TestSafeSaveTurn:
    def test_skips_when_memory_is_none(self) -> None:
        # No exception, returns False
        assert safe_save_turn(None, "user", "hello") is False

    def test_saves_when_memory_available(self) -> None:
        memory = MagicMock()
        result = safe_save_turn(memory, "user", "hello")
        memory.save_turn.assert_called_once_with("user", "hello")
        assert result is True

    def test_swallows_save_errors(self, caplog: pytest.LogCaptureFixture) -> None:
        memory = MagicMock()
        memory.save_turn.side_effect = RuntimeError("disk full")

        with caplog.at_level(logging.WARNING):
            result = safe_save_turn(memory, "assistant", "hi")

        # We still report "attempted" — caller doesn't need to retry.
        assert result is True
        assert any("save_turn(assistant) failed" in r.message for r in caplog.records)