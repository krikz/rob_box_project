"""Unit tests for tts_voice_registry (issue #1219 — LLM voice selection).

Covers:
* per-provider voice lists and default voices (yandex/minimax/silero);
* ``resolve_voice``: known voice passes through, unknown falls back to
  the provider default with ``fell_back=True``;
* ``format_tts_context``: the exact ``[TTS] ...`` LLM-context line (Q8).

Pure Python — no ROS, no rclpy, no torch.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from rob_box_voice.tts_voice_registry import (  # noqa: E402
    DEFAULT_VOICES,
    PROVIDER_VOICES,
    default_voice_for,
    format_tts_context,
    resolve_voice,
    voices_for,
)


# ── Registry shape ────────────────────────────────────────────────────────────


def test_known_providers_have_voices() -> None:
    assert set(PROVIDER_VOICES) >= {"yandex", "minimax", "silero"}
    for provider, voices in PROVIDER_VOICES.items():
        assert voices, f"provider {provider} must have at least one voice"
        assert len(set(voices)) == len(voices), f"duplicate voices in {provider}"


def test_default_voice_is_in_provider_list() -> None:
    for provider, default in DEFAULT_VOICES.items():
        assert default in PROVIDER_VOICES[provider], (
            f"default_voice {default!r} for {provider} must be in its voice list"
        )


def test_default_voice_matches_config() -> None:
    # Должны совпадать с src/rob_box_voice/config/tts_node.yaml.
    assert DEFAULT_VOICES["yandex"] == "anton"
    assert DEFAULT_VOICES["minimax"] == "male-qn-qingse"
    assert DEFAULT_VOICES["silero"] == "aidar"


def test_voices_for_returns_copy() -> None:
    lst = voices_for("yandex")
    lst.append("mutated")
    assert "mutated" not in voices_for("yandex")


def test_unknown_provider_returns_empty() -> None:
    assert voices_for("bogus") == []
    assert default_voice_for("bogus") == ""


# ── resolve_voice (Q6) ────────────────────────────────────────────────────────


def test_resolve_known_voice_no_fallback() -> None:
    voice, fell = resolve_voice("yandex", "alena")
    assert voice == "alena"
    assert fell is False


def test_resolve_unknown_voice_falls_back_to_default() -> None:
    voice, fell = resolve_voice("yandex", "terminator_3000")
    assert voice == "anton"  # default yandex
    assert fell is True


def test_resolve_none_voice_uses_default() -> None:
    voice, fell = resolve_voice("minimax", None)
    assert voice == "male-qn-qingse"
    assert fell is True  # no requested voice → fell back to default


def test_resolve_empty_string_uses_default() -> None:
    voice, fell = resolve_voice("silero", "")
    assert voice == "aidar"
    assert fell is True


def test_resolve_minimax_voice() -> None:
    voice, fell = resolve_voice("minimax", "female-shaonv")
    assert voice == "female-shaonv"
    assert fell is False


def test_resolve_cross_provider_voice_falls_back() -> None:
    # Голос Yandex недоступен у MiniMax → дефолт MiniMax (Q6/Q11).
    voice, fell = resolve_voice("minimax", "alena")
    assert voice == "male-qn-qingse"
    assert fell is True


# ── format_tts_context (Q8) ───────────────────────────────────────────────────


def test_format_context_default_current() -> None:
    line = format_tts_context("yandex")
    assert line.startswith("[TTS] provider: yandex | default_voice: anton | current_voice: anton | voices: ")
    assert "alena" in line
    assert "zahar" in line


def test_format_context_with_current_voice() -> None:
    line = format_tts_context("yandex", current_voice="zahar")
    assert "current_voice: zahar" in line
    assert "default_voice: anton" in line


def test_format_context_minimax() -> None:
    line = format_tts_context("minimax", current_voice="female-shaonv")
    assert "provider: minimax" in line
    assert "default_voice: male-qn-qingse" in line
    assert "current_voice: female-shaonv" in line
    assert "male-qn-qingse" in line
