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
    effective_provider,
    format_tts_context,
    known_provider,
    language_of,
    resolve_voice,
    role_voices_for,
    voice_info_for,
    voices_for,
    voices_info_for,
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


# ── effective_provider (issue #1229) ──────────────────────────────────────────


def _dead(dead_names):
    dead = set(dead_names)
    return lambda provider: provider in dead


def test_effective_provider_first_alive() -> None:
    chain = ["minimax", "yandex", "silero"]
    # Никто не мёртв → minimax (первый в цепочке).
    assert effective_provider(chain, _dead([])) == "minimax"


def test_effective_provider_skips_dead() -> None:
    chain = ["minimax", "yandex", "silero"]
    # MiniMax в кэше мёртвых (429/quota) → yandex (issue #1229).
    assert effective_provider(chain, _dead(["minimax"])) == "yandex"


def test_effective_provider_all_dead_falls_to_last() -> None:
    chain = ["minimax", "yandex", "silero"]
    assert effective_provider(chain, _dead(["minimax", "yandex"])) == "silero"
    assert effective_provider(chain, _dead(["minimax", "yandex", "silero"])) == "silero"


def test_effective_provider_empty_chain() -> None:
    assert effective_provider([], _dead([])) is None
    assert effective_provider(None, _dead([])) is None


# ── Каталог minimax == актуальный список MiniMax (20.08.2026) ──────────────


def test_minimax_registry_has_current_russian_system_voices() -> None:
    """Реестр должен содержать актуальные русские системные голоса MiniMax.

    Источник правды — официальный список системных голосов:
    https://platform.minimax.io/docs/faq/system-voice-id (20.08.2026).
    До фикса и реестр, и MiniMaxTTSProvider.list_voices() хранили
    несуществующие ID (male-chengshu, Russian_DeepVoice, ...), из-за чего
    LLM получала в [TTS] voices: список голосов, которых нет у провайдера.
    """
    documented_russian = {
        "Russian_HandsomeChildhoodFriend",
        "Russian_BrightHeroine",
        "Russian_AmbitiousWoman",
        "Russian_ReliableMan",
        "Russian_CrazyQueen",
        "Russian_PessimisticGirl",
        "Russian_AttractiveGuy",
        "Russian_Bad-temperedBoy",
    }
    missing = documented_russian - set(voices_for("minimax"))
    assert not missing, (
        "tts_voice_registry.PROVIDER_VOICES['minimax'] lost current Russian "
        f"system voices from the MiniMax FAQ: {sorted(missing)}"
    )


# AV-27 / issue #1919 — voice_info_for + voices_info_for (wire payload для
# /voice/tts/voices → JSON_EVENT{type:voice_list}).


def test_voice_info_for_known_provider_has_required_fields():
    """voice_info_for(yandex, anton) → dict со всеми wire-полями."""
    info = voice_info_for("yandex", "anton")
    assert info is not None
    assert info["voice_id"] == "anton"
    assert info["display_name"] == "Антон"
    assert info["language"] == "ru-RU"
    assert info["gender"] == "male"
    assert "standard" in info["presets"]


def test_voice_info_for_unknown_voice_returns_none():
    """voice_info_for(yandex, bogus) → None (нет в каталоге провайдера)."""
    assert voice_info_for("yandex", "bogus") is None


def test_voice_info_for_unknown_provider_returns_none():
    """voice_info_for(unknown, *) → None."""
    assert voice_info_for("unknown_provider", "anything") is None


def test_voice_info_for_each_provider_has_at_least_one():
    """Все известные провайдеры имеют хотя бы один голос с метаданными."""
    for provider in ("yandex", "minimax", "silero"):
        voices = voices_for(provider)
        assert voices, f"{provider} has no voices"
        first = voice_info_for(provider, voices[0])
        assert first is not None
        assert "voice_id" in first
        assert "display_name" in first


def test_voices_info_for_returns_list_in_provider_order():
    """voices_info_for(yandex) повторяет порядок PROVIDER_VOICES[yandex]."""
    raw_ids = voices_for("yandex")
    infos = voices_info_for("yandex")
    assert [i["voice_id"] for i in infos] == raw_ids


def test_voices_info_for_unknown_provider_returns_empty():
    """voices_info_for(unknown) → [] (честный пустой список, без fallback)."""
    assert voices_info_for("unknown_provider") == []


def test_voices_info_for_minimax_includes_legacy_ids():
    """male-qn-qingse + female-shaonv доступны (legacy совместимость)."""
    info_ids = {i["voice_id"] for i in voices_info_for("minimax")}
    assert "male-qn-qingse" in info_ids
    assert "female-shaonv" in info_ids


# ── Голоса ролей выступления (#3998) и «что считать провайдером» (#4024) ─────────────────────────────────


def test_minimax_role_voices_are_only_the_russian_catalogue():
    """Сказка звучала китайским ``male-qn-qingse`` на русском тексте: в роли идут только Russian_*."""
    roles = role_voices_for("minimax")
    assert roles and all(v.startswith("Russian_") for v in roles), roles
    assert "male-qn-qingse" not in roles and "female-shaonv" not in roles
    assert roles == [v for v in voices_for("minimax") if v.startswith("Russian_")]  # порядок каталога


def test_role_voices_come_from_the_metadata_language_not_from_a_second_list():
    for provider in PROVIDER_VOICES:
        for voice in role_voices_for(provider):
            assert language_of(provider, voice) == "ru", (provider, voice)


def test_yandex_role_voices_drop_foreign_language_voices():
    roles = role_voices_for("yandex")
    assert "john" not in roles and "lea" not in roles
    assert {"alena", "zahar", "omazh"} <= set(roles)


def test_silero_role_voices_are_the_whole_catalogue():
    assert role_voices_for("silero") == voices_for("silero")


def test_narrator_voice_is_not_a_role():
    """Пустая строка в ``voice_per_segment`` - рассказчик (текущий голос); роль звучит иначе."""
    roles = role_voices_for("minimax", "Russian_ReliableMan")
    assert "Russian_ReliableMan" not in roles
    assert set(roles) == set(role_voices_for("minimax")) - {"Russian_ReliableMan"}
    assert role_voices_for("minimax", "male-qn-qingse") == role_voices_for("minimax")  # не русский - и так не в ролях


def test_every_provider_has_enough_role_voices_for_a_tale():
    """Рассказчик и хотя бы два персонажа: иначе сказка «разными голосами» невозможна по структуре."""
    for provider in PROVIDER_VOICES:
        assert len(role_voices_for(provider, default_voice_for(provider))) >= 2, provider


def test_role_voices_of_an_unknown_provider_is_empty_and_the_catalogue_is_untouched():
    assert role_voices_for("prefetch") == []
    assert "male-qn-qingse" in voices_for("minimax")  # set_voice и обзоры видят весь каталог


def test_known_provider_is_the_one_rule_for_what_a_provider_is():
    assert [known_provider(p) for p in ("yandex", "MiniMax", " silero ")] == ["yandex", "minimax", "silero"]
    assert [known_provider(p) for p in ("prefetch", "", None, "auto", 5)] == [""] * 5
