"""Гард письменности по возможностям выбранного провайдера (#3728, ADR-0150 §8, PR-4 ч.3)."""

from __future__ import annotations

import pytest

from rob_box_voice.tts_text_guard import (
    UnsupportedScriptError, ensure_readable, provider_reads, readable_chain, should_skip,
)

CJK = "你好世界，今天天气很好"
MIXED_JA = "こんにちは世界"
KO = "안녕하세요"
HINDI = "नमस्ते दुनिया"
RU = "Привет, как дела?"


def test_minimax_reads_cjk_japanese_korean():
    for text in (CJK, MIXED_JA, KO):
        assert provider_reads(text, "minimax"), text


def test_yandex_and_silero_do_not_read_cjk():
    for provider in ("yandex", "silero"):
        for text in (CJK, MIXED_JA, KO):
            assert not provider_reads(text, provider), (provider, text)


def test_minimax_still_refuses_scripts_it_does_not_read():
    assert not provider_reads(HINDI, "minimax")


def test_cyrillic_and_latin_are_read_by_everyone():
    for provider in ("minimax", "yandex", "silero"):
        assert provider_reads(RU, provider) and provider_reads("Hello there", provider)


def test_default_without_provider_keeps_the_old_behaviour():
    assert should_skip(CJK) is True
    assert should_skip(RU) is False


def test_fallback_chain_drops_providers_that_cannot_read_the_text():
    assert readable_chain(["minimax", "yandex", "silero"], CJK) == ["minimax"]
    assert readable_chain(["minimax", "yandex", "silero"], RU) == ["minimax", "yandex", "silero"]


def test_last_resort_refuses_honestly_instead_of_muttering():
    ensure_readable(RU, "silero")
    with pytest.raises(UnsupportedScriptError, match="unsupported_script"):
        ensure_readable(CJK, "silero")
