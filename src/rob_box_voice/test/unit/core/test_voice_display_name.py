"""Название голоса вслух - одно место: ``tts_voice_registry.voice_display_name``.

Живое 28.10: фраза смены голоса называла id латиницей («Говорю голосом English_Jovialman.», «…male-qn-qingse.») - TTS читает
его по буквам. Название берётся из ``VOICE_METADATA`` («Весельчак», «Надёжный мужчина»), как и список имён для Tier-1
«включи голос <имя>» (``named_voices_for``).
"""

from __future__ import annotations

import pytest

from rob_box_voice.tts_voice_registry import (PROVIDER_LABELS, PROVIDER_VOICES, VOICE_ALIASES, VOICE_METADATA,
                                              named_voices_for, voice_display_name)


@pytest.mark.parametrize("provider, voice, name", [
    ("minimax", "English_Jovialman", "Весельчак"),
    ("minimax", "Russian_ReliableMan", "Надёжный мужчина"),
    ("yandex", "alena", "Алёна"),
    ("silero", "baya", "Бая (женский)"),
])
def test_catalog_voice_is_named_by_its_display_name(provider, voice, name):
    assert voice_display_name(provider, voice) == name


def test_voice_without_a_catalog_record_is_named_by_what_was_said():
    """Чужое имя («Chuck Norris») или голос другого провайдера - как сказали: придумывать название не нам."""
    assert voice_display_name("minimax", "alena") == "alena"
    assert voice_display_name("minimax", "Chuck Norris") == "Chuck Norris"
    assert voice_display_name("nowhere", "x") == "x"


def test_every_catalog_voice_has_a_russian_or_named_display_name_not_its_id():
    """Каждый голос каталога называется записью метаданных, а не id (иначе фраза опять читала бы id латиницей)."""
    for provider, ids in PROVIDER_VOICES.items():
        for voice in ids:
            assert f"{provider}:{voice}" in VOICE_METADATA, (provider, voice)
            assert voice_display_name(provider, voice) == VOICE_METADATA[f"{provider}:{voice}"]["display_name"]


def test_tier1_names_and_spoken_names_are_one_knowledge():
    """Запись Tier-1 - ``(id, название, *алиасы)`` (#4129): название в ней то же, что говорят вслух, алиасы - из
    ``VOICE_ALIASES`` и только они, порядок голосов - каталога."""
    for provider in PROVIDER_VOICES:
        expected = [(v, voice_display_name(provider, v), *VOICE_ALIASES.get(f"{provider}:{v}", ()))
                    for v in PROVIDER_VOICES[provider]]
        assert named_voices_for(provider) == expected
        assert all(entry[1] == voice_display_name(provider, entry[0]) for entry in named_voices_for(provider))


def test_alias_voice_keeps_spoken_name_second_and_aliases_after():
    """Джовиалмен: название вслух - «Весельчак», его кириллические записи идут дальше, а не подменяют название."""
    entry = next(e for e in named_voices_for("minimax") if e[0] == "English_Jovialman")
    assert entry[:2] == ("English_Jovialman", voice_display_name("minimax", "English_Jovialman"))
    assert set(entry[2:]) == set(VOICE_ALIASES["minimax:English_Jovialman"]) and entry[2:]


def test_providers_are_named_aloud_not_by_latin_ids():
    assert PROVIDER_LABELS["yandex"] == "Яндекс" and set(PROVIDER_LABELS) == set(PROVIDER_VOICES)
