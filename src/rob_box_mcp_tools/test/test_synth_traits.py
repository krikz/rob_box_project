"""Таблица свойств синтов и предупреждение о долгом хвосте (ADR-0132 PR-6).

Заменяет ``test_compose_music_heavy_brass_safety_net.py``: тихого safety net
под имя ``imperialbrass`` больше нет. Здесь проверяется:

1. каждая запись таблицы опирается на исходник в репо, и класс хвоста
   совпадает с огибающей в этом исходнике (а не с памятью автора);
2. :func:`theme_tail_warning` — только при 3 голосах темы и долгом хвосте,
   формулировка совпадает с правилом composer.txt («≤2 голоса»).
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from rob_box_mcp_tools.core.synth_traits import (
    SYNTH_TRAITS,
    THEME_VOICE_LIMIT,
    theme_tail_warning,
    traits_of,
)

_REPO = Path(__file__).resolve().parents[3]
_CUSTOM = _REPO / "docker" / "vision" / "voice_assistant" / "custom_synthdefs"
_PATCHES = _REPO / "src" / "rob_box_voice" / "rob_box_voice" / "core" / "renardo_synthdef_patches.py"
_COMPOSER = _REPO / "src" / "rob_box_voice" / "prompts" / "skills" / "composer.txt"

#: Огибающая, которой обязан соответствовать класс хвоста.
_ENVELOPE_OF_TAIL = {
    "held": re.compile(r"Env\.adsr\("),
    "fixed": re.compile(r"Env\.linen\("),
    "short": re.compile(r"Env\.(perc|asr)\("),
}


def _synthdef_source(name: str) -> str:
    custom = _CUSTOM / f"{name}.scd"
    if custom.exists():
        return custom.read_text(encoding="utf-8")
    text = _PATCHES.read_text(encoding="utf-8")
    match = re.search(r"SynthDef\.new\(\\\\" + name + r",.*?\}\)\.add;", text, re.DOTALL)
    assert match, f"исходник {name} не найден ни в custom_synthdefs, ни в патчах"
    return match.group(0)


@pytest.mark.parametrize("name", sorted(SYNTH_TRAITS))
def test_every_entry_matches_its_synthdef_source(name):
    traits = SYNTH_TRAITS[name]
    source = _synthdef_source(name)
    assert _ENVELOPE_OF_TAIL[traits.tail].search(source), (name, traits.tail)
    assert traits.register in {"lead", "bass", "pad"}
    assert traits.source


@pytest.mark.parametrize("name", sorted(n for n, t in SYNTH_TRAITS.items() if t.tail == "held"))
def test_held_synths_have_no_gate(name):
    """``held`` = ``Env.adsr`` без gate: релиз внутри SynthDef не наступает."""
    assert "gate" not in _synthdef_source(name)


@pytest.mark.parametrize("name", sorted(n for n, t in SYNTH_TRAITS.items() if t.tail == "fixed"))
def test_fixed_tail_seconds_come_from_source(name):
    seconds = SYNTH_TRAITS[name].tail_note.split()[0]
    pattern = r"Env\.linen\(atk, sus, " + re.escape(seconds) + r","
    assert re.search(pattern, _synthdef_source(name))


def test_imperialbrass_is_long_release_and_brass_is_not():
    assert traits_of("imperialbrass").long_release is True
    assert traits_of(" ImperialBrass ").long_release is True
    assert traits_of("brass").long_release is False
    assert traits_of("pianovel") is None  # исходника в репо нет — не классифицирован
    assert traits_of(None) is None and traits_of("") is None


_EXPECTED = (
    "imperialbrass: долгий релиз ≈1.5 с + второй голос + октава = 3 голоса с хвостом "
    "→ counter=off или theme_octaves=off"
)


def test_warning_for_long_release_lead_with_counter_and_octave():
    assert theme_tail_warning("imperialbrass", "imperialbrass", True) == _EXPECTED
    assert theme_tail_warning("imperialbrass", "strings", True) == _EXPECTED


@pytest.mark.parametrize("counter, octave", [(None, False), ("strings", False), (None, True)])
def test_no_warning_with_two_voices_or_less(counter, octave):
    assert THEME_VOICE_LIMIT == 2
    assert theme_tail_warning("imperialbrass", counter, octave) is None


@pytest.mark.parametrize("lead", ["brass", "organ", "pianovel", "blip", None])
def test_no_warning_for_short_or_unknown_release(lead):
    assert theme_tail_warning(lead, "strings", True) is None


def test_long_release_counter_is_named_even_with_short_lead():
    warning = theme_tail_warning("brass", "marchstrings", True)
    assert warning is not None and warning.startswith("marchstrings: долгий релиз +")


def test_warning_wording_matches_composer_rule():
    """Правило скилла и предупреждение говорят одними словами."""
    rule = " ".join(_COMPOSER.read_text(encoding="utf-8").split())
    assert "не больше 2 голосов ОДНОВРЕМЕННО на теме" in rule
    assert "долгий релиз … = 3 голоса с хвостом → counter=off или theme_octaves=off" in rule
    assert "долгий релиз" in _EXPECTED and "counter=off или theme_octaves=off" in _EXPECTED
