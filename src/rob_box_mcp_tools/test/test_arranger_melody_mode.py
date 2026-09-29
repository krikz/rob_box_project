"""Режим мелодии (план 2026-09-28 §7.7): акценты, легато, дыхание, редкий пэд, баланс.

Проверяются свойства рендера, а не звук: на слух ничего из этого не
проверено (в тестовом окружении нет SuperCollider).
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import pytest

from rob_box_mcp_tools.core.arranger import (
    ACCENT_STRONG,
    ACCENT_WEAK,
    BEATS_PER_BAR,
    LEGATO_FRACTION,
    ROLE_PROFILE,
    breath_beats,
    lead_accents,
    render,
    role_peak_sum,
    spec_from_flat,
)
from rob_box_mcp_tools.core.harmonize import HarmonizeOptions
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando
from rob_box_mcp_tools.core.rtttl_compose import melody_to_compose_params, rtttl_to_melody

_FIXTURE = Path(__file__).parent / "fixtures" / "arranger_golden.json"


def _golden(key: str, combo: str = "arc_blip") -> dict:
    cases = json.loads(_FIXTURE.read_text(encoding="utf-8"))["cases"]
    return next(c for c in cases if c["key"] == key and c["combo"] == combo)


def _spec(rtttl: str, options=None, **flat):
    params = melody_to_compose_params(rtttl_to_melody(rtttl), options=options)
    base = dict(form="arc", lead_synth="blip", bass_synth="moogbass", pad_synth="strings")
    base.update(flat)
    spec = spec_from_flat(
        harmony=params["harmony"], bpm=float(params["bpm"]), root=str(params["root"]),
        scale=str(params["scale"]), lead_midi=str(params["lead_midi"]),
        lead_dur=str(params["lead_dur"]), **base,
    )
    return spec, params["harmony"]


def _player_line(code: str, player: str) -> str:
    return next(line for line in code.splitlines() if line.startswith(f"{player} >>"))


def _list_arg(line: str, key: str) -> list:
    m = re.search(rf"\b{key}=\[([^\]]*)\]", line)
    assert m, f"{key}=[...] нет в {line[:120]}"
    return [float(x) for x in m.group(1).split(",")]


class TestAccents:
    def test_downbeat_strong_offbeat_short_weak(self):
        # такт: C(доля 0, 1 бит) · D(0.5 бита на 1) · E(0.25 на 1.5) · F(0.25 на 1.75) · G(2 бита на 2)
        notes = [60, 62, 61, 60, 59]
        durs = [1.0, 0.5, 0.25, 0.25, 2.0]
        acc = lead_accents(notes, durs)
        assert acc[0] == ACCENT_STRONG            # сильная доля такта
        assert acc[2] == ACCENT_WEAK              # между долями, короткая, не вершина
        assert acc[3] == ACCENT_WEAK
        assert acc[4] == ACCENT_STRONG            # на доле и длинная
        assert max(acc) > min(acc)

    def test_every_bar_downbeat_is_strong(self):
        notes = [60, 62, 64, 62] * 4
        durs = [1.0] * 16
        acc = lead_accents(notes, durs)
        assert all(acc[i] == ACCENT_STRONG for i in range(0, 16, BEATS_PER_BAR))

    def test_rest_is_weak_and_chords_use_top_pitch(self):
        acc = lead_accents([None, (52, 64), (55, 67), (52, 64)], [0.5, 0.5, 0.5, 0.5])
        assert acc[0] == ACCENT_WEAK
        assert acc[2] > acc[3]  # (55, 67) — вершина по верхнему тону

    def test_ratio_survives_sanitizer_cap(self):
        assert ACCENT_STRONG <= 0.85
        assert 0.6 <= ACCENT_WEAK / ACCENT_STRONG <= 0.7

    def test_lead_line_has_per_note_amplify_and_legato_sus(self):
        line = _player_line(_golden("axelf")["code"], "p2")
        durs = _list_arg(line, "dur")
        amplify = _list_arg(line, "amplify")
        sus = _list_arg(line, "sus")
        assert len(amplify) == len(durs) == len(sus)
        assert set(amplify) <= {0.85, 0.7, 0.55} and len(set(amplify)) > 1
        assert sus == pytest.approx([d * LEGATO_FRACTION for d in durs], abs=1e-3)
        assert "amp=var(" in line  # секционная огибающая осталась


class TestBreathing:
    def test_phrase_ending_on_a_note_gets_a_bar_of_rest(self):
        # 4 такта, последняя нота без паузы
        lead = [(60, 1.0)] * 15 + [(67, 1.0)]
        assert breath_beats(lead) == BEATS_PER_BAR

    def test_existing_tail_rest_of_a_beat_is_kept_as_is(self):
        lead = [(60, 1.0)] * 15 + [(None, 1.0)]
        assert breath_beats(lead) == 0.0

    def test_short_riff_loops_without_breath(self):
        lead = [(60, 0.5)] * 16  # 2 такта
        assert breath_beats(lead) == 0.0

    def test_rendered_theme_ends_with_rest_and_cycle_grows_by_a_bar(self):
        tetris = _golden("tetris")["rtttl"]
        spec, harmony = _spec(tetris)
        assert harmony.lead[-1][0] is not None  # тема кончается звучащей нотой
        assert spec.theme_bars == int(harmony.bars) + 1
        lead = next(layer for layer in spec.layers if layer.role == "lead")
        assert lead.midi[-1] is None and lead.durs[-1] == BEATS_PER_BAR
        # Все слои темы — одной длины цикла: дыхание не сдвигает бас/пэд от лида.
        cycles = {
            layer.role: round(sum(layer.durs), 4) for layer in spec.layers if layer.durs is not None
        }
        assert len(set(cycles.values())) == 1, cycles
        assert set(cycles.values()) == {spec.theme_bars * BEATS_PER_BAR}


class TestPadDensity:
    def test_auto_pad_is_one_chord_per_bar(self):
        spec, harmony = _spec(_golden("tetris")["rtttl"])
        pad = next(layer for layer in spec.layers if layer.role == "pad")
        # не больше одного удара на такт плюс смены аккорда внутри такта
        assert len(pad.durs) <= spec.theme_bars + len(harmony.chords)
        assert min(pad.durs) >= 1.0
        assert pad.sus is None  # аккорд держится, а не бьёт стаккато

    def test_explicit_stab_keeps_the_ostinato(self):
        spec, _h = _spec(_golden("tetris")["rtttl"], options=HarmonizeOptions(pad_style="stab"))
        pad = next(layer for layer in spec.layers if layer.role == "pad")
        assert max(pad.durs[:-1]) <= 2.0  # удары по долям (хвост — дыхание)
        assert pad.sus is not None

    def test_axelf_pad_hits_fewer_than_before(self):
        line = _player_line(_golden("axelf")["code"], "p3")
        durs = _list_arg(line, "dur")
        assert len(durs) < 28 and sum(durs) == 28  # было 28 ударов по доле


class TestBalance:
    def test_peak_sum_is_in_budget(self):
        assert 1.0 <= role_peak_sum() <= 1.2

    def test_lead_is_the_loudest_role(self):
        lead = ROLE_PROFILE["lead"][2]
        assert all(lead > amp for role, (_p, _o, amp) in ROLE_PROFILE.items() if role != "lead")


@pytest.mark.parametrize("key", ["axelf", "tetris", "imperial"])
def test_sanitizer_accepts_and_keeps_lead_amplify(key):
    code = render(_spec(_golden(key)["rtttl"])[0])
    result = sanitize_renando(code, 0.85)
    assert not result.quality_errors, result.quality_errors
    assert not result.security_error and not result.slot_error
    before = re.search(r"amplify=\[[^\]]*\]", _player_line(code, "p2")).group(0)
    after = re.search(r"amplify=\[[^\]]*\]", _player_line(result.code, "p2")).group(0)
    assert before == after
