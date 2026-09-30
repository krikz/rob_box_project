"""music_material (issue #3227, umbrella #3223): материал из сообщения → ноты → RTTTL → хук."""

from pathlib import Path

import pytest

from rob_box_mcp_tools.core.club_fragments import is_musical
from rob_box_mcp_tools.core.club_hook import extract_hook
from rob_box_mcp_tools.core.mini_notation import Unsupported, parse_events
from rob_box_mcp_tools.core.music_material import (
    material_name, material_to_rtttl, parse_material, parse_scale, pattern_score,
)
from rob_box_mcp_tools.core.rtttl import parse_rtttl

FIXTURE = Path(__file__).parent / "fixtures" / "strudel_stranger_things.txt"
CONTRA = "Contra:d=4,o=6,b=285:a#5,a#5,c#,a#5,e.,d#.,c#,a#5,a#5,c#,a#5,e.,d#.,c#"


@pytest.fixture(scope="module")
def stranger():
    return parse_material(FIXTURE.read_text(encoding="utf-8"))


# ------------------------------------------------------------------ mini-notation
def test_group_repeat_and_weights():
    events, cycles = parse_events("[0 2 4 6 7 6 4 2]*2")
    assert cycles == 1 and len(events) == 16 and all(d * 16 == 1 for _s, d, _t in events)
    events, _ = parse_events("c d e@2 ~ f")
    assert [(str(s), str(d), t) for s, d, t in events] == [
        ("0", "1/6", "c"), ("1/6", "1/6", "d"), ("1/3", "1/3", "e"), ("5/6", "1/6", "f")]


def test_alternation_expands_sequentially_with_weights():
    events, cycles = parse_events("<~@1 [e2,g2]@2 c3>")
    assert cycles == 4
    assert sorted((str(s), str(d), t) for s, d, t in events) == [
        ("1", "2", "e2"), ("1", "2", "g2"), ("3", "1", "c3")]


def test_chord_alternation_is_a_stack():
    events, cycles = parse_events("<e2,g2,b2>")
    assert cycles == 1 and sorted(t for _s, _d, t in events) == ["b2", "e2", "g2"]


@pytest.mark.parametrize("bad", ["c*<2 3>", "c/2", "bd:6", "c?", "c(3,8)", "[c d", "c d]", "{c d}"])
def test_outside_subset_is_rejected(bad):
    with pytest.raises(Unsupported):
        parse_events(bad)


def test_scale_parsing():
    assert parse_scale("c3:major") == (48, (0, 2, 4, 5, 7, 9, 11))
    assert parse_scale("e:minor pentatonic")[0] == 52  # без октавы — 3, как в Strudel
    assert parse_scale("c3:nosuchmode") is None


# ------------------------------------------------------------------ форматы
def test_rtttl_text_is_validated_and_kept_verbatim():
    mat = parse_material(f"вот тебе ноты: {CONTRA}")
    assert mat is not None and mat.source_format == "rtttl" and mat.bpm == 285 and mat.title == "Contra"
    assert material_to_rtttl(mat).startswith("Contra:d=4,o=6,b=285:")
    assert parse_material("Bad:d=4,o=5,b=120:zzz,qqq") is None


def test_note_list_text():
    mat = parse_material("сыграй e4 g4 b4 c5 на 100 bpm")
    assert mat is not None and mat.source_format == "notelist" and mat.bpm == 100
    assert mat.sounding() == [64, 67, 71, 72]
    assert parse_material("привет, как дела, e4 g4") is None  # слишком мало нот


def test_strudel_numbers_need_scale_and_cpm_math():
    mat = parse_material('setcpm(30)\nnote("0 2 4 7").scale("c4:major")')
    assert mat is not None and mat.bpm == 120  # cpm * 4
    assert mat.sounding() == [60, 64, 67, 72]
    assert parse_material('n("0 2 4 7")') is None  # n() без scale — ступени чего? не выдумываем
    midi = parse_material('note("60 64 67 72")')
    assert midi is not None and midi.bpm == 120 and midi.sounding() == [60, 64, 67, 72]  # cpm по умолчанию 0.5


def test_unfaithful_chain_is_rejected_not_guessed():
    assert parse_material('note("c e g b").rev()') is None
    assert parse_material('note("c e g b").add(7)') is None


@pytest.mark.parametrize("junk", ["", "   ", "привет как дела", "note(", 'note(foo).sound("x")', "const x = 5"])
def test_garbage_is_none(junk):
    assert parse_material(junk) is None


# ------------------------------------------------------------------ фикстура 30.09
def test_stranger_things_fixture(stranger):
    assert stranger is not None
    assert stranger.bpm == 83  # setcpm(83/4): 20.75 циклов/мин * 4 доли
    assert stranger.title == "Stranger Things Intro Theme" and stranger.source_format == "strudel"
    # Выбран мотив-арпеджио [0 2 4 6 7 6 4 2]*2 в c-мажоре: c e g b c b g e ×2 (BASS и LEAD_ARP дают те же ноты).
    assert stranger.detail in ("BASS", "LEAD_ARP")
    pcs = [m % 12 for m in stranger.sounding()]
    assert pcs == [0, 4, 7, 11, 0, 11, 7, 4] * 2
    assert all(d == 0.25 for m, d in stranger.notes if m is not None)  # 16-е


def test_fixture_arp_beats_fx_pad_flute_and_brass(stranger):
    # FX-дорожка на 32-х (openingFx) тех же нот проигрывает: темп нечитаем; флейта/медь — мало атак, паузы.
    assert pattern_score(stranger) == (1, 5, 16, 1)


def test_fixture_rtttl_roundtrip_and_hook(stranger):
    rtttl = material_to_rtttl(stranger)
    _name, bpm, notes = parse_rtttl(rtttl)  # проходит rtttl.py
    assert bpm == 83 and len(notes) == 16
    midis = [m for m, _d in notes]
    assert all(60 <= m <= 107 for m in midis)  # октавы RTTTL 4…7
    assert [m % 12 for m in midis] == [m % 12 for m in stranger.sounding()]
    hook = extract_hook(rtttl, 124)
    assert len(hook.notes) >= 6 and is_musical(hook.notes, hook.bars)


def test_rtttl_quantization_handles_rests_and_long_notes():
    mat = parse_material('note("<c3@2 ~@1 [e3 g3 b3 c4]@1 c4@1>")')
    assert mat is not None and mat.notes[0] == (48, 8.0)
    _n, _b, parsed = parse_rtttl(material_to_rtttl(mat))
    assert parsed[0][1] == 4.0 and parsed[1] == (None, 4.0)  # целая нота, остаток — пауза (лиг в RTTTL нет)
    assert sum(d for _m, d in parsed) == pytest.approx(sum(d for _m, d in mat.notes))


def test_material_name_is_deterministic(stranger):
    assert material_name(stranger) == material_name(stranger)
    assert material_name(stranger).startswith("user_stranger_things_intro_th_")
