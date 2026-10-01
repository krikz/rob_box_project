"""Хук RTTTL-мелодии на lead клубного трека (issue #3181).

Мелодии — дословные записи архива ``rtttl_melodies.jsonl.gz`` (тест не
зависит от архива). Хэши без хука сняты с ``origin/develop`` (ab1b48ff0)
ДО правки: без ``hook=`` вывод ``render_club`` обязан совпадать побайтно.
"""

from __future__ import annotations

import hashlib

import pytest

from rob_box_mcp_tools.core.club_arranger import (
    CLUB_SYNTHS,
    LEAD_LOW,
    LEAD_TOP_LIMIT,
    PUMP_HIGH,
    club_kit,
    hook_pump_weights,
    render_club,
    render_club_kit,
)
from rob_box_mcp_tools.core.club_hook import (
    HOOK_MAX_BARS,
    LEAD_LOOP_BARS,
    extract_hook,
    quantize,
    time_scale,
)
from rob_box_mcp_tools.core.club_hook import _MINOR_CHORDS
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando

CONTRA = (
    "Contra:d=4,o=6,b=285:a#5,a#5,c#,a#5,e.,d#.,c#,a#5,a#5,c#,a#5,e.,d#.,c#,a#5,a#5,c#,a#5,e.,d#.,c#,"
    "a#5,a#5,c#,a#5,d#.,e.,f,c,c,d#,c,f#.,f.,d#,c,c,d#,c,f#.,f.,d#,c,c,d#,c,f#.,f.,d#,c,c,d#,c,f.,f#.,g"
)
TETRIS = (
    "Tetris:d=4,o=5,b=160:e6,8b,8c6,8d6,16e6,16d6,8c6,8b,a,8a,8c6,e6,8d6,8c6,b.,8c6,d6,e6,c6,a,2a,8p,"
    "d6,8f6,a6,8g6,8f6,e.6,8c6,e6,8d6,8c6,b,8b,8c6,d6,e6,c6,a,2a"
)
ZELDA = (
    "Zelda:d=16,o=5,b=200:2a#,2f,4p,8a#,8c6,8d6,8d#6,2f6,2p,4f6,4f6,8f#6,8g#6,2a#6,2p,4a#6,8a#6,8p,"
    "8g#6,8f#6,4g#6,8f#6,2f6,2p,2f6,4d#6,8d#6,8f6,2f#6,2p,4f6,4d#6,4c#6,8c#6,8d#6,2f6,2p,4d#6,4c#6,"
    "4c6,8c6,8d6,2e6,2p,2g6,1f6"
)
#: Overworld-тема (архив ``supermar_4``; ``supermar`` в архиве — Super Mario World).
MARIO = (
    "Supermar:d=16,o=6,b=100:e,e,p,e,p,c,e,p,g,p,g5,p,c,8p,g5,8p,e5,8p,a5,p,b5,p,a#5,a5,p,g5,e,g,a,p,"
    "f,g,p,e,p,c,d,b5,4p,g,f#,f,d#,p,e,p,g#5,a5,c,p,a5,c,d,8p,8d#,p,d"
)
THEMES = {"contra": CONTRA, "tetris": TETRIS, "zelda": ZELDA, "mario": MARIO}

#: sha256 ``render_club(bpm=124, root, seed, **kw)`` на origin/develop ab1b48ff0.
DEVELOP_SHA256 = {
    (0, "A#", ()): "3dd03e9eb811d16312ffe60d5f2f83d7418c8535979d273f1c96a1877732011f",
    (0, "A#", ("repeat",)): "d8a5af10eb8174718048bbefcfbb2508238836fa94542ac5dc65bff5bb2b5a70",
    (0, "A#", ("align_clock", "dj_entry")): "28ac0b12a04ba8f29dd352dc896f9517f3fd9225ad72a2d8d0360fb3d6a71c44",
    (1, "C", ()): "b4e10169f533ec41ddfd3bb5cfa414bd112c5379122d8ac696aac2bdfd065e9d",
    (7, "F", ()): "36b61952513793c1e802407b8d2f15f71f8a5c83e6a283f05c1375cff71bf744",
    (42, "D#", ("repeat",)): "fbc946ec471baa9fcfe8f691ebaa2fa1747a5b91ae3aea65b6770b612676022a",
    (3113, "G", ("align_clock", "dj_entry")): "3d242398dea1cf19f1748fb236e9fcf4b9f0afdaa268f2f0e1aaffcc6ced11e1",
    (7056510, "F", ()): "870e4c30186ef76e6124e11ca3f4c7c6c6384c90fcb10bdb8faef5ab1534cbb6",
    (7036511, "A#", ("align_clock", "dj_entry")): "9aef354e5e94db39ae469e150c6a199ad4ee09ca662656eacd81eb49ab259273",
}


@pytest.mark.parametrize("key", sorted(DEVELOP_SHA256), ids=str)
def test_without_hook_output_is_byte_identical_to_develop(key):
    seed, root, flags = key
    code = render_club(bpm=124, root=root, seed=seed, **{f: True for f in flags})
    assert hashlib.sha256(code.encode()).hexdigest() == DEVELOP_SHA256[key]


def test_time_scale_is_power_of_two_toward_set_tempo():
    assert time_scale(124, 285, [1.0, 1.5]) == 0.5  # Contra: четверти → восьмые
    assert time_scale(124, 160, [0.25]) == 1.0  # Tetris как есть
    assert time_scale(124, 80, [0.125]) == 2.0  # 32-е удваиваются до 16-х
    assert time_scale(124, 300, [0.25]) == 1.0  # 16-я не сжимается мельче шага


def test_quantize_keeps_first_attack_and_clips_length():
    notes = [(60, 0.1), (62, 0.1), (None, 0.3), (64, 1.0)]  # 62 попадает в шаг 60
    assert quantize(notes, 1.0) == [(0, 1, 60), (2, 4, 64)]


def test_contra_hook_is_the_opening_motif():
    hook = extract_hook(CONTRA, 124, "contra", "Contra")
    assert (hook.bars, hook.time_scale, hook.key_name) == (HOOK_MAX_BARS, 0.5, "A# minor")
    # a#5 a#5 c# a#5 e. d#. c# — восьмые, точки = 3 шага
    assert hook.notes[:7] == ((0, 2, 82), (2, 2, 82), (4, 2, 85), (6, 2, 82), (8, 3, 88), (11, 3, 87), (14, 2, 85))


def test_tetris_hook_keeps_rhythm():
    hook = extract_hook(TETRIS, 124)
    assert hook.time_scale == 1.0 and hook.key_name == "A minor"
    assert hook.notes[:8] == ((0, 4, 88), (4, 2, 83), (6, 2, 84), (8, 2, 86), (10, 1, 88), (11, 1, 86),
                              (12, 2, 84), (14, 2, 83))


@pytest.mark.parametrize("theme", sorted(THEMES))
@pytest.mark.parametrize("root", ["A#", "F", "C", "E"])
def test_arrange_keeps_every_interval_and_register(theme, root):
    """Мотив узнаётся, только если интервалы целы: один общий сдвиг на все ноты."""
    from rob_box_mcp_tools.core.arranger import VALID_ROOTS

    hook = extract_hook(THEMES[theme], 124)
    arranged = hook.arrange(VALID_ROOTS.index(root))
    played = [n for n in arranged.lead[:hook.bars * 16] if n is not None]
    source = [m for _s, _l, m in hook.notes]
    assert all(LEAD_LOW <= n <= LEAD_TOP_LIMIT for n in played)
    shifts = [p - s for p, s in zip(played, source)]
    assert len(played) == len(source)
    # переносы на октаву — только у выбросов; у этих тем их нет
    assert set(shifts) == {arranged.transpose}, (theme, root, shifts)


def test_minor_theme_lands_on_set_tonic():
    hook = extract_hook(CONTRA, 124)
    first = hook.arrange(5).lead[0]  # F minor
    assert first % 12 == 5


def test_major_theme_plays_in_relative_major_of_the_set():
    hook = extract_hook(MARIO, 124)
    assert hook.key_name == "C major"
    arranged = hook.arrange(5)  # F minor → параллельный мажор G# (Ab)
    assert arranged.lead[0] % 12 == 0  # E (3-я ступень C) → C (3-я ступень Ab)
    assert "G# major" in arranged.label


@pytest.mark.parametrize("theme", sorted(THEMES))
def test_cycle_matches_the_eight_bar_progression(theme):
    hook = extract_hook(THEMES[theme], 124)
    arranged = hook.arrange(10)
    assert len(arranged.lead) == len(arranged.sus) == LEAD_LOOP_BARS * 16
    assert len(arranged.chords) == LEAD_LOOP_BARS // 2
    assert all(chord in _MINOR_CHORDS for chord in arranged.chords)
    assert arranged.lead[:hook.bars * 16] * (LEAD_LOOP_BARS // hook.bars) == arranged.lead


def test_contra_opens_on_the_tonic_chord():
    arranged = extract_hook(CONTRA, 124).arrange(10)
    assert arranged.chords[0] == (0, "m")


def test_extract_hook_rejects_silence():
    with pytest.raises(ValueError, match="нет ни одной ноты"):
        extract_hook("x:d=4,o=5,b=100:p,p", 124)


@pytest.mark.parametrize("theme", sorted(THEMES))
@pytest.mark.parametrize("seed", [0, 7, 7056510])
def test_hook_track_passes_sanitizer_unchanged(theme, seed):
    code = render_club(bpm=124, root="F", seed=seed, hook=extract_hook(THEMES[theme], 124, theme))
    result = sanitize_renando(code, 0.85, known_synths=CLUB_SYNTHS)
    assert result.security_error is None
    assert result.quality_errors == ()
    assert result.slot_error is None
    assert result.warnings == ()
    assert result.code == code


def _player_block(code: str, slot: str) -> str:
    start = code.index(f"{slot} >> ")
    end = code.find("\n\n", start)
    return code[start:] if end < 0 else code[start:end]


@pytest.mark.parametrize("seed", [0, 7, 7056510])
def test_hook_replaces_only_lead_notes(seed):
    """Ударные, гейты секций и калибровка — те же; меняются ноты lead и аккорды."""
    hook = extract_hook(CONTRA, 124, "contra", "Contra")
    plain = render_club(bpm=124, root="F", seed=seed)
    hooked = render_club(bpm=124, root="F", seed=seed, hook=hook)
    for slot in ("d1", "d2", "d3"):
        assert _player_block(plain, slot) == _player_block(hooked, slot)
    for slot in ("p1", "p2", "p3"):
        amp_plain = [ln for ln in _player_block(plain, slot).splitlines() if "amp=var" in ln]
        amp_hooked = [ln for ln in _player_block(hooked, slot).splitlines() if "amp=var" in ln]
        assert amp_plain == amp_hooked
    lead = _player_block(hooked, "p1")
    assert f"amplify={[PUMP_HIGH] * 16}".replace(" ", "") in lead.replace(" ", "")
    assert hook_pump_weights() == [PUMP_HIGH] * 16
    assert "None" in lead and "sus=[" in lead and "dur=1/4" in lead
    assert "# хук contra «Contra», 4 такта" in hooked


def test_render_club_kit_hook_overrides_progression():
    hook = extract_hook(TETRIS, 124, "tetris")
    kit = club_kit(5)
    a = render_club_kit(kit, root="A", seed=5, progression="i-VI-III-VII", hook=hook)
    b = render_club_kit(kit, root="A", seed=5, hook=hook)
    assert a == b
    assert "хук:" in a.splitlines()[0]
