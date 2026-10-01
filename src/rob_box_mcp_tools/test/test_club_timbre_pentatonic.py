"""Issue #3268 — club: пентатонические лады и тембр лида/пэда от модели.

Живой лог 01.10.2026 (voice-assistant, 10.1.1.21): club отвечал на
``scale='minorPentatonic'`` «не поддержан клубным режимом», а
``lead_synth``/``pad_synth`` вызова шли в «Проигнорировано» — азиатский и
славянский сеты звучали одним пулом pluck/blip/arpy/karp/marimba.
"""

from __future__ import annotations

import re
from statistics import median

import pytest

from rob_box_mcp_tools.core import _classic_loudness_table as table
from rob_box_mcp_tools.core import club_loudness as L
from rob_box_mcp_tools.core.arranger import VALID_ROOTS
from rob_box_mcp_tools.core.club_arranger import (
    CLUB_SYNTHS,
    CLUB_TEMPLATES,
    LAYER_LEVELS,
    MAX_LAYER_AMP,
    PROGRESSIONS,
    ROLE_PALETTE,
    ROLE_SYNTHS,
    TRIAD,
    bass_root,
    build_matrix,
    club_kit,
    pad_voicing,
    render_club,
)
from rob_box_mcp_tools.core.club_fragments import hook_scale_note
from rob_box_mcp_tools.core.club_hook import extract_hook
from rob_box_mcp_tools.core.club_progressions import SCALE_PENTATONIC, progression_mode, progressions_for
from rob_box_mcp_tools.core.club_timbre import (
    LEAD_TAIL_MAX_S,
    PAD_DROP_MAX_DB,
    PAD_TOO_QUIET,
    TIMBRE_EXTRAS,
    estimated_unit_db,
    timbre_request,
    timbre_sentence,
)
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando
from rob_box_mcp_tools.core.synth_traits import traits_of

PENTAS = tuple(SCALE_PENTATONIC)
ROOTS = ("C", "D", "A#", "F#")


def _clean(code: str) -> None:
    result = sanitize_renando(code, 0.85, known_synths=CLUB_SYNTHS)
    assert result.security_error is None and result.quality_errors == ()
    assert result.slot_error is None and result.warnings == ()


def _player_notes(code: str, slot: str):
    block = re.search(rf"{slot} >> \w+\(\[(.*?)\],", code, re.DOTALL).group(1)
    return [int(v) for v in re.findall(r"\d+", block)]


def _header(code: str):
    return re.search(r"^# club: [^,]+, ([^,]+), ([^,]+),", code, re.MULTILINE).groups()


# ------------------------------------------------------------- пентатоника
@pytest.mark.parametrize("scale", PENTAS)
def test_pentatonic_progressions_stay_inside_the_scale(scale):
    """Все звуки аккордов (бас = корень, пэд = трезвучие/sus4) — в 5 ступенях лада."""
    steps = set(SCALE_PENTATONIC[scale])
    names = progressions_for(scale)
    assert len(names) >= 4
    bank = dict(PROGRESSIONS)
    for name in names:
        chords = bank[name]
        assert len(chords) == 4 and chords[0][0] in (0, 3, 9)
        for offset, quality in chords:
            assert {(offset + i) % 12 for i in TRIAD[quality]} <= steps, (name, offset, quality)


@pytest.mark.parametrize("scale", PENTAS)
@pytest.mark.parametrize("root", ROOTS)
@pytest.mark.parametrize("seed", [0, 1, 7, 6261504])
def test_pentatonic_render_lead_bass_pad_in_scale(scale, root, seed):
    code = render_club(seed=seed, scale=scale, root=root)
    _clean(code)
    key, prog = _header(code)
    assert key == f"{root} {scale}" and progression_mode(prog) == scale
    tonic = VALID_ROOTS.index(root)
    allowed = {(tonic + i) % 12 for i in SCALE_PENTATONIC[scale]}
    assert {n % 12 for n in _player_notes(code, "p1")} <= allowed
    assert {n % 12 for n in _player_notes(code, "p3")} <= allowed
    chords = dict(PROGRESSIONS)[prog]
    assert {bass_root(tonic, c) % 12 for c in chords} <= allowed
    assert all(50 <= n <= 68 for c in chords for n in pad_voicing(tonic, c))
    assert all(58 <= n <= 84 for n in _player_notes(code, "p1"))
    assert code == render_club(seed=seed, scale=scale, root=root)


@pytest.mark.parametrize("scale", PENTAS)
def test_pentatonic_with_history_uses_its_pool(scale):
    rows = []
    seen = set()
    for i in range(12):
        code = render_club(seed=900 + i, scale=scale, root="D", recent=rows)
        _, prog = _header(code)
        assert progression_mode(prog) == scale
        seen.add(prog)
        rows.insert(0, dict(club_kit(900 + i, recent=rows), progression=prog))
    assert len(seen) >= 3


def test_seven_step_scales_are_unchanged_by_pentatonic_bank():
    """Пул прежних ладов и байты seed-рендеров minor не сдвинулись."""
    assert all(not n.startswith(("pmin:", "pmaj:")) for s in ("minor", "dorian", "phrygian", "major")
               for n in progressions_for(s))
    assert render_club(seed=3, scale="minor") == render_club(seed=3, scale="minor", timbre=None)


def test_hook_keeps_its_intervals_and_says_scale_is_not_applied():
    """RTTTL-хук на лад НЕ переносится (узнаваемость #3181) — и ответ это говорит."""
    hook = extract_hook("smb:d=4,o=5,b=100:16e6,16e6,32p,8e6,16c6,8e6,8g6,8p,8g,8p", 124, "smb", "Mario")
    minor = render_club(seed=5, root="D", hook=hook)
    penta = render_club(seed=5, root="D", scale="minorPentatonic", hook=hook)
    assert _player_notes(minor, "p1") == _player_notes(penta, "p1")
    assert hook_scale_note("minor") == "" and hook_scale_note(None) == ""
    note = hook_scale_note("minorPentatonic")
    assert "minorPentatonic к хуку не применён" in note and "без name=/theme=" in note


# ------------------------------------------------------------- тембр от модели
def test_extras_follow_the_pool_rules():
    """Палитра поверх пула: CRITICAL (через CLUB_SYNTHS — test_club_arranger),
    есть в классик-таблице, не held, у лида короткий хвост."""
    for role, extras in TIMBRE_EXTRAS.items():
        assert not set(extras) & set(ROLE_SYNTHS[role]), role
        assert ROLE_PALETTE[role] == ROLE_SYNTHS[role] + extras
        for synth in extras:
            assert synth in table.NOTE_DB, synth
            traits = traits_of(synth)
            assert traits is None or traits.tail != "held", synth
            if role == "lead":
                assert table.TAIL_S[synth][0] <= LEAD_TAIL_MAX_S, synth
    assert table.TAIL_S["marimba"][0] <= LEAD_TAIL_MAX_S  # пул лида укладывается в правило


def test_seed_never_picks_extras_without_request():
    """ADR-0146: без тембра от модели выбор сида — только из прежнего пула."""
    for seed in range(1, 300):
        kit = club_kit(seed)
        for role in ("lead", "pad"):
            assert kit[role] in ROLE_SYNTHS[role], (seed, kit)


def test_timbre_overrides_seed_choice_in_code():
    code = render_club(seed=7, scale="minorPentatonic", root="D", timbre={"lead": "sitar", "pad": "ambi"})
    _clean(code)
    assert "p1 >> sitar(" in code and "p3 >> ambi(" in code
    plain = render_club(seed=7, scale="minorPentatonic", root="D")
    assert _player_notes(code, "p1") == _player_notes(plain, "p1")


@pytest.mark.parametrize("call, applied, reason", [
    ({"lead_synth": "Sitar", "pad_synth": "ambi"}, {"lead": "sitar", "pad": "ambi"}, None),
    ({"lead_synth": "imperialbrass"}, {}, "держит ноту (held)"),
    ({"lead_synth": "flute"}, {}, "хвост flute 0.47 с длиннее 16-й"),
    ({"pad_synth": "strings"}, {}, "strings в club слишком тихий пэд"),
    ({"lead_synth": "theremin"}, {}, "theremin нет в палитре club для роли lead"),
    ({"lead_synth": "none", "pad_synth": ""}, {}, None),
])
def test_timbre_request_applies_or_refuses_with_reason(call, applied, reason):
    chosen, refused = timbre_request(call, ROLE_PALETTE)
    assert chosen == applied
    if reason is None:
        assert refused == []
    else:
        assert len(refused) == 1 and reason in refused[0]
        assert "играет тембр из пула" in refused[0] and "допустимо:" in refused[0]


def test_timbre_sentence():
    assert timbre_sentence(None) == "" and timbre_sentence({"applied": {}, "refused": []}) == ""
    assert "lead sitar" in timbre_sentence({"applied": {"lead": "sitar"}})


# ------------------------------------------------------------- громкость новых тембров
def test_estimate_matches_measured_pool_within_documented_error():
    """Оценка по классик-таблице на синтах пула, замеренных в club (ESTIMATE_NOTE)."""
    for lane, bound in (("lead", 4.2), ("pad", 0.5)):
        errors = [estimated_unit_db(lane, s)[0] - L.LANE_DB_AT_UNIT[lane][s] for s in ROLE_SYNTHS[lane]]
        assert max(abs(e) for e in errors) <= bound, (lane, errors)
        assert abs(median(errors)) <= 0.5, (lane, errors)


def test_measured_synths_keep_their_measurement():
    for lane in ("lead", "pad"):
        for synth in ROLE_SYNTHS[lane]:
            assert L.unit_db(lane, synth) == (L.LANE_DB_AT_UNIT[lane][synth], L.AMP_EXPONENT.get(synth, 1.0))


def _worst_drop(role: str, synth: str) -> float:
    worst = 0.0
    for template in CLUB_TEMPLATES:
        kit = dict(template=template, kick="by_design", hats="by_design", lead="pluck", bass="bass", pad="warmpad")
        kit[role] = synth
        report = L.kit_report(build_matrix(template), kit, LAYER_LEVELS, MAX_LAYER_AMP)
        assert report["after"]["main_db"] == pytest.approx(L.TARGET_MAIN_DB, abs=0.5), (synth, template)
        worst = max(worst, report["after"]["worst_drop_db"])
    return worst


@pytest.mark.parametrize("role, synth", [(r, s) for r, extras in TIMBRE_EXTRAS.items() for s in extras])
def test_extra_timbre_is_calibrated_and_quiet_sections_hold(role, synth):
    assert _worst_drop(role, synth) <= PAD_DROP_MAX_DB


@pytest.mark.parametrize("synth", sorted(PAD_TOO_QUIET))
def test_too_quiet_pads_are_refused_by_the_model_numbers(synth):
    worst = _worst_drop("pad", synth)
    assert worst > PAD_DROP_MAX_DB
    assert worst == pytest.approx(PAD_TOO_QUIET[synth], abs=0.3)
