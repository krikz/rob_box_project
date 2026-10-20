"""Бас на роботе (#3457) и ширина синта (#3458): поведение модели микса, палитры и рендера.

#3457: ``dub`` на образе 06.10 звучит серединой (доля низа на роботе 0.0, +3 дБ; ``jbass`` −6.5 дБ) — A9-модель
(``mix.low_share``) считает бас в шкале робота, а семьи тембров берут только басы, держащие низ на роботе.
#3458: лиды ``blip``/``cs80lead``/``hoover`` и ``tb303`` — два голоса (``knowledge.SYNTH_STEREO``); у ``tb303`` — без
Хааса (низ двух голосов в фазе), прочие басы и бочка — в центре.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange import mix
from rob_box_music.arrange.compose import compose
from rob_box_music.model import Stereo, TrackError, validate
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

CLUB = kn.STYLES["club"]
PAIRED = {s for synths in kn.BASS_FIGURE_SYNTHS.values() for s in synths}


def _drop(track):
    i = next(i for i, s in enumerate(track.form.sections) if s.name == "drop")
    return i, kn.PAD_FIGURES[track.history_key.pad_figure].level_offset_db


@pytest.fixture(scope="module")
def warm_track():
    plan = seeded_plan(seeded_profile("славянская вечеринка"), 3, set_id="b3")
    return compose(plan, 2, deck="A")


def _share(track, bass_synth):
    i, offset = _drop(track)
    parts = {**track.parts, "bass": replace(track.parts["bass"], synth_or_sample=bass_synth)}
    return mix.low_share(CLUB, parts, track.form, i, track.mix.duck[i], track.mix.duck_roles, offset)


def test_bass_on_the_robot_falls_back_to_nrt_bands():
    assert kn.bass_low_on_robot("bass") == kn.LAYER_BANDS["bass"]["bass"][0]
    assert kn.bass_low_on_robot("dub") == kn.BASS_ROBOT_LOW["dub"] < kn.BASS_MIN_LOW


def test_dub_counts_as_middle_in_the_drop_model(warm_track):
    """Тот же дроп с ``dub`` вместо ``bass``: бас идёт в сумму мощности со сдвигом робота, но не в низ."""
    i, offset = _drop(warm_track)
    without = {r: p for r, p in warm_track.parts.items() if r != "bass"}
    base = mix.low_share(CLUB, without, warm_track.form, i, warm_track.mix.duck[i], warm_track.mix.duck_roles, offset)
    assert _share(warm_track, "dub") < base < _share(warm_track, "bass")


def test_jbass_robot_shift_lowers_the_modelled_low(warm_track, monkeypatch):
    shifted = _share(warm_track, "jbass")
    monkeypatch.setattr(kn, "BASS_ROBOT_DB", {k: v for k, v in kn.BASS_ROBOT_DB.items() if k != "jbass"})
    assert shifted < _share(warm_track, "jbass")


@pytest.mark.parametrize("style", sorted(kn.STYLES))
def test_family_basses_hold_the_low_on_the_robot(style):
    """Палитра басов — по доле низа НА РОБОТЕ: ``dub`` (середина на образе 06.10) в семьи не идёт; в семье ≥ 2 баса."""
    for name, family in kn.STYLES[style].timbres.items():
        plain = [s for s in family["bass"] if s not in PAIRED]
        assert all(kn.bass_low_on_robot(s) >= kn.BASS_MIN_LOW for s in plain), (style, name)
        assert len(plain) >= 2 and "dub" not in family["bass"], (style, name)


def test_warm_family_keeps_three_basses():
    """Тема вне ``THEMES`` (семья по умолчанию ``warm``) — три баса на серию (A16a: бас ≥ 3)."""
    assert len(CLUB.timbres[CLUB.default_timbre]["bass"]) == 3


@pytest.fixture(scope="module")
def hard_tracks():
    out = []
    for seed in range(12):
        plan = seeded_plan(seeded_profile("киберпанк"), seed, set_id=f"h{seed}")
        out += [compose(plan, no, deck="A") for no in (1, 2, 3)]
    return out


def test_hard_leads_and_tb303_are_two_voices(hard_tracks):
    seen = set()
    for track in hard_tracks:
        lead, bass = track.parts["lead"].synth_or_sample, track.parts["bass"].synth_or_sample
        if lead in kn.SYNTH_STEREO:
            st = track.mix.stereo["lead"]
            assert st.voices == 2 and st.pan == kn.PAD_SPREAD and st.haas_ms > 0, track.track_id
            seen.add(lead)
        else:
            assert "lead" not in track.mix.stereo
        if bass in kn.SYNTH_STEREO:
            st = track.mix.stereo["bass"]
            assert st.voices == 2 and st.haas_ms == 0 and st.detune > 0, track.track_id
            seen.add(bass)
        else:
            assert "bass" not in track.mix.stereo
        assert "kick" not in track.mix.stereo
    assert {"tb303", "blip"} <= seen, seen


def test_tb303_renders_two_voices_at_the_same_moment(hard_tracks):
    """Рендер ``tb303``: на каждую ноту два голоса на −1/+1 без задержки (низ в фазе), второй — с расстройкой."""
    track = next(t for t in hard_tracks if t.parts["bass"].synth_or_sample == "tb303")
    program = render(track, "A")
    _p, events = program_events(program.code, program.form_beats)
    bass = [e for e in events if e.slot == program.slots["bass"]]
    by_beat = {}
    for e in bass:
        by_beat.setdefault(round(e.beat, 6), []).append(e)
    assert by_beat and all(sorted(v.pan for v in evs) == [-kn.PAD_SPREAD, kn.PAD_SPREAD] for evs in by_beat.values())
    assert {round(v.detune, 3) for evs in by_beat.values() for v in evs} == {0.0, kn.PAD_DETUNE}


def test_validate_keeps_the_low_in_the_center(hard_tracks):
    track = next(t for t in hard_tracks if t.parts["bass"].synth_or_sample == "tb303")
    validate(track)
    with_haas = replace(track.mix.stereo["bass"], haas_ms=kn.PAD_HAAS_MS)
    for bad in (with_haas, Stereo(0.3)):
        broken = replace(track, mix=replace(track.mix, stereo={**track.mix.stereo, "bass": bad}))
        with pytest.raises(TrackError, match="mix.stereo.bass"):
            validate(broken)
    other = next(t for t in hard_tracks if t.parts["bass"].synth_or_sample not in kn.SYNTH_STEREO)
    widened = replace(other, mix=replace(other.mix, stereo={**other.mix.stereo, "bass": track.mix.stereo["bass"]}))
    with pytest.raises(TrackError, match="mix.stereo.bass"):
        validate(widened)
