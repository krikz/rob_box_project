"""Валидатор Track: 1000 случайных валидных и набор невалидных с путём ошибки (ADR-0149 §9 PR-1)."""

from dataclasses import replace

import pytest
from track_factory import make_track

from rob_box_music.knowledge import AUTHOR_QUALITIES, LEVEL_CEILINGS, scale_pitch_classes
from rob_box_music.model import (
    Chord, Duck, Form, Grid, Part, PitchEvent, Section, Step, Stereo, Transition, TrackError, chord_tones, validate,
)

SEEDS = range(1000)


@pytest.mark.parametrize("chunk", range(10))
def test_random_valid_tracks_pass(chunk):
    for seed in range(chunk * 100, chunk * 100 + 100):
        validate(make_track(seed))


def test_generator_is_deterministic_and_varied():
    assert make_track(7) == make_track(7)
    assert len({make_track(s).key for s in range(50)}) > 10
    assert {make_track(s).form.bars_total for s in range(50)} == {32, 48, 64}


def _with_part(track, role, **changes):
    return replace(track, parts={**track.parts, role: replace(track.parts[role], **changes)})


def _bass_with(track, **ev_changes):
    bass = track.parts["bass"]
    first = replace(bass.pitches[0], **ev_changes)
    return _with_part(track, "bass", pitches=(first,) + bass.pitches[1:])


def _off_scale_midi(track):
    lo, hi = track.parts["bass"].register
    scale_pcs = scale_pitch_classes(track.key.root, track.key.mode)
    return next(m for m in range(lo, hi + 1) if m % 12 not in scale_pcs)


CASES = {
    "key.mode": lambda t: replace(t, key=replace(t.key, mode="chromatic")),
    "key.root": lambda t: replace(t, key=replace(t.key, root=12)),
    "bpm": lambda t: replace(t, bpm=300),
    "energy": lambda t: replace(t, energy=6),
    "track_id": lambda t: replace(t, track_id=""),
    "form.bars_total": lambda t: replace(
        t, form=Form(t.form.sections[:-1] + (Section("outro", 4, 1, frozenset({"kick"})),))),
    "form.sections[-1]": lambda t: replace(
        t, form=Form(t.form.sections[:-1] + (replace(t.form.sections[-1], name="drop"),))),
    "form.sections[-1].roles": lambda t: replace(
        t, form=Form(t.form.sections[:-1] + (replace(t.form.sections[-1], roles=frozenset({"kick", "lead"})),))),
    "form.sections[0].roles": lambda t: replace(
        t, form=Form((replace(t.form.sections[0], roles=frozenset({"kick", "zzz"})),) + t.form.sections[1:])),
    "form.sections[1].energy": lambda t: replace(
        t, form=Form((t.form.sections[0], replace(t.form.sections[1], energy=11)) + t.form.sections[2:])),
    "parts.kick.grid.steps": lambda t: _with_part(t, "kick", grid=Grid(tuple(Step(True) for _ in range(20)))),
    "parts.kick.grid.steps[0].accent": lambda t: _with_part(
        t, "kick", grid=Grid((Step(True, 5),) + t.parts["kick"].grid.steps[1:])),
    "parts.bass.register": lambda t: _with_part(t, "bass", register=(20, 40)),
    "parts.bass.pitches[0].midi": lambda t: _bass_with(t, midi=_off_scale_midi(t), dur_beats=1.0),
    "parts.bass.pitches[0].beat": lambda t: _bass_with(t, beat=-1.0),
    "parts.lead.pitches": lambda t: _with_part(t, "lead", pitches=()),
    "parts.kick.pitches": lambda t: _with_part(t, "kick", pitches=(PitchEvent(40, 0.0, 1.0),)),
    "parts.pad.level_db": lambda t: _with_part(t, "pad", level_db=0.0),
    "mix.level_db.hats": lambda t: replace(t, mix=replace(t.mix, level_db={**t.mix.level_db, "hats": -1.0})),
    "mix.duck[0].depth": lambda t: replace(t, mix=replace(t.mix, duck=(Duck(2.0, (0,)),) + t.mix.duck[1:])),
    "mix.lpf.pad[0]": lambda t: replace(t, mix=replace(t.mix, lpf={"pad": ((9000.0, 0.0),) * len(t.form.sections)})),
    "mix.lpf.kick": lambda t: replace(t, mix=replace(t.mix, lpf={"kick": ((400.0, 400.0),) * len(t.form.sections)})),
    "mix.stereo.bass": lambda t: replace(t, mix=replace(t.mix, stereo={**t.mix.stereo, "bass": Stereo(0.3)})),
    "mix.stereo.kick": lambda t: replace(t, mix=replace(t.mix, stereo={**t.mix.stereo, "kick": Stereo(0.0, -1)})),
    "mix.stereo.pad": lambda t: replace(t, mix=replace(t.mix, stereo={**t.mix.stereo, "pad": Stereo(0.9, haas_ms=45)})),
    "hook.bars": lambda t: replace(t, hook=replace(t.hook, bars=12)),
    "harmony.progression.nowhere": lambda t: replace(t, harmony=replace(t.harmony, progression={"nowhere": ()})),
    "transition_in.phrase_bars": lambda t: replace(t, transition_in=Transition(5, 0, True)),
    "transition_out.bass_swap_bar": lambda t: replace(t, transition_out=Transition(8, 8, True)),
}


@pytest.mark.parametrize("path", sorted(CASES))
def test_invalid_track_reports_path(path):
    seed = next(s for s in SEEDS if make_track(s).hook is not None and "bass" in make_track(s).parts)
    with pytest.raises(TrackError) as exc:
        validate(CASES[path](make_track(seed)))
    assert exc.value.path == path
    assert exc.value.reason


def test_off_scale_bass_tone_is_valid_as_a_tone_of_the_declared_chord():
    """Аудит Ф2 (#3530): пэд и бас играют тоны ОБЪЯВЛЕННОГО аккорда такта (вводный тон мажорной V в миноре вне лада);
    нота вне лада проходит, только если аккорд её такта её объявил (``Chord.quality``)."""
    seed = next(s for s in SEEDS if "bass" in make_track(s).parts)
    track = make_track(seed)
    midi = _off_scale_midi(track)
    bad = _bass_with(track, midi=midi, dur_beats=1.0)
    with pytest.raises(TrackError, match="parts.bass.pitches"):
        validate(bad)
    chord = next(Chord(d, (60,), q) for d in range(7) for q in sorted(AUTHOR_QUALITIES)
                 if midi % 12 in chord_tones(track.key, Chord(d, (60,), q)))
    prog = {s.name: (chord,) * s.bars for s in track.form.sections}
    validate(replace(bad, harmony=replace(bad.harmony, progression=prog)))


def test_unknown_chord_quality_is_refused():
    track = make_track(0)
    name = track.form.sections[0].name
    prog = {**track.harmony.progression, name: (Chord(0, (60, 64, 67), "zz"),)}
    with pytest.raises(TrackError, match="quality"):
        validate(replace(track, harmony=replace(track.harmony, progression=prog)))


def test_missing_part_for_section_role():
    track = make_track(1)
    parts = {r: p for r, p in track.parts.items() if r != "hats"}
    with pytest.raises(TrackError) as exc:
        validate(replace(track, parts=parts))
    assert exc.value.path.startswith("form.sections[") and "hats" in exc.value.reason


def test_sum_of_peaks_over_ceiling():
    track = make_track(2)
    loud = {r: replace(p, level_db=LEVEL_CEILINGS[r]) for r, p in track.parts.items()}  # каждая роль на своём потолке
    mix = replace(track.mix, level_db={r: p.level_db for r, p in loud.items()})
    with pytest.raises(TrackError) as exc:
        validate(replace(track, parts=loud, mix=mix))
    assert exc.value.path.startswith("form.sections[") and "сумма пиков" in exc.value.reason


def test_part_type_is_frozen():
    with pytest.raises(AttributeError):
        make_track(3).parts["kick"].level_db = 0.0  # type: ignore[misc]
    assert isinstance(make_track(3).parts["kick"], Part)
