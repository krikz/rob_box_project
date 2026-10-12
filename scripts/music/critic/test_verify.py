"""Тесты ``verify.py`` (ADR-0157 PR-4, J1): по паре утверждений каждого класса на синтетическом ``Track``.

``python -m pytest scripts/music/critic/test_verify.py -v``. Трек собран здесь же: 120 BPM (1 доля = 0.5 с),
4 такта ``intro`` (kick, pad) + 8 тактов ``drop`` (kick, bass, pad, lead) = 24 с. Реальные модели с робота в репо не кладутся.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[2] / "src" / "rob_box_music"))

import judge as jd  # noqa: E402
import verify as vf  # noqa: E402
from rob_box_music import model as m  # noqa: E402

CHORD = m.Chord(degree=0, voicing=(48, 52, 55), quality="maj")
CLUSTER = m.Chord(degree=0, voicing=(48, 52, 53, 55), quality="maj")


def _part(role, pitches=None, on=(0, 4, 8, 12)):
    steps = tuple(m.Step(on=i in on) for i in range(16))
    return m.Part(role=role, synth_or_sample="x", grid=m.Grid(steps=steps), pitches=pitches, level_db=-30.0, register=(0, 127))


def _notes(*rows):
    return tuple(m.PitchEvent(midi=mi, beat=b, dur_beats=d) for mi, b, d in rows)


def make_track(bass=(), lead=(), pad_chord=CHORD, hook=None, duck=frozenset({"pad"})):
    sections = (m.Section("intro", 4, 2, frozenset({"kick", "pad"})),
                m.Section("drop", 8, 8, frozenset({"kick", "bass", "pad", "lead"})))
    parts = {"kick": _part("kick"), "bass": _part("bass", _notes(*bass)), "lead": _part("lead", _notes(*lead)),
             "pad": _part("pad", _notes((48, 0.0, 4.0)))}
    tr = m.Transition(phrase_bars=8, bass_swap_bar=4, filter_in=True)
    return m.Track(
        track_id="t", seed=1, bpm=120, key=m.Key(root=0, mode="major"), form=m.Form(sections=sections), parts=parts,
        harmony=m.Harmony(progression={"intro": (CHORD,) * 4, "drop": (pad_chord,) * 8}), hook=hook,
        mix=m.Mix(level_db={"kick": -30.0, "bass": -30.0, "pad": -45.0, "lead": -40.0}, stereo={}, duck_roles=duck),
        energy=3, transition_in=tr, transition_out=tr,
        history_key=m.HistoryKey(kit="k", progression="p", hook=None, sample=None, root=0))


def claim(kind, t0, t1, **args):
    return jd.parse_claims([{"t0": t0, "t1": t1, "kind": kind, "claim": "x", "confidence": 0.5, **({"args": args} if args else {})}])[0]


def one(plan, c):
    return vf.verify(plan, [c])[0].verdict


A = [vf.Placed(make_track(), 0.0)]


def test_hole_bass_absent_in_intro_confirmed_and_kick_refuted():
    assert one(A, claim("hole", 0, 8, role="bass")) == vf.CONFIRMED  # бас входит только в drop (с 8-й секунды)
    assert one(A, claim("hole", 0, 8, role="kick")) == vf.REFUTED  # бочка бьёт по сетке


def test_overlap_arithmetic_and_claimed_seconds_tolerance():
    plan = [vf.Placed(make_track(), 0.0), vf.Placed(make_track(), 20.0)]  # A кончается на 24 с — 4 с нахлёста
    assert one(plan, claim("overlap", 18, 26, seconds=4)) == vf.CONFIRMED
    assert one(plan, claim("overlap", 0, 10)) == vf.REFUTED
    assert one(plan, claim("overlap", 18, 26, seconds=18)) == vf.REFUTED  # судья назвал 18 с, на деле 4


def test_level_gap_by_model():
    assert one(A, claim("level", 10, 12, role="pad", direction="low")) == vf.CONFIRMED  # pad -45 против -30
    assert one(A, claim("level", 10, 12, role="pad", direction="high")) == vf.REFUTED
    assert one(A, claim("level", 10, 12, direction="low")) == vf.UNVERIFIABLE  # нет роли


def test_sustain_long_note_only_defect_outside_pad():
    plan = [vf.Placed(make_track(lead=[(72, 16.0, 12.0)]), 0.0)]
    assert one(plan, claim("sustain", 8, 24, role="lead")) == vf.CONFIRMED
    assert one(plan, claim("sustain", 0, 24, role="pad")) == vf.REFUTED  # H6: у pad можно
    assert one(A, claim("sustain", 8, 24, role="lead")) == vf.UNVERIFIABLE  # нот в окне нет — не «0 долей -> опровергнуто»


def test_note_checks_on_roles_without_notes_are_unverifiable_not_refuted():
    # kick — ударная (pitches=None): sus сэмпла/луп не в нотах модели; роли loop в этой модели нет вовсе
    assert one(A, claim("sustain", 0, 24, role="kick")) == vf.UNVERIFIABLE
    assert one(A, claim("out_of_key", 0, 24, role="kick", midi=[60])) == vf.UNVERIFIABLE
    assert one(A, claim("sustain", 0, 24, role="loop")) == vf.UNVERIFIABLE
    assert one(A, claim("hole", 0, 24, role="loop")) == vf.UNVERIFIABLE


def test_phrasing_continuous_sixteenths_vs_rests():
    solid = [(72, 16.0 + i * 0.25, 0.25) for i in range(32)]
    rests = [(72, 16.0 + i * 1.0, 0.25) for i in range(8)]
    assert one([vf.Placed(make_track(lead=solid), 0.0)], claim("phrasing", 8, 12, role="lead")) == vf.CONFIRMED
    assert one([vf.Placed(make_track(lead=rests), 0.0)], claim("phrasing", 8, 12, role="lead")) == vf.REFUTED


def test_out_of_key_note_confirmed_authored_and_missing_refuted():
    bass = [(30, 16.0, 1.0)]  # F#1 в C major, не тон аккорда C-E-G
    assert one([vf.Placed(make_track(bass=bass), 0.0)], claim("out_of_key", 8, 12, role="bass", midi=[30])) == vf.CONFIRMED
    hook = m.Hook(notes=_notes((30, 0.0, 1.0)), bars=4, source=None)
    assert one([vf.Placed(make_track(bass=bass, hook=hook), 0.0)], claim("out_of_key", 8, 12, role="bass", midi=[30])) == vf.REFUTED
    assert one([vf.Placed(make_track(bass=bass), 0.0)], claim("out_of_key", 8, 12, role="bass", midi=[31])) == vf.REFUTED


def test_out_of_key_chord_tone_is_by_design():
    plan = [vf.Placed(make_track(bass=[(52, 16.0, 1.0)]), 0.0)]  # E в C major — в ладу, claim опровергнут: нот вне лада нет
    assert one(plan, claim("out_of_key", 8, 12, role="bass", midi=[52])) == vf.REFUTED


def test_chord_clash_minor_second_in_pad_voicing():
    clash = [vf.Placed(make_track(pad_chord=CLUSTER), 0.0)]
    assert one(clash, claim("chord_clash", 8, 16, role="pad", interval="m2")) == vf.CONFIRMED
    assert one(A, claim("chord_clash", 8, 16, role="pad", interval="m2")) == vf.REFUTED
    assert one(A, claim("chord_clash", 8, 16, role="lead")) == vf.UNVERIFIABLE


def test_mix_needs_recording_masking_sidechain_checked_on_model():
    assert one(A, claim("mix", 0, 10)) == vf.UNVERIFIABLE
    assert one(A, claim("masking", 0, 10)) == vf.UNVERIFIABLE
    assert one(A, claim("masking", 0, 10, param="sidechain", role="bass")) == vf.CONFIRMED  # bass не в duck_roles
    assert one(A, claim("masking", 0, 10, param="sidechain", role="pad")) == vf.REFUTED


def test_param_reading_refuted_by_knowledge_table():
    assert one(A, claim("param_reading", 0, 10, param="lpf", value=0)) == vf.REFUTED  # LPF_OPEN
    assert one(A, claim("param_reading", 0, 10, param="delay", value=0.1)) == vf.REFUTED  # доли
    assert one(A, claim("param_reading", 0, 10, param="lpf", value=4000)) == vf.UNVERIFIABLE
    assert one(A, claim("param_reading", 0, 10, param="cutoff")) == vf.UNVERIFIABLE
    # судья пишет параметр вместе со значением: tempo у лупа — родной темп сэмпла (renardo.py/knowledge.py), не BPM трека
    refuted = vf.verify(A, [claim("param_reading", 0, 10, param="tempo=128 vs 121")])[0]
    assert refuted.verdict == vf.REFUTED and "родной темп сэмпла" in refuted.reason


def test_missing_role_is_unverifiable_not_crash():
    assert one(A, claim("hole", 100, 110, role="lead")) == vf.UNVERIFIABLE  # окно вне треков


def test_j1_table_counts_by_class_and_report_renders():
    vs = vf.verify(A, [claim("mix", 0, 5), claim("hole", 0, 8, role="bass"), claim("param_reading", 0, 5, param="lpf", value=0)])
    tab = vf.j1_table(vs)
    assert tab["ВСЕГО"][vf.CONFIRMED] == 1 and tab["mix"][vf.UNVERIFIABLE] == 1 and tab["param"][vf.REFUTED] == 1
    assert "| ВСЕГО | 3 | 1 | 1 | 1 |" in vf.render_report({"s": vs})


def test_committed_fixtures_follow_the_schema():
    files = sorted((HERE / "fixtures").glob("*.json"))
    assert len(files) == 10
    for path in files:
        data = json.loads(path.read_text(encoding="utf-8"))
        assert data["tracks"] and jd.parse_claims(data["claims"])


def test_real_omni_answers_do_not_crash_verify_and_args_are_normalized():
    """Реальные ответы qwen_omni (breaks: seconds списком, jazz/chiptune: midi числом) — раньше TypeError в verify."""
    for path in sorted((HERE / "fixtures" / "omni").glob("*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        claims = jd.parse_claims(data["claims"])
        assert data["provider"] == "qwen_omni" and claims
        for c in claims:
            assert all(isinstance(m, int) for m in c.args.get("midi", []))
            assert isinstance(c.args.get("seconds", 0.0), float)
        assert len(vf.verify(A, claims)) == len(claims)


def test_int_midi_and_list_seconds_are_checked_not_crashed():
    plan = [vf.Placed(make_track(bass=[(30, 16.0, 1.0)]), 0.0)]
    assert one(plan, claim("out_of_key", 8, 12, role="bass", midi=30)) == vf.CONFIRMED
    two = [vf.Placed(make_track(), 0.0), vf.Placed(make_track(), 20.0)]
    assert one(two, claim("overlap", 18, 26, seconds=[20.0, 24.0])) == vf.CONFIRMED


def test_exception_in_check_gives_unverifiable_with_counter(monkeypatch):
    def boom(plan, c):
        raise TypeError("кривые args")

    monkeypatch.setitem(vf.CHECKS, "hole", boom)
    vs = vf.verify(A, [claim("hole", 0, 8, role="bass"), claim("mix", 0, 5)])
    assert [v.verdict for v in vs] == [vf.UNVERIFIABLE, vf.UNVERIFIABLE]
    assert vs[0].reason.startswith(vf.BAD_ARGS) and "TypeError" in vs[0].reason
    assert f"{vf.BAD_ARGS} (исключение в проверке): 1" in vf.render_report({"s": vs})
