"""ADR-0153 S4: стиль ``lofi`` — запись ``knowledge.STYLES``: свинг оффбит-восьмых, comping септаккордами, walking-бас,
кит паков. Поведение — на ``SetPlan``/``Track``/событиях программы Renardo (симулятор ``render.events``): свинг-ratio
по онсетам модели, comping мимо сильных долей хука, подход walking ≤ 1 доли, септаккорды пэда, окно темпа. Клуб
побайтно тот же — ``test_style_same_tracks``.
"""

from __future__ import annotations

import dataclasses
import statistics

import pytest

from melodies import MELODIES
from rob_box_music import knowledge as kn
from rob_box_music.arrange import harmony
from rob_box_music.arrange.compose import compose
from rob_box_music.diversity import track_composition
from rob_box_music.model import BEATS_PER_BAR, Stereo, validate
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import RenderError, render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import match_style_text, seeded_profile

LOFI = kn.STYLES["lofi"]
#: Цель свинга (ADR-0153 §5 S2 для lo-fi, эталоны 07.10): ratio долгой/короткой восьмой 1.3–1.6, не триоль 2.0.
RATIO = (1.3, 1.6)
STRONG = (0, 8)


def _plan(theme: str, seed: int, hooks: bool = False):
    profile = seeded_profile(theme, "lofi")
    profile = dataclasses.replace(profile, hook_ids=tuple(MELODIES)) if hooks else profile
    return seeded_plan(profile, seed, set_id=f"lo{seed}")


@pytest.fixture(scope="module")
def tracks():
    """3 сида × 2 трека (сид 0 и 2 — хуки RTTTL темы, сид 1 — мотив лида): (план, трек, программа, события по ролям)."""
    out = []
    for seed in range(3):
        plan = _plan("дождливый вечер", seed, hooks=seed != 1)
        history: list = []
        for no in (1, 2):
            track = compose(plan, no, history=history, melodies=MELODIES if seed != 1 else None)
            validate(track)
            program = render(track, "A")
            _p, events = program_events(program.code, program.form_beats, loops=True)
            role_of = {slot: role for role, slot in program.slots.items()}
            by_role: dict = {}
            for ev in events:
                by_role.setdefault(role_of[ev.slot], []).append(ev)
            out.append((plan, track, program, by_role))
            history.insert(0, track_composition(track))
    return out


def test_tempo_window_and_swing_of_the_set():
    """Темп сета — в окне стиля (72–92, эталон trip-hop 88), свинг — доля восьмой 0.14–0.22 → ratio 1.33–1.56."""
    bpms = [w.bpm for w in LOFI.genre_windows.values()]
    assert (min(b for b, _ in bpms), max(b for _, b in bpms)) == (72, 92)
    assert kn.genre_style(LOFI, next(iter(LOFI.genre_windows))) == LOFI, "поля стиля — первое окно"
    for seed in range(20):
        plan = _plan("дождливый вечер", seed)
        window = LOFI.genre_windows[plan.genre]
        assert window.bpm[0] <= plan.bpm <= window.bpm[1], (seed, plan.genre, plan.bpm)
        ratio = (1 + plan.swing) / (1 - plan.swing)
        assert RATIO[0] <= ratio <= RATIO[1], (seed, plan.swing, ratio)


def _ratio(events, beat_len: float = 1.0) -> list:
    """(долгая/короткая) восьмая по онсетам: доля с онсетом на начале и на втором онсете внутри доли."""
    onsets = sorted({round(e.beat, 6) for e in events})
    out = []
    for beat in sorted({int(b) for b in onsets}):
        inner = [b - beat for b in onsets if beat < b < beat + beat_len - 1e-9]
        if beat in onsets and len(inner) == 1:
            out.append(inner[0] / (beat_len - inner[0]))
    return out


def test_swing_ratio_by_model_onsets_is_in_the_target_window(tracks):
    """Хэты восьмыми: ratio по онсетам событий программы = (1 + swing)/(1 − swing) и в окне 1.3–1.6 в каждом треке."""
    for plan, _track, _program, by_role in tracks:
        ratios = _ratio(by_role["hats"])
        assert len(ratios) > 50
        med = statistics.median(ratios)
        assert RATIO[0] <= med <= RATIO[1] and med == pytest.approx((1 + plan.swing) / (1 - plan.swing), abs=0.02)


def test_one_groove_for_every_role_offbeat_eighths_late_nothing_on_the_straight_and(tracks):
    """Свинг — грув всех ролей стиля (``Style.swing_roles`` + хэты): ни одного онсета на прямом «и», оффбит-восьмые —
    на «и» + swing·½ доли, остальное — на сетке 16-х."""
    for plan, track, _program, by_role in tracks:
        late = 0.5 + round(plan.swing * 30000 / plan.bpm) * plan.bpm / 60000  # offset_ms — целые мс
        for role in ("hats", "kick", "clap", "bass", "pad", "lead"):
            assert by_role.get(role), (role, track.track_id)
            for e in by_role[role]:
                frac = e.beat % 1
                assert min(abs(frac - f) for f in (0.0, 0.25, 0.75, late, 1.0)) < 1e-3, (role, frac, late)
        assert any(abs(e.beat % 1 - late) < 1e-3 for e in by_role["lead"] + by_role["pad"] + by_role["bass"])


def test_comping_never_hits_a_strong_beat_and_plays_seventh_chords(tracks):
    seen = 0
    for _plan_, track, _program, _by in tracks:
        if track.history_key.pad_figure != "comping":
            continue
        seen += 1
        pad, lead = track.parts["pad"], track.parts["lead"]
        pad_steps = {round(e.beat * 4) for e in pad.pitches}
        assert all(s % 16 not in STRONG for s in pad_steps), "comping мимо сильных долей 1 и 3"
        strong_lead = {round(e.beat * 4) for e in lead.pitches if round(e.beat * 4) % 16 in STRONG}
        assert not pad_steps & strong_lead, "хук на сильной доле — comping молчит"
        assert all(e.dur_beats <= 0.75 for e in pad.pitches), "аккорды короткие"
        for chords in track.harmony.progression.values():
            for chord in chords:
                pcs = harmony.chord_pcs(LOFI, track.key, chord.degree)
                assert len(pcs) == 4 and {m % 12 for m in chord.voicing} == set(pcs), (chord, pcs)
    assert seen >= 2


def test_walking_bass_approaches_the_next_bar_within_one_beat(tracks):
    chromatic = 0
    for _plan_, track, _program, _by in tracks:
        assert track.history_key.bass_figure == "walking"
        notes = sorted(track.parts["bass"].pitches, key=lambda e: e.beat)
        by_bar: dict = {}
        for e in notes:
            by_bar.setdefault(int(e.beat // BEATS_PER_BAR), []).append(e)
        scale = kn.scale_pitch_classes(track.key.root, track.key.mode)
        for bar, line in by_bar.items():
            assert [e.beat % BEATS_PER_BAR for e in line] == [0, 1, 2, 3], "четверти на долях"
            last = line[-1]
            assert last.dur_beats <= LOFI.approach_max_beats == 1.0
            if bar + 1 in by_bar:
                assert abs(last.midi - by_bar[bar + 1][0].midi) == 1, "подход полутоном на 4-й доле"
                chromatic += last.midi % 12 not in scale
            assert all(e.midi % 12 in scale for e in line[:3]), "доли 1–3 — тоны аккорда"
    assert chromatic > 0, "подход бывает хроматическим — валидатор держит потолок стиля (1 доля)"


def test_drums_are_pack_files_and_their_events_are_the_model(tracks):
    for _plan_, track, program, by_role in tracks:
        kick = track.parts["kick"]
        assert kick.synth_or_sample in {kn.KICK_SOUNDS[k].pack for k in LOFI.kick_pool}
        for role in ("clap", "hats"):
            assert track.parts[role].synth_or_sample in LOFI.drum_files[role]
        files = {kn.SAMPLE_CATALOG[track.parts[r].synth_or_sample].path for r in ("kick", "clap", "hats")}
        assert files <= program.sample_files and not program.samples, "удары — файлы паков, не play()"
        assert track_composition(track)["kick"] in LOFI.kick_pool
        assert track.mix.a9_model >= LOFI.a9_model_low, track.mix.a9_model
        assert not track.mix.duck_roles, "без сайдчейна"
        model = sorted(round(i / 4 + st.offset_ms * track.bpm / 60000.0, 3)
                       for i, st in enumerate(kick.grid.steps) if st.on and _sounds(track, "kick", i / 4))
        assert sorted(round(e.beat, 3) for e in by_role["kick"]) == model, "события бочки = модель со свингом"


def _sounds(track, role: str, beat: float) -> bool:
    start = 0
    for sec in track.form.sections:
        if start * 4 <= beat < (start + sec.bars) * 4:
            return role in sec.roles
        start += sec.bars
    return False


def test_haas_voice_and_note_swing_cannot_share_the_delay_key(tracks):
    _plan_, track, _program, _by = tracks[0]
    stereo = {**track.mix.stereo, "lead": Stereo(pan=1.0, detune=0.125, haas_ms=15.0)}
    with pytest.raises(RenderError, match="Хаас"):
        render(dataclasses.replace(track, mix=dataclasses.replace(track.mix, stereo=stereo)), "A")


@pytest.mark.parametrize("text", ["лоуфай", "включи лоуфай сет", "лоу-фай про дождь", "лоу фай", "lo-fi", "lo fi",
                                  "lofi beats", "лофай", "чилл", "чиллаут сет", "chill"])
def test_lofi_words_pick_the_style(text):
    assert match_style_text(text) == "lofi"


@pytest.mark.parametrize("text", ["чили", "лофт", "логистика", "фай", "лоток"])
def test_words_close_to_lofi_words_are_not_a_style(text):
    assert match_style_text(text) is None
