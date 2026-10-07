"""ADR-0153 S3: стили ``breaks`` и ``dnb`` — записи ``knowledge.STYLES``, брейк пака нарезкой по сиду.

Поведение — на ``SetPlan``/``Track``/событиях программы Renardo: нарезка на сетке 16-х и покрывает такт, события
лупа = модель (``Part.chop``), темп в окне стиля, бочка и брейк из каталогов, стиль из фразы, события в секунду не
выше клуба. Клуб побайтно тот же — ``test_style_same_tracks``.
"""

from __future__ import annotations

import json
import random
from pathlib import Path

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange import samples
from rob_box_music.arrange.compose import compose
from rob_box_music.diversity import kick_name, track_composition
from rob_box_music.model import STEPS_PER_BAR, validate
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import match_style_text, seeded_profile, style_marks

STYLES = ("breaks", "dnb")
BREAKS = sorted(n for n, i in kn.SAMPLE_CATALOG.items() if i.role == "break")
PACK = json.loads((Path(kn.__file__).resolve().parent / "data" / "sample_sonicpi.json").read_text(encoding="utf-8"))


def _plan(theme: str, seed: int, style: str):
    return seeded_plan(seeded_profile(theme, style), seed, set_id=f"b{seed}")


def test_breaks_come_from_the_pack_catalog_with_tempo_and_level_of_the_catalog():
    """Брейк — только файл пака с ролью ``break`` (амены Sonic Pi), темп = доли·60/длина, уровень — rms каталога."""
    pack_breaks = {n for n, m in PACK["samples"].items() if m["role"] == "break"}
    assert set(BREAKS) == pack_breaks == set(kn.PACK_BREAK_BEATS)
    for name in BREAKS:
        info, meta = kn.SAMPLE_CATALOG[name], PACK["samples"][name]
        assert info.path == f"{PACK['pack_dir']}/{meta['path']}" and info.loop_arg == f"../../sonicpi/{meta['path']}"
        assert info.beats == kn.PACK_BREAK_BEATS[name] and abs(info.seconds * info.bpm / 60 - info.beats) < 0.05
        assert (info.mean_db, info.peak_db) == (meta["rms_db"], meta["peak_db"])
    assert {kn.SAMPLE_CATALOG[n].bpm for n in BREAKS} == {137, 140, 126}


@pytest.mark.parametrize("name", BREAKS)
def test_chop_slices_sit_on_the_16th_grid_cover_every_bar_and_follow_the_seed(name):
    beats = kn.SAMPLE_CATALOG[name].beats
    variants = set()
    for seed in range(12):
        part = samples.breakbeat_chop(name, random.Random(seed))
        assert part == samples.breakbeat_chop(name, random.Random(seed)), "детерминирована от сида"
        steps = part.grid.steps
        assert len(steps) == samples.BREAK_CYCLE_BARS * STEPS_PER_BAR
        starts = [i for i, st in enumerate(steps) if st.on]
        assert len(part.chop) == len(starts) and starts[0] == 0 and part.chop[0] == 0.0
        gaps = [b - a for a, b in zip(starts, starts[1:] + [len(steps)])]
        assert set(gaps) <= {1, 2}, "кусок — восьмая или 16-я, без дыр: такт покрыт"
        for bar in range(samples.BREAK_CYCLE_BARS):  # каждая доля такта начинается куском
            assert all(bar * STEPS_PER_BAR + beat * 4 in starts for beat in range(4))
        assert all(0 <= p < beats and (p * 4).is_integer() for p in part.chop), "кусок — 16-я внутри файла"
        variants.add(part.chop)
    assert len(variants) >= 10, "сид меняет нарезку"


@pytest.mark.parametrize("style", STYLES)
def test_set_tempo_window_kick_and_break_pools(style):
    table = kn.STYLES[style]
    lo, hi = {"breaks": (128, 140), "dnb": (160, 176)}[style]
    bpms = [w.bpm for w in table.genre_windows.values()]
    assert min(b for b, _ in bpms) == lo and max(b for _, b in bpms) == hi
    assert kn.genre_style(table, next(iter(table.genre_windows))) == table, "поля стиля — первое окно"
    assert not set(table.genre_windows) & set(kn.STYLES["club"].genre_windows), "окна не путаются с окнами клуба"
    for seed in range(10):
        plan = _plan("космос", seed, style)
        window = table.genre_windows[plan.genre]
        assert window.bpm[0] <= plan.bpm <= window.bpm[1] and lo <= plan.bpm <= hi, (seed, plan.genre, plan.bpm)
        assert all(t.kick in window.kick_pool and kn.KICK_SOUNDS[t.kick].measured for t in plan.tracks)
    for window in table.genre_windows.values():  # бас мимо бочки дропа и долей (ADR-0149 §3.4)
        drop = next(v for t, v in window.looks if t >= 7)
        kicks = {i for i, ch in enumerate(drop.kick) if ch == "X"}
        for figure in window.bass_figures:
            assert not set(kn.BASS_FIGURES[figure].steps) & (kicks | {0, 4, 8, 12}), (figure, drop.kick)


@pytest.mark.parametrize("style", STYLES)
def test_track_plays_a_chopped_pack_break_and_its_events_are_the_model(style):
    for seed in range(2):
        plan = _plan("киберпанк", seed, style)
        history: list = []
        for no in (1, 2):
            track = compose(plan, no, history=history)
            validate(track)
            loop = track.parts["loop"]
            assert loop.synth_or_sample in BREAKS and loop.chop, "брейк пака нарезкой"
            assert "sample" not in track.parts, "psr-слоя DJ_Dave у брейк-стилей нет"
            assert kick_name(track.parts["kick"].sample, track.parts["kick"].play_symbol) in table_kicks(style)
            row = track_composition(track)
            assert row["style"] == style and row["sample"] == loop.synth_or_sample
            assert track.mix.a9_model >= kn.STYLES[style].a9_model_low, (seed, no, track.mix.a9_model)
            program = render(track, "A")
            _, events = program_events(program.code, program.form_beats, loops=True)
            got = sorted((e.beat, e.pos, e.sus_beats) for e in events if e.sample == kn.SAMPLE_CATALOG[
                loop.synth_or_sample].loop_arg)
            assert got == _model_events(track), "события лупа = модель нарезки"
            assert f"tempo={kn.SAMPLE_CATALOG[loop.synth_or_sample].bpm}" in program.code  # rate = bpm сета / tempo
            history.insert(0, {"sample": loop.synth_or_sample, **row})


def table_kicks(style: str) -> set:
    return {k for w in kn.STYLES[style].genre_windows.values() for k in w.kick_pool}


def _model_events(track):
    """(доля, кусок, sus) запусков лупа модели в секциях, где луп звучит: сетка цикла по всей форме."""
    loop = track.parts["loop"]
    steps = loop.grid.steps
    starts = [i for i, st in enumerate(steps) if st.on]
    sections = []
    for sec in track.form.sections:
        sections += [kn.STYLES[track.style].layer_sections["loop"].count(sec.name) > 0] * (sec.bars * STEPS_PER_BAR)
    out = []
    for cycle in range(len(sections) // len(steps)):
        for k, s in enumerate(starts):
            step = cycle * len(steps) + s
            if sections[step]:
                nxt = starts[k + 1] if k + 1 < len(starts) else len(steps)
                out.append((step / 4, loop.chop[k], (nxt - s) / 4))
    return sorted(out)


@pytest.mark.parametrize("text,style", [
    ("включи брейкбит сет на тему космос", "breaks"), ("брейкс про город", "breaks"),
    ("включи брейк бит", "breaks"), ("включи драм-н-бейс сет на тему киберпанк", "dnb"),
    ("драм н бейс", "dnb"), ("драм энд бейс", "dnb"), ("drum and bass", "dnb"), ("давай днб", "dnb"),
    ("dnb set", "dnb"), ("джангл про котов", "dnb"), ("включи драмнбейс", "dnb"),
])
def test_style_words_pick_the_style(text, style):
    assert match_style_text(text) == style


@pytest.mark.parametrize("text", ["драма про любовь", "мелодрама", "сет про джунгли", "брейкданс", "брейк трека",
                                  "бейсбол", "космос"])
def test_words_close_to_style_words_are_not_a_style(text):
    assert match_style_text(text) is None


def test_multiword_style_words_are_marked_whole_and_leave_the_theme():
    words = "включи драм н бейс сет на тему лес".split()
    assert style_marks(words) == [None, "dnb", "dnb", "dnb", None, None, None, None]


def _events_per_second(style: str) -> float:
    """Худшая секция (запусков синтов в секунду, с голосами и нотами аккордов) по 2 сидам × 2 трека."""
    worst = 0.0
    for seed in range(2):
        plan = _plan("космос", seed, style)
        for no in (1, 2):
            track = compose(plan, no)
            program = render(track, "A")
            _, events = program_events(program.code, program.form_beats, loops=True)
            start = 0
            for sec in track.form.sections:
                b0, b1 = start * 4, (start + sec.bars) * 4
                start += sec.bars
                n = sum(1 for e in events if b0 <= e.beat < b1)
                worst = max(worst, n / ((b1 - b0) * 60.0 / track.bpm))
    return worst


def test_dnb_at_170_stays_under_the_club_event_rate():
    """ADR-0153 В8 открыт — держим потолок клуба: dnb 160–176 без psr-слоя и ``pumped16`` запускает синты реже клуба."""
    assert _events_per_second("dnb") < _events_per_second("club")
