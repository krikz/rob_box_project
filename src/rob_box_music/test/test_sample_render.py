"""Сэмплы DJ_Dave в рендере как у неё (#3432): удар и кусок не наползают на следующий, луп и psr под сайдчейном,
файл пула громче эталона уровня — тише на разницу по каталогу.

Отзыв Шифу 05.10 по треку «терминатора» (set26040 трек 1): «шумы — сэмпл использовался неправильно». В записи:
клэп ddm110 в psr-пуле (−5.5 дБ среднего против −15.2 у эталона psr_07) на тех же ``amp``, что psr; синт ``loop``
Renardo нарастал 50 мс и звучал 50 мс после ``sus`` (файл 77 мс начинался заново внутри спада).
"""

from __future__ import annotations

import itertools
from collections import defaultdict
from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange.compose import club_track
from rob_box_music.arrange.mix import duck_envelope, file_gain
from rob_box_music.model import BEATS_PER_BAR, STEPS_PER_BAR
from rob_box_music.render.events import program_events, run_program
from rob_box_music.render.renardo import render

SEEDS = range(8)
SAMPLE_SLOT_ROLES = ("sample", "loop", "fx")


def _events(track):
    program = render(track, "A")
    _, events = program_events(program.code, program.form_beats, loops=True)
    by_slot = defaultdict(list)
    for ev in events:
        by_slot[ev.slot].append(ev)
    return program, by_slot


def _look(track, beat):
    """Сайдчейн секции на доле ``beat``."""
    starts = list(itertools.accumulate(sec.bars * BEATS_PER_BAR for sec in track.form.sections))
    return track.mix.duck[next(i for i, end in enumerate(starts) if beat < end)]


@pytest.mark.parametrize("seed", SEEDS)
def test_each_hit_and_piece_ends_before_the_next_starts(seed):
    """``legato(1)``: событие сэмпла звучит не дольше шага до следующего события того же голоса; края огибающей
    (``knowledge.SAMPLE_EDGE_S``) — внутри ``sus`` у каждого плеера ``loop``."""
    track = club_track(seed)
    program, by_slot = _events(track)
    players = run_program(program.code).players
    sec_per_beat = 60.0 / track.bpm
    for role in SAMPLE_SLOT_ROLES:
        slot = program.slots[role]
        kw = players[slot].kwargs
        assert (kw["atk"], kw["rel"]) == kn.SAMPLE_EDGE_S, role
        voices = defaultdict(list)
        for ev in by_slot[slot]:
            voices[ev.pan].append(ev)
        for evs in voices.values():
            evs.sort(key=lambda e: e.beat)
            for a, b in zip(evs, evs[1:]):
                assert a.sus_beats <= b.beat - a.beat + 1e-9, (role, a.beat, a.sus_beats, b.beat)
            assert all(sum(kn.SAMPLE_EDGE_S) < ev.sus_beats * sec_per_beat for ev in evs), role


@pytest.mark.parametrize("seed", SEEDS)
def test_loop_pieces_dip_on_the_kick_like_pad_and_psr(seed):
    """Луп нарезкой — под той же огибающей, что пэд, бас и psr: кусок на ударе триггера тише куска между ударами;
    куски на сетке восьмых, ``sus`` — восьмая."""
    track = club_track(seed)
    assert "loop" in track.mix.duck_roles
    program, by_slot = _events(track)
    events = by_slot[program.slots["loop"]]
    assert events
    for ev in events:
        assert ev.beat * 2 == int(ev.beat * 2) and ev.sus_beats == 0.5
    for duck in set(track.mix.duck):
        mine = [ev for ev in events if _look(track, ev.beat) == duck]
        on_kick = [ev.amp for ev in mine if round(ev.beat * 4) % STEPS_PER_BAR in duck.trigger]
        between = [ev.amp for ev in mine if round(ev.beat * 4) % STEPS_PER_BAR not in duck.trigger]
        if mine and duck.depth > 0:
            assert on_kick and between and max(on_kick) < min(between), duck


@pytest.mark.parametrize("seed", SEEDS)
def test_pool_file_louder_than_the_level_file_is_attenuated_by_catalog(seed):
    """``amplify`` удара psr = акцент × сайдчейн × ``file_gain`` его файла: громкость удара по каталогу
    (``amp`` × 10^(mean_db/20)) не выше эталона уровня партии."""
    track = club_track(seed)
    part = track.parts["sample"]
    ref = kn.SAMPLE_CATALOG[part.synth_or_sample]
    by_path = {kn.SAMPLE_CATALOG[n].loop_arg: n for n in part.pool}
    accents = [st.accent for st in part.grid.steps]
    program, by_slot = _events(track)
    events = by_slot[program.slots["sample"]]
    assert events
    for ev in events:
        step = round(ev.beat * 4)
        name = by_path[ev.sample]
        accent = kn.ACCENT_AMPLIFY[accents[step % len(accents)]] if len(set(accents)) > 1 else 1.0
        duck = _look(track, ev.beat)
        want = accent * duck_envelope(duck.trigger, duck.depth)[step % STEPS_PER_BAR] * file_gain(name, ref.name)
        assert ev.amp / ev.gate == pytest.approx(want, abs=1.5e-3), (ev.beat, name)
        level = ev.amp / ev.gate * 10 ** (kn.SAMPLE_CATALOG[name].mean_db / 20)
        assert level <= accent * 10 ** (ref.mean_db / 20) + 1e-3


def test_terminator_clap_is_pulled_down_to_the_psr_level():
    """Пул трека «терминатора» (set26040:01): клэп ddm110 (−5.5 дБ) против эталона psr_07 (−15.2) — тише на
    9.7 дБ при том же акценте и сайдчейне; psr_24 (−33.4, файл 3.1 с) не поднимается."""
    track = club_track(0)
    pool = ("dirt_psr_07", "ddm110_cp_Clap", "dirt_psr_24", "dirt_psr_10")
    part = replace(track.parts["sample"], synth_or_sample="dirt_psr_07", pool=pool * 8)
    track = replace(track, parts={**track.parts, "sample": part})
    accents = [st.accent for st in part.grid.steps]
    program, by_slot = _events(track)
    ratio = defaultdict(set)
    for ev in by_slot[program.slots["sample"]]:
        step = round(ev.beat * 4)
        duck = _look(track, ev.beat)
        shape = kn.ACCENT_AMPLIFY[accents[step % len(accents)]] * duck_envelope(duck.trigger, duck.depth)[
            step % STEPS_PER_BAR]
        if shape >= 0.3:  # округление amplify до 3 знаков
            ratio[ev.sample].add(round(ev.amp / ev.gate / shape, 2))
    cat = kn.SAMPLE_CATALOG
    assert ratio[cat["dirt_psr_07"].loop_arg] == {1.0} == ratio[cat["dirt_psr_24"].loop_arg]
    assert ratio[cat["ddm110_cp_Clap"].loop_arg] == {round(10 ** (-9.7 / 20), 2)}
