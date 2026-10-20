"""PR-4 ADR-0152 (§3.4, #3435): пул бочек стиля, выбор в плане со штрафом, одна бочка на такт блэнда."""

from __future__ import annotations

import math
import random
from collections import defaultdict
from dataclasses import replace

import pytest

from parallel import pmap
from rob_box_music import knowledge as kn
from rob_box_music.arrange import mix
from rob_box_music.arrange.compose import compose
from rob_box_music.diversity import kick_name, track_composition
from rob_box_music.model import BEATS_PER_BAR, TrackError, blend_bars, validate
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import pick_kick, plan_kicks, plan_salt, seeded_plan
from rob_box_music.theme import seeded_profile

STYLE = kn.STYLES[kn.DEFAULT_STYLE]
THEMES = ("космос", "киберпанк", "детский праздник", "славянская вечеринка", "")


def _plan(theme, seed):
    return seeded_plan(seeded_profile(theme), seed, set_id="k")


def test_pool_has_measured_kicks_with_a_real_low_end():
    """Пул стиля — 3–4+ бочки из таблицы, у каждой низ замера (доля < 250 Гц ≥ 0.8, < 120 Гц ≥ 0.6) и свой файл."""
    assert 3 <= len(STYLE.kick_pool) <= 4 and set(STYLE.kick_pool) <= set(kn.KICK_SOUNDS)
    play = [k for k in kn.KICK_SOUNDS.values() if not k.pack]
    assert len({k.sample for k in play}) == len(play), "номера сэмплов разные"
    for name, kick in kn.KICK_SOUNDS.items():
        # sample=0 рендер отбрасывает; бочка-файл пака (ADR-0153 S4) — ключ каталога с ролью kick
        assert kick.low >= 0.8 and (kick.sample > 0 if not kick.pack
                                    else kn.SAMPLE_CATALOG[kick.pack].role == "kick"), name
        # ADR-0153 S1: бочки рейва на роботе не мерены — ``low`` по файлу, ``sub`` неизвестна (не выдумана)
        assert kick.sub >= 0.6 if kick.measured else math.isnan(kick.sub), name


@pytest.mark.parametrize("theme", THEMES)
def test_kick_of_every_track_is_from_the_style_pool(theme):
    for seed in range(5):
        plan = _plan(theme, seed)
        for no in range(1, 13):  # за пределами плана (10) — выбор в compose по истории
            track = compose(plan, no)
            assert kick_name(track.parts["kick"].sample) in STYLE.kick_pool, (theme, seed, no)
            assert track_composition(track)["kick"] in STYLE.kick_pool
            validate(track)


def test_plan_carries_the_kick_and_compose_plays_it():
    plan = _plan("космос", 4)
    assert [t.kick for t in plan.tracks] == list(plan_kicks(plan.table, 4, plan_salt(plan.profile), len(plan.tracks)))
    for step in plan.tracks:
        track = compose(plan, step.no)
        assert track.parts["kick"].sample == kn.KICK_SOUNDS[step.kick].sample


def test_kick_changes_between_neighbours_and_across_sets():
    """Подряд один трек не повторяет бочку; за 30 сидов бочек не меньше двух (в пуле все)."""
    seen = set()
    for seed in range(30):
        kicks = [t.kick for t in _plan("киберпанк", seed).tracks]
        assert all(a != b for a, b in zip(kicks, kicks[1:])), (seed, kicks)
        seen.add(kicks[0])
    assert len(seen) >= 2, seen
    used = {t.kick for seed in range(30) for t in _plan("космос", seed).tracks}
    assert used == set(STYLE.kick_pool)


def test_recent_kick_is_penalised_across_sets():
    """История ``kick`` штрафует: сыгранная бочка в первом треке следующего сета выпадает реже остальных."""
    history = [{"kick": "house"}, {"kick": "deep"}]
    firsts = [pick_kick(STYLE, history, random.Random(s)) for s in range(300)]
    share = {k: firsts.count(k) / len(firsts) for k in STYLE.kick_pool}
    assert share["house"] == 0.0, "прошлый трек подряд не повторяется"
    assert share["deep"] < min(share["techno"], share["garage"]), share


def test_validator_rejects_a_kick_outside_the_table():
    track = compose(_plan("космос", 1), 1)
    with pytest.raises(TrackError, match="parts.kick.sample"):
        validate(replace(track, parts={**track.parts, "kick": replace(track.parts["kick"], sample=99)}))
    with pytest.raises(ValueError, match="не из пула"):
        mix.kick_sound(STYLE, "nope")


def test_one_kick_per_blend_bar_when_the_neighbours_have_different_kicks():
    """Бочка уходящего гаснет там, где входит бочка входящего — и это разные файлы: в такте блэнда одна бочка.
    Шесть сидов независимы и считаются параллельно (#3504); ошибка сида пробрасывается как есть."""
    pmap(_one_kick_per_blend_bar, range(6))


def _one_kick_per_blend_bar(seed):
    plan = _plan("космос", seed)
    checked = 0
    for no in range(1, len(plan.tracks)):
        leaving, incoming = compose(plan, no, deck="A"), compose(plan, no + 1, deck="B")
        if leaving.parts["kick"].sample == incoming.parts["kick"].sample:
            continue
        bars = blend_bars(leaving, incoming)
        assert bars == 8
        at = float(leaving.form.bars_total * BEATS_PER_BAR) - bars * BEATS_PER_BAR
        hits = defaultdict(set)
        for deck, track, shift in (("A", leaving, 0.0), ("B", incoming, at)):
            program = render(track, deck)
            _parsed, events = program_events(program.code, program.form_beats)
            slot = program.slots["kick"]
            for ev in events:
                if ev.slot == slot and ev.beat + shift >= at:
                    hits[int((ev.beat + shift - at) // BEATS_PER_BAR)].add((deck, ev.sample))
        assert hits, "в блэнде есть бочка"
        for bar, kicks in hits.items():
            assert len({deck for deck, _s in kicks}) == 1, (seed, no, bar, kicks)
        checked += 1
    assert checked >= 3, "в плане должны быть соседи с разными бочками"
