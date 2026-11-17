"""PR-8 ADR-0149 (§3.12, §7.2 A4): блэнд двух треков сета — на событиях нот двух дек, без снапшотов.

Уходящий трек N на деке A играет одну форму (дека снимается на её границе), входящий N+1 на деке B встаёт за
``blend_bars(N, N+1)`` тактов до неё. Свойства: оба трека слышны ≥ 4 такта, в каждом такте блэнда ровно одна
бочка и один бас (никогда двух и никогда ни одной), ноты двух басов не перекрываются, у входящего лид в блэнде
молчит, вход — с первой доли формы входящего (фаза 0 — по ``start=`` секций, а не по доле клока).
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import replace

import pytest

from melodies import MELODIES, profile
from rob_box_music.arrange.compose import compose
from rob_box_music.model import BEATS_PER_BAR, Form, Section, Transition, blend_bars, roles_at_bar, validate
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

THEMES = ("космос", "киберпанк", "детский праздник", "славянская вечеринка", "")
SEEDS = range(6)


def _plan(theme, seed):
    return seeded_plan(seeded_profile(theme), seed, set_id="b")


def _roles(track, deck, shift):
    """События по ролям одной формы трека; доли — от начала формы уходящего (``shift`` — доля входа)."""
    program = render(track, deck)
    _parsed, events = program_events(program.code, program.form_beats)
    slot_role = {slot: role for role, slot in program.slots.items()}
    out = defaultdict(list)
    for ev in events:
        out[slot_role[ev.slot]].append(replace(ev, beat=ev.beat + shift))
    return out


def _pair(plan, no, melodies=None):
    leaving = compose(plan, no, deck="A", melodies=melodies)
    incoming = compose(plan, no + 1, deck="B", melodies=melodies)
    return leaving, incoming


def _blend_window(leaving, incoming):
    bars = blend_bars(leaving, incoming)
    form = float(leaving.form.bars_total * BEATS_PER_BAR)
    return bars, form - bars * BEATS_PER_BAR, form


@pytest.mark.parametrize("theme", THEMES)
def test_consecutive_set_tracks_blend_for_eight_bars(theme):
    """Любые соседние треки сета сводятся: 8 тактов (4–8), один темп, своп на такте свопа."""
    for seed in SEEDS:
        plan = _plan(theme, seed)
        for no in range(1, 6):
            leaving, incoming = _pair(plan, no)
            assert blend_bars(leaving, incoming) == 8, (theme, seed, no)


@pytest.mark.parametrize("seed", SEEDS)
def test_one_kick_one_bass_and_both_tracks_heard_in_every_blend_bar(seed):
    plan = _plan("космос", seed)
    for no in (1, 2, 3):
        leaving, incoming = _pair(plan, no, MELODIES if no % 2 else None)
        bars, at, end = _blend_window(leaving, incoming)
        old = _roles(leaving, "A", 0.0)
        new = _roles(incoming, "B", at)
        both_heard = 0
        for bar in range(bars):
            lo, hi = at + bar * BEATS_PER_BAR, at + (bar + 1) * BEATS_PER_BAR

            def decks(role):
                return {name for name, roles in (("A", old), ("B", new))
                        if any(lo <= e.beat < hi for e in roles.get(role, ()))}

            assert len(decks("kick")) == 1, f"такт {bar} блэнда: бочек {decks('kick')}"
            assert len(decks("bass")) == 1, f"такт {bar} блэнда: басов {decks('bass')}"
            assert not decks("lead") & {"A"}, "в хвосте уходящего нет лида"
            heard_a = any(lo <= e.beat < hi for evs in old.values() for e in evs)
            heard_b = any(lo <= e.beat < hi for evs in new.values() for e in evs)
            both_heard += heard_a and heard_b
        assert both_heard >= 4, f"оба трека слышны {both_heard} тактов (A4: ≥ 4)"
        # ноты двух басов не перекрываются ни на долю: бас уходящего гаснет до первой ноты входящего
        last_old = max(e.beat + e.sus_beats for e in old["bass"] if e.beat < end)
        first_new = min(e.beat for e in new["bass"])
        assert last_old <= first_new + 1e-9 and at <= first_new < end
        # бочка уходящего кончилась до первой бочки входящего; после границы формы звучит только входящий
        assert max(e.beat for e in old["kick"]) < min(e.beat for e in new["kick"])
        assert all(e.beat < end for evs in old.values() for e in evs)


def test_incoming_track_enters_with_its_own_intro_from_phase_zero():
    """Первое событие входящего — на доле входа (фаза 0); первые такты — хэты и пэд, бочка/бас — с такта свопа."""
    plan = _plan("киберпанк", 3)
    leaving, incoming = _pair(plan, 2)
    bars, at, _end = _blend_window(leaving, incoming)
    new = _roles(incoming, "B", at)
    assert min(e.beat for evs in new.values() for e in evs) == at
    swap = incoming.transition_in.bass_swap_bar
    for role in ("kick", "bass"):
        assert min(e.beat for e in new[role]) >= at + swap * BEATS_PER_BAR
    assert roles_at_bar(incoming, 0) == {"hats", "pad"}
    assert roles_at_bar(leaving, leaving.form.bars_total - 1) == {"hats", "pad"}


def test_no_blend_when_forms_do_not_mix():
    """Формы не сводятся — ``blend_bars == 0`` (движок делает стык встык на границе формы, PR-5)."""
    plan = _plan("космос", 1)
    leaving, incoming = _pair(plan, 1)
    other = Transition(16, 4, True)
    assert blend_bars(leaving, replace(incoming, transition_in=other)) == 0  # длины блэнда разные
    assert blend_bars(leaving, replace(incoming, bpm=incoming.bpm + 1)) == 0  # темп один на сет
    no_low = tuple(replace(s, roles=s.roles - {"kick", "bass"}) if s.name == "intro_low" else s
                   for s in incoming.form.sections)
    assert blend_bars(leaving, replace(incoming, form=Form(no_low))) == 0  # был бы такт без бочки и баса
    two_basses = tuple(replace(s, roles=s.roles | {"bass"}) if s.name == "intro" else s
                       for s in incoming.form.sections)
    assert blend_bars(leaving, replace(incoming, form=Form(two_basses))) == 0  # два баса разом


def test_split_outro_is_a_valid_dj_outro():
    """Аутро делится под своп (``outro`` + ``outro_tail``) — вместе ≥ 8 тактов без лида; короче — ошибка."""
    track = compose(_plan("космос", 2), 1)
    validate(track)
    tail = [s for s in track.form.sections if s.name.startswith("outro")]
    assert sum(s.bars for s in tail) == 8 and all("lead" not in s.roles for s in tail)
    short = Form(track.form.sections[:-2] + (Section("drop3", 4, 9, frozenset({"kick", "hats"})),
                                             Section("outro_tail", 4, 2, frozenset({"hats", "pad"}))))
    with pytest.raises(Exception, match="outro"):
        validate(replace(track, form=short))


def test_seeded_profile_tracks_from_plan_profile_blend_too():
    """Трек темы с хуком и без — те же формы входа/выхода (развитие хука не трогает интро/аутро)."""
    plan = seeded_plan(profile(root=4, hooks=("long",)), 5, set_id="h")
    leaving = compose(plan, 1, melodies=MELODIES)
    incoming = compose(plan, 2, deck="B")
    assert leaving.hook is not None and incoming.hook is None
    assert blend_bars(leaving, incoming) == 8
