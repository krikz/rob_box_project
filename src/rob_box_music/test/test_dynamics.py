"""PR-7 ADR-0149: динамика — вид секции (бочка + сайдчейн одной осью энергии), LPF-свип build→drop, trim энергии.

Свойства — на событиях ``render.events.program_events`` (что услышит scsynth), без снапшотов. Приёмы DJ_Dave
(интервью и код, присланные Шифу 02.10): «одна переменная переключает и рисунок бочки, и форму сайдчейна»,
«LPF на басе и нотах — один слайдер».
"""

from __future__ import annotations

from collections import defaultdict

import pytest

from melodies import MELODIES, compose_p, profile
from rob_box_music import knowledge as kn
from rob_box_music.arrange import mix
from rob_box_music.model import BEATS_PER_BAR
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render

SEEDS = range(8)
STEP = 0.25


def _events(track):
    program = render(track, "A")
    _p, events = program_events(program.code, program.form_beats)
    slot_role = {slot: role for role, slot in program.slots.items()}
    by_role = defaultdict(list)
    for ev in events:
        by_role[slot_role[ev.slot]].append(ev)
    return by_role


def _spans(track):
    """{имя секции: (секция, первая доля, доля конца)}."""
    out, start = {}, 0.0
    for sec in track.form.sections:
        end = start + sec.bars * BEATS_PER_BAR
        out[sec.name] = (sec, start, end)
        start = end
    return out


def _in(events, span):
    _sec, start, end = span
    return sorted((e for e in events if start - 1e-9 <= e.beat < end - 1e-9), key=lambda e: e.beat)


def _track(seed, track_no):
    return compose_p(profile(root=seed % 12), track_no, set_seed=seed, melodies=MELODIES if seed % 2 else None)


def test_looks_switch_kick_and_sidechain_on_one_energy_axis():
    """Чем выше энергия секции, тем «насос» глубже; build — бочка на 1 и 3, дроп и интро/аутро — прямая."""
    looks = [mix.look(kn.STYLES["club"], e) for e in range(11)]
    assert [lk.duck_depth for lk in looks[:5]] == [0.6] * 5 and looks[5] == looks[6] and looks[7] == looks[10]
    assert looks[5].duck_depth < looks[0].duck_depth < looks[7].duck_depth == 1.0
    assert mix.kick_steps(looks[7].kick) == (0, 4, 8, 12) == mix.kick_steps(looks[0].kick)
    assert mix.kick_steps(looks[5].kick) == (0, 8)


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("track_no", [2, 3, 4])  # энергии 3, 4, 5
def test_build_and_drop_differ_in_kick_and_pump(seed, track_no):
    """Вид секции слышен в событиях: бочка build-а реже бочки дропа, провал пэда в дропе глубже (кроме трека
    энергии 5, где build уже «дроповый»); интро/аутро — прямая бочка под блэнд."""
    track = _track(seed, track_no)
    by_role, spans = _events(track), _spans(track)
    build, drop = spans["build"][0], spans["drop"][0]
    assert track.mix.duck[[s.name for s in track.form.sections].index("drop")].depth == 1.0

    def kicks_per_bar(name):
        sec, start, end = spans[name]
        hits = [e for e in _in(by_role["kick"], spans[name]) if e.beat < end - BEATS_PER_BAR]  # без такта fill-а
        return len(hits) / (sec.bars - 1)

    def deepest_pad_dip(name):
        return min(e.amp / e.gate for e in _in(by_role["pad"], spans[name]))

    assert kicks_per_bar("drop") == 4 and kicks_per_bar("intro_low") == 4 and kicks_per_bar("outro") == 4
    if mix.look(kn.STYLES["club"], build.energy) == mix.look(kn.STYLES["club"], drop.energy):
        assert kicks_per_bar("build") == 4 and track.energy == 5
    else:
        assert kicks_per_bar("build") == 2
        # провал пэда на 16-х — у pumped16; stabs (удар на «и») и held (без насоса) — ``test_pad_figures``
        if track.history_key.pad_figure == "pumped16":
            assert deepest_pad_dip("drop") < deepest_pad_dip("build") - 0.1
        build_kicks = {round((e.beat - spans["build"][1]) % BEATS_PER_BAR / STEP) for e in
                       _in(by_role["kick"], spans["build"])}
        assert build_kicks <= {0, 8}, "build: бочка на 1 и 3"


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("track_no", [1, 3, 4])
def test_lpf_opens_through_the_build_is_off_in_drops_and_half_closed_in_the_break(seed, track_no):
    """Один «слайдер» на басе, пэде и лиде: в build срез монотонно растёт 400→4000 Гц, в дропах фильтра нет,
    в брейке прикрыт (1200 Гц) — у всех ролей одна и та же кривая в одну долю."""
    track = _track(seed, track_no)
    by_role, spans = _events(track), _spans(track)
    curves = {}
    for role in kn.STYLES["club"].lpf_roles:
        build = _in(by_role[role], spans["build"])
        if not build:
            continue
        cut = [e.fx["lpf"] for e in build]
        assert all(a <= b for a, b in zip(cut, cut[1:])), (role, cut)
        assert 400 <= cut[0] and cut[-1] <= kn.LPF_TOP_HZ
        # бас и пэд звучат весь build; лид — только начало хука (развитие build); held-пэд берёт срез на атаке
        # аккорда — раз в 2 такта (ADR-0152 §3.2), последний аккорд build-а — за 2 такта до дропа
        if role == "bass" or (role == "pad" and track.history_key.pad_figure != "held"):
            assert cut[0] == pytest.approx(400, rel=0.02) and cut[-1] == pytest.approx(kn.LPF_TOP_HZ, rel=0.02)
        elif role == "pad":
            assert cut[0] == pytest.approx(400, rel=0.02)
        curves[role] = {e.beat // 1: e.fx["lpf"] for e in build}
        for name in ("drop", "drop2"):
            assert all("lpf" not in e.fx for e in _in(by_role[role], spans[name])), (role, name)
        assert {e.fx.get("lpf") for e in _in(by_role[role], spans["break"])} <= {1200.0}
    beats = set.intersection(*(set(c) for c in curves.values()))
    assert beats and all(len({c[b] for c in curves.values()}) == 1 for b in beats), "одна кривая на все роли"


@pytest.mark.parametrize("seed", SEEDS)
def test_leaving_tail_closes_4000_to_300_on_everything_but_the_kick(seed):
    """ADR-0149 §3.12: после свопа баса остальное уходящей деки (пэд, хэты) уходит в фильтр 4000→300 за 4 такта."""
    track = _track(seed, 2)
    by_role, spans = _events(track), _spans(track)
    for role in ("pad", "hats"):
        cut = [e.fx["lpf"] for e in _in(by_role[role], spans["outro_tail"])]
        assert cut and all(a >= b for a, b in zip(cut, cut[1:])), role
        assert cut[0] == pytest.approx(4000, rel=0.02)
        if not (role == "pad" and track.history_key.pad_figure == "held"):  # held: атака раз в 2 такта
            assert cut[-1] == pytest.approx(300, rel=0.03)
        assert all("lpf" not in e.fx for e in _in(by_role[role], spans["drop"])), role
    assert "kick" not in track.mix.lpf and all(not e.fx for e in by_role["kick"])
    assert all("lpf" not in e.fx for e in by_role["hats"] if e.beat < spans["outro_tail"][1]), "хэты — только хвост"


def test_trim_follows_track_energy_and_never_boosts():
    """``trim`` мастер-шины — по энергии трека сета (ADR-0147 §3.2): пик 0 дБ, тише — ниже; подъёма нет (A7)."""
    trims = [mix.set_master(e)["trim"] for e in kn.ENERGY_LEVELS]
    assert trims == [kn.ENERGY_TRIM_DB[e] for e in kn.ENERGY_LEVELS]
    assert all(a < b for a, b in zip(trims, trims[1:])) and trims[-1] == 0.0 and max(trims) <= 0.0
    assert trims[-1] - trims[0] >= 6.0, "A6: волна энергии сама даёт ≥ 6 дБ между спадом и пиком"
    assert set(mix.set_master(3)) == {"trim"} | set(kn.SET_LEVELER)
    assert set(kn.SET_LEVELER) | set(kn.DJ_LEVELER) <= set(kn.MASTER_DEFAULTS), "профиль — ручки SynthDef"
    assert kn.MASTER_DEFAULTS["trim"] == 0.0, "дефолт — v1 звучит как раньше"


@pytest.mark.parametrize("seed", SEEDS)
def test_power_of_two_kick_pattern_with_masked_hits(seed):
    """Рисунок бочки — период степени двойки (санитайзер v1 #1803 не трогает), лишние удары build-а глушит
    ``amplify`` 0, а не потерянная фаза: удары по событиям совпадают с моделью."""
    track = _track(seed, 3)
    program = render(track, "A")
    line = next(ln for ln in program.code.splitlines() if ln.startswith(program.slots["kick"] + " "))
    steps = len(line.split('play("')[1].split('"')[0])
    assert steps & (steps - 1) == 0
    model = {i * STEP for i, st in enumerate(track.parts["kick"].grid.steps) if st.on}
    active = set()
    for sec, start, end in _spans(track).values():
        if "kick" in sec.roles:
            active |= {b for b in model if start <= b < end}
    assert {round(e.beat, 6) for e in _events(track)["kick"]} == {round(b, 6) for b in active}
