"""PR-3c ADR-0149: микс — сайдчейн-огибающая, уровни ролей из одной таблицы, тембр темы, бочка жанра.

Свойства проверяются на событиях ``render.events.program_events`` (``amp`` события = ``amp × amplify``), без снапшотов.
"""

from __future__ import annotations

import random
from collections import defaultdict
from dataclasses import replace

import pytest

from melodies import MELODIES, compose_p, profile
from rob_box_music import knowledge as kn
from rob_box_music.arrange import mix
from rob_box_music.arrange.compose import compose
from rob_box_music.model import STEPS_PER_BAR, TrackError, validate
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

SEEDS = range(12)
FOUR_ON_FLOOR = (0, 4, 8, 12)


def _track(seed, track_no=1):
    hooks = MELODIES if seed % 2 == 0 else None
    return compose_p(profile(root=seed % 12), track_no, set_seed=seed, melodies=hooks)


def _events(track):
    program = render(track, "A")
    _p, events = program_events(program.code, program.form_beats)
    slot_role = {slot: role for role, slot in program.slots.items()}
    by_role = defaultdict(list)
    for ev in events:
        by_role[slot_role[ev.slot]].append(ev)
    return by_role


def _step(beat):
    return int(round(beat * 4)) % STEPS_PER_BAR


@pytest.mark.parametrize("trigger", [FOUR_ON_FLOOR, (0, 10), (0, 3, 6, 10, 13), (4, 12)])
@pytest.mark.parametrize("depth", [1.0, 0.6, 0.3])
def test_duck_envelope_rises_without_a_step(trigger, depth):
    """Огибающая: провал только на шаге удара, дальше монотонный подъём шагами ≤ шага формы — не «ступенька»
    на всю ноту (v1 ``pump_weights``: 0.25 на ударе, 0.75 везде, без подъёма)."""
    env = mix.duck_envelope(trigger, depth)
    shape = kn.SIDECHAIN_SHAPE
    max_rise = depth * max(b - a for a, b in zip(shape, shape[1:])) + 1e-9
    assert len(env) == STEPS_PER_BAR and all(0 < g <= 1.0 for g in env)
    for i in range(STEPS_PER_BAR):
        prev, cur = env[i - 1], env[i]
        if i in trigger:
            assert cur == pytest.approx(1.0 - depth * (1.0 - shape[0]))
        else:
            assert prev <= cur <= prev + max_rise, (i, env)
    gaps = [(b - a) % STEPS_PER_BAR or STEPS_PER_BAR for a, b in zip(trigger, trigger[1:] + trigger[:1])]
    for t, gap in zip(trigger, gaps):  # полный подъём, если до следующего удара хватает шагов
        if gap >= len(shape):
            assert env[(t + len(shape) - 1) % STEPS_PER_BAR] == pytest.approx(1.0)
    assert mix.duck_envelope(trigger, 0.0) == (1.0,) * STEPS_PER_BAR


@pytest.mark.parametrize("seed", SEEDS)
def test_sidechain_is_on_the_bass_and_pad_events_only(seed):
    """``amp`` события ÷ гейт = акцент × огибающая по шагу такта для баса/пэда; бочка/хэты/лид без сайдчейна."""
    track = _track(seed)
    by_role = _events(track)
    env = mix.duck_envelope(track.mix.duck_trigger, track.mix.duck_depth)
    assert track.mix.duck_roles == frozenset({"bass", "pad"}) and track.mix.duck_trigger == FOUR_ON_FLOOR
    pad = [round(e.amp / e.gate, 3) for e in by_role["pad"]]
    assert set(pad) == set(kn.SIDECHAIN_SHAPE), "пэд — аккорд на каждой 16-й под огибающей"
    for ev in by_role["pad"]:
        assert ev.amp / ev.gate == pytest.approx(env[_step(ev.beat)]) and ev.sus_beats == 0.25
    accents = {(p.beat, p.midi): p.accent for p in track.parts["bass"].pitches}
    for ev in by_role["bass"]:
        want = kn.ACCENT_AMPLIFY[accents[(ev.beat, ev.midi)]] * env[_step(ev.beat)]
        assert ev.amp / ev.gate == pytest.approx(want, abs=1e-3)
    for role in ("kick", "lead"):
        assert {round(e.amp / e.gate, 3) for e in by_role[role]} <= set(kn.ACCENT_AMPLIFY) | {1.0}, role


@pytest.mark.parametrize("seed", SEEDS)
def test_pad_dips_only_on_the_trigger(seed):
    """Пэд проваливается только на шаге удара триггера (рисунок бочки), между ударами только растёт; там, где
    бочка звучит, провал пэда совпадает с ударом бочки."""
    track = _track(seed)
    by_role = _events(track)
    kicks = {round(e.beat, 6) for e in by_role["kick"]}
    pad = sorted({(round(e.beat, 6), round(e.amp / e.gate, 3)) for e in by_role["pad"]})
    dips = [b1 for (_b0, r0), (b1, r1) in zip(pad, pad[1:]) if r1 < r0]
    assert dips and all(_step(b) in FOUR_ON_FLOOR for b in dips)
    assert any(b in kicks for b in dips)
    for (b0, r0), (b1, r1) in zip(pad, pad[1:]):
        if _step(b1) not in FOUR_ON_FLOOR and b1 - b0 <= 0.25 + 1e-9:
            assert r1 >= r0, (b1, r0, r1)


def _max_gap_s(track) -> float:
    """Самое длинное окно без звучащей тональной ноты, с: события [доля, доля + sus] по кругу формы (стык прохода).
    Ударные не считаются: длина звука ``play()`` — длина файла, в модели её нет (хэт ≈ десятки мс)."""
    by_role = _events(track)
    form = float(track.form.bars_total * 4)
    spans = sorted((e.beat % form, e.beat % form + e.sus_beats)
                   for role in kn.TONAL_ROLES for e in by_role[role] if e.amp > 0)
    gap, reach = 0.0, spans[0][1]
    for start, end in spans[1:]:
        gap = max(gap, start - reach)
        reach = max(reach, end)
    gap = max(gap, spans[0][0] + form - reach)  # конец формы → начало следующего прохода
    return gap * 60.0 / track.bpm


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("track_no", [1, 2, 3, 4, 5])
def test_no_silent_window_inside_the_track_or_at_the_form_seam(seed, track_no):
    """ADR-0149 A4/I8: внутри трека и на стыке формы нет окна ≥ 50 мс без звучащей ноты. До PR-3c fill снимал
    бочку на последней доле, а аккорд пэда кончался за полдоли до границы: −180 dBFS ≈ 150 мс (находка PR-5)."""
    assert _max_gap_s(_track(seed, track_no)) < 0.05


@pytest.mark.parametrize("seed", SEEDS)
def test_levels_come_from_one_table(seed):
    """Уровень роли — ``ROLE_LEVEL_DB`` (или честный потолок синта); гейт рендера даёт этот уровень по модели."""
    track = _track(seed, track_no=1 + seed % 5)
    program = render(track, "A")
    for role, part in track.parts.items():
        unit, exponent = mix._unit(role, part)
        amp = mix.level_amp(role, part)
        assert part.level_db <= kn.ROLE_LEVEL_DB[role]
        if part.level_db < kn.ROLE_LEVEL_DB[role]:
            assert amp == pytest.approx(kn.MAX_LAYER_AMP)
        assert mix.layer_db(unit, exponent, amp) == pytest.approx(part.level_db, abs=0.01)
        assert track.mix.level_db[role] == part.level_db
        line = next(ln for ln in program.code.splitlines() if ln.startswith(program.slots[role] + " "))
        assert f"{round(amp, 3):g}" in line.split("amp=var(")[1].split(")")[0], role


def test_same_level_whatever_the_timbre():
    """Смена синта роли не меняет её уровень в модели — меняется ``amp`` (вся разница громкости синтов — в таблице)."""
    track = _track(3)
    for role, synths in (("bass", ("bass", "dub")), ("pad", ("sinepad", "space"))):
        parts = [replace(track.parts[role], synth_or_sample=s) for s in synths]
        leveled = [mix.mix_parts({role: p}, FOUR_ON_FLOOR)[0][role] for p in parts]
        assert leveled[0].level_db == leveled[1].level_db == kn.ROLE_LEVEL_DB[role]
        assert mix.level_amp(role, leveled[0]) != mix.level_amp(role, leveled[1])


@pytest.mark.parametrize("theme", ["космос", "киберпанк", "детский праздник", "славянская вечеринка", "новый год",
                                   "просто вечеринка"])
def test_timbre_follows_the_theme_and_is_deterministic(theme):
    prof = seeded_profile(theme)
    family = kn.TIMBRES[kn.THEME_TIMBRE.get(prof.row or "", kn.DEFAULT_TIMBRE)]
    seen = set()
    for seed in range(8):
        plan = seeded_plan(prof, seed)
        track = compose(plan, 1 + seed % 3)
        synths = {r: track.parts[r].synth_or_sample for r in kn.TONAL_ROLES}
        assert all(synths[r] in family[r] for r in synths), (theme, synths)
        assert synths == {r: compose(plan, 1 + seed % 3).parts[r].synth_or_sample for r in kn.TONAL_ROLES}
        seen.add(tuple(sorted(synths.items())))
    if any(len(v) > 1 for v in family.values()):
        assert len(seen) > 1, "сид меняет тембр внутри семьи темы"
    assert mix.timbres(prof.row, random.Random(5)) == mix.timbres(prof.row, random.Random(5))


def test_timbre_table_is_playable():
    """Каждый синт семьи — из палитры роли, с замером громкости и не ``held``; пэд — без фиксированного хвоста
    (``warmpad`` 1.2 с размазал бы сайдчейн 16-х)."""
    assert set(kn.THEME_TIMBRE) == set(kn.THEMES) and set(kn.THEME_TIMBRE.values()) | {kn.DEFAULT_TIMBRE} <= set(
        kn.TIMBRES)
    for family in kn.TIMBRES.values():
        assert set(family) == set(kn.TONAL_ROLES)
        for role, synths in family.items():
            for synth in synths:
                traits = kn.traits_of(synth)
                assert synth in kn.SYNTH_PALETTE[role] and synth in kn.LANE_DB_AT_UNIT[role], (role, synth)
                assert traits is None or traits.tail != "held", synth
                assert role != "pad" or traits is None or traits.tail == "short", synth


@pytest.mark.parametrize("seed", SEEDS)
def test_kick_is_the_genre_sample_from_the_table(seed):
    """Бочка — ``KICK_SOUNDS[GENRE_KICK['club']]`` (замер: низ ≥ 0.9 записи), а не ``X`` без ``sample``."""
    kick = kn.KICK_SOUNDS[kn.GENRE_KICK["club"]]
    track = _track(seed)
    assert track.parts["kick"].sample == kick.sample and kick.low >= 0.9
    assert {e.sample for e in _events(track)["kick"]} == {f"{kick.symbol}{kick.sample}"}
    assert {e.sample for e in _events(track)["hats"]} == {"-0"}, "остальные ударные — без sample="


def test_validator_guards_the_sidechain():
    track = _track(1)
    with pytest.raises(TrackError, match="mix.duck_roles"):
        validate(replace(track, mix=replace(track.mix, duck_roles=frozenset({"kick"}))))
    with pytest.raises(TrackError, match="mix.duck_trigger"):
        validate(replace(track, mix=replace(track.mix, duck_trigger=(4, 0))))
    with pytest.raises(TrackError, match="mix.duck_trigger"):
        validate(replace(track, mix=replace(track.mix, duck_trigger=())))
    with pytest.raises(TrackError, match="parts.kick.sample"):
        validate(replace(track, parts={**track.parts, "kick": replace(track.parts["kick"], sample=-1)}))
