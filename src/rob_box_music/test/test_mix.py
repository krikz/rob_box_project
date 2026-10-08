"""PR-3c ADR-0149: микс — сайдчейн-огибающая, уровни ролей из одной таблицы, тембр темы, бочка жанра.

Свойства проверяются на событиях ``render.events.program_events`` (``amp`` события = ``amp × amplify``), без снапшотов.
"""

from __future__ import annotations

import math
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
    return compose_p(profile(root=seed % 12), track_no, set_seed=seed, melodies=hooks,
                     template="" if track_no == 1 else "club48")  # структура club48 (PR-7: формы — test_forms)


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


def _section(track, beat):
    """Номер секции формы, где звучит доля ``beat``."""
    start = 0.0
    for i, sec in enumerate(track.form.sections):
        start += sec.bars * 4
        if beat < start - 1e-9:
            return i
    raise AssertionError(beat)


def _env(track, beat):
    duck = track.mix.duck[_section(track, beat)]
    return mix.duck_envelope(duck.trigger, duck.depth)


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
    """``amp`` события ÷ гейт = акцент × огибающая по шагу такта для баса/пэда; бочка/хэты/лид без сайдчейна.
    psr-слой (PR-3d) и луп нарезкой (#3432) — тоже под огибающей (``test_diversity``, ``test_sample_render``)."""
    track = _track(seed)
    by_role = _events(track)
    figure = kn.PAD_FIGURES[track.history_key.pad_figure]  # held — без насоса (ADR-0152 §3.2)
    ducked = {"bass", "pad", "sample", "loop"} - (set() if figure.ducked else {"pad"})
    assert track.mix.duck_roles == frozenset(ducked) & set(track.parts)
    assert len(track.mix.duck) == len(track.form.sections)
    for ev in by_role["pad"]:  # пэд под огибающей вида своей секции (или ровно 1.0 у held)
        want = _env(track, ev.beat)[_step(ev.beat)] if figure.ducked else 1.0
        assert ev.amp / ev.gate == pytest.approx(want, abs=1e-3)
        figure_name = track.history_key.pad_figure
        assert (ev.sus_beats % 4 == 0 if figure_name == "held"  # аккорд держится до смены — целые такты (#3529)
                else ev.sus_beats == {"pumped16": 0.25, "stabs": 1.0}[figure_name] or (
                    figure_name == "stabs" and ev.sus_beats == 0.5)), ev
    accents = {(p.beat, p.midi): p.accent for p in track.parts["bass"].pitches}
    for ev in by_role["bass"]:
        want = kn.ACCENT_AMPLIFY[accents[(ev.beat, ev.midi)]] * _env(track, ev.beat)[_step(ev.beat)]
        assert ev.amp / ev.gate == pytest.approx(want, abs=1e-3)
    for role in ("kick", "lead"):
        assert {round(e.amp / e.gate, 3) for e in by_role[role]} <= set(kn.ACCENT_AMPLIFY) | {1.0}, role


@pytest.mark.parametrize("seed", SEEDS)
def test_pad_dips_only_on_the_trigger(seed):
    """Пэд проваливается только на шаге удара триггера (рисунок бочки), между ударами только растёт; там, где
    бочка звучит, провал пэда совпадает с ударом бочки."""
    # провал на 16-х — у pumped16 (stabs и held — ``test_pad_figures``): первый такой трек сета сида
    track = next(t for t in map(lambda n: _track(seed, n), range(1, 11)) if t.history_key.pad_figure == "pumped16")
    by_role = _events(track)
    kicks = {round(e.beat, 6) for e in by_role["kick"]}
    pad = sorted({(round(e.beat, 6), round(e.amp / e.gate, 3)) for e in by_role["pad"]})
    dips = [b1 for (_b0, r0), (b1, r1) in zip(pad, pad[1:]) if r1 < r0]
    trigger = {b: track.mix.duck[_section(track, b)].trigger for b, _r in pad}
    assert dips and all(_step(b) in trigger[b] for b in dips)
    assert any(b in kicks for b in dips)
    for (b0, r0), (b1, r1) in zip(pad, pad[1:]):
        same_look = track.mix.duck[_section(track, b0)] == track.mix.duck[_section(track, b1)]
        if _step(b1) not in trigger[b1] and b1 - b0 <= 0.25 + 1e-9 and same_look:
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
    """Уровень роли — ``Style.role_level_db`` (или честный потолок синта); гейт рендера даёт этот уровень по модели."""
    track = _track(seed, track_no=1 + seed % 5)
    program = render(track, "A")
    for role, part in track.parts.items():
        unit, exponent = mix._unit(role, part)
        amp = mix.level_amp(role, part)
        # поправка A9 отсчитывается от уровня после потолка синта (``mix._level``): ``min(цель, потолок) + поправка``
        target = mix._level(kn.STYLES["club"], role, part, track.history_key.pad_figure)
        target += track.mix.a9_trim.get(role, 0)
        assert part.level_db <= target + 1e-9
        if part.level_db < target - 1e-9:
            assert amp == pytest.approx(kn.MAX_LAYER_AMP)
        assert mix.layer_db(unit, exponent, amp) == pytest.approx(part.level_db, abs=0.01)
        assert track.mix.level_db[role] == part.level_db
        st = track.mix.stereo.get(role)
        voices = st.voices if st else 1
        player_amp = mix.voice_amp(role, part, voices)
        assert mix.layer_db(unit, exponent, player_amp) + 10 * math.log10(voices) == pytest.approx(
            min(part.level_db, mix.layer_db(unit, exponent, kn.MAX_LAYER_AMP) + 10 * math.log10(voices)), abs=0.01)
        line = next(ln for ln in program.code.splitlines() if ln.startswith(program.slots[role] + " "))
        assert f"{round(player_amp, 3):g}" in line.split("amp=var(")[1].split(")")[0], role


def test_same_level_whatever_the_timbre():
    """Смена синта роли не меняет её уровень в модели — меняется ``amp`` (вся разница громкости синтов — в таблице)."""
    track = _track(3)
    for role, synths in (("bass", ("bass", "dub")), ("pad", ("sinepad", "space"))):
        parts = [replace(track.parts[role], synth_or_sample=s) for s in synths]
        leveled = [mix.mix_parts(kn.STYLES["club"], {role: p}, track.form)[0][role] for p in parts]
        assert leveled[0].level_db == leveled[1].level_db == kn.STYLES["club"].role_level_db[role]
        assert mix.level_amp(role, leveled[0]) != mix.level_amp(role, leveled[1])


@pytest.mark.parametrize("theme", ["космос", "киберпанк", "детский праздник", "славянская вечеринка", "новый год",
                                   "просто вечеринка"])
def test_timbre_follows_the_theme_and_is_deterministic(theme):
    prof = seeded_profile(theme)
    seen, families = set(), set()
    for seed in range(8):
        plan = seeded_plan(prof, seed)
        families.add(plan.family)
        family = kn.STYLES["club"].timbres[plan.family]  # тема вне таблицы — семья по сиду (#3460)
        track = compose(plan, 1 + seed % 3)
        synths = {r: track.parts[r].synth_or_sample for r in kn.TONAL_ROLES}
        assert all(synths[r] in family[r] for r in synths), (theme, synths)
        assert synths == {r: compose(plan, 1 + seed % 3).parts[r].synth_or_sample for r in kn.TONAL_ROLES}
        seen.add(tuple(sorted(synths.items())))
    if any(len(v) > 1 for f in families for v in kn.STYLES["club"].timbres[f].values()):
        assert len(seen) > 1, "сид меняет тембр внутри семьи темы"
    club = kn.STYLES["club"]
    for role in ("bass", "lead"):
        assert (mix.role_timbre(club, kn.family_of(club, prof.row), role, (), random.Random(5))
                == mix.role_timbre(club, kn.family_of(club, prof.row), role, (), random.Random(5)))


def test_timbre_table_is_playable():
    """Каждый синт семьи — из палитры роли, с замером громкости и не ``held``; пэд с фиксированным хвостом
    (``warmpad`` 1.2 с размазал бы сайдчейн 16-х) — только в рисунке ``held`` (``mix.pad_synths``, ADR-0152 §3.2)."""
    club = kn.STYLES["club"]
    assert set(kn.THEME_TIMBRE) == set(kn.THEMES)
    assert set(kn.THEME_TIMBRE.values()) | {club.default_timbre} <= set(club.timbres)
    for family in kn.STYLES["club"].timbres.values():
        assert set(family) == set(kn.TONAL_ROLES)
        for role, synths in family.items():
            for synth in synths:
                traits = kn.traits_of(synth)
                assert synth in kn.SYNTH_PALETTE[role] and synth in kn.LANE_DB_AT_UNIT[role], (role, synth)
                assert traits is None or traits.tail != "held", synth
                assert role != "pad" or traits is None or traits.tail == "short" or all(
                    synth not in mix.pad_synths(club, kn.family_of(club, row), figure) for row in kn.THEME_TIMBRE
                    for figure in club.pad_figures if not kn.PAD_FIGURES[figure].long_tails), synth


@pytest.mark.parametrize("seed", SEEDS)
def test_kick_is_a_sample_from_the_style_pool(seed):
    """Бочка — запись пула стиля ``KICK_SOUNDS`` (замер: низ ≥ 0.9 записи), а не ``X`` без ``sample``."""
    track = _track(seed)
    pool = (kn.KICK_SOUNDS[n] for n in kn.STYLES["club"].kick_pool)
    kick = next(k for k in pool if k.sample == track.parts["kick"].sample)
    assert kick.low >= 0.9
    assert {e.sample for e in _events(track)["kick"]} == {f"{kick.symbol}{kick.sample}"}
    assert {e.sample for e in _events(track)["hats"]} == {"-0"}, "остальные ударные — без sample="


def test_validator_guards_the_sidechain():
    track = _track(1)
    with pytest.raises(TrackError, match="mix.duck_roles"):
        validate(replace(track, mix=replace(track.mix, duck_roles=frozenset({"kick"}))))
    first = track.mix.duck[0]
    with pytest.raises(TrackError, match=r"mix.duck\[0\].trigger"):
        validate(replace(track, mix=replace(track.mix, duck=(replace(first, trigger=(4, 0)),) + track.mix.duck[1:])))
    with pytest.raises(TrackError, match=r"mix.duck\[0\].trigger"):
        validate(replace(track, mix=replace(track.mix, duck=(replace(first, trigger=()),) + track.mix.duck[1:])))
    with pytest.raises(TrackError, match="mix.duck"):
        validate(replace(track, mix=replace(track.mix, duck=track.mix.duck[1:])))
    with pytest.raises(TrackError, match="parts.kick.sample"):
        validate(replace(track, parts={**track.parts, "kick": replace(track.parts["kick"], sample=-1)}))
