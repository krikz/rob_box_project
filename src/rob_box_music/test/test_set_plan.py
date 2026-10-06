"""PR-3b ADR-0149: план сета (темп, дуга энергии, ход тоники) и ритм по плану — на событиях нот, без снапшотов."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import replace

import pytest

from melodies import MELODIES, profile
from rob_box_music import knowledge as kn
from rob_box_music.arrange import mix, rhythm
from rob_box_music.arrange.compose import compose
from rob_box_music.arrange.mix import level_amp
from rob_box_music.model import BEATS_PER_BAR
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import FIFTH, root_shift, seeded_plan, track_energy
from rob_box_music.theme import seeded_profile

THEMES = ("космос", "киберпанк", "детский праздник", "славянская вечеринка", "новый год", "что-то своё", "")
SEEDS = range(30)
STEP = BEATS_PER_BAR / 16


def _by_role(track, deck="A"):
    program = render(track, deck)
    _parsed, events = program_events(program.code, program.form_beats)
    slot_role = {slot: role for role, slot in program.slots.items()}
    out = defaultdict(list)
    for ev in events:
        out[slot_role[ev.slot]].append(ev)
    return program, out


def _section_spans(track):
    start = 0
    for sec in track.form.sections:
        yield sec, start, start + sec.bars * BEATS_PER_BAR
        start += sec.bars * BEATS_PER_BAR


@pytest.mark.parametrize("theme", THEMES)
def test_one_tempo_for_the_whole_set_in_the_club_window(theme):
    """Один темп на сет (В6: club 128–138) для всех сидов; у каждого трека и программы — темп плана."""
    lo, hi = kn.STYLES["club"].bpm
    for seed in SEEDS:
        plan = seeded_plan(seeded_profile(theme), seed, genre="club")
        assert lo <= plan.bpm <= hi
        for no in (1, 2, 5, 11):
            track = compose(plan, no)
            assert track.bpm == plan.bpm and render(track, "A").bpm == plan.bpm


def test_tempo_outside_the_window_is_seeded_inside_it_not_kept():
    for bpm in (120, 150):
        bpms = {seeded_plan(profile(bpm=bpm), s, genre="club").bpm for s in range(20)}
        assert bpms <= set(range(128, 139)) and len(bpms) > 1
    assert seeded_plan(profile(bpm=131), 0, genre="club").bpm == 131


def test_energy_wave_rises_to_the_peak_then_falls():
    """ADR-0147 §3.4: трек 1 — интро (2); период «разгон → пик 5 → спад», повторяется в открытом сете."""
    energies = [track_energy(no) for no in range(1, 21)]
    assert energies[:10] == [2, 3, 4, 5, 4, 2, 3, 4, 5, 4]
    period = len(kn.ENERGY_WAVE)
    for start in range(0, 20, period):
        wave = energies[start:start + period]
        peak = wave.index(max(wave))
        assert max(wave) == 5 and all(a < b for a, b in zip(wave[:peak], wave[1:peak + 1])), wave
        assert all(a > b for a, b in zip(wave[peak:], wave[peak + 1:])), wave
    plan = seeded_plan(profile(), 3, n_tracks=7)
    assert [t.energy for t in plan.tracks] == energies[:7] and plan.track(42).energy == track_energy(42)


@pytest.mark.parametrize("seed", range(10))
def test_section_energy_follows_the_track_energy(seed):
    """Энергия секций растёт с энергией трека; форма внутри трека — intro < build < drop, break < drop2."""
    plan = seeded_plan(profile(root=seed % 12), seed)
    plan = replace(plan, tracks=tuple(replace(t, template="club48") for t in plan.tracks))  # одна форма (PR-7)
    tracks = {plan.track(no).energy: compose(plan, no) for no in range(1, 6)}
    for low, high in zip(sorted(tracks), sorted(tracks)[1:]):
        high_energy = {s.name: s.energy for s in tracks[high].form.sections}  # порядок секций трека 1 другой (#3427)
        for a in tracks[low].form.sections:
            assert a.energy <= high_energy[a.name], (a.name, low, high)
    for track in tracks.values():
        e = {s.name: s.energy for s in track.form.sections}
        assert e["intro"] < e["build"] < e["drop"] and e["break"] < e["drop2"], e


@pytest.mark.parametrize("seed", range(10))
def test_low_energy_thins_roles_not_levels(seed):
    """ADR-0149 §4.6 б: энергия 1–2 — без клэпа (состав ролей), уровни ролей те же."""
    plan = seeded_plan(profile(root=seed % 12), seed)
    low, peak = compose(plan, 1), compose(plan, 4)  # энергия 2 и 5
    _p, low_events = _by_role(low)
    _p, peak_events = _by_role(peak)
    assert "clap" not in low.parts and not low_events["clap"]
    assert peak_events["clap"] and all("clap" in sec.roles for sec in peak.form.sections if sec.name == "drop2")
    for track in (low, peak):  # уровень — цель роли (или потолок синта), энергия его не трогает (PR-3c: тембры разные)
        for role, part in track.parts.items():
            capped = level_amp(role, part) == pytest.approx(kn.MAX_LAYER_AMP, rel=1e-3)
            # цель роли (пэд — + прибавка рисунка) и поправка A9-модели трека (ADR-0152 PR-5)
            # поправка A9 отсчитывается от уровня после потолка синта (``mix._level``)
            target = mix._level(kn.STYLES["club"], role, part, track.history_key.pad_figure)
            target += track.mix.a9_trim.get(role, 0)
            assert part.level_db == pytest.approx(target, abs=0.011) or (part.level_db < target and capped)


@pytest.mark.parametrize("seed", SEEDS)
def test_fills_sit_on_phrase_boundaries_before_drops(seed):
    """Fill перед дропом — два последних такта 8-тактовой фразы: ролл клэпа восьмыми, затем 16-ми; бочка молчит
    на последней доле; вне роллов клэп — только бэкбит дропов (2 и 4)."""
    plan = seeded_plan(profile(root=seed % 12), seed, genre="club")  # fill с прямой бочкой: ломаная — отдельно
    track = compose(plan, 4, melodies=MELODIES if seed % 2 else None)  # энергия 5: клэп есть
    _program, events = _by_role(track)
    claps = sorted(e.beat for e in events["clap"])
    kicks = {e.beat for e in events["kick"]}
    roll = {i * STEP for i in rhythm.ROLL_STEPS}
    fill_bars = 0
    spans = list(_section_spans(track))
    for i, (sec, start, end) in enumerate(spans):
        before_drop = i + 1 < len(spans) and spans[i + 1][0].name.startswith("drop")
        last_bar = end - BEATS_PER_BAR
        roll_from = end - rhythm.ROLL_BARS * BEATS_PER_BAR if before_drop else last_bar
        in_bar = {round(b - last_bar, 6) for b in claps if last_bar <= b < end}
        if sec.fill_last_bar and "clap" in sec.roles:
            fill_bars += 1
            assert (end // BEATS_PER_BAR) % 8 == 0, "fill на границе 8-тактовой фразы"
            assert roll <= in_bar, (sec.name, in_bar)
            eighths = {round(b - roll_from, 6) for b in claps if roll_from <= b < last_bar}
            assert eighths == {i * 0.5 for i in range(8)}, "предпоследний такт — ролл восьмыми"
        if sec.fill_last_bar and "kick" in sec.roles:
            assert end - 1 not in kicks and end - 2 in kicks, sec.name
        for b in claps:
            if start <= b < end and not (sec.fill_last_bar and b >= roll_from):
                assert sec.name.startswith("drop") and b % BEATS_PER_BAR in (1.0, 3.0), (sec.name, b)
    assert fill_bars == 2, "fill перед drop и перед drop2"


def test_fill_roll_accent_builds_up():
    plan = seeded_plan(profile(), 0)
    track = compose(plan, 4)
    _program, events = _by_role(track)
    build_end = next(end for sec, _s, end in _section_spans(track) if sec.name == "build")
    roll = sorted((e.beat, e.amp) for e in events["clap"] if build_end - 2 <= e.beat < build_end)
    amps = [a for _b, a in roll]
    assert len(roll) == 8 and amps == sorted(amps) and amps[-1] > amps[0]


@pytest.mark.parametrize("seed", range(10))
def test_swing_moves_only_the_off_beat_sixteenths(seed):
    """Свинг плана сдвигает нечётные 16-е хэтов на ``swing`` восьмой; доли, оффбит-хэты и бочка на месте."""
    plan = seeded_plan(profile(root=seed % 12), seed)
    assert kn.STYLES["club"].swing[0] <= plan.swing <= kn.STYLES["club"].swing[1]
    straight = compose(replace(plan, swing=0.0), 3)
    swung = compose(plan, 3)
    _p, ev0 = _by_role(straight)
    _p, ev1 = _by_role(swung)
    shift = rhythm.swing_offset_ms(plan.swing, plan.bpm) * plan.bpm / 60000.0
    assert shift > 0
    hats0, hats1 = sorted(e.beat for e in ev0["hats"]), sorted(e.beat for e in ev1["hats"])
    assert len(hats0) == len(hats1)
    moved = 0
    for a, b in zip(hats0, hats1):
        odd = round(a / STEP) % 2 == 1
        if odd:
            moved += 1
            assert b - a == pytest.approx(shift, abs=1e-3), a
        else:
            assert b == a
    assert moved > 0
    assert sorted(e.beat for e in ev0["kick"]) == sorted(e.beat for e in ev1["kick"])
    for role in kn.TONAL_ROLES:
        assert sorted((e.beat, e.midi) for e in ev0[role]) == sorted((e.beat, e.midi) for e in ev1[role])


def test_tonic_walks_by_fifths_between_tracks():
    plan = seeded_plan(profile(root=9), 0)
    roots = [plan.root(no) for no in range(1, 13)]
    assert roots[0] == 9 and len(set(roots)) == 12
    assert all((b - a) % 12 == FIFTH for a, b in zip(roots, roots[1:]))
    assert [compose(plan, no).key.root for no in range(1, 6)] == roots[:5]  # без хука тоника трека = плана
    assert root_shift(1) == 0 and root_shift(13) == 0


@pytest.mark.parametrize("seed", range(10))
def test_plan_and_tracks_are_deterministic_by_seed(seed):
    prof = seeded_profile("космос")
    plan = seeded_plan(prof, seed)
    assert plan == seeded_plan(prof, seed)
    for no in (1, 4):
        a, b = compose(plan, no, melodies=MELODIES), compose(seeded_plan(prof, seed), no, melodies=MELODIES)
        assert a == b and render(a, "A") == render(b, "A")


def test_seed_changes_the_groove_but_not_the_tempo():
    prof = seeded_profile("киберпанк")
    plans = [seeded_plan(prof, s, genre="club") for s in range(12)]
    assert len({p.bpm for p in plans}) == 1
    assert len({p.swing for p in plans}) >= 6


def test_the_line_up_grows_from_build_to_the_second_drop():
    """Развитие формы (слух Шифу и отзыв эксперта 05.10): build и оба дропа играли одним составом. Теперь состав
    растёт: build без баса и лупа, первый дроп возвращает бас, второй добавляет брейк-луп."""
    track = compose(seeded_plan(profile(), 3), 4)
    line_up = {sec.name: set(sec.roles) for sec in track.form.sections}
    assert "bass" not in line_up["build"] and "loop" not in line_up["build"]
    assert {"kick", "pad", "lead", "clap"} <= line_up["build"]
    assert "bass" in line_up["drop"] and "loop" not in line_up["drop"]
    assert line_up["drop2"] == line_up["drop"] | {"loop"}
    assert line_up["build"] < line_up["drop"] < line_up["drop2"], "каждая ступень добавляет слой"
    _program, events = _by_role(track)
    spans = {sec.name: (start, end) for sec, start, end in _section_spans(track)}
    bass_in = {name: sum(1 for e in events["bass"] if lo <= e.beat < hi) for name, (lo, hi) in spans.items()}
    assert bass_in["build"] == 0 and bass_in["drop"] > 0 and bass_in["drop2"] > 0


# --- #3460 (A16b): семья тембров темы вне таблицы -----------------------------------------------------------------
OUT_OF_TABLE = ("Mozart 40", "House Some More", "Baby One More Time", "In Your Eyes", "Mambo Nr 5")


def _timbres_of_series(themes, base_seed=20261010):
    """Семьи тембров серии сетов подряд: история — строки прошлых треков (свежие первыми), как у плеера."""
    history, out = [], []
    for n, theme in enumerate(themes):
        plan = seeded_plan(seeded_profile(theme), base_seed + n, set_id=f"s{n}", history=history)
        out.append(plan.family)
        history.insert(0, {"set_id": f"s{n}", "timbre": plan.family})
    return out


def test_theme_outside_the_table_is_not_always_the_default_family():
    """Приёмка 06.10: все пять тем случайной серии (row=None) звучали ``warm`` — A16b требует ≥ 3 семей."""
    assert all(seeded_profile(t).row is None for t in OUT_OF_TABLE)
    families = _timbres_of_series(OUT_OF_TABLE)
    assert len(set(families)) >= 3, families
    assert all(a != b for a, b in zip(families, families[1:])), families  # с прошлым сетом подряд семья не та же


@pytest.mark.parametrize("seed", range(10))
def test_series_of_unknown_themes_covers_three_families(seed):
    assert len(set(_timbres_of_series(OUT_OF_TABLE, base_seed=seed * 1000 + 7))) >= 3


def test_table_theme_keeps_its_family_and_plan_is_deterministic():
    for theme, row in (("космос", "space"), ("киберпанк", "cyber"), ("детский праздник", "kids")):
        plan = seeded_plan(seeded_profile(theme), 5)
        assert plan.family == kn.THEME_TIMBRE[row]
    again = _timbres_of_series(OUT_OF_TABLE)
    assert again == _timbres_of_series(OUT_OF_TABLE)


def test_family_reaches_the_track_synths_and_history_axis():
    """Семья плана — не только метка: синты лида/баса/пэда трека из неё, ось ``timbre`` пишется в историю."""
    from rob_box_music.diversity import track_history

    for seed in range(12):
        plan = seeded_plan(seeded_profile("Mambo Nr 5"), seed)
        track = compose(plan, 1)
        family = plan.table.timbres[plan.family]
        for role in kn.TONAL_ROLES:
            assert track.parts[role].synth_or_sample in family[role], (seed, role)
        assert track.history_key.timbre == plan.family and track_history(track, plan.set_id)["timbre"] == plan.family


def test_reasoner_row_sets_its_family_and_none_keeps_the_sets_own():
    from rob_box_music import reasoner as rz

    plan = seeded_plan(seeded_profile("Mambo Nr 5"), 3)
    ref = rz.Refinement("cyber", plan.profile.mode, plan.profile.hook_ids, (2, 3, 4, 5))
    assert rz.apply(plan, ref).family == kn.THEME_TIMBRE["cyber"]
    assert rz.apply(plan, replace(ref, row=None)).family == plan.family
