"""Разнообразие v2 (ADR-0149 §3.11, I17, A12, A13; PR-3d): история ``music_history`` по всем осям, сэмплы DJ_Dave.

5 сетов × 10 треков подряд с одной персистентной историей (``MusicHistory(":memory:")``), как у владельца плеера:
каркас ударных не повторяется подряд, прогрессия ≤ 3 из любых 10 треков подряд, хук-фрагмент не повторяется подряд,
охват — ≥ 30 разных сэмплов Дэйва; лупы и FX звучат в своих секциях (события программы). Всё детерминировано по сиду.
"""

from __future__ import annotations

import itertools
import math
from collections import Counter, defaultdict

import pytest

from melodies import MELODIES, profile
from rob_box_music import knowledge as kn
from rob_box_music.arrange import samples
from rob_box_music.arrange.compose import compose
from rob_box_music.diversity import MusicHistory, track_history, weighted_pick
from rob_box_music.model import BEATS_PER_BAR
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import TONIC_MEMORY, seeded_plan

SETS, TRACKS = 5, 10
#: (тоника профиля, лад, хуки) пяти сетов: с мелодиями темы и без них (мотив лида).
SET_PROFILES = [(9, "minor", True), (2, "dorian", False), (9, "minor", True), (5, "phrygian", False),
                (0, "major", True)]


def run_sets(base_seed: int = 0):
    """5 сетов подряд с общей историей; трек → (трек, строка истории до него)."""
    history = MusicHistory(":memory:")
    tracks = []
    for n, (root, mode, hooked) in enumerate(SET_PROFILES):
        plan = seeded_plan(profile(root=root, mode=mode), base_seed + n, set_id=f"s{n}", history=history.recent(50))
        for no in range(1, TRACKS + 1):
            track = compose(plan, no, melodies=MELODIES if hooked else None, history=history.recent(50))
            assert history.record(**track_history(track, plan.set_id))
            tracks.append(track)
    return tracks, history


@pytest.fixture(scope="module")
def fifty():
    return run_sets()


def test_kit_never_repeats_back_to_back(fifty):
    kits = [t.history_key.kit for t in fifty[0]]
    assert len(kits) == SETS * TRACKS
    assert all(a != b for a, b in zip(kits, kits[1:])), kits
    assert set(kits) == set(kn.STYLES["club"].kits)


def test_progression_at_most_three_in_any_ten(fifty):
    progs = [t.history_key.progression for t in fifty[0]]
    for i in range(len(progs) - 9):
        top = Counter(progs[i:i + 10]).most_common(1)[0]
        assert top[1] <= 3, (i, progs[i:i + 10])


def test_hook_fragment_never_repeats_back_to_back(fifty):
    keys = [t.history_key for t in fifty[0]]
    assert all(k.hook_fingerprint for k in keys)
    assert all(a.hook_fingerprint != b.hook_fingerprint for a, b in zip(keys, keys[1:]))
    melodies = [k.hook for k in keys]
    assert all(a is None or a != b for a, b in zip(melodies, melodies[1:]))
    assert sum(m is not None for m in melodies) >= 20, "сеты с мелодиями берут хук темы"


def _used(track):
    key = track.history_key
    return {key.sample, key.fx, *key.perc.split(",")}


def test_dave_coverage_at_least_thirty_in_five_sets(fifty):
    """A12: ≥ 30 разных сэмплов Дэйва за 5 сетов (было 7/123): psr-пулы, лупы, FX."""
    used = set().union(*(_used(t) for t in fifty[0]))
    assert len(used) >= 30, sorted(used)
    roles = Counter(kn.SAMPLE_CATALOG[name].role for name in used)
    assert roles["loop"] >= 3 and roles["fx"] >= 5 and roles["perc"] >= 15, roles
    assert len(used & set(kn.DAVE_PSR)) >= 8, "psr из её списка"


def _events_by_slot(track, deck="A"):
    program = render(track, deck)
    _p, events = program_events(program.code, program.form_beats, loops=True)
    by_slot = defaultdict(list)
    for ev in events:
        by_slot[ev.slot].append(ev)
    return program, by_slot


def _section_spans(track):
    spans, beat = {}, 0
    for sec in track.form.sections:
        spans[sec.name] = (beat, beat + sec.bars * BEATS_PER_BAR)
        beat += sec.bars * BEATS_PER_BAR
    return spans


def test_layers_sound_only_in_their_sections(fifty):
    """psr, луп, FX — только в своих секциях; FX — ровно один удар на первой доле каждой своей секции."""
    for track in fifty[0][::7]:
        program, by_slot = _events_by_slot(track)
        spans = _section_spans(track)
        for role in ("sample", "loop", "fx"):
            events = by_slot[program.slots[role]]
            if not any(n in spans for n in kn.STYLES["club"].layer_sections[role]):
                assert not events, role  # форма без секций слоя (``short32``: нет drop2 под луп)
                continue
            assert events, role
            for ev in events:
                assert any(lo <= ev.beat < hi for name, (lo, hi) in spans.items()
                           if name in kn.STYLES["club"].layer_sections[role])
        fx_beats = sorted({ev.beat for ev in by_slot[program.slots["fx"]]})
        assert fx_beats == sorted(spans[n][0] for n in kn.STYLES["club"].layer_sections["fx"] if n in spans)
        assert program.sample_files == {kn.SAMPLE_CATALOG[n].path for n in _used(track)}


def test_loop_is_chopped_into_eighths_each_restarted_on_its_beat(fifty):
    """DJ_Dave ``loopAt(l).chop(l*8).legato(1)``: событие на каждой восьмой, кусок ``pos`` = доля от начала лупа
    по модулю его длины, ``sus`` = восьмая, ``tempo`` = темп оригинала; без ``beat_stretch``."""
    for track in (t for t in fifty[0][::5] if any(s.name == "drop2" for s in t.form.sections)):  # луп — в drop2
        program, by_slot = _events_by_slot(track)
        info = kn.SAMPLE_CATALOG[track.parts["loop"].synth_or_sample]
        events = by_slot[program.slots["loop"]]
        assert events and all(ev.sample == info.loop_arg for ev in events)
        for ev in events:
            assert ev.beat * 2 == int(ev.beat * 2), "кусок на сетке восьмых"
            assert ev.pos == pytest.approx(ev.beat % info.beats), (ev.beat, ev.pos)
            assert ev.sus_beats == 0.5
        line = next(ln for ln in program.code.splitlines() if ln.startswith(program.slots["loop"] + " >>"))
        assert f"tempo={info.bpm}" in line and "beat_stretch" not in line


def test_psr_takes_a_random_pool_file_on_every_sixteenth_under_the_sidechain(fifty):
    """psr-слой: удар на каждой 16-й, файл — из пула трека и меняется от удара к удару; ``sus`` ≤ длины файла
    (звучит один раз); громкость качает та же огибающая, что пэд и бас (тише на ударе бочки)."""
    from rob_box_music.arrange.mix import duck_envelope, file_gain
    for track in fifty[0][::5]:
        program, by_slot = _events_by_slot(track)
        pool = set(track.history_key.perc.split(","))
        events = [ev for ev in by_slot[program.slots["sample"]] if ev.pan <= 0]  # один голос из пары L/R
        steps = sorted({round(ev.beat * 4) for ev in events})
        assert steps == list(range(steps[0], steps[-1] + 1)) or len(steps) > 100, "каждая 16-я"
        files = [ev.sample for ev in sorted(events, key=lambda e: e.beat)]
        assert {f for f in files} <= {kn.SAMPLE_CATALOG[n].loop_arg for n in pool}
        assert len(set(files)) >= 3 and sum(a != b for a, b in zip(files, files[1:])) >= len(files) // 3
        by_path = {kn.SAMPLE_CATALOG[n].loop_arg: kn.SAMPLE_CATALOG[n] for n in pool}
        assert all(ev.sus_beats <= by_path[ev.sample].seconds * track.bpm / 60 + 1e-3 for ev in events)
        assert "sample" in track.mix.duck_roles
        starts = list(itertools.accumulate(sec.bars * 4 for sec in track.form.sections))

        def look(beat):  # сайдчейн вида секции (PR-7): триггер и глубина — свои у каждой секции
            return track.mix.duck[next(i for i, end in enumerate(starts) if beat < end)]

        for duck in set(track.mix.duck):
            env = duck_envelope(duck.trigger, duck.depth)
            mine = [ev for ev in events if look(ev.beat) == duck]
            # уровень файла пула (#3432: громче эталона — тише) — не сайдчейн: сравнение без него
            level = {kn.SAMPLE_CATALOG[n].loop_arg: file_gain(n, track.parts["sample"].synth_or_sample) for n in pool}
            on_kick = [ev.amp / level[ev.sample] for ev in mine if round(ev.beat * 4) % 16 in duck.trigger]
            off_kick = [ev.amp / level[ev.sample] for ev in mine if env[round(ev.beat * 4) % 16] == 1.0]
            assert not mine or (on_kick and off_kick and max(on_kick) < min(off_kick)), duck


def test_psr_is_two_voices_left_and_right_from_role_stereo(fifty):
    """Ширина psr — ``Style.stereo`` (два голоса с Хаасом, аналог ``jux``): баланс L/R по энергии 0."""
    for track in fifty[0][:6]:
        program, by_slot = _events_by_slot(track)
        events = by_slot[program.slots["sample"]]
        pans = Counter(ev.pan for ev in events)
        assert set(pans) == {-kn.PAN_PSR, kn.PAN_PSR} and pans[-kn.PAN_PSR] == pans[kn.PAN_PSR]
        assert abs(sum(ev.amp ** 2 * ev.pan for ev in events)) < 1e-9


def test_kits_hats_are_balanced_left_right(fifty):
    """PR-9 меняет сторону хэта на каждом ударе: акценты каркаса делятся между сторонами поровну."""
    from rob_box_music.arrange import rhythm
    from rob_box_music.arrange.mix import alternate_pan
    for name, kit in kn.STYLES["club"].kits.items():
        grid = rhythm.pattern_grid(kit["hats"])
        hits = [st.on for st in grid.steps]
        pans = alternate_pan(hits, 1.0)
        weights = [kn.ACCENT_AMPLIFY[st.accent] ** 2 if st.on else 0 for st in grid.steps] * 2
        assert abs(sum(w * p for w, p in zip(weights, pans))) < 1e-9, name


def test_blend_has_no_sample_layer_on_both_decks(fifty):
    """PR-8: в блэнде intro входящего (дека B) и outro уходящего (дека A) звучат вместе. Ни psr, ни лупа, ни FX
    в блэнде нет ни у одной деки (удар не удваивается на стыке); слоты сэмплов дек не пересекаются."""
    from rob_box_music.model import blend_bars
    assert not set(kn.DECK_SLOTS["A"]) & set(kn.DECK_SLOTS["B"])
    tracks = fifty[0]
    for leaving, incoming in zip(tracks[:9], tracks[1:10]):
        bars = blend_bars(leaving, incoming)
        assert bars == 8
        form = leaving.form.bars_total * BEATS_PER_BAR
        at = form - bars * BEATS_PER_BAR
        hits = []
        for track, deck, shift in ((leaving, "A", 0.0), (incoming, "B", at)):
            program, by_slot = _events_by_slot(track, deck)
            for role in ("sample", "loop", "fx"):
                hits += [(deck, role, ev.beat + shift) for ev in by_slot[program.slots[role]]
                         if at <= ev.beat + shift < form]
        assert hits == [], hits[:5]


def test_history_is_written_on_every_axis(fifty):
    rows = fifty[1].recent(SETS * TRACKS)
    assert len(rows) == SETS * TRACKS
    for row in rows:
        assert row["style"] == "club_v2"
        for axis in ("kit", "progression", "hook_fingerprint", "sample", "fx", "perc", "root", "scale", "bpm",
                     "lead", "bass", "pad", "set_id", "kick"):
            assert row[axis] not in (None, ""), (axis, row)
        assert row["kick"] in kn.KICK_SOUNDS, row
    assert {r["set_id"] for r in rows} == {f"s{n}" for n in range(SETS)}


def test_deterministic_by_seed_and_history():
    a, _ha = run_sets(3)
    b, _hb = run_sets(3)
    assert a == b
    assert [render(t, "A").code for t in a[:5]] == [render(t, "A").code for t in b[:5]]


def test_seed_changes_material_not_tempo():
    """Два сета на одну тему без истории: темп один и в окне club, материал разный."""
    prof = profile()
    sets = []
    for seed in (1, 2):
        plan = seeded_plan(prof, seed)
        sets.append([compose(plan, no, melodies=MELODIES) for no in range(1, 6)])
    lo, hi = kn.STYLES["club"].bpm
    assert {t.bpm for s in sets for t in s} == {prof.bpm} and lo <= prof.bpm <= hi
    material = [[(t.history_key.kit, t.history_key.sample, t.history_key.fx, t.history_key.perc) for t in s]
                for s in sets]
    assert material[0] != material[1]


def test_set_tonic_moves_away_from_recent_roots():
    prof = profile(root=9)
    assert seeded_plan(prof, 1).profile.root == 9  # без истории — тоника темы
    history = [{"root": kn.ROOTS[r]} for r in (9, 2, 7, 0)][:TONIC_MEMORY]
    roots = {seeded_plan(prof, seed, history=history).profile.root for seed in range(20)}
    assert roots and not roots & {9, 2, 7, 0}
    assert len(roots) >= 3, "сид выбирает тонику, а не одну и ту же"


def test_only_in_key_samples_enter_a_track():
    from rob_box_music.model import Key
    roles = ("perc",) + samples.LOOP_ROLES + samples.FX_ROLES
    for key in (Key(10, "minor"), Key(0, "major")):
        for name in samples.pool(roles, key):
            info = kn.SAMPLE_CATALOG[name]
            assert info.role in ("perc", "loop", "fx")
            assert not info.tonal or info.key == (key.root, key.mode)
    assert len(samples.pool(roles, Key(0, "major"))) >= 30
    assert "array_perc_guit1" not in samples.pool(("perc",), Key(0, "major")), "гитара тональная"
    assert "dirt_psr_11" not in samples.pool(("perc",), Key(0, "major")), "27 мс — короче атаки огибающей"


def _kw(line, key):
    import re
    return re.search(rf"\b{key}=([^,)]+)", line).group(1)


def test_sample_level_comes_from_the_loudness_model_and_the_file_mean(fifty):
    """Одна ось громкости (PR-3c): уровень роли ``Style.role_level_db``, ``amp`` — из среднего уровня файла
    (у psr — по медиане пула), у двух голосов — мощность делится пополам."""
    import re
    for track in fifty[0][:10]:
        code = render(track, "A").code
        names = {s.name for s in track.form.sections}
        for role in ("sample", "loop", "fx"):
            if not names & set(kn.STYLES["club"].layer_sections[role]):
                continue  # форма без секций слоя (``short32``: нет drop2 под луп)
            part = track.parts[role]
            info = kn.SAMPLE_CATALOG[part.synth_or_sample]
            line = next(ln for ln in code.splitlines() if ln.startswith(render(track, "A").slots[role] + " >>"))
            amp = max(float(v) for v in re.search(r"amp=var\(\[([^\]]*)\]", line).group(1).split(", "))
            voices = 2 if role == "sample" else 1
            want = min(kn.MAX_LAYER_AMP, 10 ** ((part.level_db - 10 * math.log10(voices) - info.mean_db) / 20))
            assert amp == pytest.approx(want, rel=1e-2, abs=1e-3)
            assert part.level_db <= kn.STYLES["club"].role_level_db[role]
    assert all(-60 < i.peak_db <= 0 and i.mean_db < i.peak_db for i in kn.SAMPLE_CATALOG.values())


def test_fx_plays_once(fifty):
    """Раунд 2 на слух — «что-то не то»: синт ``loop`` — ``PlayBuf(loop: 1)`` на всю ``sus``; FX ``dur=32`` крутил
    крэш всю секцию. FX: ``sus`` ≤ длины файла; tn1hit2 — райзер, не удар."""
    for track in fifty[0]:
        program = render(track, "A")
        info = kn.SAMPLE_CATALOG[track.parts["fx"].synth_or_sample]
        line = next(ln for ln in program.code.splitlines() if ln.startswith(program.slots["fx"] + " >>"))
        sus, dur = float(_kw(line, "sus")), float(_kw(line, "dur"))
        assert info.role == "fx" and dur == 32 and sus <= info.seconds * track.bpm / 60 + 1e-3
    assert kn.SAMPLE_CATALOG["dirt_tech_tn1hit2"].role == "riser", "пик на 81 % длины — не удар на сильную долю"


def test_weighted_pick_prefers_the_unplayed():
    import random
    rng = random.Random(0)
    picks = Counter(weighted_pick(["a", "b"], ["a"] * 5, rng) for _ in range(200))
    assert picks["b"] > picks["a"] * 5
