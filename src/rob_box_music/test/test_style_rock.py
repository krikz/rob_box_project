"""ADR-0153 S5: стиль ``rock`` — запись ``knowledge.STYLES``: окна-подстили ``classic``/``grunge``, кит Muldjord файлами
паков, бэкбит 2/4, филлы томами на стыках секций, пауэр-аккорды без терции, бас-рифф на тонах аккорда, форма
куплет/припев/бридж. Поведение — на ``SetPlan``/``Track``/событиях программы Renardo (симулятор ``render.events``).
Клуб побайтно тот же — ``test_style_same_tracks``.
"""

from __future__ import annotations

import dataclasses

import pytest

from melodies import MELODIES
from rob_box_music import knowledge as kn
from rob_box_music.arrange import bass, harmony, pad
from rob_box_music.arrange import hook as hooks
from rob_box_music.arrange.compose import compose
from rob_box_music.diversity import track_composition
from rob_box_music.model import BEATS_PER_BAR, STEPS_PER_BAR, Chord, Key, chord_tones, validate
from rob_box_music.render.events import program_events
from rob_box_music.rtttl import fix_octaves, parse_rtttl
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import match_style_text, seeded_profile

ROCK = kn.STYLES["rock"]
BACKBEAT = (4, 12)


def _plan(theme: str, seed: int, genre: str, hooks: bool = False):
    profile = seeded_profile(theme, "rock")
    profile = dataclasses.replace(profile, hook_ids=tuple(MELODIES)) if hooks else profile
    return seeded_plan(profile, seed, set_id=f"rk{seed}", genre=genre)


@pytest.fixture(scope="module")
def tracks():
    """2 окна × 2 сида × 2 трека (сид 0 — хуки RTTTL, сид 1 — мотив лида): (план, трек, программа, события по ролям)."""
    out = []
    for genre in ROCK.genre_windows:
        for seed in range(2):
            plan = _plan("в пещере горного короля", seed, genre, hooks=seed == 0)
            history: list = []
            for no in (1, 2):
                track = compose(plan, no, history=history, melodies=MELODIES if seed == 0 else None)
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


def _bars(track):
    """(первый такт, секция) по форме трека."""
    start, out = 0, []
    for sec in track.form.sections:
        out.append((start, sec))
        start += sec.bars
    return out


def _bar_steps(grid, bar: int):
    return [i for i, st in enumerate(grid.steps[bar * STEPS_PER_BAR:(bar + 1) * STEPS_PER_BAR]) if st.on]


def test_two_windows_with_their_own_tempo_and_low_targets():
    """Окна подстилей: classic 100–125 (эталон ≈ 110), grunge 110–130 (≈ 121); у каждого — свои уровни ролей и порог
    A9-модели (эталон низа classic 0.37, grunge 0.66: у гранжа бочка и бас громче); поля стиля — первое окно."""
    assert {n: w.bpm for n, w in ROCK.genre_windows.items()} == {"classic": (100, 125), "grunge": (110, 130)}
    assert kn.genre_style(ROCK, "classic") == ROCK
    grunge = kn.genre_style(ROCK, "grunge")
    assert grunge.role_level_db["kick"] > ROCK.role_level_db["kick"] and grunge.role_level_db["bass"] > ROCK.role_level_db["bass"]
    assert grunge.role_level_db["pad"] <= ROCK.role_level_db["pad"]
    for seed in range(10):
        for genre, window in ROCK.genre_windows.items():
            plan = _plan("космос", seed, genre)
            assert window.bpm[0] <= plan.bpm <= window.bpm[1] and plan.genre == genre


def test_grunge_tracks_model_more_low_than_classic(tracks):
    low = {g: [t.mix.a9_model for p, t, _pr, _b in tracks if p.genre == g] for g in ROCK.genre_windows}
    assert max(low["classic"]) < min(low["grunge"]), low
    for plan, track, _pr, _b in tracks:
        assert track.mix.a9_model >= kn.genre_style(ROCK, plan.genre).a9_model_low - kn.A9_MIN_GAIN, track.mix.a9_model
        assert track.mix.duck_roles == {"bass"}, "мягкий насос баса (ADR-0157 H7), power_chords без сайдчейна"


def _by_bar(events):
    out: dict = {}
    for e in events:
        out.setdefault(int(e.beat // BEATS_PER_BAR), []).append(e)
    return {bar: sorted(line, key=lambda e: (e.beat, e.midi)) for bar, line in out.items()}


def test_power_chords_have_no_third_and_sit_on_the_bass_root(tracks):
    """Пэд-«гитара» (``saw``, В4: ``fuzz`` проиграл) — прима, чистая квинта, октава: интервалы голосов такта только 0/5/7 (терции нет
    никогда), прима — тон баса такта (без материала бас стоит на приме аккорда); квинта — почти во всех тактах."""
    with_fifth = bars = 0
    for _plan_, track, _program, _by in tracks:
        assert track.history_key.pad_figure == "power_chords" and track.parts["pad"].synth_or_sample == "saw"
        bass = _by_bar(track.parts["bass"].pitches)
        for bar, line in _by_bar(track.parts["pad"].pitches).items():
            pcs = {e.midi % 12 for e in line}
            assert all((a - b) % 12 in (0, 5, 7) for a in pcs for b in pcs), (bar, pcs)
            if bar in bass:
                root = bass[bar][0].midi % 12
                assert pcs <= {root, (root + pad.FIFTH) % 12}, (bar, pcs, root)
                bars += 1
                with_fifth += len(pcs) == 2
    assert bars and with_fifth / bars > 0.9, (with_fifth, bars)


def test_power_voicing_drops_a_diminished_fifth_and_inverts_under_a_low_top():
    assert pad.power_voicing(11, 5, (43, 67)) == (47, 59), "vii°: квинта не чистая — прима и октава"
    assert pad.power_voicing(9, 4, (43, 67)) == (45, 52, 57)
    assert pad.power_voicing(9, 4, (43, 54)) == (45, 52), "октава не помещается — без неё"
    assert pad.power_voicing(5, 0, (43, 54)) == (48, 53), "квинта над примой не помещается — под примой"


def _section_at(track, bar: int):
    """Секция формы трека, в которой такт ``bar``."""
    return next(sec for start, sec in _bars(track) if start <= bar < start + sec.bars)


def _feel(track):
    """Стиль трека с подстилем его каркаса (``knowledge.feel_style``, #3594)."""
    return kn.feel_style(ROCK, track.history_key.kit)


def test_bass_riff_plays_eighths_on_the_chord_tones(tracks):
    """Бас — восьмые риффа (партии секции подстиля, иначе ``Style.bass_riffs``; в ритме темы — ``bass_follow``):
    первая восьмая такта — тон такта, дальше тон, его чистая квинта или октава (без хроматики, в ладу); такт заполнен
    целиком."""
    for _plan_, track, _program, _by in tracks:
        assert track.history_key.bass_figure == "riff"
        scale = kn.scale_pitch_classes(track.key.root, track.key.mode)
        for bar, line in _by_bar(track.parts["bass"].pitches).items():
            root = line[0].midi % 12
            assert line[0].beat % BEATS_PER_BAR == 0
            assert all((e.beat * 2) % 1 == 0 and e.dur_beats % 0.5 == 0 for e in line), "восьмые"
            assert all((e.midi - root) % 12 in (0, 7) and e.midi % 12 in scale for e in line), (bar, line)
            assert sum(e.dur_beats for e in line) == BEATS_PER_BAR


def _steps(pattern: str):
    return [i for i, ch in enumerate(pattern) if ch != "."]


def test_backbeat_on_two_and_four_and_kick_on_the_rock_pattern(tracks):
    """Малый — бэкбит 2/4 или малый партии секции подстиля в каждом такте тела, кроме такта филла (там молчит с
    последней доли); бочка секции без своей — рисунок окна по энергии."""
    for plan, track, _program, _by in tracks:
        clap, kick = track.parts["clap"].grid, track.parts["kick"].grid
        feel = _feel(track)
        for start, sec in _bars(track):
            if kn.section_kind(sec.name) not in ROCK.backbeat_kinds:
                continue
            part = kn.section_part(feel, sec.name)
            for bar in range(start, start + sec.bars - 1):
                assert _bar_steps(clap, bar) == (_steps(part.snare) if part.snare else list(BACKBEAT)), (sec.name, bar)
                if part.kick:
                    assert _bar_steps(kick, bar) == _steps(part.kick), (sec.name, bar)
            last = start + sec.bars - 1
            assert all(s < 12 for s in _bar_steps(clap, last)) and all(s < 12 for s in _bar_steps(kick, last))


# ── #3594: подстили трека и песня — секции различаются партиями, гитара и бас служат теме ────────────────────────────

def _pitched(part, start: int, bars: int):
    return [e for e in part.pitches if start * BEATS_PER_BAR <= e.beat < (start + bars) * BEATS_PER_BAR]


@pytest.fixture(scope="module")
def feels():
    """Треки всех подстилей рока (каркасы ``ROCK.kits``): сиды подряд, по 3 трека сета с хуками RTTTL, пока каждый
    каркас не встретится в треке с бриджем и в треке со вторым куплетом."""
    found: dict = {}
    for seed in range(40):
        plan = _plan("нирвана", seed, "grunge" if seed % 2 else "classic", hooks=True)
        history: list = []
        for no in (1, 2, 3):
            track = compose(plan, no, history=history, melodies=MELODIES)
            validate(track)
            history.insert(0, track_composition(track))
            names = {sec.name for sec in track.form.sections}
            for need in ("bridge", "verse2"):
                if need in names:
                    found.setdefault((track.history_key.kit, need), (track, render(track, "A")))
        if len(found) == 2 * len(ROCK.kits):
            break
    assert {kit for kit, _n in found} == set(ROCK.kits), sorted(found)
    return found


def test_set_tracks_take_different_rock_feels(tracks):
    """Соседние треки сета — разные подстили (каркас не повторяет прошлый трек, ``compose._kit``), и каждый каркас рока
    — подстиль с партиями секций (``Style.kit_parts``)."""
    assert set(ROCK.kit_parts) == set(ROCK.kits)
    for (plan, a, _p, _b), (plan2, b, _p2, _b2) in zip(tracks, tracks[1:]):
        if plan is plan2:
            assert a.history_key.kit != b.history_key.kit


def _signature(track, start: int, sec):
    """Что играет секция (без последнего такта — там филл): бочка, малый, хэты, ритм гитары, бас."""
    body = range(start, start + sec.bars - 1)
    drums = tuple(tuple(tuple(_bar_steps(track.parts[r].grid, bar % (len(track.parts[r].grid.steps) // STEPS_PER_BAR)))
                        for bar in body) for r in ("kick", "clap", "hats"))
    tonal = tuple(tuple(tuple(sorted({(round(e.beat % BEATS_PER_BAR * 4), round(e.dur_beats * 4), e.accent)
                                      for e in _pitched(track.parts[r], bar, 1)})) for bar in body)
                  for r in ("pad", "bass"))
    return drums, tonal


def test_body_sections_are_not_one_row_and_feels_differ(feels):
    """Тело трека — не одна строка (Шифу 13.10: «нет никакого развития»): куплет, припев и бридж играют разное; и
    подстили между собой — разное (бочка/малый/хэты куплета и припева)."""
    for (kit, _need), (track, _program) in feels.items():
        rows = {kn.section_kind(sec.name).rstrip("2"): _signature(track, start, sec)
                for start, sec in _bars(track) if sec.name in ("verse", "chorus", "bridge")}
        assert len(set(rows.values())) == len(rows) >= 2, (kit, sorted(rows))
    drums = {kit: tuple(_signature(track, start, sec)[0] for start, sec in _bars(track) if sec.name in ("verse", "chorus"))
             for (kit, need), (track, _p) in feels.items() if need == "verse2"}
    assert len(set(drums.values())) == len(drums), "подстили отличаются ударными куплета и припева"


def test_guitar_and_bass_follow_the_theme(feels):
    """Гитара служит теме: ``unison`` — аккорд на каждой ноте темы такта, ``answer`` — глушёные удары только в паузах
    темы, ``accent`` — акцент 3 там, где нота темы, 1 — где её нет; бас ``bass_follow`` — тоника на восьмых нот темы."""
    checked = set()
    for (kit, _need), (track, _program) in feels.items():
        feel = _feel(track)
        lead = track.parts["lead"].pitches
        for start, sec in _bars(track):
            part = kn.section_part(feel, sec.name)
            for bar in range(start, start + sec.bars):
                notes = _pitched(track.parts["lead"], bar, 1)
                onsets = {round(e.beat * 4) % 16 for e in notes}
                sounding = {s % 16 for e in lead for s in range(round(e.beat * 4), max(round(e.beat * 4) + 1,
                                                                                       round((e.beat + e.dur_beats) * 4)))
                            if s // 16 == bar}
                hits = {round(e.beat * 4) % 16: e.accent for e in _pitched(track.parts["pad"], bar, 1)}
                if not part.follow or len(onsets) < 2 or not hits:
                    continue
                if part.follow == "unison":
                    assert set(hits) == onsets, (kit, sec.name, bar)
                elif part.follow == "answer" and len(set(hits) - sounding) >= 2:
                    assert not set(hits) & sounding, (kit, sec.name, bar)
                elif part.follow == "accent":
                    assert all((a == 3) == bool({s, s + 1} & onsets) for s, a in hits.items()), (kit, sec.name, bar)
                checked.add(part.follow)
                if part.bass_follow:
                    bass_on = {round(e.beat * 2) % 8 for e in _pitched(track.parts["bass"], bar, 1)}
                    assert bass_on == {0} | {s // 2 for s in onsets}, (kit, sec.name, bar)
    assert checked == set(kn.FOLLOW_MODES), checked


def test_theme_is_the_loudest_tonal_voice_and_sings_one_line_in_the_last_chorus(feels):
    """Тема — главный голос (Шифу 13.10: «тема практически не слышна»): уровень лида в модели выше гитары и всех ролей
    середины и верха (малый, хэты, томы, крэш); бочка и бас гранжа громче — они в полосе ниже 150 Гц, тему не глушат;
    последний припев — тема одним голосом (без параллельных терций ``drop2``)."""
    for (kit, _need), (track, _program) in feels.items():
        lv = track.mix.level_db
        assert all(lv["lead"] > lv[r] for r in ("pad", "clap", "hats", "toms", "fx")), (kit, lv)
        assert lv["lead"] - lv["pad"] >= 2.5, (kit, lv)
        start = 0
        for sec in track.form.sections:
            if sec.name in ("chorus2", "chorus3"):
                notes = _pitched(track.parts["lead"], start, sec.bars)
                assert len({e.beat for e in notes}) == len(notes), (kit, sec.name, "две ноты темы разом")
            start += sec.bars


def test_quiet_verse_loud_chorus_in_the_volume_arc(feels):
    """Дуга громкости программы (``trim`` после выравнивателя мастер-шины): куплет тише припева, последний припев —
    пик; интро и хвост блэнда — как у клуба (деки сводятся без скачка)."""
    for (kit, _need), (track, program) in feels.items():
        trim = {sec.name: offset for sec, (_b, offset, _l) in zip(track.form.sections, program.arc)}
        assert trim["intro"] == kn.SECTION_TRIM_DB["intro"][0] and trim["outro_tail"] == kn.SECTION_TRIM_DB["outro_tail"][0]
        assert trim["verse"] <= trim["chorus"] - 2.0, (kit, trim)
        last = trim.get("chorus2")
        assert last is None or last == max(trim.values()), (kit, trim)


def test_bridge_is_its_own_section(feels):
    """Бридж каждого подстиля — свой приём (брейкдаун полутемпом, стоп-тайм, тутти-чаг, Бо Диддли): бочка партии
    секции — она же триггер сайдчейна; бас — не больше двух нот в такте, кроме тутти-чага."""
    for (kit, need), (track, _program) in feels.items():
        if need != "bridge":
            continue
        part = kn.section_part(_feel(track), "bridge")
        names = [s.name for s in track.form.sections]
        start = dict((sec.name, s) for s, sec in _bars(track))["bridge"]
        for bar in range(start, start + 7):
            assert _bar_steps(track.parts["kick"].grid, bar) == _steps(part.kick), (kit, bar)
            if part.toms:
                assert _bar_steps(track.parts["toms"].grid, bar) == _steps(part.toms), (kit, bar)
        if track.mix.duck:
            assert track.mix.duck[names.index("bridge")].trigger == tuple(i for i, c in enumerate(part.kick) if c == "X")


def test_intro_and_outro_keep_the_track_tables(tracks):
    """Интро/аутро (блэнд двух дек) не трогает партия секций: гитара — ритмы ``Style.power_rhythms`` по кругу от такта
    формы, хэты — каркас трека."""
    for _plan_, track, _program, _by in tracks:
        hats = track.parts["hats"].grid
        kit = [i for i, ch in enumerate(ROCK.kits[track.history_key.kit]["hats"]) if ch != "."]
        for start, sec in _bars(track):
            if not sec.name.startswith(("intro", "outro")):
                continue
            for bar in range(start, start + sec.bars):
                assert _bar_steps(hats, bar % (len(hats.steps) // STEPS_PER_BAR)) == kit
                rhythm_ = ROCK.power_rhythms[bar % len(ROCK.power_rhythms)]
                onsets = sorted({round(e.beat % BEATS_PER_BAR * 4) for e in _pitched(track.parts["pad"], bar, 1)})
                assert onsets == [step for step, *_rest in rhythm_], (sec.name, bar)


def test_tom_fills_on_every_section_joint_and_crash_after_them(tracks):
    """Томы бьют в последнем такте секций (филл на стыке, рисунок ``Style.tom_fills`` по сиду) и том-грувом партии
    секции; крэш (``fx``, файл пака) — на первой доле секций куплет/припев/бридж; события программы = модель."""
    for _plan_, track, program, by_role in tracks:
        toms = track.parts["toms"]
        feel = _feel(track)
        assert toms.synth_or_sample in ROCK.drum_files["toms"]
        assert kn.SAMPLE_CATALOG[track.parts["fx"].synth_or_sample].role == "crash"
        for start, sec in _bars(track):
            groove = _steps(kn.section_part(feel, sec.name).toms)
            for bar in range(start, start + sec.bars):
                steps = _bar_steps(toms.grid, bar)
                if bar < start + sec.bars - 1 or "toms" not in sec.roles:
                    assert steps in ([], groove) or "toms" not in sec.roles, (sec.name, bar)
                    continue
                pattern = "".join("x" if i in steps else "." for i in range(STEPS_PER_BAR))
                assert pattern in {p.replace("X", "x") for p in ROCK.tom_fills}, (sec.name, pattern)
        model = sorted(round(i / 4, 3) for i, st in enumerate(toms.grid.steps) if st.on and _sounds(track, "toms", i / 4))
        assert sorted(round(e.beat, 3) for e in by_role["toms"] if e.amp > 0) == model, "события томов = модель"
        crash = sorted(round(e.beat, 3) for e in by_role["fx"])
        starts = [start * BEATS_PER_BAR for start, sec in _bars(track) if "fx" in sec.roles]
        assert set(starts) <= set(crash), (starts, crash)
        assert kn.SAMPLE_CATALOG[toms.synth_or_sample].path in program.sample_files


def _sounds(track, role: str, beat: float) -> bool:
    start = 0
    for sec in track.form.sections:
        if start * 4 <= beat < (start + sec.bars) * 4:
            return role in sec.roles
        start += sec.bars
    return False


def test_form_is_verse_chorus_bridge_and_the_theme_is_in_the_chorus(tracks):
    """Форма — куплет/припев/бридж (данные стиля), первый трек — припев сразу после интро; хук темы — в припеве
    (вид ``drop``), читаемым лидом (``THEME_LEAD_OK``)."""
    for _plan_, track, _program, _by in tracks:
        names = [sec.name for sec in track.form.sections]
        assert {"verse", "chorus"} <= set(names) and names[:2] == ["intro", "intro_low"]
        assert names[-2:] == ["outro", "outro_tail"]
        assert track.parts["lead"].synth_or_sample in kn.THEME_LEAD_OK
        chorus = next(start for start, sec in _bars(track) if sec.name == "chorus")
        assert any(chorus * 4 <= e.beat < (chorus + 8) * 4 for e in track.parts["lead"].pitches)
    firsts = [t for p, t, _pr, _b in tracks if t.track_id.split(":")[1] == "01"]
    assert all(t.form.sections[2].name == "chorus" for t in firsts)


def test_kit_is_muldjord_pack_files(tracks):
    for _plan_, track, program, _by in tracks:
        assert track.parts["kick"].synth_or_sample.startswith("muldjord_kd")
        for role in ("clap", "hats", "toms"):
            assert track.parts[role].synth_or_sample in ROCK.drum_files[role]
        files = {kn.SAMPLE_CATALOG[track.parts[r].synth_or_sample].path for r in ("kick", "clap", "hats", "toms", "fx")}
        assert files <= program.sample_files and not program.samples, "удары — файлы паков, не play()"


@pytest.mark.parametrize("text", ["рок", "включи рок сет", "рок-н-ролл", "рок н ролл", "хард-рок", "хардрок",
                                  "гранж", "гранжевый сет", "rock", "grunge", "hard rock", "rock'n'roll"])
def test_rock_words_pick_the_style(text):
    assert match_style_text(text) == "rock"


@pytest.mark.parametrize("text", ["роковая любовь", "рокки", "барокко", "рокот", "rocket", "rocky", "рокировка"])
def test_words_close_to_rock_words_are_not_a_style(text):
    assert match_style_text(text) is None


# ── Ф2 (#3530): рок играет объявленный аккорд такта ──────────────────────────────────────────────────────────────

A_MINOR = Key(9, "minor")


def test_riff_stands_on_the_declared_chord_and_takes_only_a_perfect_fifth():
    """Рифф — тоны объявленного аккорда: такт на терции мажорной V ля минора — G# (вводный тон, не G натуральной v);
    на уменьшённом аккорде тритона нет — R/F/O риффа стоят на приме (аудит П6)."""
    reg = ROCK.registers["bass"]
    major_v = bass.riff(ROCK, A_MINOR, [(0, Chord(4, (64, 68, 71), "maj"))], "bass", reg,
                        {0: bass.BassTone(bass.ANCHORS["third"])})
    assert major_v.pitches[0].midi % 12 == 8
    assert {e.midi % 12 for e in major_v.pitches} <= set(chord_tones(A_MINOR, Chord(4, (), "maj")))
    dim = bass.riff(ROCK, A_MINOR, [(0, Chord(1, (59, 62, 65), "dim"))], "bass", reg)
    assert {e.midi % 12 for e in dim.pitches} == {11}


def test_power_chord_takes_the_fifth_of_the_declared_quality():
    """Пауэр-аккорд без терции: качество меняет только квинту — у ув. трезвучия она не чистая, пауэр-аккорд без неё."""
    reg = ROCK.registers["pad"]
    aug = pad.power_chords(ROCK, Key(0, "major"), [(0, Chord(0, (60, 64, 68), "aug"))], "pad", reg)
    assert {e.midi % 12 for e in aug.pitches} == {0}
    plain = pad.power_chords(ROCK, Key(0, "major"), [(0, Chord(0, (60, 64, 67), "maj"))], "pad", reg)
    assert {e.midi % 12 for e in plain.pitches} == {0, 7}


# ── #3594: ноты темы и гитары — «ноты не те» (Шифу 13.10, трек 1 сета set82119) ──────────────────────────────────

#: Записи библиотеки робота (``rtttl_melodies``, 13.10): «Litium» — арпеджио хроматических аккордов Lithium (E, G#, C#,
#: A, C, D, B), «Teen Spirit» — диатоническая.
LITIUM = ("Litium:d=4,o=6,b=125:8e,b,8e,8g_5,d_,8g_5,8c_,g_,8c_,8a5,e,8a5,8c,g,8c,8d,a,8d,8b5,f_,8b5,8d,a,8d,8e,b,8e,"
          "8g_5,d_,8g_5,8c_,g_,8c_,8a5,e,8a5,8c,g,8c,8d,a,8d,8b5,f_,8b5,8d,a,8d,2e")
TEENSPIR = ("TeenSpir:d=4,o=6,b=70:32p,32c#.,16e.,16f#.,8a.5,16f#.,16e.,16c#.,32d.,16c#.,8b5,16a.5,32b.5,16c#.,32b.5,"
            "32a.5,8g#5,32a.5,8g#5,32a.5,8g#5,32a.5,16g#.5,32a.5,32g#.5,32g#.5,16f#.5,32a.5,8g#5,32a.5,8g#5,32a.5,8g#5,"
            "32a.5,32g#.5,32g#.5,16f#.5")


def _rock_track(melodies, no: int = 1, seed: int = 0):
    profile = dataclasses.replace(seeded_profile("нирвана", "rock"), hook_ids=tuple(melodies))
    return compose(seeded_plan(profile, seed, set_id=f"nt{seed}", genre="classic"), no, melodies=melodies)


def test_rock_passes_over_a_chromatic_hook_it_cannot_harmonize():
    """Хук, у которого больше 10 % нот хука и темы вне лада трека, рок не берёт (``Style.hook_in_key_min``): аккорд модели
    — ступень лада, и хроматика темы над диатоническим пауэр-аккордом звучала «не теми нотами»; берётся следующая
    мелодия, лид трека в ладу."""
    track = _rock_track({"litium": LITIUM, "teenspir": TEENSPIR})
    assert track.hook is not None and track.hook.source == "teenspir"
    assert hooks.in_key_share(track.hook, track.key) >= ROCK.hook_in_key_min
    scale = set(kn.scale_pitch_classes(track.key.root, track.key.mode))
    assert all(e.midi % 12 in scale for e in track.parts["lead"].pitches)


def test_lead_plays_the_hook_transposed_whole():
    """Лид = хук после транспонирования: в припеве (вид ``drop``) — ноты хука подряд, те же доли и длины, интервалы
    между нотами — как у источника RTTTL (ни одна нота не сдвинута ладом или октавой внутри мотива)."""
    track = _rock_track({"teenspir": TEENSPIR})
    hook = track.hook
    _name, mbpm, notes = parse_rtttl(fix_octaves(TEENSPIR, "teenspir"))
    src = hooks._onsets(notes, hooks.time_scale(mbpm, track.bpm))[:len(hook.notes)]
    assert [b - a for (_x, _y, a), (_u, _v, b) in zip(src, src[1:])] == \
        [b.midi - a.midi for a, b in zip(hook.notes, hook.notes[1:])]
    start = next(s for s, sec in _bars(track) if sec.name == "chorus") * BEATS_PER_BAR
    lead = [e for e in track.parts["lead"].pitches if start <= e.beat < start + hook.bars * BEATS_PER_BAR]
    assert [(e.beat - start, e.midi) for e in lead] == [(e.beat, e.midi) for e in hook.notes]


def test_guitar_plays_only_the_bar_chord(feels):
    """Ноты гитары ∈ аккорд такта: пауэр-аккорд — прима и квинта объявленного аккорда (у ритма, идущего за темой,
    тоже — тема задаёт только ритм), бас — тон аккорда."""
    for (kit, _need), (track, _program) in feels.items():
        chords, start = {}, 0
        for sec in track.form.sections:
            for i, chord in enumerate(track.harmony.progression.get(sec.name, ())):
                chords[start + i] = chord
            start += sec.bars
        for role in ("pad", "bass"):
            for e in track.parts[role].pitches:
                chord = chords[int(e.beat // BEATS_PER_BAR)]
                pcs = set(harmony.chord_pcs(ROCK, track.key, chord.degree, chord.quality or None))
                assert e.midi % 12 in pcs, (kit, role, e)
