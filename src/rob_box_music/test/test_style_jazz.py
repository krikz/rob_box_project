"""ADR-0153 S6: стиль ``jazz`` — запись ``knowledge.STYLES``: окна ``swing``/``ballad`` (90–125), свинг восьмых 1.5–1.7
(эталон джаз-кафе 1.54, не триоль 2.0), форма head–solo–head (голова — тема целиком), соло по сменам темы с каденцией
ii–V–I, walking контрабасового регистра, comping нонаккордами мимо сильных долей, райд и педаль хэта файлами паков.
Поведение — на ``SetPlan``/``Track``/событиях программы Renardo (симулятор ``render.events``); клуб побайтно тот же —
``test_style_same_tracks``. Тесты дешёвые (бюджет пакета, ``conftest``): 2 сида × 2 трека + один трек с материалом.
"""

from __future__ import annotations

import dataclasses
import random
import statistics

import pytest

from melodies import MELODIES
from rob_box_music import knowledge as kn
from rob_box_music.arrange import bass as bass_mod
from rob_box_music.arrange import harmony, lead, mix
from rob_box_music.arrange import compose as cp
from rob_box_music.diversity import track_composition
from rob_box_music.model import BEATS_PER_BAR, Key, Part, PitchEvent, validate
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import ThemeProfile, match_style_text, match_window_text, seeded_profile
from test_theme_whole import material

JAZZ = kn.STYLES["jazz"]
#: Цель свинга S6 (эталон джаз-кафе 07.10: 1.54 [1.13..2.39]): ratio долгой/короткой восьмой 1.5–1.7.
RATIO = (1.5, 1.7)
STRONG = (0, 8)


def _plan(theme: str, seed: int, hooks: bool = True, genre=None):
    profile = seeded_profile(theme, "jazz")
    profile = dataclasses.replace(profile, hook_ids=tuple(MELODIES)) if hooks else profile
    return seeded_plan(profile, seed, set_id=f"jz{seed}", genre=genre)


@pytest.fixture(scope="module")
def tracks():
    """2 окна × 2 трека (окно ``swing`` — хуки RTTTL, ``ballad`` — мотив лида): (план, трек, события по ролям)."""
    out = []
    for seed, genre in enumerate(JAZZ.genre_windows):
        plan = _plan("кафе", seed, hooks=seed == 0, genre=genre)
        history: list = []
        for no in (1, 2):
            track = cp.compose(plan, no, history=history, melodies=MELODIES if seed == 0 else None)
            validate(track)
            program = render(track, "A")
            _p, events = program_events(program.code, program.form_beats, loops=True)
            role_of = {slot: role for role, slot in program.slots.items()}
            by_role: dict = {}
            for ev in events:
                by_role.setdefault(role_of[ev.slot], []).append(ev)
            out.append((plan, track, by_role))
            history.insert(0, track_composition(track))
    return out


def _sections(track):
    start = 0
    for sec in track.form.sections:
        yield start, sec
        start += sec.bars


def _in(track, kind: str):
    """(первый такт, секция) секций вида ``kind``."""
    return [(start, sec) for start, sec in _sections(track) if kn.section_kind(sec.name) == kind]


def _notes(part, start: int, bars: int):
    lo, hi = start * BEATS_PER_BAR, (start + bars) * BEATS_PER_BAR
    return sorted((e for e in part.pitches if lo <= e.beat < hi), key=lambda e: (e.beat, e.midi))


def test_tempo_windows_and_swing_of_the_set():
    """Окна 90–125 (эталон ≈ 96 [90..121]); темп сета — в окне; свинг — доля восьмой 0.20–0.25 → ratio 1.50–1.67."""
    bpms = [w.bpm for w in JAZZ.genre_windows.values()]
    assert (min(b for b, _ in bpms), max(b for _, b in bpms)) == (90, 125)
    assert kn.genre_style(JAZZ, next(iter(JAZZ.genre_windows))) == JAZZ, "поля стиля — первое окно"
    for seed in range(12):
        plan = _plan("кафе", seed, hooks=False)
        window = JAZZ.genre_windows[plan.genre]
        assert window.bpm[0] <= plan.bpm <= window.bpm[1], (seed, plan.genre, plan.bpm)
        assert RATIO[0] - 1e-9 <= (1 + plan.swing) / (1 - plan.swing) <= RATIO[1], (seed, plan.swing)


def _ratio(events) -> list:
    """(долгая/короткая) восьмая по онсетам: доля с онсетом на начале и одним онсетом внутри доли."""
    onsets = sorted({round(e.beat, 6) for e in events})
    out = []
    for beat in sorted({int(b) for b in onsets}):
        inner = [b - beat for b in onsets if beat < b < beat + 1 - 1e-9]
        if beat in onsets and len(inner) == 1:
            out.append(inner[0] / (1 - inner[0]))
    return out


def test_swing_ratio_by_model_onsets_of_the_ride_and_the_soloist(tracks):
    """Райд «динь-да-динь» и восьмые солиста: ratio по онсетам событий программы = (1 + swing)/(1 − swing) в 1.5–1.7."""
    for plan, _track, by_role in tracks:
        target = (1 + plan.swing) / (1 - plan.swing)
        for role in ("hats", "lead"):
            ratios = _ratio(by_role[role])
            assert len(ratios) > 20, role
            med = statistics.median(ratios)
            assert RATIO[0] <= med <= RATIO[1] and med == pytest.approx(target, abs=0.02), (role, med)


def test_form_is_head_solo_head(tracks):
    """Голова (вид дропа: тема/хук) → соло (16 тактов, свой вид) → вторая голова (хук в терциях); соло не повторяет
    хук — у вида ``solo`` развития хука нет, ноты строит солист."""
    for _plan_, track, _by in tracks:
        kinds = [kn.section_kind(s.name) for s in track.form.sections if "lead" in s.roles]
        assert kinds[0] == "drop" and kinds[-1] == "drop2" and set(kinds[1:-1]) == {"solo"}, kinds
        start, solo = _in(track, "solo")[0]
        assert solo.bars == 16 and _notes(track.parts["lead"], start, solo.bars)
        assert track.history_key.template in JAZZ.forms


def _solo_notes(track):
    return [(start, sec, _notes(track.parts["lead"], start, sec.bars)) for start, sec in _in(track, "solo")]


def test_soloist_plays_chord_tones_on_the_beats_and_never_leaps_over_an_octave(tracks):
    """Соло: нота на доле — тон аккорда своего такта (≥ 60 %, здесь — каждая), скачок ≤ октавы, хроматика
    (вне лада и вне аккорда) — только на слабых долях как подход полутоном к ноте на доле."""
    chromatic = 0
    for _plan_, track, _by in tracks:
        chords = {}
        for start, sec in _sections(track):
            for i, chord in enumerate(track.harmony.progression[sec.name]):
                chords[start + i] = set(harmony.chord_pcs(JAZZ, track.key, chord.degree, chord.quality or None))
        scale = kn.scale_pitch_classes(track.key.root, track.key.mode)
        for _start, _sec, notes in _solo_notes(track):
            on_beat = [e for e in notes if (e.beat * 4) % 4 == 0]
            hits = sum(e.midi % 12 in chords[int(e.beat // BEATS_PER_BAR)] for e in on_beat)
            assert hits / len(on_beat) >= 0.6, (hits, len(on_beat))
            assert all(abs(b.midi - a.midi) <= 12 for a, b in zip(notes, notes[1:]))
            for a, b in zip(notes, notes[1:]):
                bar_tones = chords[int(a.beat // BEATS_PER_BAR)]
                if a.midi % 12 not in scale and a.midi % 12 not in bar_tones:
                    chromatic += 1
                    assert (a.beat * 4) % 4 != 0 and abs(b.midi - a.midi) == 1, (a, b)
    assert chromatic > 0, "солист подходит хроматически"


def test_solo_ends_with_a_two_five_one_cadence(tracks):
    """Последние такты каждого соло — ii–V–I: ii диатоническая, V — мажорная (доминанта и в миноре), I — тоника."""
    for _plan_, track, _by in tracks:
        for start, sec in _in(track, "solo"):
            tail = track.harmony.progression[sec.name][-3:]
            assert [c.degree for c in tail] == [1, 4, 0], track.track_id
            assert tail[1].quality == "maj"


def test_solo_plays_the_changes_of_the_theme_and_the_head_the_whole_theme():
    """Тема целиком (#3523) — голова: 16 тактов материала в первой голове; соло идёт по аккордам темы (такт за тактом)
    до каденции; вторая голова — хук (не тема)."""
    m = material()
    profile = ThemeProfile("", "jazz", 110, 0, "major", (), None)
    plan = seeded_plan(profile, 7)
    plan = dataclasses.replace(plan, tracks=(dataclasses.replace(plan.track(1), material=m.material_id),))
    track = cp.compose(plan, 1, materials={m.material_id: m})
    validate(track)
    assert track.hook is not None and track.hook.theme_bars == 16
    (h_start, head), = _in(track, "drop")
    assert head.bars >= 16
    (s_start, solo), = _in(track, "solo")
    theme = [c.degree for c in track.harmony.progression[head.name][:16]]
    changes = [c.degree for c in track.harmony.progression[solo.name]]
    assert changes[:-3] == theme[:solo.bars - 3] and changes[-3:] == [1, 4, 0]


def test_walking_bass_in_the_double_bass_register_approaches_within_one_beat(tracks):
    for _plan_, track, _by in tracks:
        part = track.parts["bass"]
        assert track.history_key.bass_figure == "walking" and part.register == JAZZ.registers["bass"] == (28, 50)
        by_bar: dict = {}
        for e in sorted(part.pitches, key=lambda e: e.beat):
            by_bar.setdefault(int(e.beat // BEATS_PER_BAR), []).append(e)
        scale = kn.scale_pitch_classes(track.key.root, track.key.mode)
        for bar, line in by_bar.items():
            assert [e.beat % BEATS_PER_BAR for e in line] == [0, 1, 2, 3], "четверти"
            assert line[-1].dur_beats <= JAZZ.approach_max_beats == 1.0
            if bar + 1 in by_bar:
                step = abs(line[-1].midi - by_bar[bar + 1][0].midi)
                # #3549: полутон трётся о лид — подход ступенью лада (bass.against_lead)
                assert step == 1 or (step == 2 and line[-1].midi % 12 in scale), "подход на 4-й доле"


def test_comping_misses_strong_beats_and_plays_ninth_chords(tracks):
    seen = 0
    for _plan_, track, _by in tracks:
        if track.history_key.pad_figure != "comping":
            continue
        seen += 1
        pad = track.parts["pad"]
        assert all(round(e.beat * 4) % 16 not in STRONG for e in pad.pitches), "мимо долей 1 и 3"
        sizes = {len(c.voicing) for cs in track.harmony.progression.values() for c in cs}
        assert sizes <= {4, 5} and 5 in sizes, "септ- и нонаккорды (b9 не берётся — тогда 4 звука)"
    assert seen >= 2


def test_kit_is_ride_and_pedal_hat_files_without_sidechain(tracks):
    for _plan_, track, by_role in tracks:
        assert track.parts["hats"].synth_or_sample in JAZZ.drum_files["hats"]
        assert track.parts["clap"].synth_or_sample == "sonicpi_drum_cymbal_pedal"
        clap = {round(e.beat % BEATS_PER_BAR, 3) for e in by_role["clap"]}
        assert clap <= {1.0, 3.0}, "педаль хэта на 2 и 4"
        assert not track.mix.duck_roles and track.mix.a9_model >= JAZZ.a9_model_low


def test_solo_generator_on_a_two_five_one_by_seed():
    """``lead.solo`` сам по себе: 40 сидов по ii–V–I–I в до мажоре — ноты на долях — тоны аккорда, скачок ≤ 12, все
    ноты в регистре, хвост фраз — пауза."""
    key = Key(0, "major")
    chords = [(b, harmony.sym(key, d)) for b, d in enumerate((1, 4, 0, 0, 1, 4, 0, 0))]
    for seed in range(40):
        notes = lead.solo(JAZZ, key, chords, (62, 81), random.Random(seed))
        assert notes and all(62 <= e.midi <= 81 for e in notes)
        for e in notes:
            if (e.beat * 4) % 4 == 0:
                pcs = harmony.chord_pcs(JAZZ, key, chords[int(e.beat // 4)][1].degree)
                assert e.midi % 12 in pcs, (seed, e)
        assert all(abs(b.midi - a.midi) <= lead.MAX_LEAP for a, b in zip(notes, notes[1:]))
        assert all(e.beat + e.dur_beats <= 8 * BEATS_PER_BAR for e in notes)


@pytest.mark.parametrize("text", ["джаз", "включи джаз сет на тему Моцарт", "джазовый сет", "jazz", "свинг",
                                  "сыграй свинг про котов", "бибоп"])
def test_jazz_words_pick_the_style(text):
    assert match_style_text(text) == "jazz"


@pytest.mark.parametrize("text", ["свинья", "свин", "джаггер", "рок со свингом", "лоуфай"])
def test_words_close_to_jazz_words_are_not_jazz(text):
    assert match_style_text(text) != "jazz"


def test_window_words_pick_the_jazz_window_and_only_in_jazz():
    assert match_window_text("jazz", "включи свинг сет") == "swing"
    assert match_window_text("jazz", "джазовая баллада про луну") == "ballad"
    assert match_window_text("jazz", "джаз сет") is None
    assert match_window_text("rock", "рок-баллада") is None, "у рока окна ballad нет"


# ── #3549: микс и голосоведение джаза по приёмке S7 (низ 0.58, середина 0.41, crest 13.1, LR 0.99; м2 лид–бас 100 %) ──

def _sounding(events, beat):
    return [e for e in events if e.beat <= beat + 1e-6 and beat < e.beat + e.dur_beats - 1e-6]


def test_bass_never_a_semitone_from_the_lead_and_no_parallels(tracks):
    """Метрики аудита теории (``arranger_theory_audit``: «лид–бас м2», «пар.5/8»), но по всем голосам лида: ни одна нота
    лида не звучит на м2/м9 (или б7) от звучащей ноты баса; соседние онсеты лида не дальше доли не идут с басом в одну
    сторону параллельными квинтами или октавами. До #3549 — 20 нот на трек, параллели у 62 % треков."""
    for _plan_, track, _by in tracks:
        bass = track.parts["bass"].pitches
        lead_ev = sorted(track.parts["lead"].pitches, key=lambda e: (e.beat, e.midi))
        clashes = [(e.beat, e.midi, b.midi) for e in lead_ev for b in _sounding(bass, e.beat)
                   if (e.midi - b.midi) % 12 in (1, 11)]
        assert not clashes, clashes[:3]
        onsets = sorted({e.beat for e in lead_ev})
        for t0, t1 in zip(onsets, onsets[1:]):
            b0, b1 = _sounding(bass, t0), _sounding(bass, t1)
            if not (b0 and b1) or t1 - t0 > 1.0 + 1e-6:
                continue
            p, m = b0[0].midi, b1[0].midi
            for a in (e.midi for e in lead_ev if e.beat == t0):
                for b in (e.midi for e in lead_ev if e.beat == t1):
                    assert not ((b - a) * (m - p) > 0 and (a - p) % 12 == (b - m) % 12 in (0, 7)), (t0, t1, a, b, p, m)


def test_against_lead_moves_the_clashing_approach_and_leaves_other_styles_alone():
    """Подход полутоном под долгой нотой лида на м9 уходит (другая сторона, ступень или другой тон такта); ритм тот же.
    У клуба правил нет — партия та же объектом (клуб побайтно прежний, ``test_style_same_tracks``)."""
    key = Key(0, "major")
    chords = [(0, harmony.sym(key, 4)), (1, harmony.sym(key, 0))]  # G → C
    walk = bass_mod.walking(JAZZ, key, chords, "bass", JAZZ.registers["bass"])
    line = sorted(walk.pitches, key=lambda e: e.beat)
    assert abs(line[3].midi - line[4].midi) == 1, "подход полутоном на 4-й доле"
    held = PitchEvent(line[3].midi + 1 + 36, 3.0, 2.0)  # м9 над подходом, тянется через долю 1 следующего такта
    out = bass_mod.against_lead(JAZZ, key, chords, walk, (held,))
    near = [e for e in out.pitches if e.beat < held.beat + held.dur_beats and held.beat < e.beat + e.dur_beats]
    assert near and all((held.midi - e.midi) % 12 not in JAZZ.bass_lead_clash for e in near), near
    assert [(e.beat, e.dur_beats, e.accent) for e in out.pitches] == [(e.beat, e.dur_beats, e.accent) for e in line]
    club = kn.STYLES["club"]
    assert not club.bass_lead_clash and not club.bass_lead_parallels
    assert bass_mod.against_lead(club, key, chords, walk, (held,)) is walk


def test_jazz_levels_put_the_middle_over_the_double_bass():
    """Уровни ролей (шкала модели) по эталону джаз-кафе (низ 0.27, середина 0.73): comping громче баса, лид не тише
    его больше чем на 6 дБ, бочка «пёрышком» тише баса, бас на ≥ 4 дБ тише, чем у lo-fi (там низ — сам бит); синты
    comping дотягивают до цели пэда без провала больше 2 дБ (``strings`` на потолке ``amp`` −41.7 — снят)."""
    lv = JAZZ.role_level_db
    assert lv["pad"] > lv["bass"] and lv["lead"] >= lv["bass"] - 6.0 and lv["kick"] < lv["bass"]
    assert lv["bass"] <= kn.STYLES["lofi"].role_level_db["bass"] - 4.0
    for synth in {s for fam in JAZZ.timbres.values() for s in fam["pad"]}:
        assert mix._cap("pad", Part("pad", synth, None, (), lv["pad"], JAZZ.registers["pad"])) >= lv["pad"] - 2.0, synth


def test_jazz_comping_is_wider_than_the_centre_but_not_club_wide():
    """Ширина (эталон LR 0.85; робот: ±0.4 — 0.99, ±0.6 — 0.93–0.98, ±0.9 — 0.35–0.55): comping — два расстроенных
    голоса между 0.6 и 0.9; бас, бочка и лид — в центре."""
    pad = JAZZ.stereo["pad"]
    assert pad["detune"] > 0 and 0.6 < pad["pan"] < 0.9
    assert set(JAZZ.stereo) == {"pad"}


def test_jazz_master_drops_the_compressor_for_crest():
    """Динамика (эталон crest 17.8, робот S7 13.1): профиль мастер-шины джаза — ручки SynthDef, компрессор выключен
    (ratio 1, makeup 0), выравниватель тянет к уровню, при котором crest 17 дБ встаёт пиком под потолок лимитера
    (−1 dBFS). Прочие стили — без профиля; следующий трек возвращает дефолты (все ручки выставляются каждый старт)."""
    jazz = mix.set_master(3, JAZZ)
    assert set(jazz) <= set(kn.MASTER_DEFAULTS) and jazz["trim"] == kn.ENERGY_TRIM_DB[3]
    assert jazz["cmpRatio"] == 1.0 and jazz["makeup"] == 0.0 and jazz["lvlTarget"] + 17.0 <= 1.0
    assert mix.set_master(3, kn.STYLES["club"]) == mix.set_master(3) == {"trim": kn.ENERGY_TRIM_DB[3], **kn.SET_LEVELER}
    assert all(not st.master for name, st in kn.STYLES.items() if name != "jazz")
