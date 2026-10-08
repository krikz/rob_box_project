"""ADR-0153 S2: стили ``synthwave`` и ``chiptune`` — записи ``knowledge.STYLES``, ``pad.arp``, выбор стиля кодом.

Поведение — на ``SetPlan``/``Track``/событиях рендера: темп в окне стиля, бочки и синты из таблиц стиля (только
замеренные), арпеджио из тонов аккорда в регистре по сетке 16-х, сайдчейн мягкий (synthwave) или его нет (chiptune),
A9-модель стиля на всех комбинациях тембров. Клуб побайтно тот же — ``test_style_same_tracks``.
"""

from __future__ import annotations

import itertools
from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange import mix, pad
from rob_box_music.arrange.compose import (
    BASS_GENERATORS, PAD_GENERATORS, _bar_chords, compose, form_spec, theme_form,
)
from rob_box_music.diversity import kick_name, track_composition, track_history
from rob_box_music.model import BEATS_PER_BAR, STEPS_PER_BAR, Chord, Key, TrackError, blend_bars, validate
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import match_style, match_style_text, seeded_profile, style_for, style_marks

NEW = ("synthwave", "chiptune")
#: Строки таблицы тем с разными семьями (cyber → hard, kids → bright) и тема не из таблицы (семья по сиду). Короткий
#: список: пакет музыки в CI идёт под таймаутом 240 с (PR #3501), клубные темы уже покрыты гардами клуба.
THEMES = ("киберпанк", "детский праздник", "бухгалтерский отчёт")
STEP = BEATS_PER_BAR / STEPS_PER_BAR


def _plan(theme: str, seed: int, style: str, **kw):
    return seeded_plan(seeded_profile(theme, style), seed, set_id=f"s{seed}", **kw)


# ── Генератор арпеджио ─────────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("name", NEW)
def test_arp_plays_chord_tones_in_register_one_per_sixteenth(name):
    style = kn.STYLES[name]
    key = Key(9, "minor")
    register = (50, 70)
    chords = [(0, Chord(0, (57, 60, 64))), (1, Chord(0, (57, 60, 64))), (4, Chord(5, (53, 57, 60)))]
    part = pad.arp(style, key, chords, "pulse", register)
    assert part.role == "pad" and part.register == register and part.synth_or_sample == "pulse"
    assert [s.on for s in part.grid.steps] == [True] * STEPS_PER_BAR
    by_bar = {bar: chord for bar, chord in chords}
    assert len(part.pitches) == STEPS_PER_BAR * len(chords)
    for i, ev in enumerate(part.pitches):
        bar, step = int(ev.beat // BEATS_PER_BAR), round((ev.beat % BEATS_PER_BAR) / STEP)
        tones = sorted(by_bar[bar].voicing)
        assert ev.midi in tones and register[0] <= ev.midi <= register[1], ev
        assert ev.midi == tones[style.arp_order[step % len(style.arp_order)] % len(tones)], (i, ev)
        assert ev.dur_beats == STEP and ev.accent == (3 if step % 4 == 0 else 2)
    assert {e.midi for e in part.pitches if e.beat < BEATS_PER_BAR} == {57, 60, 64}, "все тоны аккорда звучат"


def test_arp_order_wraps_on_chords_with_fewer_voices():
    style = replace(kn.STYLES["chiptune"], arp_order=(0, 1, 2, 3))
    part = pad.arp(style, Key(0, "major"), [(0, Chord(0, (60, 67)))], "square", (48, 70))
    assert [e.midi for e in part.pitches[:4]] == [60, 67, 60, 67]


# ── Таблицы стилей ─────────────────────────────────────────────────────────────────────────────────────────────


def test_windows_cover_the_style_tempos_and_kicks_are_the_style_own():
    """§3: synthwave 100–120, chiptune 120–160; бочки — свои файлы пака стиля (#3550: клубные ``deep``/``house`` и
    ``techno``/``garage`` делали стили «клубом с другим синтом»), с настоящим низом, не жёсткие рейва."""
    spans = {name: [w.bpm for w in kn.STYLES[name].genre_windows.values()] for name in NEW}
    assert min(lo for lo, _ in spans["synthwave"]) == 100 and max(hi for _, hi in spans["synthwave"]) == 120
    assert min(lo for lo, _ in spans["chiptune"]) == 120 and max(hi for _, hi in spans["chiptune"]) == 160
    for name in NEW:
        style = kn.STYLES[name]
        assert kn.genre_style(style, next(iter(style.genre_windows))) == style, "поля стиля — первое окно"
        for window in style.genre_windows.values():
            assert all(kn.KICK_SOUNDS[k].pack and kn.KICK_SOUNDS[k].low >= 0.9 for k in window.kick_pool), (
                name, window.kick_pool)
    assert {"synthwave", "retrowave", "chiptune"} <= kn.GENRE_NAMES


@pytest.mark.parametrize("name", NEW)
def test_bass_figures_never_hit_a_drop_kick_step(name):
    for window in kn.STYLES[name].genre_windows.values():
        drop = mix.kick_steps(window.looks[0][1].kick)
        for figure in window.bass_figures:
            assert not set(kn.BASS_FIGURES[figure].steps) & set(drop), (name, figure, drop)


def test_synthwave_drop_is_outrun_with_a_soft_pump_and_chiptune_has_no_sidechain():
    synth, chip = kn.STYLES["synthwave"], kn.STYLES["chiptune"]
    assert synth.looks[0][1].kick == kn.KICK_PATTERNS["outrun"]
    assert all(look.duck_depth <= 0.3 for w in synth.genre_windows.values() for _t, look in w.looks)
    assert chip.duck_roles == () and all(look.duck_depth == 0 for _t, look in chip.looks)
    assert "held" in synth.pad_figures and "arp" in synth.pad_figures and "arp" in chip.pad_figures


@pytest.mark.parametrize("name", NEW)
def test_every_synth_of_the_style_is_measured_and_leads_reach_their_target(name):
    """Только замеренные синты (ADR-0152 PR-2): громкость и полосы роли есть; лид на потолке ``amp`` достаёт цель роли;
    у каждого рисунка пэда в семье есть синт."""
    style = kn.STYLES[name]
    for family, roles in style.timbres.items():
        for role, synths in roles.items():
            for synth in synths:
                assert synth in kn.LANE_DB_AT_UNIT[role] and synth in kn.LAYER_BANDS[role], (family, role, synth)
                assert synth in kn.SYNTH_PALETTE[role], (family, role, synth)
        assert all(mix.layer_db(kn.LANE_DB_AT_UNIT["lead"][s], kn.AMP_EXPONENT.get(s, 1.0), kn.MAX_LAYER_AMP)
                   >= style.role_level_db["lead"] for s in roles["lead"]), family
        assert all(kn.LAYER_BANDS["lead"][s][0] < 0.05 for s in roles["lead"]), family
        figures = {f for w in style.genre_windows.values() for f in w.pad_figures}
        assert all(mix.pad_synths(style, family, f) for f in figures), family


@pytest.mark.parametrize("name", NEW)
def test_no_lead_whose_filter_crosses_nyquist_in_the_lead_register(name):
    """Робот 07.10: ``cs80lead`` (срез ``freq·12``) выше MIDI 76 рвёт выход — в новых стилях его нет."""
    style = kn.STYLES[name]
    top = style.registers["lead"][1]
    leads = {s for fam in style.timbres.values() for s in fam["lead"]}
    assert not {s for s in leads if kn.NYQUIST_MAX_MIDI.get(s, 128) < top}, leads


def test_pulse_bass_plays_only_octaves():
    """8-битный бас: ``pulse`` (низ 0.58) — только рисунком ``octave8``, а ``octave8`` — только им."""
    chip = kn.STYLES["chiptune"]
    assert all(mix.bass_synths(chip, fam, "octave8") == ("pulse",) for fam in chip.timbres)
    assert "pulse" not in mix.bass_synths(chip, "bright", "offbeat")
    assert "octave8" not in mix.bass_figures(kn.STYLES["club"], "bright"), "у клуба pulse в семьях баса нет"


def test_octave8_alternates_root_and_octave_off_the_beats():
    chip = kn.STYLES["chiptune"]
    part = BASS_GENERATORS["octave8"](chip, Key(0, "major"), [(0, Chord(0, (48, 52, 55)))], "pulse", (36, 52))
    steps = [round(e.beat / STEP) for e in part.pitches]
    assert steps == [2, 3, 6, 7, 10, 11, 14, 15] and not set(steps) & {0, 4, 8, 12}
    assert [e.midi for e in part.pitches] == [36, 48] * 4


# ── Выбор стиля кодом из фразы ─────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("text,style", [
    ("включи синтвейв", "synthwave"), ("ретровейв про ночной город", "synthwave"), ("аутран", "synthwave"),
    ("synthwave", "synthwave"), ("чиптюн на тему марио", "chiptune"), ("восьмибитный сет", "chiptune"),
    ("8-битный сет", "chiptune"), ("ты диджей 8битный", "chiptune"), ("16 bit музыка", "chiptune"),
    ("рейв про котов", "rave"), ("сет про космос", None), ("ретро сет", None),
])
def test_style_from_the_phrase(text, style):
    assert match_style_text(text) == style


def test_digit_and_bit_split_into_two_words_still_mark_the_style():
    """Грамматика роутера режет «8-битный» на «8» и «битный»: оба слова — стиль, ни одно — не тема (#3476)."""
    assert style_marks(["включи", "8", "битный", "сет"]) == [None, "chiptune", "chiptune", None]
    assert style_marks(["сыграй", "1812", "overture"]) == [None, None, None]
    assert match_style(["8", "бит"]) == "chiptune" and match_style(["8", "марта"]) is None


def test_theme_rows_give_the_style_when_none_is_named():
    """§4.2: киберпанк → synthwave, детский праздник → chiptune, тема без строки — клуб; слово стиля важнее строки."""
    assert style_for("киберпанк") == "synthwave" and style_for("детский праздник") == "chiptune"
    assert style_for("космос") == "club" and style_for("бухгалтерский отчёт") == "club"
    assert style_for("рейв", "киберпанк") == "rave" and style_for("", "детский праздник") == "chiptune"
    assert {row.style for row in kn.THEMES.values()} - {None} <= set(kn.STYLES)
    assert set(kn.STYLE_WORDS.values()) | set(kn.STYLE_PATTERNS.values()) <= set(kn.STYLES)


def test_every_style_has_a_word_and_club_word_beats_the_theme_row():
    """#3508: у каждого стиля есть слово фразы; «клубный сет на тему киберпанк» — club, хотя строка темы даёт synthwave."""
    assert set(kn.STYLE_WORDS.values()) == set(kn.STYLES)
    assert style_for("киберпанк") == "synthwave"
    for phrase in ("клубный сет на тему киберпанк", "клубняк про роботов", "club set", "включи клубную музыку"):
        assert style_for(phrase, "киберпанк") == "club", phrase
    assert style_for("восьмибитный сет", "киберпанк") == "chiptune" and style_for("8-бит клубный") == "chiptune"
    assert match_style_text("8-битный сет") == "chiptune"
    assert match_style_text("свежая клубника") is None  # «клубника» — не «клуб»
    assert match_style_text("сет в жанре deep house") is None  # окна клуба стиль не выбирают


# ── Сеты стиля ─────────────────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("name", NEW)
@pytest.mark.parametrize("theme", THEMES)
def test_set_tempo_kicks_and_synths_come_from_the_style_tables(name, theme):
    style = kn.STYLES[name]
    for seed in range(8):
        plan = _plan(theme, seed, name)
        window = style.genre_windows[plan.genre]
        assert plan.style == name and window.bpm[0] <= plan.bpm <= window.bpm[1], (seed, plan.genre, plan.bpm)
        assert all(t.kick in window.kick_pool for t in plan.tracks)
    for seed in range(2):
        plan = _plan(theme, seed, name)
        family = style.timbres[plan.family]
        history: list = []
        for no in (1, 2):
            track = compose(plan, no, history=history)
            validate(track)
            kick = track.parts["kick"]
            assert kick_name(kick.sample, kick.play_symbol, "" if kick.synth_or_sample == kn.PLAY_SYNTH else kick.synth_or_sample) in style.genre_windows[plan.genre].kick_pool
            for role in kn.TONAL_ROLES:
                assert track.parts[role].synth_or_sample in family[role], (role, track.parts[role].synth_or_sample)
            assert track.history_key.pad_figure in style.genre_windows[plan.genre].pad_figures
            assert track.style == name and track.key.mode in kn.SCALES
            if not style.duck_roles:
                assert track.mix.duck_roles == frozenset() and track.mix.duck == ()
            row = track_history(track, plan.set_id)
            assert row["style"] == name and track_composition(track)["style"] == name
            history.insert(0, row)


@pytest.mark.parametrize("name", NEW)
def test_one_style_one_tempo_per_set_neighbours_blend_and_render(name):
    for seed in range(1):
        plan = _plan("киберпанк", seed, name)
        tracks = [compose(plan, no, deck="AB"[no % 2]) for no in range(1, 4)]
        assert {t.style for t in tracks} == {name} and {t.bpm for t in tracks} == {plan.bpm}
        assert all(blend_bars(a, b) == kn.STYLES[name].blend[0] for a, b in zip(tracks[1:], tracks[2:]))
        for t in tracks:
            program = render(t, "A")
            _p, events = program_events(program.code, program.form_beats)
            synths = {e.synth for e in events}
            assert {t.parts[r].synth_or_sample for r in kn.TONAL_ROLES} <= synths, (t.track_id, synths)


def test_arp_pad_reaches_the_rendered_program():
    """Трек с рисунком ``arp``: пэд звучит каждой 16-й нотами аккорда, без сайдчейна."""
    for seed in range(40):
        plan = _plan("марио", seed, "chiptune")
        track = compose(plan, 2)
        if track.history_key.pad_figure != "arp":
            continue
        part = track.parts["pad"]
        chords = {c.voicing for cs in track.harmony.progression.values() for c in cs}
        tones = {m for v in chords for m in v}
        assert {e.midi for e in part.pitches} <= tones and all(e.dur_beats == STEP for e in part.pitches)
        assert "pad" not in track.mix.duck_roles
        return
    pytest.fail("ни один из 40 сидов не дал arp — пул рисунков chiptune не работает")


def test_validator_takes_the_style_from_the_track():
    synth = compose(_plan("космос", 2, "synthwave"), 2)
    chip = compose(_plan("космос", 2, "chiptune"), 2)
    with pytest.raises(TrackError, match="parts.kick.sample"):  # бочка chiptune не из пула synthwave
        validate(replace(synth, parts={**synth.parts, "kick": chip.parts["kick"]}))
    with pytest.raises(TrackError, match="mix.duck_roles"):  # у chiptune сайдчейна нет
        validate(replace(chip, mix=replace(chip.mix, duck_roles=frozenset({"bass"}), duck=synth.mix.duck)))


# ── A9-модель стиля на всех комбинациях ────────────────────────────────────────────────────────────────────────


def _combinations(name):
    style = kn.STYLES[name]
    figures = sorted({f for w in style.genre_windows.values() for f in w.pad_figures})
    bass_figures = sorted({f for w in style.genre_windows.values() for f in w.bass_figures})
    for family, figure in itertools.product(style.timbres, figures):
        for pad_synth in mix.pad_synths(style, family, figure):
            for bass_figure in bass_figures:
                for bass in mix.bass_synths(style, family, bass_figure):
                    yield name, family, figure, pad_synth, bass, bass_figure


@pytest.fixture(scope="module")
def base_tracks():
    """Форма открытия (трек 1) и обычная форма (трек 2) одного сида на стиль."""
    return {name: [compose(_plan("киберпанк", 0, name), no) for no in (1, 2)] for name in NEW}


@pytest.mark.parametrize("name,family,figure,pad_synth,bass,bass_figure",
                         [c for name in NEW for c in _combinations(name)])
def test_a9_model_holds_on_every_combination(base_tracks, name, family, figure, pad_synth, bass, bass_figure):
    """A9-модель стиля (порог ``a9_model_low``): семья × рисунок пэда × пэд × бас × рисунок баса × лид семьи."""
    style = kn.STYLES[name]
    for track, lead in itertools.product(base_tracks[name], style.timbres[family]["lead"]):
        spec = theme_form(form_spec(style, track.history_key.template), track.hook.theme_bars if track.hook else 0)
        chords = track.harmony.progression
        parts = {**track.parts, "lead": replace(track.parts["lead"], synth_or_sample=lead),
                 "pad": PAD_GENERATORS[figure](style, track.key, _bar_chords(spec, "pad", chords), pad_synth,
                                               track.parts["pad"].register),
                 "bass": BASS_GENERATORS[bass_figure](style, track.key, _bar_chords(spec, "bass", chords), bass,
                                                      style.registers["bass"])}
        _leveled, track_mix = mix.mix_parts(style, parts, track.form, figure)
        assert track_mix.a9_model >= style.a9_model_low, (family, figure, pad_synth, bass, bass_figure, lead,
                                                          track_mix.a9_model, dict(track_mix.a9_trim))
