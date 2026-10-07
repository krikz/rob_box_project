"""ADR-0153 S1: стиль ``rave`` — запись ``knowledge.STYLES``, один стиль на сет, ключ стиля в треке.

Поведение — на ``SetPlan``/``Track``/программе Renardo: темп в окне рейва, бочки и синты из пулов рейва, A9-модель
рейва на всех комбинациях тембров, валидатор берёт стиль из трека. Клуб побайтно тот же — ``test_style_same_tracks``.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music import reasoner as rz
from rob_box_music.arrange import mix
from rob_box_music.arrange.compose import BASS_GENERATORS, PAD_GENERATORS, _bar_chords, compose, form_spec
from rob_box_music.diversity import kick_name, track_composition, track_history
from rob_box_music.model import TrackError, blend_bars, validate
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import match_style, seeded_profile

RAVE = kn.STYLES["rave"]
THEMES = ("космос", "киберпанк", "детский праздник", "калинка", "бухгалтерский отчёт")
#: Пулы бочек всех окон рейва.
RAVE_KICKS = {k for w in RAVE.genre_windows.values() for k in w.kick_pool}


def _plan(theme: str, seed: int, style: str = "rave", **kw):
    return seeded_plan(seeded_profile(theme, style), seed, set_id=f"r{seed}", **kw)


def _family(style: kn.Style, theme: str):
    return style.timbres[kn.THEME_TIMBRE.get(seeded_profile(theme).row or "", style.default_timbre)]


def test_rave_windows_span_140_to_160_and_kicks_are_hard_and_flagged_unmeasured():
    """Окна rave/acid/hardcore покрывают 140–160; бочки — из таблицы, своих файлов, не из клуба; без замера на
    роботе — помечены (``measured=False``), низ по файлу ≥ 0.8."""
    assert list(RAVE.genre_windows) == ["rave", "acid", "hardcore"]
    assert kn.genre_style(RAVE, "rave") == RAVE, "поля стиля — первое окно"
    bpms = [w.bpm for w in RAVE.genre_windows.values()]
    assert min(lo for lo, _ in bpms) == 140 and max(hi for _, hi in bpms) == 160
    club_kicks = {k for w in kn.STYLES["club"].genre_windows.values() for k in w.kick_pool}
    assert RAVE_KICKS <= set(kn.KICK_SOUNDS) and not RAVE_KICKS & club_kicks
    for name in RAVE_KICKS:
        kick = kn.KICK_SOUNDS[name]
        assert kick.symbol in ("A", "W") and not kick.measured and math.isnan(kick.sub) and kick.low >= 0.8, name
    assert set(kn.GENRE_NAMES) == {w for st in kn.STYLES.values() for w in st.genre_windows}  # S2: и окна новых стилей
    assert RAVE.genre_windows.keys() <= kn.GENRE_NAMES


def test_rave_timbres_lead_with_hoover_rave_supersaw_and_every_family_has_acid_tb303():
    leads = {s for fam in RAVE.timbres.values() for s in fam["lead"]}
    assert {"hoover", "rave", "supersawlead"} <= leads and leads <= set(kn.SYNTH_PALETTE["lead"])
    paired = {s for synths in kn.BASS_FIGURE_SYNTHS.values() for s in synths}
    for name, fam in RAVE.timbres.items():
        assert "tb303" in fam["bass"] and "acid16" in mix.bass_figures(RAVE, kn.family_of(RAVE, None)), name
        assert all(kn.bass_low_on_robot(s) >= kn.BASS_MIN_LOW for s in fam["bass"] if s not in paired), name
        assert all(kn.LAYER_BANDS["lead"][s][0] < 0.05 for s in fam["lead"]), name
        assert all(mix.layer_db(kn.LANE_DB_AT_UNIT["lead"][s], kn.AMP_EXPONENT.get(s, 1.0), kn.MAX_LAYER_AMP)
                   >= RAVE.role_level_db["lead"] for s in fam["lead"]), name


@pytest.mark.parametrize("theme", THEMES)
def test_rave_set_tempo_kicks_and_synths_come_from_the_rave_tables(theme):
    for seed in range(12):
        plan = _plan(theme, seed)
        window = RAVE.genre_windows[plan.genre]
        assert plan.style == "rave" and plan.genre in RAVE.genre_windows
        assert window.bpm[0] <= plan.bpm <= window.bpm[1] and 140 <= plan.bpm <= 160, (seed, plan.genre, plan.bpm)
        assert all(t.kick in window.kick_pool for t in plan.tracks)
    for seed in range(3):
        plan = _plan(theme, seed)
        family = RAVE.timbres[plan.family]  # тема вне таблицы — семья по сиду (#3460)
        history: list = []
        for no in (1, 2, 3):
            track = compose(plan, no, history=history)
            validate(track)
            kick = track.parts["kick"]
            assert kick_name(kick.sample, kick.play_symbol) in RAVE.genre_windows[plan.genre].kick_pool
            assert track.parts["lead"].synth_or_sample in family["lead"]
            assert track.parts["bass"].synth_or_sample in family["bass"]
            assert track.parts["pad"].synth_or_sample in family["pad"]
            assert track.key.mode in kn.SCALES and track.style == "rave"
            row = track_history(track, plan.set_id)
            assert row["style"] == "rave" and track_composition(track)["style"] == "rave"
            assert row["kick"] in RAVE_KICKS
            history.insert(0, row)


def test_rave_kick_renders_with_its_own_symbol_and_buffer():
    plan = _plan("космос", 3)
    track = compose(plan, 2)
    kick = kn.KICK_SOUNDS[plan.track(2).kick]
    program = render(track, "A")
    assert f'play("{kick.symbol}' in program.code and f"sample={kick.sample}" in program.code
    assert f"{kick.symbol}:{kick.sample}" in program.samples


def test_one_style_one_tempo_per_set_and_neighbours_blend():
    for seed in range(4):
        plan = _plan("киберпанк", seed)
        tracks = [compose(plan, no, deck="AB"[no % 2]) for no in range(1, 5)]
        assert {t.style for t in tracks} == {"rave"} and {t.bpm for t in tracks} == {plan.bpm}
        assert {t.history_key.genre for t in tracks} == {plan.genre}
        assert all(blend_bars(a, b) == RAVE.blend[0] for a, b in zip(tracks[1:], tracks[2:]))


def test_theme_without_style_words_is_club_and_style_words_pick_rave():
    assert seeded_profile("космос").style == "club" and _plan("космос", 1, "club").style == "club"
    assert match_style("сет про космос".split()) is None
    for words in ("рейв про котов", "эйсид", "в стиле хардкор", "РЭЙВОВЫЙ сет", "acid house"):
        assert match_style(words.split()) == "rave", words


def test_row_mode_outside_the_style_falls_back_to_a_style_mode():
    """«Детский праздник» — мажор строки тем: у клуба он есть, у рейва (минор/фригийский) — лад стиля по хешу."""
    assert seeded_profile("детский праздник").mode == "major"
    assert seeded_profile("детский праздник", "rave").mode in RAVE.modes


def test_validator_takes_the_style_from_the_track():
    rave = compose(_plan("космос", 2), 2)
    club = compose(_plan("космос", 2, "club"), 2)
    house = replace(rave.parts["kick"], symbol="X", sample=kn.KICK_SOUNDS["house"].sample)
    with pytest.raises(TrackError, match="parts.kick.sample"):  # клубная бочка в рейв-треке — не из пула стиля
        validate(replace(rave, parts={**rave.parts, "kick": house}))
    with pytest.raises(TrackError, match="parts.kick.sample"):  # и наоборот
        validate(replace(club, parts={**club.parts, "kick": rave.parts["kick"]}))
    with pytest.raises(TrackError, match="history_key.style"):
        validate(replace(rave, history_key=replace(rave.history_key, style="jazz")))


def test_reasoner_cannot_switch_the_style_inside_a_set():
    """LLM-ризонер стиль не выбирает (один стиль на сет, ADR-0153 §4.3): поля ``style`` в схеме нет, ответ с ним —
    ``PlanInvalid("style")``; лады схемы — лады стиля сета."""
    plan = _plan("космос", 1)
    schema = rz.schema(plan.style, profile=plan.profile)
    assert "style" not in schema["properties"] and schema["properties"]["mode"]["enum"] == list(RAVE.modes)
    payload = {"theme_row": "space", "mode": "minor", "hooks": [plan.profile.hook_ids[0]], "energy": [3],
               "style": "club"}
    with pytest.raises(rz.PlanInvalid) as err:
        rz.validate(payload, plan.style, profile=plan.profile)
    assert err.value.path == "style"


def _combinations():
    for family, figure in itertools.product(RAVE.timbres, ("pumped16", "held", "stabs")):
        pads = [s for s in RAVE.timbres[family]["pad"] if kn.PAD_FIGURES[figure].long_tails or mix.sustains_to_sus(s)]
        for pad, bass, bass_figure in itertools.product(pads, RAVE.timbres[family]["bass"], kn.BASS_FIGURES):
            if bass_figure != "broken" and mix.bass_pair_ok(bass_figure, bass):
                yield family, figure, pad, bass, bass_figure


@pytest.fixture(scope="module")
def base_tracks():
    """Треки-основа рейва: форма открытия и обычная форма, два сида."""
    return [compose(_plan("киберпанк", seed), no) for seed, no in itertools.product(range(2), (1, 2))]


@pytest.mark.parametrize("family,figure,pad,bass,bass_figure", list(_combinations()))
def test_rave_a9_model_holds_on_every_combination(base_tracks, family, figure, pad, bass, bass_figure):
    """A9-модель рейва (порог ``a9_model_low``): семья × рисунок пэда × пэд × бас × рисунок баса × лид семьи."""
    for track, lead in itertools.product(base_tracks, RAVE.timbres[family]["lead"]):
        spec = form_spec(RAVE, track.history_key.template)
        chords = track.harmony.progression[track.form.sections[0].name]
        parts = {**track.parts, "lead": replace(track.parts["lead"], synth_or_sample=lead),
                 "pad": PAD_GENERATORS[figure](RAVE, track.key, _bar_chords(spec, "pad", chords), pad,
                                               track.parts["pad"].register),
                 "bass": BASS_GENERATORS[bass_figure](RAVE, track.key, _bar_chords(spec, "bass", chords), bass,
                                                      RAVE.registers["bass"])}
        _leveled, track_mix = mix.mix_parts(RAVE, parts, track.form, figure)
        assert track_mix.a9_model >= RAVE.a9_model_low, (family, figure, pad, bass, bass_figure, lead,
                                                         track_mix.a9_model, dict(track_mix.a9_trim))
