"""#4029 — правка ИГРАЮЩЕГО трека: решение кодом (``rob_box_music.track_edit``), а не заказ нового трека.

Живой робот 27.10 (кейс Шифу): посреди рэпа под бит «сделай бит потяжелее, более металлический» шло в
``request_music(intent=track)`` → платная генерация KIE, мост ``synth:breaks``, рэп без тактов, «Включаю музыку.».
Марафон 27.10: n504 «продолжай развивать эту тему и добавь баса» и n506 «ускорь» — новый трек 60 BPM, n505
«сделай атмосферу помрачнее, как в киберпанке» — музыка не тронута. Здесь — слова → виды правки → решение → фраза.
"""

from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange.compose import compose
from rob_box_music.model import validate
from rob_box_music.render.renardo import FORM_START, at_form_start, render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.source import ask_tweak
from rob_box_music.theme import ThemeProfile
from rob_box_music.track_edit import (
    FILE,
    NONE,
    PIECE,
    SET,
    SYNTH,
    EditTarget,
    edit_route,
    edit_text,
    edits_in,
    new_order,
    next_family,
    plan_edit,
    route_edits,
    with_bass,
)

#: Бит выступления (подложка рэпа) — свой синт-трек: 136 BPM, тёплая семья клуба (живой лог 27.10 15:16:17).
BACKING = EditTarget(SYNTH, 136, "club", "warm", 0.0, backing=True)
TRACK = EditTarget(SYNTH, 128, "club", "warm")


@pytest.mark.parametrize(
    "text,kinds",
    [
        ("Робот, сделай бит потяжелее, более металлический!", ("heavier",)),  # кейс Шифу 27.10 посреди рэпа
        ("сделай бит более металлическим", ("heavier",)),
        ("Робот, сделай атмосферу помрачнее, как в киберпанке", ("darker",)),  # n505
        ("Робот, продолжай развивать эту тему и добавь баса", ("develop", "more_bass")),  # n504
        ("Робот, ускорь темп", ("faster",)),  # n506
        ("роботы ускорь там", ("faster",)),
        ("давай помедленнее", ("slower",)),
        ("сделай посветлее и помягче", ("brighter", "softer")),
        ("поменьше баса", ("less_bass",)),  # длинная запись побеждает «баса» (больше)
        ("убери басы", ("less_bass",)),
        ("больше баса", ("more_bass",)),
    ],
)
def test_edit_words_name_the_kind_of_edit(text, kinds):
    assert edits_in(text) == kinds
    assert ask_tweak(text)  # каждый вид правки — правка играющего и для наследования движка (одна таблица)


@pytest.mark.parametrize(
    "text",
    [
        "включи металл",
        "сыграй тяжёлый рок",
        "поставь медленную музыку",
        "не быстрее, а громче",
        "расскажи анекдот",
        "сделай погромче",
    ],
)
def test_genre_names_volume_and_negation_are_not_an_edit(text):
    """«Металл», «тяжёлый рок» — жанр (новый заказ), «громче» — громкость (Tier-1), «не быстрее» — отказ от правки."""
    assert edits_in(text) == ()


@pytest.mark.parametrize(
    "text,route",
    [
        ("Робот, сделай бит потяжелее, более металлический!", "edit:heavier"),
        ("Робот, сделай атмосферу помрачнее, как в киберпанке", "edit:darker"),
        ("Робот, продолжай развивать эту тему и добавь баса", "edit:develop+more_bass"),
        ("Робот, ускорь темп", "edit:faster"),
        ("Робот, поставь что-нибудь помрачнее", ""),  # заказ другого трека, а не правка играющего
        ("Робот, включи другую, побыстрее", ""),
        ("Робот, расскажи анекдот", ""),
    ],
)
def test_edit_route_under_playing_music(text, route):
    assert edit_route(text, playing=True) == route
    assert route_edits(route) == edits_in(text) if route else route_edits(route) == ()


def test_no_edit_route_in_silence():
    """В тишине править нечего: «ускорь» решает модель как раньше (форма хода не меняется)."""
    assert edit_route("Робот, ускорь темп", playing=False) == ""
    assert new_order("поставь что-нибудь") and not new_order("сделай побыстрее")


def test_shifu_case_heavier_backing_changes_timbre_and_bass_not_tempo():
    """«Потяжелее, металличнее» под рэп: семья ``hard`` и бас громче на том же темпе — сетка куплетов держится."""
    plan = plan_edit(edits_in("сделай бит потяжелее, более металлический!"), BACKING)
    assert plan.applied == ("heavier",) and plan.changes_sound and not plan.refused
    assert (plan.bpm, plan.timbre, plan.bass_db) == (136, "hard", 2.0)
    assert edit_text(plan) == "Сделал звук тяжелее."


def test_n505_darker_is_an_edit_of_the_playing_track():
    plan = plan_edit(edits_in("сделай атмосферу помрачнее, как в киберпанке"), TRACK)
    assert plan.applied == ("darker",) and plan.timbre == "dark" and plan.bpm == TRACK.bpm


def test_n504_develop_and_more_bass():
    plan = plan_edit(("develop", "more_bass"), TRACK)
    assert plan.applied == ("more_bass",) and plan.kept == ("develop",) and plan.bass_db == 3.0
    assert edit_text(plan) == "Добавил баса и играю дальше эту тему."


def test_faster_changes_tempo_by_one_step_and_says_the_new_tempo():
    plan = plan_edit(("faster",), TRACK)
    assert plan.bpm == round(128 * kn.EDIT_TEMPO_STEP) and plan.applied == ("faster",)
    assert edit_text(plan) == f"Ускорил до {plan.bpm:g} BPM."
    slower = plan_edit(("slower",), TRACK)
    assert slower.bpm == round(128 / kn.EDIT_TEMPO_STEP)


def test_tempo_at_the_limit_is_honest():
    top = replace(TRACK, bpm=kn.BPM_RANGE[1])
    plan = plan_edit(("faster",), top)
    assert not plan.changes_sound and plan.limited == ("faster",)
    assert edit_text(plan) == f"Быстрее уже некуда — {kn.BPM_RANGE[1]} BPM."


def test_tempo_of_a_performance_backing_is_refused_but_the_rest_applies():
    """Под выступление темп не меняется (сетка куплетов на тактах этого темпа) — честно, остальное делается."""
    plan = plan_edit(("faster", "heavier"), BACKING)
    assert plan.refused == "backing_tempo" and plan.applied == ("heavier",) and plan.bpm == 136
    assert edit_text(plan) == "Сделал звук тяжелее. " + kn.EDIT_REFUSALS["backing_tempo"]


def test_same_family_and_bass_at_the_limit_change_nothing():
    plan = plan_edit(("heavier",), replace(TRACK, timbre="hard", bass_db=kn.EDIT_BASS_DB_RANGE[1]))
    assert not plan.changes_sound and plan.limited == ("heavier",)
    assert edit_text(plan) == "Тяжелее этот бит уже не сделаю."


@pytest.mark.parametrize("kind", [FILE, PIECE, SET, NONE])
def test_not_our_synth_track_is_an_honest_refusal_with_an_offer(kind):
    """Кэш/генерация, пьеса, сет, тишина: звук не трогаем и не генерируем молча — отказ кодом с предложением."""
    plan = plan_edit(("faster",), EditTarget(kind))
    assert plan.refused == kind and not plan.changes_sound
    assert edit_text(plan) == kn.EDIT_REFUSALS[kind]
    if kind in (FILE, PIECE, SET):
        assert "громче или тише" in edit_text(plan)  # «Могу …» — что можно вместо


def test_nothing_named_asks_what_to_change():
    assert edit_text(plan_edit((), TRACK)) == kn.EDIT_REFUSALS["unknown"]


def test_one_table_of_edits():
    """``TRACK_EDITS`` — единственная таблица: ``ENGINE_TWEAK_WORDS`` её проекция по видам, у каждой правки есть
    фраза, семьи тембров есть у каждого стиля синтеза."""
    assert set(kn.ENGINE_TWEAK_WORDS) == set(kn.TRACK_EDIT_GROUPS)
    for group, words in kn.ENGINE_TWEAK_WORDS.items():
        assert words == tuple(w for e in kn.TRACK_EDITS.values() if e.group == group for w in e.words)
    for name, edit in kn.TRACK_EDITS.items():
        assert edit.group in kn.TRACK_EDIT_GROUPS and edit.done, name
        assert all(w == w.lower() and "ё" not in w for w in edit.words), name
        if edit.timbre and edit.timbre != kn.EDIT_NEXT_TIMBRE:
            assert all(edit.timbre in style.timbres for style in kn.STYLES.values()), name


def test_other_timbre_goes_round_the_families_of_the_style():
    families = tuple(kn.STYLES["club"].timbres)
    assert next_family("club", families[-1]) == families[0]
    assert plan_edit(("other_timbre",), TRACK).timbre == next_family("club", "warm")


PROFILE = ThemeProfile("тест", "club", 132, 9, "minor", (), None)


def test_more_bass_keeps_the_model_valid_and_stops_at_the_ceiling():
    track = compose(seeded_plan(PROFILE, 7, set_id="req1"), 3)
    louder = with_bass(track, 3.0)
    validate(louder)  # уровень партии = уровень микса, не выше потолков роли и суммы секции
    assert louder.parts["bass"].level_db >= track.parts["bass"].level_db
    assert louder.mix.level_db["bass"] == louder.parts["bass"].level_db
    loudest = with_bass(track, 60.0)
    validate(loudest)
    assert loudest.parts["bass"].level_db <= kn.role_ceiling("bass")
    assert with_bass(track, -3.0).parts["bass"].level_db == pytest.approx(track.parts["bass"].level_db - 3.0)


def test_edited_program_counts_sections_from_the_start_of_the_playing_form():
    """Правка на ходу не начинает форму заново: секции ``var`` — от доли старта играющего трека."""
    code = render(compose(seeded_plan(PROFILE, 7, set_id="req1"), 3), "A").code
    assert FORM_START in code
    edited = at_form_start(code, 1236.0)
    assert FORM_START not in edited and "start=1236)" in edited
