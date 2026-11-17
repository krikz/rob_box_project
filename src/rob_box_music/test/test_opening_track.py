"""#3427 (ADR-0152 §3.5 ``dropfirst48``): первый трек сета — хук №1 темы и тема раньше.

Живой сет «терминатора» 05.10 (set26040): трек 1 взял нераспознаваемую запись ``terminat`` (порядок сида), а
тема целиком звучала только с дропа на такте 16 (~28 с); узнаваемую тему Шифу услышал на 1:38 — в дропе трека 2.
Тест проверяет исход компоновки: форму трека 1, где звучит лид, какой хук взят, блэнд всех пар форм.
"""

from __future__ import annotations

import random

import pytest

from melodies import MELODIES, profile, with_template
from rob_box_music import knowledge as kn
from rob_box_music.arrange.compose import compose, hook_candidates
from rob_box_music.model import BEATS_PER_BAR, blend_bars, validate
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

STYLE = kn.STYLES[kn.DEFAULT_STYLE]
HOOKS = ("long", "short", "slow")


def _start_bar(track, name):
    start = 0
    for sec in track.form.sections:
        if sec.name == name:
            return start
        start += sec.bars
    raise KeyError(name)


@pytest.mark.parametrize("seed", range(6))
def test_first_track_drops_with_the_hook_after_the_intro(seed):
    """Трек 1: дроп с хуком — сразу после интро (такт 8, не 16); интро и аутро — как у остальных (блэнд)."""
    plan = seeded_plan(profile(hooks=HOOKS), seed)
    first = compose(plan, 1, melodies=MELODIES)
    second = compose(with_template(plan, 2, "club48"), 2, melodies=MELODIES)  # любая форма — с дропом после build
    for track in (first, second):
        validate(track)
    blend = STYLE.blend[0]
    assert _start_bar(first, "drop") == blend < _start_bar(second, "drop")
    assert [s.name for s in first.form.sections][:2] == [s.name for s in second.form.sections][:2]
    assert [s.name for s in first.form.sections][-2:] == [s.name for s in second.form.sections][-2:]
    assert first.form.bars_total == second.form.bars_total
    lead_beats = [e.beat for e in first.parts["lead"].pitches]
    assert min(lead_beats) == blend * BEATS_PER_BAR  # тема входит первой долей дропа, а не фильтром build


@pytest.mark.parametrize("seed", range(8))
def test_first_track_takes_theme_hook_number_one(seed):
    """Хук трека 1 — первый в профиле темы при любом сиде; дальше треки идут порядком сида (I17)."""
    plan = seeded_plan(profile(hooks=HOOKS), seed)
    assert compose(plan, 1, melodies=MELODIES).hook.source == HOOKS[0]
    order = [h.source for h, _k in hook_candidates(plan.profile, MELODIES, random.Random(seed), opening=True)]
    assert order == list(HOOKS)


@pytest.mark.parametrize("theme", ("космос", "детский праздник", ""))
def test_every_pair_of_forms_blends(theme):
    """Блэнд — свойство пары форм: opening→club, club→club, а также club→opening и opening→opening."""
    plan = seeded_plan(seeded_profile(theme), 3, set_id="o")
    plan = with_template(with_template(plan, 2, "club48"), 3, "club48")
    opening, club, club2 = (compose(plan, no, deck="AB"[no % 2]) for no in (1, 2, 3))
    for leaving, incoming in ((opening, club), (club, club2), (club, opening), (opening, opening)):
        assert blend_bars(leaving, incoming) == STYLE.blend[0]
