"""#3546: тема-композитор («Моцарт») — RTTTL-эталоны разных пьес не отсеивают партитуры.

Карточка (робот 08.10): «лоуфай сет на тему Моцарт» — план отсеял все 8 партитур по эталону мотива
(совпадение ≤ 0.64 < 0.8): тема = композитор, эталоны — чужие пьесы автора. Эталон мотива только у
произведения; у композитора — первое проведение (``motif_references`` → пусто).

Материалы — синтетика (не живой пак): 8 разных мелодий + чужой RTTTL-контур, как 8/8 отказов на Моцарте.
"""

from __future__ import annotations

from dataclasses import replace

import pytest
from test_harmony_material import synthetic
from test_theme_coverage import REGISTER, _tune

from rob_box_music import knowledge as kn
from rob_box_music.arrange import compose as cp
from rob_box_music.arrange import hook as hooks
from rob_box_music.model import PitchEvent, validate
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import ThemeProfile

#: Чужой эталон (арпеджио C-мажора) — ни одна «гамма»-партитура ниже его не наберёт 0.8.
FOREIGN = "mozart:d=4,o=5,b=120:c,e,g,c6,e6,g6,c7,g6,e6,c6,g,e"


def _scales(mid: str, shift: int = 0):
    """Партитура без мотива FOREIGN: гаммы четвертями со сдвигом ``shift`` (8 разных пьес автора)."""
    m = synthetic((0, 0, 5, 5, 3, 3, 4, 4))
    steps = tuple((i + shift) % 12 for i in (0, 2, 4, 5, 7, 9, 11, 12, 11, 9, 7, 5, 4, 2, 0, 2))
    melody = tuple(PitchEvent(60 + steps[i % len(steps)], float(i), 1.0, 2) for i in range(32))
    return replace(m, melody=melody, material_id=mid, title=mid, composer="W. A. Mozart")


def _mozart_profile(materials):
    return ThemeProfile(
        "моцарт", kn.DEFAULT_STYLE, 132, 0, "major", ("mozart",), None, ("mozart",), materials=tuple(materials)
    )


@pytest.mark.parametrize(
    "theme, apply",
    [
        ("моцарт", False),
        ("Моцарт", False),
        ("mozart", False),
        ("моцарта", False),
        ("бах и моцарт", False),
        ("Bach and Mozart", False),
        ("токката баха", True),
        ("горный король", True),
        ("eine kleine nachtmusik", True),
        ("", True),
    ],
)
def test_motif_references_empty_only_for_composer_theme(theme, apply):
    refs = ("a:d=4,o=5,b=120:c", "b:d=4,o=5,b=120:d")
    assert hooks.composer_theme(theme) is (not apply)
    assert hooks.motif_references(theme, refs) == (refs if apply else ())


def test_foreign_refs_would_reject_all_eight_mozart_scores():
    """Контроль карточки: чужой эталон при пороге сета отсеивает все 8 (как на роботе ≤ 0.64)."""
    mats = {_scales(f"local:moz{i}", i).material_id: _scales(f"local:moz{i}", i) for i in range(8)}
    reasons = {mid: hooks.material_unfit(m, 132, 0, "major", REGISTER, [FOREIGN]) for mid, m in mats.items()}
    assert len(reasons) == 8 and all(r and "главного мотива" in r for r in reasons.values()), reasons


def test_composer_theme_plan_keeps_scores_foreign_refs_would_reject():
    """Тема «моцарт»: эталон неприменим — 8 партитур не выбывают, трек 1 получает материал."""
    mats = {_scales(f"local:moz{i}", i).material_id: _scales(f"local:moz{i}", i) for i in range(8)}
    profile = _mozart_profile(mats)
    rejected: dict = {}
    plan = seeded_plan(profile, 7, 8, materials=mats, rejected=rejected, references=[FOREIGN])
    assert not any("главного мотива" in why for why in rejected.values()), rejected
    assert plan.track(1).material in mats
    track = cp.compose(plan, 1, melodies={"mozart": FOREIGN}, materials=mats)
    validate(track)
    assert track.hook.source == plan.track(1).material


def test_work_theme_still_rejects_material_without_main_motif():
    """Тема-произведение: эталон применяется — материал без мотива уступает трек (как #3542)."""
    first = _scales("local:scales", 1)
    second = replace(synthetic((0, 0, 5, 5, 3, 3, 4, 4)), material_id="local:arps")
    ref = _tune("ref", [e.midi for e in second.melody[:13]], dur=4, bpm=120)
    materials = {first.material_id: first, second.material_id: second}
    profile = ThemeProfile(
        "горный король",
        kn.DEFAULT_STYLE,
        132,
        0,
        "major",
        ("ref",),
        None,
        ("ref",),
        materials=(first.material_id, second.material_id),
    )
    rejected: dict = {}
    plan = seeded_plan(profile, 7, 2, materials=materials, rejected=rejected, references=[ref])
    assert plan.track(1).material == second.material_id
    assert "главного мотива" in rejected[first.material_id]
