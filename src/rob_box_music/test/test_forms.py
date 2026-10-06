"""ADR-0152 §3.5 (PR-7): формы трека ``Style.forms`` — club48 / short32 / long64 / dropfirst48.

Форма — шаблон данных стиля; план выбирает её по энергии трека и истории (``set_plan.pick_template``). Блэнд —
свойство пары форм (``model.blend_bars``): тест гоняет все пары ``forms × forms``. Имена секций новых форм покрыты
таблицами развития хука, дуги громкости, LPF и слоёв сэмплов — без ветвления по форме в ``arrange``.
"""

from __future__ import annotations

import itertools
import random
from collections import Counter
from dataclasses import replace

import pytest

from melodies import MELODIES, profile, with_template
from rob_box_music import knowledge as kn
from rob_box_music.arrange import hook as hooks
from rob_box_music.arrange.compose import compose, form_spec
from rob_box_music.diversity import MusicHistory, track_composition, track_history
from rob_box_music.model import BLEND_BARS, OUTRO_MIN_BARS, TrackError, blend_bars, validate
from rob_box_music.set_plan import pick_template, plan_templates, seeded_plan

STYLE = kn.STYLES[kn.DEFAULT_STYLE]
NAMES = tuple(STYLE.forms)
BARS = {"club48": 48, "short32": 32, "long64": 64, "dropfirst48": 48}


def _tracks(seed: int = 4):
    """По одному треку каждой формы: тот же план, сид и тема, меняется только шаблон."""
    plan = seeded_plan(profile(hooks=("long",)), seed)
    return {name: compose(with_template(plan, 2, name), 2, melodies=MELODIES, deck="AB"[i % 2])
            for i, name in enumerate(NAMES)}


def test_forms_table_has_four_shapes_with_the_adr_lengths():
    assert set(NAMES) == set(BARS)
    for name, spec in STYLE.forms.items():
        assert sum(bars for _n, bars, _e, _r in spec) == BARS[name], name
    assert STYLE.opening_form == "dropfirst48"


def test_every_form_is_valid_and_ends_with_an_outro_without_lead():
    for name, track in _tracks().items():
        validate(track)
        assert track.history_key.template == name
        assert track.form.bars_total == BARS[name]
        outro = [s for s in track.form.sections if s.name.startswith("outro")]
        assert sum(s.bars for s in outro) >= OUTRO_MIN_BARS and all("lead" not in s.roles for s in outro), name


def test_validator_rejects_an_unknown_template_and_a_short_outro():
    track = _tracks()["club48"]
    with pytest.raises(TrackError):
        validate(replace(track, history_key=replace(track.history_key, template="zzz")))
    short = replace(track.form.sections[-2], bars=4)
    form = replace(track.form, sections=track.form.sections[:-2] + (short,) + track.form.sections[-1:-1])
    with pytest.raises(TrackError) as err:
        validate(replace(track, form=form))
    assert "outro" in str(err.value)


@pytest.mark.parametrize("leaving,incoming", list(itertools.product(NAMES, NAMES)))
def test_every_pair_of_forms_blends(leaving, incoming):
    """``blend_bars > 0`` для всех пар форм: одна бочка и один бас на такт блэнда, в хвосте нет лида."""
    tracks = _tracks()
    assert blend_bars(tracks[leaving], tracks[incoming]) == STYLE.blend[0]
    assert BLEND_BARS[0] <= STYLE.blend[0] <= BLEND_BARS[1]


def test_section_names_of_all_forms_are_covered_by_the_tables():
    names = {n for spec in STYLE.forms.values() for n, *_ in spec}
    assert names <= set(kn.SECTION_TRIM_DB)
    assert names <= set(hooks.DEVELOPMENT) | {"intro", "intro_low", "outro", "outro_tail"}
    assert all(set(secs) <= names for secs in STYLE.layer_sections.values())
    assert set(STYLE.section_lpf) <= names
    for name in ("build2", "break2"):
        twin = name[:-1]
        assert kn.SECTION_TRIM_DB[name] == kn.SECTION_TRIM_DB[twin]
        assert STYLE.section_lpf[name] == STYLE.section_lpf[twin]


def test_long_form_plays_both_builds_and_a_short_second_break():
    track = _tracks()["long64"]
    names = [s.name for s in track.form.sections]
    assert names == ["intro", "intro_low", "build", "drop", "break", "build2", "drop2", "break2", "outro",
                     "outro_tail"]
    assert "lead" in track.form.sections[names.index("break2")].roles
    assert "sample" in track.form.sections[names.index("build2")].roles  # psr-слой на втором подъёме


def test_energy_templates_name_known_forms_for_every_track_energy():
    assert set(STYLE.energy_forms) == set(range(1, 6))
    assert all(forms and set(forms) <= set(NAMES) for forms in STYLE.energy_forms.values())
    assert {f for forms in STYLE.energy_forms.values() for f in forms} == set(NAMES)


def test_thirty_seeds_give_at_least_three_forms_and_never_the_same_form_twice_in_a_row():
    used = Counter()
    for seed in range(30):
        plan = seeded_plan(profile(), seed, n_tracks=10)
        forms = [t.template for t in plan.tracks]
        assert forms[0] == STYLE.opening_form
        assert all(a != b for a, b in zip(forms, forms[1:])), (seed, forms)
        assert all(f in STYLE.energy_forms[t.energy] for t, f in zip(plan.tracks[1:], forms[1:]))
        used.update(forms)
    assert len(used) >= 3 and used["long64"] and used["short32"], used


def test_history_penalises_the_recent_form_and_never_repeats_the_last_one():
    for seed in range(40):
        rows = [{"template": "club48"}, {"template": "short32"}]
        assert pick_template(STYLE, 3, rows, random.Random(seed)) != "club48"
    plans = plan_templates(STYLE, 1, "x", [2, 3, 4, 5, 4], history=[{"template": "dropfirst48"}])
    assert plans[0] == "dropfirst48" and len(plans) == 5


def test_composition_and_history_carry_the_template():
    history = MusicHistory(":memory:")
    track = _tracks()["long64"]
    assert track_composition(track)["template"] == "long64"
    assert history.record(**track_history(track, "s"))
    assert history.recent(1)[0]["template"] == "long64"
    assert form_spec(STYLE, "long64") is STYLE.forms["long64"]
