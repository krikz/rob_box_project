"""ADR-0153 S0: стиль — запись ``knowledge.STYLES``, генераторы ``arrange`` — функции от неё, без ветвления по стилю."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange import compose

ARRANGE = Path(compose.__file__).resolve().parent
#: Ветвление по стилю/жанру и ключ стиля строкой: стиль приходит параметром, выбор — по ключу из таблицы стиля.
BRANCH = re.compile(r"\bif\s+(not\s+)?(style|genre)\b|\b(style|genre)\w*(\.\w+)*\s*[!=]=|[!=]=\s*(style|genre)\b"
                    r"|[\"'](club|jazz|rock|rave|synthwave|chiptune|dnb|breaks|lofi)[\"']")


@pytest.mark.parametrize("path", sorted(ARRANGE.glob("*.py")), ids=lambda p: p.name)
def test_arrange_has_no_style_branches(path):
    lines = path.read_text(encoding="utf-8").splitlines()
    hits = [f"{path.name}:{i}: {line.strip()}" for i, line in enumerate(lines, 1) if BRANCH.search(line)]
    assert not hits, hits


def test_lint_catches_a_style_branch():
    for bad in ('if style == "jazz":', "if genre == x:", 'kick = mix.kick_sound("club")', "if style.chord_size != 3:"):
        assert BRANCH.search(bad), bad


@pytest.mark.parametrize("name", sorted(kn.STYLES))
def test_style_figures_have_generators(name):
    style = kn.STYLES[name]
    assert style.bass_figures and set(style.bass_figures) <= set(compose.BASS_GENERATORS)
    assert style.pad_figures and set(style.pad_figures) <= set(compose.PAD_GENERATORS)
    assert style.lead_figures and set(style.lead_figures) <= set(compose.LEAD_GENERATORS)


@pytest.mark.parametrize("name", sorted(kn.STYLES))
def test_style_record_is_consistent(name):
    """Окно темпа в пределах тракта, лады известны, бочка из каталога, семья по умолчанию есть, роли — из ``ROLES``."""
    style = kn.STYLES[name]
    assert kn.BPM_RANGE[0] <= style.bpm[0] <= style.bpm[1] <= kn.BPM_RANGE[1]
    assert set(style.modes) <= set(kn.SCALES)
    assert set(style.kick_pool) <= set(kn.KICK_SOUNDS) and len(style.kick_pool) >= 2
    assert style.default_timbre in style.timbres and set(kn.THEME_TIMBRE.values()) <= set(style.timbres)
    assert all(set(fam) == set(kn.TONAL_ROLES) for fam in style.timbres.values())
    assert all(roles <= set(kn.ROLES) for _n, _b, _e, roles in style.form)
    names = {n for n, *_ in style.form}
    assert all(set(secs) <= names for secs in style.layer_sections.values())
    assert set(style.duck_roles) <= set(kn.DUCK_ROLES) and set(style.registers) == set(kn.TONAL_ROLES)
    phrase, swap = style.blend
    assert 0 < swap < phrase


def test_club_is_the_default_style():
    assert kn.DEFAULT_STYLE == "club" and kn.REGISTERS == kn.STYLES["club"].registers
    from rob_box_music.set_plan import seeded_plan
    from rob_box_music.theme import seeded_profile
    plan = seeded_plan(seeded_profile("космос"), 1)
    assert plan.style == plan.profile.style == "club"
