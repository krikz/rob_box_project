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
    assert style.opening_form in style.forms and set(style.energy_forms) == set(range(1, 6))
    assert all(set(allowed) <= set(style.forms) for allowed in style.energy_forms.values())
    assert all(roles <= set(kn.ROLES) for spec in style.forms.values() for _n, _b, _e, roles in spec)
    names = {n for spec in style.forms.values() for n, *_ in spec}
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


def test_no_style_family_role_picks_a_synth_above_its_nyquist_limit():
    """#3502: ``NYQUIST_MAX_MIDI`` применён одним местом (``mix.role_palette``) во всех стилях и семьях: ни лид, ни бас,
    ни пэд не берут синт, чей фильтр уходит за 8 кГц в коридоре роли; ``cs80lead`` из клуба/рейва выпал по этой таблице."""
    import random
    from rob_box_music.arrange import mix
    for name, style in kn.STYLES.items():
        for family, roles in style.timbres.items():
            for role in roles:
                top = style.registers[role][1]
                palette = mix.role_palette(style, family, role)
                assert palette, (name, family, role)
                assert all(kn.NYQUIST_MAX_MIDI.get(s, 128) >= top for s in palette), (name, family, role, palette)
                if role in ("lead", "bass"):
                    for seed in range(20):
                        picked = mix.role_timbre(style, family, role, (), random.Random(seed))
                        assert picked in palette, (name, family, role, picked)
    # сам гард жив: синт с пределом ниже коридора роли отсекается, даже если его вернут в таблицу
    import dataclasses
    club = kn.STYLES["club"]
    bad = dataclasses.replace(club, timbres={"hard": {**club.timbres["hard"], "lead": ("hoover", "cs80lead")}})
    assert mix.role_palette(bad, "hard", "lead") == ("hoover",)
