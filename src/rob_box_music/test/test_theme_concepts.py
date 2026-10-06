"""Issue #3455 — тема без мелодий в названии относится к строке таблицы через понятие; пул без праздничных хуков.

Живой прогон 06.10: «включи диджей сет на тему интерстеллар» → ``source=pool`` с ``jinglebe_6``, ``macarena``,
``happybir``. Знание — в ``knowledge.THEME_CONCEPTS`` и теге ``ThemeRow.pooled`` (данные, не ветки).
"""

from __future__ import annotations

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.theme import HOOK_POOL, match_row, seeded_profile


@pytest.mark.parametrize("theme", ["интерстеллар", "Interstellar", "межзвёздный перелёт", "на тему интерстеллар"])
def test_interstellar_is_space_row(theme):
    prof = seeded_profile(theme)
    assert match_row(theme) == "space" and prof.row == "space"
    assert set(prof.hook_ids) == set(kn.THEMES["space"].hooks) and prof.source == "theme"


def test_found_hooks_still_come_first():
    prof = seeded_profile("интерстеллар", found=("spaceq",))
    assert prof.hook_ids[0] == "spaceq" and set(prof.hook_ids[1:]) == set(kn.THEMES["space"].hooks)


def test_occasion_hooks_stay_out_of_the_pool():
    occasion = {h for row in kn.THEMES.values() if not row.pooled for h in row.hooks}
    assert {"happybir", "macarena", "jinglebe_6"} <= occasion
    assert not occasion & set(HOOK_POOL)
    for theme in ("котики", "завод", "бухгалтерский отчёт", "лес", "Angine de Poitrine"):
        assert not occasion & set(seeded_profile(theme).hook_ids), theme


@pytest.mark.parametrize("theme,row", [("день рождения", "kids"), ("новый год", "winter")])
def test_occasion_theme_keeps_its_hooks(theme, row):
    assert set(seeded_profile(theme).hook_ids) == set(kn.THEMES[row].hooks)
