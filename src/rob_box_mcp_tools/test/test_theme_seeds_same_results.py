"""#3493 PR-B: ручные таблицы ``knowledge.THEME_CONCEPTS`` и ``knowledge.RU_ALIASES`` удалены, их пары — семена реестра
(``rob_box_music/data/theme_link_seeds.json``, ``works.ThemeLink`` с ``source="seed"``). Фразы, которые работали
благодаря таблицам, находят то же самое: ожидания сняты на ``origin/develop`` @851d97144 тем же кодом поиска
(``theme_search``, ``find``, ``theme.match_row``) до удаления таблиц. Настоящий архив."""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.search import find, theme_search
from rob_box_music.theme import match_row

pytestmark = pytest.mark.unit

#: (тема, первые хуки, всего хуков, строка THEMES, запись ``find``) — как на develop до удаления таблиц.
BEFORE = [
    ("интерстеллар", ("lostinsp_2", "lost", "lostinsp", "startrek_3", "spaceque", "spacelor"), 8, "space",
     "startrek_3"),
    ("денди и классика", ("zelda", "rondoala", "contra", "mozart_2", "supermar_4", "jesujoyo"), 30, None, "zelda"),
    ("кино", ("superman_6", "theme_83", "harrypot_3", "harrypot_4", "thegodfa_3", "terminat"), 30, None, None),
    ("путешествие хогвардсу", ("harrypot_6", "harrypot_2", "harrypot_3", "harrypot_5", "harrypot_4", "harrypot"), 8,
     None, "harrypot_6"),
    ("калинка", ("kalinkav_2", "kalinkav"), 2, "slavic", "kalinkav_2"),
    ("чайковский", ("tchaikov", "swanlake_2"), 2, None, "tchaikov"),
    ("гимн ссср", ("unknown_111", "soviethy", "national_2", "irishnat", "walesnat", "indonesi"), 8, None,
     "unknown_111"),
    ("звёздные войны", ("starwars_4", "starwars_8", "starwars_3", "starwars_7", "starwars_5", "starwars_6"), 8,
     "space", "starwars_4"),
]


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("seeds") / "voice_memory.db"))


@pytest.mark.parametrize("theme, first, total, row, found", BEFORE, ids=[b[0] for b in BEFORE])
def test_phrase_finds_the_same_as_with_the_old_tables(library, theme, first, total, row, found):
    names = theme_search(library, theme).names
    assert names[:len(first)] == first and len(names) == total
    assert match_row(theme) == row
    assert (find(library, theme).record or {}).get("name") == found


def test_ru_title_for_voice_still_comes_from_seeds(library):
    assert library.ru_alias_for("tetris") == "тетрис" and library.ru_alias_for("furelise") == "к элизе"
