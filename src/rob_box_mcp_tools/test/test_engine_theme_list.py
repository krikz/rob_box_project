"""Тема-перечисление → хуки из разных франшиз по кругу, сет обходит их без повторов (живой прогон 06.10).

Шифу просил «мегасет … Марио, Аладдин, Тетрис, Чёрный Плащ, Контра, Dendy и другие»: ни одна мелодия не покрывала
половины слов темы (``search.FOUND_MIN``), поиск вернул пусто, сет взял пул по хешу, LLM выбрала три хука — и 50+
треков крутились по кругу из ``tetris_2, spacecha, aroundth_3``, Марио не прозвучал. Тест проверяет исход поиска по
настоящему архиву и хуки треков сета, а не текст промпта.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.search import THEME_LIST_HOOKS, round_robin, theme_parts, theme_search
from rob_box_mcp_tools.engine.session import plan_source
from rob_box_mcp_tools.engine.tools_v2 import library_melodies
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

pytestmark = pytest.mark.unit

#: Темы живого прогона 06.10 (вторая — как в логе, многоточие Шифу опущено) и фраза живой проверки.
MEGASET = ("мегасет для игроков из RTTTL-мелодий разных игр — Марио, Аладдин, Тетрис, Чёрный Плащ, Контра, Dendy "
           "и другие, чтобы не повторяться")
CHIPTUNE = "8-бит Dendy чиптюн пати с классикой — Dendy, Contra, Mario, Чайковского, Вивальди, Баха"
LIVE = "Марио, Аладдин, Тетрис, Контра, Зельда"


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("theme_list") / "voice_memory.db"))


def _franchise(library, theme: str):
    """``хук -> часть темы``, по которой он нашёлся первым (франшиза для проверки «разные подряд»)."""
    owner = {}
    for part in theme_parts(library, theme):
        for name in theme_search(library, part).names:
            owner.setdefault(name, part)
    return owner


def test_list_theme_splits_into_franchises_and_drops_frame_words(library):
    """Обрамление («мегасет для игроков из RTTTL-мелодий разных игр», «и другие», «чтобы не повторяться») — не часть;
    «8-бит» — одно слово (дефис без пробелов)."""
    assert theme_parts(library, MEGASET) == ["Марио", "Аладдин", "Тетрис", "Чёрный Плащ", "Контра", "Dendy"]
    assert theme_parts(library, CHIPTUNE) == ["8-бит Dendy чиптюн пати с классикой", "Dendy", "Contra", "Mario",
                                              "Чайковского", "Вивальди", "Баха"]
    assert theme_parts(library, LIVE) == ["Марио", "Аладдин", "Тетрис", "Контра", "Зельда"]
    assert theme_parts(library, "терминатора") == ["терминатора"]


def test_round_robin_takes_one_per_part_and_skips_repeats():
    assert round_robin([("m1", "m2", "m3"), ("a1",), ("m1", "t1", "t2")], 10) == ["m1", "a1", "t1", "m2", "t2", "m3"]
    assert round_robin([("m1", "m2"), ("a1", "a2")], 3) == ["m1", "a1", "m2"]
    assert round_robin([], 5) == []


def test_megaset_theme_finds_mario_first_and_five_franchises_in_first_five(library):
    """Приёмка 06.10: первые 5 хуков — разные франшизы, Марио первым; «Чёрный Плащ» — честное «не найдено»."""
    hits = theme_search(library, MEGASET)
    owner = _franchise(library, MEGASET)
    assert hits.names[0].startswith("supermar") and not hits.exact
    assert len({owner[h] for h in hits.names[:5]}) == 5
    assert hits.missing == ("Чёрный Плащ",)
    assert len(hits.names) == len(set(hits.names)) > 8  # не потолок одной темы: хватит на мегасет


def test_chiptune_theme_keeps_found_parts_and_reports_the_rest(library):
    hits = theme_search(library, CHIPTUNE)
    assert {"contra", "supermar_4", "zelda"} <= set(hits.names[:5])
    assert "Чайковского" in hits.missing
    profile = seeded_profile(CHIPTUNE, found=hits.names, exact=hits.exact)
    assert profile.source == "theme" and profile.hook_ids[:len(hits.names)] == hits.names


def test_live_list_fills_the_list_ceiling_without_repeats(library):
    """Пять франшиз по 8 мелодий: 30 хуков (``THEME_LIST_HOOKS``) — по кругу, без повторов."""
    hits = theme_search(library, LIVE)
    owner = _franchise(library, LIVE)
    assert len(hits.names) == len(set(hits.names)) == THEME_LIST_HOOKS
    assert [owner[h] for h in hits.names[:5]] == ["Марио", "Аладдин", "Тетрис", "Контра", "Зельда"]


def test_whole_theme_and_exact_title_still_win(library):
    """Тема без перечисления и точное название — как раньше (#3427): не делятся."""
    assert theme_search(library, "Give In To Me").names == ("giveinto",)
    assert theme_search(library, "терминатора").missing == ()


def test_set_on_a_list_theme_walks_the_found_hooks_without_repeats(library):
    """12 треков сета по теме-перечислению: хуки не повторяются, первые 5 — из ≥ 4 франшиз, Марио среди них."""
    hits = theme_search(library, LIVE)
    owner = _franchise(library, LIVE)
    plan = seeded_plan(seeded_profile(LIVE, found=hits.names), 7, n_tracks=12)
    melodies = library_melodies(lambda: library)(plan.profile.hook_ids)
    next_track = plan_source(lambda: (plan, melodies))
    played = [next_track(no, "AB"[no % 2]).hook for no in range(1, 13)]
    sources = [h.source for h in played if h is not None]
    assert len(sources) == 12 and len(set(sources)) == 12
    assert len({owner[h] for h in sources[:5]}) >= 4 and "Марио" in {owner[h] for h in sources[:5]}
