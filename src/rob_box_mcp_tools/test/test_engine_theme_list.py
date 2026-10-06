"""Тема-перечисление → хуки из разных франшиз по кругу, сет обходит их без повторов (живой прогон 06.10).

Шифу просил «мегасет … Марио, Аладдин, Тетрис, Чёрный Плащ, Контра, Dendy и другие»: ни одна мелодия не покрывала
половины слов темы (``search.FOUND_MIN``), поиск вернул пусто, сет взял пул по хешу, LLM выбрала три хука — и 50+
треков крутились по кругу из ``tetris_2, spacecha, aroundth_3``, Марио не прозвучал. Тест проверяет исход поиска по
настоящему архиву и хуки треков сета, а не текст промпта.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.search import THEME_LIST_HOOKS, round_robin, terms, theme_parts, theme_search
from rob_box_mcp_tools.engine.session import plan_source
from rob_box_mcp_tools.engine.tools_v2 import library_melodies
from rob_box_music.arrange.compose import compose
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
    assert "tchaikov" in hits.names and hits.missing == ()  # «Чайковского» — через THEME_CONCEPTS (06.10)
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


# ---------------------------------------------------------------------------
# Живой сет 06.10 ~14:05: описание стиля не ищется, цифра — целым словом, «Darkwing Duck» — не «Ducktoy»
# ---------------------------------------------------------------------------

RETRO = "ретро 8-бит: Mario, Tetris, Aladdin, Contra, Darkwing Duck, Commando, Frogger"


def test_style_prefix_is_not_a_part_and_digit_is_a_whole_word(library):
    """«ретро 8-бит» — стиль сета, не мелодия: «8» искало «1812 Overture», «8 Days Of Christmas», «Sk8er Boi»;
    «Darkwing Duck» по слову «duck» давало «Ducktoy» (Hampenberg) и «Nice Weather For Ducks» — подмена, а не находка."""
    assert theme_parts(library, RETRO) == ["Mario", "Tetris", "Aladdin", "Contra", "Darkwing Duck", "Commando",
                                           "Frogger"]
    assert terms(library, "ретро 8-бит") == [] and terms(library, "8 бит чиптюн для геймеров") == []
    hits = theme_search(library, RETRO)
    noise = {"1812over", "1812over_2", "8daysofc", "8daysofc_2", "sk8erboi", "sk8rboi_2", "8mile", "niceweat",
             "ducktoy", "ducktoyv", "racke_du"}
    assert not noise & set(hits.names)
    assert hits.missing == ("Darkwing Duck",)
    owner = _franchise(library, RETRO)
    assert [owner[h] for h in hits.names[:4]] == ["Mario", "Tetris", "Aladdin", "Contra"]
    assert [owner[p[0]] for p in hits.parts] == ["Mario", "Tetris", "Aladdin", "Contra", "Commando", "Frogger"]
    assert sorted(h for p in hits.parts for h in p) == sorted(hits.names)


def test_composers_by_russian_name(library):
    """«Чайковского» не находилось (транслит ≠ «Tchaikovsky»), «Баха» находило Baha Men."""
    assert set(theme_search(library, "Чайковского").names) >= {"tchaikov", "swanlake_2"}
    bach = theme_search(library, "Баха").names
    assert {"brandenb", "j_s_bach"} <= set(bach) and not any(n.startswith("wholet") for n in bach)


# ---------------------------------------------------------------------------
# Живой сет 06.10 ~13:57: «сет на 2 трека: Марио, Тетрис» → Тетрис и снова Тетрис
# ---------------------------------------------------------------------------

def _tracks(library, theme: str, history, n: int):
    hits = theme_search(library, theme)
    profile = seeded_profile(theme, found=hits.names, exact=hits.exact, parts=hits.parts)
    plan = seeded_plan(profile, 7, n_tracks=n)
    melodies = library_melodies(lambda: library)(plan.profile.hook_ids)
    rows = list(history)
    out = []
    for no in range(1, n + 1):
        hook = compose(plan, no, melodies=melodies, history=rows).hook
        out.append(hook.source)
        rows.insert(0, {"melody_name": hook.source})
    return out


def test_named_franchises_alternate_even_if_played_before(library):
    """Версии Марио и «tetris» сыграны в прошлых сетах (свежие первыми): трек 1 — Марио (первым назван, наименее
    недавняя версия), трек 2 — Тетрис, а не вторая версия Тетриса."""
    hits = theme_search(library, "Марио, Тетрис")
    assert hits.names[0].startswith("supermar")  # «Tetris» по теме целиком (полслова) не встаёт первым
    history = [{"melody_name": "tetris"}, {"melody_name": "supermar_4"}, {"melody_name": "supermar"}]
    first, second = _tracks(library, "Марио, Тетрис", history, 2)
    mario, tetris = hits.parts
    assert first in mario and first not in {"supermar", "supermar_4"}
    assert second in tetris and second != "tetris"


def test_parts_rotate_before_repeating_a_franchise(library):
    """Шесть треков по «Марио, Тетрис, Контра»: части по кругу, версия внутри части не повторяется."""
    played = _tracks(library, "Марио, Тетрис, Контра", [], 6)
    hits = theme_search(library, "Марио, Тетрис, Контра")
    part_of = {h: i for i, p in enumerate(hits.parts) for h in p}
    assert [part_of[h] for h in played] == [0, 1, 2, 0, 1, 2]
    assert len(set(played)) == 6


# ---------------------------------------------------------------------------
# Живой лог 06.10 после #3476: STT отдал тему БЕЗ знаков — перечисление не распознано, хуки из пула
# ---------------------------------------------------------------------------

NO_PUNCT = "ретро 8-бит Mario Tetris Aladdin Contra"


def test_theme_without_separators_is_split_by_catalog(library):
    """Части по каталогу: «ретро 8-бит» — стиль, четыре названия — четыре части, хуки по частям, source=theme."""
    assert theme_parts(library, NO_PUNCT) == ["mario", "tetris", "aladdin", "contra"]
    hits = theme_search(library, NO_PUNCT)
    owner = _franchise(library, NO_PUNCT)
    assert hits.names and hits.missing == ()
    assert [owner[h] for h in hits.names[:4]] == ["mario", "tetris", "aladdin", "contra"]
    assert len(hits.parts) == 4
    assert seeded_profile(NO_PUNCT, found=hits.names, parts=hits.parts).source == "theme"


def test_mixed_latin_and_cyrillic_without_separators(library):
    assert theme_parts(library, "Марио Tetris Аладдин Contra") == ["марио", "tetris", "аладдин", "contra"]
    assert theme_search(library, "Марио Tetris Аладдин Contra").missing == ()


def test_two_cyrillic_words_still_two_franchises(library):
    hits = theme_search(library, "Марио Тетрис")
    assert len(hits.parts) == 2 and hits.names[0].startswith("supermar")


def test_punctuated_theme_is_unchanged(library):
    assert theme_parts(library, "ретро 8-бит: Mario, Tetris, Aladdin, Contra") == ["Mario", "Tetris", "Aladdin",
                                                                                  "Contra"]


def test_single_franchise_stays_one_theme(library):
    assert theme_parts(library, "ретро 8-бит Mario") == ["ретро 8-бит Mario"]
    assert len(theme_search(library, "ретро 8-бит Mario").parts) == 0


def test_garbage_theme_stays_pool_honestly(library):
    """Слова без записи в архиве: частей нет, хуков нет — сет честно берёт пул."""
    garbage = "ретро qzxwv plorbf"
    assert len(theme_parts(library, garbage)) < 2
    hits = theme_search(library, garbage)
    assert hits.names == () and hits.parts == ()
    assert seeded_profile(garbage, found=hits.names).source != "theme"
