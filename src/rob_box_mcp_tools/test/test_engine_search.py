"""#3399 (ADR-0149 §3.3, I16): ``engine.search.find`` по словам человека — на настоящем архиве RTTTL.

Живой прогон 05.10: сет «вечеринка по темам терминатора» играл axelf/popcorn/tetris — ``get('терминатора')`` → None,
``search('вечеринка по темам терминатора')`` → мусор по «по/темам». Тест проверяет исход поиска (запись, доля
покрытых слов, ``found``), а не текст промпта.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.classic import find_record
from rob_box_mcp_tools.engine.reasoner import SetPlanBox
from rob_box_mcp_tools.engine.search import find, sound_key, stem, theme_hooks
from rob_box_mcp_tools.engine.tools_v2 import library_melodies
from rob_box_music.arrange.compose import compose
from rob_box_music.diversity import track_history
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

pytestmark = pytest.mark.unit


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("search") / "voice_memory.db"))


def _names(found):
    return {found.record["name"], *(r["name"] for r in found.alternatives)}


@pytest.mark.parametrize("text", ["вечеринка по темам терминатора", "давай тему терминатора", "терминатора",
                                  "Терминатор"])
def test_terminator_by_case_form_and_party_words(library, text):
    found = find(library, text)
    assert found.found and found.confidence == 1.0
    assert found.record["name"] == "terminat"
    assert {"theme_177", "theme_178"} <= _names(found)


@pytest.mark.parametrize("text", ["тему инспектор гаджет", "инспектор гаджет", "Inspector Gadget"])
def test_inspector_gadget_by_transliteration(library, text):
    found = find(library, text)
    assert found.found and found.confidence == 1.0
    assert all("inspector gadget" in f"{r['title']} {r['artist']}".lower()
               for r in (found.record, *found.alternatives))


@pytest.mark.parametrize("text,expected", [
    ("пираты", {"pirateso", "theme_141"}),
    ("космос", {"spacecha", "spaceque"}),
    ("денди", {"zelda", "contra", "supermar_4"}),
    ("калинку", {"kalinkav_2"}),
])
def test_concepts_and_plural_find_meaningful_melodies(library, text, expected):
    found = find(library, text)
    assert found.found and expected & _names(found), (text, _names(found) if found.record else None)


@pytest.mark.parametrize("text", ["Angine de Poitrine", "привет как дела", "бухгалтерский отчёт", "новый год",
                                  "охотники за привидениями", "вечеринка по темам", "ты диджей", ""])
def test_garbage_and_unknown_are_honest_misses(library, text):
    found = find(library, text)
    assert not found.found and found.record is None and found.alternatives == ()


def test_known_queries_keep_the_record_get_finds(library):
    """Запрос, который ``RtttlLibrary.get`` уже покрывает, даёт ту же запись (A15: не хуже v1)."""
    for text in ("super mario", "russian anthem", "гимн ссср", "имперский марш", "калинка", "tetris"):
        found = find(library, text)
        assert found.found and found.query == text and found.record["name"] == library.get(text)["name"], text


def test_classic_order_by_case_form_plays_the_melody(library):
    record, reason = find_record(library, "терминатора")
    assert record is not None and record["name"] == "terminat", reason
    assert find_record(library, "гимн германии")[0] is None  # «германии» не та запись — честный промах


def test_stem_and_sound_key():
    assert stem("терминатора") == "терминатор" and stem("темам") == "тем" and stem("tetris") == "tetris"
    assert sound_key("gadzhet") == sound_key("gadget") and sound_key("inspektor") == sound_key("inspector")


#: Темы живого прогона 05.10 и приёмки #3399 → мелодии, которые обязаны попасть в хуки темы.
SET_THEMES = {
    "вечеринка по темам терминатора": {"terminat", "theme_177", "theme_178"},
    "тему инспектор гаджет": {"inspecto", "inspecto_2", "inspecto_3"},
    "денди": {"zelda", "contra", "supermar_4"},
    "пираты": {"pirateso", "theme_141"},
    "космос": {"spacecha", "spaceque"},
}


@pytest.fixture(scope="module")
def profiles(library):
    return {t: seeded_profile(t, found=theme_hooks(library, t)) for t in SET_THEMES}


def test_set_theme_words_reach_the_hooks(profiles):
    for text, expected in SET_THEMES.items():
        prof = profiles[text]
        assert prof.source == "theme" and expected & set(prof.theme_hooks), (text, prof.hook_ids)


def test_five_themes_share_at_most_one_hook(profiles):
    """Живой прогон 05.10: на любые темы играли одни axelf_3/popcorn/robot/tetris."""
    themes = list(profiles)
    for i, a in enumerate(themes):
        for b in themes[i + 1:]:
            assert len(set(profiles[a].hook_ids) & set(profiles[b].hook_ids)) <= 1, (a, b)


@pytest.mark.parametrize("text", ["вечеринка по темам терминатора", "пираты"])
def test_theme_set_plays_hooks_found_by_theme_words_and_a11_counts_only_them(library, profiles, text):
    """Звук: треки сета строятся на мелодиях темы, и лог ``started`` пишет ``source=theme`` ровно у них.

    Не каждая мелодия темы ложится в хук (``hook.from_rtttl``: хроматика вне лада, диапазон шире коридора на
    тонике трека) — такие треки играют мотив, и A11 их не засчитывает."""
    prof = profiles[text]
    melodies = library_melodies(lambda: library)(prof.hook_ids)
    plan = seeded_plan(prof, 7, set_id="t")
    box = SetPlanBox(plan, lambda ids: melodies)
    history, themed, notes = [], 0, []
    for no in range(1, 7):
        track = box.compose_mark(compose(plan, no, melodies=melodies, history=history))
        history.insert(0, track_history(track))
        themed += track.hook is not None and track.hook.source in prof.theme_hooks
        notes.append(box.on_started(track.track_id))
    assert themed >= 2 and notes[-1].endswith(f"A11={themed}/6"), notes


def test_pool_theme_never_counts_as_theme(library):
    pool = seeded_profile("бухгалтерский отчёт")
    plan = seeded_plan(pool, 7, set_id="p")
    box = SetPlanBox(plan, library_melodies(lambda: library))
    track = box.compose_mark(compose(plan, 1, melodies=box.current()[1]))
    assert track.hook is not None and "source=pool plan=seeded A11=0/1" in box.on_started(track.track_id)


def test_garbage_theme_has_no_theme_hooks(library):
    for text in ("Angine de Poitrine", "привет как дела", "вечеринка по темам"):
        assert theme_hooks(library, text) == (), text
