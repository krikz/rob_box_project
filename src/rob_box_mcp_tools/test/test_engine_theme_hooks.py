"""#3409 (ADR-0149 §3.3, I12): мелодии тем из настоящего архива RTTTL становятся хуком трека.

Живой прогон 05.10, сет 6 треков, сид 7: «инспектор гаджет» и «денди» — 0/6 треков с мелодией темы, «терминатор» —
2/6: ``hook.from_rtttl`` отвергал тему за любой полутон вне лада и за диапазон шире коридора лида. Тест проверяет
исход компоновки (хук, ``key_fit``, регистр, источник хука в треках сета), а не текст промпта.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.session import plan_source
from rob_box_mcp_tools.engine.tools_v2 import library_melodies
from rob_box_music import knowledge as kn
from rob_box_music.arrange import hook as hooks
from rob_box_music.arrange.compose import hook_register
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import ThemeProfile
from rob_box_music.tonality import key_fit

HOOK_REGISTER = hook_register(kn.STYLES["club"])

pytestmark = pytest.mark.unit

THEMES = {
    "инспектор гаджет": ("inspecto", "theme_88", "theme_89"),
    "денди": ("zelda", "contra", "supermar_4"),
    "терминатор": ("terminat", "theme_177", "theme_178"),
}


@pytest.fixture(scope="module")
def melodies(tmp_path_factory):
    lib = RtttlLibrary(db_path=str(tmp_path_factory.mktemp("hooks") / "voice_memory.db"))
    ids = [i for ids in THEMES.values() for i in ids]
    found = library_melodies(lambda: lib)(ids)
    assert sorted(found) == sorted(ids), "мелодии тем есть в архиве"
    return found


@pytest.mark.parametrize("melody_id", [i for ids in THEMES.values() for i in ids])
def test_theme_melody_builds_a_hook_on_every_root(melodies, melody_id):
    """Хроматика темы — проходящие (key_fit ≥ 0.6 к ладу трека), регистр — перенос октавой в коридор, не отказ."""
    for root in range(12):
        hook, key = hooks.from_rtttl(melodies[melody_id], melody_id, 128, root, "minor", HOOK_REGISTER)
        notes = [(e.midi, e.dur_beats) for e in hook.notes]
        assert hook.source == melody_id and hook.key_fit >= kn.HOOK_KEY_FIT_MIN
        assert hook.key_fit == key_fit(notes, kn.ROOTS[key.root], key.mode)
        assert all(HOOK_REGISTER[0] <= m <= HOOK_REGISTER[1] for m, _d in notes)


@pytest.mark.parametrize("theme", sorted(THEMES))
@pytest.mark.parametrize("seed", [7, 1])
def test_set_on_a_theme_plays_its_melodies(melodies, theme, seed):
    """Приёмка #3409: в сете из 6 треков мелодия темы звучит хуком хотя бы в 4; каждый трек проходит валидатор."""
    ids = THEMES[theme]
    plan = seeded_plan(ThemeProfile(theme, "club", 128, 9, "minor", ids, "test"), seed, n_tracks=6)
    next_track = plan_source(lambda: (plan, {i: melodies[i] for i in ids}))
    tracks = [next_track(no, "AB"[no % 2]) for no in range(1, 7)]
    for track in tracks:
        render(track, "A")  # render валидирует трек (I12: key_fit лида)
    sources = [t.hook.source if t.hook else None for t in tracks]
    assert sum(s in ids for s in sources) >= 4, sources


def test_terminator_theme_from_real_library_cannot_be_displaced_by_table_row(tmp_path):
    """#3418: хуки «терминатора» из настоящей библиотеки; ответ LLM row=cyber hooks=robotroc/robot отвергается."""
    from rob_box_mcp_tools.engine.search import theme_hooks
    from rob_box_music import reasoner as rz
    from rob_box_music.theme import seeded_profile

    lib = RtttlLibrary(db_path=str(tmp_path / "voice_memory.db"))
    found = theme_hooks(lib, "терминатора")
    assert "terminat" in found
    profile = seeded_profile("терминатора", found=found)
    assert set(rz.schema(profile=profile)["properties"]["hooks"]["items"]["enum"]) == set(profile.theme_hooks)
    assert not {"robot", "robotroc"} & set(profile.theme_hooks)
    bad = {"theme_row": "cyber", "mode": profile.mode, "hooks": ["terminat", "robotroc", "robot"], "energy": [3]}
    with pytest.raises(rz.PlanInvalid) as err:
        rz.validate(bad, profile=profile)
    assert err.value.path == "hooks"
    good = rz.validate({**bad, "hooks": ["terminat"]}, profile=profile)
    assert good.row == "cyber" and good.hook_ids == ("terminat",)


# ── #3427: «консенсус версий» — узнаваемая тема записана в архиве несколько раз ─────────────────────────────────


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("consensus") / "voice_memory.db"))


def _rtttl(notes: str) -> str:
    return f"x:d=8,o=5,b=120:{notes}"


def test_consensus_order_puts_versions_of_one_tune_before_a_lone_record():
    """Контур без транспозиции и длительностей: две версии одной темы (другая тональность и ритм) — выше одиночной;
    повторная версия — после первых версий других тем; доля слов запроса — главнее."""
    from rob_box_mcp_tools.engine.search import consensus_order

    hits = [
        (1.0, {"name": "lone", "rtttl": _rtttl("g#6,a#6,b6,g#,g#,g#,g#,g#")}),
        (1.0, {"name": "v1", "rtttl": _rtttl("d,e,f,e,c,f4,c6,4c6")}),
        (1.0, {"name": "v2", "rtttl": _rtttl("4e,p,16f#,4g,f#,d,g4,d6,a")}),  # v1 на тон выше, другой ритм, пауза
        (1.0, {"name": "w1", "rtttl": _rtttl("c,c,c,g,g,g,a,a")}),
        (1.0, {"name": "w2", "rtttl": _rtttl("16f,16f,16f,c6,c6,c6,d6,d6")}),
        (0.5, {"name": "half", "rtttl": _rtttl("d,e,f,e,c,f4,c6,c6")}),
        (1.0, {"name": "broken", "rtttl": "not rtttl"}),
    ]
    assert consensus_order(hits) == ["v1", "w1", "v2", "w2", "lone", "broken", "half"]
    assert consensus_order(hits, limit=3) == ["v1", "v2", "lone"]  # набор — первые limit, меняется порядок


def test_terminator_hook_one_is_the_canonical_theme(library):
    """Живой сет 05.10: первым был terminat (g#6 a#6 b6 …) — не узнаётся; канон — theme_177/theme_178 (d e f e c f)."""
    from rob_box_mcp_tools.engine.search import theme_hooks

    found = theme_hooks(library, "терминатора")
    assert found[0] in {"theme_177", "theme_178"}
    assert set(found) == {"terminat", "theme_177", "theme_178"} and found[-1] == "terminat"


@pytest.mark.parametrize("theme,first", [
    ("инспектор гаджет", {"inspecto", "inspecto_2", "inspecto_3", "theme_88", "theme_89"}),
    ("пираты", {"pirateso"}),
    ("денди", {"zelda", "contra"}),
])
def test_other_themes_keep_a_recognisable_first_hook(library, theme, first):
    from rob_box_mcp_tools.engine.search import theme_hooks

    assert theme_hooks(library, theme)[0] in first


@pytest.mark.parametrize("theme,expected", [
    # живой тест 05.10 ~23:40: точная запись giveinto была седьмой среди «Give Me A Reason/Sign/The Light»
    ("Give In To Me", ("giveinto",)),
    # запись называется ``king`` (title «kingcastle»): точное совпадение по title
    ("kingcastle", ("king",)),
    # хвост «remix2, remix_3, …» по одному общему слову отсечён
    ("the colin remix", ("thecolin",)),
    # четыре версии одного названия, без starwars/x-files из строки ``space`` («star»)
    ("Twinkle Twinkle Little Star", ("twinklet_5", "twinklet", "twinklet_2", "twinklet_3")),
])
def test_exact_title_comes_first_and_cuts_one_word_matches(library, theme, expected):
    from rob_box_mcp_tools.engine.search import theme_search
    from rob_box_music.theme import seeded_profile

    hits = theme_search(library, theme)
    assert hits.exact and hits.names == expected
    profile = seeded_profile(theme, found=hits.names, exact=hits.exact)
    assert profile.row is None and profile.hook_ids == expected  # строка таблицы тем не добавляет хуков


def test_concept_theme_without_exact_title_keeps_the_table_row(library):
    """«космос» — понятие, а не название: точного совпадения нет, строка ``space`` таблицы тем по-прежнему действует."""
    from rob_box_mcp_tools.engine.search import theme_search
    from rob_box_music.theme import seeded_profile

    hits = theme_search(library, "космос")
    assert not hits.exact and hits.names
    assert seeded_profile("космос", found=hits.names, exact=hits.exact).row == "space"


@pytest.mark.parametrize("seed", range(5))
def test_first_track_of_terminator_set_plays_the_canonical_theme(library, seed):
    """Seeded-план из настоящей библиотеки: трек 1 — хук №1 (канон), не мелодия по сиду."""
    from rob_box_mcp_tools.engine.search import theme_hooks
    from rob_box_music.arrange.compose import compose
    from rob_box_music.theme import seeded_profile

    found = theme_hooks(library, "терминатора")
    plan = seeded_plan(seeded_profile("терминатора", found=found), seed)
    track = compose(plan, 1, melodies=library_melodies(lambda: library)(found))
    assert track.hook is not None and track.hook.source == found[0]
