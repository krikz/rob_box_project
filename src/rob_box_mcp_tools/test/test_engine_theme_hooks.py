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
from rob_box_music.arrange.compose import HOOK_REGISTER
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import ThemeProfile
from rob_box_music.tonality import key_fit

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
