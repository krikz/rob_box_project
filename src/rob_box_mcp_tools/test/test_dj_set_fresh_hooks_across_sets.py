"""#3399: одна фраза темы несколько сетов подряд — первый хук и набор хуков меняются (живой факт 07.10).

Товарищ Шифу несколько раз подряд просил «вечеринка любителей кино…» — в каждом сете играл popcorn или axelf: пул
без находок брался по хешу текста (одна фраза — одни семь хуков), а первый трек сета брал хук №1 профиля без
штрафа недавних. Тест идёт путём ``DjSetTool._start`` (поиск по настоящему архиву → профиль → план с
``SetMemory.peek`` → ``plan_source`` с памятью) и проверяет исход: хуки треков трёх сетов подряд.

Порог пересечения соседних сетов — не больше одной мелодии из четырёх скомпонованных (три сыграны + N+1 собран
заранее, как в живом сете, остановленном на третьем треке): у темы ≥ 12 мелодий свежих хватает на три сета,
одна общая допускается, когда свежие кончились и очередь берёт давнюю. Мелодия — ключ ``rtttl.contour``: версии
(popcorn/popcorn_6) — одна мелодия.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.search import theme_search
from rob_box_mcp_tools.engine.session import SetMemory, plan_source
from rob_box_mcp_tools.engine.tools_v2 import library_melodies
from rob_box_music.rtttl import CONTOUR_NOTES, contour
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

pytestmark = pytest.mark.unit

#: Сиды трёх сетов подряд (в живом сете — ``int(time.time())``).
SEEDS = ((1759800001, 1759800333, 1759800777), (11, 12, 13), (4242, 99, 1234567))
THEMES = {
    "космос": "theme",  # строка таблицы тем + находки поиска
    "кино, фильмов": "theme",  # жанр каталога movie (#3492)
    "бабушкин огород": "pool",  # ничего не найдено — пул
}
PLAYED = 4  # три трека сыграно, четвёртый скомпонован заранее


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("fresh") / "voice_memory.db"))


def _sets(library, theme, seeds, memory=None):
    """Хуки треков сетов подряд на одну фразу — как ``DjSetTool._start``: профиль, план, ``plan_source``."""
    memory = memory or SetMemory()
    melodies = library_melodies(lambda: library)
    out = []
    for seed in seeds:
        hits = theme_search(library, theme)
        profile = seeded_profile(theme, found=hits.names, exact=hits.exact, parts=hits.parts)
        plan = seeded_plan(profile, seed, n_tracks=10, set_id=f"set{seed % 100000:05d}", history=memory.peek())
        tunes = melodies(profile.hook_ids)
        next_track = plan_source(lambda: (plan, tunes), memory)
        hooks = [next_track(no, "AB"[no % 2]).hook for no in range(1, PLAYED + 1)]
        out.append((profile, [h.source if h else None for h in hooks], tunes))
    return out


def _tune(name, tunes):
    return contour(tunes[name], CONTOUR_NOTES) or name


@pytest.mark.parametrize("seeds", SEEDS)
@pytest.mark.parametrize("theme", sorted(THEMES))
def test_same_phrase_three_sets_open_differently_and_share_little(library, theme, seeds):
    sets = _sets(library, theme, seeds)
    assert {profile.source for profile, _h, _t in sets} == {THEMES[theme]}
    openers = [hooks[0] for _p, hooks, _t in sets]
    assert None not in openers and len(set(openers)) == 3, openers
    opening_tunes = [_tune(hooks[0], tunes) for _p, hooks, tunes in sets]
    assert len(set(opening_tunes)) == 3, openers  # не другая версия той же мелодии
    for (_p, a, tunes), (_q, b, _u) in zip(sets, sets[1:]):
        shared = {_tune(h, tunes) for h in a} & {_tune(h, tunes) for h in b}
        assert len(shared) <= 1, (a, b)


@pytest.mark.parametrize("theme", sorted(THEMES))
def test_first_set_without_history_opens_with_hook_number_one(library, theme):
    """Один сет без истории — как раньше: хук №1 профиля (#3427: узнаваемая версия темы первой)."""
    (profile, hooks, _t), = _sets(library, theme, SEEDS[0][:1])
    assert hooks[0] == profile.hook_ids[0]


def test_one_tune_theme_keeps_the_canonical_versions_on_the_first_track(library):
    """«терминатора»: одна узнаваемая мелодия (theme_177/theme_178) и запись terminat. Сет не открывается тем же
    хуком, что прошлый, но и не уходит в нераспознаваемую запись — открывает другая версия канона."""
    sets = _sets(library, "терминатора", (1, 2, 3, 4))
    openers = [hooks[0] for _p, hooks, _t in sets]
    assert set(openers) <= {"theme_177", "theme_178"}
    assert all(a != b for a, b in zip(openers, openers[1:])), openers


def test_pool_is_not_seven_hooks_by_the_phrase_hash(library):
    """Пул без находок — весь ``HOOK_POOL``; три сета подряд берут девять и больше разных мелодий из него."""
    sets = _sets(library, "бабушкин огород", SEEDS[1])
    names = {h for _p, hooks, _t in sets for h in hooks[:3]}
    assert len(names) >= 9, names
