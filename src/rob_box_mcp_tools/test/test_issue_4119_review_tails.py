"""Хвосты ревью PR #4119 (сет из генерации, #4009/#4012): три места, где код решал неверно.

(а) запас сопоставлял каверы с произведениями темы строкой ``work_id in works`` без нормализации репо (``works.norm``) -
«Stranger Things: Intro Theme» из кэша и «Stranger Things Intro Theme» из темы считались разными произведениями;
(б) запас своего слота с одним близнецом только что сыгранной задачи перебивал запас другого слота с треком чужой задачи -
близнец вставал сразу после своей пары (§1.4: подряд - вариация одного трека);
(в) ``turn_sources`` вращал и RTTTL хуков: на круге из трёх партитур и хука четвёртая задача начинала с хука, то есть с
запасного источника.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_music.source import CacheTrack, CoverWork

pytestmark = pytest.mark.unit

ST = CoverWork("Stranger Things Intro Theme", "pdmx:QmbSEvE33N", spoken="Stranger Things Intro Theme")


# ── (а) произведение кавера сравнивается через norm ──────────────────────────────────────────────────────────────
@pytest.mark.parametrize("work_id", [
    "Stranger Things: Intro Theme", "stranger-things  intro theme", "STRANGER THINGS INTRO THEME",
])
def test_reserve_matches_a_cover_by_the_normalised_work_title(work_id):
    planned = CacheTrack("plan", "ST", "cover", "synthwave", group="G1", work_id=work_id)
    other_style = CacheTrack("jazz", "ST jazz", "cover", "jazz", group="J1", work_id=work_id)
    foreign = CacheTrack("foreign", "Other", "cover", "jazz", group="F1", work_id="Imperial March")
    got = gs.set_reserve([planned, other_style, foreign], [planned], [ST.title], "", ["synthwave"], 5)
    assert [t.id for t in got] == ["jazz"]  # кавер произведения темы в другом стиле - ранг 1; чужой пьесы - не запас


# ── (б) запас: сначала треки других задач, близнец - когда других нет ─────────────────────────────────────────────
def _item(id_, group, genre="g1"):
    return gs.SetItem(id_, group, id_, "generated", genre)


def _played(pool, item):
    pool.ready.append(item)
    pool.take(item, "ok")


def _next(pool, genre="g1"):
    """Следующий трек слота, как берёт его ``GenSet._take``: лестница ``spare`` и пометка «сыгран»."""
    item, how = pool.spare(genre)
    pool.take(item, how)
    return item.id, how


def test_reserve_of_the_slot_with_only_a_twin_does_not_beat_another_task_of_another_slot():
    """Только что сыграна ``a0`` задачи ``A``; в запасе своего слота ``g1`` лежит лишь её близнец ``a1``, а в запасе слота
    ``g2`` - трек чужой задачи ``b0``. Раньше брался близнец (запас своего слота первым), теперь - ``b0``."""
    pool = gs.SetPool(reserve=[_item("a1", "A"), _item("b0", "B", "g2")])
    _played(pool, _item("a0", "A"))
    assert _next(pool) == ("b0", "reserve")
    assert _next(pool) == ("a1", "reserve")  # других задач больше нет - близнец, но не повтор


def test_ready_twin_of_the_previous_task_waits_for_another_task_from_the_reserve():
    """Близнец ``a1`` уже в наборе сета, но только что сыграна его пара ``a0``: сначала запас с чужой задачей ``b0``."""
    pool = gs.SetPool(ready=[_item("a1", "A")], reserve=[_item("b0", "B")])
    _played(pool, _item("a0", "A"))
    assert [_next(pool) for _ in range(2)] == [("b0", "reserve"), ("a1", "other")]


def test_ready_track_of_another_task_still_comes_before_the_reserve():
    pool = gs.SetPool(ready=[_item("c0", "C", "g2")], reserve=[_item("b0", "B")])
    _played(pool, _item("a0", "A"))
    assert [_next(pool) for _ in range(2)] == [("c0", "other"), ("b0", "reserve")]


def test_own_genre_of_the_reserve_still_goes_first_among_other_tasks():
    pool = gs.SetPool(reserve=[_item("c0", "C", "g2"), _item("b0", "B", "g1")])
    _played(pool, _item("a0", "A"))
    assert _next(pool)[0] == "b0"


def test_repeat_stays_last():
    pool = gs.SetPool(reserve=[_item("a1", "A")])
    _played(pool, _item("a0", "A"))
    assert [_next(pool)[1] for _ in range(2)] == ["reserve", "repeat"]


# ── (в) RTTTL хуков - только запас ───────────────────────────────────────────────────────────────────────────────
SOURCES = (("pdmx:theme", "M-theme"), ("pdmx:main", "M-main"), ("pdmx:bells", "M-bells"), ("rtttl:user_st", "M-hook"))


def _labels(sources, turn):
    return [label for label, _track in gs.turn_sources(sources, turn)]


def test_scores_rotate_and_the_hook_is_always_the_reserve_at_the_tail():
    assert _labels(SOURCES, 0) == ["pdmx:theme", "pdmx:main", "pdmx:bells", "rtttl:user_st"]
    assert _labels(SOURCES, 1) == ["pdmx:main", "pdmx:bells", "pdmx:theme", "rtttl:user_st"]
    assert _labels(SOURCES, 2) == ["pdmx:bells", "pdmx:theme", "pdmx:main", "rtttl:user_st"]
    assert _labels(SOURCES, 3) == _labels(SOURCES, 0)  # круг из трёх партитур; четвёртая задача не начинает с хука
    assert all(_labels(SOURCES, turn)[0] != "rtttl:user_st" for turn in range(12))


def test_without_scores_the_hooks_themselves_rotate_and_nothing_is_lost():
    hooks = (("rtttl:a", "A"), ("rtttl:b", "B"))
    assert _labels(hooks, 0) == ["rtttl:a", "rtttl:b"] and _labels(hooks, 1) == ["rtttl:b", "rtttl:a"]
    assert gs.turn_sources((), 3) == ()
    assert sorted(_labels(SOURCES, 1)) == sorted(label for label, _t in SOURCES)
