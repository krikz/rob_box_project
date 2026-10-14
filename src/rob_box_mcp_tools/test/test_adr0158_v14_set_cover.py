"""ADR-0158 В14 (решение Шифу 14.10): тематический сет с нашим материалом темы — каверы материала в треках.

Живой провал 14.10 08:38 UTC (`gen67109`): «Анакин Скайуокер, путешествие к Звезде Смерти» — тема распознана,
материалы `pdmx:QmRmXQ…` есть, но треки 2–5 ушли в `generate:instrumental:V6` по тегам `cosmic, spacey` — мелодии
Звёздных войн нет. Шифу: «если у нас есть материал то идем через ветку кавер … кавер в приоритете». Без сети:
фейковый KIE и исполнитель кавера, настоящий кэш ``GeneratedMusicLibrary``.
"""

from __future__ import annotations

import logging
from types import SimpleNamespace

import pytest

from rob_box_mcp_tools.engine import piece_cover
from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_mcp_tools.engine.gen.cover import CoverResult
from rob_box_mcp_tools.engine.source_route import SourceRouter
from rob_box_music.choice import DeterministicChooser
from rob_box_music.source import CoverWork, SourceAsk

from .test_adr0158_pr8_gen_set import Kie, _dj, _lib

pytestmark = pytest.mark.unit

A = CoverWork("STAR WARS", "pdmx:QmRmXQhpQ2qesBm9oFVP58yuByZcZRVfvPYAbHRAYbLvsZ", spoken="Звёздные войны")
B = CoverWork("Imperial March", "pdmx:QmImperial", spoken="Имперский марш")
COVER, GEN = "generate:cover:V6", "generate:instrumental:V6"
CLUB = [SourceAsk(genre="club")] * 4


class Sets:
    """Исполнитель сета для ``SourceRouter``: генерация и кавер готовы, баланс ``balance``."""

    def __init__(self, balance=900.0, cover=True):
        self._balance, self._cover = balance, cover

    def ready(self):
        return True

    def balance(self):
        return self._balance

    def cover_ready(self):
        return self._cover


class Maker:
    """``CoverMaker`` без сети: заказ запоминается, ответ — два варианта."""

    def __init__(self):
        self.made = []

    def ready(self):
        return True

    def make(self, track, work, ask, set_spent=None, early=True):
        self.made.append((track, work, ask.genre, set_spent, early))
        n = len(self.made)
        return CoverResult(f"c{n}a", "", f"T{n}", 12.0, track_ids=(f"c{n}a", f"c{n}b"))


def _router(tmp_path, sets):
    return SourceRouter(DeterministicChooser(), lambda: _lib(tmp_path), sets=sets)


def test_theme_set_with_our_material_covers_every_track_two_tracks_per_task(tmp_path, caplog):
    """Деньги есть: каждый трек — ``generate:cover``; второй вариант задачи закрывает следующий трек того же стиля,
    материалы — по кругу задачами (A, A, B, B); строка ``[source] сет i/N`` показывает кавер в кандидатах и выборе."""
    with caplog.at_level(logging.INFO):
        routes = _router(tmp_path, Sets()).route_slots(CLUB, works=[A, B])
    assert [r.id for r in routes] == [COVER] * 4 and [r.work for r in routes] == [A, A, B, B]
    lines = [r.message for r in caplog.records if "[source] сет" in r.message]
    assert len(lines) == 4 and all(f"кандидаты=['{COVER}', 'synth:club'] → {COVER}" in line for line in lines)
    assert "(второй вариант задачи)" in lines[1] and "(второй вариант задачи)" not in lines[2]


def test_theme_without_material_stays_tag_instrumental(tmp_path):
    assert [r.id for r in _router(tmp_path, Sets()).route_slots(CLUB, works=[])] == [GEN] * 4


def test_no_money_no_cover_synth_is_the_fallback(tmp_path):
    routes = _router(tmp_path, Sets(balance=0.0)).route_slots(CLUB, works=[A])
    assert [r.id for r in routes] == ["synth:club"] * 4 and routes[0].gen_reason == "no_credits"


def test_cover_executor_not_ready_falls_back_to_tag_instrumental(tmp_path):
    routes = _router(tmp_path, Sets(cover=False)).route_slots(CLUB, works=[A])
    assert [r.id for r in routes] == [GEN] * 4


def test_cached_cover_of_the_same_piece_and_style_comes_first(tmp_path):
    """Кэш кавера той же пьесы и стиля — первым, раньше новой задачи; дальше — кавер следующего материала."""
    lib = _lib(tmp_path)
    cached = lib.save_track(b"MP3:90.0", prompt="cover", title="STAR WARS (RoBBox cover)", is_instrumental=True,
                            kind="cover", style_key="club", task_id="OLD", variant=0, duration_ms=90000,
                            work_id="STAR WARS")["id"]
    router = SourceRouter(DeterministicChooser(), lambda: lib, sets=Sets())
    routes = router.route_slots(CLUB[:2], works=[A, B])
    assert [r.id for r in routes] == [f"cache:{cached}", COVER] and routes[1].work == B


def test_gen_set_orders_cover_tasks_through_the_same_cover_maker(tmp_path):
    """Исполнитель трека сета — тот же ``CoverMaker.make``: материал по кругу, стиль слота, лимит сета, оба варианта
    в сет одним ключом слота (соседями варианты не ставит ``pick_item``)."""
    maker, track = Maker(), object()
    plan = gs.SetRoutes(("club", "club"), tuple(_router(tmp_path, Sets()).route_slots(CLUB[:2], works=[A])),
                        theme_covers=((A, track),))
    assert plan.keys() == ["cover|club", "cover|club"] and plan.covers() == {"cover|club": [A]}
    run = gs.GenSetMaker(Kie(), lambda: _lib(tmp_path), DeterministicChooser(), cover=maker)._set_cover(plan)
    gen = gs.GenSet(provider=Kie(), library=_lib(tmp_path), ask=SourceAsk(genre="club"), genre="club", theme="",
                    set_id="gen1", bpm=120, tracks=2, cache=[], keys=plan.keys(),
                    covers=gs.SetCovers(plan.covers(), run), spawn=lambda fn: fn())
    gen.order_more()
    assert maker.made == [(track, A, "club", 0.0, False)] and gen.task_ids == ["T1"] and gen.spent == 12.0
    title = "Звёздные войны (кавер)"
    ready = [(i.id, i.genre, i.title) for i in gen._ready]  # noqa: SLF001
    assert ready == [("c1a", "cover|club", title), ("c1b", "cover|club", title)]
    assert gs.genre_name("cover|club") == "клуб"


def test_theme_covers_need_a_reference_motif_to_prove_recognition(monkeypatch):
    """Материал без эталона RTTTL темы — узнаваемость не доказать, каверов нет; с эталоном — те, где мотив нашёлся."""
    profile = SimpleNamespace(materials=(A.material, B.material), theme_hooks=("starwars_6",))
    dj = SimpleNamespace(theme_profile=lambda theme, find=None: profile, quick_find=None, _scores=None,
                         _melodies=lambda ids: {})
    assert piece_cover.theme_covers(dj, "звёздные войны", 7) == []
    dj._melodies = lambda ids: {"starwars_6": "sw6:d=4,o=5,b=120:c"}
    picks = {A.material: SimpleNamespace(found=True, work="STAR WARS", title="Звёздные войны", track="T", reason=""),
             B.material: SimpleNamespace(found=False, reason="нет мотива эталона")}
    monkeypatch.setattr(piece_cover, "pick_score", lambda materials, ids, q, seed, references: picks[ids[0]])
    assert piece_cover.theme_covers(dj, "звёздные войны", 7) == [(A, "T")]


def test_star_wars_set_through_dj_set_routes_covers(tmp_path, monkeypatch):
    """«сет на тему звёздных войн» при деньгах: путь каждого трека — кавер материала темы, мост — наш синтез."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.tools_v2.theme_covers", lambda dj, theme, seed, log: [(A, object())])
    lib, maker = _lib(tmp_path), Maker()
    dj = _dj(lib, Kie())
    dj.gen_sets = gs.GenSetMaker(Kie(), lambda: lib, DeterministicChooser(), cover=maker)
    out = dj.execute(action="start", theme="звёздные войны", style="club", tracks=4,
                     heard_text="сыграй сет на тему звёздных войн на 4 трека")
    assert out.success, out.error
    assert out.data["routes"] == [COVER] * 4 and out.data["source"] == "generated"
    dj.execute(action="stop")
