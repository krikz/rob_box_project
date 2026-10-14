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
    """Деньги есть: каждый трек — ``generate:cover``; произведения чередуются (В16: A, B, A, B — не A, A, B, B), второй
    вариант задачи произведения закрывает его следующий трек; строка ``[source] сет i/N`` — кавер в выборе."""
    with caplog.at_level(logging.INFO):
        routes = _router(tmp_path, Sets()).route_slots(CLUB, works=[A, B])
    assert [r.id for r in routes] == [COVER] * 4 and [r.work for r in routes] == [A, B, A, B]
    lines = [r.message for r in caplog.records if "[source] сет" in r.message]
    assert len(lines) == 4 and all(f"кандидаты=['{COVER}', 'synth:club'] → {COVER}" in line for line in lines)
    assert ["(второй вариант задачи)" in line for line in lines] == [False, False, True, True]


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
    key = "cover|club|starwars"
    assert plan.keys() == [key, key] and plan.covers() == {key: A}
    run = gs.GenSetMaker(Kie(), lambda: _lib(tmp_path), DeterministicChooser(), cover=maker)._set_cover(plan)
    gen = gs.GenSet(provider=Kie(), library=_lib(tmp_path), ask=SourceAsk(genre="club"), genre="club", theme="",
                    set_id="gen1", bpm=120, tracks=2, cache=[], keys=plan.keys(),
                    covers=gs.SetCovers(plan.covers(), run), spawn=lambda fn: fn())
    gen.order_more()
    assert maker.made == [(track, A, "club", 0.0, False)] and gen.task_ids == ["T1"] and gen.spent == 12.0
    title = "Звёздные войны (кавер)"
    ready = [(i.id, i.genre, i.title) for i in gen._ready]  # noqa: SLF001
    assert ready == [("c1a", key, title), ("c1b", key, title)]
    assert gs.genre_name(key) == "клуб"


# ── В16 (14.10, Шифу: «он одни и те же треки ставит»): разные произведения темы в треках ────────────────────────
#: Контуры двух пьес (13 нот, ``THEME_REF_NOTES`` = 12): версии одной пьесы — транспозиция, другая — иной рисунок.
THEME_NOTES, MARCH_NOTES = "c d e c e f g e g a g f e", "g g g d# a# g d# a# g d6 d6 d6 d#6"


def _rtttl(name, notes, octave=5):
    return f"{name}:d=8,o={octave},b=120:" + ",".join(notes.split())


SW6, SW8 = _rtttl("sw6", THEME_NOTES), _rtttl("sw8", THEME_NOTES, octave=4)
IMP = _rtttl("imp", MARCH_NOTES)


def test_versions_of_one_piece_group_by_melody_not_by_name():
    from rob_box_music.arrange.hook import same_tune, tune_groups

    assert same_tune(SW6, SW8) and not same_tune(SW6, IMP)
    groups = tune_groups({"starwars_4": IMP, "starwars_8": SW8, "starwars_6": SW6})
    assert groups == [("starwars_4",), ("starwars_8", "starwars_6")]


def _records(rtttls, titles):
    return lambda ids: {h: ({"name": h, "rtttl": rtttls[h]}, titles[h]) for h in ids if h in rtttls}


def test_theme_covers_one_cover_per_piece_main_theme_first(monkeypatch, caplog):
    """Пьесы темы — по мелодии хуков; главная (с первым хуком строки ``space``: starwars_6) первой; мелодия — партитура
    с мотивом ЭТОГО эталона, иначе сам RTTTL; лог перечисляет произведения и источник каждого."""
    found = ("starwars_4", "starwars_8", "starwars_6")
    profile = SimpleNamespace(materials=(A.material,), theme_hooks=found + ("the_x_files",), found=found, row="space")
    dj = SimpleNamespace(theme_profile=lambda theme, find=None: profile, quick_find=None, _scores=None)
    records = _records({"starwars_4": IMP, "starwars_8": SW8, "starwars_6": SW6, "the_x_files": IMP},
                       {"starwars_4": "Имперский марш", "starwars_8": "Star Wars", "starwars_6": "Star Wars - Start",
                        "the_x_files": "The X-files"})

    def score(materials, ids, q, seed, references):  # партитура A — главная тема, марша в ней нет
        ok = SW6 in references
        return SimpleNamespace(found=ok, melody_id=A.material, track="T-score", reason="" if ok else "нет мотива")

    monkeypatch.setattr(piece_cover, "pick_score", score)
    monkeypatch.setattr(piece_cover, "pick_record", lambda rec, title, q, seed: SimpleNamespace(
        found=True, track=f"T-{rec['name']}"))
    monkeypatch.setattr(piece_cover, "theme_fit", lambda material, refs: 0.88)
    monkeypatch.setattr(piece_cover, "PlanMaterials", lambda lib, ids, log: {A.material: object()})
    with caplog.at_level(logging.INFO):
        got = piece_cover.theme_covers(dj, "звёздные войны", 7, records=records)
    assert [(w.title, w.material, t) for w, t in got] == [("Star Wars", A.material, "T-score"),
                                                          ("Имперский марш", "starwars_4", "T-starwars_4")]
    line = next(r.message for r in caplog.records if "[cover] тема" in r.message)
    assert f"«Star Wars» ← {A.material} 0.88" in line and "«Имперский марш» ← rtttl:starwars_4" in line
    assert "the_x_files" not in line  # хук строки таблицы, а не вселенной: найденные по словам темы есть


def test_theme_covers_need_a_reference_motif_to_prove_recognition():
    """Эталонов RTTTL нет — узнаваемость не доказать, каверов нет."""
    profile = SimpleNamespace(materials=(A.material,), theme_hooks=("starwars_6",), found=(), row="space")
    dj = SimpleNamespace(theme_profile=lambda theme, find=None: profile, quick_find=None, _scores=None)
    assert piece_cover.theme_covers(dj, "звёздные войны", 7, records=lambda ids: {}) == []
    assert piece_cover.theme_covers(dj, "звёздные войны", 7) == []


C = CoverWork("Star Wars - Cantina", "starwars", spoken="Star Wars - Cantina")


def test_ten_minute_star_wars_set_plays_different_pieces_never_back_to_back(tmp_path):
    """Приёмка В16: сет «звёздные войны» на 10 минут при деньгах — ≥ 3 произведения, главная тема первой, одно
    произведение не подряд; вторые варианты задач — у другого места сета, новых задач — по числу произведений."""
    routes = _plan(tmp_path, [A, B, C]).routes
    works = [r.work for r in routes]
    assert len(set(works)) == 3 and works[0] == A and all(a != b for a, b in zip(works, works[1:]))
    assert works == [A, B, C, A, B, C, A]


def test_two_pieces_from_one_score_keep_their_own_melody(tmp_path):
    """Медли — материал двух произведений (Cantina 0.71 на роботе): у каждого свой трек (свой такт мотива), ключ
    исполнителя — произведение, а не id партитуры."""
    maker = Maker()
    medley = CoverWork("Star Wars - Cantina", A.material, spoken="Кантина")
    plan = gs.SetRoutes(("club", "club"), tuple(_router(tmp_path, Sets()).route_slots(CLUB[:2], works=[A, medley])),
                        theme_covers=((A, "T-main"), (medley, "T-cantina")))
    run = gs.GenSetMaker(Kie(), lambda: _lib(tmp_path), DeterministicChooser(), cover=maker)._set_cover(plan)
    covers = gs.SetCovers(plan.covers(), run)
    for key in plan.keys():
        covers.task(key, SourceAsk(genre="club"), 0.0)
    made = [(track, work.title) for track, work, *_rest in maker.made]
    assert made == [("T-main", "STAR WARS"), ("T-cantina", "Star Wars - Cantina")]


def _played_cover(lib, hours):
    """Кэш кавера главной темы (стиль клуб), игранный ``hours`` часов назад."""
    from datetime import datetime, timedelta, timezone

    track = lib.save_track(b"MP3:90.0", prompt="cover", title="STAR WARS (RoBBox cover)", is_instrumental=True,
                           kind="cover", style_key="club", task_id="OLD", variant=0, duration_ms=90000,
                           work_id="STAR WARS")["id"]
    stamp = (datetime.now(timezone.utc) - timedelta(hours=hours)).isoformat()
    lib._conn.execute("UPDATE generated_tracks SET last_played_at = ? WHERE id = ?", (stamp, track))  # noqa: SLF001
    lib._conn.commit()  # noqa: SLF001
    return track


@pytest.mark.parametrize("hours, money, first", [(1.0, 900.0, COVER), (20.0, 900.0, "cache"), (1.0, 0.0, "cache")])
def test_cache_played_an_hour_ago_loses_to_a_new_order_with_money(tmp_path, hours, money, first):
    """Кэш, игранный за ``GEN_FRESH_HOURS``, при деньгах проигрывает новому заказу (14.10: 4 трека из сета часом
    раньше); давно игранный — первым, как раньше; без денег сыгранное лучше синтеза."""
    lib = _lib(tmp_path)
    cached = _played_cover(lib, hours)
    router = SourceRouter(DeterministicChooser(), lambda: lib, sets=Sets(balance=money))
    routes = router.route_slots(CLUB[:2], works=[A, B])
    assert routes[0].id == (f"cache:{cached}" if first == "cache" else first)


def test_star_wars_set_through_dj_set_routes_covers(tmp_path, monkeypatch):
    """«сет на тему звёздных войн» при деньгах: путь каждого трека — кавер материала темы, мост — наш синтез."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.tools_v2.theme_covers",
                        lambda dj, theme, seed, log, records: [(A, object())])
    lib, maker = _lib(tmp_path), Maker()
    dj = _dj(lib, Kie())
    dj.gen_sets = gs.GenSetMaker(Kie(), lambda: lib, DeterministicChooser(), cover=maker)
    out = dj.execute(action="start", theme="звёздные войны", style="club", tracks=4,
                     heard_text="сыграй сет на тему звёздных войн на 4 трека")
    assert out.success, out.error
    assert out.data["routes"] == [COVER] * 4 and out.data["source"] == "generated"
    dj.execute(action="stop")


# ── 14.10 #3642: сет «на N минут» набирает треки по плановой длине выбранного пути ────────────────────────────────
def _plan(tmp_path, works, **kw):
    maker = gs.GenSetMaker(Kie(), lambda: _lib(tmp_path), DeterministicChooser(), cover=Maker())
    maker.router = _router(tmp_path, Sets())
    return maker.plan("", "club", kw.pop("heard", "сет на 10 минут"), 5, covers=[(w, object()) for w in works], **kw)


def test_ten_minute_set_with_covers_is_filled_by_cover_length(tmp_path):
    """600 с, каверы по 90 с: прежний подсчёт по 150 с давал 4 трека = 6 минут. Файл звучит весь, не последний
    трек короче на кроссфейд 1.5 с (В15): 88.5 с на трек, последний 90 с — плановый конец ≥ 600 − 90."""
    from rob_box_music.source import route_seconds

    plan = _plan(tmp_path, [A, B])
    assert {r.id for r in plan.routes} == {COVER} and len(plan.routes) == len(plan.genres) == 7
    assert route_seconds(plan.routes[0]) == 88.5 and route_seconds(plan.routes[0], True) == 90
    end = sum(route_seconds(r) for r in plan.routes[:-1]) + route_seconds(plan.routes[-1], True)
    assert end >= 600 - 90 and end - route_seconds(plan.routes[-1], True) < 600 - 60  # последний трек нужен


def test_ten_minute_set_with_instrumental_stays_four_tracks(tmp_path):
    plan = _plan(tmp_path, [])
    assert [r.id for r in plan.routes] == [GEN] * 4


def test_named_tracks_and_set_without_time_work_as_before(tmp_path):
    named = gs.GenSetMaker(Kie(), lambda: _lib(tmp_path), DeterministicChooser())
    named.router = _router(tmp_path, Sets())
    assert len(named.plan("", "club", "сыграй 4 трека", 4, named=True, covers=[(A, object())]).routes) == 4
    assert len(named.plan("", "club", "сыграй сет", 5).routes) == 5
    assert gs.slot_count(1, 4, True, None) == 4 and gs.slot_count(2, 1, False, None) == 2


def test_timed_set_stops_at_the_task_limit(tmp_path, monkeypatch):
    """Задач генерации не больше ``GEN_SET_MAX_TASKS``: набор по времени не заказывает 11-ю."""
    monkeypatch.setattr("rob_box_music.knowledge.GEN_SET_MAX_TASKS", 2)
    routes = _router(tmp_path, Sets()).route_slots(CLUB * 10, works=[A], seconds=3600)
    assert len(routes) == 4  # 2 задачи × 2 варианта
