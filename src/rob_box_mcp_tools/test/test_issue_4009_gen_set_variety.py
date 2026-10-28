"""#4009 — сет из генерации не повторяет сыгранное, пока есть что-то другое, и не повторяет молча.

Живой сет ``gen64903`` (24.10, «сет в стиле Stranger Things»): трек 2 ждал кавер своего слота, задача вышла на 4 с
позже срока — сет молча повторил трек 1 (``d2e8a33b``, вариант 1 задачи ``b2d4a503``), хотя вариант 0 той же задачи
(``337b8ffb``) лежал в библиотеке. Лестница выбора (``gen_session.SetPool``): свой слот → вариант-сосед → готовый трек
другого слота → запас библиотеки (``set_reserve``) → повтор — последним, с причиной в логе и фразой кода. Без сети и
денег: фейковый KIE, настоящий кэш ``GeneratedMusicLibrary``, фейковый ffmpeg.
"""

from __future__ import annotations

import itertools
import logging

import pytest

from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_mcp_tools.engine.gen.cover import CoverResult
from rob_box_mcp_tools.engine.source_route import cache_tracks
from rob_box_music import knowledge as kn
from rob_box_music.source import CacheTrack, CoverWork, SourceAsk, cover_slot

from .test_adr0158_pr8_gen_set import Kie, _lib, fake_ffmpeg

pytestmark = pytest.mark.unit

ST = CoverWork("Stranger Things Intro Theme", "pdmx:QmbSEvE33N", spoken="Stranger Things Intro Theme")
A = CoverWork("STAR WARS", "pdmx:main", spoken="Звёздные войны")
B = CoverWork("Imperial March", "starwars_4", spoken="Имперский марш")
CLUB = SourceAsk(genre="club")


def _cover(lib, work, style, task, variant, title=None):
    """Кавер ``work`` в кэше (как его сохраняет ``CoverMaker``): id строки."""
    return lib.save_track(b"MP3:90.0", prompt="cover", title=title or f"{work.title} (RoBBox cover)",
                          is_instrumental=True, kind="cover", style_key=style, task_id=task, variant=variant,
                          duration_ms=90000, work_id=work.title)["id"]


def _instrumental(lib, style, task, variant=0, prompt="club"):
    return lib.save_track(b"MP3:90.0", prompt=prompt, title=f"{style} {task}", is_instrumental=True,
                          kind="instrumental", style_key=style, genre=style, task_id=task, variant=variant,
                          duration_ms=90000)["id"]


def _hows(gen, tracks):
    return [gen._items[t.track_id][1] for t in tracks]  # noqa: SLF001


def _ids(gen, tracks):
    return [gen._items[t.track_id][0].id for t in tracks]  # noqa: SLF001


# ── запас библиотеки: что ближе к сету — раньше ─────────────────────────────────────────────────────────────────
def test_set_reserve_takes_tracks_close_to_the_set_nearest_first():
    """Вариант задачи плана → кавер произведения темы (любой стиль) → по теме → инструментал стиля сета; чужой стиль,
    песня, «не нравится» и треки самого плана — не запас; игранный за ``GEN_FRESH_HOURS`` — позже равных."""
    planned = CacheTrack("d2e8", "ST (RoBBox cover)", "cover", "synthwave", group="B2D4", work_id=ST.title)
    library = [
        planned,
        CacheTrack("club1", "Club", "instrumental", "club", group="C1"),  # чужой стиль, не по теме
        CacheTrack("syn-old", "Synth", "instrumental", "synthwave", group="S1", played_ago_h=1.0),
        CacheTrack("syn", "Synth", "instrumental", "synthwave", group="S2"),
        CacheTrack("theme", "Dark", "instrumental", "club", group="T1", prompt="80s retro, stranger things"),
        CacheTrack("st-jazz", "ST jazz", "cover", "jazz", group="J1", work_id=ST.title),
        CacheTrack("song", "Song", "song", "synthwave", instrumental=False, group="V1"),
        CacheTrack("bad", "Bad", "instrumental", "synthwave", rating=-1, group="X1"),
        CacheTrack("337b", "ST (RoBBox cover)", "cover", "synthwave", group="B2D4", work_id=ST.title),
    ]
    got = gs.set_reserve(library, [planned], [ST.title], "stranger things", ["synthwave"], 10)
    assert [t.id for t in got] == ["337b", "st-jazz", "theme", "syn", "syn-old"]
    assert [t.id for t in gs.set_reserve(library, [planned], [ST.title], "", ["synthwave"], 2)] == ["337b", "st-jazz"]


# ── живой случай gen64903: задача слота не успела к сроку ───────────────────────────────────────────────────────
def test_slot_waiting_for_its_task_takes_the_other_variant_from_the_library_not_a_repeat(tmp_path, caplog):
    """Кэш закрыл трек 1 вариантом 1 задачи, задача слота 2 не успела к сроку: трек 2 — вариант 0 той же задачи из
    библиотеки (``reserve``), а не повтор трека 1. Запас кончился, задача всё ещё пишется — повтор с причиной
    ``pending`` и фразой кода."""
    lib, said, held = _lib(tmp_path), [], []
    v1 = _cover(lib, ST, "synthwave", "B2D4", 1)
    v0 = _cover(lib, ST, "synthwave", "B2D4", 0)
    library = cache_tracks(lib)
    cache = [t for t in library if t.id == v1]
    key = cover_slot("synthwave", ST.title)
    reserve = gs.set_reserve(library, cache, [ST.title], "", ["synthwave"], 3)
    assert [t.id for t in reserve] == [v0]
    covers = gs.SetCovers({key: ST}, lambda work, ask, spent: CoverResult(reason="provider_down"))
    gen = gs.GenSet(provider=Kie(), library=lib, ask=SourceAsk(genre="synthwave"), genre="synthwave", theme="",
                    set_id="gen64903", bpm=120, tracks=3, cache=cache, reserve=reserve, keys=[key] * 3, covers=covers,
                    speak=said.append, spawn=held.append, run=fake_ffmpeg,  # задача заказана и не вернулась
                    clock=itertools.count(0, 1000).__next__)  # срок ожидания прошёл сразу
    with caplog.at_level(logging.INFO):
        tracks = [gen.next_track(no, "AB"[(no - 1) % 2]) for no in (1, 2, 3)]
    assert len(held) == 1 and gen.ordered == 1  # одна задача в полёте, вторую не заказывали
    assert _hows(gen, tracks) == ["ok", "reserve", "repeat"]
    assert _ids(gen, tracks)[:2] == [v1, v0]
    assert said == [f"Новый трек не успел дописаться — повторяю сыгранное: «{ST.title}»."]
    repeat = [r.message for r in caplog.records if "повтор" in r.message and "[gen set]" in r.message]
    assert len(repeat) == 1 and "новый трек не успел дописаться (pending" in repeat[0]


# ── M произведений < N слотов: повтор не раньше, чем кончилось всё остальное ────────────────────────────────────
class Maker:
    """Исполнитель кавера сета: ``A`` — 2 варианта в настоящий кэш, остальное — ``provider_down``."""

    def __init__(self, lib):
        self.lib, self.calls = lib, []

    def __call__(self, work, ask, spent):
        self.calls.append(work.title)
        if work != A:
            return CoverResult(reason="provider_down")
        task = f"T{len(self.calls)}"
        ids = tuple(_cover(self.lib, work, ask.genre, task, v, title=f"{work.title} {v}") for v in range(2))
        return CoverResult(ids[0], "", task, 12.0, track_ids=ids)


def test_two_pieces_six_slots_repeat_only_after_variants_other_slots_and_reserve(tmp_path, caplog):
    """2 произведения на 6 слотов, второе не сгенерировалось: сет играет свой вариант, запас библиотеки (кавер того же
    произведения в другом стиле, инструментал стиля сета) и вариант-близнец — но близнец не сразу за своей парой, а
    после трека другой задачи (ревью #4119); только потом повтор; фраза провала и одна фраза повтора на серию (не на
    каждый трек); в строке ``[gen set]`` — ``repeat`` только у последних треков."""
    lib, said = _lib(tmp_path), []
    old_cover = _cover(lib, A, "jazz", "OLD1", 0)
    old_club = _instrumental(lib, "club", "OLD2")
    _instrumental(lib, "rock", "OLD3")  # чужой стиль — не запас
    reserve = gs.set_reserve(cache_tracks(lib), [], [A.title, B.title], "", ["club"], 6)
    assert [t.id for t in reserve] == [old_cover, old_club]
    keys = [cover_slot("club", w.title) for w in (A, B, A, B, A, B)]
    maker = Maker(lib)
    covers = gs.SetCovers({k: w for k, w in zip(keys, (A, B))}, maker, (A, B))
    gen = gs.GenSet(provider=Kie(), library=lib, ask=CLUB, genre="club", theme="", set_id="gen9", bpm=120, tracks=6,
                    cache=[], reserve=reserve, keys=keys, covers=covers, speak=said.append, spawn=lambda fn: fn(),
                    run=fake_ffmpeg)
    with caplog.at_level(logging.INFO):
        tracks = [gen.next_track(no, "AB"[(no - 1) % 2]) for no in range(1, 7)]
    hows, ids = _hows(gen, tracks), _ids(gen, tracks)
    assert hows == ["ok", "reserve", "ok", "reserve", "repeat", "repeat"]
    assert len(set(ids[:4])) == 4 and [ids[1], ids[3]] == [old_cover, old_club]  # до повтора — всё разное
    assert set(ids[4:]) <= set(ids[:4]) and gen.gen_reason == "provider_down"
    first = gen._items[tracks[4].track_id][0]  # noqa: SLF001
    assert said == [gs.failed_text(keys[1], "provider_down"),
                    f"Сервис генерации не отвечает — повторяю сыгранное: «{first.hook or first.title}»."]
    lines = [r.message for r in caplog.records if r.message.startswith("🎧 [gen set] gen9 трек")]
    assert ["(repeat," in line for line in lines] == [False] * 4 + [True] * 2


def test_without_reserve_repeat_is_still_the_last_step_and_names_its_reason():
    """Запаса нет: готовый трек другого слота раньше повтора; повтор — давнее сыгранное, не сосед по задаче."""
    a0, a1 = gs.SetItem("a0", "A", "a", genre="x"), gs.SetItem("a1", "A", "a", genre="x")
    b0 = gs.SetItem("b0", "B", "b", genre="y")
    pool = gs.SetPool([a0, a1, b0])
    for item, how in ((a0, "ok"), (b0, "ok")):
        assert pool.own(item.genre) == (item, how)
        assert pool.take(item, how) is False
    assert pool.own("y") == (None, "") and pool.spare("y") == (a1, "other")
    assert pool.take(a1, "other") is False
    assert pool.spare("y") == (b0, "repeat")  # не сосед по задаче A
    assert pool.take(b0, "repeat", "pending") is True  # начало серии — фраза
    assert pool.spare("x") == (a0, "repeat") and pool.take(a0, "repeat", "pending") is False  # та же серия
    assert pool.take(a1, "repeat", "no_credits") is True  # причина сменилась — фраза


@pytest.mark.parametrize("gen_reason, inflight, ordered, why", [
    ("no_credits", False, 3, "no_credits"), ("", True, 3, "pending"), ("", False, kn.GEN_SET_MAX_TASKS, "tasks_full"),
    ("", False, 1, "no_order")])
def test_repeat_reason_is_decided_by_code(gen_reason, inflight, ordered, why):
    assert gs.repeat_why(gen_reason, inflight, ordered) == why
    words = gs.repeat_words(why)
    assert words and gs.repeat_text(gs.SetItem("x", "g", "Rock T1.0"), why) == \
        f"{words[:1].upper()}{words[1:]} — повторяю сыгранное: «Rock T1.0»."


def test_gen_set_maker_passes_the_library_reserve_to_the_set(tmp_path):
    """Запуск сета читает библиотеку один раз: запас — треки вне плана, близкие к сету (:func:`set_reserve`)."""
    from rob_box_music.source import CACHE, Route

    lib = _lib(tmp_path)
    v1 = _cover(lib, ST, "synthwave", "B2D4", 1)
    v0 = _cover(lib, ST, "synthwave", "B2D4", 0)
    planned = next(t for t in cache_tracks(lib) if t.id == v1)
    plan = gs.SetRoutes(("synthwave",), (Route(CACHE, f"cache:{v1}", SourceAsk(genre="synthwave"), track=planned),))
    maker = gs.GenSetMaker(Kie(), lambda: lib, None)
    assert [t.id for t in maker._reserve(plan, ["synthwave"], 4, logging.getLogger("t"))] == [v0]  # noqa: SLF001
