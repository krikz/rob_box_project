"""#4015 — «спой <название>»: настоящая причина отказа, а не ложный ``not_wired``.

Живой прогон 24.10 (Sound Pi): «робот спой раннинг ап дэд хилл на балалайке и гуслях» → ``[source] asked=[vocal]
кандидаты=['generate:song:V6'] → decline:not_wired (code) gen=ok`` → «генерация у меня ещё не подключена», хотя
студия песни была подключена (тот же вечер — каверы KIE). Причина: ``SourceRouter._executable`` считал песню
исполнимой только при ``route.work is None``. Теперь «не подключено» — только когда исполнителя нет, а песня на
названное произведение — своя причина (``named_song``) до выбора, с фразой кода и строкой лога с ``work``.

Ревью PR #4124: «названное» — это пьеса, чьё название несёт понятые слова заказа (``CoverWork.named``,
``piece.named_pick``), а не любая собранная пьеса: «спой что-нибудь», «спой народную песню» — песня робота.
"""

from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import pytest

from rob_box_mcp_tools.base import MCPToolResult
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_mcp_tools.engine.search import ThemeHits, ThemeQuery, part_query
from rob_box_mcp_tools.engine.song_order import SongOrder
from rob_box_mcp_tools.engine.instrumental_order import InstrumentalOrder
from rob_box_mcp_tools.engine.source_route import SourceRouter
from rob_box_mcp_tools.engine.tools_v2 import SOURCE_DECLINED, DjSetTool, RequestMusicTool
from rob_box_music.choice import DeterministicChooser
from rob_box_music.source import (DECLINE, GENERATE, NAMED_SONG, REASON_WORDS, CoverWork, Route, SourceAsk,
                                  SourceContext, decline_text, source_ask, source_candidates)

from .test_adr0158_pr4_cover import FakeMaker, _records
from .test_adr0158_pr7_song import FakeStudio
from .test_engine_session import _rig
from .test_issue_3604_composer_filter import HITS, _both
from .test_piece_fail_open import Scores

pytestmark = pytest.mark.unit

HILL = CoverWork("Running Up That Hill", "runningu", "balalaika", "Running Up That Hill", named=True)
#: Та же пьеса, но подобранная кодом (жанр, «что-нибудь»), а не названная человеком.
PICKED = replace(HILL, named=False)
SING = SourceAsk(vocal=True)


class Log:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(msg)

    warning = info


def _router(studio=None, instrumental=None, log=None):
    log = log or Log()
    return SourceRouter(DeterministicChooser(), log=lambda: log, studio=studio, instrumental=instrumental), log


# ── причина: исполнитель есть, названное — не «не подключено» ───────────────────────────────────────────────────

def test_live_executor_with_a_named_work_is_not_not_wired():
    """Исполнитель песни подключён + ``route.work`` задан → не ``not_wired`` (ровно строка 198 из карточки)."""
    router, _log = _router(studio=FakeStudio())
    assert router._executable(Route(GENERATE, "generate:song:V6", SING, work=HILL)) == ""  # noqa: SLF001
    assert router._executable(Route(GENERATE, "generate:robot_song:V6", SING, work=HILL)) == ""  # noqa: SLF001
    instrumental, _log = _router(instrumental=SimpleNamespace())
    route = Route(GENERATE, "generate:instrumental:V6", SourceAsk(), work=HILL)
    assert instrumental._executable(route) == ""  # noqa: SLF001


def test_missing_executor_is_still_not_wired():
    router, _log = _router()
    assert router._executable(Route(GENERATE, "generate:song:V6", SING)) == "not_wired"  # noqa: SLF001
    assert router._executable(Route(GENERATE, "generate:song:V6", SING, work=HILL)) == "not_wired"  # noqa: SLF001
    route = Route(GENERATE, "generate:instrumental:V6", SourceAsk())
    assert router._executable(route) == "not_wired"  # noqa: SLF001


def test_sing_a_named_work_has_its_own_reason_before_the_choice():
    """Песня робота — свой текст (ADR-0158 §3.4): кандидата песни на названное нет, причина ``named_song`` (не
    ``not_wired``), генерация готова (``gen_ready``) — это не её отказ."""
    ctx = SourceContext(gen_ready=True, day_left=120.0)
    options, reason = source_candidates(SING, ctx, piece=True, work=HILL)
    assert [o.id for o in options] == [f"decline:{NAMED_SONG}"] and reason == NAMED_SONG
    options, reason = source_candidates(SING, ctx, piece=True)  # пьеса не нашлась — песня на тему, как было
    assert [o.id for o in options] == ["generate:song:V6"] and reason == ""
    options, reason = source_candidates(SING, ctx, piece=True, work=PICKED)  # пьесу подобрал код — песня робота
    assert [o.id for o in options] == ["generate:song:V6"] and reason == ""
    options, _ = source_candidates(SourceAsk(), replace(ctx, cover_ready=True), piece=True, work=HILL)
    assert "generate:cover:V6" in [o.id for o in options]  # не поют — кавер, как было


def test_route_log_names_the_work_and_the_true_reason():
    router, log = _router(studio=FakeStudio())
    route = router.route(SING, piece=True, work=HILL)
    assert route.kind == DECLINE and route.gen_reason == NAMED_SONG and not route.failed
    line = log.lines[-1]
    assert "[source]" in line and "work=«Running Up That Hill»" in line
    assert f"→ decline:{NAMED_SONG}" in line and f"gen={NAMED_SONG}" in line and "not_wired" not in line
    router.route(SING, piece=True, work=PICKED)  # пьесу подобрал код: лог не утверждает, что её назвали
    assert "piece=«Running Up That Hill»" in log.lines[-1] and "work=" not in log.lines[-1]


def test_decline_phrase_is_built_from_the_reason_and_offers_the_melody():
    router, _log = _router(studio=FakeStudio())
    text = decline_text(router.route(SING, piece=True, work=HILL))
    assert text == (f"Спеть «Running Up That Hill» не могу: {REASON_WORDS[NAMED_SONG]}. Могу сыграть её мелодию.")
    assert "не подключена" not in text


def test_song_and_instrumental_orders_take_their_kinds_only():
    """Кто исполняет генерацию — по виду (песня / инструментал), без условия на ``work``; кавер — ни один из них."""
    songs = SongOrder(FakeStudio(), route=lambda *a, **k: None)
    instrumental = InstrumentalOrder(object(), None, None)
    song, robot = Route(GENERATE, "generate:song:V6", SING), Route(GENERATE, "generate:robot_song:V6", SING)
    instr, cover = Route(GENERATE, "generate:instrumental:V6", SourceAsk()), Route(GENERATE, "generate:cover:V6",
                                                                                   SourceAsk(), work=HILL)
    assert songs.handles(song) and songs.handles(robot) and not songs.handles(instr) and not songs.handles(cover)
    assert instrumental.handles(instr) and not instrumental.handles(song) and not instrumental.handles(cover)
    assert not SongOrder(None, route=lambda *a, **k: None).handles(song)


# ── заказ целиком: «спой валькирии» при живых студии и кавере ───────────────────────────────────────────────────

def _echo(word):
    """Подставной поиск: находки фикстуры (валькирии), но слова заказа — свои, а не «валькирии» на любую просьбу:
    пьеса без названия собирается, как у кода, по «ближайшему»."""
    return ThemeHits(tuple(HITS), query=ThemeQuery(word, (part_query(None, word),)))


def _request(tmp_path, studio, maker, played, words=lambda word: _both(HITS, False)):
    rig = _rig()
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=words, seed=lambda: 4242, scores=Scores())

    def play(track_id):
        played.append(track_id)
        return MCPToolResult(success=True, data={"ok": True, "track_id": f"gen_{track_id[:8]}_1"})

    return RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 5, records=_records,
                            words=words, gen_library=lambda: GeneratedMusicLibrary(str(tmp_path / "lib")),
                            play_cached=play, cover=maker, speak=lambda text: None, songs=studio), rig


def test_sing_a_named_piece_declines_with_the_true_reason_and_starts_nothing(tmp_path):
    studio, maker, played = FakeStudio(), FakeMaker(), []
    req, rig = _request(tmp_path, studio, maker, played)
    assert source_ask("спой валькирии").vocal
    result = req.execute(intent="melody", text="спой валькирии", heard_text="Робот, спой валькирии")
    assert not result.success and result.data["reason"] == SOURCE_DECLINED
    assert result.data["route"] == f"decline:{NAMED_SONG}"
    message = result.data["message"]
    assert message.startswith("Спеть «") and REASON_WORDS[NAMED_SONG] in message and "не подключена" not in message
    assert studio.requests == [] and maker.orders == [] and played == [] and not rig.owner.is_playing()


@pytest.mark.parametrize("intent, genre, text", [("track", "folk", "спой народную песню"),
                                                 ("melody", None, "спой что-нибудь")])
def test_sing_with_a_piece_the_code_picked_orders_a_song_not_named_song(tmp_path, intent, genre, text):
    """Ревью #4124: пьеса собралась, но человек её не называл (жанр, «что-нибудь») — не «чужие песни по названию я не
    пою», а песня робота (студия подключена). Признак «назвал» — слова заказа в названии пьесы (``played["named"]``),
    а не «пьеса собралась»."""
    studio, maker, played = FakeStudio(), FakeMaker(), []
    req, _rig_ = _request(tmp_path, studio, maker, played, words=_echo)
    result = req.execute(intent=intent, genre=genre, text=text, heard_text=f"Робот, {text}")
    assert result.data.get("route") != f"decline:{NAMED_SONG}", result.data
    assert REASON_WORDS[NAMED_SONG] not in str(result.error or "") + str(result.data.get("message") or "")
    assert len(studio.requests) == 1 and maker.orders == []


def test_named_piece_is_marked_named_and_a_picked_one_is_not(tmp_path):
    """``played["named"]`` (``piece.named_pick``) — слова названия заказа есть в названии собранной пьесы."""
    studio, maker, played = FakeStudio(), FakeMaker(), []
    named, _ = _request(tmp_path, studio, maker, played)
    picked, _ = _request(tmp_path, studio, maker, played, words=_echo)
    assert named._stage_piece("сыграй валькирии", None)["played"]["named"] is True  # noqa: SLF001
    assert picked._stage_piece("сыграй что-нибудь", None)["played"]["named"] is False  # noqa: SLF001


def test_play_the_same_piece_still_goes_the_cover_way(tmp_path):
    """Петь не просили — кавер пьесы, как было (ADR-0158 PR-4): правка причины песни его не трогает."""
    studio, maker, played = FakeStudio(), FakeMaker(), []
    req, _ = _request(tmp_path, studio, maker, played)
    result = req.execute(intent="melody", text="сыграй валькирии", heard_text="Робот, сыграй валькирии")
    assert result.success, result.error
    assert studio.requests == [] and len(maker.orders) == 1
