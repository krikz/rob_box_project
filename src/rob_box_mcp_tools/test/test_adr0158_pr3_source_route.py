"""ADR-0158 PR-3 — ``request_music``/``generate_music`` идут путём музыки: кэш генераций, честный отказ, фразы кода.

Живой прогон 13.10: «сгенерируй пиздатый музон своим голосом» → синтез и «Включаю музыку.». Теперь: трек кэша, спетый
голосом робота, с фразой кода о том, что генерации нет; кэша нет — отказ с фразой, синтез не ставится.
"""

from datetime import datetime, timezone

import pytest

from rob_box_mcp_tools.base import MCPToolResult
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.source_route import cache_track, cache_tracks
from rob_box_mcp_tools.engine.tools_v2 import (SOURCE_DECLINED, DjSetTool, GenerateMusicTool,
                                               RequestMusicTool)
from rob_box_voice.core.media_phrases import SOURCE_DECLINED_REASON, launch_text, source_phrase

from .test_engine_session import _rig

pytestmark = pytest.mark.unit

LIVE = "Робот, сгенерируй пиздатый музон своим голосом"


@pytest.fixture
def library(tmp_path):
    lib = GeneratedMusicLibrary(str(tmp_path / "music_library"))
    for variant in (0, 1):
        lib.save_track(b"ID3song%d" % variant, prompt="grunge", title="Я робот (РОББОКС)", kind="robot_song",
                       style_key="grunge", genre="grunge", task_id="7e10941d", variant=variant,
                       voice="Russian_ReliableMan")
    lib.save_track(b"ID3inst", prompt="grunge", title="Robbox Grunge Test", kind="instrumental", style_key="grunge",
                   is_instrumental=True, task_id="f7cff936", variant=0)
    return lib


def _tools(rig, library=None, played=None):
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 4242)

    def play(track_id):
        played.append(track_id)
        return MCPToolResult(success=True, data={"ok": True, "track_id": f"gen_{track_id[:8]}_1", "started": True})

    req = RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 777, records=lambda ids: {},
                           words=lambda word: ThemeHits(), gen_library=lambda: library,
                           play_cached=play if played is not None else None)
    return req, GenerateMusicTool(None, req)


def test_codes_agree_with_the_voice_side():
    assert SOURCE_DECLINED == SOURCE_DECLINED_REASON


def test_live_phrase_plays_the_robot_song_from_cache_and_says_why(library):
    rig, played = _rig(), []
    req, _gen = _tools(rig, library, played)
    result = req.execute(intent="track", text="сгенерируй музон", heard_text=LIVE)
    assert result.success and result.data["source"] == "cache" and result.data["started"] is True
    assert len(played) == 1 and result.data["title"] == "Я робот (РОББОКС)"
    assert launch_text("Включаю музыку.", result.data) == (
        "Спеть своим голосом сейчас не могу: генерация у меня ещё не подключена. "
        "Ставлю спетое раньше своим голосом: «Я робот (РОББОКС)».")
    assert rig.owner.is_playing() is not True  # синтез не ставился


def test_without_cache_it_declines_with_a_code_phrase_and_plays_nothing():
    rig = _rig()
    req, gen = _tools(rig)
    for result in (req.execute(intent="track", text="сгенерируй музон", heard_text=LIVE),
                   gen.execute(kind="robot_song", heard_text=LIVE)):
        assert not result.success and result.data["reason"] == SOURCE_DECLINED
        assert source_phrase(result.error).startswith("Спеть своим голосом сейчас не могу: генерация у меня ещё не "
                                                      "подключена, а синтез не поёт.")
    assert rig.owner.is_playing() is not True


def test_generate_music_is_a_request_even_without_the_word(library):
    """«поставь что-нибудь» + LLM позвала ``generate_music`` — это просьба сгенерировать: кэш, а не синтез молча."""
    rig, played = _rig(), []
    _req, gen = _tools(rig, library, played)
    result = gen.execute(kind="instrumental", style="grunge", heard_text="поставь что-нибудь")
    assert result.success and result.data["title"] == "Robbox Grunge Test"
    assert result.data["source_message"].startswith("Сгенерировать сейчас не могу: генерация у меня ещё не подключена.")
    assert not gen.execute(kind="opera").success  # перечень, а не свободная строка


def test_plain_club_order_is_unchanged_synth(library):
    rig, played = _rig(), []
    req, _gen = _tools(rig, library, played)
    result = req.execute(intent="track", text="поставь клубный трек")
    assert result.success and result.data["track_id"].startswith("req00777:") and played == []
    assert "source_note" not in result.data


def test_generation_ask_without_cache_plays_synth_and_says_so():
    rig = _rig()
    req, _gen = _tools(rig)
    result = req.execute(intent="track", text="сгенерируй музыку", heard_text="сгенерируй музыку")
    assert result.success and result.data["source"] == "synth"
    assert launch_text("Включаю музыку.", result.data) == (
        "Сгенерировать сейчас не могу: генерация у меня ещё не подключена, играю своё. Включаю музыку.")


def test_cache_rows_become_candidates(library):
    tracks = cache_tracks(library)
    assert {t.title for t in tracks} == {"Я робот (РОББОКС)", "Robbox Grunge Test"}
    assert len({t.group for t in tracks}) == 2  # варианты одной задачи — одна группа
    legacy = cache_track({"id": "x", "title": "Весна пришла", "kind": "", "genre": "indie folk",
                          "is_instrumental": 0, "last_played_at": "2026-10-13T10:00:00+00:00"},
                         datetime(2026, 10, 13, 12, 0, tzinfo=timezone.utc))
    assert (legacy.kind, legacy.style_key, legacy.played_ago_h) == ("song", "", 2.0)


def test_day_left_is_counted_from_the_ledger_not_a_stub(tmp_path):
    """Остаток дня — дневной лимит минус траты ``gen_ledger`` за сегодня; вчерашние не считаются."""
    from rob_box_mcp_tools.engine.gen.budget import DAILY_CREDITS_DEFAULT
    from rob_box_mcp_tools.engine.source_route import day_left

    lib = GeneratedMusicLibrary(str(tmp_path / "music_library"))
    assert day_left(lib) == DAILY_CREDITS_DEFAULT and day_left(None) == 0.0
    now = datetime.now(timezone.utc).replace(microsecond=0)
    lib.ledger.record_spend("today", 12.0, ts=now.isoformat())
    lib.ledger.record_spend("old", 12.0, ts=now.replace(year=now.year - 1).isoformat())
    assert day_left(lib) == DAILY_CREDITS_DEFAULT - 12.0
