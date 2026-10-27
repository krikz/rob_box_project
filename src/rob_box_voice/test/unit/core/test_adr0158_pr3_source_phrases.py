"""ADR-0158 PR-3 — «сгенерируй …» решает код: роутер без LLM, фраза о пути и отказ — фразы кода (ADR-0148).

Живой прогон 13.10: «Робот, сгенерируй пиздатый музон своим голосом» уходила в LLM (грамматика не узнавала глагол),
та ставила синтез, и робот говорил «Включаю музыку.» — не сказав, что генерировать не умеет.
"""

from __future__ import annotations

import asyncio

from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_phrases import launch_text, source_error, source_phrase
from rob_box_voice.core.media_plan_run import run_media_plan
from rob_box_voice.core.music_player_state import MusicEventLog, build_music_event_payload
from rob_box_voice.core.media_router import MediaRouter, MediaState
from rob_box_voice.core.music_turn import MusicTurn

LIVE = "Робот, сгенерируй пиздатый музон своим голосом"
DECLINE = ("Сгенерировать песню своим голосом сейчас не могу: генерация у меня ещё не подключена. "
           "Могу спеть сам вживую под синт.")
CACHE = {"ok": True, "track_id": "gen_c0fe_1", "source": "cache",
         "source_message": "Ставлю спетое раньше своим голосом: «Я робот (РОББОКС)»."}


def test_generate_verb_goes_to_generate_music_without_llm():
    # ADR-0150 §9.1 ревизия 5 (#3696): «сгенерируй/спой/сочини» — класс генерации кодом, путь решает generate_music
    for text in (LIVE, "сгенерируй музыку", "давай сгенерируй трек про космос", "спой песню про кота"):
        assert parse_media_command(text).intent is MediaIntent.GENERATE
    plan = MediaRouter().route(LIVE, MediaState())
    assert [(c.name, c.arguments["kind"]) for c in plan.tool_calls] == [("generate_music", "robot_song")]


def test_launch_text_takes_the_route_phrase_of_code():
    assert launch_text("Включаю музыку.", CACHE) == CACHE["source_message"]
    note = {"source_note": "Сгенерировать сейчас не могу: генерация у меня ещё не подключена, играю своё."}
    assert launch_text("Включаю музыку.", note) == f"{note['source_note']} Включаю музыку."


def test_decline_phrase_survives_the_transport():
    assert source_phrase(source_error(DECLINE)) == DECLINE
    assert source_phrase(f"Ошибка: {source_error(DECLINE)}") == DECLINE
    assert source_phrase("music: compose_error") == ""


def test_llm_turn_says_the_decline_phrase_not_a_generic_failure():
    turn = MusicTurn()
    turn.record("generate_music", {"kind": "robot_song"}, is_error=True, content=source_error(DECLINE))
    assert turn.phrase() == DECLINE
    ok = MusicTurn()
    ok.record("request_music", {"intent": "track"}, is_error=False, content=repr(CACHE))
    assert ok.phrase() == CACHE["source_message"]


def test_router_cache_track_is_confirmed_by_started_of_its_deck_launch():
    """ADR-0158 PR-2: трек кэша играет на деке — роутер ждёт его ``started`` (как у синтеза), без него — не успех."""
    plan = MediaRouter().route(LIVE, MediaState())
    events = MusicEventLog()
    events.observe_json(build_music_event_payload("started", "gen_c0fe_1", ts=1.0))

    async def cached(call):
        return True, repr(CACHE)

    async def declined(call):
        return False, source_error(DECLINE)

    assert asyncio.run(run_media_plan(plan, cached, events))[:2] == (True, CACHE["source_message"])
    assert asyncio.run(run_media_plan(plan, declined, events))[:2] == (False, DECLINE)
