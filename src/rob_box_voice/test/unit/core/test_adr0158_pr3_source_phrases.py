"""ADR-0158 PR-3 — «сгенерируй …» решает код: роутер без LLM, фраза о пути и отказ — фразы кода (ADR-0148).

Живой прогон 13.10: «Робот, сгенерируй пиздатый музон своим голосом» уходила в LLM (грамматика не узнавала глагол),
та ставила синтез, и робот говорил «Включаю музыку.» — не сказав, что генерировать не умеет.
"""

from __future__ import annotations

import asyncio

from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_phrases import launch_text, source_error, source_phrase
from rob_box_voice.core.media_plan_run import SOUND_NODE, run_media_plan
from rob_box_voice.core.media_router import MediaRouter, MediaState
from rob_box_voice.core.music_turn import MusicTurn

LIVE = "Робот, сгенерируй пиздатый музон своим голосом"
DECLINE = "Спеть своим голосом сейчас не могу: генерация у меня ещё не подключена, а синтез не поёт."
CACHE = {"ok": True, "via": SOUND_NODE, "track_id": "c0fe", "playback": "published",
         "source_message": "Ставлю спетое раньше своим голосом: «Я робот (РОББОКС)»."}


def test_generate_verb_goes_to_request_music_without_llm():
    for text in (LIVE, "сгенерируй музыку", "давай сгенерируй трек про космос"):
        assert parse_media_command(text).intent is MediaIntent.REQUEST_MUSIC
    plan = MediaRouter().route(LIVE, MediaState())
    assert [(c.name, c.arguments["intent"]) for c in plan.tool_calls] == [("request_music", "track")]
    assert parse_media_command("спой песню про кота").intent is MediaIntent.NONE  # пение под бит — у LLM


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


def test_router_cache_track_has_no_started_event_and_is_not_a_false_failure():
    plan = MediaRouter().route(LIVE, MediaState())

    async def cached(call):
        return True, repr(CACHE)

    async def declined(call):
        return False, source_error(DECLINE)

    assert asyncio.run(run_media_plan(plan, cached))[:2] == (True, CACHE["source_message"])
    assert asyncio.run(run_media_plan(plan, declined))[:2] == (False, DECLINE)
