"""ADR-0149 PR-6 — роутер медиакоманд движка v2 (эпик #3312, #3265).

Поведение, а не текст промпта: какая фраза → какой тул (v1-путь удалён в PR-13a), фраза об успехе только после
``started`` из ``/voice/music/event``, ``rejected`` → честный отказ (A14), ``<music_state>`` — из
latched-снимка плеера. Без ROS: исполнитель тулов и поток событий — фейки.
"""

from __future__ import annotations

import asyncio
import json
import threading

import pytest

from rob_box_voice.core import media_plan_run
from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_plan_run import run_media_plan
from rob_box_voice.core.media_router import (
    DJ_FAIL_TEXT,
    NOT_STARTED_TEXT,
    REQUEST_FAIL_TEXT,
    STOP_OK_TEXT,
    MediaRouter,
    MediaState,
)
from rob_box_voice.core.music_player_state import (
    MusicEventLog,
    build_music_event_payload,
    build_music_state_payload,
    parse_music_event,
    parse_music_state,
)
from rob_box_voice.core.music_state_prompt import MusicStateMemory

V2 = MediaRouter()


def _tools(plan):
    return [(c.name, c.arguments) for c in plan.tool_calls]


# ---------------------------------------------------------------------------
# Маршрутизация фраз: в dj_set / request_music без LLM
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("text,tool,args", [
    ("включи диджей сет на тему космос", "dj_set", {"action": "start", "theme": "космос"}),
    ("запусти диджей-сет про киберпанк", "dj_set", {"action": "start", "theme": "киберпанк"}),
    ("ты диджей Робокс на тему детский праздник", "dj_set",
     {"action": "start", "theme": "детский праздник", "persona": "диджей Робокс"}),
    ("ты диджей Снупдог", "dj_set", {"action": "start", "persona": "диджей Снупдог"}),
    ("включи диджей сет у нас сегодня славянская вечеринка", "dj_set",
     {"action": "start", "theme": "славянская вечеринка"}),
    ("включи диджей сет", "dj_set", {"action": "start"}),
    ("поставь клубный трек", "request_music", {"intent": "track", "text": "поставь клубный трек"}),
    ("включи музыку", "request_music", {"intent": "track", "text": "включи музыку"}),
])
def test_v2_routes_start_phrases_to_engine_tools_without_llm(text, tool, args):
    plan = V2.route(text, MediaState())
    assert plan is not None and plan.confirm_started
    assert _tools(plan) == [(tool, args)]


def test_v2_persona_does_not_swallow_the_theme_tail():
    command = parse_media_command("ты диджей Робокс на тему космос")
    assert command.set_persona == "диджей Робокс" and command.set_theme == "космос"
    assert command.persona == "диджей Робокс на тему космос" and command.closed


@pytest.mark.parametrize("text", [
    "включи свет", "поставь что-то", "сыграй что-нибудь весёлое", "включи звук",
    "поставь музыку про зайчиков", "поставь музыку погромче",
])
def test_not_a_generic_music_request(text):
    assert parse_media_command(text).intent is not MediaIntent.REQUEST_MUSIC


def test_v2_classic_melody_stays_on_the_old_named_path():
    plan = V2.route("поставь к элизе", MediaState())
    assert plan.play_name == "к элизе" and plan.tool_calls == ()  # named_play → request_music (PR-11)


def test_v2_open_dj_request_goes_to_llm():
    assert V2.route("ты диджей Снупдог и сыграй Still Dre и Next Episode", MediaState()) is None


def test_v2_stop_stops_engine_deck_and_old_player():
    plan = V2.route("выключи музыку", MediaState(music_playing=True))
    assert _tools(plan) == [("dj_set", {"action": "stop"}), ("stop_music", {})]
    assert plan.say_ok == STOP_OK_TEXT and not plan.confirm_started


# ---------------------------------------------------------------------------
# Фраза — только по событию плеера (A14)
# ---------------------------------------------------------------------------


class _Tools:
    def __init__(self, result, ok=True):
        self.calls, self._result, self._ok = [], result, ok

    async def __call__(self, call):
        self.calls.append(call.name)
        return self._ok, "Сет начат\n" + repr(self._result)


def _run(plan, tools, events):
    return asyncio.run(run_media_plan(plan, tools, events))


def _event(name, track_id, **fields):
    return build_music_event_payload(name, track_id, ts=1.0, **fields)


@pytest.fixture
def fast_wait(monkeypatch):
    monkeypatch.setattr(media_plan_run, "STARTED_WAIT_S", 0.2)


def test_success_phrase_only_after_started(fast_wait):
    plan = V2.route("включи диджей сет на тему космос", MediaState())
    events = MusicEventLog()
    tools = _Tools({"ok": True, "track_id": "set1:01:A:ab"})
    timer = threading.Timer(0.05, events.observe_json, [_event("started", "set1:01:A:ab", phase_in_form=0.0)])
    timer.start()
    ok, phrase, done = _run(plan, tools, events)
    timer.join()
    assert ok and phrase == "Включаю диджей-сет. Тема — космос." and done == ["dj_set"]


def test_started_before_tool_answer_is_not_lost(fast_wait):
    plan = V2.route("поставь клубный трек", MediaState())
    events = MusicEventLog()
    events.observe_json(_event("started", "req1:02:A:cd"))  # событие обогнало ответ тула
    ok, phrase, _ = _run(plan, _Tools({"ok": True, "track_id": "req1:02:A:cd"}), events)
    assert ok and phrase == "Включаю клубный трек."


def test_rejected_gives_honest_refusal_and_no_success_phrase(fast_wait):
    plan = V2.route("включи диджей сет на тему космос", MediaState())
    events = MusicEventLog()
    events.observe_json(_event("rejected", "set1:01:A:ab", reason="server_fail", detail="/s_new not found"))
    ok, phrase, _ = _run(plan, _Tools({"ok": True, "track_id": "set1:01:A:ab"}), events)
    assert not ok and phrase == DJ_FAIL_TEXT


def test_other_tracks_started_does_not_count(fast_wait):
    plan = V2.route("поставь клубный трек", MediaState())
    events = MusicEventLog()
    events.observe_json(_event("started", "чужой:01:A:00"))
    ok, phrase, _ = _run(plan, _Tools({"ok": True, "track_id": "req1:02:A:cd"}), events)
    assert not ok and phrase == NOT_STARTED_TEXT


def test_failed_tool_says_fail_without_waiting(fast_wait):
    plan = V2.route("поставь клубный трек", MediaState())
    ok, phrase, done = _run(plan, _Tools({"ok": False, "reason": "not_started"}, ok=False), MusicEventLog())
    assert not ok and phrase == REQUEST_FAIL_TEXT and done == []


def test_no_event_stream_means_no_success_phrase(fast_wait):
    plan = V2.route("включи диджей сет", MediaState())
    ok, phrase, _ = _run(plan, _Tools({"ok": True, "track_id": "set1:01:A:ab"}), None)
    assert not ok and phrase == NOT_STARTED_TEXT


def test_event_parser_ignores_garbage_and_unknown_events():
    assert parse_music_event("не json") is None
    assert parse_music_event(json.dumps({"event": "boom", "track_id": "x"})) is None
    ev = parse_music_event(_event("rejected", "t1", reason="exec_error"))
    assert ev.event == "rejected" and ev.track_id == "t1" and ev.fields["reason"] == "exec_error"


# ---------------------------------------------------------------------------
# <music_state> — из latched-снимка
# ---------------------------------------------------------------------------


def test_music_state_tag_is_built_from_the_v2_snapshot():
    dj = {"enabled": True, "set_id": "set42", "track_no": 3, "bpm": 132, "theme": "космос",
          "persona": "диджей Робокс", "title": "космос · трек 3"}
    snap = parse_music_state(build_music_state_payload(playing=True, track_id="set42:03:A:9f", dj=dj, ts=100.0))
    memory = MusicStateMemory()
    memory.observe_state(snap, now=100.0)
    tag = memory.render(now=101.0)
    assert 'playing="yes"' in tag and 'track="космос · трек 3"' in tag and 'dj="on"' in tag
    assert 'set_theme="космос"' in tag and 'set_track_no="3"' in tag and 'bpm="132"' in tag


def test_music_state_tag_without_set_fields():
    snap = parse_music_state(build_music_state_payload(playing=True, track_id="t1", dj=False, ts=100.0))
    memory = MusicStateMemory()
    memory.observe_state(snap, now=100.0)
    tag = memory.render(now=101.0)
    assert 'track="без названия"' in tag and "set_theme" not in tag and "bpm=" not in tag


@pytest.mark.parametrize("text,mood", [
    ("включи музыку для танцев", "groove"),
    ("Робот включи музыку для танцев", "groove"),
    ("поставь музыку для работы", "calm"),
    ("включи музыку для сна", "calm"),
    ("поставь музыку для тренировки", "epic"),
])
def test_music_for_occasion_is_request_music_with_mood(text, mood):
    from rob_box_music import knowledge
    command = parse_media_command(text)
    assert command.intent is MediaIntent.REQUEST_MUSIC and command.closed and command.mood == mood
    assert mood in knowledge.MOOD_ENERGY
    plan = V2.route(text, MediaState())
    assert [c.name for c in plan.tool_calls] == ["request_music"]
    assert plan.tool_calls[0].arguments["mood"] == mood and plan.confirm_started


@pytest.mark.parametrize("text,name", [
    ("поставь калинку", "калинку"),
    ("включи песню калинка", "калинка"),
    ("поставь к элизе", "к элизе"),
])
def test_named_play_is_unchanged_by_occasion_rule(text, name):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.PLAY_NAMED and command.name == name and command.mood == ""


@pytest.mark.parametrize("text", ["включи музыку для Маши", "включи музыку для танцев на тему космос"])
def test_unknown_occasion_is_not_request_music(text):
    assert parse_media_command(text).intent is not MediaIntent.REQUEST_MUSIC
