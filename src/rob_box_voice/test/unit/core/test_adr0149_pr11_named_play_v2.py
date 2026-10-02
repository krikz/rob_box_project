"""ADR-0149 PR-11 — заказ мелодии по имени играет движок v2 (эпик #3312).

Поиск ``lookup_melody`` и правило промаха (реплика в LLM), играет ``request_music(intent=melody)`` движка v2;
старый шаг ``compose_music`` удалён в PR-13a. Без ROS: тулы — фейк.
"""

from __future__ import annotations

import asyncio

from rob_box_voice.core.media_router import MediaRouter, MediaState
from rob_box_voice.core.named_play import NamedPlayStatus, run_named_play

HIT = "{'name': 'kalinkav_2', 'title': 'Kalinka V2.0', 'match': {'unmatched': [], 'ignored': []}}"
MISS = "{'name': 'national', 'title': 'Soviet Anthem', 'match': {'unmatched': [], 'ignored': ['германии']}}"


def test_router_sends_play_named_to_the_named_play_flow():
    plan = MediaRouter().route("робот поставь калинка", MediaState())
    assert plan is not None and plan.play_name == "калинка"
    assert plan.tool_calls == ()  # заказ исполняет поток named_play, не план тулов


def _executor(lookup_content, play_ok=True):
    calls = []

    async def execute(name, args):
        calls.append((name, args))
        if name == "lookup_melody":
            return True, lookup_content
        return play_ok, "{'ok': True}" if play_ok else "мелодия не заиграла: not_started"

    return execute, calls


def test_found_melody_is_played_by_the_engine_tool():
    tool, args = "request_music", {"intent": "melody", "text": "калинка"}
    execute, calls = _executor(HIT)
    outcome = asyncio.run(run_named_play(execute, "калинка"))
    assert outcome.status is NamedPlayStatus.PLAYED and outcome.tools_done == ("lookup_melody", tool)
    assert calls == [("lookup_melody", {"name": "калинка"}), (tool, args)]


def test_v2_miss_goes_to_llm_without_touching_the_player():
    execute, calls = _executor(MISS)
    outcome = asyncio.run(run_named_play(execute, "гимн германии"))
    assert outcome.status is NamedPlayStatus.MISS and [c[0] for c in calls] == ["lookup_melody"]


def test_v2_found_but_not_started_is_an_honest_failure():
    execute, _calls = _executor(HIT, play_ok=False)
    outcome = asyncio.run(run_named_play(execute, "калинка"))
    assert outcome.status is NamedPlayStatus.FAILED and "request_music" in outcome.reason
