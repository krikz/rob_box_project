#!/usr/bin/env python3
"""Офлайн-прогон фраз через роутер медиакоманд v2 (ADR-0149 PR-6): решение и задержка до вызова тула.

Без ROS и без звука: фраза → ``strip_wake_word`` (как приём STT) → ``MediaRouter().route``
→ тул движка v2 (``dj_set``/``request_music``) с настоящими ``compose``/``render`` и фейковым
владельцем плеера (``play`` принимает программу сразу). Это НЕ замер A2 на роботе: CPU хоста,
нет Renardo, нет старта на границе такта (Clock.latency 0.5 с + до такта ≈ 1.9 с при 130 BPM).

Запуск из корня репо (пакеты своего чекаута):
    PYTHONUTF8=1 PYTHONPATH="src/rob_box_music;src/rob_box_core;src/rob_box_voice;src/rob_box_harness;\
src/rob_box_mcp_tools" python scripts/music/live_dj/router_offline.py [фраза ...]
"""

from __future__ import annotations

import argparse
import json
import time
from types import SimpleNamespace

from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, RequestMusicTool
from rob_box_voice.core.dialogue_text import strip_wake_word
from rob_box_voice.core.media_router import MediaRouter, MediaState

PHRASES = (
    "Робот включи диджей сет на тему космос",
    "Робот запусти диджей сет про киберпанк",
    "Робот ты диджей Робокс на тему детский праздник",
    "Робот включи диджей сет у нас сегодня славянская вечеринка",
    "Робот ты диджей Снупдог",
    "Робот включи диджей сет",
    "Робот поставь клубный трек",
    "Робот включи музыку",
    "Робот поставь техно",
    "Робот поставь калинку",
)


class _Owner:
    """Владелец плеера, который принимает любую программу (нужен только путь до вызова плеера)."""

    on_started = None

    def __init__(self) -> None:
        self.played = []

    def play(self, program, dj=None):
        self.played.append(program)
        return {"ok": True, "track_id": program.track_id}

    def stop(self, reason="user_stop"):
        return {"ok": True, "track_id": None}

    def reject(self, track_id, reason, detail):
        return {"ok": False, "track_id": track_id, "reason": reason, "detail": detail}


def _tools(owner):
    dj = DjSetTool(None, owner, melodies=lambda ids: {}, seed=lambda: int(time.time()))
    return {"dj_set": dj, "request_music": RequestMusicTool(None, owner, dj, melodies=lambda ids: {})}


def run(phrase: str) -> dict:
    owner = _Owner()
    tools = _tools(owner)
    t0 = time.perf_counter()
    text = strip_wake_word(phrase)
    plan = MediaRouter().route(text, MediaState())
    t_route = time.perf_counter() - t0
    row = {"phrase": phrase, "route_ms": round(t_route * 1000, 2)}
    if plan is None:
        return {**row, "decision": "LLM (роутер не взял)"}
    calls = [(c.name, c.arguments) for c in plan.tool_calls]
    row.update(decision=plan.command.intent.value, tools=calls, play_name=plan.play_name or None,
               say_on_started=plan.say_ok if plan.confirm_started else None)
    if not calls or calls[0][0] not in tools:
        return row
    name, args = calls[0]
    result = tools[name].execute(**args)
    t_tool = time.perf_counter() - t0
    program = owner.played[-1] if owner.played else SimpleNamespace(track_id=None, bpm=None)
    return {**row, "to_player_ms": round(t_tool * 1000, 1), "ok": result.success,
            "track_id": program.track_id, "bpm": program.bpm}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("phrases", nargs="*")
    opts = parser.parse_args()
    for phrase in opts.phrases or PHRASES:
        print(json.dumps(run(phrase), ensure_ascii=False))


if __name__ == "__main__":
    main()
