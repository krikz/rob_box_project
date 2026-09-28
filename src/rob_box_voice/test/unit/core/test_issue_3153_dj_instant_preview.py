"""Issue #3153 — «ты диджей X» в тишине: первый звук сразу, а не через 50 с.

Живой прогон 28.09.2026 (деплой develop ``82c973e62``): роутер (#3134)
включал DJ с переходом через 15 с; переход #1 «СТАРТ ВЕЧЕРИНКИ» гнал модель
в search_web / search_samples / gen_search_library, план и compose_music ×3
— первый звук через ~50 с после фразы «запускаю сет».

Контракт после фикса (ADR-0142 «мгновенное превью»):

* роутер в ТИШИНЕ: ``compose_music(style="club", repeat=True, seed=<свой>)``
  первым, затем ``set_dj_mode(..., next_transition_sec=DJ_PREVIEW_FORM_SEC)``;
* превью не встало — DJ не включается, фраза честная; ``set_dj_mode`` не
  прошёл после превью — тоже честно («музыку включил, а сет нет»);
* сет уже идёт / над играющим треком / открытый запрос — как раньше;
* превью — трек #1 сета: переход #1 — обычный переход к треку #2, без
  исследования и без «СТАРТ ВЕЧЕРИНКИ».

Здесь — чистые части (роутер, DJ-контроллер); адаптер ноды — в
``test/unit/node/test_issue_3153_dj_instant_preview_node.py``.
"""

from __future__ import annotations

import json
import logging
import random

import pytest

from rob_box_voice.core.dj_mode import CLUB_ROOTS, DJModeController
from rob_box_voice.core.media_router import (
    DJ_MODE_FAIL_AFTER_PREVIEW_TEXT,
    DJ_PREVIEW_FAIL_TEXT,
    DJ_PREVIEW_FORM_SEC,
    DJ_START_TRANSITION_SEC,
    MediaRouter,
    MediaState,
)

QUIET = MediaState()
PLAYING = MediaState(music_playing=True, track_name="В пещере горного короля")
DJ_SET = MediaState(music_playing=True, dj_enabled=True, track_name="Still Dre")

RESEARCH_TOOLS = ("search_web(", "search_samples(", "gen_search_library(")
T0 = 1_800_000_000.0


def _names(plan):
    return [c.name for c in plan.tool_calls]


# ── 1. Роутер: порядок тулов по состоянию ──────────────────────────────


def test_silence_starts_preview_then_dj_mode() -> None:
    plan = MediaRouter(rng=random.Random(7)).route("ты диджей Снупдог, давай сет", QUIET)
    assert plan.handled and plan.cancel_inflight
    assert _names(plan) == ["compose_music", "set_dj_mode"]
    compose, dj = plan.tool_calls
    assert compose.arguments["style"] == "club"
    assert compose.arguments["repeat"] is True
    assert compose.arguments["bpm"] == 124  # темп сета по умолчанию, #3113
    assert compose.arguments["seed"] >= 1  # seed=0 — эталон, не превью
    assert compose.arguments["root"] in CLUB_ROOTS
    assert dj.arguments == {
        "enabled": True,
        "persona": "диджей Снупдог",
        "next_transition_sec": DJ_PREVIEW_FORM_SEC,
    }
    assert plan.preview_root == compose.arguments["root"]
    assert plan.say_ok == "Я диджей Снупдог, запускаю сет."


def test_each_set_gets_its_own_preview_seed() -> None:
    router = MediaRouter(rng=random.Random(1))
    seeds = {
        router.route("ты диджей Снупдог", QUIET).tool_calls[0].arguments["seed"]
        for _ in range(5)
    }
    assert len(seeds) == 5


def test_preview_failure_texts_are_honest() -> None:
    plan = MediaRouter().route("ты диджей Снупдог", QUIET)
    compose, dj = plan.tool_calls
    # Превью не встало — DJ не включаем, о сете не врём.
    assert compose.fail_text == DJ_PREVIEW_FAIL_TEXT
    assert "не включил" in DJ_PREVIEW_FAIL_TEXT
    # Превью играет, set_dj_mode не прошёл — музыка есть, сета нет.
    assert dj.fail_text == DJ_MODE_FAIL_AFTER_PREVIEW_TEXT
    assert "запускаю" not in DJ_MODE_FAIL_AFTER_PREVIEW_TEXT


def test_over_playing_track_no_compose_but_claims_track_one() -> None:
    """Issue #3153 (доп.) — играющий трек не получает свой compose_music
    (он уже звучит), но заявка «трек #1 сета» всё равно уходит — без
    тоники, роутер её не знает."""
    plan = MediaRouter().route("ты диджей Снупдог, давай сет", PLAYING)
    assert _names(plan) == ["set_dj_mode"]
    assert plan.tool_calls[0].arguments["next_transition_sec"] == DJ_START_TRANSITION_SEC
    assert plan.preview_root == ""
    assert plan.claim_track_one is True


def test_over_playing_track_transition_falls_back_without_form(monkeypatch) -> None:
    import rob_box_voice.core.media_router as media_router_module

    monkeypatch.setattr(media_router_module.time, "time", lambda: T0)
    state = MediaState(
        music_playing=True, track_name="В пещере горного короля",
        form_ends_at=T0 + 40.0,
    )
    plan = MediaRouter().route("ты диджей Снупдог, давай сет", state)
    assert plan.tool_calls[0].arguments["next_transition_sec"] == 40

    no_form_state = MediaState(
        music_playing=True, track_name="В пещере горного короля"
    )
    plan = MediaRouter().route("ты диджей Снупдог, давай сет", no_form_state)
    assert plan.tool_calls[0].arguments["next_transition_sec"] == DJ_START_TRANSITION_SEC


def test_over_playing_track_first_transition_skips_research() -> None:
    """Играющий трек засчитан треком #1 — переход #1 без «СТАРТ ВЕЧЕРИНКИ»."""
    ctrl, hook, clock = _controller()
    ctrl.claim_preview("")  # тоника неизвестна, как заявка роутера
    ctrl.handle_message(json.dumps({
        "enabled": True,
        "persona": "диджей Снупдог",
        "next_transition_sec": DJ_START_TRANSITION_SEC,
    }))
    assert ctrl.state.tracks_started == 1
    assert ctrl.state.preview_started
    prompt = _tick_until_dispatch(ctrl, hook, clock)
    assert prompt.startswith("[DJ_AUTO переход #1]")
    assert "СТАРТ ВЕЧЕРИНКИ" not in prompt
    for tool in RESEARCH_TOOLS:
        assert tool not in prompt, tool


def test_running_set_only_switches_persona() -> None:
    plan = MediaRouter().route("ты диджей Снупдог", DJ_SET)
    assert plan.tool_calls[0].arguments == {"enabled": True, "persona": "диджей Снупдог"}
    assert _names(plan) == ["set_dj_mode"]
    assert plan.preview_root == ""


def test_open_request_in_silence_leaves_music_to_llm() -> None:
    plan = MediaRouter().route(
        "Ты диджей PAUL OAKENFOLD, сыграй Still Dre и Next Episode на 10 минут", QUIET
    )
    assert plan.to_llm
    assert _names(plan) == ["set_dj_mode"]
    assert plan.preview_root == ""


def test_preview_form_length_matches_club_form() -> None:
    """32 такта club при 124 BPM — ``club_arranger.club_duration_seconds``."""
    assert abs(DJ_PREVIEW_FORM_SEC - 32 * 4 * 60.0 / 124) < 1.0


# ── 2. DJ-контроллер: превью — трек #1, переход #1 без исследования ────


class _Clock:
    def __init__(self, now: float) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


class _Hook:
    persona_default = "Роббокс"
    on_stop = None

    def __init__(self) -> None:
        self.dispatches: list = []

    def dispatch(self, prompt: str, from_tick: bool = False) -> None:  # noqa: ARG002
        self.dispatches.append(prompt)

    def is_active(self) -> bool:
        return False

    def is_dialogue_active(self) -> bool:
        return False


def _controller():
    clock = _Clock(T0)
    hook = _Hook()
    ctrl = DJModeController(hook=hook, logger=logging.getLogger("t"), clock=clock)
    return ctrl, hook, clock


def _enable(ctrl) -> None:
    ctrl.handle_message(json.dumps({
        "enabled": True,
        "persona": "диджей Снупдог",
        "next_transition_sec": DJ_PREVIEW_FORM_SEC,
    }))


def _tick_until_dispatch(ctrl, hook, clock, horizon_s: float = 600.0) -> str:
    while not hook.dispatches and clock.now < T0 + horizon_s:
        clock.now += DJModeController.DJ_TICK_INTERVAL_S
        ctrl.tick()
    assert hook.dispatches, "переход #1 не случился"
    return hook.dispatches[0]


def test_claimed_preview_counts_as_track_one() -> None:
    ctrl, _hook, _clock = _controller()
    ctrl.claim_preview("F")
    _enable(ctrl)
    assert ctrl.state.tracks_started == 1
    assert ctrl.state.preview_started
    assert ctrl.state.set_root == "F"


def test_first_transition_after_preview_does_not_research() -> None:
    ctrl, hook, clock = _controller()
    ctrl.claim_preview("F")
    _enable(ctrl)
    prompt = _tick_until_dispatch(ctrl, hook, clock)
    # Переход #1 — на конце формы превью, а не через 15 с.
    assert clock.now - T0 >= DJ_PREVIEW_FORM_SEC
    assert "СТАРТ ВЕЧЕРИНКИ" not in prompt
    assert prompt.startswith("[DJ_AUTO переход #1]")
    for tool in RESEARCH_TOOLS:
        assert tool not in prompt, tool  # ни одного вызова ресёрча в промпте
    assert "НЕ исследуй" in prompt
    # Играет трек #2 сета: его сид и тоника — квинта от тоники превью (KEY_WALK).
    assert 'compose_music(style="club", bpm=124, root="C"' in prompt
    assert f"seed={ctrl._track_seed(2)}" in prompt
    # Свободный текст на этом переходе не озвучивается — диджей уже представился.
    assert ctrl.suppresses_free_text(ctrl.state.transition_count)


def test_waits_for_real_form_end_of_preview() -> None:
    ctrl, hook, clock = _controller()
    ctrl.claim_preview("F")
    _enable(ctrl)
    ctrl.state.form_ends_at = T0 + 90.0  # /voice/music/form: форма длиннее оценки
    _tick_until_dispatch(ctrl, hook, clock)
    assert clock.now >= T0 + 90.0


def test_without_preview_start_of_party_is_unchanged() -> None:
    ctrl, hook, clock = _controller()
    _enable(ctrl)
    prompt = _tick_until_dispatch(ctrl, hook, clock)
    assert prompt.startswith("[DJ_AUTO — СТАРТ ВЕЧЕРИНКИ]")
    assert ctrl.state.tracks_started == 0
    assert not ctrl.suppresses_free_text(1)


@pytest.mark.parametrize("spoil", ["dropped", "stale"])
def test_dropped_or_stale_claim_is_not_taken(spoil: str) -> None:
    ctrl, _hook, clock = _controller()
    ctrl.claim_preview("F")
    if spoil == "dropped":
        ctrl.drop_preview_claim()
    else:
        clock.now += DJModeController.PREVIEW_CLAIM_TTL_S + 1.0
    _enable(ctrl)
    assert ctrl.state.tracks_started == 0
    assert not ctrl.state.preview_started


def test_preview_flag_does_not_leak_into_next_set() -> None:
    ctrl, _hook, _clock = _controller()
    ctrl.claim_preview("F")
    _enable(ctrl)
    ctrl.handle_message(json.dumps({"enabled": False}))
    _enable(ctrl)
    assert not ctrl.state.preview_started
    assert ctrl.state.tracks_started == 0
