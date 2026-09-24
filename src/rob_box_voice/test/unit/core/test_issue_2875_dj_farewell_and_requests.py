"""test_issue_2875_dj_farewell_and_requests.py — дополнение к #2875.

Живой прогон 23.09.2026 17:52–17:55 (образ 6fabd2c1):

1. Прощание прозвучало В НАЧАЛЕ финального трека: переход #6 «ФИНАЛЬНЫЙ
   ТРЕК» → compose_music(repeat=false, форма ~250 с) → через 2 с
   set_dj_mode(enabled=false) → сразу «Вечеринка подошла к концу…»; модель
   ещё и сказала прощание через speak_text.
2. Просьба юзера посреди сета («сыграй тему марио…», 17:52:11) — Марио
   заиграл, а переход #6 в 17:54:52 его заменил.

Контракт после фикса:

* выключение DJ, пока форма играет → прощание после конца формы (не дольше
  ``FAREWELL_MAX_DEFER_S``); остановка музыки → прощание сразу;
* промпт финального трека запрещает speak_text;
* музыкальная просьба юзера посреди сета не расходует трек плана и
  откладывает следующий переход до конца своей формы (вариант «сет ждёт
  заказ», обоснование — ``DJModeController._hold_for_user_track``).
"""
from __future__ import annotations

import json
import logging

from rob_box_voice.core.dj_mode import DJModeController

T0 = 1_800_000_000.0
MUSIC = frozenset({"compose_music"})
PLAN = "Трек 1: Still Dre\nТрек 2: Next Episode\nТрек 3: Drop It Like It's Hot"
TICK = DJModeController.DJ_TICK_INTERVAL_S


class _Clock:
    def __init__(self, now: float) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


class _Hook:
    def __init__(self, clock: _Clock) -> None:
        self.persona_default = "Роббокс"
        self.dispatches: list = []
        self.farewells: list = []  # (time, persona)
        self.on_stop = lambda persona: self.farewells.append((clock.now, persona))

    def dispatch(self, prompt: str, from_tick: bool = False) -> None:  # noqa: ARG002
        self.dispatches.append(prompt)

    def is_active(self) -> bool:
        return False

    def is_dialogue_active(self) -> bool:
        return False


def _controller(payload: dict):
    clock = _Clock(T0)
    hook = _Hook(clock)
    ctrl = DJModeController(hook=hook, logger=logging.getLogger("t"), clock=clock)
    ctrl.handle_message(json.dumps({"enabled": True, **payload}))
    return ctrl, hook, clock


def _tick_for(ctrl, clock, seconds: float) -> None:
    end = clock.now + seconds
    while clock.now < end:
        clock.now += TICK
        ctrl.tick()


# ── 1. Прощание после конца финальной формы ──────────────────────────


def test_stop_during_final_form_defers_farewell_to_form_end() -> None:
    ctrl, hook, clock = _controller({"persona": "диджей Снупдог"})
    ctrl.state.form_ends_at = clock.now + 250.0  # финальная форма играет

    ctrl.handle_message(json.dumps({"enabled": False}))
    assert hook.farewells == [], "прощание не должно звучать в начале трека"

    _tick_for(ctrl, clock, 240.0)
    assert hook.farewells == []

    _tick_for(ctrl, clock, 15.0)
    assert len(hook.farewells) == 1
    said_at, persona = hook.farewells[0]
    assert T0 + 250.0 <= said_at <= T0 + 250.0 + TICK
    assert persona == "диджей Снупдог"


def test_farewell_is_immediate_when_music_stops_early() -> None:
    """«Хватит» + stop_music: mcp_server шлёт form_ends_at=null → сразу."""
    ctrl, hook, clock = _controller({})
    ctrl.state.form_ends_at = clock.now + 250.0
    ctrl.handle_message(json.dumps({"enabled": False}))

    clock.now += 3.0
    ctrl.state.form_ends_at = None  # /voice/music/form: музыка остановлена
    ctrl.tick()

    assert len(hook.farewells) == 1
    assert hook.farewells[0][0] == T0 + 3.0


def test_farewell_is_immediate_without_a_playing_form() -> None:
    ctrl, hook, clock = _controller({})
    ctrl.state.form_ends_at = clock.now - 1.0  # форма уже доиграла

    ctrl.handle_message(json.dumps({"enabled": False}))

    assert len(hook.farewells) == 1


def test_farewell_deferral_is_capped() -> None:
    ctrl, hook, clock = _controller({})
    ctrl.state.form_ends_at = clock.now + 3600.0  # протухшее/ошибочное

    ctrl.handle_message(json.dumps({"enabled": False}))
    _tick_for(ctrl, clock, DJModeController.FAREWELL_MAX_DEFER_S + TICK)

    assert len(hook.farewells) == 1
    assert hook.farewells[0][0] - T0 <= DJModeController.FAREWELL_MAX_DEFER_S + TICK


def test_repeated_disable_keeps_the_pending_farewell_once() -> None:
    ctrl, hook, clock = _controller({})
    ctrl.state.form_ends_at = clock.now + 60.0
    ctrl.handle_message(json.dumps({"enabled": False}))
    ctrl.handle_message(json.dumps({"enabled": False}))  # модель повторила

    _tick_for(ctrl, clock, 70.0)

    assert len(hook.farewells) == 1


def test_new_session_cancels_the_pending_farewell() -> None:
    ctrl, hook, clock = _controller({})
    ctrl.state.form_ends_at = clock.now + 60.0
    ctrl.handle_message(json.dumps({"enabled": False}))

    ctrl.reset_silently()
    _tick_for(ctrl, clock, 70.0)

    assert hook.farewells == []


def test_fresh_set_cancels_the_previous_sets_pending_farewell() -> None:
    ctrl, hook, clock = _controller({})
    ctrl.state.form_ends_at = clock.now + 60.0
    ctrl.handle_message(json.dumps({"enabled": False}))

    ctrl.handle_message(json.dumps({"enabled": True}))
    _tick_for(ctrl, clock, 70.0)

    assert hook.farewells == []


def test_final_prompt_forbids_speak_text() -> None:
    ctrl, _, _ = _controller({"plan": PLAN})
    ctrl.state.tracks_started = 2

    prompt = ctrl.build_auto_prompt(3)

    assert "ФИНАЛЬНЫЙ ТРЕК" in prompt
    assert "НЕ вызывай speak_text" in prompt


# ── 2. Просьба юзера посреди сета ─────────────────────────────────────


def test_user_music_request_is_not_a_plan_track() -> None:
    ctrl, _, _ = _controller({"plan": PLAN})
    ctrl.note_turn_tools(["compose_music"], MUSIC, is_dj_auto=True)  # Трек 1

    counted = ctrl.note_turn_tools(["lookup_melody", "compose_music"], MUSIC)

    assert counted is False
    assert ctrl.state.tracks_started == 1
    assert 'name="Next Episode"' in ctrl.build_auto_prompt(2)


def test_user_music_request_holds_the_next_transition() -> None:
    """Переход, назначенный на «через 5 с», не заменит заказ юзера."""
    ctrl, hook, clock = _controller({"plan": PLAN, "next_transition_sec": 15})
    ctrl.state.next_transition_at = clock.now + 5.0

    ctrl.note_turn_tools(["compose_music"], MUSIC)  # «сыграй тему марио»
    _tick_for(ctrl, clock, 60.0)
    assert hook.dispatches == [], "до прихода формы заказа переход не стреляет"

    # mcp_server прислал конец формы заказа: 150 с от старта.
    ctrl.state.form_ends_at = T0 + 150.0
    _tick_for(ctrl, clock, 85.0)  # t = 145 с
    assert hook.dispatches == [], "форма заказа ещё играет"

    _tick_for(ctrl, clock, 10.0)  # t = 155 с
    assert len(hook.dispatches) == 1, "после конца формы сет продолжается"


def test_hold_never_shortens_an_already_later_transition() -> None:
    ctrl, _, clock = _controller({"next_transition_sec": 300})
    later = ctrl.state.next_transition_at

    ctrl.note_turn_tools(["compose_music"], MUSIC)

    assert ctrl.state.next_transition_at == later


def test_retry_of_a_dj_transition_is_a_set_track_not_a_request() -> None:
    """Bug-D ретрай DJ-перехода идёт с is_dj_auto=False, но с текстом перехода."""
    ctrl, _, _ = _controller({"plan": PLAN})

    counted = ctrl.note_turn_tools(
        ["compose_music"],
        MUSIC,
        turn_text="[Speaker:unknown] [DJ_AUTO — СТАРТ ВЕЧЕРИНКИ] Ты …",
    )

    assert counted is True
    assert ctrl.state.tracks_started == 1
