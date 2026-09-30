"""Issue #3227 — материал из реплики (Strudel/RTTTL/ноты) → хук следующего DJ-перехода.

Живой лог 30.09: Шифу прислал в TG Strudel-код Stranger Things «для материала
продолжения сэта», робот повторил прежний seed=6261504 — ноты потерялись.
Приём идёт кодом: детектор → ``add_music_material`` → ``DJState.pending_material``
→ ``_club_call`` ближайшего перехода несёт ``name="user_..."``.
"""

from __future__ import annotations

import asyncio
import json
import logging
import re
from pathlib import Path
from types import SimpleNamespace

import pytest

from rob_box_voice.core.dj_material import (
    MATERIAL_TTL_S,
    MaterialIntake,
    looks_like_material,
)
from rob_box_voice.core.dj_mode import DJHook, DJModeController

START = 1_759_000_000.0
FIXTURE = (
    Path(__file__).resolve().parents[4]
    / "rob_box_mcp_tools" / "test" / "fixtures" / "strudel_stranger_things.txt"
)
NAME_RE = re.compile(r'compose_music\(style="club", name="(?P<name>user_[^"]+)", bpm=')


def _controller():
    clock = {"t": START}
    dispatched = []
    hook = DJHook(
        dispatch=lambda prompt, from_tick=False: dispatched.append(prompt),
        is_active=lambda: False,
        is_dialogue_active=lambda: False,
    )
    ctrl = DJModeController(hook=hook, logger=logging.getLogger("test"), clock=lambda: clock["t"])
    ctrl.handle_message(json.dumps({"enabled": True, "next_transition_sec": 45}))
    return ctrl, clock, dispatched


class FakeExecutor:
    """Отвечает как SchedulerToolExecutor: ``message`` + ``\\n`` + ``repr(data)``."""

    def __init__(self, name="user_stranger_things_intro_th_69bb5f", is_error=False):
        self.calls = []
        self._name = name
        self._is_error = is_error

    async def execute(self, call):
        self.calls.append(call)
        data = {"name": self._name, "added": True}
        return SimpleNamespace(content=f"Материал принят.\n{data!r}", is_error=self._is_error)


def _intake(ctrl, executor):
    return MaterialIntake(ctrl, lambda: executor, lambda: None, logging.getLogger("test"), clock=lambda: START)


# ── детектор ────────────────────────────────────────────────────────────


@pytest.mark.parametrize("text", [
    FIXTURE.read_text(encoding="utf-8"),
    'вот: note("c e g b")',
    'n("0 2 4").scale("c:major")',
    "Contra:d=4,o=6,b=285:a#5,a#5,c#",
    "setcpm(120)",
])
def test_detector_hits_material(text):
    assert looks_like_material(text)


@pytest.mark.parametrize("text", ["", "включи музыку", "сыграй что-нибудь в духе ноты", "n = 5", "не слышу, громче"])
def test_detector_ignores_chat(text):
    assert not looks_like_material(text)


# ── приём → DJ-состояние → _club_call ───────────────────────────────────


def test_next_transition_uses_received_material_then_it_is_consumed():
    ctrl, clock, dispatched = _controller()
    executor = FakeExecutor()
    name = asyncio.run(_intake(ctrl, executor).ingest(FIXTURE.read_text(encoding="utf-8"), executor))
    assert name == "user_stranger_things_intro_th_69bb5f"
    assert executor.calls[0].name == "add_music_material" and "Stranger Things" in executor.calls[0].arguments["text"]
    assert ctrl.state.pending_material == name

    clock["t"] += 60
    ctrl.tick()
    match = NAME_RE.search(dispatched[-1])
    assert match and match["name"] == name, dispatched[-1]

    # Трек сета запущен (ход DJ_AUTO с compose_music) — материал отыгран, дальше обычный вызов.
    assert ctrl.note_turn_tools(("compose_music",), ("compose_music",), is_dj_auto=True)
    assert ctrl.state.pending_material == ""
    before = len(dispatched)
    clock["t"] += 120
    ctrl.tick()
    assert len(dispatched) == before + 1, "второй переход не выстрелил"
    assert "user_" not in dispatched[-1]


def test_material_played_by_llm_in_same_turn_is_not_repeated_on_transition():
    ctrl, _clock, _d = _controller()
    executor = FakeExecutor()
    text = FIXTURE.read_text(encoding="utf-8")
    asyncio.run(_intake(ctrl, executor).ingest(text, executor))
    assert ctrl.state.pending_material
    # Ход юзера (не DJ_AUTO, без set_dj_mode) с материалом в тексте: LLM сыграла его сама.
    assert not ctrl.note_turn_tools(("compose_music",), ("compose_music",), turn_text=text)
    assert ctrl.state.pending_material == ""
    assert "user_" not in ctrl._club_call(1)  # заказ юзера держит переход — смотрим готовый вызов


def test_unrelated_user_track_keeps_pending_material():
    ctrl, _clock, _d = _controller()
    executor = FakeExecutor()
    asyncio.run(_intake(ctrl, executor).ingest('note("c e g b")', executor))
    ctrl.note_turn_tools(("compose_music",), ("compose_music",), turn_text="сыграй тему марио")
    assert ctrl.state.pending_material


def test_unlaunched_material_is_offered_again_but_expires():
    ctrl, clock, dispatched = _controller()
    executor = FakeExecutor()
    asyncio.run(_intake(ctrl, executor).ingest('note("c e g b")', executor))
    ctrl.state.pending_material_at = clock["t"]
    clock["t"] += 60
    ctrl.tick()
    assert NAME_RE.search(dispatched[-1])
    before = len(dispatched)
    clock["t"] += MATERIAL_TTL_S + 1
    ctrl.tick()
    assert len(dispatched) == before + 1 and "user_" not in dispatched[-1]


def test_failed_tool_leaves_state_untouched():
    ctrl, clock, dispatched = _controller()
    executor = FakeExecutor(is_error=True)
    assert asyncio.run(_intake(ctrl, executor).ingest("note(мусор)", executor)) is None
    assert ctrl.state.pending_material == ""
    before = len(dispatched)
    clock["t"] += 120
    ctrl.tick()
    assert len(dispatched) == before + 1, "второй переход не выстрелил"
    assert "user_" not in dispatched[-1]


def test_on_text_without_executor_or_material_does_nothing():
    ctrl, _clock, _d = _controller()
    assert _intake(ctrl, None).on_text('note("c e g b")') is False
    assert _intake(ctrl, FakeExecutor()).on_text("привет") is False
