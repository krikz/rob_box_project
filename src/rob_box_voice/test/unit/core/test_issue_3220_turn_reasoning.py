"""Issue #3220 — какой ход думает, рассуждение не звучит, переход с запасом.

* политика ``turn_wants_reasoning``: DJ-переход по тику и заказ музыки —
  думают; обычный голосовой ход, стоп, ретраи (Bug B, синтетические) — нет;
* ``strip_thinking_blocks`` (последняя линия перед TTS) снимает рассуждение
  во всех формах, и planning-narration гуард #1882 не глушит из-за него
  нормальный ответ;
* ``finite_form_lead_s`` включает бюджет рассуждения: ход с thinking + ретрай
  + фейд укладываются до остановки конечного трека.
"""
from __future__ import annotations

import logging
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "src" / "rob_box_voice"))

from rob_box_voice.core.dialogue_guards import is_planning_narration  # noqa: E402
from rob_box_voice.core.dj_mode import (  # noqa: E402
    DJ_FADE_BARS,
    DJ_MIN_TRACK_PLAY_S,
    DJ_REASONING_BUDGET_S,
    DJ_TURN_BUDGET_S,
    DJModeController,
    finite_form_lead_s,
)
from rob_box_voice.core.speak_helpers import strip_thinking_blocks  # noqa: E402
from rob_box_voice.core.turn_reasoning import turn_wants_reasoning  # noqa: E402

DJ_PROMPT = "[DJ_AUTO — ПЕРЕХОД #3] Трек 3: кислотный бас. Вызови compose_music."


# ── политика ────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    ("kwargs", "expected", "why"),
    [
        (dict(is_dj_auto=True, dj_transition=True, is_synthetic=False, user_input=DJ_PROMPT),
         True, "DJ-переход по тику — время подумать есть"),
        (dict(is_dj_auto=True, dj_transition=False, is_synthetic=False, user_input=DJ_PROMPT),
         False, "ретрай Bug B — бюджет перехода уже тратится"),
        (dict(is_dj_auto=True, dj_transition=True, is_synthetic=True, user_input=DJ_PROMPT),
         False, "синтетический ретрай гуарда внутри DJ-хода"),
        (dict(is_dj_auto=False, dj_transition=False, is_synthetic=False,
              user_input="[Speaker:Саша] сыграй что-нибудь про осень",
              raw_user_command="сыграй что-нибудь про осень"),
         True, "заказ музыки — сочинение/подбор"),
        (dict(is_dj_auto=False, dj_transition=False, is_synthetic=False,
              user_input="давай грига", raw_user_command=None),
         True, "заказ по имени композитора (#2834)"),
        (dict(is_dj_auto=False, dj_transition=False, is_synthetic=False,
              user_input="выключи музыку", raw_user_command="выключи музыку"),
         False, "стоп — не сочинение"),
        (dict(is_dj_auto=False, dj_transition=False, is_synthetic=False,
              user_input="как тебя зовут", raw_user_command="как тебя зовут"),
         False, "обычный голосовой ход — латентность"),
        (dict(is_dj_auto=False, dj_transition=False, is_synthetic=True,
              user_input="[CRITICAL] retry", raw_user_command="сыграй про осень"),
         False, "синтетический ретрай хода юзера"),
    ],
)
def test_turn_wants_reasoning(kwargs, expected, why):
    assert turn_wants_reasoning(**kwargs) is expected, why


# ── рассуждение не звучит ───────────────────────────────────────────


@pytest.mark.parametrize(
    ("raw", "spoken"),
    [
        ("<think>The user wants autumn.</think>Держи осень!", "Держи осень!"),
        # срезано max_tokens посреди рассуждения — раньше звучало целиком
        ("<think>The user wants autumn. Call compose_music with", ""),
        ("Держи осень! <think>now return done", "Держи осень!"),
        # открывающий тег потерян
        ("The user wants autumn.</think>Держи осень!", "Держи осень!"),
        ("  Держи осень!  ", "Держи осень!"),
        ("", ""),
    ],
)
def test_strip_thinking_blocks_shapes(raw, spoken):
    assert strip_thinking_blocks(raw) == spoken


def test_reasoning_does_not_trip_planning_narration_mute():
    """#1882 глушит ответ, где назван тул; в рассуждении тулы называются всегда."""
    raw = "<think>Юзер просит клуб, вызову compose_music.</think>Готово, играю!"
    assert is_planning_narration(raw), "контроль: без вырезания ответ ушёл бы в mute"
    assert not is_planning_narration(strip_thinking_blocks(raw))


def test_truncated_reasoning_is_not_spoken_as_narration():
    raw = "<think>Юзер хочет осень. Вызову compose_music(style="
    assert strip_thinking_blocks(raw) == ""


# ── тайминг перехода ────────────────────────────────────────────────


def test_finite_form_lead_includes_reasoning_budget():
    fade_s = (DJ_FADE_BARS + 1) * 4 * 60.0 / 124
    assert finite_form_lead_s(124) == pytest.approx(
        DJ_REASONING_BUDGET_S + DJ_TURN_BUDGET_S + fade_s
    )
    assert DJ_REASONING_BUDGET_S >= 20.0, "верх оценки «+10-20 с на ход» (6901a14e)"


class _Clock:
    def __init__(self, now: float) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


class _Hook:
    persona_default = "Роббокс"
    on_stop = None

    def __init__(self, clock: _Clock) -> None:
        self._clock = clock
        self.dispatched: list = []

    def dispatch(self, prompt: str, from_tick: bool = False) -> None:
        self.dispatched.append((self._clock.now, from_tick))

    def is_active(self) -> bool:
        return False

    def is_dialogue_active(self) -> bool:
        return False


def test_reasoning_transition_lands_before_finite_track_stops():
    """Трек 200 с: переход стартует за ход с рассуждением + ретрай + фейд.

    Худший случай (рассуждение на весь бюджет, один ретрай Bug B, фейд с
    выравниванием на такт) — новый трек встаёт не позже остановки
    уходящего, то есть без тишины.
    """
    t0 = 1_790_611_553.0
    clock = _Clock(t0)
    hook = _Hook(clock)
    ctrl = DJModeController(hook=hook, logger=logging.getLogger("test"), clock=clock)
    ctrl.state.enabled = True
    ctrl.state.started_at = t0 - 600.0
    ctrl.state.set_bpm = 124
    stops_at = t0 + 200.0
    ctrl.state.next_transition_at = stops_at + 5.0
    ctrl.state.form_ends_at = stops_at
    ctrl.note_form_stop(stops_at)

    while clock.now <= stops_at:
        ctrl.tick()
        clock.now += 5.0

    assert hook.dispatched, "переход не случился"
    fired_at, from_tick = hook.dispatched[0]
    assert from_tick is True, "тик должен помечать свежий переход (thinking)"
    fade_s = (DJ_FADE_BARS + 1) * 4 * 60.0 / 124
    worst_new_track_at = fired_at + DJ_REASONING_BUDGET_S + DJ_TURN_BUDGET_S + fade_s
    assert worst_new_track_at <= stops_at + 5.0, (
        f"новый трек встанет через {worst_new_track_at - stops_at:.0f} с после "
        "остановки — тишина"
    )
    # без рассуждения переход стартовал бы на DJ_REASONING_BUDGET_S позже
    assert fired_at <= stops_at - DJ_TURN_BUDGET_S - fade_s - DJ_REASONING_BUDGET_S + 5.0
    assert fired_at >= t0 + DJ_MIN_TRACK_PLAY_S
