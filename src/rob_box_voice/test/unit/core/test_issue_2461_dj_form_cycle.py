"""
test_issue_2461_dj_form_cycle.py — DJModeController.tick() не должен
диспатчить переход раньше конца текущего прохода формы (issue #2461).

Контекст: до этого фикса единственным каналом «форма ещё не доиграла»
было текстовое сообщение ``compose_music``, которое ЛЛМ должна была
прочитать и вручную перенести в ``next_transition_sec``. Модель регулярно
промахивалась (ставила 45с на форму 96-190с) — переход срезал дроп и
кульминацию на каждом сете (30 часов живого лога, ни одного дропа).

Структурный канал (issue #2461, части 1-2 уже в develop): mcp_server
считает конец прохода формы в своём процессе (``MusicManager``,
``time.monotonic()``-based) и публикует его в dialogue_node как
АБСОЛЮТНОЕ epoch-время через ``/voice/music/form`` (см.
``mcp_server.publish_music_state`` и ``dialogue_node._on_music_form``).
``DJState.form_ends_at`` хранит это значение как получено — ``tick()``
сравнивает его со своим ``time.time()``.

Эти тесты бьют только по ``DJModeController.tick()`` — без ROS, без
dialogue_node — прямой импорт ``core/dj_mode.py`` по образцу
``test_dramaturgy_fix_1016.py``.
"""
from __future__ import annotations

import logging
import sys
import time
import unittest
from pathlib import Path

# Make the in-tree package importable without `pip install -e`.
ROOT = Path(__file__).resolve().parents[4]  # .../test/unit/core/
sys.path.insert(0, str(ROOT / "src" / "rob_box_voice"))

from rob_box_voice.core.dj_mode import DJModeController


class _StubHook:
    """Minimal DJHook-shaped stub — only what the controller reads."""

    def __init__(self) -> None:
        self.persona_default = "Роббокс"
        self.dispatch_calls: list = []
        self.on_stop = None

    def dispatch(self, prompt: str, is_auto: bool = False) -> None:  # noqa: ARG002
        self.dispatch_calls.append(prompt)

    def is_active(self) -> bool:
        return False

    def is_dialogue_active(self) -> bool:
        return False


def _build_controller() -> DJModeController:
    ctrl = DJModeController(hook=_StubHook(), logger=logging.getLogger("test"))
    ctrl.state.enabled = True
    return ctrl


# ── (a) tick() откладывает переход, пока форма не доиграла ────────────


class TestFormCycleGatesTick(unittest.TestCase):
    def test_tick_does_not_dispatch_before_form_cycle_ends(self) -> None:
        """Живой баг #2461: next_transition_sec=45 на форме, которая
        реально играет ещё 150с — next_transition_at уже в прошлом,
        form_ends_at ещё в будущем. tick() не должен диспатчить."""
        ctrl = _build_controller()
        now = time.time()
        ctrl.state.next_transition_at = now - 1.0
        ctrl.state.form_ends_at = now + 150.0

        ctrl.tick()

        self.assertEqual(
            ctrl._hook.dispatch_calls, [],
            "tick() не должен диспатчить переход раньше конца формы",
        )
        # Гейт по form_ends_at не должен «сжирать» исходный
        # next_transition_at — если следующее сообщение формы обнулит
        # form_ends_at, старое значение должно быть по-прежнему видно.
        self.assertEqual(ctrl.state.next_transition_at, now - 1.0)

    def test_tick_dispatches_once_form_cycle_has_ended(self) -> None:
        ctrl = _build_controller()
        now = time.time()
        ctrl.state.next_transition_at = now - 1.0
        ctrl.state.form_ends_at = now - 1.0  # форма закончилась секунду назад

        ctrl.tick()

        self.assertEqual(len(ctrl._hook.dispatch_calls), 1)

    def test_tick_still_respects_a_longer_manual_next_transition_sec(self) -> None:
        """next_transition_sec остаётся РУЧНЫМ ПЕРЕКРЫТИЕМ — форма не
        укорачивает интервал, который модель попросила удлинить."""
        ctrl = _build_controller()
        now = time.time()
        ctrl.state.next_transition_at = now + 200.0  # модель попросила больше
        ctrl.state.form_ends_at = now + 50.0  # форма закончится раньше

        ctrl.tick()

        self.assertEqual(
            ctrl._hook.dispatch_calls, [],
            "next_transition_at длиннее form_ends_at должен победить",
        )


# ── (b) при отсутствии/протухании значения — поведение прежнее ────────


class TestMissingOrStaleFormEndsAtFallsBackToOldBehaviour(unittest.TestCase):
    def test_tick_dispatches_on_next_transition_at_when_form_ends_at_is_none(self) -> None:
        """Топик ещё не пришёл (None) — старое поведение по
        next_transition_at, форма ничего не блокирует."""
        ctrl = _build_controller()
        now = time.time()
        ctrl.state.next_transition_at = now - 1.0
        ctrl.state.form_ends_at = None

        ctrl.tick()

        self.assertEqual(len(ctrl._hook.dispatch_calls), 1)

    def test_tick_ignores_a_stale_form_ends_at_in_the_past(self) -> None:
        """Протухшее значение (трек сменился, сообщение не обновилось)
        не должно держать DJ замороженным навсегда — не блокирует."""
        ctrl = _build_controller()
        now = time.time()
        ctrl.state.next_transition_at = now - 1.0
        ctrl.state.form_ends_at = now - 9999.0

        ctrl.tick()

        self.assertEqual(len(ctrl._hook.dispatch_calls), 1)


# ── Существующие защиты не сломаны новым гейтом ────────────────────────


class TestFormEndsAtDoesNotBreakExistingGuards(unittest.TestCase):
    def test_dialogue_active_still_postpones_after_form_gate_passes(self) -> None:
        ctrl = _build_controller()
        now = time.time()
        ctrl.state.next_transition_at = now - 1.0
        ctrl.state.form_ends_at = now - 1.0  # форма уже доиграла

        ctrl._hook.is_dialogue_active = lambda: True  # type: ignore[assignment]
        ctrl.tick()

        self.assertEqual(ctrl._hook.dispatch_calls, [])
        self.assertGreater(ctrl.state.next_transition_at, now)

    def test_auto_max_transitions_still_stops_dj_with_form_ends_at_set(self) -> None:
        """DJ_AUTO_MAX_TRANSITIONS (без плана) по-прежнему останавливает
        DJ, даже когда form_ends_at корректно взведён и уже в прошлом."""
        ctrl = _build_controller()
        now = time.time()
        ctrl.state.transition_count = DJModeController.DJ_AUTO_MAX_TRANSITIONS
        ctrl.state.next_transition_at = now - 1.0
        ctrl.state.form_ends_at = now - 1.0

        ctrl.tick()

        self.assertEqual(ctrl._hook.dispatch_calls, [])
        self.assertFalse(ctrl.state.enabled, "лимит переходов должен выключить DJ")

    def test_reset_state_clears_form_ends_at(self) -> None:
        ctrl = _build_controller()
        ctrl.state.form_ends_at = time.time() + 100.0

        ctrl.handle_message('{"enabled": false}')

        self.assertIsNone(ctrl.state.form_ends_at)


if __name__ == "__main__":
    unittest.main()
