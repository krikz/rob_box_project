"""
test_issue_3113_dj_finite_form_transition.py — нет тишины между треками сета.

Живой прогон 28.09.2026 (issue #3113, коммент «Живой прогон»): Star Wars
собран с ``Clock.future(264, Clock.clear)`` при 112 bpm — звучит 141 с;
DJ-переход #2 сработал через 146 с (``next_transition_at``/конец формы),
модель запустила следующий трек ещё через 10 с — около 15 с тишины.

Фикс: mcp_server публикует ``stops_at`` (когда КОНЕЧНЫЙ трек замолчит),
``DJModeController`` зовёт переход за ``finite_form_lead_s(bpm)`` до этого
момента (ход модели + фейд), один раз на форму, не раньше
``DJ_MIN_TRACK_PLAY_S`` от старта трека. Зацикленные треки — как раньше.

Часы — фейковые (``clock=`` контроллера), тики каждые 5 с, как таймер ноды.
"""
from __future__ import annotations

import logging
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "src" / "rob_box_voice"))

from rob_box_voice.core.dj_mode import (  # noqa: E402
    DJ_MIN_TRACK_PLAY_S,
    DJModeController,
    finite_form_lead_s,
)

T0 = 1_790_611_553.0  # старт Star Wars в живом логе 28.09


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
        self.dispatched_at: list = []

    def dispatch(self, prompt: str, is_auto: bool = False) -> None:  # noqa: ARG002
        self.dispatched_at.append(self._clock.now)

    def is_active(self) -> bool:
        return False

    def is_dialogue_active(self) -> bool:
        return False


def _controller():
    clock = _Clock(T0)
    hook = _Hook(clock)
    ctrl = DJModeController(hook=hook, logger=logging.getLogger("test"), clock=clock)
    ctrl.state.enabled = True
    ctrl.state.started_at = T0 - 600.0
    ctrl.state.set_bpm = 124
    return ctrl, clock, hook


def _run(ctrl, clock, until: float, tick_s: float = 5.0) -> None:
    while clock.now <= until:
        ctrl.tick()
        clock.now += tick_s


class TestFiniteFormTransition(unittest.TestCase):
    def test_live_star_wars_transition_fires_before_track_end(self) -> None:
        """Трек 141 с, переход был назначен на 146 с → переход ≤ конца трека,
        с запасом на ход модели и фейд."""
        ctrl, clock, hook = _controller()
        track_end = T0 + 141.0
        ctrl.state.next_transition_at = T0 + 146.0
        ctrl.state.form_ends_at = track_end
        ctrl.note_form_stop(track_end)

        _run(ctrl, clock, until=T0 + 200.0)

        self.assertTrue(hook.dispatched_at, "переход не случился вовсе")
        first = hook.dispatched_at[0]
        self.assertLessEqual(first, track_end)
        self.assertLessEqual(first, track_end - finite_form_lead_s(124) + 5.0)
        self.assertGreaterEqual(first, T0 + DJ_MIN_TRACK_PLAY_S)
        # один переход на эту форму (следующий — только по FALLBACK/новой форме)
        self.assertEqual(len(hook.dispatched_at), 1, hook.dispatched_at)

    def test_regression_before_fix_semantics_would_be_after_track_end(self) -> None:
        """Контроль: без ``stops_at`` (зацикленный трек) переход по-старому —
        не раньше конца формы и ``next_transition_at``."""
        ctrl, clock, hook = _controller()
        ctrl.state.next_transition_at = T0 + 146.0
        ctrl.state.form_ends_at = T0 + 141.0
        ctrl.note_form_stop(None)

        _run(ctrl, clock, until=T0 + 200.0)

        self.assertTrue(hook.dispatched_at)
        self.assertGreaterEqual(hook.dispatched_at[0], T0 + 146.0)

    def test_short_finite_track_plays_at_least_min_time(self) -> None:
        """Заказ на 40 с не уходит в фейд сразу после старта."""
        ctrl, clock, hook = _controller()
        ctrl.state.next_transition_at = T0 + 120.0
        ctrl.state.form_ends_at = T0 + 40.0
        ctrl.note_form_stop(T0 + 40.0)

        _run(ctrl, clock, until=T0 + 60.0)

        self.assertTrue(hook.dispatched_at)
        self.assertGreaterEqual(hook.dispatched_at[0], T0 + DJ_MIN_TRACK_PLAY_S)
        self.assertLessEqual(hook.dispatched_at[0], T0 + 40.0)

    def test_republished_stop_with_jitter_is_the_same_form(self) -> None:
        """``stops_at`` пересчитывается на каждой публикации (±мс) — это не
        новая форма: второго раннего перехода нет, отсчёт старта не сдвинут."""
        ctrl, clock, hook = _controller()
        ctrl.state.next_transition_at = T0 + 146.0
        ctrl.note_form_stop(T0 + 141.0)
        seen = ctrl.state.form_stops_seen_at
        while clock.now <= T0 + 200.0:
            ctrl.note_form_stop(T0 + 141.0 + 0.003)
            ctrl.tick()
            clock.now += 5.0
        self.assertEqual(ctrl.state.form_stops_seen_at, seen)
        self.assertEqual(len(hook.dispatched_at), 1, hook.dispatched_at)

    def test_new_finite_form_gets_its_own_early_transition(self) -> None:
        ctrl, clock, hook = _controller()
        ctrl.state.next_transition_at = T0 + 146.0
        ctrl.note_form_stop(T0 + 141.0)
        _run(ctrl, clock, until=T0 + 130.0)
        self.assertEqual(len(hook.dispatched_at), 1)
        # модель запустила новый конечный трек на 141 с
        start = clock.now
        ctrl.note_form_stop(start + 141.0)
        ctrl.state.form_ends_at = start + 141.0
        _run(ctrl, clock, until=start + 141.0)
        self.assertEqual(len(hook.dispatched_at), 2)
        self.assertLessEqual(hook.dispatched_at[1], start + 141.0)

    def test_reset_clears_stop(self) -> None:
        ctrl, _, _ = _controller()
        ctrl.note_form_stop(T0 + 141.0)
        ctrl.handle_message('{"enabled": false}')
        self.assertIsNone(ctrl.state.form_stops_at)

    def test_garbage_stop_is_ignored(self) -> None:
        ctrl, _, _ = _controller()
        for bad in ("soon", True, None, [1]):
            ctrl.note_form_stop(bad)
            self.assertIsNone(ctrl.state.form_stops_at)


if __name__ == "__main__":
    unittest.main()
