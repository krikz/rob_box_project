"""Issue #3136 — страховка от дыр тишины: доигравший трек в DJ-режиме.

Живой замер 01.10: club без repeat доиграл на 97 с, переход только на 203 с.
Теперь снимок плеера с ``finished_track_id`` назначает переход на ближайший тик.
"""
from __future__ import annotations

import logging
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "src" / "rob_box_voice"))

from rob_box_voice.core.dj_mode import (  # noqa: E402
    DJ_TRANSITION_INFLIGHT_S,
    DJModeController,
    dj_note_track_finished,
)

T0 = 1_790_900_000.0


class _Clock:
    def __init__(self, now):
        self.now = now

    def __call__(self):
        return self.now


class _Snap:
    def __init__(self, finished, state="idle"):
        self.finished_track_id = finished
        self.state = state


class _Hook:
    persona_default = "Роббокс"
    on_stop = None

    def __init__(self):
        self.n = 0

    def dispatch(self, prompt, is_auto=False):  # noqa: ARG002
        self.n += 1

    def is_active(self):
        return False

    def is_dialogue_active(self):
        return False


def _ctrl():
    clock, hook = _Clock(T0), _Hook()
    ctrl = DJModeController(hook=hook, logger=logging.getLogger("t"), clock=clock)
    ctrl.state.enabled = True
    ctrl.state.started_at = T0 - 600.0
    ctrl.state.set_bpm = 124
    ctrl.state.next_transition_at = T0 + 106.0  # таймер далеко (как 203 с в замере)
    return ctrl, clock, hook


class TestFinishedTrackTransition(unittest.TestCase):
    def test_finished_track_fires_transition_on_next_tick_not_timer(self):
        ctrl, clock, hook = _ctrl()
        ctrl.tick()
        self.assertEqual(hook.n, 0)
        self.assertTrue(dj_note_track_finished(ctrl, _Snap("trk-1")))
        clock.now += 5.0
        ctrl.tick()
        self.assertEqual(hook.n, 1)

    def test_repeated_snapshot_of_same_track_does_not_double_dispatch(self):
        ctrl, clock, hook = _ctrl()
        self.assertTrue(dj_note_track_finished(ctrl, _Snap("trk-1")))
        ctrl.tick()
        self.assertFalse(dj_note_track_finished(ctrl, _Snap("trk-1")))
        clock.now += 5.0
        ctrl.tick()
        self.assertEqual(hook.n, 1)

    def test_transition_in_flight_is_not_duplicated(self):
        ctrl, clock, hook = _ctrl()
        ctrl.state.last_transition_at = T0 - 30.0
        self.assertFalse(dj_note_track_finished(ctrl, _Snap("trk-old")))
        ctrl.tick()
        self.assertEqual(hook.n, 0)

    def test_old_transition_does_not_block(self):
        ctrl, clock, hook = _ctrl()
        ctrl.state.last_transition_at = T0 - DJ_TRANSITION_INFLIGHT_S - 1
        self.assertTrue(dj_note_track_finished(ctrl, _Snap("trk-2")))

    def test_dj_disabled_ignores_finish(self):
        ctrl, clock, hook = _ctrl()
        ctrl.state.enabled = False
        self.assertFalse(dj_note_track_finished(ctrl, _Snap("trk-1")))
        self.assertEqual(ctrl.state.next_transition_at, T0 + 106.0)

    def test_playing_snapshot_is_ignored(self):
        ctrl, _, _ = _ctrl()
        self.assertFalse(dj_note_track_finished(ctrl, _Snap("trk-1", state="playing")))

    def test_no_finished_id_is_ignored(self):
        ctrl, _, _ = _ctrl()
        self.assertFalse(dj_note_track_finished(ctrl, _Snap(None)))
        self.assertFalse(dj_note_track_finished(ctrl, _Snap("")))


if __name__ == "__main__":
    unittest.main()
