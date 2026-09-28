"""
test_issue_3113_dj_no_repeat.py — одна песня не звучит дважды в одном DJ-сете.

Живой прогон 28.09.2026 (issue #3113, п.5): классический сет, трек #1 —
Für Elise, авто-переход #1 снова выбрал ``compose_music(name='Fur Elise')``.

Фикс: mcp_server публикует ``track`` (заголовок играющей темы) в
``/voice/music/form``; ``DJModeController.note_track_name`` копит сыгранное
в ``DJState.played_names``; промпт перехода запрещает повтор.
"""
from __future__ import annotations

import json
import logging
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "src" / "rob_box_voice"))

from rob_box_voice.core.dj_mode import DJModeController  # noqa: E402


class _Hook:
    persona_default = "Роббокс"
    on_stop = None

    def dispatch(self, prompt: str, is_auto: bool = False) -> None:  # noqa: ARG002
        pass

    def is_active(self) -> bool:
        return False

    def is_dialogue_active(self) -> bool:
        return False


PLAN = "Трек 1: Für Elise\nТрек 2: Турецкий марш\nТрек 3: Лунная соната\nТрек 4: Ода к радости"


def _enabled(plan: str = PLAN) -> DJModeController:
    ctrl = DJModeController(hook=_Hook(), logger=logging.getLogger("test"), clock=lambda: 1_790_612_000.0)
    ctrl.handle_message(json.dumps({"enabled": True, "plan": plan, "next_transition_sec": 60}))
    return ctrl


class TestPlayedNames(unittest.TestCase):
    def test_live_fur_elise_is_forbidden_on_transition_1(self) -> None:
        """Живой сценарий: Für Elise сыграна (трек сета #1) → переход #1 её запрещает."""
        ctrl = _enabled()
        ctrl.note_track_name("Für Elise")
        ctrl.state.tracks_started = 1
        prompt = ctrl.build_auto_prompt(1)
        self.assertIn("В этом сете уже звучали: «Für Elise»", prompt)
        self.assertIn("НЕ играй их снова", prompt)

    def test_mid_set_and_final_prompts_forbid_repeats(self) -> None:
        ctrl = _enabled()
        for name in ("Für Elise", "Star Wars"):
            ctrl.note_track_name(name)
        ctrl.state.tracks_started = 2
        mid = ctrl.build_auto_prompt(2)
        self.assertIn("«Für Elise», «Star Wars»", mid)
        ctrl.state.tracks_started = 4
        final = ctrl.build_auto_prompt(5)
        self.assertIn("ФИНАЛЬНЫЙ ТРЕК", final)
        self.assertIn("«Für Elise», «Star Wars»", final)

    def test_republish_and_case_do_not_duplicate(self) -> None:
        ctrl = _enabled()
        for name in ("Für Elise", "Für Elise", "für elise ", None, "", 42):
            ctrl.note_track_name(name)
        self.assertEqual(ctrl.state.played_names, ["Für Elise"])

    def test_nothing_played_means_no_line(self) -> None:
        ctrl = _enabled()
        ctrl.state.tracks_started = 2
        self.assertNotIn("уже звучали", ctrl.build_auto_prompt(2))

    def test_not_recorded_while_dj_off(self) -> None:
        ctrl = DJModeController(hook=_Hook(), logger=logging.getLogger("test"))
        ctrl.note_track_name("Für Elise")
        self.assertEqual(ctrl.state.played_names, [])

    def test_new_set_starts_clean(self) -> None:
        ctrl = _enabled()
        ctrl.note_track_name("Für Elise")
        ctrl.handle_message(json.dumps({"enabled": False}))
        self.assertEqual(ctrl.state.played_names, [])
        ctrl.handle_message(json.dumps({"enabled": True, "plan": PLAN}))
        self.assertEqual(ctrl.state.played_names, [])

    def test_same_set_update_keeps_history(self) -> None:
        """set_dj_mode(next_transition_sec=...) внутри сета историю не стирает."""
        ctrl = _enabled()
        ctrl.note_track_name("Für Elise")
        ctrl.handle_message(json.dumps({"enabled": True, "next_transition_sec": 90}))
        self.assertEqual(ctrl.state.played_names, ["Für Elise"])


if __name__ == "__main__":
    unittest.main()
