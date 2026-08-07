"""Smoke test for the dramaturgy fix in build_auto_prompt (issue #1016 follow-up).

Verifies that:
  * n=1 prompt contains the library_line (load_track) and stage_line markers
  * n=2 prompt contains the "С РАЗВИТИЕМ" requirement + the listed patterns
  * Neither prompt leaks task-tracking identifiers (#NNNN, "Refs:", "issue #")
    that carry no semantic value for the LLM and may trigger hallucinated
    "fix #NNNN" comments in generated code.
"""
from __future__ import annotations

import logging
import sys
from pathlib import Path

# Make the in-tree package importable without `pip install -e`.
ROOT = Path(__file__).resolve().parents[4]  # .../test/unit/core/dj_mode/
sys.path.insert(0, str(ROOT / "src" / "rob_box_voice"))

from rob_box_voice.core.dj_mode import DJModeController


class _StubHook:
    """Minimal DJHook-shaped stub — only the attributes the controller reads."""

    def __init__(self) -> None:
        self.persona_default = "Роббокс"
        self.dispatch_calls: list[str] = []

    def dispatch(self, prompt: str, is_auto: bool) -> None:  # noqa: ARG002
        self.dispatch_calls.append(prompt)

    def is_active(self) -> bool:
        return False

    def is_dialogue_active(self) -> bool:
        return False


def _build_controller() -> DJModeController:
    return DJModeController(hook=_StubHook(), logger=logging.getLogger("test"))


def test_n1_contains_library_and_stage_lines() -> None:
    ctrl = _build_controller()
    ctrl.state.theme = "летняя дискотека"
    prompt = ctrl.build_auto_prompt(1)

    # Library line: instructs the LLM to call load_track / list_tracks.
    assert "load_track" in prompt, "library_line missing from n=1 prompt"
    assert "list_tracks" in prompt, "list_tracks hint missing from n=1 prompt"

    # Stage line: progress indicator for the LLM.
    assert "переход #1" in prompt, f"stage_line missing — got: {prompt[:200]!r}"

    # Persona + theme still reach the prompt (regression guard).
    assert "Роббокс" in prompt
    assert "летняя дискотека" in prompt

    # Start-of-party marker preserved.
    assert "СТАРТ ВЕЧЕРИНКИ" in prompt


def test_n2_requires_dramaturgy() -> None:
    ctrl = _build_controller()
    ctrl.state.theme = "тёмный техно"
    prompt = ctrl.build_auto_prompt(2)

    assert "DJ_AUTO переход #2" in prompt
    assert "РАЗВИТИЕМ" in prompt, "transition prompt must demand dramaturgy"

    # All four dramaturgy patterns must be listed (model picks at least one).
    for pattern in (".every", "PVar", "LinExp", "Clock.future"):
        assert pattern in prompt, f"{pattern!r} missing from dramaturgy list"

    # Library line still present on transitions (so the model can re-find a ref).
    assert "load_track" in prompt
    assert "list_tracks" in prompt


def test_no_task_ids_leak_into_prompt() -> None:
    """Bug fix follow-up: do NOT pollute prompts with issue numbers / 'Refs:'.

    The LLM has no use for issue IDs and may echo them into generated code
    as a fake 'fix reference'. Such identifiers belong in git commit
    messages and PR descriptions, not in the prompt itself.
    """
    for n in (1, 2, 3, 5):
        ctrl = _build_controller()
        ctrl.state.theme = "хаус"
        prompt = ctrl.build_auto_prompt(n)

        for forbidden in ("#1016", "#990", "#1000", "Refs:", "issue #"):
            assert forbidden not in prompt, (
                f"n={n} prompt leaked {forbidden!r}: {prompt[:300]!r}"
            )
