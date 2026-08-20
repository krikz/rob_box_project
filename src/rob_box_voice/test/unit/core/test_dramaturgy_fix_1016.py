"""Smoke test for the dramaturgy fix in build_auto_prompt (issue #1016 follow-up).

Verifies that:
  * n=1 prompt BANS load_track/list_tracks (library auto-play) and carries
    the stage_line markers
  * n=2 prompt contains the "С РАЗВИТИЕМ" requirement + the listed patterns
  * Neither prompt leaks task-tracking identifiers (#NNNN, "Refs:", "issue #")
    that carry no semantic value for the LLM and may trigger hallucinated
    "fix #NNNN" comments in generated code.
"""
from __future__ import annotations

import json
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

    # Library line: DJ transitions must NOT load stored tracks (load_track
    # auto-plays them → harsh cut between tracks).
    assert "НЕ вызывай load_track" in prompt, "load_track ban missing from n=1 prompt"
    assert "С НУЛЯ" in prompt, "fresh-generation instruction missing from n=1 prompt"

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
    for pattern in (".every", "Pvar", "linvar", "Clock.future"):
        assert pattern in prompt, f"{pattern!r} missing from dramaturgy list"

    # Library line still present on transitions — as a BAN (no auto-play).
    assert "НЕ вызывай load_track" in prompt
    assert "С НУЛЯ" in prompt


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


def test_n1_contains_research_and_plan_instructions() -> None:
    """Переход #1 должен: сначала исследовать персону/музыку, затем план."""
    ctrl = _build_controller()
    ctrl.state.theme = "панк"
    prompt = ctrl.build_auto_prompt(1)

    # research-first: поиск персоны, сэмплов и готовых композиций.
    assert "search_web" in prompt, "research-first: search_web missing"
    assert "search_samples" in prompt, "research-first: search_samples missing"
    assert "gen_search_library" in prompt, "research-first: gen_search_library missing"
    # план сета через set_dj_mode(plan=...).
    assert "ПЛАН СЕТА" in prompt, "plan instruction missing from n=1"
    assert "set_dj_mode(enabled=true, plan=" in prompt


def test_tech_guardrails_present_on_transition() -> None:
    """Каждый переход обязан нести тех-лимиты против какофонии."""
    ctrl = _build_controller()
    ctrl.state.theme = "техно"
    prompt = ctrl.build_auto_prompt(2)

    assert "Clock.clear()" in prompt
    assert "НЕ используй chop" in prompt
    assert "СУММА amp" in prompt
    assert "d4/d5/p4/p5" in prompt
    assert "search_samples" in prompt


def test_chop_is_banned_not_recommended() -> None:
    """chop= должен встречаться только как запрет (щелчки на 16kHz)."""
    ctrl = _build_controller()
    ctrl.state.theme = "техно"
    prompt = ctrl.build_auto_prompt(2)

    assert ".every()" in prompt
    assert "НЕ используй chop=" in prompt


def test_final_track_when_plan_set() -> None:
    """При достижении последнего трека плана — финальный промпт с выключением DJ."""
    ctrl = _build_controller()
    ctrl.state.theme = "диско"
    ctrl.state.set_plan = "Трек 1: старт\nТрек 2: пик"

    prompt = ctrl.build_auto_prompt(2)

    assert "ФИНАЛЬНЫЙ ТРЕК" in prompt
    assert "set_dj_mode(enabled=false)" in prompt


def test_first_plan_does_not_reset_transition_count() -> None:
    """Первый план приходит на переходе #1 — счётчик НЕ должен сброситься,
    иначе следующий тик снова запустит переход #1."""
    ctrl = _build_controller()
    ctrl.state.enabled = True
    ctrl.state.transition_count = 1

    ctrl.handle_message(json.dumps({
        "enabled": True,
        "plan": "Трек 1: a\nТрек 2: b",
        "next_transition_sec": 45,
    }))

    assert ctrl.state.set_plan == "Трек 1: a\nТрек 2: b"
    assert ctrl.state.transition_count == 1, "первый план не должен сбрасывать счётчик"


def test_plan_rewrite_resets_transition_count() -> None:
    """Переписывание сета (новый план при уже существующем) — счётчик с нуля."""
    ctrl = _build_controller()
    ctrl.state.enabled = True
    ctrl.state.set_plan = "Трек 1: old"
    ctrl.state.transition_count = 5

    ctrl.handle_message(json.dumps({
        "enabled": True,
        "plan": "Трек 1: new\nТрек 2: new2",
    }))

    assert ctrl.state.transition_count == 0, "переписывание плана должно сбрасывать счётчик"
