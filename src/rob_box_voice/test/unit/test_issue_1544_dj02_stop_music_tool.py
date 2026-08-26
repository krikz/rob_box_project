"""Regression guard for issue #1544 / #1561 — ``stop_music`` tool enforcement.

Live e2e (run #32910796351, dj02 step): user asks «Робот, стоп музыку»,
LLM answers verbally («Готово, выключаю!») without invoking ``stop_music``
tool — Renardo keeps playing, e2e GATE-1 fails on
``missing expected_tool_calls: ['stop_music']``. Same root cause as the
original issue #1544 (dj02 #1506), still leaking through after the
RULE #MUSIC-CHOICE fix in PR #1565.

These tests pin the wording in :file:`src/rob_box_voice/prompts/
master_prompt_compact.txt` so future prompt refactors cannot silently
remove the enforcement. The same approach is used in
``test_issue_1532_voice_multi_skazka.py`` and
``test_issue_982_prompt_no_stop_music.py`` — one guard per acceptance
boundary that the LLM has historically drifted on.

Acceptance map (mirrors :file:`master_prompt_compact.txt` RULE #MUSIC
section after PR #1565 stop-music enforcement fix):

* RULE #MUSIC is present and explicitly anchored on issue #1544 #1561.
* The rule mandates calling ``stop_music()`` (NOT verbal-only
  «Выключаю!»).
* Covers BOTH Renardo-бит AND AI-сгенерированный mp3 (issue #1544
  split — ``<generated_music>: idle`` does NOT mean «музыки нет» for
  Renardo).
* Lists stop triggers: стоп / хватит / выключи / убери.
* Demands ``set_dj_mode(enabled=false)`` when DJ-mode is active.
* Anchored on «после Renardo» / «после dj01».
* Scenario ``dj02_stop_music`` has acceptance block with expected
  ``stop_music``.
* ``dj02`` step text contains «стоп музыку» and wake word «Робот».

If any of these tests fails after a prompt edit, the regression is
exactly the issue #1544 #1561 dj02 GATE-1 fail-streak the e2e harness
already reported. Do NOT loosen the test — fix the prompt instead.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest


MASTER_PROMPT_PATH = Path(__file__).resolve().parents[2] / "prompts" / "master_prompt_compact.txt"
SCENARIO_PATH = (
    Path(__file__).resolve().parents[4]
    / ".github"
    / "e2e"
    / "scenarios"
    / "voice_core_suite_v1.json"
)


def _read_prompt() -> str:
    assert MASTER_PROMPT_PATH.exists(), (
        f"master_prompt_compact.txt not found at {MASTER_PROMPT_PATH}"
    )
    return MASTER_PROMPT_PATH.read_text(encoding="utf-8")


def _read_scenario() -> str:
    assert SCENARIO_PATH.exists(), (
        f"voice_core_suite_v1.json not found at {SCENARIO_PATH}"
    )
    return SCENARIO_PATH.read_text(encoding="utf-8")


def _extract_rule_music(prompt: str) -> str:
    """Извлекает блок RULE #MUSIC — ОСТАНОВКА МУЗЫКИ.

    Блок начинается с ``� **RULE #MUSIC — ОСТАНОВКА МУЗЫКИ`` и
    заканчивается перед следующим ``🚨 **RULE`` (или концом файла).
    Используем не-жадный ``+?`` чтобы не вылезти за границы."""
    match = re.search(
        r"RULE #MUSIC — ОСТАНОВКА МУЗЫКИ.+?(?=RULE #|\Z)",
        prompt,
        flags=re.DOTALL,
    )
    assert match, "RULE #MUSIC — ОСТАНОВКА МУЗЫКИ block not found in master prompt"
    return match.group(0)


# ---------------------------------------------------------------------------
# Prompt-level guard — RULE #MUSIC must be present and enforce stop_music
# ---------------------------------------------------------------------------


class TestMasterPromptStopMusicRule:
    def test_rule_music_anchor_1544_1561(self) -> None:
        """🔴 FIX (issue #1544 #1561 dj02): RULE #MUSIC in
        master_prompt_compact.txt MUST be anchored on
        issue #1544 / #1561 so future refactors don't silently drop
        the enforcement (live 23.08 incident)."""
        prompt = _read_prompt()
        assert "RULE #MUSIC" in prompt, "RULE #MUSIC header missing"
        # Anchor: rule explicitly names the issues (anchored text).
        assert re.search(
            r"RULE #MUSIC.*1544.*1561|#1544.*#1561.*RULE #MUSIC",
            prompt,
            flags=re.DOTALL,
        ), (
            "RULE #MUSIC must explicitly anchor on issue #1544 #1561 — "
            "otherwise future refactors will silently lose enforcement"
        )

    def test_rule_music_mandates_stop_music_tool(self) -> None:
        """Rule MUST demand the LLM call ``stop_music()`` (not just
        talk about stopping). Live e2e: LLM answered «Выключаю!» —
        no tool, Renardo kept playing."""
        prompt = _read_prompt()
        rule = _extract_rule_music(prompt)
        assert "stop_music()" in rule, (
            "RULE #MUSIC MUST demand ``stop_music()`` tool call"
        )

    def test_rule_music_explicitly_prohibits_verbal_only(self) -> None:
        """Rule MUST prohibit verbal-only «Выключаю!» / «Готово!» —
        this is the bug dj02 keeps hitting."""
        prompt = _read_prompt()
        rule = _extract_rule_music(prompt)
        # Either explicit «ЗАПРЕЩЕНО verbal-only» OR «❌ BUG» example.
        has_prohibition = (
            "ЗАПРЕЩЕНО" in rule
            or "❌ ЗАПРЕЩЕНО" in rule
            or "❌ BUG" in rule
            or "verbal-only" in rule
        )
        assert has_prohibition, (
            "RULE #MUSIC MUST explicitly prohibit verbal-only answers — "
            "the dj02 GATE-1 fail-streak is caused by LLM answering "
            "«Выключаю!» without calling stop_music()"
        )

    def test_rule_music_covers_renardo_path(self) -> None:
        """Issue #1544 specifically called out that
        ``<generated_music>: idle`` does NOT mean «музыки нет» for
        Renardo-бит. Rule MUST cover the Renardo path (otherwise
        GATE-1 will fail on dj02 after every Renardo step)."""
        prompt = _read_prompt()
        rule = _extract_rule_music(prompt)
        assert "Renardo" in rule, (
            "RULE #MUSIC MUST explicitly cover Renardo-бит path — "
            "this is the root cause of the original dj02 #1544 bug"
        )

    def test_rule_music_lists_stop_triggers(self) -> None:
        """Rule MUST enumerate the stop triggers (стоп / хватит /
        выключи / убери) so the LLM can't claim «не понял команду»."""
        prompt = _read_prompt()
        rule = _extract_rule_music(prompt)
        for trigger in ("стоп", "хватит", "выключи", "убери"):
            assert trigger in rule, (
                f"RULE #MUSIC MUST list the stop trigger «{trigger}» — "
                "otherwise LLM will refuse «не знаю такой команды»"
            )

    def test_rule_music_demands_dj_mode_off(self) -> None:
        """When DJ-mode is active the rule MUST demand
        ``set_dj_mode(enabled=false)`` — otherwise the DJ tick (5s)
        will start a fresh track even after ``stop_music()``."""
        prompt = _read_prompt()
        rule = _extract_rule_music(prompt)
        assert "set_dj_mode" in rule and "enabled=false" in rule, (
            "RULE #MUSIC MUST demand set_dj_mode(enabled=false) — "
            "DJ tick auto-fires next track after stop_music() otherwise"
        )


# ---------------------------------------------------------------------------
# Scenario-level guard — dj02 must keep its expected_tool_calls
# ---------------------------------------------------------------------------


class TestScenarioDj02StopMusicAcceptance:
    def test_dj02_step_has_expected_stop_music(self) -> None:
        """The dj02 step in voice_core_suite_v1 MUST keep
        ``expected_tool_calls: [stop_music]`` — otherwise GATE-1 has
        nothing to validate against."""
        import json

        scenario = json.loads(_read_scenario())
        steps = {s["label"]: s for s in scenario["steps"]}
        assert "dj02_stop_music" in steps, (
            "dj02_stop_music step is missing from voice_core_suite_v1"
        )
        dj02 = steps["dj02_stop_music"]
        acceptance = dj02.get("acceptance", {})
        expected = acceptance.get("expected_tool_calls", [])
        assert "stop_music" in expected, (
            "dj02_stop_music acceptance MUST expect stop_music tool — "
            "without it GATE-1 has no validation for the dj02 path"
        )

    def test_dj02_step_text_has_wake_word_and_stop(self) -> None:
        """Voice command text MUST contain «Робот» (wake word) and
        «стоп музыку» (the actual command). If either drifts the
        e2e harness will time out on the wake-gate preflight."""
        import json

        scenario = json.loads(_read_scenario())
        steps = {s["label"]: s for s in scenario["steps"]}
        text = steps["dj02_stop_music"]["text"].lower()
        assert "робот" in text, (
            "dj02_stop_music step text MUST contain wake word «Робот»"
        )
        assert "стоп" in text and "музык" in text, (
            "dj02_stop_music step text MUST contain stop-music phrase"
        )
