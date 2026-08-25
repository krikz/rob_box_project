"""Unit tests for issue #1629 — DJ-mode autonomous e2e breakages.

Regression suite for issue #1629 / #1506: DJ-mode kept firing
autonomous music transitions and inventing Confucius/Lao-tzu
"wise man" quotes without the user asking, which broke the e2e
harness (VAD picked up the music, STT returned empty, and the
harness saw unprovoked TTS traffic).

Acceptance map:

* :class:`TestDjDefaultDisabled` — acceptance #1: DJ-mode must be
  off by default. Even if a previous e2e round left
  ``DJState.enabled=True`` in some persistent store, a fresh
  ``DJModeController`` instance must start disabled. The
  :meth:`DJModeController.tick` method must therefore be a no-op
  until :meth:`handle_message` flips ``enabled=True`` explicitly.

* :class:`TestDjAutoTransitionLogsTimestamp` — acceptance #4:
  every auto-transition must log a timestamped line so that the
  e2e operator can see *when* DJ-mode fired and *why* (retros on
  t_9435a3c5 / t_0c0a98ac showed that silent background ticks
  make diagnosis impossible).

* :class:`TestDjHandleMessageResetsEverything` — acceptance
  helper: handle_message('{"enabled": false}') must clear all
  state regardless of prior content. This is the contract the
  e2e harness relies on when it pokes ``/voice/dj_mode`` at the
  start of every round.

* :class:`TestMasterPromptHasNoBsQuotesRule` — acceptance #3:
  the master prompt must contain an explicit NO_BS_QUOTES rule
  that forbids the LLM from inventing philosopher quotes
  (Конфуций / Лао-цзы / Сократ) without an explicit user
  request.

* :class:`TestE2eHarnessCallsDisableDj` — acceptance #2:
  the e2e harness script ``.github/workflows/scripts/e2e_voice_test.sh``
  must call a ``disable_dj_mode_round_start`` helper (or
  equivalent ``ros2 topic pub /voice/dj_mode`` sequence) at the
  start of each round so that a sticky DJ state from a prior
  session does not leak into the next e2e round.

These tests are pure Python — no ROS2, no LLM API. They run
under ``pytest src/rob_box_voice/test/unit/`` (the
``rob_box_voice/test/unit`` testpath that CI exercises).
"""

from __future__ import annotations

import json
import logging
import re
from pathlib import Path
from typing import List

import pytest

from rob_box_voice.core.dj_mode import DJHook, DJModeController, DJState


# ── helpers ───────────────────────────────────────────────────────────


def _noop_dispatch(prompt: str, from_tick: bool = False) -> None:
    """Stand-in for the shell's coroutine launcher — never invoked
    in these tests because we don't drive ticks past the
    ``enabled=True`` gate without a real LLM stub."""


def _make_controller() -> DJModeController:
    """Build a controller wired to inert hooks for unit-testing."""

    hook = DJHook(
        dispatch=_noop_dispatch,
        is_active=lambda: False,
        is_dialogue_active=lambda: False,
    )
    return DJModeController(hook=hook, logger=_capturing_logger())


class _CapturingHandler(logging.Handler):
    """Logging handler that records formatted records as plain strings."""

    def __init__(self) -> None:
        super().__init__()
        self.messages: List[str] = []

    def emit(self, record: logging.LogRecord) -> None:
        self.messages.append(self.format(record))


def _capturing_logger() -> logging.Logger:
    """Return a logging.Logger with a single :class:`_CapturingHandler`.

    Real ``logging.Logger`` (so pyright is happy) but with
    capture on so tests can grep the formatted output.
    """
    logger = logging.getLogger(
        f"test_issue_1629.{id(_capturing_logger)}"
    )
    logger.setLevel(logging.DEBUG)
    handler = _CapturingHandler()
    handler.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))
    logger.handlers = [handler]
    logger.propagate = False
    # Stash handler on the logger for ergonomic access in tests.
    logger._test_handler = handler  # type: ignore[attr-defined]
    return logger


# ── acceptance #1: DJ off by default ──────────────────────────────────


class TestDjDefaultDisabled:
    """Acceptance #1: ``DJState.enabled`` must be ``False`` by default.

    Issue #1506 / #1629: the e2e harness expects DJ-mode to be
    off when a round starts. The previous default was already
    ``False`` but a regression test pins the contract so a future
    refactor can't silently flip it on.
    """

    def test_fresh_dj_state_has_enabled_false(self) -> None:
        state = DJState()
        assert state.enabled is False, (
            "DJState().enabled must default to False — issue #1629 "
            "acceptance #1 (no autonomous transitions without "
            "explicit user opt-in)"
        )

    def test_fresh_controller_tick_is_noop_when_disabled(self) -> None:
        """Tick() must not dispatch anything while ``enabled=False``.

        This is the safety net — even if a previous session left
        DJ-mode on, the tick guard must short-circuit before
        publishing any prompt to the LLM.
        """
        controller = _make_controller()
        dispatched: List[str] = []
        controller._hook.dispatch = lambda prompt, from_tick=False: dispatched.append(
            prompt
        )

        controller.tick()
        controller.tick()
        controller.tick()

        assert dispatched == [], (
            f"tick() must be a no-op while enabled=False; "
            f"dispatched={dispatched!r}"
        )

    def test_tick_does_not_run_when_disabled_even_with_elapsed_transition(
        self,
    ) -> None:
        """Tick() must skip even if ``next_transition_at`` is in the past.

        Belt-and-braces — the gate is ``enabled`` (not the timer),
        because a stale ``next_transition_at`` from a previous
        round must NOT wake the controller up.
        """
        controller = _make_controller()
        dispatched: List[str] = []
        controller._hook.dispatch = lambda prompt, from_tick=False: dispatched.append(
            prompt
        )
        controller.state.next_transition_at = 0.0  # ancient

        controller.tick()

        assert dispatched == [], (
            "stale next_transition_at must NOT wake DJ when enabled=False; "
            f"dispatched={dispatched!r}"
        )


# ── acceptance #4: log with timestamp ─────────────────────────────────


class TestDjAutoTransitionLogsTimestamp:
    """Acceptance #4: every auto-transition log line must carry a timestamp.

    Issue #1629 acceptance #4 requires the operator to see WHEN
    DJ-mode fired — the retro on t_9435a3c5 / t_0c0a98ac showed
    that a silent background tick makes diagnosis impossible. The
    existing log is ``🎧 DJ auto-transition #{next_n}``; the new
    contract is ``🎧 DJ_AUTO transition #N fired at HH:MM:SS``.
    """

    def test_tick_emits_timestamped_log_line_when_firing(self) -> None:
        """tick() with enabled=True and an elapsed timer must log
        ``🎧 DJ_AUTO transition #1 fired at HH:MM:SS`` (or any
        time-formatted string) and dispatch the prompt. The
        explicit ``DJ_AUTO transition #N fired at`` prefix lets
        the operator grep e2e logs without false positives on
        user-initiated DJ prompts."""

        hook = DJHook(
            dispatch=_noop_dispatch,
            is_active=lambda: False,
            is_dialogue_active=lambda: False,
        )
        logger = _capturing_logger()
        controller = DJModeController(hook=hook, logger=logger)
        # Enable DJ-mode and arm the timer for "right now".
        controller.state.enabled = True
        controller.state.next_transition_at = 0.0  # ancient

        controller.tick()

        # Find the timestamped transition line — accept either
        # HH:MM:SS or HH:MM:SS.mmm or any ISO-like wall-clock.
        pattern = re.compile(
            r"DJ_AUTO transition #1 fired at \d{2}:\d{2}:\d{2}"
        )
        messages = logger._test_handler.messages  # type: ignore[attr-defined]
        matches = [m for m in messages if pattern.search(m)]
        assert matches, (
            "DJ-mode must log a timestamped transition line on every "
            "auto-tick. Expected pattern: "
            "'DJ_AUTO transition #N fired at HH:MM:SS'. "
            f"Got messages={messages!r}"
        )


# ── helper: handle_message('{"enabled": false}') resets state ────────


class TestDjHandleMessageResetsEverything:
    """handle_message('{"enabled": false}') must clear DJ-state
    regardless of prior content — this is the contract the e2e
    harness relies on when it pokes ``/voice/dj_mode`` at the
    start of every round.

    Acceptance #2 (e2e harness disables DJ) only works if the
    state machine respects the disable payload. Pinning it here
    protects the wiring even if the harness script moves around.
    """

    def test_disable_payload_clears_all_state(self) -> None:
        controller = _make_controller()
        # First, enable DJ with rich state — theme, persona, plan,
        # transition_count, next_transition_at.
        controller.handle_message(json.dumps({
            "enabled": True,
            "theme": "acid jazz",
            "persona": "DJ Pulse",
            "plan": "Трек 1: warmup\nТрек 2: peak",
            "next_transition_sec": 45,
        }))
        assert controller.state.enabled is True
        assert controller.state.theme == "acid jazz"
        assert controller.state.persona == "DJ Pulse"
        assert controller.state.transition_count == 0  # arm only, no tick yet
        assert controller.state.next_transition_at > 0.0

        # Now the e2e harness disables DJ.
        controller.handle_message(json.dumps({"enabled": False}))

        assert controller.state.enabled is False, (
            "handle_message(enabled=False) must flip the master switch off"
        )
        assert controller.state.theme == "", (
            "theme must be cleared so the next enable starts fresh"
        )
        assert controller.state.persona == "", (
            "persona must be cleared so the next enable starts fresh"
        )
        assert controller.state.set_plan == ""
        assert controller.state.transition_count == 0
        assert controller.state.next_transition_at == 0.0

    def test_disable_payload_works_even_from_idle_state(self) -> None:
        """Sending a disable to a fresh controller must not raise
        and must leave state clean (idempotent)."""
        controller = _make_controller()
        controller.handle_message(json.dumps({"enabled": False}))
        assert controller.state.enabled is False
        assert controller.state.theme == ""


# ── acceptance #3: NO_BS_QUOTES rule in master prompt ───────────────


class TestMasterPromptHasNoBsQuotesRule:
    """Acceptance #3: master prompt must forbid invented philosopher
    quotes.

    Issue #1629 noted that LLM (under DJ-mode auto-transitions)
    invented Конфуций/Лао-цзы quotes even though the prompt told
    it not to. The fix is a hard, named RULE that the LLM
    grader can catch, plus an explicit list of forbidden
    authors.

    The rule must:

    * Be present in ``master_prompt_compact.txt``.
    * Explicitly forbid invented quotes from Конфуций /
      Лао-цзы / Сократ / etc.
    * Tell the LLM to play music via tools instead of babbling
      quotes.
    """

    @pytest.fixture()
    def master_prompt(self) -> str:
        # The text file lives at src/rob_box_voice/prompts/ —
        # ``test/unit`` testpath picks it up relative to package
        # root, but we resolve it from the package __file__ so
        # the test is robust against pytest's collection cwd.
        prompt_path = (
            Path(__file__).resolve().parents[3]
            / "prompts"
            / "master_prompt_compact.txt"
        )
        return prompt_path.read_text(encoding="utf-8")

    def test_prompt_contains_no_bs_quotes_rule(self, master_prompt: str) -> None:
        assert "NO_BS_QUOTES" in master_prompt, (
            "master_prompt_compact.txt must declare a "
            "RULE #NO_BS_QUOTES (issue #1629 acceptance #3) — "
            "explicit named rule lets post-validators grep for it"
        )

    def test_prompt_mentions_confucius_and_laotzu(self, master_prompt: str) -> None:
        # The rule must explicitly name the offenders so the LLM
        # does not fall back on "wise man" defaults.
        for needle in ("Конфуций", "Лао-цзы"):
            assert needle in master_prompt, (
                f"NO_BS_QUOTES rule must mention '{needle}' by name — "
                f"omitting the example makes the rule vague. "
                f"Prompt snippet near NO_BS_QUOTES:\n"
                f"{_extract_rule_block(master_prompt, 'NO_BS_QUOTES')}"
            )

    def test_prompt_forbids_speak_text_with_quotes(self, master_prompt: str) -> None:
        # The rule must explicitly say "do not call speak_text
        # with invented quotes" — otherwise the LLM falls back
        # to silent quotes wrapped in speak_text.
        rule_block = _extract_rule_block(master_prompt, "NO_BS_QUOTES")
        assert "speak_text" in rule_block, (
            "NO_BS_QUOTES rule must explicitly forbid "
            "speak_text with invented quotes "
            "(otherwise the LLM will fall back to silent quotes)"
        )


def _extract_rule_block(prompt: str, rule_name: str) -> str:
    """Return the lines around ``rule_name`` (plus a 5-line tail)
    so failure messages show context."""
    lines = prompt.splitlines()
    for idx, line in enumerate(lines):
        if rule_name in line:
            return "\n".join(lines[idx : idx + 10])
    return ""


# ── acceptance #2: e2e harness disables DJ-mode ─────────────────────


class TestE2eHarnessCallsDisableDj:
    """Acceptance #2: the e2e harness script must poke DJ-mode off
    at the start of every round. This is the e2e-side counterpart
    to acceptance #1 (default-disabled).

    The harness implementation is bash, not Python, so we use a
    plain string assertion rather than importing anything. We
    require:

    * A function named ``disable_dj_mode_round_start`` (or an
      equivalent ``ros2 topic pub /voice/dj_mode String ...``
      sequence) exists in the harness script.
    * It is called from the scenario-loop entry point (so the
      disable actually fires on every round).
    * It uses ``ros2 topic pub`` (the working channel for the
      in-container dialogue_node — see the
      ``voice_e2e_dialog.sh`` diagnostic for precedent).
    """

    @pytest.fixture()
    def harness_script(self) -> str:
        # .github/ lives at the repository root. The test file
        # is at ``src/rob_box_voice/test/unit/core/test_*.py`` —
        # that's 5 levels up from the test file (``..`` per
        # directory): unit → core → test → rob_box_voice → src
        # → repo root.
        script_path = (
            Path(__file__).resolve().parents[5]
            / ".github"
            / "workflows"
            / "scripts"
            / "e2e_voice_test.sh"
        )
        return script_path.read_text(encoding="utf-8")

    def test_harness_defines_disable_dj_helper(self, harness_script: str) -> None:
        assert "disable_dj_mode_round_start" in harness_script, (
            "e2e_voice_test.sh must define a "
            "disable_dj_mode_round_start() helper (issue #1629 "
            "acceptance #2) — without it, sticky DJ state from a "
            "prior round leaks into the next"
        )

    def test_harness_calls_disable_dj_helper(self, harness_script: str) -> None:
        """The helper must be invoked from the scenario loop so
        every round starts clean."""
        # Strip comments and blank lines so a `called` token in a
        # comment doesn't count as a real call.
        code_lines = "\n".join(
            line for line in harness_script.splitlines()
            if line.strip() and not line.strip().startswith("#")
        )
        assert "disable_dj_mode_round_start" in code_lines, (
            "disable_dj_mode_round_start must be invoked from the "
            "scenario loop (non-comment code) so the disable "
            "actually fires on every e2e round"
        )

    def test_harness_disable_uses_voice_dj_mode_topic(
        self, harness_script: str,
    ) -> None:
        """The disable must publish on ``/voice/dj_mode`` (the
        topic ``dialogue_node`` subscribes to) — using a
        different topic would be a silent no-op."""
        assert "/voice/dj_mode" in harness_script, (
            "disable_dj_mode_round_start must publish on "
            "/voice/dj_mode — dialogue_node subscribes there; "
            "any other topic is a silent no-op"
        )

    def test_harness_disable_payload_sets_enabled_false(
        self, harness_script: str,
    ) -> None:
        """The published payload must explicitly carry
        ``"enabled": false`` (or the equivalent ``enabled=false``)
        — anything else could leave DJ-mode in indeterminate state."""
        assert re.search(
            r"enabled['\"]?\s*:\s*false|enabled=false",
            harness_script,
        ), (
            "disable_dj_mode_round_start payload must carry "
            "enabled=false (JSON or YAML form) so DJ-mode is "
            "guaranteed off — issue #1629 acceptance #2"
        )