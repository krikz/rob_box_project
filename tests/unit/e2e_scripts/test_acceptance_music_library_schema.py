"""Regression tests for music_library_acceptance_v1.json — ADR-0022 GATE-1 schema.

Issue #1392/#1358/#1403 — MiniMax music generation acceptance contract.

Without these tests, an editor could silently break GATE-1 compliance (remove
top-level ``expected_tool_calls`` / ``must_not_call``) and the agent-flow
e2e-process would auto-block ``e2e-done`` on PR #1398. We want CI to catch it
*before* merge-gate, not after.

Schema reference: docs/adr/0022-process-e2e-done-gates.md §4.1.

Note (t_cca7c074 ретро 19.08): файл лежит как music_library_acceptance_v1.json
(без «_suite»), но e2e_voice_test.sh auto-discovery (issue #1452) умеет
резолвить его через PREFIX logic (strip _v\d+ / _suite → music_library).
Имя файла оставлено КАК ЕСТЬ — это не баг, а legacy naming из PR #1398.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

SCENARIOS_DIR = Path(__file__).resolve().parents[3] / ".github/e2e/scenarios"

# All 7 new MCP tools introduced by PR #1398 (issue #1358). The top-level
# ``expected_tool_calls`` covers the 6 tools EXERCISED by the paired scenario
# music_library_suite_v1.json. ``gen_play_from_library`` is intentionally
# excluded: the scenario has no step that plays a saved track (play needs a
# pre-existing track = post-condition of generate_music), and GATE-1 aggregate
# is AND-semantics — listing an unexercised tool would guarantee a false FAIL.
ALL_7_GEN_TOOLS = (
    "generate_music",
    "gen_list_library",
    "gen_search_library",
    "gen_save_to_library",
    "gen_play_from_library",
    "gen_delete_from_library",
    "gen_get_track_info",
)
EXERCISED_GEN_TOOLS = tuple(t for t in ALL_7_GEN_TOOLS if t != "gen_play_from_library")


def _acceptance_path() -> Path:
    p = SCENARIOS_DIR / "music_library_acceptance_v1.json"
    if not p.exists():
        pytest.skip(f"acceptance.json not present: {p}")
    return p


def _load_acceptance() -> dict:
    p = _acceptance_path()
    return json.loads(p.read_text(encoding="utf-8"))


# ───────────────────────── GATE-1 schema (top-level) ─────────────────────────


class TestGate1TopLevelSchema:
    """The schema fields required by ADR-0022 §4.1 + agent-flow-e2e-process.sh."""

    def test_has_top_level_expected_tool_calls(self) -> None:
        a = _load_acceptance()
        assert "expected_tool_calls" in a, (
            "ADR-0022 GATE-1: missing top-level 'expected_tool_calls'. "
            "agent-flow-e2e-process.sh will refuse to set e2e-done."
        )
        assert isinstance(a["expected_tool_calls"], list)
        assert len(a["expected_tool_calls"]) > 0, (
            "expected_tool_calls must be a non-empty list (ADR-0022 §7.1 test 3)"
        )

    def test_has_top_level_must_not_call(self) -> None:
        a = _load_acceptance()
        assert "must_not_call" in a, (
            "ADR-0022 GATE-1: missing top-level 'must_not_call'. "
            "agent-flow-e2e-process.sh will refuse to set e2e-done."
        )
        assert isinstance(a["must_not_call"], list)
        # Top-level must_not_call may be EMPTY: execute_music_code is legal for
        # the renardo negative-control step and is forbidden per-step only.

    def test_expected_tool_calls_covers_exercised_gen_tools(self) -> None:
        """The 6 gen_* tools exercised by music_library_suite_v1.json MUST appear
        in expected_tool_calls (ADR-0022 §5.3, AND-semantics).

        gen_play_from_library is intentionally excluded — the scenario has no
        step that plays a saved track, and listing an unexercised tool would
        guarantee a false GATE-1 FAIL.
        """
        a = _load_acceptance()
        expected = set(a.get("expected_tool_calls", []))
        missing = set(EXERCISED_GEN_TOOLS) - expected
        assert not missing, (
            f"Top-level expected_tool_calls missing: {sorted(missing)}. "
            "ADR-0022 §5.3 says: 'Если хотя бы один не работает — e2e-done не ставится.'"
        )
        assert "gen_play_from_library" not in expected, (
            "gen_play_from_library must NOT be in top-level expected_tool_calls: "
            "the scenario music_library_suite_v1.json does not exercise it "
            "(GATE-1 aggregate is AND-semantics → false FAIL otherwise)."
        )

    def test_gen_steps_forbid_execute_music_code_per_step(self) -> None:
        """Voice vocal requests («спой песенку»/gen_*) MUST NOT route to Renardo.

        execute_music_code is forbidden PER-STEP for every gen_* step (the #1403
        regression: LLM replied «у меня нет такой функции»). Top-level
        must_not_call stays EMPTY because the renardo negative-control step
        legitimately calls execute_music_code.
        """
        a = _load_acceptance()
        gen_steps = [
            s for s in a.get("steps", [])
            if s.get("acceptance", {}).get("expected_tool_calls")
            and "execute_music_code" not in s["acceptance"]["expected_tool_calls"]
        ]
        assert gen_steps, "expected gen_* steps with per-step acceptance blocks"
        for s in gen_steps:
            assert "execute_music_code" in s["acceptance"].get("must_not_call", []), (
                f"gen_* step '{s.get('label')}' must forbid execute_music_code "
                "per-step (top-level must_not_call is empty by design)"
            )


# ───────────────────────── Per-step acceptance blocks ─────────────────────────


class TestPerStepAcceptanceBlocks:
    """Per-step acceptance.expected_tool_calls must align with patterns."""

    def test_each_step_has_acceptance_block(self) -> None:
        a = _load_acceptance()
        steps = a.get("steps", [])
        assert len(steps) >= 6, (
            f"Expected ≥6 step acceptance blocks (one per gen_* tool family); "
            f"got {len(steps)}"
        )
        for step in steps:
            assert "acceptance" in step, (
                f"Step '{step.get('label')}' missing 'acceptance' block. "
                "Per-step blocks give the validator fine-grained checks."
            )
            assert "expected_tool_calls" in step["acceptance"], (
                f"Step '{step.get('label')}' acceptance block missing "
                "'expected_tool_calls'"
            )
            assert "must_not_call" in step["acceptance"], (
                f"Step '{step.get('label')}' acceptance block missing 'must_not_call'"
            )

    def test_renardo_step_must_still_call_execute_music_code(self) -> None:
        """Negative control: «сыграй renardo бит» MUST go to Renardo (NOT gen_*).

        Without this, the LLM could over-route to MiniMax and break the existing
        Renardo live-engine feature (regression).
        """
        a = _load_acceptance()
        renardo_steps = [
            s for s in a.get("steps", [])
            if "renardo" in s.get("label", "").lower()
            or "renardo" in s.get("text", "").lower()
        ]
        assert renardo_steps, (
            "Acceptance suite must include a negative-control Renardo step "
            "to catch over-routing regressions"
        )
        rs = renardo_steps[0]
        assert "execute_music_code" in rs["acceptance"]["expected_tool_calls"], (
            f"Renardo step '{rs['label']}' must expect execute_music_code"
        )
        assert "generate_music" in rs["acceptance"].get("must_not_call", []), (
            f"Renardo step '{rs['label']}' must forbid generate_music"
        )

    def test_sing_song_step_routes_to_generate_music_not_renardo(self) -> None:
        """Issue #1403 — exact failure phrase must route to generate_music."""
        a = _load_acceptance()
        sing_steps = [
            s for s in a.get("steps", [])
            if "спой" in s.get("text", "").lower()
            or "sing" in s.get("label", "").lower()
        ]
        assert sing_steps, (
            "Acceptance suite must include the #1403 «спой песенку» step"
        )
        ss = sing_steps[0]
        assert "generate_music" in ss["acceptance"]["expected_tool_calls"], (
            f"Sing step '{ss['label']}' must expect generate_music (not Renardo)"
        )
        assert "execute_music_code" in ss["acceptance"]["must_not_call"], (
            f"Sing step '{ss['label']}' must forbid execute_music_code"
        )


# ───────────────────────── Voice command file references ─────────────────────────


class TestVoiceCommandFiles:
    """Per VOICE_COMMANDS_RESEARCH.md: short pre-recorded .ogg, VAD max=12s."""

    def test_each_step_references_existing_ogg(self) -> None:
        a = _load_acceptance()
        repo_root = Path(__file__).resolve().parents[3]
        missing: list[str] = []
        for step in a.get("steps", []):
            vf = step.get("voice_file")
            if not vf:
                continue  # synthesize-on-the-fly steps are also OK
            full = repo_root / vf
            if not full.exists():
                missing.append(f"{step['label']} → {vf}")
        assert not missing, (
            "Voice command files referenced by acceptance suite but missing "
            "from the repo (agent-flow will skip them silently and fall back "
            "to Yandex TTS synthesis, which masks real failures):\n  "
            + "\n  ".join(missing)
        )

    def test_voice_provenance_metadata_present(self) -> None:
        a = _load_acceptance()
        prov = a.get("voice_provenance")
        assert prov, (
            "Missing 'voice_provenance' block — agent-flow needs llm/tts/voice "
            "for provenance metadata in run summary"
        )
        for key in ("voice", "tts", "llm"):
            assert key in prov, f"voice_provenance missing '{key}'"


# ───────────────────────── Cross-checks with suite_v1.json ─────────────────────────


class TestConsistencyWithSuite:
    """Structural overlap guards drift between suite (runner) and acceptance (contract).

    The suite defines which voice commands the runner will play; the acceptance
    defines what the validator must check after each command. They may carry
    slightly different step labels (the acceptance can include extra «negative
    control» steps or split one suite step into two acceptance steps), but they
    MUST share:

      - at least one core gen_* step (so the validator isn't checking nothing)
      - at least one Renardo negative-control step (so over-routing regression
        is caught in BOTH the runner AND the contract)

    Exact 1:1 label equality is NOT required — see PR #1398 / acceptance.json
    ml02_sing_song_about_cat (acceptance-only step that proves the #1403 fix
    routes «спой песенку» to generate_music).
    """

    def test_renardo_negative_control_in_both_files(self) -> None:
        """Both files must include a Renardo «execute_music_code» step.

        If the suite lacks it, the runner never exercises Renardo live-engine
        (so any over-routing regression to MiniMax goes unnoticed). If the
        acceptance lacks it, the validator never checks over-routing either.
        """
        suite_path = SCENARIOS_DIR / "music_library_suite_v1.json"
        if not suite_path.exists():
            pytest.skip("music_library_suite_v1.json not present")
        suite = json.loads(suite_path.read_text(encoding="utf-8"))
        acceptance = _load_acceptance()

        def _is_renardo(step: dict) -> bool:
            label = step.get("label", "").lower()
            text = step.get("text", "").lower()
            return "renardo" in label or "renardo" in text

        assert any(_is_renardo(s) for s in suite.get("steps", [])), (
            "Suite has no Renardo negative-control step — runner won't catch "
            "over-routing regressions to MiniMax."
        )
        assert any(_is_renardo(s) for s in acceptance.get("steps", [])), (
            "Acceptance has no Renardo negative-control step — validator won't "
            "catch over-routing regressions to MiniMax."
        )

    def test_at_least_one_gen_tool_step_in_both_files(self) -> None:
        """Both files must exercise at least one gen_* tool step.

        Guards the case where the suite is empty / only renardo / only the
        acceptance's renardo control — either way the validator proves nothing
        about the new MCP tools.
        """
        suite_path = SCENARIOS_DIR / "music_library_suite_v1.json"
        if not suite_path.exists():
            pytest.skip("music_library_suite_v1.json not present")
        suite = json.loads(suite_path.read_text(encoding="utf-8"))
        acceptance = _load_acceptance()

        def _has_gen_step(steps: list[dict]) -> bool:
            for s in steps:
                patterns = s.get("patterns", [])
                acc = s.get("acceptance", {}) or {}
                calls = acc.get("expected_tool_calls", []) or []
                if any("gen_" in str(p) or p == "generate_music" for p in patterns):
                    return True
                if any("gen_" in str(c) or c == "generate_music" for c in calls):
                    return True
            return False

        assert _has_gen_step(suite.get("steps", [])), (
            "Suite has no gen_* / generate_music step — runner won't exercise "
            "the new MCP tools."
        )
        assert _has_gen_step(acceptance.get("steps", [])), (
            "Acceptance has no gen_* / generate_music step — validator won't "
            "check the new MCP tools."
        )