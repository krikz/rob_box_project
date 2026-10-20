"""Scheduler shadow PoC и офлайн-оценка acceptance (issue #3084 §C/§E)."""

from __future__ import annotations

import asyncio
from pathlib import Path

import pytest
import yaml

from rob_box_harness.core.confirmation_policy import ConfirmationKind, load_default_policy
from rob_box_harness.decision.evaluation import Scenario, evaluate_acceptance
from rob_box_harness.decision.orchestrator import DecisionOrchestrator
from rob_box_harness.decision.providers import FakeDecisionProvider
from rob_box_harness.decision.routing import ClassRoute, RoutingConfig
from rob_box_harness.decision.scheduler_shadow import shadow_quick_verdict
from rob_box_harness.decision.types import ChoiceAnswer, DecisionClass, NoulAnswer, ProviderUnavailable

SCENARIOS_FILE = Path(__file__).resolve().parents[3] / "docs" / "research" / "jev" / "scenarios" / "acceptance.yaml"


def _orch(provider, min_confidence=0.0):
    route = ClassRoute((provider.name,), 200, min_confidence)
    return DecisionOrchestrator({provider.name: provider}, routing=RoutingConfig({c: route for c in DecisionClass}))


# --- scheduler shadow ----------------------------------------------------------


def test_shadow_always_executes_baseline_even_when_model_disagrees():
    model = FakeDecisionProvider(
        "laya", answers={"verdict": ChoiceAnswer("REPLACE", {"REPLACE": 0.9, "IGNORE": 0.1}, 0.9)}
    )
    result = asyncio.run(
        shadow_quick_verdict(_orch(model), utterance="и ещё про енота", baseline_verdict="PENDING_LLM")
    )
    assert result.executed == "PENDING_LLM"
    assert result.model_verdict == "REPLACE"
    assert result.agreed is False


def test_shadow_model_down_reports_no_verdict():
    down = FakeDecisionProvider("laya", error=ProviderUnavailable("down"))
    result = asyncio.run(shadow_quick_verdict(_orch(down), utterance="стоп", baseline_verdict="REPLACE"))
    assert (result.executed, result.model_verdict, result.agreed) == ("REPLACE", None, None)
    assert result.decision.fallback_used


def test_shadow_rejects_unknown_baseline():
    with pytest.raises(ValueError):
        asyncio.run(shadow_quick_verdict(_orch(FakeDecisionProvider("laya")), utterance="x", baseline_verdict="MERGE"))


# --- acceptance evaluation -------------------------------------------------------


def _scenarios():
    raw = yaml.safe_load(SCENARIOS_FILE.read_text(encoding="utf-8"))
    return [Scenario.from_mapping(item) for item in raw["scenarios"]]


def test_scenarios_file_covers_issue_cases():
    tools = {s.tool for s in _scenarios()}
    for required in (
        "speak_text",
        "stop_music",
        "navigate_to_waypoint",
        "move_direction",
        "save_waypoint",
        "delete_waypoint",
        "clear_waypoints",
        "start_mapping",
        "finish_mapping",
        "stop_navigation",
    ):
        assert required in tools


def test_evaluation_with_model_down_equals_policy():
    report = asyncio.run(
        evaluate_acceptance(
            _scenarios(), _orch(FakeDecisionProvider("jev", error=ProviderUnavailable("x"))), load_default_policy()
        )
    )
    rows = report["rows"]
    assert all(r["guarded"] is r["deterministic"] for r in rows)
    assert report["summary"]["model_only"]["answered"] == 0
    assert report["summary"]["provider_outcomes"]["fallback_used"] == len(rows)


def test_evaluation_model_always_yes_never_weakens_stop_navigation():
    yes = FakeDecisionProvider("jev", answers={"needs_review": NoulAnswer(1.0)})
    report = asyncio.run(evaluate_acceptance(_scenarios(), _orch(yes), load_default_policy()))
    for row in report["rows"]:
        if row["tool"] == "stop_navigation":
            assert row["guarded"] is ConfirmationKind.PASS_THROUGH
            assert row["model_only"] is ConfirmationKind.REQUIRE  # вот почему model-only запрещён
    # «всегда да» эскалирует и delete_waypoint → опасных промахов нет, но review растёт.
    assert report["summary"]["guarded"]["false_safe"] == []
    assert report["summary"]["guarded"]["human_review"] > report["summary"]["deterministic"]["human_review"]


def test_evaluation_model_always_no_cannot_weaken_policy():
    no = FakeDecisionProvider("jev", answers={"needs_review": NoulAnswer(0.0)})
    report = asyncio.run(evaluate_acceptance(_scenarios(), _orch(no), load_default_policy()))
    summary = report["summary"]
    assert summary["guarded"] == summary["deterministic"]
    assert len(summary["model_only"]["false_safe"]) > len(summary["deterministic"]["false_safe"])
