"""Routing-конфиг и acceptance PoC decision layer (issue #3084).

Доказываем инварианты, которые модель ослабить не может:
* ``stop_navigation`` не зависит от сигнала вообще;
* ``require`` не понижается;
* Laya не допускается в safety_critical конфигом.
"""

from __future__ import annotations

import math

import pytest
import yaml

from rob_box_harness.core.confirmation_policy import (
    EMERGENCY_TOOLS,
    AcceptanceDecision,
    ConfirmationKind,
    load_default_policy,
)
from rob_box_harness.decision.acceptance_escalation import escalate_acceptance
from rob_box_harness.decision.routing import DEFAULT_ROUTES, RoutingConfig
from rob_box_harness.decision.types import DecisionClass

# --- routing ----------------------------------------------------------------

YAML_OK = """
decision_routing:
  safety_critical:
    providers: [jev, deterministic]
    deadline_ms: 300
    min_confidence: 0.95
  latency_critical:
    providers: [laya, deterministic]
  quality_critical:
    providers: [jev, laya, deterministic]
"""


def test_routing_from_yaml():
    cfg = RoutingConfig.from_mapping(yaml.safe_load(YAML_OK))
    safety = cfg.route(DecisionClass.SAFETY_CRITICAL)
    assert safety.providers == ("jev",)
    assert (safety.deadline_ms, safety.min_confidence) == (300, 0.95)
    assert cfg.route(DecisionClass.QUALITY_CRITICAL).providers == ("jev", "laya")
    assert (
        cfg.route(DecisionClass.LATENCY_CRITICAL).deadline_ms
        == DEFAULT_ROUTES[DecisionClass.LATENCY_CRITICAL].deadline_ms
    )


def test_default_safety_route_excludes_laya():
    assert "laya" not in RoutingConfig.default().route(DecisionClass.SAFETY_CRITICAL).providers


@pytest.mark.parametrize(
    "section",
    [
        {"safety_critical": {"providers": ["laya"]}},
        {"safety_critical": {"providers": ["jev", "laya", "deterministic"]}},
        {"quality_critical": {"providers": ["deterministic", "jev"]}},
        {"quality_critical": {"providers": ["jev", "jev"]}},
        {"quality_critical": {"deadline_ms": 30000}},
        {"quality_critical": {"deadline_ms": 0}},
        {"quality_critical": {"min_confidence": 1.5}},
        {"quality_critical": {"providers": "jev"}},
        {"made_up_class": {"providers": ["jev"]}},
    ],
)
def test_invalid_routing_fails_loud(section):
    with pytest.raises(ValueError):
        RoutingConfig.from_mapping({"decision_routing": section})


# --- acceptance escalation ---------------------------------------------------

POLICY = load_default_policy()


def _decision(kind, tool="navigate_to_waypoint"):
    return AcceptanceDecision(kind=kind, tool=tool, reason="policy")


@pytest.mark.parametrize("signal", [None, 0.0, 0.49, 0.5, 0.99, 1.0, math.nan])
def test_stop_navigation_never_depends_on_signal(signal):
    for tool in EMERGENCY_TOOLS:
        base = POLICY.classify(tool)
        assert escalate_acceptance(base, signal) is base
        assert base.kind is not ConfirmationKind.REQUIRE


@pytest.mark.parametrize("signal", [None, 0.0, 0.01, 0.99])
def test_require_is_never_downgraded(signal):
    base = _decision(ConfirmationKind.REQUIRE)
    assert escalate_acceptance(base, signal).kind is ConfirmationKind.REQUIRE


@pytest.mark.parametrize("kind", [ConfirmationKind.PASS_THROUGH, ConfirmationKind.NOTIFY])
def test_escalation_only_upgrades(kind):
    base = _decision(kind)
    assert escalate_acceptance(base, None) is base, "no model signal → original policy"
    assert escalate_acceptance(base, 0.2) is base
    upgraded = escalate_acceptance(base, 0.8)
    assert upgraded.kind is ConfirmationKind.REQUIRE
    assert upgraded.plan_text
    assert "escalated" in upgraded.reason


def test_nan_signal_is_treated_conservatively():
    # NaN не проходит «< threshold» → эскалация до require (безопасная сторона).
    assert escalate_acceptance(_decision(ConfirmationKind.NOTIFY), math.nan).kind is ConfirmationKind.REQUIRE


def test_every_catalog_tool_is_never_weakened():
    """Для каждого инструмента каталога и любого сигнала: kind не мягче политики."""
    rank = {ConfirmationKind.PASS_THROUGH: 0, ConfirmationKind.NOTIFY: 1, ConfirmationKind.REQUIRE: 2}
    for tool in sorted(POLICY.known_tools):
        base = POLICY.classify(tool)
        for signal in (None, 0.0, 0.3, 0.7, 1.0):
            assert rank[escalate_acceptance(base, signal).kind] >= rank[base.kind], (tool, signal)
