#!/usr/bin/env python3
"""Офлайн/онлайн прогон acceptance-сценариев через decision layer (issue #3084).

Провайдер выбирается явно, чтобы «замер» нельзя было перепутать с фейком:

  --provider fake-agree   офлайн: модель отвечает по разметке (санити-чек)
  --provider fake-down    офлайн: модель недоступна → только fallback
  --provider jev          реальный Jev, нужен JEV_API_KEY (JEV_MODEL опционально)
  --provider laya         реальная Laya через Ollaya, нужен LAYA_BASE_URL

Пример:
  PYTHONPATH=src/rob_box_harness:src/rob_box_llm:src/rob_box_core \\
    python scripts/research/jev_eval.py --provider fake-down

Вывод — JSON с построчными результатами и сводкой (accuracy, false_safe,
false_danger, human_review, p50/p95, исходы провайдера, токены).
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from pathlib import Path

import yaml

from rob_box_harness.core.confirmation_policy import ConfirmationKind, load_default_policy
from rob_box_harness.decision import (
    ClassRoute,
    DecisionClass,
    DecisionOrchestrator,
    FakeDecisionProvider,
    NoulAnswer,
    RoutingConfig,
    jev_from_env,
    laya_from_env,
)
from rob_box_harness.decision.evaluation import Scenario, evaluate_acceptance
from rob_box_harness.decision.types import ProviderUnavailable

REPO = Path(__file__).resolve().parents[2]
DEFAULT_SCENARIOS = REPO / "docs" / "research" / "jev" / "scenarios" / "acceptance.yaml"


class _AgreeingFake(FakeDecisionProvider):
    """Фейк, отвечающий по разметке сценария (state несёт expected)."""

    def __init__(self, expected_by_tool_args):
        super().__init__("jev")
        self._expected = expected_by_tool_args

    async def decide(self, state, questions):
        key = json.dumps([state["tool"], state["args"], state["context"]], sort_keys=True, ensure_ascii=False)
        p = 0.95 if self._expected[key] is ConfirmationKind.REQUIRE else 0.05
        self._answers = {"needs_review": NoulAnswer(p)}
        return await super().decide(state, questions)


def _build_provider(kind, scenarios, timeout_s):
    if kind == "fake-agree":
        expected = {
            json.dumps([s.tool, dict(s.args), dict(s.context)], sort_keys=True, ensure_ascii=False): s.expected
            for s in scenarios
        }
        return "jev", _AgreeingFake(expected)
    if kind == "fake-down":
        return "jev", FakeDecisionProvider("jev", error=ProviderUnavailable("offline run"))
    factory = jev_from_env if kind == "jev" else laya_from_env
    provider = factory(timeout_s=timeout_s)
    if provider is None:
        sys.exit(f"provider {kind} is not configured (see --help)")
    return provider.name, provider


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--provider", required=True, choices=["fake-agree", "fake-down", "jev", "laya"])
    parser.add_argument("--scenarios", type=Path, default=DEFAULT_SCENARIOS)
    parser.add_argument("--deadline-ms", type=int, default=2000)
    parser.add_argument("--threshold", type=float, default=0.5)
    args = parser.parse_args(argv)

    raw = yaml.safe_load(args.scenarios.read_text(encoding="utf-8"))
    scenarios = [Scenario.from_mapping(item) for item in raw["scenarios"]]
    name, provider = _build_provider(args.provider, scenarios, args.deadline_ms / 1000.0)
    # Для замера safety-класс маршрутизируется на выбранного провайдера.
    # Laya в safety_critical запрещена конфигом — здесь это офлайн-оценка,
    # а не боевой маршрут, поэтому RoutingConfig собирается напрямую.
    route = ClassRoute((name,), args.deadline_ms, 0.0)
    orchestrator = DecisionOrchestrator({name: provider}, routing=RoutingConfig({c: route for c in DecisionClass}))
    report = asyncio.run(evaluate_acceptance(scenarios, orchestrator, load_default_policy(), threshold=args.threshold))
    json.dump(report, sys.stdout, ensure_ascii=False, indent=2, default=lambda o: getattr(o, "value", str(o)))
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
