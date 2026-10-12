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

Музыкальные вопросы (ADR-0157 §4.2, PR-3): ``--domain music`` — режим/инструмент/стиль грамматикой и выбор
произведения ``Chooser`` на наборе ``src/rob_box_music/test/data/music_requests.yaml`` (``music_eval.py``):

  --provider deterministic   только веса (§3.3)
  --provider fake-down       модель неоднозначных вопросов падает → детерминированный ответ
  --provider fake-last       модель неоднозначных вопросов отвечает последним вариантом (видно, где решает она)

Реальные провайдеры (``llm_harness``/``jev``) для музыки — PR-7.
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


class _MusicFake:
    """Подставная модель выбора произведения (контракт ``rob_box_music.choice.Chooser``)."""

    def __init__(self, kind):
        self.kind, self.calls = kind, 0

    def choose(self, question, *, deadline_ms=500):
        from rob_box_music.choice import Choice

        self.calls += 1
        if self.kind == "fake-down":
            raise ProviderUnavailable("offline run")
        return Choice(question.options[-1].id, 0.9, "fake")


def music_main(args):
    """``--domain music``: набор §4.1, выбор ``DeterministicChooser`` (с подставной моделью — ``fake-*``)."""
    import time

    import music_eval
    from rob_box_music.choice import DeterministicChooser

    if args.provider not in ("deterministic", "fake-down", "fake-last"):
        sys.exit(f"--domain music: провайдер {args.provider} — PR-7 (здесь deterministic | fake-down | fake-last)")
    model = None if args.provider == "deterministic" else _MusicFake(args.provider)
    started = time.perf_counter()
    report = music_eval.evaluate(music_eval.load(args.dataset), music_eval.offline_chain(args.scores),
                                 DeterministicChooser(model))
    report["summary"].update(provider=args.provider, model_calls=model.calls if model else 0,
                             total_s=round(time.perf_counter() - started, 2))
    json.dump(report if args.rows else report["summary"], sys.stdout, ensure_ascii=False, indent=2)
    sys.stdout.write("\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--domain", choices=["acceptance", "music"], default="acceptance")
    parser.add_argument("--provider", required=True,
                        choices=["fake-agree", "fake-down", "fake-last", "deterministic", "jev", "laya"])
    parser.add_argument("--scenarios", type=Path, default=DEFAULT_SCENARIOS)
    parser.add_argument("--dataset", type=Path,
                        default=REPO / "src" / "rob_box_music" / "test" / "data" / "music_requests.yaml")
    parser.add_argument("--scores", help="music: каталог библиотеки партитур (score_index.db), иначе без партитур")
    parser.add_argument("--rows", action="store_true", help="music: печатать строки, не только сводку")
    parser.add_argument("--deadline-ms", type=int, default=2000)
    parser.add_argument("--threshold", type=float, default=0.5)
    args = parser.parse_args(argv)
    if args.domain == "music":
        return music_main(args)
    if args.provider in ("deterministic", "fake-last"):
        sys.exit(f"--domain acceptance: провайдер {args.provider} не поддержан")

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
