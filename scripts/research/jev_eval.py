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

Реальные провайдеры (PR-7 #3564): ключи из env или ``--env-file`` (значения не печатаются), отчёт markdown со
сравнением с deterministic: точность на всём наборе и на неоднозначном, выборы вне кандидатов, p50/p95, токены и цена,
критерии Go §4:

  --provider jev          JEV_API_KEY (JEV_MODEL, JEV_BASE_URL — опционально)
  --provider llm_harness  LLM_HARNESS_BASE_URL, LLM_HARNESS_MODEL, LLM_HARNESS_API_KEY
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import time
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

    def choose(self, question, *, deadline_ms=None):
        from rob_box_music.choice import Choice

        self.calls += 1
        if self.kind == "fake-down":
            raise ProviderUnavailable("offline run")
        return Choice(question.options[-1].id, 0.9, "fake")


#: Цены, USD за 1 млн токенов (вход, выход). НЕ СВЕРЕНО с прайсом провайдеров — для порядка величины; перед решением
#: Go сверить с актуальным тарифом TypeSafe и выбранного chat-провайдера.
PRICE_PER_MTOK = {"jev": (1.0, 3.0), "llm_harness": (0.30, 1.20)}
#: Допущение для проекции «≤ $1/день» (§4): неоднозначных заказов в сутки. НЕ измерено на роботе.
CALLS_PER_DAY = 200
GO = {"accuracy_gain_pp": 10.0, "out_of_candidates": 0, "p95_ms": 500.0, "usd_per_day": 1.0}


def load_env_file(path):
    """``KEY=VALUE`` из файла в ``os.environ`` (заданное не затирает); значения нигде не печатаются."""
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            key, _, value = line.partition("=")
            os.environ.setdefault(key.removeprefix("export ").strip(), value.strip().strip("\"'"))


class RecordingProvider:
    """Обёртка над провайдером decision layer: время каждого вызова, токены и ответы вне перечня кандидатов
    (до проверки оркестратора, который такие ответы отбрасывает, — иначе в отчёте их не видно)."""

    def __init__(self, inner):
        self.name, self.inner = inner.name, inner
        self.calls, self.errors, self.out_of_candidates = 0, 0, 0
        self.ms, self.input_tokens, self.output_tokens = [], 0, 0

    async def decide(self, state, questions):
        self.calls += 1
        started = time.perf_counter()
        try:
            decision = await self.inner.decide(state, questions)
        except BaseException:
            self.errors += 1
            raise
        finally:
            self.ms.append((time.perf_counter() - started) * 1000.0)
        self.input_tokens += int(decision.usage.get("input_tokens") or 0)
        self.output_tokens += int(decision.usage.get("output_tokens") or 0)
        self.out_of_candidates += sum(getattr(a, "answer", None) not in questions[q].options
                                      for q, a in decision.answers.items())
        return decision


def percentile(values, pct):
    ordered = sorted(values)
    return ordered[max(1, -(-pct * len(ordered) // 100)) - 1] if ordered else None


def accuracy(rows, ids=None):
    checked = [r for r in rows if not r["context"] and (ids is None or r["id"] in ids)]
    return sum(r["ok"] for r in checked), len(checked)


def run_music(args, provider=None):
    """Два прогона набора: детерминированный и с моделью (``provider`` — подставной в тестах)."""
    import music_eval
    from rob_box_mcp_tools.engine import chooser_model
    from rob_box_music.choice import DeterministicChooser

    chain = music_eval.offline_chain(args.scores)
    items = music_eval.load(args.dataset)
    base = music_eval.evaluate(items, chain, DeterministicChooser())
    if provider is None:
        provider = chooser_model.build_provider(args.provider, os.environ, timeout_s=args.deadline_ms / 1000.0 + 0.1)
    if provider is None:
        sys.exit(f"провайдер {args.provider} не настроен: нет ключа/конфига в env (--env-file PATH). "
                 "jev: JEV_API_KEY; llm_harness: LLM_HARNESS_BASE_URL, LLM_HARNESS_MODEL, LLM_HARNESS_API_KEY")
    recorder = RecordingProvider(provider)
    model = chooser_model.ModelChooser(recorder, deadline_ms=args.deadline_ms)
    report = music_eval.evaluate(items, chain, DeterministicChooser(model))
    return {"provider": args.provider, "deterministic": base, "model": report, "recorder": recorder,
            "counters": model.orchestrator.metrics.counters()}


def pct(done, total):
    return 100.0 * done / total if total else 0.0


def _ms(value):
    return "—" if value is None else f"{value:.0f} мс"


def go_verdict(result):
    """Критерии Go §4: ``[(критерий, выполнен, измерено)]``. Цена — по несверенным константам."""
    det, rec, name = result["deterministic"], result["recorder"], result["provider"]
    amb = set(det["summary"]["ambiguous"])
    (d_ok, d_n), (m_ok, m_n) = accuracy(det["rows"], amb), accuracy(result["model"]["rows"], amb)
    gain = pct(m_ok, m_n) - pct(d_ok, d_n)
    p95 = percentile(rec.ms, 95)
    price_in, price_out = PRICE_PER_MTOK[name]
    per_call = (rec.input_tokens * price_in + rec.output_tokens * price_out) / 1e6 / rec.calls if rec.calls else 0.0
    daily = per_call * CALLS_PER_DAY
    return [(f"точность на неоднозначном >= детерминированной + {GO['accuracy_gain_pp']:g} п.п.",
             gain >= GO["accuracy_gain_pp"], f"{gain:+.1f} п.п. при N={m_n}"),
            ("0 выборов вне кандидатов", rec.out_of_candidates == GO["out_of_candidates"], str(rec.out_of_candidates)),
            (f"p95 <= {GO['p95_ms']:g} мс", p95 is not None and p95 <= GO["p95_ms"], _ms(p95)),
            (f"<= ${GO['usd_per_day']:g}/день (допущение {CALLS_PER_DAY} вызовов/день)",
             daily <= GO["usd_per_day"], f"${daily:.4f}, цены НЕ сверены")]


def music_markdown(result):
    """Отчёт: точность на всём наборе и неоднозначном подмножестве, вне-кандидатов, p50/p95, цена, критерии Go."""
    det, model, rec, name = result["deterministic"], result["model"], result["recorder"], result["provider"]
    amb = set(det["summary"]["ambiguous"])
    changed = [d["id"] for d, m in zip(det["rows"], model["rows"])
               if (d["got"].get("choice") or {}).get("id") != (m["got"].get("choice") or {}).get("id")]
    price_in, price_out = PRICE_PER_MTOK[name]
    cost = (rec.input_tokens * price_in + rec.output_tokens * price_out) / 1e6
    lines = [f"# Chooser: {name} против deterministic (ADR-0157 §4)", "",
             f"Набор: {len(det['rows'])} реплик, проверяемых {accuracy(det['rows'])[1]}, неоднозначных {len(amb)} "
             f"({', '.join(sorted(amb)) or '—'}). Оффлайн-каталог, партитуры из --scores.", "",
             "| выбор | верно на всём наборе | верно на неоднозначном |", "|---|---|---|"]
    for label, rep in (("deterministic", det), (name, model)):
        (a_ok, a_n), (b_ok, b_n) = accuracy(rep["rows"]), accuracy(rep["rows"], amb)
        lines.append(f"| {label} | {a_ok}/{a_n} ({pct(a_ok, a_n):.1f}%) | {b_ok}/{b_n} ({pct(b_ok, b_n):.1f}%) |")
    lines += ["", f"Выбор модели отличается от детерминированного в {len(changed)} репликах: "
              f"{', '.join(changed) or '—'}.", "", "| метрика | значение |", "|---|---|",
              f"| вызовов модели | {rec.calls} (ошибок транспорта/протокола {rec.errors}) |",
              f"| выборов вне кандидатов | {rec.out_of_candidates} |",
              f"| исходы оркестратора | {json.dumps(result['counters'], ensure_ascii=False, sort_keys=True)} |",
              f"| латентность p50 / p95 | {_ms(percentile(rec.ms, 50))} / {_ms(percentile(rec.ms, 95))} |",
              f"| токены вход / выход | {rec.input_tokens} / {rec.output_tokens} |",
              f"| цена прогона (НЕ сверено: {price_in:g}/{price_out:g} USD за Мтокен вх./вых.) | ${cost:.4f} |", "",
              "## Критерии Go (§4)", "", "| критерий | выполнен | измерено |", "|---|---|---|"]
    lines += [f"| {c} | {'да' if ok else 'нет'} | {value} |" for c, ok, value in go_verdict(result)]
    lines += ["", f"Малая выборка (неоднозначных {len(amb)}): решение Go/No-go принимает Шифу после shadow на роботе; "
              "этот отчёт предварительный."]
    return "\n".join(lines) + "\n"


def music_main(args):
    """``--domain music``: набор §4.1. ``deterministic``/``fake-*`` — проверка конвейера; ``jev``/``llm_harness`` —
    реальный провайдер (ключ из env/``--env-file``), отчёт markdown со сравнением с deterministic."""
    import music_eval
    from rob_box_music.choice import DeterministicChooser

    if args.env_file:
        load_env_file(args.env_file)
    if args.provider in ("jev", "llm_harness"):
        text = music_markdown(run_music(args))
        if args.out:
            args.out.write_text(text, encoding="utf-8")
        sys.stdout.write(text)
        return
    if args.provider not in ("deterministic", "fake-down", "fake-last"):
        sys.exit(f"--domain music: провайдер {args.provider} не поддержан")
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
                        choices=["fake-agree", "fake-down", "fake-last", "deterministic", "jev", "laya", "llm_harness"])
    parser.add_argument("--scenarios", type=Path, default=DEFAULT_SCENARIOS)
    parser.add_argument("--dataset", type=Path,
                        default=REPO / "src" / "rob_box_music" / "test" / "data" / "music_requests.yaml")
    parser.add_argument("--scores", help="music: каталог библиотеки партитур (score_index.db), иначе без партитур")
    parser.add_argument("--rows", action="store_true", help="music: печатать строки, не только сводку")
    parser.add_argument("--env-file", type=Path,
                        help="файл KEY=VALUE с ключами (JEV_API_KEY, LLM_HARNESS_*); значения не печатаются")
    parser.add_argument("--out", type=Path, help="music + реальный провайдер: записать markdown-отчёт ещё и в файл")
    parser.add_argument("--deadline-ms", type=int, default=None,
                        help="дедлайн ответа модели, мс (по умолчанию: music = CHOOSER_DEADLINE_MS, иначе 2000)")
    parser.add_argument("--threshold", type=float, default=0.5)
    args = parser.parse_args(argv)
    if args.domain == "music":
        from rob_box_music.knowledge import CHOOSER_DEADLINE_MS
        args.deadline_ms = args.deadline_ms or CHOOSER_DEADLINE_MS
    else:
        args.deadline_ms = args.deadline_ms or 2000
    if args.domain == "music":
        return music_main(args)
    if args.env_file:
        load_env_file(args.env_file)
    if args.provider in ("deterministic", "fake-last", "llm_harness"):
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
