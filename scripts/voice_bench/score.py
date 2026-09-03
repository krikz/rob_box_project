#!/usr/bin/env python3
"""Градиентная оценка прогона бенчмарка голосового диалога.

Почему градиент, а не pass/fail
-------------------------------

Бинарная оценка (как в BFCL, где несовпадение набора вызовов = 0) хороша
для лидерборда из тысяч задач: там градиент получается агрегацией. У нас
кейсов два десятка, и «провалено 3 из 19» не отличает ход, где робот
промолчал, от хода, где он вызвал правильный инструмент, но заболтал
пользователя на три абзаца. Поэтому:

* **внутри кейса** — частичный кредит по взвешенным критериям (0..1);
* **между повторами** — надёжность в духе ``pass^k`` из τ-bench: доля
  кейсов, где ВСЕ k повторов выше порога. Один удачный прогон из трёх —
  это не 33% качества, это ненадёжный робот;
* **качественная часть** — рубрика с обоснованием, оценки хранятся рядом
  с сырьём, чтобы любую можно было перепроверить.

Критерии и веса живут здесь, а не в головах: любое изменение оценки видно
диффом.
"""
from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

#: Веса критериев. Сумма — 1.0. Выбор инструмента весит больше всего,
#: потому что именно «ответил словами, не вызвав ничего» — тот дефект,
#: ради которого весь бенчмарк и существует.
WEIGHTS: dict[str, float] = {
    "tool_choice": 0.45,
    "prohibitions": 0.15,
    "speech_form": 0.15,
    "efficiency": 0.10,
    "stability": 0.15,
}

#: Порог, выше которого ход считается удачным для pass^k.
PASS_THRESHOLD = 0.8

#: Максимум символов речи по §3 BREVITY мастер-промпта.
DEFAULT_MAX_SPEECH = 200

_CODE_MARKERS = ("p1 >>", "d1 >>", "Clock.bpm", "```", "Scale.default")


@dataclass
class Criterion:
    """Один критерий с баллом 0..1 и объяснением, почему именно столько."""

    name: str
    score: float
    why: str


@dataclass
class CaseResult:
    case_id: str
    domain: str
    repeat: int
    criteria: list[Criterion] = field(default_factory=list)

    @property
    def score(self) -> float:
        return round(
            sum(WEIGHTS[c.name] * c.score for c in self.criteria), 3
        )

    @property
    def failures(self) -> list[str]:
        return [f"{c.name}={c.score:.2f} ({c.why})" for c in self.criteria if c.score < 1.0]


def _degenerate(text: str) -> bool:
    """Вырожденный текст: коротко и почти без букв — как ``[e~[``.

    Живой инцидент 02.09: модель вернула такой текст вместе с валидными
    тул-вызовами, он уехал в Telegram и попал в историю, где размножился
    на тридцать ходов.
    """
    body = text.strip()
    if not body or len(body) > 12:
        return False
    alnum = sum(ch.isalnum() for ch in body)
    return alnum / len(body) < 0.5


def score_tool_choice(case: Mapping[str, Any], turn: Mapping[str, Any]) -> Criterion:
    wanted = list(case.get("tools_any") or [])
    called = list(turn.get("tools") or [])
    if not wanted:
        return Criterion("tool_choice", 1.0, "кейс не требует инструментов")
    hit = [t for t in called if t in wanted]
    if hit:
        return Criterion("tool_choice", 1.0, f"вызван {', '.join(hit)}")
    if called:
        # Инструменты были, но не те: домен нащупан, действие не то.
        return Criterion(
            "tool_choice", 0.35, f"вызваны только чужие: {', '.join(called)}"
        )
    return Criterion("tool_choice", 0.0, "НИ ОДНОГО вызова — ответ словами")


def score_prohibitions(case: Mapping[str, Any], turn: Mapping[str, Any]) -> Criterion:
    banned = set(case.get("tools_none") or [])
    called = set(turn.get("tools") or [])
    bad = sorted(banned & called)
    if not bad:
        return Criterion("prohibitions", 1.0, "запрещённых вызовов нет")
    # Каждый запрещённый вызов режет половину оставшегося.
    return Criterion(
        "prohibitions", max(0.0, 1.0 - 0.5 * len(bad)), f"вызвано запрещённое: {', '.join(bad)}"
    )


def score_speech_form(case: Mapping[str, Any], turn: Mapping[str, Any]) -> Criterion:
    text = str(turn.get("spoken") or "")
    limit = int(case.get("max_speech") or DEFAULT_MAX_SPEECH)
    penalties: list[str] = []
    score = 1.0
    if _degenerate(text):
        score -= 0.6
        penalties.append(f"вырожденный текст {text!r}")
    if len(text) > limit:
        score -= 0.3
        penalties.append(f"речь {len(text)} симв > {limit}")
    if any(m in text for m in _CODE_MARKERS):
        score -= 0.6
        penalties.append("код в речи — робот зачитает его вслух")
    return Criterion(
        "speech_form", max(0.0, round(score, 2)), "; ".join(penalties) or "форма ответа в норме"
    )


def score_efficiency(case: Mapping[str, Any], turn: Mapping[str, Any]) -> Criterion:
    n = int(turn.get("llm_requests") or 1)
    for limit, value in ((2, 1.0), (3, 0.7), (5, 0.4)):
        if n <= limit:
            return Criterion("efficiency", value, f"{n} обращений к LLM")
    return Criterion("efficiency", 0.0, f"{n} обращений — ход почти упёрся в лимит цикла")


def score_stability(case: Mapping[str, Any], turn: Mapping[str, Any]) -> Criterion:
    events = {
        "срез tool-call": int(turn.get("truncated") or 0),
        "ретрай без музыки": int(turn.get("retry_no_music") or 0),
        "падение провайдера": int(turn.get("provider_fallback") or 0),
        "ошибка хода": 1 if turn.get("error") else 0,
    }
    hits = {k: v for k, v in events.items() if v}
    if not hits:
        return Criterion("stability", 1.0, "без ретраев и падений")
    total = sum(hits.values())
    return Criterion(
        "stability",
        max(0.0, 1.0 - 0.34 * total),
        ", ".join(f"{k}×{v}" for k, v in hits.items()),
    )


SCORERS = (
    score_tool_choice,
    score_prohibitions,
    score_speech_form,
    score_efficiency,
    score_stability,
)


def score_turn(case: Mapping[str, Any], turn: Mapping[str, Any]) -> CaseResult:
    """Оценить один ход одного кейса."""
    result = CaseResult(
        case_id=str(case.get("id")),
        domain=str(case.get("domain") or "—"),
        repeat=int(turn.get("repeat") or 0),
    )
    result.criteria = [fn(case, turn) for fn in SCORERS]
    return result


def pass_hat_k(results: Sequence[CaseResult], threshold: float = PASS_THRESHOLD) -> float:
    """Доля кейсов, где ВСЕ повторы прошли порог (надёжность, τ-bench).

    Один удачный прогон из трёх — не «33% качества», а ненадёжный робот,
    и метрика обязана это показывать.
    """
    by_case: dict[str, list[float]] = {}
    for r in results:
        by_case.setdefault(r.case_id, []).append(r.score)
    if not by_case:
        return 0.0
    solid = sum(1 for scores in by_case.values() if all(s >= threshold for s in scores))
    return round(solid / len(by_case), 3)


def report(cases: Sequence[Mapping[str, Any]], run: Mapping[str, Any]) -> dict[str, Any]:
    index = {str(c["id"]): c for c in cases}
    results: list[CaseResult] = []
    for turn in run.get("turns", []):
        case = index.get(str(turn.get("case_id")))
        if case is None:
            continue
        results.append(score_turn(case, turn))

    by_domain: dict[str, list[float]] = {}
    for r in results:
        by_domain.setdefault(r.domain, []).append(r.score)

    return {
        "config": run.get("config", {}),
        "turns": len(results),
        "mean": round(statistics.fmean(r.score for r in results), 3) if results else 0.0,
        "pass_hat_k": pass_hat_k(results),
        "by_domain": {
            d: round(statistics.fmean(v), 3) for d, v in sorted(by_domain.items())
        },
        "results": results,
    }


def _print(rep: Mapping[str, Any], verbose: bool) -> None:
    cfg = rep.get("config") or {}
    print(f"конфигурация : {json.dumps(cfg, ensure_ascii=False)}")
    print(f"ходов        : {rep['turns']}")
    print(f"средний балл : {rep['mean']:.3f}   (0..1, частичный кредит)")
    print(f"pass^k       : {rep['pass_hat_k']:.3f}   (доля кейсов, где ВСЕ повторы ≥ {PASS_THRESHOLD})")
    print("\nпо доменам:")
    for domain, value in rep["by_domain"].items():
        bar = "█" * int(round(value * 20))
        print(f"  {domain:14} {value:.3f}  {bar}")
    weak = sorted(rep["results"], key=lambda r: r.score)[:8]
    print("\nсамые слабые ходы:")
    for r in weak:
        print(f"  {r.score:.3f}  {r.case_id}#{r.repeat}")
        for line in r.failures:
            print(f"          {line}")
    if verbose:
        print("\nвсе ходы:")
        for r in rep["results"]:
            print(f"  {r.score:.3f}  {r.case_id}#{r.repeat}  {r.domain}")


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cases", required=True, type=Path)
    ap.add_argument("--run", required=True, type=Path, help="JSON от run_bench.py")
    ap.add_argument("--compare", type=Path, help="второй прогон для сравнения A/B")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)

    import yaml  # локальный импорт: скорер работает и без PyYAML в тестах

    cases = yaml.safe_load(args.cases.read_text(encoding="utf-8"))
    rep_a = report(cases, json.loads(args.run.read_text(encoding="utf-8")))
    _print(rep_a, args.verbose)

    if args.compare:
        rep_b = report(cases, json.loads(args.compare.read_text(encoding="utf-8")))
        print("\n" + "=" * 62)
        _print(rep_b, args.verbose)
        print("\n" + "=" * 62)
        print("A/B (второй минус первый):")
        print(f"  средний балл : {rep_b['mean'] - rep_a['mean']:+.3f}")
        print(f"  pass^k       : {rep_b['pass_hat_k'] - rep_a['pass_hat_k']:+.3f}")
        domains = sorted(set(rep_a["by_domain"]) | set(rep_b["by_domain"]))
        for d in domains:
            a = rep_a["by_domain"].get(d, 0.0)
            b = rep_b["by_domain"].get(d, 0.0)
            print(f"  {d:14} {b - a:+.3f}   ({a:.3f} → {b:.3f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
