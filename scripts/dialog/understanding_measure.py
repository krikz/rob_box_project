#!/usr/bin/env python3
"""Замер «кто понимает фразу»: код по словам или модель (карточка #4163, этап 1).

    python scripts/dialog/understanding_measure.py --variants A,B --runs 1 --budget 800 --out ./out        # DeepSeek
    python scripts/dialog/understanding_measure.py --variants A,B,C --fake oracle --out ./out              # без ключа
    python scripts/dialog/understanding_measure.py --variants A --live-dir tests/e2e/_artifacts/night_20261028_092532

Корпус - ``understanding_corpus/marathon.json`` (часть 1, заморожен коммитом, хеш сверяется при загрузке). Каждый
вариант (``understanding_variants``: A - как сейчас, B - понимает модель, C - понимает Jev) проходит корпус N раз на своём
стенде, акты по порядку, как живой марафон: 90 оцениваемых текстовых шагов и 42 акустических как контекст состояния.
Вердикт шага - только ``dry_marathon.check_step`` (``e2e_tool_match.step_verdict`` / ``patterns_missing``): второй
реализации правил здесь нет. Упал стенд или судья - «не оценено», не FAIL. Метрики 1-6 карточки - ``understanding_metrics``.

``--budget`` - потолок вызовов LLM на ВСЕ варианты прогона (делится поровну между оставшимися вариантами: каждому не
больше (остаток) / (сколько вариантов ещё ждёт)); вариант C сверх того считает вызовы Jev отдельно (``--jev-budget``).
``--fake oracle|blind`` - фейковый провайдер вместо DeepSeek: проверка обвязки, а не качества (oracle отвечает по
ожиданиям шага, blind - всегда «Не знаю.»). Ключи берутся из ``scripts/dialogue/.env`` и окружения; значения не
печатаются и в отчёт не попадают.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE / "understanding_corpus"))

import build_marathon  # noqa: E402
import dry_marathon as dm  # noqa: E402
import jev_router  # noqa: E402
import live_compare  # noqa: E402
import understanding_metrics as um  # noqa: E402
import understanding_variants as uv  # noqa: E402
from dry_stand import REPO, DryStand, new_workdir  # noqa: E402
from env_files import load_env_files  # noqa: E402
from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog.words import strip_wake_word  # noqa: E402

CORPUS = _HERE / "understanding_corpus" / "marathon.json"
ENV_FILES = (REPO / "scripts" / "dialogue" / ".env", REPO / ".env")
#: Решение Шифу 28.10: B принимаем, если на фразах марафона он хуже A не больше чем на 3 п.п.
MAX_PASS_DROP_PP = 3.0
#: Допуск расхождения вердиктов A с живым марафоном (поручение координатора): больше - B как вывод не подаётся.
MAX_LIVE_DISAGREE = 0.10

Run = Dict[str, List[dm.StepReport]]


# ── Корпус ────────────────────────────────────────────────────────────────────────────────────────────────────────


def load_corpus(path: Path = CORPUS) -> Dict[str, Any]:
    """Корпус части 1; хеш содержимого обязан совпасть с записанным (корпус заморожен коммитом, руками не правится)."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if build_marathon.digest(data["acts"]) != data["corpus_sha256"]:
        raise SystemExit(f"{path}: хеш корпуса не совпал с corpus_sha256 - файл правили руками; пересобрать "
                         f"build_marathon.py и закоммитить")
    return data


def corpus_index(corpus: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    return {s["id"]: s for a in corpus["acts"] for s in a["steps"]}


def corpus_acts(corpus: Dict[str, Any], acts: Sequence[str] = (), only: Sequence[str] = ()) -> List[dm.Act]:
    """Акты для ``dry_marathon.run_act``: все шаги акта по порядку (контекст состояния нужен) или выбранные ``only``."""
    out = []
    for a in corpus["acts"]:
        if acts and str(a["act"]) not in acts:
            continue
        steps = [s["step"] for s in a["steps"] if not only or s["id"] in only or s["step"]["label"] in only]
        if steps:
            out.append(dm.Act(str(a["act"]), a["title"], CORPUS, steps, a["inherits"], a["registers_speakers"],
                              a["writes_memory"]))
    return out


# ── Фейковый провайдер (проверка обвязки) ─────────────────────────────────────────────────────────────────────────

#: Аргументы тулов, которых ждут шаги корпуса и у которых есть обязательные поля: фейк-oracle подставляет их по таблице.
ORACLE_ARGS: Dict[str, Dict[str, Any]] = {
    "memory_save": {"fact": "факт для проверки", "category": "preference"},
    "memory_search": {"query": "чай"},
    "search_web": {"query": "новости космоса"},
    "set_voice": {"voice": "default"},
    "set_volume": {"action": "louder"},
    "music_library": {"action": "list"},
}


class Oracle:
    """Знание фейка: реплика → шаг корпуса (по тексту после вейка); отвечает по ожиданиям шага, а не «понимает»."""

    def __init__(self, corpus: Dict[str, Any], wake: Sequence[str]) -> None:
        self._by_text = {strip_wake_word(s["step"]["text"], list(wake)): s for a in corpus["acts"]
                         for s in a["steps"]}

    def step(self, text: str) -> Optional[Dict[str, Any]]:
        hits = [k for k in self._by_text if k and k in text]
        return self._by_text[max(hits, key=len)] if hits else None

    def keywords(self, entry: Optional[Dict[str, Any]]) -> str:
        acc = (entry["step"].get("acceptance") or {}) if entry else {}
        return " ".join(acc.get("expected_keywords") or []) or "Хорошо."


def oracle_rule(oracle: Oracle, blind: bool) -> Callable[[List[Any], List[str]], Dict[str, Any]]:
    """Ответ фейковой модели. ``blind`` - всегда «Не знаю.»; иначе: ждёт тул из среза - зовёт его, ответ тула - речь."""

    def rule(msgs: List[Any], tools: List[str]) -> Dict[str, Any]:
        if blind:
            return {"speech": "Не знаю."}
        entry = next((e for e in (oracle.step(str(m.content)) for m in reversed(msgs) if m.role == "user") if e), None)
        if msgs[-1].role == "tool":
            return {"speech": oracle.keywords(entry)}
        wanted = [t for t in (um.expected_tools(entry) if entry else []) if t in tools]
        if wanted:  # речь вместе с действиями мост не принимает (speech_with_actions): слова - после данных тула
            return {"actions": [{"tool": t, "args": dict(ORACLE_ARGS.get(t, {}))} for t in wanted]}
        return {"speech": oracle.keywords(entry)}

    return rule


class OracleRouter:
    """Фейковый Jev: группа ожидаемого тула шага, уверенность 0.9; нет ожидания - ``none``."""

    def __init__(self, oracle: Oracle) -> None:
        self._oracle = oracle
        self.calls = 0

    async def choose(self, text: str, music_playing: bool = False) -> jev_router.Route:
        self.calls += 1
        entry = self._oracle.step(text)
        tools = um.expected_tools(entry) if entry else []
        group = next((g for t in tools for g, grp in kn.TOOL_GROUPS.items() if t in grp.tools), uv.NO_GROUP)
        return jev_router.Route(group, 0.9, True, 1.0)

    def stats(self) -> Dict[str, Any]:
        return {"calls": self.calls, "errors": 0, "input_tokens": 0, "output_tokens": 0, "threshold": 0.6,
                "language": "fake", "max_calls": None}


def fake_stand(rule: Callable[[List[Any], List[str]], Dict[str, Any]], workdir: Path, variant: str,
               router: Any, memory_path: Optional[Path] = None, clock: Any = None) -> DryStand:
    """Стенд с фейковым провайдером за НАСТОЯЩИМ мостом команд (как в ``tests/unit/dialog/test_dry_marathon.py``)."""
    from rob_box_llm.provider import LLMResponse, ProviderCapabilities, ToolCall  # noqa: PLC0415
    from rob_box_llm.providers.fake import FakeLLMProvider  # noqa: PLC0415
    from rob_box_voice.core import command_bridge  # noqa: PLC0415
    from rob_box_voice.core.dialog_v2_ports import CallCounter, Counted  # noqa: PLC0415

    counter = CallCounter()

    async def on_complete(msgs: Any, tools: Any, settings: Any) -> Any:
        args = rule(list(msgs), [t["function"]["name"] for t in tools])
        return LLMResponse(content="", tool_calls=(ToolCall("c1", "command", args),), finish_reason="tool_calls")

    inner = FakeLLMProvider(on_complete=on_complete, name="deepseek",
                            capabilities=ProviderCapabilities(tools=True, tool_choice=True))
    relay = dm.LogRelay()
    llm = command_bridge.wrapper_for("deepseek", log=relay)(Counted(inner, counter))
    stand = DryStand(llm, counter, "Ты робот.", workdir, variant=variant, router=router, memory_path=memory_path,
                     clock=clock)
    relay.target = stand
    return stand


# ── Прогон ────────────────────────────────────────────────────────────────────────────────────────────────────────


@dataclass
class VariantRun:
    variant: str
    run: int
    reports: Run
    llm_calls: int
    elapsed_s: float
    router: Optional[Dict[str, Any]] = None
    cap: int = 0
    notes: List[str] = field(default_factory=list)


StandFactory = Callable[[str], Tuple[DryStand, Any]]


def real_factory(args: argparse.Namespace, jev: Optional[Callable[[], Any]]) -> StandFactory:
    providers = [p for p in args.providers.split(",") if p.strip()]

    def make(variant: str) -> Tuple[DryStand, Any]:
        router = jev() if variant == uv.C and jev is not None else None
        return dm.build_real_stand(providers, new_workdir("understanding_"), args.deadline_s, variant=variant,
                                   router=router, model=args.model), router

    return make


def fake_factory(corpus: Dict[str, Any], blind: bool) -> StandFactory:
    def make(variant: str) -> Tuple[DryStand, Any]:
        from dry_stand import WAKE_WORDS_YAML  # noqa: PLC0415
        from rob_box_dialog.address import wake_words  # noqa: PLC0415

        oracle = Oracle(corpus, wake_words(str(WAKE_WORDS_YAML))[0])
        router = OracleRouter(oracle) if variant == uv.C else None
        return fake_stand(oracle_rule(oracle, blind), new_workdir("understanding_fake_"), variant, router), router

    return make


def jev_factory(args: argparse.Namespace) -> Callable[[], Any]:
    """Классификатор Jev (ключ ``JEV_API_KEY`` из env). Нет ключа - громкий отказ, а не тихий срез B."""
    from rob_box_harness.decision import jev_from_env  # noqa: PLC0415

    provider = jev_from_env(timeout_s=jev_router.DEFAULT_TIMEOUT_S)
    if provider is None:
        raise SystemExit("вариант C: нет JEV_API_KEY (scripts/dialogue/.env или окружение)")
    spec = jev_router.load_spec(Path(args.jev_groups))
    return lambda: jev_router.JevRouter(provider, spec, args.threshold, args.jev_budget)


async def run_variant(variant: str, run_no: int, acts: Sequence[dm.Act], make: StandFactory, cap: int,
                      max_retries: Optional[int]) -> VariantRun:
    """Один вариант, один прогон корпуса на свежем стенде; акты по порядку, состояние переходит между актами."""
    stand, router = make(variant)
    started = time.monotonic()
    run: Run = {}
    try:
        for act in acts:
            run[act.key] = await dm.run_act(stand, act, cap, max_retries)
            print(f"-- {variant} прогон {run_no}, акт {act.key}: запросов LLM {stand.counter.calls}/{cap}", flush=True)
    finally:
        calls = stand.counter.calls
        stand.close()
    return VariantRun(variant, run_no, run, calls, time.monotonic() - started,
                      router.stats() if router is not None else None, cap)


async def run_all(variants: Sequence[str], runs: int, acts: Sequence[dm.Act], make: StandFactory, budget: int,
                  max_retries: Optional[int]) -> List[VariantRun]:
    """Все варианты × N прогонов. Бюджет делится по очереди: каждому прогону - остаток / число ждущих прогонов."""
    plan = [(r, v) for r in range(1, runs + 1) for v in variants]
    spent, done = 0, []
    for n, (run_no, variant) in enumerate(plan):
        cap = (budget - spent) // (len(plan) - n)
        result = await run_variant(variant, run_no, acts, make, cap, max_retries)
        spent += result.llm_calls
        done.append(result)
    return done


# ── Отчёт ─────────────────────────────────────────────────────────────────────────────────────────────────────────


def step_record(act: str, report: dm.StepReport, entry: Dict[str, Any]) -> Dict[str, Any]:
    return {"id": um.key(act, report), "kind": report.kind, "scored": entry["scored"], "verdict": report.verdict,
            "first_verdict": report.first_verdict, "attempts": report.attempts, "fail": report.fail,
            "tools": report.tools, "said": report.said, "llm_calls": report.llm_calls, "turns": report.turns}


def price_line(stats: Dict[str, Any]) -> Dict[str, Any]:
    """Цена Jev по ``PRICE_PER_MTOK`` из ``scripts/research/jev_eval.py`` (там же оговорено: НЕ СВЕРЕНО с тарифом)."""
    sys.path.insert(0, str(REPO / "scripts" / "research"))
    try:
        from jev_eval import PRICE_PER_MTOK  # noqa: PLC0415
        price_in, price_out = PRICE_PER_MTOK["jev"]
    except Exception as exc:  # noqa: BLE001 — нет yaml/пакетов harness: цену не считаем, а не угадываем
        return {"usd": None, "note": f"цена не посчитана: {type(exc).__name__}"}
    usd = (stats["input_tokens"] * price_in + stats["output_tokens"] * price_out) / 1e6
    return {"usd": usd, "price_per_mtok_in_out": [price_in, price_out], "note": "PRICE_PER_MTOK jev_eval.py, НЕ СВЕРЕНО с тарифом"}


def build_report(results: Sequence[VariantRun], corpus: Dict[str, Any], args: argparse.Namespace,
                 live: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    index = corpus_index(corpus)
    variants: Dict[str, Any] = {}
    for variant in dict.fromkeys(r.variant for r in results):
        mine = [r for r in results if r.variant == variant]
        metrics = um.variant_metrics([r.reports for r in mine], index)
        items = [i for r in mine for i in um.scored_items(r.reports, index)]
        entry: Dict[str, Any] = {"metrics": metrics, "llm_calls_total": sum(r.llm_calls for r in mine),
                                 "elapsed_s": [round(r.elapsed_s, 1) for r in mine],
                                 "caps": [r.cap for r in mine],
                                 "runs": [[step_record(a, rep, index[um.key(a, rep)]) for a, reps in r.reports.items()
                                           for rep in reps if um.key(a, rep) in index] for r in mine]}
        if variant == uv.C:
            routers = [r.router for r in mine if r.router]
            jev = {k: sum(x[k] for x in routers) for k in ("calls", "errors", "input_tokens", "output_tokens")}
            entry["jev"] = {**jev, "threshold": routers[0]["threshold"] if routers else None,
                            "groups_file": str(Path(args.jev_groups).name), "price": price_line(jev),
                            "route_agreement": um.route_agreement(items)}
        variants[variant] = entry
    report = {"meta": meta(corpus, args, results), "variants": variants, "decision_rule": decision_rule(variants)}
    if live is not None:
        report["live"] = live
    return report


def meta(corpus: Dict[str, Any], args: argparse.Namespace, results: Sequence[VariantRun]) -> Dict[str, Any]:
    return {"generated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S"),
            "corpus": {"name": corpus["name"], "sha256": corpus["corpus_sha256"], "counts": corpus["counts"]},
            "variants": args.variants, "runs": args.runs, "budget": args.budget, "jev_budget": args.jev_budget,
            "providers": "fake:" + args.fake if args.fake else args.providers, "model": args.model or None,
            "fake": args.fake or None, "threshold": args.threshold,
            "note": f"N={args.runs}, только корпус марафона (часть 1), предварительно"}


def _pp(a: Optional[float], b: Optional[float]) -> Optional[float]:
    return None if a is None or b is None else (a - b) * 100.0


def decision_rule(variants: Dict[str, Any]) -> Dict[str, Any]:
    """Критерии «B принимаем» (карточка), заданные ДО прогона: что можно проверить на части 1, посчитано; остальное -
    «не измерено». Вердикта «принимаем» здесь нет: решает Шифу по таблице."""
    base = variants.get(uv.A)
    out: Dict[str, Any] = {}
    for name in (uv.B, uv.C):
        if base is None or name not in variants:
            continue
        mine, ref = variants[name]["metrics"], base["metrics"]
        drop = _pp(ref["pass"]["share"], mine["pass"]["share"])
        d_lat = (mine["latency"]["median_ms"] or 0) - (ref["latency"]["median_ms"] or 0)
        out[name] = {
            "pass_drop_pp": drop, "pass_drop_ok": None if drop is None else drop <= MAX_PASS_DROP_PP,
            "paraphrases_and_stt": "не измерено (этап 2)",
            "danger_A_B": [ref["danger"]["total"], mine["danger"]["total"]],
            "danger_ok": mine["danger"]["total"] <= ref["danger"]["total"],
            "median_latency_growth_ms": d_lat, "median_latency_limit": "X не задан Шифу"}
    return out


# ── Markdown ──────────────────────────────────────────────────────────────────────────────────────────────────────


def pct(value: Optional[float]) -> str:
    return "n/a" if value is None else f"{value * 100:.1f} %"


def num(value: Optional[float], digits: int = 0) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def _row(label: str, variants: Dict[str, Any], cell: Callable[[Dict[str, Any]], str]) -> str:
    return f"| {label} | " + " | ".join(cell(v) for v in variants.values()) + " |"


def metrics_table(variants: Dict[str, Any]) -> List[str]:
    heads = list(variants)
    lines = ["| метрика | " + " | ".join(heads) + " |", "|---|" + "---|" * len(heads)]
    m = lambda v: v["metrics"]  # noqa: E731
    lines += [
        _row("1. оцениваемых шагов (PASS / FAIL / не оценено / NOT_RUN)", variants, lambda v: "{} / {} / {} / {} / {}".format(
            m(v)["pass"]["steps"], m(v)["pass"]["PASS"], m(v)["pass"]["FAIL"], m(v)["pass"]["NOT_EVALUATED"],
            m(v)["pass"]["NOT_RUN"])),
        _row("1. доля PASS (из PASS+FAIL)", variants, lambda v: pct(m(v)["pass"]["share"])),
        _row("1. доля PASS с первой попытки", variants, lambda v: pct(m(v)["pass"]["first_attempt_share"])),
        _row("2. опасных ошибок всего", variants, lambda v: str(m(v)["danger"]["total"])),
    ]
    for kind, label in (("false_stop", "ложный стоп"), ("motion_instead_of_look", "движение вместо взгляда"),
                        ("delete_without_confirm", "удаление без подтверждения"),
                        ("claimed_without_tool", "«сделал» без исполненного тула")):
        lines.append(_row(f"2.  - {label}", variants, lambda v, k=kind: str(m(v)["danger"][k]["count"]) + (
            "" if m(v)["danger"][k]["measurable"] else " (в корпусе не измеряется)")))
    lines += [
        _row("3. размер среза (медиана / p90 / макс), тулов", variants, lambda v: "{} / {} / {}".format(
            num(m(v)["slice"]["size"]["median"]), num(m(v)["slice"]["size"]["p90"]), num(m(v)["slice"]["size"]["max"]))),
        _row("3. ожидаемый тул: вызван / нет в срезе / мост добавил / в срезе, но выбран другой", variants,
             lambda v: "{called} / {missing} / {rescued} / {wrong_choice}".format(**m(v)["slice"]["expected_tool_checks"])),
        _row("4. время хода, мс (медиана / p90)", variants,
             lambda v: f"{num(m(v)['latency']['median_ms'])} / {num(m(v)['latency']['p90_ms'])}"),
        _row("4. доля ходов без LLM", variants, lambda v: pct(m(v)["latency"]["without_llm_share"])),
        _row("5. вызовов LLM на ход", variants, lambda v: num(m(v)["llm"]["calls_per_turn"], 2)),
        _row("5. токенов на ход с LLM", variants, lambda v: num(m(v)["llm"]["tokens_per_llm_turn"])
             if m(v)["llm"]["tokens_reported"] else "n/a (провайдер не прислал usage)"),
        _row("6. устойчивость (тот же вердикт во всех прогонах)", variants,
             lambda v: pct(m(v)["stability"]["same_verdict_share"]) if m(v)["stability"]["same_verdict_share"] is not None
             else m(v)["stability"]["note"]),
        _row("вызовов LLM за прогон(ы), всего", variants, lambda v: str(v["llm_calls_total"])),
    ]
    return lines


def act_table(variants: Dict[str, Any]) -> List[str]:
    acts = list(dict.fromkeys(a for v in variants.values() for a in v["metrics"]["by_act"]))
    heads = list(variants)
    lines = ["| акт | " + " | ".join(f"{h}: PASS/FAIL" for h in heads) + " |", "|---|" + "---|" * len(heads)]
    for act in acts:
        cells = []
        for v in variants.values():
            row = v["metrics"]["by_act"].get(act)
            cells.append(f"{row['PASS']}/{row['FAIL']}" if row else "-")
        lines.append(f"| {act} | " + " | ".join(cells) + " |")
    return lines


def disagreements(variants: Dict[str, Any]) -> List[str]:
    """Шаги, где вердикты вариантов разошлись (по первому прогону): что разное - то и надо читать глазами."""
    base = list(variants)[0]
    lines: List[str] = []
    first = {name: {s["id"]: s for s in v["runs"][0] if s["scored"]} for name, v in variants.items()}
    for item_id, step in first[base].items():
        verdicts = {name: first[name].get(item_id, {}).get("verdict", "-") for name in variants}
        if len(set(verdicts.values())) > 1:
            why = "; ".join(f"{n}: {first[n][item_id]['fail'][0][:90]}" for n in variants
                            if item_id in first[n] and first[n][item_id]["fail"])
            lines.append(f"| {item_id} | " + " | ".join(verdicts.values()) + f" | {why} |")
    head = ["| шаг | " + " | ".join(variants) + " | чем не прошёл |", "|---|" + "---|" * len(variants) + "---|"]
    return head + lines if lines else ["(вердикты вариантов совпали на всех шагах)"]


def render(report: Dict[str, Any]) -> str:
    variants, meta_ = report["variants"], report["meta"]
    out = [f"# Замер понимания фразы, этап 1 ({meta_['generated_utc']} UTC)", "",
           f"Пометка: **{meta_['note']}**. Корпус `{meta_['corpus']['name']}` sha256 `{meta_['corpus']['sha256'][:12]}`: "
           f"{meta_['corpus']['counts']}. Провайдер: {meta_['providers']}, варианты {meta_['variants']}, "
           f"прогонов {meta_['runs']}, бюджет LLM {meta_['budget']}.", ""]
    if meta_["fake"]:
        out += [f"**Фейковый провайдер (`{meta_['fake']}`): проверка обвязки, цифры не характеризуют ни код, ни модель.**", ""]
    out += ["## Метрики", "", *metrics_table(variants), "", "## По актам", "", *act_table(variants), ""]
    out += ["## Где вердикты вариантов разошлись (прогон 1)", "", *disagreements(variants), ""]
    out += ["## Опасные ошибки по шагам", ""]
    for name, v in variants.items():
        for kind, found in v["metrics"]["danger"].items():
            if isinstance(found, dict) and found["count"]:
                out.append(f"- {name} / {kind}: {', '.join(found['steps'])}")
    out += ["", "## Правило решения Шифу (часть 1)", ""]
    for name, rule in report["decision_rule"].items():
        out.append(f"- {name} против A: потеря PASS {num(rule['pass_drop_pp'], 1)} п.п. (допуск {MAX_PASS_DROP_PP:g}: "
                   f"{rule['pass_drop_ok']}); перефразы и искажения STT: {rule['paraphrases_and_stt']}; опасных A/{name}: "
                   f"{rule['danger_A_B'][0]}/{rule['danger_A_B'][1]} ({rule['danger_ok']}); медиана хода "
                   f"{rule['median_latency_growth_ms']:+.0f} мс ({rule['median_latency_limit']}).")
    for name, v in variants.items():
        if "jev" in v:
            j = v["jev"]
            out += ["", f"## Jev (вариант {name}) - отдельная строка бюджета", "",
                    f"- вызовов Jev: {j['calls']}, сбоев: {j['errors']}, токены вход/выход: {j['input_tokens']}/"
                    f"{j['output_tokens']}, порог уверенности {j['threshold']}, файл описаний групп `{j['groups_file']}`;",
                    f"- цена: {j['price']}; совпадение выбора Jev с группой ожидаемого тула: {j['route_agreement']}."]
    if "live" in report:
        out += ["", *render_live(report["live"])]
    return "\n".join(out) + "\n"


def render_live(live: Dict[str, Any]) -> List[str]:
    cmp_ = live["compare"]
    lines = [f"## Сверка A с живым марафоном ({live['dir']})", "",
             f"- сравнено шагов: {cmp_['compared']}, расхождений: {cmp_['disagree']} ({pct(cmp_['share'])}; допуск "
             f"{MAX_LIVE_DISAGREE * 100:.0f} %): {'в допуске' if live['within_tolerance'] else 'ВЫШЕ ДОПУСКА - B как вывод не подавать'};",
             f"- живой FAIL, стенд PASS: {', '.join(cmp_['live_fail_dry_pass']) or '-'}; "
             f"живой PASS, стенд FAIL: {', '.join(cmp_['live_pass_dry_fail']) or '-'}; "
             f"не сравнено: {len(cmp_['skipped'])}.", ""]
    rows = [r for r in cmp_["rows"] if not r["agree"]]
    if rows:
        lines += ["| шаг | живой | стенд A | живая строка |", "|---|---|---|---|"]
        lines += [f"| {r['id']} | {r['live']} | {r['dry']} | {r['live_line'][:110]} |" for r in rows]
    return lines


def live_section(results: Sequence[VariantRun], corpus: Dict[str, Any], directory: Path) -> Optional[Dict[str, Any]]:
    """Вердикты A (первый прогон) против последних строк ``>>> STEP`` живого марафона по оцениваемым шагам."""
    base = next((r for r in results if r.variant == uv.A), None)
    if base is None:
        return None
    index = corpus_index(corpus)
    dry = {um.key(a, rep): rep.verdict for a, reps in base.reports.items() for rep in reps}
    ids = [i for i, e in index.items() if e["scored"] and i in dry]
    cmp_ = live_compare.compare(dry, live_compare.live_verdicts(directory), ids)
    return {"dir": str(directory), "compare": cmp_,
            "within_tolerance": cmp_["share"] is not None and cmp_["share"] <= MAX_LIVE_DISAGREE}


def write_out(out: Path, report: Dict[str, Any], results: Sequence[VariantRun]) -> None:
    """report.json, report.md и сырые логи шагов ``logs/<вариант>_<прогон>/act<акт>_<метка>.log`` (в репозиторий не кладутся)."""
    out.mkdir(parents=True, exist_ok=True)
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    (out / "report.md").write_text(render(report), encoding="utf-8")
    for res in results:
        folder = out / "logs" / f"{res.variant}_{res.run}"
        folder.mkdir(parents=True, exist_ok=True)
        for act, reports in res.reports.items():
            for rep in reports:
                (folder / f"act{act}_{rep.label}.log").write_text(rep.log + "\n", encoding="utf-8")


# ── CLI ───────────────────────────────────────────────────────────────────────────────────────────────────────────


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--variants", default="A,B", help="варианты через запятую: A, B, Bn, C")
    ap.add_argument("--runs", type=int, default=1, help="прогонов корпуса на вариант (N)")
    ap.add_argument("--budget", type=int, default=800, help="потолок вызовов LLM на все варианты прогона")
    ap.add_argument("--jev-budget", type=int, default=400, help="потолок вызовов Jev (вариант C), отдельно от LLM")
    ap.add_argument("--providers", default=os.environ.get("ROBBOX_LLM_PROVIDERS", "deepseek"))
    ap.add_argument("--model", default="", help="модель цепочки (по умолчанию - провайдера)")
    ap.add_argument("--fake", choices=("oracle", "blind"), default="", help="фейковый провайдер вместо настоящей LLM")
    ap.add_argument("--acts", default="", help="акты через запятую (по умолчанию все)")
    ap.add_argument("--only", default="", help="id или метки шагов через запятую")
    ap.add_argument("--max-retries", type=int, default=None, help="потолок повторов шага (по умолчанию как в сценарии)")
    ap.add_argument("--deadline-s", type=float, default=None, help="потолок хода, с")
    ap.add_argument("--threshold", type=float, default=jev_router.DEFAULT_THRESHOLD, help="порог уверенности Jev")
    ap.add_argument("--jev-groups", default=str(jev_router.DEFAULT_SPEC), help="файл описаний групп для Jev")
    ap.add_argument("--live-dir", default="", help="каталог логов живого марафона для сверки A")
    ap.add_argument("--out", default="", help="каталог отчёта (report.json, report.md, logs/)")
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    for path, applied in load_env_files(ENV_FILES):
        print(f"env: {path.name} ({applied} перем.)")
    os.environ.setdefault("ROBOT_LLM_VERBOSE", "0")
    variants = [uv.check(v.strip()) for v in args.variants.split(",") if v.strip()]
    corpus = load_corpus()
    acts = corpus_acts(corpus, [a.strip() for a in args.acts.split(",") if a.strip()],
                       [s.strip() for s in args.only.split(",") if s.strip()])
    if args.fake:
        make = fake_factory(corpus, args.fake == "blind")
    else:
        make = real_factory(args, jev_factory(args) if uv.C in variants else None)
    results = asyncio.run(run_all(variants, args.runs, acts, make, args.budget, args.max_retries))
    live = live_section(results, corpus, Path(args.live_dir)) if args.live_dir else None
    report = build_report(results, corpus, args, live)
    print(render(report))
    if args.out:
        write_out(Path(args.out), report, results)
    return 0


if __name__ == "__main__":
    sys.exit(main())
