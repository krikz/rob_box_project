#!/usr/bin/env python3
"""Замер «кто понимает фразу»: код по словам или модель (карточка #4163, этапы 1-2).

    python scripts/dialog/understanding_measure.py --variants A,C --parts 1,2,3,4 --runs 3 --budget 4000 --out ./out
    python scripts/dialog/understanding_measure.py --variants A,B,C --fake oracle --out ./out              # без ключа
    python scripts/dialog/understanding_measure.py --variants A --live-dir tests/e2e/_artifacts/night_20261028_092532

Корпус - ``understanding_corpus/``: часть 1 - фразы марафона (``marathon.json``), части 2-4 - другие тексты тех же шагов
(перефразы, реальные искажения STT, английский; :mod:`alternatives`). Все части заморожены коммитом, хеши сверяются
при загрузке. Каждый вариант (``understanding_variants``: A - как сейчас, B - понимает модель, C - понимает Jev) проходит
каждую часть N раз на свежем стенде, акты по порядку, как живой марафон. В частях 2-4 у оцениваемого шага с
альтернативой текст заменён (прогон ``r`` - альтернатива ``((r - 1) mod K) + 1``), ожидание шага то же; остальные шаги
идут исходным текстом как контекст и в метрику части не входят.

Вердикт шага - только ``dry_marathon.check_step`` (``e2e_tool_match.step_verdict`` / ``patterns_missing``): второй
реализации правил здесь нет. Упал стенд или судья - «не оценено», не FAIL. Метрики 1-6 карточки -
``understanding_metrics``, отчёт и правило решения - ``understanding_report``.

Бюджет. ``--budget`` - потолок вызовов LLM на ВЕСЬ прогон. Порядок - «прогон 1 всех частей, потом прогон 2…», внутри -
варианты подряд на одних текстах (слот). Слот запускается, только если остатка хватит всем его вариантам по самому
дорогому уже сделанному проходу; иначе весь слот ``NOT_RUN`` (это видно в отчёте). Внутри слота остаток делится
поровну между вариантами: варианты одного слота в равных условиях. Вызовы Jev (вариант C) - отдельный общий потолок
``--jev-budget`` на весь прогон. ``--fake oracle|blind`` - фейковый провайдер вместо DeepSeek: проверка обвязки, а не
качества. Ключи берутся из ``scripts/dialogue/.env`` и окружения; значения не печатаются и в отчёт не попадают.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import time
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE / "understanding_corpus"))

import alternatives as alt  # noqa: E402
import build_marathon  # noqa: E402
import dry_marathon as dm  # noqa: E402
import jev_router  # noqa: E402
import live_compare  # noqa: E402
import understanding_metrics as um  # noqa: E402
import understanding_report as ur  # noqa: E402
import understanding_variants as uv  # noqa: E402
from dry_stand import REPO, DryStand, new_workdir  # noqa: E402
from env_files import load_env_files  # noqa: E402
from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog.words import strip_wake_word  # noqa: E402
from understanding_report import (MAX_LATENCY_GROWTH_S, MAX_LIVE_DISAGREE, MAX_PASS_DROP_PP,  # noqa: E402,F401
                                  decision_rule, render)

CORPUS = _HERE / "understanding_corpus" / "marathon.json"
ENV_FILES = (REPO / "scripts" / "dialogue" / ".env", REPO / ".env")
DEEPSEEK_BALANCE_URL = "https://api.deepseek.com/user/balance"

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


def load_parts(base: Dict[str, Any], parts: Sequence[int]) -> Dict[int, Dict[str, Any]]:
    """Части 2-4 из ``understanding_corpus`` (часть 1 - сам ``base``); неизвестная часть - громкий отказ."""
    unknown = [p for p in parts if p != 1 and p not in alt.PARTS]
    if unknown:
        raise SystemExit(f"--parts: неизвестные части {unknown}; есть 1, {', '.join(map(str, alt.PARTS))}")
    return {p: alt.load_part(p, base) for p in parts if p != 1}


def part_view(base: Dict[str, Any], loaded: Dict[int, Dict[str, Any]], part: int, run: int) -> Dict[str, Any]:
    """Корпус прогона ``run`` части: часть 1 как есть, части 2-4 - с подставленными альтернативами."""
    return base if part == 1 else alt.part_corpus(base, loaded[part]["alternatives"], run)


def corpus_meta(base: Dict[str, Any], loaded: Dict[int, Dict[str, Any]], parts: Sequence[int]) -> List[Dict[str, Any]]:
    out = []
    for p in parts:
        data = base if p == 1 else loaded[p]
        out.append({"part": p, "title": alt.TITLES[p], "name": data["name"], "sha256": data["corpus_sha256"],
                    "counts": data["counts"]})
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
    """Знание фейка: реплика → шаг корпуса (по тексту после вейка, в том числе по текстам альтернатив частей 2-4);
    отвечает по ожиданиям шага, а не «понимает»."""

    def __init__(self, corpus: Dict[str, Any], wake: Sequence[str],
                 extra: Sequence[Tuple[str, Dict[str, Any]]] = ()) -> None:
        pairs = [(s["step"]["text"], s) for a in corpus["acts"] for s in a["steps"]] + list(extra)
        self._by_text = {strip_wake_word(text, list(wake)): entry for text, entry in pairs}

    def step(self, text: str) -> Optional[Dict[str, Any]]:
        hits = [k for k in self._by_text if k and k in text]
        return self._by_text[max(hits, key=len)] if hits else None

    def keywords(self, entry: Optional[Dict[str, Any]]) -> str:
        acc = (entry["step"].get("acceptance") or {}) if entry else {}
        return " ".join(acc.get("expected_keywords") or []) or "Хорошо."


def alternative_texts(base: Dict[str, Any], loaded: Dict[int, Dict[str, Any]]) -> List[Tuple[str, Dict[str, Any]]]:
    """Тексты альтернатив всех загруженных частей с записью шага части 1 - для фейка-oracle."""
    index = corpus_index(base)
    return [(row["text"], index[sid]) for data in loaded.values() for sid, rows in data["alternatives"].items()
            for row in rows]


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
class Slot:
    """Один проход марафона: прогон ``run`` части ``part`` - его делают все варианты на одних текстах."""

    run: int
    part: int
    acts: List[dm.Act]


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
    part: int = 1
    skipped: bool = False


StandFactory = Callable[[str], Tuple[DryStand, Any]]


def real_factory(args: argparse.Namespace, jev: Optional[Callable[[], Any]]) -> StandFactory:
    providers = [p for p in args.providers.split(",") if p.strip()]

    def make(variant: str) -> Tuple[DryStand, Any]:
        router = jev() if variant == uv.C and jev is not None else None
        return dm.build_real_stand(providers, new_workdir("understanding_"), args.deadline_s, variant=variant,
                                   router=router, model=args.model), router

    return make


def fake_factory(corpus: Dict[str, Any], blind: bool,
                 extra: Sequence[Tuple[str, Dict[str, Any]]] = ()) -> StandFactory:
    def make(variant: str) -> Tuple[DryStand, Any]:
        from dry_stand import WAKE_WORDS_YAML  # noqa: PLC0415
        from rob_box_dialog.address import wake_words  # noqa: PLC0415

        oracle = Oracle(corpus, wake_words(str(WAKE_WORDS_YAML))[0], extra)
        router = OracleRouter(oracle) if variant == uv.C else None
        return fake_stand(oracle_rule(oracle, blind), new_workdir("understanding_fake_"), variant, router), router

    return make


def jev_factory(args: argparse.Namespace) -> Callable[[], Any]:
    """Классификатор Jev (ключ ``JEV_API_KEY`` из env). Нет ключа - громкий отказ, а не тихий срез B. Потолок вызовов
    ``--jev-budget`` - общий на весь прогон: каждый новый роутер получает остаток после прошлых."""
    from rob_box_harness.decision import jev_from_env  # noqa: PLC0415

    provider = jev_from_env(timeout_s=jev_router.DEFAULT_TIMEOUT_S)
    if provider is None:
        raise SystemExit("вариант C: нет JEV_API_KEY (scripts/dialogue/.env или окружение)")
    spec = jev_router.load_spec(Path(args.jev_groups))
    made: List[jev_router.JevRouter] = []

    def make() -> jev_router.JevRouter:
        left = max(0, args.jev_budget - sum(r.calls for r in made))
        made.append(jev_router.JevRouter(provider, spec, args.threshold, left))
        return made[-1]

    return make


async def run_variant(variant: str, slot: Slot, make: StandFactory, cap: int,
                      max_retries: Optional[int]) -> VariantRun:
    """Один вариант, один проход марафона на свежем стенде; акты по порядку, состояние переходит между актами."""
    stand, router = make(variant)
    started = time.monotonic()
    run: Run = {}
    try:
        for act, phase in zip(slot.acts, dm.marathon_phases(slot.acts)):  # режим e2e - как у раннера марафона (#3972)
            run[act.key] = await dm.run_act(stand, act, cap, max_retries, phase=phase)
            print(f"-- {variant} часть {slot.part} прогон {slot.run}, акт {act.key}: запросов LLM "
                  f"{stand.counter.calls}/{cap}", flush=True)
    finally:
        calls = stand.counter.calls
        stand.close()
    return VariantRun(variant, slot.run, run, calls, time.monotonic() - started,
                      router.stats() if router is not None else None, cap, part=slot.part)


def skipped_run(variant: str, slot: Slot, why: str) -> VariantRun:
    """Проход, на который не хватило бюджета: все шаги ``NOT_RUN`` с причиной - видно в отчёте, а не пропадает."""
    reports: Run = {}
    for act in slot.acts:
        reports[act.key] = []
        for step in act.steps:
            kind, reasons = dm.classify_step(step)
            reports[act.key].append(dm.StepReport(act.key, step["label"], step["text"], kind, reasons,
                                                  verdict=dm.NOT_RUN, fail=[why]))
    return VariantRun(variant, slot.run, reports, 0, 0.0, None, 0, part=slot.part, skipped=True)


async def run_plan(variants: Sequence[str], slots: Sequence[Slot], make: StandFactory, budget: int,
                   max_retries: Optional[int]) -> List[VariantRun]:
    """Слоты по порядку. Слот идёт, если остатка хватит всем вариантам по самому дорогому сделанному проходу (+ запас
    шага); иначе весь слот ``NOT_RUN``. Внутри слота у каждого варианта один и тот же потолок - остаток на начало слота
    поровну; недобранное вариантом переходит в следующие слоты, а не соседу по слоту."""
    spent, costliest, done = 0, 0, []
    for slot in slots:
        remaining = budget - spent
        need = len(variants) * (costliest + dm.STEP_CALLS_RESERVE) if costliest else 1
        if remaining < need:
            why = f"бюджет LLM: проход не запущен (остаток {remaining} < {need} на {len(variants)} вариант(а))"
            done += [skipped_run(v, slot, why) for v in variants]
            continue
        cap = remaining // len(variants)
        for variant in variants:
            result = await run_variant(variant, slot, make, cap, max_retries)
            spent += result.llm_calls
            costliest = max(costliest, result.llm_calls)
            done.append(result)
    return done


def plan_slots(base: Dict[str, Any], loaded: Dict[int, Dict[str, Any]], parts: Sequence[int], runs: int,
               acts: Sequence[str] = (), only: Sequence[str] = ()) -> List[Slot]:
    """Порядок проходов: прогон 1 всех частей, потом прогон 2… - кончился бюджет, теряются последние прогоны, а не части."""
    return [Slot(r, p, corpus_acts(part_view(base, loaded, p, r), acts, only)) for r in range(1, runs + 1)
            for p in parts]


async def run_all(variants: Sequence[str], runs: int, acts: Sequence[dm.Act], make: StandFactory, budget: int,
                  max_retries: Optional[int]) -> List[VariantRun]:
    """Часть 1 (этап 1): N прогонов одних и тех же актов."""
    return await run_plan(variants, [Slot(r, 1, list(acts)) for r in range(1, runs + 1)], make, budget, max_retries)


# ── Отчёт ─────────────────────────────────────────────────────────────────────────────────────────────────────────


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


def deepseek_balance() -> Optional[Dict[str, Any]]:
    """Баланс аккаунта DeepSeek (``/user/balance``): ключ из env, наружу - только валюта и сумма. Сбой - ``None``."""
    key = os.environ.get("DEEPSEEK_API_KEY", "")
    if not key:
        return None
    request = urllib.request.Request(DEEPSEEK_BALANCE_URL, headers={"Authorization": f"Bearer {key}"})
    try:
        with urllib.request.urlopen(request, timeout=10) as resp:  # noqa: S310 — фиксированный https-адрес
            info = json.loads(resp.read().decode("utf-8"))["balance_infos"][0]
    except Exception:  # noqa: BLE001 — баланс справочный: нет ответа - так и пишем, прогон не роняем
        return None
    return {"currency": str(info.get("currency")), "total": float(info.get("total_balance"))}


def deepseek_spend(before: Optional[Dict[str, Any]], after: Optional[Dict[str, Any]], calls: int) -> Dict[str, Any]:
    """Цена прогона по балансу аккаунта. Баланс не получен или не сдвинулся при вызовах - цена «не измерена», а не 0."""
    if before is None or after is None or before["currency"] != after["currency"]:
        return {"before": before, "after": after, "spent": None, "note": "баланс не получен - цена не измерена"}
    out = {"before": f"{before['total']:.4f} {before['currency']}", "after": f"{after['total']:.4f} {after['currency']}"}
    if calls and before["total"] == after["total"]:
        return {**out, "spent": None, "note": f"баланс не сдвинулся за {calls} вызовов (обновляется с задержкой или "
                                              f"аккаунт в минусе) - по балансу цена не измерена"}
    return {**out, "spent": f"{before['total'] - after['total']:.4f} {before['currency']}",
            "note": "разница баланса аккаунта до и после прогона; ключ общий - в неё входят и чужие вызовы этого ключа"}


def meta(corpus: List[Dict[str, Any]], args: argparse.Namespace, parts: Sequence[int],
         results: Sequence[VariantRun]) -> Dict[str, Any]:
    tokens = sum(t["tokens"] for r in results for reps in r.reports.values() for rep in reps for t in rep.turns)
    return {"generated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S"), "corpus": corpus,
            "parts": list(parts), "variants": args.variants, "runs": args.runs, "budget": args.budget,
            "jev_budget": args.jev_budget, "providers": "fake:" + args.fake if args.fake else args.providers,
            "model": args.model or None, "fake": args.fake or None, "threshold": args.threshold,
            "max_latency_growth_s": args.max_latency_growth_s,
            "llm_calls_total": sum(r.llm_calls for r in results), "prompt_tokens_total": tokens,
            "note": f"N={args.runs}, части {','.join(map(str, parts))}" + (
                ", только корпус марафона (часть 1), предварительно" if list(parts) == [1] else ""),
            "rotation": "Части 2-4: прогон r берёт альтернативу ((r-1) mod K)+1 шага (K - сколько их у шага). Поэтому "
                        "устойчивость части 2 при K = N - тот же вердикт на РАЗНЫХ перефразах (шум модели и перефраз "
                        "вместе), а не повтор одного текста; у частей 1 и 4 (K = 1) - повтор одного текста."}


def build_report(results: Sequence[VariantRun], corpus: Dict[str, Any], args: argparse.Namespace,
                 live: Optional[Dict[str, Any]], loaded: Optional[Dict[int, Dict[str, Any]]] = None,
                 deepseek: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    loaded = loaded or {}
    parts = list(dict.fromkeys(r.part for r in results))
    indexes = {p: corpus_index(part_view(corpus, loaded, p, 1)) for p in parts}
    head = meta(corpus_meta(corpus, loaded, parts), args, parts, results)
    if deepseek is not None:
        head["deepseek"] = deepseek
    return ur.build_report(results, indexes, head, live, Path(args.jev_groups).name, price_line)


def live_section(results: Sequence[VariantRun], corpus: Dict[str, Any], directory: Path) -> Optional[Dict[str, Any]]:
    """Вердикты A (первый прогон части 1) против последних строк ``>>> STEP`` живого марафона по оцениваемым шагам."""
    base = next((r for r in results if r.variant == uv.A and r.part == 1 and not r.skipped), None)
    if base is None:
        return None
    index = corpus_index(corpus)
    dry = {um.key(a, rep): rep.verdict for a, reps in base.reports.items() for rep in reps}
    ids = [i for i, e in index.items() if e["scored"] and i in dry]
    cmp_ = live_compare.compare(dry, live_compare.live_verdicts(directory), ids)
    return {"dir": str(directory), "compare": cmp_,
            "within_tolerance": cmp_["share"] is not None and cmp_["share"] <= MAX_LIVE_DISAGREE}


def write_out(out: Path, report: Dict[str, Any], results: Sequence[VariantRun]) -> None:
    """report.json, report.md и сырые логи шагов ``logs/<вариант>[_p<часть>]_<прогон>/act<акт>_<метка>.log``."""
    out.mkdir(parents=True, exist_ok=True)
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str), encoding="utf-8")
    (out / "report.md").write_text(render(report), encoding="utf-8")
    for res in results:
        folder = out / "logs" / (f"{res.variant}_{res.run}" if res.part == 1 else f"{res.variant}_p{res.part}_{res.run}")
        folder.mkdir(parents=True, exist_ok=True)
        for act, reports in res.reports.items():
            for rep in reports:
                (folder / f"act{act}_{rep.label}.log").write_text(rep.log + "\n", encoding="utf-8")


# ── CLI ───────────────────────────────────────────────────────────────────────────────────────────────────────────


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--variants", default="A,B", help="варианты через запятую: A, B, Bn, C")
    ap.add_argument("--parts", default="1", help="части корпуса через запятую: 1 - марафон, 2 - перефразы, "
                                                 "3 - искажения STT, 4 - английский")
    ap.add_argument("--runs", type=int, default=1, help="прогонов каждой части на вариант (N)")
    ap.add_argument("--budget", type=int, default=800, help="потолок вызовов LLM на весь прогон")
    ap.add_argument("--jev-budget", type=int, default=400, help="потолок вызовов Jev (вариант C) на весь прогон")
    ap.add_argument("--providers", default=os.environ.get("ROBBOX_LLM_PROVIDERS", "deepseek"))
    ap.add_argument("--model", default="", help="модель цепочки (по умолчанию - провайдера)")
    ap.add_argument("--fake", choices=("oracle", "blind"), default="", help="фейковый провайдер вместо настоящей LLM")
    ap.add_argument("--acts", default="", help="акты через запятую (по умолчанию все)")
    ap.add_argument("--only", default="", help="id или метки шагов через запятую")
    ap.add_argument("--max-retries", type=int, default=None, help="потолок повторов шага (по умолчанию как в сценарии)")
    ap.add_argument("--deadline-s", type=float, default=None, help="потолок хода, с")
    ap.add_argument("--max-latency-growth-s", type=float, default=MAX_LATENCY_GROWTH_S,
                    help="правило решения: на сколько секунд медиана хода варианта может вырасти над A (решение Шифу "
                         "28.10: «до 3-х секунд»)")
    ap.add_argument("--threshold", type=float, default=jev_router.DEFAULT_THRESHOLD, help="порог уверенности Jev")
    ap.add_argument("--jev-groups", default=str(jev_router.DEFAULT_SPEC), help="файл описаний групп для Jev")
    ap.add_argument("--live-dir", default="", help="каталог логов живого марафона для сверки A")
    ap.add_argument("--out", default="", help="каталог отчёта (report.json, report.md, logs/)")
    return ap


def split(value: str) -> List[str]:
    return [v.strip() for v in value.split(",") if v.strip()]


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    for path, applied in load_env_files(ENV_FILES):
        print(f"env: {path.name} ({applied} перем.)")
    os.environ.setdefault("ROBOT_LLM_VERBOSE", "0")
    variants = [uv.check(v) for v in split(args.variants)]
    parts = [int(p) for p in split(args.parts)]
    corpus = load_corpus()
    loaded = load_parts(corpus, parts)
    slots = plan_slots(corpus, loaded, parts, args.runs, split(args.acts), split(args.only))
    if args.fake:
        make = fake_factory(corpus, args.fake == "blind", alternative_texts(corpus, loaded))
    else:
        make = real_factory(args, jev_factory(args) if uv.C in variants else None)
    before = None if args.fake else deepseek_balance()
    results = asyncio.run(run_plan(variants, slots, make, args.budget, args.max_retries))
    spend = None if args.fake else deepseek_spend(before, deepseek_balance(), sum(r.llm_calls for r in results))
    live = live_section(results, corpus, Path(args.live_dir)) if args.live_dir else None
    report = build_report(results, corpus, args, live, loaded, spend)
    print(render(report))
    if args.out:
        write_out(Path(args.out), report, results)
    return 0


if __name__ == "__main__":
    sys.exit(main())
