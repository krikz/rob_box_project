#!/usr/bin/env python3
"""Сухой прогон актов ночного марафона без робота (эпик #3683, карточка #3906).

    python scripts/dialog/dry_marathon.py --acts 1,2,3,4 --out ./out          # настоящая LLM (DEEPSEEK_API_KEY в env)
    python scripts/dialog/dry_marathon.py --acts 1 --list                     # только разметка шагов, 0 запросов LLM

Текст шагов актов (``scripts/e2e/gen_night_marathon.py`` → ``.github/e2e/scenarios/night``) идёт через ход v2 без
ROS (:mod:`dry_stand`): настоящая адресация, сессия, бэклог, цепочка LLM с мостом команд, срез тулов, исполнитель;
владельцы речи, музыки, лица, голоса и диктора — фейки, отвечающие событием сразу. Сверка ожиданий шага — те же
функции, что у живого харнесса: ``e2e_tool_match.step_verdict`` (acceptance) и ``patterns_missing`` (patterns), по
логу в форматах строк робота. Второй реализации правил здесь нет.

Каждый шаг помечен (:func:`classify_step`):

* ``text`` — проверяется сухо: решает код и LLM, акустика ничего не добавляет;
* ``acoustic`` — на нём держится STT, вейк-слово по звуку, биометрия, TTS-голос или тайминг перебивания: сухой вердикт
  только справочный, засчитывает его живой робот.

Бюджет запросов LLM (``--budget``, по умолчанию 300) считает счётчик настоящих вызовов провайдера (повторы моста — тоже
вызовы); упёрся — прогон останавливается, остальные шаги ``NOT_RUN``. Ключ API не печатается и нигде не сохраняется.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import re
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

from dry_stand import REPO, DryStand, new_workdir  # noqa: E402

sys.path.insert(0, str(REPO / ".github" / "workflows" / "scripts"))
import e2e_tool_match as etm  # noqa: E402

SCENARIOS = REPO / ".github" / "e2e" / "scenarios" / "night"
MANIFEST = SCENARIOS / "night_marathon_manifest.json"
PROMPT_DIR = REPO / "src" / "rob_box_voice" / "prompts"
#: Один шаг тратит не больше стольких вызовов LLM (итерации ``query`` + повтор моста): запас до стопа по бюджету.
STEP_CALLS_RESERVE = 8

TEXT, ACOUSTIC = "text", "acoustic"
PASS, FAIL, NOT_RUN = "PASS", "FAIL", "NOT_RUN"

#: Паттерны шага, которые ловят строки акустических владельцев (STT, биометрия, синтез) — их в сухом логе нет.
_ACOUSTIC_PATTERN = re.compile(r"Speaker|speaker_id|identify candidates|ПРИНЯТО|SpeechEvent synthesized"
                               r"|SpeechEvent voice_changed|TTS current_voice|Воспроизведение")


@dataclass
class Act:
    """Акт марафона: шаги сценария и флаги изоляции, как их читает ``isolate_act_start``."""

    key: str
    title: str
    path: Path
    steps: List[Dict[str, Any]]
    inherits: bool
    registers_speakers: bool
    writes_memory: bool


@dataclass
class StepReport:
    act: str
    label: str
    text: str
    kind: str
    reasons: List[str]
    verdict: str = NOT_RUN
    fail: List[str] = field(default_factory=list)
    attempts: int = 0
    addressed: Optional[bool] = None
    tools: List[str] = field(default_factory=list)
    said: List[str] = field(default_factory=list)
    llm_calls: int = 0
    fakes: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)
    log: str = ""


def load_act(key: str) -> Act:
    """Акт из манифеста марафона по ключу (``1``, ``2b``…)."""
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    entry = next((a for a in manifest["acts"] if str(a["act"]) == str(key)), None)
    if entry is None:
        raise SystemExit(f"акта {key!r} нет в манифесте {MANIFEST}")
    path = REPO / entry["scenario_file"]
    raw = path.read_text(encoding="utf-8")
    data = json.loads(raw)
    return Act(str(entry["act"]), str(entry["title"]), path, list(data["steps"]),
               data.get("inherits_previous_act") is True, "register_speaker" in raw, "memory_save" in raw)


def classify_step(step: Dict[str, Any]) -> Tuple[str, List[str]]:
    """``(text | acoustic, причины)`` шага по его ожиданиям: что в них держится на звуке, а не на тексте и тулах."""
    acc = step.get("acceptance") or {}
    reasons: List[str] = []
    if acc.get("must_not_identify_as"):
        reasons.append("биометрия: опознание диктора")
    if etm.registration_expected(acc):
        reasons.append("биометрия: регистрация голоса принята speaker_id_node")
    if acc.get("voice_changed"):
        reasons.append("TTS-владелец: голос реально сменился")
    if step.get("barge_in"):
        reasons.append("перебивание речи по таймингу")
    for pattern in step.get("patterns") or []:
        if _ACOUSTIC_PATTERN.search(pattern):
            reasons.append(f"паттерн по строке акустического владельца: {pattern[:50]}")
    return (ACOUSTIC if reasons else TEXT), reasons


def expectations(step: Dict[str, Any]) -> str:
    """``backlog`` — реплика без вейка копится; иначе адресованный ход (``cycle`` / ``wake-gated`` / авто)."""
    return "backlog" if step.get("expect") == "backlog" else "turn"


def check_step(step: Dict[str, Any], heard: Any) -> List[str]:
    """Причины провала шага по одной попытке: адресация, ход, паттерны, acceptance. Пусто — шаг прошёл."""
    fails: List[str] = []
    want = expectations(step)
    if want == "backlog" and heard.addressed:
        fails.append("ожидался бэклог (expect=backlog), а реплика адресована роботу")
    elif want == "turn" and not heard.addressed:
        fails.append(f"реплика не адресована роботу ({heard.reason}): вейк-слова нет в тексте")
    elif want == "turn":
        agg = heard.aggregate
        if agg.status != "finished":
            fails.append(f"ход не дошёл до finished: stage={agg.stage} status={agg.status}")
        elif not heard.said and not step.get("silent_ok"):
            fails.append("полного цикла нет: робот ничего не сказал (silent_ok не задан)")
    missing = etm.patterns_missing(heard.log, step.get("patterns") or [])
    if missing:
        fails.append(f"паттерны шага не найдены в логе: {missing}")
    acc = step.get("acceptance")
    if acc:
        verdict = etm.step_verdict(acc, heard.log, label=step.get("label", ""))
        if not verdict["pass"]:
            fails.append(f"acceptance: {verdict['reason']}")
    return fails


async def run_step(stand: DryStand, act: str, step: Dict[str, Any], prev_speech: str, budget: int,
                   max_retries: Optional[int]) -> Tuple[StepReport, str]:
    """Шаг сценария на стенде с повтором по ``retry_acceptance`` (как харнесс). Возвращает отчёт и речь шага."""
    kind, reasons = classify_step(step)
    report = StepReport(act, step["label"], step["text"], kind, reasons)
    asked = step.get("when_robot_asked")
    if asked and not re.search(asked, prev_speech, re.IGNORECASE):
        report.verdict, report.fail = PASS, [f"SKIP: робот не задал вопрос {asked!r}"]
        return report, prev_speech
    if step.get("sleep_before_sec"):
        stand.clock.advance(float(step["sleep_before_sec"]))
    retries = int(step.get("retry_acceptance") or 0)
    retries = retries if max_retries is None else min(retries, max_retries)
    for attempt in range(1, retries + 2):
        if stand.counter.calls + STEP_CALLS_RESERVE > budget:
            report.fail.append(f"бюджет LLM исчерпан ({stand.counter.calls}/{budget})")
            report.verdict = NOT_RUN if attempt == 1 else report.verdict
            break
        heard = await stand.hear(step["text"], step.get("voice", ""))
        report.attempts, report.addressed, report.log = attempt, heard.addressed, heard.log
        report.tools, report.said, report.fakes = heard.tools, heard.said, sorted(heard.touched)
        report.llm_calls += heard.llm_calls
        report.notes = [f"{t}: {st}" + (f"({why})" if why else "") for t, st, why in heard.outcomes if st != "done"]
        fails = check_step(step, heard)
        report.fail, report.verdict = fails, (FAIL if fails else PASS)
        if not fails or etm.retry_block_reason(heard.log):
            break
    return report, "\n".join(report.said)


async def run_act(stand: DryStand, act: Act, budget: int, max_retries: Optional[int],
                  only: Optional[Sequence[str]] = None) -> List[StepReport]:
    """Акт целиком: изоляция в начале (как ``isolate_act_start``), затем шаги по порядку."""
    if not act.inherits:
        stand.reset_session()
    if act.registers_speakers:
        stand.reset_speakers()
    if act.registers_speakers or act.writes_memory:
        stand.reset_memory()
    reports: List[StepReport] = []
    speech = ""
    for step in act.steps:
        if only and step["label"] not in only:
            continue
        report, speech = await run_step(stand, act.key, step, speech, budget, max_retries)
        reports.append(report)
        if report.verdict == NOT_RUN:
            break
    return reports


# ── Отчёт ─────────────────────────────────────────────────────────────────────────────────────────────────────────


def tally(reports: Sequence[StepReport]) -> Dict[str, int]:
    text = [r for r in reports if r.kind == TEXT]
    acoustic = [r for r in reports if r.kind == ACOUSTIC]
    return {"total": len(reports), "text": len(text), "acoustic": len(acoustic),
            "text_pass": sum(r.verdict == PASS for r in text), "text_fail": sum(r.verdict == FAIL for r in text),
            "acoustic_info_pass": sum(r.verdict == PASS for r in acoustic),
            "acoustic_info_fail": sum(r.verdict == FAIL for r in acoustic),
            "not_run": sum(r.verdict == NOT_RUN for r in reports)}


def render(acts: Dict[str, Sequence[StepReport]], llm_calls: int, budget: int) -> str:
    lines = ["акт | шагов | текстовых | PASS | FAIL | акустических | (справочно PASS/FAIL) | NOT_RUN",
             "----|-------|-----------|------|------|--------------|-----------------------|--------"]
    for key, reports in acts.items():
        t = tally(reports)
        lines.append(f"{key} | {t['total']} | {t['text']} | {t['text_pass']} | {t['text_fail']} | {t['acoustic']} | "
                     f"{t['acoustic_info_pass']}/{t['acoustic_info_fail']} | {t['not_run']}")
    lines += ["", "шаг | вид | вердикт | попыток | LLM | инструменты | чем держится / причина"]
    for key, reports in acts.items():
        for r in reports:
            why = "; ".join(r.fail) if r.fail else ("фейки: " + ",".join(r.fakes) if r.fakes else "")
            why += f" [исходы не done: {', '.join(r.notes)}]" if r.notes else ""
            if r.kind == ACOUSTIC:
                why = f"[акустика: {'; '.join(r.reasons)}] {why}"
            lines.append(f"{key}/{r.label} | {r.kind} | {r.verdict} | {r.attempts} | {r.llm_calls} | "
                         f"{','.join(r.tools) or '-'} | {why}")
    lines += ["", f"запросов LLM: {llm_calls} из бюджета {budget}"]
    return "\n".join(lines)


def write_out(out: Path, acts: Dict[str, Sequence[StepReport]], llm_calls: int, budget: int) -> None:
    """Сырые логи шагов и сводка — в каталог прогона (в репозиторий не кладутся)."""
    out.mkdir(parents=True, exist_ok=True)
    for key, reports in acts.items():
        for r in reports:
            (out / f"act{key}_{r.label}.log").write_text(r.log + "\n", encoding="utf-8")
    summary = {key: [asdict(r) for r in reports] for key, reports in acts.items()}
    (out / "report.json").write_text(json.dumps({"llm_calls": llm_calls, "budget": budget, "acts": summary},
                                                ensure_ascii=False, indent=1), encoding="utf-8")
    (out / "summary.txt").write_text(render(acts, llm_calls, budget) + "\n", encoding="utf-8")


# ── Настоящая LLM ─────────────────────────────────────────────────────────────────────────────────────────────────


class LogRelay:
    """Мост команд логирует до того, как стенд собран: строки идут в лог текущего шага, когда стенд уже есть."""

    def __init__(self) -> None:
        self.target: Optional[DryStand] = None

    def __call__(self, line: str) -> None:
        if self.target is not None:
            self.target.emit(line)


def build_real_stand(providers: Sequence[str], workdir: Path, deadline_s: Optional[float]) -> DryStand:
    """Стенд с настоящей цепочкой LLM (ключи из окружения; нет ключа — ``RuntimeError`` от сборки цепочки)."""
    from rob_box_voice.core.dialog_v2_ports import CallCounter, assemble_llm  # noqa: PLC0415

    relay, counter = LogRelay(), CallCounter()
    llm, system = assemble_llm(providers, counter, prompt_dir=PROMPT_DIR,
                               system_prompt_file="master_prompt_compact.txt", log=relay)
    stand = DryStand(llm, counter, system, workdir)
    relay.target = stand
    if deadline_s:
        stand.engine._deadline_s = deadline_s  # noqa: SLF001 — бюджет хода сети песочницы, не знание робота
    return stand


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--acts", default="1,2,3,4", help="акты через запятую (ключи манифеста: 1,2,2b,3…)")
    ap.add_argument("--only", default="", help="метки шагов через запятую (по умолчанию все)")
    ap.add_argument("--providers", default="deepseek", help="цепочка LLM (как llm_providers dialog_node)")
    ap.add_argument("--budget", type=int, default=300, help="потолок запросов LLM за прогон")
    ap.add_argument("--max-retries", type=int, default=None, help="потолок повторов шага (по умолчанию как в сценарии)")
    ap.add_argument("--deadline-s", type=float, default=None, help="бюджет хода, с (по умолчанию TURN_DEADLINE_S)")
    ap.add_argument("--out", default="", help="каталог сырых логов шагов и сводки")
    ap.add_argument("--list", action="store_true", help="только разметка шагов text/acoustic, без LLM")
    args = ap.parse_args(argv)
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    acts = [load_act(k.strip()) for k in args.acts.split(",") if k.strip()]
    if args.list:
        for act in acts:
            print(f"== акт {act.key} «{act.title}»: {len(act.steps)} шагов")
            for step in act.steps:
                kind, reasons = classify_step(step)
                print(f"  {step['label']:36} {kind:9} {'; '.join(reasons)}")
        return 0
    workdir = new_workdir()
    stand = build_real_stand([p for p in args.providers.split(",") if p.strip()], workdir, args.deadline_s)
    only = [s.strip() for s in args.only.split(",") if s.strip()] or None
    results: Dict[str, Sequence[StepReport]] = {}

    async def go() -> None:
        for act in acts:
            results[act.key] = await run_act(stand, act, args.budget, args.max_retries, only)
            print(f"-- акт {act.key}: запросов LLM {stand.counter.calls}/{args.budget}", flush=True)

    try:
        asyncio.run(go())
    finally:
        stand.close()
    print(render(results, stand.counter.calls, args.budget))
    if args.out:
        write_out(Path(args.out), results, stand.counter.calls, args.budget)
    return 1 if any(t["text_fail"] for t in map(tally, results.values())) else 0


if __name__ == "__main__":
    sys.exit(main())
