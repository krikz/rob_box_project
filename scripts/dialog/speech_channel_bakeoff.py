#!/usr/bin/env python3
"""Бейк-офф канала речи: поле ``speech`` (FieldChannel) против тула ``speak_text`` (ToolChannel). ADR-0150 §4.4, В1/О4; #3697.

    python scripts/dialog/speech_channel_bakeoff.py [--providers deepseek,minimax,...] [--runs 3] [--limit 0]
        [--field-modes auto,tool_choice] [--concurrency 4] [--raw raw.jsonl] [--json out.json] [--out out.md]

Каждая фраза ``reference_set.yaml`` (кроме tier1: LLM там не зовётся) × канал × адаптер × ``--runs`` прогонов.
Тулы-заглушки только пишут события (имя вызова), побочных эффектов нет. Промпт НЕ боевой: меряется транспорт слота речи.

* FieldChannel — один вызов, речь в ``speech`` команды (режим по ``select_mode``; ``--field-modes`` добавляет принудительные).
* ToolChannel  — речь ``speak_text``, молчание ``stay_silent``; после тулов — второй вызов с заглушками результатов
  (как цикл v1): он и даёт ``llm_calls`` = 2 и шанс на дубль/утечку («Готово» в content).

Метрики по прогонам (сырые счётчики, смысл ответа не оценивается):
Определения ADR §4.4, на уровне запроса речи (``SpeechRequest`` = непустая строка из слота канала):
* ``delivered`` — ожидание фразы выполнено каналом: speech_only/ask — ровно одна непустая речь; actions — хоть одно
  действие; делитель — прогоны фраз, где что-то ожидается (не silence);
* ``leak``    — служебное в тексте речи (разметка, имена тулов, ``done``);
* ``double``  — два запроса речи на одну реплику (два ``speak_text``, в т.ч. во втором вызове цикла тулов);
* ``phantom`` — в речи отчёт об исполнении («Готово», «включил») до события владельца;
Сверх ADR (чтобы видеть причины недостачи):
* ``lost``    — речь ждали, слот пуст, а текст ушёл мимо слота (content / текст вне JSON): именно так терялась речь (#3269);
* ``stray``   — любой текст вне слота (его канал не озвучивает); ``spurious`` — молчать, а есть речь/действие, либо
  речь/вопрос, а есть действие;
* ``invalid`` — канал не собрал команду (нет JSON/тула); ``errors`` — отказ самого провайдера (не качество канала).
Отказ провайдера на каждом вызове -> строка «не прогнан: <причина>» (не 0 %). Ключи не печатаются.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import re
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from scripts.dialog import adapter_benchmark as ab  # noqa: E402  (добавляет в sys.path пакеты src/*)
from rob_box_dialog.command import Command, Invalid  # noqa: E402
from rob_box_dialog.command import parse as command_parse  # noqa: E402
from rob_box_dialog.degrade import select_mode  # noqa: E402
from rob_box_dialog.speech_channel import SPEAK_TOOL, FieldChannel, ModeReply, ToolChannel, _function_tool  # noqa: E402
from rob_box_llm.provider import LLMMessage, LLMProvider, LLMSettings, ToolResult  # noqa: E402
from scripts.dialog.turn_metrics import percentile  # noqa: E402

SILENT_TOOL = "stay_silent"
#: Служебное в тексте речи (ADR §4.4 ``leak``): разметка, имена тулов/полей, «done».
SERVICE_WORDS = re.compile(r"[{}\[\]]|speak_text|stay_silent|tool_call|\"speech\"|<think|passed|done", re.IGNORECASE)
#: Отчёт об исполнении в речи (ADR §4.4 ``phantom``): событие владельца ещё не пришло, а речь уже «сделано».
CLAIM_WORDS = re.compile(r"готово|сделано|выполнено|(?:включил|запустил|сделал|выполнил|поставил|переключил|сохранил)\w*", re.IGNORECASE)
PERSONA = "Ты голос робота Робби. К тебе обращаются словом «Робот»; фраза без него не тебе - молчи."
#: Доп. поля запроса на адаптер: v4-pro думает по умолчанию, а thinking-режим не принимает tool_choice (HTTP 400, 21.10).
ADAPTER_EXTRA: Mapping[str, Mapping[str, Any]] = {"deepseek-pro": {"thinking": {"type": "disabled"}}}
SYSTEM_FIELD = (
    f"{PERSONA} Ответь одной командой: поле speech - реплика (до 280 символов), actions - действия "
    "[{{tool, args}}] из списка: {tools}; passed=not_addressed - если фраза не тебе."
)
SYSTEM_TOOL = (
    f"{PERSONA} Говори ТОЛЬКО тулом speak_text (реплика до 280 символов); действия - тулами: {{tools}}; "
    "если фраза не тебе - вызови stay_silent. Текстом вне тулов не отвечай."
)
STUB_RESULT = "ok"


@dataclass(frozen=True)
class Phrase:
    id: str
    text: str
    kind: str  # speech_only | ask | actions | silence
    actions: tuple[str, ...] = ()


@dataclass
class Outcome:
    """Что канал вынул из ответов хода (все вызовы LLM хода сложены)."""

    speeches: list[str] = field(default_factory=list)
    actions: list[str] = field(default_factory=list)
    passed: bool = False
    leaks: list[str] = field(default_factory=list)  # текст вне слота
    invalid: str | None = None
    invalid_detail: str = ""  # Invalid.detail: какое поле/форма не прошли разбор
    head: str = ""  # начало ответа модели при invalid (без ключей) - для разбора причин
    llm_calls: int = 0
    latency_ms: float = 0.0
    served: str | None = None
    round2_error: str | None = None


def load_phrases(limit: int = 0) -> list[Phrase]:
    data = yaml.safe_load((REPO / "scripts" / "dialog" / "reference_set.yaml").read_text(encoding="utf-8"))
    out: list[Phrase] = []
    for p in data["phrases"]:
        exp = p["expected"]
        if "tier1" in exp:
            continue
        kind = "actions" if "actions" in exp else "ask" if "ask" in exp else "silence" if "silence" in exp else "speech_only"
        acts = tuple(a for a in exp.get("actions", ()) if a != SPEAK_TOOL)
        out.append(Phrase(p["id"], p["text"], kind, acts))
    return out[:limit] if limit else out


def action_tool_names(phrases: Sequence[Phrase]) -> list[str]:
    return sorted({a for p in phrases for a in p.actions})


def stub_tools(names: Sequence[str]) -> tuple[Mapping[str, Any], ...]:
    schema = {"type": "object", "properties": {"value": {"type": "string"}}}
    return tuple(_function_tool(n, f"Выполнить действие {n}", schema) for n in names)


def outside_json(text: str) -> str:
    """Текст ответа без первого JSON-объекта (то, что парсер команды отбрасывает и не озвучивает)."""
    start = text.find("{")
    while start != -1:
        try:
            _, end = json.JSONDecoder().raw_decode(text, start)
        except ValueError:
            start = text.find("{", start + 1)
            continue
        return (text[:start] + text[end:]).strip()
    return text.strip()


def field_outcome(reply: ModeReply, mode: str, out: Outcome) -> None:
    """Fill ``out`` from a FieldChannel reply (single call)."""
    parsed = FieldChannel({}).parse(reply, mode, command_parse)  # type: ignore[arg-type]
    free = reply.content.strip() if mode == "tool_choice" else outside_json(reply.content)
    if free:
        out.leaks.append(free)
    if isinstance(parsed, Invalid):
        out.invalid, out.invalid_detail = parsed.reason, parsed.detail
        out.head = ab.redact(reply.content or str(reply.tool_calls))[:200]
        return
    assert isinstance(parsed, Command)
    for text in (parsed.speech, parsed.ask.text if parsed.ask else None):
        if text:
            out.speeches.append(text)
    out.actions += [a.tool for a in parsed.actions]
    out.passed = bool(parsed.passed)


def tool_outcome(reply: ModeReply, out: Outcome) -> None:
    """Fill ``out`` from one ToolChannel reply; ``content`` is never read, so any text is a leak."""
    if reply.content.strip():
        out.leaks.append(reply.content.strip())
    for name, args in reply.tool_calls:
        if name == SPEAK_TOOL:
            text = str(args.get("text", "")).strip()
            if text:
                out.speeches.append(text)
        elif name == SILENT_TOOL:
            out.passed = True
        else:
            out.actions.append(name)


def judge(phrase: Phrase, out: Outcome) -> dict[str, bool]:
    """Метрики одного прогона по разметке ``expected`` (чистая функция; определения - в docstring модуля)."""
    speech = bool(out.speeches)
    expects = phrase.kind != "silence"
    wants_speech = phrase.kind in ("speech_only", "ask")
    ok = out.invalid is None and (bool(out.actions) if phrase.kind == "actions" else len(out.speeches) == 1)
    return {
        "expects": expects,
        "delivered": expects and ok,
        "leak": any(SERVICE_WORDS.search(s) for s in out.speeches),
        "double": len(out.speeches) > 1,
        "phantom": any(CLAIM_WORDS.search(s) for s in out.speeches),
        "lost": wants_speech and not speech and bool(out.leaks),
        "stray": bool(out.leaks),
        "spurious": (not expects and (speech or bool(out.actions))) or (wants_speech and bool(out.actions)),
        "invalid": out.invalid is not None,
    }


async def run_turn(
    provider: LLMProvider, channel: str, mode: str, phrase: Phrase, tools: Sequence[Mapping[str, Any]], names: Sequence[str],
    more: Mapping[str, Any] = {},  # noqa: B006 - только чтение
) -> Outcome:
    out = Outcome()
    lst = ", ".join(names)
    if channel == "field":
        ch: Any = FieldChannel(ab.SCHEMA)
        req = ch.build_request(mode)
        system = SYSTEM_FIELD.format(tools=lst) + " " + req.instruction
    else:
        silent = _function_tool(SILENT_TOOL, "Промолчать: фраза обращена не к роботу", {"type": "object", "properties": {}})
        req = ToolChannel((*tools, silent)).build_request(mode)
        system = SYSTEM_TOOL.format(tools=lst)
    extra = {**({"response_format": req.response_format} if req.response_format else {}), **more}
    settings = LLMSettings(tool_choice=req.tool_choice, extra=extra)
    msgs = [LLMMessage(role="system", content=system), LLMMessage(role="user", content=phrase.text)]
    started = time.perf_counter()
    resp = await provider.complete(msgs, tools=req.tools, settings=settings)
    out.llm_calls, out.served = 1, ab.served_model(resp)
    reply = ModeReply(resp.content, tuple((tc.name, tc.arguments) for tc in resp.tool_calls))
    if channel == "field":
        field_outcome(reply, mode, out)
    else:
        tool_outcome(reply, out)
        await tool_round_two(provider, msgs, resp, req, out, more)
    out.latency_ms = (time.perf_counter() - started) * 1000
    return out


async def tool_round_two(
    provider: LLMProvider, msgs: list[LLMMessage], resp: Any, req: Any, out: Outcome, more: Mapping[str, Any]
) -> None:
    """Как цикл v1: результаты тулов (заглушки) идут обратно модели; всё, что она скажет ещё, считается."""
    calls = [tc for tc in resp.tool_calls if tc.name != SILENT_TOOL]
    if not calls or out.passed:
        return
    follow = [*msgs, LLMMessage(role="assistant", content=resp.content or "", tool_calls=tuple(resp.tool_calls))]
    follow += [LLMMessage(role="tool", content=STUB_RESULT, tool_result=ToolResult(tc.id, STUB_RESULT)) for tc in resp.tool_calls]
    out.llm_calls = 2
    try:
        again = await provider.complete(follow, tools=req.tools, settings=LLMSettings(tool_choice="auto", extra=more))
    except Exception as exc:  # noqa: BLE001 - второй вызов упал: ход уже собран первым, пишем причину
        out.round2_error = ab.error_reason(exc)
        return
    tool_outcome(ModeReply(again.content, tuple((tc.name, tc.arguments) for tc in again.tool_calls)), out)


async def with_backoff(call: Any, retries: int) -> Any:
    """Повтор вызова при 429 (общий ключ с живым роботом): паузы 10, 20, 30 с. Остальные ошибки - сразу наружу."""
    for attempt in range(retries + 1):
        try:
            return await call()
        except Exception as exc:  # noqa: BLE001
            if attempt == retries or "RateLimit" not in type(exc).__name__:
                raise
            await asyncio.sleep(10 * (attempt + 1))


@dataclass
class Cell:
    """Агрегат по (адаптер, канал/режим)."""

    outcomes: list[tuple[Phrase, Outcome]] = field(default_factory=list)
    errors: Counter = field(default_factory=Counter)


async def run_cell(
    provider: LLMProvider, channel: str, mode: str, phrases: Sequence[Phrase], runs: int, concurrency: int,
    names: Sequence[str], raw: Any = None, label: str = "", more: Mapping[str, Any] = {}, retries: int = 0,  # noqa: B006
) -> Cell:
    cell, gate, tools = Cell(), asyncio.Semaphore(concurrency), stub_tools(names)

    async def one(phrase: Phrase, run: int) -> None:
        async with gate:
            try:
                out = await with_backoff(lambda: run_turn(provider, channel, mode, phrase, tools, names, more), retries)
            except Exception as exc:  # noqa: BLE001 - отказ провайдера != качество канала
                cell.errors[ab.error_reason(exc)] += 1
                return
        cell.outcomes.append((phrase, out))
        if raw is not None:
            raw.write(json.dumps({"cell": label, "id": phrase.id, "run": run, "kind": phrase.kind, **judge(phrase, out),
                                  "speeches": out.speeches, "actions": out.actions, "passed": out.passed,
                                  "leaks": [ab.redact(x)[:200] for x in out.leaks], "invalid": out.invalid,
                                  "invalid_detail": out.invalid_detail, "head": out.head,
                                  "llm_calls": out.llm_calls, "ms": round(out.latency_ms), "served": out.served,
                                  "round2_error": out.round2_error}, ensure_ascii=False) + "\n")
            raw.flush()

    await asyncio.gather(*(one(p, r) for p in phrases for r in range(runs)))
    return cell


def summarize(cell: Cell, total: int) -> dict[str, Any]:
    errors = sum(cell.errors.values())
    if total and errors == total:
        return {"calls": total, "not_run": "; ".join(f"{r} ×{n}" for r, n in cell.errors.most_common(2))}
    judged = [(p, o, judge(p, o)) for p, o in cell.outcomes]
    expecting = [j for _, _, j in judged if j["expects"]]
    lat = [o.latency_ms for _, o, _ in judged]
    return {
        "calls": total, "errors": errors, "error_reasons": dict(cell.errors),
        "expecting": len(expecting), "delivered": sum(j["delivered"] for j in expecting),
        **{k: sum(j[k] for _, _, j in judged) for k in ("leak", "double", "phantom", "lost", "stray", "spurious", "invalid")},
        "silence_runs": sum(1 for p, _, _ in judged if p.kind == "silence"),
        "llm_calls": sum(o.llm_calls for _, o, _ in judged) / max(1, len(judged)),
        "p50": percentile(lat, 0.5), "p95": percentile(lat, 0.95),
        "served": dict(Counter(o.served or "?" for _, o, _ in judged)),
        "invalid_reasons": dict(Counter(o.invalid for _, o, _ in judged if o.invalid)),
        "round2_errors": sum(1 for _, o, _ in judged if o.round2_error),
    }


def render(rows: Sequence[tuple[str, str, str, dict[str, Any] | None]], n_phrases: int, runs: int) -> str:
    cols = ("leak", "double", "phantom", "lost", "stray", "spurious", "invalid", "errors")
    out = [
        f"### speech_channel_bakeoff ({n_phrases} фраз × {runs} прогона; парсер: rob_box_dialog.command.parse)",
        "",
        "| адаптер | модель (из ответа API) | канал | режим | прогонов | delivered | " + " | ".join(cols)
        + " | llm_calls | p50, мс | p95, мс |",
        "|" + "---|" * (9 + len(cols)),
    ]
    for name, channel, mode, res in rows:
        if res is None or "not_run" in res:
            why = ab.unavailable(name) if res is None else res["not_run"]
            out.append(f"| {name} | — | {channel} | {mode} | — | не прогнан: {why} |" + " — |" * (len(cols) + 3))
            continue
        pct = f"{100 * res['delivered'] / res['expecting']:.0f} %" if res["expecting"] else "—"
        out.append(
            f"| {name} | {', '.join(res['served'])} | {channel} | {mode} | {res['calls']} | {res['delivered']}/{res['expecting']} "
            f"({pct}) | " + " | ".join(str(res[c]) for c in cols)
            + f" | {res['llm_calls']:.2f} | {ab._ms(res['p50'])} | {ab._ms(res['p95'])} |"
        )
    return "\n".join(out)


def rescore(raw_path: Path, summary_path: Path | None) -> list[Any]:
    """Пересчитать таблицу из сырого JSONL (определения метрик меняются дешевле, чем платные прогоны).

    Отказы провайдера в JSONL не пишутся - их число и причины берутся из сводки прогона (``--summary``)."""
    phrases = {p.id: p for p in load_phrases()}
    errs = {(r[0], r[1], r[2]): r[3] for r in json.loads(summary_path.read_text(encoding="utf-8"))} if summary_path else {}
    cells: dict[str, Cell] = {}
    for line in raw_path.read_text(encoding="utf-8").splitlines():
        row = json.loads(line)
        out = Outcome(row["speeches"], row["actions"], row["passed"], row["leaks"], row["invalid"],
                      row.get("invalid_detail", ""), row.get("head", ""), row["llm_calls"], row["ms"], row["served"],
                      row["round2_error"])
        cells.setdefault(row["cell"], Cell()).outcomes.append((phrases[row["id"]], out))
    rows: list[Any] = []
    for label, cell in cells.items():
        name, channel, mode = label.split("/")
        old = errs.get((name, channel, mode)) or {}
        cell.errors.update(old.get("error_reasons", {}))
        rows.append((name, channel, mode, summarize(cell, len(cell.outcomes) + sum(cell.errors.values()))))
    rows += [(k[0], k[1], k[2], v) for k, v in errs.items() if v is None or "not_run" in v]
    order = list(ab.KEY_ENV)
    return sorted(rows, key=lambda r: (order.index(r[0]) if r[0] in order else 99, r[1] != "field", r[2]))


def raw_phrase_count(raw_path: Path) -> int:
    """Сколько фраз было в прогоне (по сырому JSONL), а не в нынешнем наборе: набор растёт (#3696 добавил В19)."""
    return len({json.loads(line)["id"] for line in raw_path.read_text(encoding="utf-8").splitlines() if line.strip()})


def plan(provider: LLMProvider, field_modes: Sequence[str]) -> list[tuple[str, str]]:
    """(канал, режим) для адаптера: field в режиме по ``select_mode`` и принудительных, tool — по умолчанию адаптера."""
    auto = select_mode(provider.capabilities)
    modes = [auto, *[m for m in field_modes if m not in ("auto", auto)]]
    return [("field", m) for m in modes] + [("tool", auto)]


async def bench_adapter(name: str, args: argparse.Namespace, phrases: Sequence[Phrase], raw: Any) -> list[Any]:
    provider = ab.build_provider(name)
    if provider is None:
        return [(name, "—", "—", None)]
    names, rows = action_tool_names(phrases), []
    for channel, mode in plan(provider, args.field_modes.split(",")):
        cell = await run_cell(
            provider, channel, mode, phrases, args.runs, args.concurrency, names, raw, f"{name}/{channel}/{mode}",
            ADAPTER_EXTRA.get(name, {}), args.retry_429,
        )
        rows.append((name, channel, mode, summarize(cell, len(phrases) * args.runs)))
    return rows


async def main_async(args: argparse.Namespace) -> tuple[str, list[Any]]:
    ab.load_env(Path(args.env) if args.env else None)
    phrases = load_phrases(args.limit)
    raw = open(args.raw, "w", encoding="utf-8") if args.raw else None
    try:
        parts = await asyncio.gather(*(bench_adapter(n.strip(), args, phrases, raw) for n in args.providers.split(",") if n.strip()))
    finally:
        if raw:
            raw.close()
    rows = [r for part in parts for r in part]
    return render(rows, len(phrases), args.runs), rows


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--env", help="файл .env с ключами (по умолчанию только окружение)")
    ap.add_argument("--providers", default=",".join(ab.KEY_ENV))
    ap.add_argument("--field-modes", default="auto,tool_choice", help="режимы FieldChannel сверх select_mode")
    ap.add_argument("--limit", type=int, default=0, help="сколько фраз (0 = все не-tier1)")
    ap.add_argument("--runs", type=int, default=3)
    ap.add_argument("--concurrency", type=int, default=4)
    ap.add_argument("--retry-429", type=int, default=0, help="повторов вызова при RateLimitError (паузы 10/20/30 с)")
    ap.add_argument("--raw", help="JSONL: строка на прогон")
    ap.add_argument("--json", help="сводка в JSON")
    ap.add_argument("--out", help="markdown в файл")
    ap.add_argument("--rescore", help="не звать LLM: пересчитать таблицу из JSONL (--raw прошлого прогона)")
    ap.add_argument("--summary", help="с --rescore: --json прошлого прогона (отказы провайдеров, not_run)")
    args = ap.parse_args(argv)
    if args.rescore:
        rows = rescore(Path(args.rescore), Path(args.summary) if args.summary else None)
        report = render(rows, raw_phrase_count(Path(args.rescore)), args.runs)
    else:
        report, rows = asyncio.run(main_async(args))
    print(report)
    if args.out:
        Path(args.out).write_text(report + "\n", encoding="utf-8")
    if args.json:
        Path(args.json).write_text(json.dumps([list(r) for r in rows], ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
