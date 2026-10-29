#!/usr/bin/env python3
"""Замер адаптеров LLM по эталонному набору в выбранном режиме команд (ADR-0150 §4.5, A14; карточка #3686).

    python scripts/dialog/adapter_benchmark.py [--env FILE] [--providers deepseek,minimax,mimo,qwen-flash,jev]
        [--mode auto|structured_output|tool_choice|json_mode|json_in_text] [--limit 20] [--runs 1] [--out report.md]

Для каждого адаптера, чей ключ найден (``DEEPSEEK_API_KEY`` / ``MINIMAX_API_KEY`` / ``MIMO_API_KEY`` из окружения
или из ``--env``), гоняет фразы ``reference_set.yaml`` (кроме tier1: LLM там не зовётся) через ``FieldChannel``
и печатает markdown: ``delivered / invalid / latency_p50 / latency_p95`` по адаптеру.

Метрики (сырые, без оценки смысла):
* ``delivered`` — ``rob_box_dialog.command.parse`` дал команду, где есть хоть одно из speech / actions / ask / passed;
* ``invalid``   — парсер вернул ``Invalid`` (нет JSON, неверная форма) или команда пустая; причины — в ``reasons``;
* ``errors``    — отказ САМОГО провайдера (402/401/сеть/таймаут): это не качество режима, в ``invalid`` не идёт;
  если отказали все вызовы — адаптер «не прогнан: <причина>», а не «0 %».
  ``delivered + invalid + errors = вызовов``. В таблицу пишется модель из ответа API (``served``), не запрошенная.
Промпт здесь минимальный и НЕ боевой: замер проверяет форму ответа в режиме, а не качество диалога.
Ключи не печатаются. Адаптер без ключа выводится строкой «не прогнано: нет ключа» — не пропускается молча.

Путь моста (``--cells``, #3698 ч.7d): ход идёт РОВНО как у робота в режиме команд — ``CommandModeLLM.shape``
(схема ``knowledge.command_schema``, добавка ``mode_note``, тулы каталога), ``CommandModeLLM.answer`` (``judge`` по срезу
тулов, один повтор с нарушениями). Ячейка — ``адаптер[:модель]/режим[/вариант]``, например
``minimax:MiniMax-M3/tool_offered/adaptive``; вариант — поля тела из :data:`VARIANTS` (thinking, ``reasoning_split``).
Метрики хода — как в бейк-оффе PR-1c (#3697): ``delivered`` по разметке ``expected`` (первый ответ и после повтора),
``phantom`` (в речи «готово/включил», а действий нет), ``spurious``; плюс причины invalid, ответы без тула, двойные
``command``, ``finish_reason``, ``<think>`` в тексте / ``reasoning_content``, токены (``cached_tokens``), p50/p90.
``--raw`` — строка JSON на ход; ``--retry-429 N`` — пауза 10/20/30… с на 429 (общий ключ с живым роботом).
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import re
import sys
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

REPO = Path(__file__).resolve().parents[2]
_PACKAGES = ("rob_box_dialog", "rob_box_llm", "rob_box_core", "rob_box_harness", "rob_box_voice")
for _p in (*(REPO / "src" / p for p in _PACKAGES), REPO):  # REPO первым: у rob_box_voice свой каталог scripts/
    sys.path.insert(0, str(_p))

from rob_box_dialog.command import Command  # noqa: E402
from rob_box_dialog.command import parse as command_parse  # noqa: E402
from rob_box_dialog.degrade import MODE_ORDER, select_mode  # noqa: E402
from rob_box_dialog.speech_channel import ModeReply, FieldChannel  # noqa: E402
from rob_box_llm.provider import LLMMessage, LLMProvider, LLMSettings  # noqa: E402
from scripts.dialog.turn_metrics import percentile  # noqa: E402

KEY_ENV = {
    "deepseek": "DEEPSEEK_API_KEY", "deepseek-pro": "DEEPSEEK_API_KEY", "minimax": "MINIMAX_API_KEY",
    "minimax-flash": "MINIMAX_API_KEY", "mimo": "MIMO_API_KEY", "qwen-flash": "QWEN_API_KEY",
    "qwen-max": "QWEN_API_KEY", "jev": "JEV_API_KEY",
}
QWEN_BASE_URL = "https://token-plan.maas.qwencloudapi.com/compatible-mode/v1"
#: Имена моделей — как их отдаёт ``GET /models`` (сверено 21.10); у ``deepseek`` модель — дефолт кода (``deepseek-v4-flash``).
ADAPTER_MODELS = {
    "deepseek-pro": "deepseek-v4-pro", "minimax-flash": "MiniMax-M3.1-Flash", "qwen-flash": "qwen3.8-flash",
    "qwen-max": "qwen3.8-max",
}
#: Адаптеры, которые нельзя гонять как чат: причина фиксируется строкой, а не молчаливым пропуском.
UNAVAILABLE = {
    "jev": "System One API (choice/score): POST /v1/chat/completions -> 404, речь/тулы им не выражаются",
}
SCHEMA: Mapping[str, Any] = {
    "type": "object",
    "properties": {
        "speech": {"type": "string", "maxLength": 280},
        "actions": {"type": "array", "maxItems": 3, "items": {"type": "object"}},
        "passed": {"type": "string", "enum": ["not_addressed", "nothing_to_say", "unclear"]},
    },
}
SYSTEM = (
    "Ты голос робота Робби. Ответь одной командой: поле speech — реплика (до 280 символов), actions — действия "
    "[{tool, args}], passed — если фраза не тебе. Если тебе не говорили — passed=not_addressed."
)


def load_env(path: Path | None) -> None:
    """Дописать переменные из .env в окружение, не затирая уже заданные. Значения нигде не печатаются."""
    if path is None or not path.is_file():
        return
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            key, _, value = line.partition("=")
            os.environ.setdefault(key.strip(), value.strip().strip("'\""))


def build_provider(name: str, timeout: float = 60.0) -> LLMProvider | None:
    """Адаптер по имени из ``KEY_ENV`` (``имя:модель`` — другая модель того же адаптера); ``None`` — нет ключа или
    адаптер не чатовый (причина: :func:`unavailable`)."""
    name, _, model_override = name.partition(":")
    key = os.environ.get(KEY_ENV[name])
    if not key or name in UNAVAILABLE:
        return None
    from rob_box_llm.providers.deepseek import DeepSeekProvider, _OpenAICompatibleProvider
    from rob_box_llm.providers.mimo import MiMoProvider
    from rob_box_llm.providers.minimax import MiniMaxProvider

    model = model_override or ADAPTER_MODELS.get(name)
    if name.startswith("qwen"):
        return _OpenAICompatibleProvider(
            name="qwen", base_url=QWEN_BASE_URL, default_model=str(model), api_key=key, timeout=timeout
        )
    cls = {"deepseek": DeepSeekProvider, "minimax": MiniMaxProvider, "mimo": MiMoProvider}[name.split("-")[0]]
    return cls(api_key=key, timeout=timeout, **({"model": model} if model else {}))


def unavailable(name: str) -> str:
    """Почему адаптер не прогнан без обращения к сети."""
    return UNAVAILABLE.get(name.partition(":")[0]) or "нет ключа"


def redact(text: str) -> str:
    """Убрать из текста ошибки любые значения секретов окружения (ключи не печатаются, ADR/AGENTS)."""
    for env in set(KEY_ENV.values()):
        val = os.environ.get(env)
        if val and len(val) >= 8:
            text = text.replace(val, "***")
    return text


def error_reason(exc: BaseException) -> str:
    """Короткая причина отказа провайдера: класс + начало сообщения (без ключей)."""
    text = re.sub(r"\s*\(request_id: [^)]*\)", "", f"{type(exc).__name__}: {exc}")  # id запроса делает причины разными
    return redact(text)[:160]


def served_model(resp: Any) -> str | None:
    """Модель, реально обслужившая запрос (поле ``model`` ответа API), а не запрошенное имя."""
    return getattr(getattr(resp, "raw", None), "model", None)


def load_phrases(limit: int) -> list[str]:
    data = yaml.safe_load((REPO / "scripts" / "dialog" / "reference_set.yaml").read_text(encoding="utf-8"))
    texts = [p["text"] for p in data["phrases"] if "tier1" not in p["expected"]]
    return texts[:limit] if limit else texts


def is_delivered(parsed: Any) -> bool:
    return isinstance(parsed, Command) and any((parsed.speech, parsed.actions, parsed.ask, parsed.passed))


def classify(parsed: Any) -> str:
    """Исход разбора: ``delivered`` или причина invalid (``Invalid.reason`` / ``empty_command``)."""
    if is_delivered(parsed):
        return "delivered"
    return getattr(parsed, "reason", "empty_command")


async def run_adapter(
    provider: LLMProvider, mode: str, phrases: Sequence[str], runs: int, parse: Any = command_parse,
    concurrency: int = 1,
) -> dict[str, Any]:
    channel = FieldChannel(SCHEMA)
    req = channel.build_request(mode)  # type: ignore[arg-type]
    extra = {"response_format": req.response_format} if req.response_format else {}
    settings = LLMSettings(tool_choice=req.tool_choice, extra=extra)
    gate = asyncio.Semaphore(concurrency)
    stats: dict[str, Any] = {"outcomes": Counter(), "errors": Counter(), "latencies": [], "served": Counter(), "samples": []}

    async def one(text: str) -> None:
        msgs = [LLMMessage(role="system", content=f"{SYSTEM} {req.instruction}"), LLMMessage(role="user", content=text)]
        async with gate:
            started = time.perf_counter()
            try:
                resp = await provider.complete(msgs, tools=req.tools, settings=settings)
            except Exception as exc:  # noqa: BLE001 - отказ провайдера != качество режима
                stats["errors"][error_reason(exc)] += 1
                return
            stats["latencies"].append((time.perf_counter() - started) * 1000)
        stats["served"][served_model(resp) or "?"] += 1
        reply = ModeReply(resp.content, tuple((tc.name, tc.arguments) for tc in resp.tool_calls))
        outcome = classify(channel.parse(reply, mode, parse))  # type: ignore[arg-type]
        stats["outcomes"][outcome] += 1
        if outcome != "delivered" and len(stats["samples"]) < 3:
            stats["samples"].append(redact(reply.content or str(reply.tool_calls))[:160])

    await asyncio.gather(*(one(t) for t in phrases for _ in range(runs)))
    return summarize(stats, len(phrases) * runs)


def summarize(stats: Mapping[str, Any], total: int) -> dict[str, Any]:
    errors = sum(stats["errors"].values())
    if total and errors == total:  # отказал сам провайдер на каждом вызове: не 0 %, а «не прогнан»
        return {"calls": total, "not_run": "; ".join(f"{r} ×{n}" for r, n in stats["errors"].most_common(2))}
    delivered = stats["outcomes"]["delivered"]
    return {
        "calls": total,
        "delivered": delivered,
        "invalid": sum(stats["outcomes"].values()) - delivered,
        "errors": errors,
        "reasons": {k: v for k, v in stats["outcomes"].items() if k != "delivered"},
        "error_reasons": dict(stats["errors"]),
        "served": dict(stats["served"]),
        "samples": list(stats["samples"]),
        "p50": percentile(stats["latencies"], 0.5),
        "p95": percentile(stats["latencies"], 0.95),
    }


def render(rows: Sequence[tuple[str, str, dict[str, Any] | None]], parser_kind: str, n_phrases: int) -> str:
    out = [
        f"### adapter_benchmark ({n_phrases} фраз, парсер: {parser_kind})",
        "",
        "| адаптер | модель (из ответа API) | режим | вызовов | delivered | invalid | ошибок провайдера | p50, мс | p95, мс |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for name, mode, res in rows:
        if res is None or "not_run" in res:
            why = unavailable(name) if res is None else res["not_run"]
            out.append(f"| {name} | — | {mode} | — | не прогнано: {why} | — | — | — | — |")
            continue
        pct = f"{100 * res['delivered'] / res['calls']:.0f} %" if res["calls"] else "—"
        served = ", ".join(res["served"]) or "—"
        out.append(
            f"| {name} | {served} | {mode} | {res['calls']} | {res['delivered']} ({pct}) | {res['invalid']} "
            f"{res['reasons'] or ''} | {res['errors']} | {_ms(res['p50'])} | {_ms(res['p95'])} |"
        )
    return "\n".join(out)


def _ms(value: float | None) -> str:
    return "—" if value is None else f"{value:.0f}"


# ---------------------------------------------------------------------------------------------------------------
# Путь моста режима команд (--cells): ход как у робота, метрики бейк-оффа PR-1c
# ---------------------------------------------------------------------------------------------------------------

#: Поля тела запроса поверх политики адаптера (``robot`` — как у робота: M3 ``thinking: disabled``, Flash ``low``).
VARIANTS: Mapping[str, Mapping[str, Any]] = {
    "robot": {},
    "split": {"reasoning_split": True},
    "adaptive": {"thinking": {"type": "adaptive"}},
    "adaptive_split": {"thinking": {"type": "adaptive"}, "reasoning_split": True},
    "effort_high": {"reasoning_effort": "high"},
}
#: Отчёт об исполнении в речи (``phantom`` бейк-оффа PR-1c): «готово» без события владельца.
CLAIM_WORDS = re.compile(
    r"готово|сделано|выполнено|(?:включил|запустил|сделал|выполнил|поставил|переключил|сохранил)\w*", re.IGNORECASE
)
BRIDGE_PERSONA = (
    "Ты голос робота Робби. К тебе обращаются словом «Робот»; фраза без него не тебе - промолчи (passed). "
    "Тулы действий: {tools}."
)
#: Боевой мастер-промпт робота (``system_prompt_file`` в ``dialogue_node.yaml``) — для ``--system-file``.
MASTER_PROMPT = REPO / "src" / "rob_box_voice" / "prompts" / "master_prompt_compact.txt"
#: Как у робота: ``minimax`` / ``deepseek`` в ``docker/sound/config/voice_assistant/dialogue_node.yaml``.
ROBOT_TEMPERATURE, ROBOT_MAX_TOKENS = 0.7, 3000


@dataclass(frozen=True)
class Phrase:
    id: str
    text: str
    kind: str  # speech_only | ask | actions | silence
    actions: tuple[str, ...] = ()


def load_reference(limit: int = 0) -> list[Phrase]:
    """Фразы ``reference_set.yaml`` без tier1 (там LLM не зовётся), с видом ожидания."""
    data = yaml.safe_load((REPO / "scripts" / "dialog" / "reference_set.yaml").read_text(encoding="utf-8"))
    out = []
    for p in data["phrases"]:
        exp = p["expected"]
        if "tier1" in exp:
            continue
        kind = next((k for k in ("actions", "ask", "silence") if k in exp), "speech_only")
        out.append(Phrase(p["id"], p["text"], kind, tuple(exp.get("actions", ()))))
    return out[:limit] if limit else out


def catalog_tools(phrases: Sequence[Phrase]) -> list[Mapping[str, Any]]:
    """Тулы действий, которых ждёт набор, — из каталога с настоящими схемами (видимые LLM, не тул речи)."""
    from rob_box_core.tool_catalog import get_tool
    from rob_box_dialog import knowledge as kn

    out = []
    for name in sorted({a for p in phrases for a in p.actions} - set(kn.SPEECH_CHANNEL_TOOLS)):
        try:
            entry = get_tool(name)
        except KeyError:
            continue
        if entry.llm_visible:
            out.append({"type": "function", "function": {"name": entry.name, "description": entry.description,
                                                         "parameters": dict(entry.parameters)}})
    return out


def spoken_text(cmd: Command) -> str:
    if cmd.speech:
        return cmd.speech
    if cmd.ask is not None and cmd.ask.text:
        return cmd.ask.text
    return "\n".join(s for s in cmd.perform.segments if s) if cmd.perform is not None else ""


def turn_verdict(phrase: Phrase, result: Any) -> dict[str, Any]:
    """Метрики одного ответа по разметке (как ``judge`` бейк-оффа PR-1c, на команде моста)."""
    if not isinstance(result, Command):
        return {"ok": False, "reason": getattr(result, "reason", "?"), "phantom": False, "spurious": False}
    said = spoken_text(result)
    acts = [a.tool for a in result.actions]
    if phrase.kind == "actions":
        ok = bool(acts) or (result.perform is not None and "perform" in phrase.actions)
    elif phrase.kind == "silence":
        ok = bool(result.passed)
    else:
        ok = bool(said)
    return {
        "ok": ok, "reason": "delivered" if ok else "missed", "phantom": bool(said) and not acts and bool(CLAIM_WORDS.search(said)),
        "spurious": (phrase.kind == "silence" and bool(said or acts)) or (phrase.kind != "actions" and bool(acts)),
        "speech": redact(said)[:160], "actions": acts, "passed": result.passed,
    }


def call_facts(resp: Any, ms: float) -> dict[str, Any]:
    """Что пришло в одном ответе: вызовы, текст, мысли, токены (из ``raw`` ответа API)."""
    from rob_box_dialog import knowledge as kn

    raw = getattr(resp, "raw", None)
    choice = (getattr(raw, "choices", None) or [None])[0]
    msg = getattr(choice, "message", None)
    usage = getattr(raw, "usage", None)
    details = getattr(usage, "prompt_tokens_details", None)
    return {
        "ms": round(ms), "served": served_model(resp), "finish": resp.finish_reason,
        "calls": [c.name for c in resp.tool_calls],
        "command_calls": sum(1 for c in resp.tool_calls if c.name == kn.COMMAND_TOOL),
        "content_len": len(resp.content or ""), "think_in_content": "<think>" in (resp.content or ""),
        "reasoning_content": bool(getattr(msg, "reasoning_content", None) or getattr(msg, "reasoning_details", None)),
        "prompt_tokens": getattr(usage, "prompt_tokens", None), "completion_tokens": getattr(usage, "completion_tokens", None),
        "cached_tokens": getattr(details, "cached_tokens", None),
    }


async def with_backoff(call: Any, retries: int) -> Any:
    """Повтор вызова при 429 (общий ключ с живым роботом): паузы 10, 20, 30 с. Остальные ошибки — сразу наружу."""
    for attempt in range(retries + 1):
        try:
            return await call()
        except Exception as exc:  # noqa: BLE001
            if attempt == retries or "RateLimit" not in type(exc).__name__:
                raise
            await asyncio.sleep(10 * (attempt + 1))


async def bridge_turn(provider: LLMProvider, mode: str, extra: Mapping[str, Any], phrase: Phrase,
                      tools: Sequence[Mapping[str, Any]], retries: int, system_text: str = "") -> dict[str, Any]:
    """Один ход через мост режима команд робота; ответы модели и вердикты — первого ответа и итоговый.

    ``system_text`` — системный промпт (пусто — минимальный :data:`BRIDGE_PERSONA`); температура и лимит вывода —
    как в конфиге робота (без ``max_tokens`` MiniMax режет ответ на своём дефолте)."""
    from rob_box_harness.providers.reasoning import without_reasoning
    from rob_box_voice.core.command_bridge import CommandModeLLM, judge

    wrapped = CommandModeLLM(provider)
    wrapped.mode = mode  # type: ignore[assignment]  # режим ячейки, а не select_mode
    names = ", ".join(t["function"]["name"] for t in tools)
    system = LLMMessage(role="system", content=system_text or BRIDGE_PERSONA.format(tools=names))
    settings = LLMSettings(temperature=ROBOT_TEMPERATURE, max_tokens=ROBOT_MAX_TOKENS, extra=dict(extra))
    msgs, offered, shaped = wrapped.shape([system, LLMMessage(role="user", content=phrase.text)], tools, settings)
    responses: list[Any] = []
    facts: list[dict[str, Any]] = []

    async def ask(m: list[LLMMessage]) -> Any:
        started = time.perf_counter()
        resp = await with_backoff(lambda: provider.complete(m, tools=offered, settings=shaped), retries)
        responses.append(resp)
        facts.append(call_facts(resp, (time.perf_counter() - started) * 1000))
        return resp

    started = time.perf_counter()
    await wrapped.answer(ask, msgs, offered)
    available = frozenset(t["function"]["name"] for t in offered)
    first, final = (judge(without_reasoning(r), available) for r in (responses[0], responses[-1]))
    return {"id": phrase.id, "kind": phrase.kind, "first": turn_verdict(phrase, first),
            "final": turn_verdict(phrase, final), "retried": len(responses) > 1,
            "ms_total": round((time.perf_counter() - started) * 1000), "responses": facts,
            "first_detail": redact(getattr(first, "detail", "") or "")[:160],
            "first_head": "" if isinstance(first, Command) else redact(responses[0].content or "")[:160]}


def parse_cell(spec: str) -> tuple[str, str, str]:
    """``адаптер[:модель]/режим[/вариант]`` → (адаптер, режим, вариант); незнакомое — ``ValueError``."""
    parts = spec.split("/")
    adapter, mode, variant = parts[0], parts[1] if len(parts) > 1 else "auto", parts[2] if len(parts) > 2 else "robot"
    if mode not in ("auto", *MODE_ORDER) or variant not in VARIANTS:
        raise ValueError(f"ячейка {spec!r}: режим из {MODE_ORDER}, вариант из {tuple(VARIANTS)}")
    return adapter, mode, variant


async def run_cell(spec: str, phrases: Sequence[Phrase], runs: int, retries: int, raw: Any = None,
                   system_text: str = "") -> dict[str, Any]:
    adapter, mode, variant = parse_cell(spec)
    provider = build_provider(adapter)
    if provider is None:
        return {"cell": spec, "not_run": unavailable(adapter)}
    mode = select_mode(provider.capabilities) if mode == "auto" else mode
    tools, rows, errors = catalog_tools(phrases), [], Counter()
    for run in range(runs):
        for phrase in phrases:
            try:
                row = await bridge_turn(provider, mode, VARIANTS[variant], phrase, tools, retries, system_text)
            except Exception as exc:  # noqa: BLE001 - отказ провайдера != качество режима
                errors[error_reason(exc)] += 1
                continue
            row.update(cell=spec, run=run)
            rows.append(row)
            if raw is not None:
                raw.write(json.dumps(row, ensure_ascii=False) + "\n")
                raw.flush()
    await provider.aclose()
    return summarize_cell(spec, rows, errors, len(phrases) * runs)


def summarize_cell(spec: str, rows: Sequence[Mapping[str, Any]], errors: Counter, total: int) -> dict[str, Any]:
    """Сводка ячейки из строк ходов (её же пересчитывает ``--rescore`` из ``--raw``)."""
    if total and sum(errors.values()) == total:
        return {"cell": spec, "not_run": "; ".join(f"{r} ×{n}" for r, n in errors.most_common(2))}
    expecting = [r for r in rows if r["kind"] != "silence"]
    calls = [c for r in rows for c in r["responses"]]

    def count(pred: Any) -> int:
        return sum(1 for r in rows if pred(r))

    def mean(key: str) -> float | None:
        vals = [c[key] for c in calls if isinstance(c.get(key), int)]
        return sum(vals) / len(vals) if vals else None

    return {
        "cell": spec, "turns": total, "errors": sum(errors.values()), "error_reasons": dict(errors),
        "served": dict(Counter(c["served"] or "?" for c in calls)), "expecting": len(expecting),
        "delivered_first": sum(r["first"]["ok"] for r in expecting), "delivered": sum(r["final"]["ok"] for r in expecting),
        "invalid_first": dict(Counter(r["first"]["reason"] for r in rows if r["first"]["reason"] not in ("delivered", "missed"))),
        "invalid_final": dict(Counter(r["final"]["reason"] for r in rows if r["final"]["reason"] not in ("delivered", "missed"))),
        "retried": count(lambda r: r["retried"]),
        "fixed": count(lambda r: r["retried"] and r["final"]["reason"] in ("delivered", "missed")),
        "phantom": count(lambda r: r["final"]["phantom"]), "spurious": count(lambda r: r["final"]["spurious"]),
        "no_tool": sum(1 for c in calls if not c["calls"]), "multi_command": sum(1 for c in calls if c["command_calls"] > 1),
        "finish": dict(Counter(str(c["finish"]) for c in calls)),
        "think_in_content": sum(c["think_in_content"] for c in calls),
        "reasoning_content": sum(c["reasoning_content"] for c in calls), "llm_calls": len(calls),
        "p50": percentile([r["ms_total"] for r in rows], 0.5), "p90": percentile([r["ms_total"] for r in rows], 0.9),
        "prompt_tokens": mean("prompt_tokens"), "completion_tokens": mean("completion_tokens"),
        "cached_tokens": mean("cached_tokens"),
    }


def render_cells(rows: Sequence[Mapping[str, Any]], n_phrases: int, runs: int) -> str:
    out = [
        f"### adapter_benchmark, мост режима команд ({n_phrases} фраз × {runs})",
        "",
        "| ячейка | модель (из ответа API) | ходов | delivered 1-й | delivered итог | invalid 1-й | invalid итог | "
        "повтор / исправил | phantom | spurious | без тула | 2× command | finish | think / reasoning | вызовов LLM | "
        "p50 / p90, мс | токены вход / выход / кэш |",
        "|" + "---|" * 17,
    ]
    for r in rows:
        if "not_run" in r:
            out.append(f"| {r['cell']} | — | — | не прогнано: {r['not_run']} |" + " — |" * 13)
            continue
        exp = r["expecting"] or 1
        tok = " / ".join("—" if r[k] is None else f"{r[k]:.0f}" for k in ("prompt_tokens", "completion_tokens", "cached_tokens"))
        out.append(
            f"| {r['cell']} | {', '.join(r['served'])} | {r['turns']} (ошибок {r['errors']}) | "
            f"{r['delivered_first']}/{exp} ({100 * r['delivered_first'] / exp:.0f} %) | "
            f"{r['delivered']}/{exp} ({100 * r['delivered'] / exp:.0f} %) | {r['invalid_first'] or 0} | "
            f"{r['invalid_final'] or 0} | {r['retried']} / {r['fixed']} | {r['phantom']} | {r['spurious']} | "
            f"{r['no_tool']} | {r['multi_command']} | {r['finish']} | {r['think_in_content']} / {r['reasoning_content']} | "
            f"{r['llm_calls']} | {_ms(r['p50'])} / {_ms(r['p90'])} | {tok} |"
        )
    return "\n".join(out)


def rescore(raw_path: Path) -> list[dict[str, Any]]:
    """Сводка ячеек из сырого JSONL (отказы провайдера в JSONL не пишутся — их нет в пересчёте)."""
    cells: dict[str, list[dict[str, Any]]] = {}
    for line in raw_path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            row = json.loads(line)
            cells.setdefault(row["cell"], []).append(row)
    return [summarize_cell(spec, rows, Counter(), len(rows)) for spec, rows in cells.items()]


async def cells_async(args: argparse.Namespace) -> tuple[str, list[dict[str, Any]]]:
    load_env(Path(args.env) if args.env else None)
    phrases = load_reference(args.limit)
    system_text = Path(args.system_file).read_text(encoding="utf-8") if args.system_file else ""
    raw = open(args.raw, "a", encoding="utf-8") if args.raw else None
    try:
        rows = [await run_cell(s.strip(), phrases, args.runs, args.retry_429, raw, system_text)
                for s in args.cells.split(",") if s.strip()]
    finally:
        if raw:
            raw.close()
    return render_cells(rows, len(phrases), args.runs), rows


async def main_async(args: argparse.Namespace) -> str:
    load_env(Path(args.env) if args.env else None)
    phrases = load_phrases(args.limit)
    rows: list[tuple[str, str, dict[str, Any] | None]] = []
    for name in [n.strip() for n in args.providers.split(",") if n.strip()]:
        provider = build_provider(name)
        if provider is None:
            rows.append((name, args.mode, None))
            continue
        mode = select_mode(provider.capabilities) if args.mode == "auto" else args.mode
        rows.append((name, mode, await run_adapter(provider, mode, phrases, args.runs, concurrency=args.concurrency)))
    return render(rows, "rob_box_dialog.command.parse", len(phrases))


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--env", help="файл .env с ключами (по умолчанию только окружение)")
    ap.add_argument("--providers", default="deepseek,minimax,mimo")
    ap.add_argument("--concurrency", type=int, default=1)
    ap.add_argument("--mode", default="auto", choices=("auto", *MODE_ORDER))
    ap.add_argument("--limit", type=int, default=20, help="сколько фраз (0 = все)")
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--out", help="записать markdown в файл")
    ap.add_argument("--cells", help="путь моста: ячейки 'адаптер[:модель]/режим[/вариант]' через запятую")
    ap.add_argument("--retry-429", type=int, default=0, help="с --cells: повторов при 429 (паузы 10/20/30… с)")
    ap.add_argument("--system-file", help=f"с --cells: системный промпт из файла (боевой — {MASTER_PROMPT.name})")
    ap.add_argument("--raw", help="с --cells: JSONL, строка на ход (дописывается)")
    ap.add_argument("--json", help="с --cells/--rescore: сводка ячеек в JSON")
    ap.add_argument("--rescore", help="не звать LLM: сводка ячеек из JSONL прошлого прогона (--raw)")
    args = ap.parse_args(argv)
    if args.cells or args.rescore:
        if args.rescore:
            rows = rescore(Path(args.rescore))
            report = render_cells(rows, len({json.loads(x)["id"] for x in Path(args.rescore).read_text(
                encoding="utf-8").splitlines() if x.strip()}), args.runs)
        else:
            report, rows = asyncio.run(cells_async(args))
        if args.json:
            Path(args.json).write_text(json.dumps(rows, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    else:
        report = asyncio.run(main_async(args))
    print(report)
    if args.out:
        Path(args.out).write_text(report + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
