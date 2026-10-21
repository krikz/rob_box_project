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
"""

from __future__ import annotations

import argparse
import asyncio
import os
import re
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

REPO = Path(__file__).resolve().parents[2]
for _p in (REPO, REPO / "src" / "rob_box_dialog", REPO / "src" / "rob_box_llm", REPO / "src" / "rob_box_core"):
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
    """Адаптер по имени из ``KEY_ENV``; ``None`` — нет ключа или адаптер не чатовый (причина: :func:`unavailable`)."""
    key = os.environ.get(KEY_ENV[name])
    if not key or name in UNAVAILABLE:
        return None
    from rob_box_llm.providers.deepseek import DeepSeekProvider, _OpenAICompatibleProvider
    from rob_box_llm.providers.mimo import MiMoProvider
    from rob_box_llm.providers.minimax import MiniMaxProvider

    model = ADAPTER_MODELS.get(name)
    if name.startswith("qwen"):
        return _OpenAICompatibleProvider(
            name="qwen", base_url=QWEN_BASE_URL, default_model=str(model), api_key=key, timeout=timeout
        )
    cls = {"deepseek": DeepSeekProvider, "minimax": MiniMaxProvider, "mimo": MiMoProvider}[name.split("-")[0]]
    return cls(api_key=key, timeout=timeout, **({"model": model} if model else {}))


def unavailable(name: str) -> str:
    """Почему адаптер не прогнан без обращения к сети."""
    return UNAVAILABLE.get(name) or "нет ключа"


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
    args = ap.parse_args(argv)
    report = asyncio.run(main_async(args))
    print(report)
    if args.out:
        Path(args.out).write_text(report + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
