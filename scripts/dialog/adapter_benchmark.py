#!/usr/bin/env python3
"""Замер адаптеров LLM по эталонному набору в выбранном режиме команд (ADR-0150 §4.5, A14; карточка #3686).

    python scripts/dialog/adapter_benchmark.py [--env FILE] [--providers deepseek,minimax,mimo]
        [--mode auto|structured_output|tool_choice|json_mode|json_in_text] [--limit 20] [--runs 1] [--out report.md]

Для каждого адаптера, чей ключ найден (``DEEPSEEK_API_KEY`` / ``MINIMAX_API_KEY`` / ``MIMO_API_KEY`` из окружения
или из ``--env``), гоняет фразы ``reference_set.yaml`` (кроме tier1: LLM там не зовётся) через ``FieldChannel``
и печатает markdown: ``delivered / invalid / latency_p50 / latency_p95`` по адаптеру.

Метрики (сырые, без оценки смысла):
* ``delivered`` — ответ разобран парсером в команду, где есть хоть одно из speech / actions / ask / passed;
* ``invalid``   — парсер отказал (нет JSON, неверная форма), вызов упал или команда пустая;
  ``delivered + invalid = вызовов``.
Промпт здесь минимальный и НЕ боевой: замер проверяет форму ответа в режиме, а не качество диалога.
Ключи не печатаются. Адаптер без ключа выводится строкой «не прогнано: нет ключа» — не пропускается молча.
"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

REPO = Path(__file__).resolve().parents[2]
for _p in (REPO, REPO / "src" / "rob_box_dialog", REPO / "src" / "rob_box_llm"):
    sys.path.insert(0, str(_p))

from rob_box_dialog.degrade import MODE_ORDER, select_mode  # noqa: E402
from rob_box_dialog.speech_channel import ModeReply, FieldChannel  # noqa: E402
from rob_box_llm.provider import LLMMessage, LLMProvider, LLMSettings  # noqa: E402
from scripts.dialog.stub_parse import pick_parser  # noqa: E402
from scripts.dialog.turn_metrics import percentile  # noqa: E402

KEY_ENV = {"deepseek": "DEEPSEEK_API_KEY", "minimax": "MINIMAX_API_KEY", "mimo": "MIMO_API_KEY"}
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
_DELIVERY_KEYS = ("speech", "actions", "ask", "passed")


def load_env(path: Path | None) -> None:
    """Дописать переменные из .env в окружение, не затирая уже заданные. Значения нигде не печатаются."""
    if path is None or not path.is_file():
        return
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            key, _, value = line.partition("=")
            os.environ.setdefault(key.strip(), value.strip().strip("'\""))


def build_provider(name: str) -> LLMProvider | None:
    key = os.environ.get(KEY_ENV[name])
    if not key:
        return None
    from rob_box_llm.providers.deepseek import DeepSeekProvider
    from rob_box_llm.providers.mimo import MiMoProvider
    from rob_box_llm.providers.minimax import MiniMaxProvider

    return {"deepseek": DeepSeekProvider, "minimax": MiniMaxProvider, "mimo": MiMoProvider}[name](api_key=key)


def load_phrases(limit: int) -> list[str]:
    data = yaml.safe_load((REPO / "scripts" / "dialog" / "reference_set.yaml").read_text(encoding="utf-8"))
    texts = [p["text"] for p in data["phrases"] if "tier1" not in p["expected"]]
    return texts[:limit] if limit else texts


def is_delivered(parsed: Any) -> bool:
    return isinstance(parsed, dict) and any(parsed.get(k) for k in _DELIVERY_KEYS)


async def run_adapter(
    provider: LLMProvider, mode: str, phrases: Sequence[str], runs: int, parse: Any
) -> dict[str, Any]:
    channel = FieldChannel(SCHEMA)
    req = channel.build_request(mode)  # type: ignore[arg-type]
    extra = {"response_format": req.response_format} if req.response_format else {}
    settings = LLMSettings(tool_choice=req.tool_choice, extra=extra)
    delivered = invalid = 0
    latencies: list[float] = []
    for text in phrases:
        for _ in range(runs):
            msgs = [LLMMessage(role="system", content=f"{SYSTEM} {req.instruction}"), LLMMessage(role="user", content=text)]
            started = time.perf_counter()
            try:
                resp = await provider.complete(msgs, tools=req.tools, settings=settings)
                reply = ModeReply(resp.content, tuple((tc.name, tc.arguments) for tc in resp.tool_calls))
                parsed = channel.parse(reply, mode, parse)  # type: ignore[arg-type]
            except Exception:  # noqa: BLE001 - любой отказ = invalid, причина не нужна метрике
                invalid += 1
            else:
                ok = is_delivered(parsed)
                delivered += ok
                invalid += not ok
            latencies.append((time.perf_counter() - started) * 1000)
    total = len(phrases) * runs
    return {
        "calls": total,
        "delivered": delivered,
        "invalid": invalid,
        "p50": percentile(latencies, 0.5),
        "p95": percentile(latencies, 0.95),
    }


def render(rows: Sequence[tuple[str, str, dict[str, Any] | None]], parser_kind: str, n_phrases: int) -> str:
    out = [
        f"### adapter_benchmark ({n_phrases} фраз, парсер: {parser_kind})",
        "",
        "| адаптер | режим | вызовов | delivered | invalid | latency_p50, мс | latency_p95, мс |",
        "|---|---|---|---|---|---|---|",
    ]
    for name, mode, res in rows:
        if res is None:
            out.append(f"| {name} | {mode} | — | не прогнано: нет ключа | — | — | — |")
            continue
        pct = f"{100 * res['delivered'] / res['calls']:.0f} %" if res["calls"] else "—"
        out.append(
            f"| {name} | {mode} | {res['calls']} | {res['delivered']} ({pct}) | {res['invalid']} | "
            f"{res['p50']:.0f} | {res['p95']:.0f} |"
        )
    return "\n".join(out)


async def main_async(args: argparse.Namespace) -> str:
    load_env(Path(args.env) if args.env else None)
    phrases = load_phrases(args.limit)
    parser_kind, parse = pick_parser()
    rows: list[tuple[str, str, dict[str, Any] | None]] = []
    for name in [n.strip() for n in args.providers.split(",") if n.strip()]:
        provider = build_provider(name)
        if provider is None:
            rows.append((name, args.mode, None))
            continue
        mode = select_mode(provider.capabilities) if args.mode == "auto" else args.mode
        rows.append((name, mode, await run_adapter(provider, mode, phrases, args.runs, parse)))
    return render(rows, parser_kind, len(phrases))


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--env", help="файл .env с ключами (по умолчанию только окружение)")
    ap.add_argument("--providers", default="deepseek,minimax,mimo")
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
