#!/usr/bin/env python3
"""minimax_tool_choice_probe.py — держит ли MiniMax ``tool_choice`` (issue #3135 → ADR-0143; перемер #3698, 22.10.2026).

Сырой HTTP к OpenAI-совместимому ``https://api.minimax.io/v1/chat/completions`` — без нашего адаптера, чтобы мерить
сервер, а не код. Одна сетка: модель × ``tool_choice`` × запрос × ``--n`` повторов, один тул ``get_time`` без
аргументов:

* ``tool_choice``: ``auto`` / ``required`` / именованный ``{"type":"function","function":{"name":"get_time"}}`` /
  ``omit`` (поле не отправляется — так делают Mini-Agent и minimax-code);
* запрос: ``relevant`` («Сколько сейчас времени?» — тул уместен, так проверяет верификатор MiniMax) и ``irrelevant``
  («привет, как дела?» — тул неуместен, так мерил ADR-0143 28.09; режим команд — именно этот случай);
* thinking — как у робота (``MiniMaxProvider``): M3 ``{"type":"disabled"}``, M3.1-Flash ``reasoning_effort=low``
  (``disabled`` у неё — HTTP 400), M2.x — без поля (по доке игнорируется).

Запуск в образе ``voice-assistant`` (ключ ``MINIMAX_API_KEY`` только из окружения; не печатается, в файлы не пишется)::

    python3 minimax_tool_choice_probe.py --list-models
    python3 minimax_tool_choice_probe.py --models MiniMax-M3,MiniMax-M3.1-Flash --n 10 --raw probe.jsonl

Печатает таблицу «доля ответов с ``tool_calls``» по ячейкам; ``--raw`` — строка JSON на запрос (статус, finish_reason,
имена вызовов, начало текста, есть ли ``reasoning_content`` / ``<think>``, usage). 429 — пауза 10/20/30… с, не долбить.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
import urllib.error
import urllib.request
from collections import defaultdict
from typing import Any, Optional

BASE_URL = "https://api.minimax.io/v1"
API_KEY_ENV = "MINIMAX_API_KEY"

GET_TIME_TOOL: dict[str, Any] = {
    "type": "function",
    "function": {
        "name": "get_time",
        "description": "Вернуть текущее время робота.",
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
    },
}

#: Значение ``tool_choice`` по имени варианта; ``None`` — поле не отправляется.
VARIANTS: dict[str, Any] = {
    "auto": "auto",
    "required": "required",
    "named": {"type": "function", "function": {"name": "get_time"}},
    "omit": None,
}
PROMPTS: dict[str, str] = {"relevant": "Сколько сейчас времени?", "irrelevant": "привет, как дела?"}


def thinking_fields(model: str) -> dict[str, Any]:
    """Поля thinking тела запроса — как их шлёт робот (``apply_model_thinking`` в ``providers/minimax.py``)."""
    name = model.lower()
    if name.startswith("minimax-m3.1-flash"):
        return {"reasoning_effort": "low"}
    if name.startswith("minimax-m3"):
        return {"thinking": {"type": "disabled"}}
    return {}


def _request(method: str, path: str, api_key: str, payload: Optional[dict[str, Any]] = None) -> dict[str, Any]:
    body = json.dumps(payload).encode("utf-8") if payload is not None else None
    req = urllib.request.Request(
        f"{BASE_URL}{path}", data=body, method=method,
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {api_key}"},
    )
    t0 = time.monotonic()
    try:
        with urllib.request.urlopen(req, timeout=60) as resp:
            status, raw = resp.status, resp.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        status, raw = exc.code, exc.read().decode("utf-8", "replace")
    except (urllib.error.URLError, TimeoutError) as exc:
        return {"http_status": None, "transport_error": type(exc).__name__, "ms": round((time.monotonic() - t0) * 1000)}
    try:
        parsed: Any = json.loads(raw)
    except json.JSONDecodeError:
        parsed = {"_unparsable_raw": raw[:300]}
    return {"http_status": status, "ms": round((time.monotonic() - t0) * 1000), "body": parsed}


def _rate_limited(result: dict[str, Any]) -> bool:
    if result.get("http_status") == 429:
        return True
    base = (result.get("body") or {}).get("base_resp") if isinstance(result.get("body"), dict) else None
    return isinstance(base, dict) and "rate" in str(base.get("status_msg", "")).lower()


def summarise(result: dict[str, Any]) -> dict[str, Any]:
    """Поля, которые проба смотрит (ключа в ответе нет никогда)."""
    out: dict[str, Any] = {"http_status": result.get("http_status"), "ms": result.get("ms")}
    if "transport_error" in result:
        out["error"] = result["transport_error"]
        return out
    body = result.get("body") if isinstance(result.get("body"), dict) else {}
    base = body.get("base_resp")
    if isinstance(base, dict) and base.get("status_code") not in (0, None):
        out["error"] = f"base_resp {base.get('status_code')} {str(base.get('status_msg', ''))[:120]}"
        return out
    choices = body.get("choices")
    if not choices:
        out["error"] = f"no_choices {json.dumps(body, ensure_ascii=False)[:160]}"
        return out
    msg = choices[0].get("message") or {}
    calls = msg.get("tool_calls") or []
    content = msg.get("content") or ""
    out.update({
        "served": body.get("model"),
        "finish_reason": choices[0].get("finish_reason"),
        "tool_calls": [(c.get("function") or {}).get("name") for c in calls],
        "content_head": content[:80],
        "think_in_content": "<think>" in content,
        "reasoning_content": bool(msg.get("reasoning_content") or msg.get("reasoning_details")),
        "usage": body.get("usage"),
    })
    return out


def probe_one(model: str, variant: str, prompt: str, api_key: str, retry_429: int) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "model": model, "messages": [{"role": "user", "content": PROMPTS[prompt]}], "tools": [GET_TIME_TOOL],
        **thinking_fields(model),
    }
    if VARIANTS[variant] is not None:
        payload["tool_choice"] = VARIANTS[variant]
    waits = 0
    for attempt in range(retry_429 + 1):
        result = _request("POST", "/chat/completions", api_key, payload)
        if not _rate_limited(result) or attempt == retry_429:
            break
        time.sleep(10 * (attempt + 1))
        waits += 1
    row = summarise(result)
    row["retries_429"] = waits
    return row


def table(rows: list[dict[str, Any]]) -> str:
    cells: dict[tuple[str, str, str], list[dict[str, Any]]] = defaultdict(list)
    for r in rows:
        cells[(r["model"], r["variant"], r["prompt"])].append(r)
    out = [
        "| модель (запрошена → из ответа) | tool_choice | запрос | с tool_calls | ошибок | finish_reason | p50, мс |",
        "|---|---|---|---|---|---|---|",
    ]
    for (model, variant, prompt), group in cells.items():
        ok = [r for r in group if "error" not in r]
        hits = sum(1 for r in ok if r["tool_calls"])
        finishes: dict[str, int] = defaultdict(int)
        for r in ok:
            finishes[str(r["finish_reason"])] += 1
        served = ", ".join(sorted({str(r.get("served")) for r in ok})) or "—"
        errs = defaultdict(int)
        for r in group:
            if "error" in r:
                errs[r["error"][:60]] += 1
        p50 = statistics.median([r["ms"] for r in ok]) if ok else None
        out.append(
            f"| {model} → {served} | {variant} | {prompt} | {hits}/{len(ok)} | {len(group) - len(ok)}"
            f"{' ' + json.dumps(dict(errs), ensure_ascii=False) if errs else ''} | {dict(finishes)} | "
            f"{'—' if p50 is None else f'{p50:.0f}'} |"
        )
    return "\n".join(out)


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--models", default="MiniMax-M3")
    ap.add_argument("--variants", default=",".join(VARIANTS))
    ap.add_argument("--prompts", default=",".join(PROMPTS))
    ap.add_argument("--n", type=int, default=10, help="повторов на ячейку")
    ap.add_argument("--retry-429", type=int, default=4)
    ap.add_argument("--raw", help="JSONL: строка на запрос")
    ap.add_argument("--list-models", action="store_true", help="только GET /models")
    args = ap.parse_args(argv)
    api_key = os.environ.get(API_KEY_ENV)
    if not api_key:
        print(f"{API_KEY_ENV} не задан в окружении")
        return 1
    if args.list_models:
        res = _request("GET", "/models", api_key)
        data = (res.get("body") or {}).get("data") if isinstance(res.get("body"), dict) else None
        print(json.dumps({"http_status": res.get("http_status"),
                          "models": [m.get("id") for m in data or []]}, ensure_ascii=False))
        return 0
    rows: list[dict[str, Any]] = []
    raw = open(args.raw, "w", encoding="utf-8") if args.raw else None
    try:
        for model in [m.strip() for m in args.models.split(",") if m.strip()]:
            for variant in args.variants.split(","):
                for prompt in args.prompts.split(","):
                    for i in range(args.n):
                        row = {"model": model, "variant": variant, "prompt": prompt, "i": i,
                               **probe_one(model, variant, prompt, api_key, args.retry_429)}
                        rows.append(row)
                        if raw:
                            raw.write(json.dumps(row, ensure_ascii=False) + "\n")
                            raw.flush()
    finally:
        if raw:
            raw.close()
    print(table(rows))
    print(f"\nзапросов: {len(rows)} (+ повторов после 429: {sum(r['retries_429'] for r in rows)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
