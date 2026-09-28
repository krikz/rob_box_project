#!/usr/bin/env python3
"""minimax_tool_choice_probe.py — issue #3135 (Ш3 of the music/DJ audit).

Behavioural probe: does MiniMax's OpenAI-compatible Chat Completions API
(``https://api.minimax.io/v1``, model ``MiniMax-M3`` — same endpoint/model
rob_box_voice uses, see
``docker/vision/config/voice_assistant/dialogue_node.yaml``) actually
honour ``tool_choice="required"`` and the named-function form
``{"type": "function", "function": {"name": "<tool>"}}``, or does it silently
degrade to plain text the way MiMo did (only ``auto`` supported as of
06.2026, see ``03.2-CONTEXT.md`` on ``feature/phase-3.2-music-testing``)?

Run INSIDE the ``voice-assistant`` container on the robot, where
``MINIMAX_API_KEY`` already lives as an env var:

    docker exec voice-assistant python3 /tmp/minimax_tool_choice_probe.py

The key is read from the environment ONLY. This script never prints,
logs, or returns the key value — only pass/fail per request plus the raw
JSON response body (which never contains the key).

For each of the three ``tool_choice`` variants (auto / required / named
function) it sends 3 requests with an innocuous prompt ("привет, как
дела?") that a model NOT forced to call a tool would normally answer with
plain text, plus a single harmless tool (``get_time``, no args). It
records for each request:

  * HTTP status
  * whether the response contains ``tool_calls``
  * ``finish_reason``
  * any API-level error (HTTP error body or MiniMax's in-body
    ``base_resp`` error envelope)

Output is a raw JSON report on stdout — paste directly into the issue,
no summarising required.
"""

from __future__ import annotations

import json
import os
import sys
import time
import urllib.error
import urllib.request
from typing import Any

BASE_URL = "https://api.minimax.io/v1"
MODEL = "MiniMax-M3"
API_KEY_ENV = "MINIMAX_API_KEY"
REQUESTS_PER_VARIANT = 3
PROMPT = "привет, как дела?"

GET_TIME_TOOL: dict[str, Any] = {
    "type": "function",
    "function": {
        "name": "get_time",
        "description": "Вернуть текущее время робота.",
        "parameters": {
            "type": "object",
            "properties": {},
            "additionalProperties": False,
        },
    },
}

VARIANTS: dict[str, Any] = {
    "auto": "auto",
    "required": "required",
    "named_get_time": {"type": "function", "function": {"name": "get_time"}},
}


def _post(payload: dict[str, Any], api_key: str) -> dict[str, Any]:
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        f"{BASE_URL}/chat/completions",
        data=body,
        method="POST",
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}",
        },
    )
    t0 = time.monotonic()
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            status = resp.status
            raw = resp.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        status = exc.code
        raw = exc.read().decode("utf-8")
    except urllib.error.URLError as exc:
        return {
            "http_status": None,
            "transport_error": str(exc),
            "elapsed_s": round(time.monotonic() - t0, 3),
        }
    elapsed = round(time.monotonic() - t0, 3)
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError:
        parsed = {"_unparsable_raw": raw[:2000]}
    return {"http_status": status, "elapsed_s": elapsed, "body": parsed}


def _summarise(result: dict[str, Any]) -> dict[str, Any]:
    """Pull out the fields the probe actually cares about (no key, ever)."""
    out: dict[str, Any] = {
        "http_status": result.get("http_status"),
        "elapsed_s": result.get("elapsed_s"),
    }
    if "transport_error" in result:
        out["transport_error"] = result["transport_error"]
        return out
    body = result.get("body", {})
    base_resp = body.get("base_resp") if isinstance(body, dict) else None
    if base_resp and base_resp.get("status_code", 0) not in (0, None):
        out["base_resp_error"] = base_resp
        return out
    choices = body.get("choices") if isinstance(body, dict) else None
    if not choices:
        out["no_choices"] = True
        out["raw_body"] = body
        return out
    msg = choices[0].get("message", {})
    out["finish_reason"] = choices[0].get("finish_reason")
    tool_calls = msg.get("tool_calls")
    out["has_tool_calls"] = bool(tool_calls)
    if tool_calls:
        out["tool_call_names"] = [
            tc.get("function", {}).get("name") for tc in tool_calls
        ]
    out["content_snippet"] = (msg.get("content") or "")[:200]
    return out


def main() -> int:
    api_key = os.environ.get(API_KEY_ENV)
    if not api_key:
        print(
            json.dumps(
                {"error": f"{API_KEY_ENV} not set in environment"},
                ensure_ascii=False,
            )
        )
        return 1

    report: dict[str, Any] = {
        "base_url": BASE_URL,
        "model": MODEL,
        "prompt": PROMPT,
        "requests_per_variant": REQUESTS_PER_VARIANT,
        "variants": {},
    }

    for variant_name, tool_choice_value in VARIANTS.items():
        runs = []
        for i in range(REQUESTS_PER_VARIANT):
            payload = {
                "model": MODEL,
                "messages": [{"role": "user", "content": PROMPT}],
                "tools": [GET_TIME_TOOL],
                "tool_choice": tool_choice_value,
                # Thinking disabled to match the production voice path
                # (DEFAULT_THINKING_POLICY in
                # src/rob_box_llm/rob_box_llm/providers/minimax.py) and to
                # avoid thinking-vs-tool-call interaction confounding the
                # probe (see docs/design/.../phase-3.2 lessons-learned).
                "thinking": {"type": "disabled"},
            }
            raw = _post(payload, api_key)
            runs.append(_summarise(raw))
        report["variants"][variant_name] = {
            "tool_choice_sent": tool_choice_value,
            "runs": runs,
        }

    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
