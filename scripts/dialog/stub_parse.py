"""Заглушка ``rob_box_dialog.command.parse`` до слияния PR-1 (#3685) — для контрактного теста и adapter_benchmark.

Те же инварианты, что в ADR-0150 §4.1: первый валидный JSON-объект в тексте, остальное отбрасывается;
``speech`` <= 280, ``actions`` <= 3. Удалить, когда ``command.parse`` окажется в develop (потребители берут его
через ``pick_parser``, поэтому правка одна).
"""

from __future__ import annotations

import json
from typing import Any, Callable

_ALLOWED = {"speech", "ask", "actions", "perform", "when", "expression", "passed", "edit_pending"}
_PASSED = {"not_addressed", "nothing_to_say", "unclear"}


def stub_parse(text: str) -> dict[str, Any]:
    decoder = json.JSONDecoder()
    for i, ch in enumerate(text):
        if ch != "{":
            continue
        try:
            obj, _ = decoder.raw_decode(text[i:])
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict):
            return _validate(obj)
    raise ValueError(f"no JSON object in {text!r}")


def _validate(obj: dict[str, Any]) -> dict[str, Any]:
    if not set(obj) <= _ALLOWED:
        raise ValueError(f"unknown keys {set(obj) - _ALLOWED}")
    if "speech" in obj and not (isinstance(obj["speech"], str) and 0 < len(obj["speech"]) <= 280):
        raise ValueError("bad speech")
    actions = obj.get("actions", [])
    if len(actions) > 3 or not all(isinstance(a.get("tool"), str) and isinstance(a.get("args"), dict) for a in actions):
        raise ValueError("bad actions")
    if "passed" in obj and obj["passed"] not in _PASSED:
        raise ValueError("bad passed")
    return obj


def pick_parser() -> tuple[str, Callable[[str], Any]]:
    """("real", command.parse) если PR-1 уже в пакете, иначе ("stub", stub_parse)."""
    try:
        from rob_box_dialog import command  # PR-1
    except ImportError:
        return "stub", stub_parse
    return "real", command.parse
