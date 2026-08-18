#!/usr/bin/env python3
"""Validate literal ``_llm_skipped_counter`` keys against its enum SSoT."""

from __future__ import annotations

import argparse
import ast
from pathlib import Path
from typing import Iterable

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DIALOGUE_NODE = (
    REPO_ROOT / "src" / "rob_box_voice" / "rob_box_voice" / "dialogue_node.py"
)


def _declared_reasons() -> set[str]:
    from rob_box_voice.core.llm_skip_reasons import LLMSkipReason

    return {reason.value for reason in LLMSkipReason}


def find_literal_skip_reasons(source_path: Path) -> set[str]:
    """Return literal string keys used to index the skip counter."""

    tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=str(source_path))
    reasons: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Subscript):
            continue
        value = node.value
        if not (
            isinstance(value, ast.Attribute) and value.attr == "_llm_skipped_counter"
        ):
            continue
        key = node.slice
        if isinstance(key, ast.Constant) and isinstance(key.value, str):
            reasons.add(key.value)
    return reasons


def find_unknown_skip_reasons(source_path: Path) -> list[str]:
    """Return sorted literal keys absent from ``LLMSkipReason``."""

    return sorted(find_literal_skip_reasons(source_path) - _declared_reasons())


def _format_violations(reasons: Iterable[str]) -> str:
    return ", ".join(repr(reason) for reason in reasons)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", nargs="?", type=Path, default=DEFAULT_DIALOGUE_NODE)
    args = parser.parse_args()

    unknown = find_unknown_skip_reasons(args.source)
    if unknown:
        print(
            "Unknown _llm_skipped_counter keys: "
            f"{_format_violations(unknown)}. Add them to LLMSkipReason."
        )
        return 1

    print(f"LLM skip reasons OK: {args.source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
