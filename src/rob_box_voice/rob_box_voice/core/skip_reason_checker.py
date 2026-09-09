"""AST checks for literal ``DialogueNode`` LLM skip-reason keys."""

from __future__ import annotations

import ast
from pathlib import Path

from rob_box_voice.core.llm_skip_reasons import LLMSkipReason


def find_literal_skip_reasons(source_path: Path) -> set[str]:
    """Return literal string keys used to index the skip counter."""

    tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=str(source_path))
    reasons: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Subscript):
            continue
        value = node.value
        if not (
            isinstance(value, ast.Attribute)
            and value.attr == "_llm_skipped_counter"
        ):
            continue
        key = node.slice
        if isinstance(key, ast.Constant) and isinstance(key.value, str):
            reasons.add(key.value)
    return reasons


def find_unknown_skip_reasons(source_path: Path) -> list[str]:
    """Return sorted literal keys absent from ``LLMSkipReason``."""

    declared = {reason.value for reason in LLMSkipReason}
    return sorted(find_literal_skip_reasons(source_path) - declared)
