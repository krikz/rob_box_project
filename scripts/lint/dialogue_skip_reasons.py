#!/usr/bin/env python3
"""Validate literal ``_llm_skipped_counter`` keys against its enum SSoT."""

from __future__ import annotations

import argparse
from pathlib import Path

from rob_box_voice.core.skip_reason_checker import find_unknown_skip_reasons

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DIALOGUE_NODE = (
    REPO_ROOT / "src" / "rob_box_voice" / "rob_box_voice" / "dialogue_node.py"
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", nargs="?", type=Path, default=DEFAULT_DIALOGUE_NODE)
    args = parser.parse_args()

    unknown = find_unknown_skip_reasons(args.source)
    if unknown:
        formatted = ", ".join(repr(reason) for reason in unknown)
        print(
            "Unknown _llm_skipped_counter keys: "
            f"{formatted}. Add them to LLMSkipReason."
        )
        return 1

    print(f"LLM skip reasons OK: {args.source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
