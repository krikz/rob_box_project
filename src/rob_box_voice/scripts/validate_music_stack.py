#!/usr/bin/env python3
"""Validate the Renardo / SuperCollider startup log and print a health report."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_path", help="Path to the sclang startup log")
    parser.add_argument(
        "--critical-synth",
        action="append",
        default=[],
        dest="critical_synths",
        help="Critical SynthDef that must be available",
    )
    return parser


def main() -> int:
    workspace_src = Path("/ws/src/rob_box_voice")
    if workspace_src.exists() and str(workspace_src) not in sys.path:
        sys.path.insert(0, str(workspace_src))

    from rob_box_voice.core.music_stack_validation import classify_sclang_log, format_music_stack_report

    args = _build_parser().parse_args()
    log_path = Path(args.log_path)
    if not log_path.exists():
        print("Music stack degraded")
        print("OSCdef ready: no")
        print(f"Missing log file: {log_path}")
        return 1

    log_text = log_path.read_text(encoding="utf-8", errors="replace")
    status = classify_sclang_log(log_text, critical_synths=args.critical_synths)
    print(format_music_stack_report(status))
    return 0 if status.is_healthy else 1


if __name__ == "__main__":
    raise SystemExit(main())