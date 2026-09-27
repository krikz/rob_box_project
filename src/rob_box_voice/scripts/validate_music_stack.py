#!/usr/bin/env python3
"""Validate the Renardo / SuperCollider startup log and print a health report.

Supports two output modes:
  * human-readable (default) — for boot logs
  * JSON (``--json``)        — for downstream consumers (e.g. start_voice_assistant.sh,
                               rob_box_mcp_tools MusicManager) that need to know
                               whether the sclang runtime started healthy or degraded.

Exit codes:
  * 0 — music stack healthy
  * 1 — music stack degraded (fatal errors, missing critical synths, OSCdef
        not registered, or log file missing)
  * 2 — invalid invocation (bad CLI args)
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "log_path",
        nargs="?",
        default=os.environ.get("SCLANG_LOG_PATH", "/tmp/sclang.log"),
        help=(
            "Path to the sclang startup log. "
            "Defaults to env SCLANG_LOG_PATH or /tmp/sclang.log."
        ),
    )
    parser.add_argument(
        "--critical-synth",
        action="append",
        default=[],
        dest="critical_synths",
        help="Critical SynthDef that must be available",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        dest="json_output",
        help="Emit machine-readable JSON instead of the human report.",
    )
    return parser


def main() -> int:
    workspace_src = Path("/ws/src/rob_box_voice")
    if workspace_src.exists() and str(workspace_src) not in sys.path:
        sys.path.insert(0, str(workspace_src))

    from rob_box_voice.core.music_stack_validation import (
        classify_sclang_log,
        format_music_stack_report,
        load_sclang_health,
        missing_log_hint,
    )

    args = _build_parser().parse_args()
    log_path = Path(args.log_path)

    # load_sclang_health returns a MusicStackStatus that already handles a
    # missing log file (fatal_errors=("Missing sclang log file: ...",)) — so
    # we don't need a separate branch for that case here.
    status = load_sclang_health(log_path, critical_synths=args.critical_synths)

    if args.json_output:
        print(json.dumps(
            {
                "is_healthy": status.is_healthy,
                "oscdef_registered": status.oscdef_registered,
                "missing_synths": list(status.missing_synths),
                "fatal_errors": list(status.fatal_errors),
                "log_path": str(log_path),
            },
            ensure_ascii=False,
        ))
    else:
        print(format_music_stack_report(status))
        # Issue #2716: this used to fire whenever the stack was unhealthy
        # for ANY reason with no fatal sclang errors — including the common
        # case where the log file existed and was read fine, but some
        # critical SynthDef simply wasn't confirmed yet. missing_log_hint
        # only returns non-None when the file is genuinely absent from disk.
        hint = missing_log_hint(status, log_path)
        if hint:
            print(hint)

    return 0 if status.is_healthy else 1


if __name__ == "__main__":
    raise SystemExit(main())