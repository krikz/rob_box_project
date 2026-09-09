"""Regression test for issue #1586 / retro t_df4fff46.

`scripts/agent_flow/agent-flow-e2e-process.sh` calls ``whoami_add_label`` /
``whoami_remove_label`` (defined in ``hermes_github.sh``) at top level — not
inside a function.  Bash resolves function names at call time, so the
``hermes_github.sh`` source line MUST appear ABOVE every top-level call site
of a ``whoami_*`` helper, otherwise the script crashes with
``command not found`` on the first invocation.

The pre-fix layout placed ``. hermes_github.sh`` at line 1115 while the
earliest top-level ``whoami_add_label`` call was at line 908 — every cron
tick died with::

    line 908: whoami_add_label: command not found

and the entire e2e rotation froze for 3+ hours until the file was patched
on the host out-of-band.

This test pins the source-order contract so the regression cannot return:

1. every ``. hermes_github.sh`` source line in the script must appear
   ABOVE the earliest top-level ``whoami_*`` call site;
2. the script must ``bash -n`` parse cleanly;
3. sourcing the script in a subshell with a stubbed ``main()`` must define
   every ``whoami_*`` helper without error (i.e. source block actually
   runs, not just exists in the text).

Why a Python test (not pure bash): the bash source-order check is exactly
the kind of structural invariant that drifts silently across refactors;
running it under pytest gives us a single CLI entry point
(``pytest tests/unit/scripts/test_e2e_process_source_order.py``) that fits
the existing test layout and CI invocation.
"""

from __future__ import annotations

import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-e2e-process.sh"

WHOAMI_RE = re.compile(
    r"^[^#]*\bwhoami_(add_label|remove_label|close_issue|reopen_issue|"
    r"set_assignee|close_pr)\b"
)
SOURCE_RE = re.compile(r"""^\s*\.\s+"\$_LIB_DIR_HERE/hermes_github\.sh"\s*$""")


def _first_match_line(text: str, regex: re.Pattern[str]) -> int | None:
    """Return the 1-indexed line number of the first regex match, or None."""
    for i, line in enumerate(text.splitlines(), start=1):
        if regex.search(line):
            return i
    return None


class SourceOrderInvariant(unittest.TestCase):
    """The hermes_github.sh source line must precede every whoami_* call site."""

    @classmethod
    def setUpClass(cls) -> None:
        if not SCRIPT_PATH.exists():
            raise unittest.SkipTest(f"script not present at {SCRIPT_PATH}")

    def test_script_parses(self) -> None:
        """``bash -n`` must succeed — no syntax errors introduced by refactors."""
        result = subprocess.run(
            ["bash", "-n", str(SCRIPT_PATH)],
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            result.returncode,
            0,
            msg=f"bash -n failed:\nstdout={result.stdout}\nstderr={result.stderr}",
        )

    def test_hermes_github_source_precedes_whoami_calls(self) -> None:
        """``. hermes_github.sh`` must appear ABOVE the earliest whoami_* call."""
        text = SCRIPT_PATH.read_text()
        source_line = _first_match_line(text, SOURCE_RE)
        whoami_line = _first_match_line(text, WHOAMI_RE)
        self.assertIsNotNone(
            source_line,
            "no `. \"$_LIB_DIR_HERE/hermes_github.sh\"` source line found — "
            "the script must source hermes_github.sh before any whoami_* call",
        )
        self.assertIsNotNone(
            whoami_line,
            "no top-level whoami_* call site found — if the script truly has "
            "no whoami_* usage, drop the source line and the dependency "
            "together; do not silently keep a stale source",
        )
        # Narrow the Optional[int] return to int for the comparison. The
        # assertIsNotNone calls above guarantee these are not None.
        source_line_int: int = source_line  # type: ignore[assignment]
        whoami_line_int: int = whoami_line  # type: ignore[assignment]
        self.assertLess(
            source_line_int,
            whoami_line_int,
            msg=(
                f"hermes_github.sh sourced at line {source_line_int}, but "
                f"earliest top-level whoami_* call is at line {whoami_line_int}. "
                f"Bash resolves function names at call time, so the source "
                f"must appear before the call — otherwise the script crashes "
                f"with \"command not found\" (ретро t_df4fff46, issue #1586)."
            ),
        )

    def test_source_block_defines_whoami_helpers(self) -> None:
        """Sourcing the script in an isolated subshell defines whoami_* helpers.

        Skipped unless ``E2E_SOURCE_SMOKE=1`` is set: the e2e-process script
        is top-level (not wrapped in a ``main()``), so simply sourcing it
        executes the real cron flow — including ``flock``, ``gh api`` and
        credential bootstrap — which makes the test slow and network-bound.
        CI runs this assertion via the slower shell-level test
        (``scripts/agent_flow/tests/test_e2e_process_source_order.sh``);
        the Python unit suite keeps the structural invariant only.
        """
        import os
        if os.environ.get("E2E_SOURCE_SMOKE") != "1":
            self.skipTest(
                "set E2E_SOURCE_SMOKE=1 to run the network-bound smoke "
                "sourcing test (default: skipped in unit runs)"
            )

        if not shutil.which("bash"):
            self.skipTest("bash not available")

        with tempfile.TemporaryDirectory() as tmp:
            stub = pathlib.Path(tmp) / "smoke.sh"
            stub.write_text(
                "#!/usr/bin/env bash\n"
                "set +e\n"
                # Stub main flow so the script does not run real cron logic.
                'main() { return 0; }\n'
                "export -f main\n"
                # Required env so the script does not abort on early bootstrap.
                'GH_REPO="${GH_REPO:-krikz/rob_box_project}"\n'
                "export GH_REPO\n"
                f'source "{SCRIPT_PATH}"\n'
                "for fn in whoami_add_label whoami_remove_label "
                "whoami_close_issue whoami_reopen_issue whoami_set_assignee "
                "whoami_close_pr; do\n"
                "  if declare -F \"$fn\" >/dev/null 2>&1; then\n"
                "    echo \"DEFINED $fn\"\n"
                "  else\n"
                "    echo \"MISSING $fn\"\n"
                "    exit 1\n"
                "  fi\n"
                "done\n"
            )
            result = subprocess.run(
                ["bash", str(stub)],
                capture_output=True,
                text=True,
                timeout=15,
            )
            self.assertEqual(
                result.returncode,
                0,
                msg=(
                    "whoami_* helpers not defined after sourcing script. "
                    f"stdout={result.stdout!r} stderr={result.stderr!r}"
                ),
            )
            for fn in (
                "whoami_add_label",
                "whoami_remove_label",
                "whoami_close_issue",
                "whoami_reopen_issue",
                "whoami_set_assignee",
                "whoami_close_pr",
            ):
                self.assertIn(f"DEFINED {fn}", result.stdout, msg=fn)


if __name__ == "__main__":
    unittest.main()
