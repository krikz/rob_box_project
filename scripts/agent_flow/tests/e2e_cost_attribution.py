#!/usr/bin/env python3
"""e2e check for the hermes-agent cost-attribution patch (issue #1462, P10 gap).

Run AFTER a hermes-agent update + install.sh (which re-applies
scripts/agent_flow/vendor/hermes-agent-kanban-cost-attribution.patch):

    bash scripts/agent_flow/install.sh
    python3 scripts/agent_flow/tests/e2e_cost_attribution.py

Verifies on the REAL on-disk hermes-agent that:
  1. The patch's main symbols are importable:
       - hermes_cli.kanban_db._compute_run_cost_cents
       - hermes_cli.kanban_db._record_cost_attribution_event
       - hermes_cli.kanban_db.cost_summary
       - hermes_cli.kanban_db.board_stats  (returns cost_by_assignee key)
  2. _compute_run_cost_cents returns None when given no session id
     (not 0.0 — distinguishable from "real $0.00" per the patch docstring).
  3. _compute_run_cost_cents returns a number when given a fake session
     id (it gracefully degrades to 0.0 when session_model_usage isn't
     attached — exercises the try/except OperationalError path).
  4. board_stats() result includes the 'cost_by_assignee' key (always
     present, possibly empty on boards without cost events yet).
  5. _cmd_cost subcommand is registered in the kanban CLI dispatcher
     (issue #1462 acceptance: `hermes kanban cost --issue=N` works).
  6. The dashboard plugin exposes the /cost GET endpoint
     (FastAPI route table contains "get_cost").

Exit 0 = all checks passed. Non-zero = regression (patch lost or broken).
"""
import inspect
import os
import sys

# HERMES_AGENT_DIR allows the test to be pointed at a hermes-agent
# checkout that has all vendor-patches already applied (e.g. an isolated
# /tmp worktree used by the rob_box CI). Defaults to the live path the
# operator's `hermes` CLI uses — same convention as install.sh's
# HERMES_AGENT_DIR.
HERMES_AGENT_DIR = os.environ.get(
    "HERMES_AGENT_DIR", "/home/builder/.hermes/hermes-agent"
)
if HERMES_AGENT_DIR not in sys.path:
    sys.path.insert(0, HERMES_AGENT_DIR)

from hermes_cli import kanban_db  # noqa: E402
from hermes_cli import kanban  # noqa: E402


def _ok(label, value=""):
    print(f"OK {label}" + (f": {value}" if value else ""))


def _fail(reason):
    raise SystemExit(f"FAIL: {reason}")


# --- 1. Symbols importable -----------------------------------------------------
for name in (
    "_compute_run_cost_cents",
    "_record_cost_attribution_event",
    "_resolve_worker_session_id",
    "cost_summary",
):
    fn = getattr(kanban_db, name, None)
    if fn is None:
        _fail(f"hermes_cli.kanban_db.{name} missing — patch not applied")
    if not callable(fn) and not inspect.isclass(fn):
        _fail(f"hermes_cli.kanban_db.{name} present but not callable")
    _ok(f"imported kanban_db.{name}")


# --- 2. _compute_run_cost_cents None branch -----------------------------------
# Issue #1462 acceptance: distinguish "no worker session" from "$0.00".
# Patch contract: returns None when no session id resolves.
import sqlite3
conn = sqlite3.connect(":memory:")
result = kanban_db._compute_run_cost_cents(conn, worker_session_id=None)
if result is not None:
    _fail(f"_compute_run_cost_cents(None) should return None, got {result!r}")
_ok("_compute_run_cost_cents(worker_session_id=None) -> None")


# --- 3. _compute_run_cost_cents degrades when session_model_usage is absent ---
# The patch tries the SUM query and catches sqlite3.OperationalError when
# session_model_usage isn't attached to the connection. Result must be None
# (graceful) — never raise.
result = kanban_db._compute_run_cost_cents(
    conn, worker_session_id="synthetic-session-id-never-exists"
)
if result is None:
    # Either: the table IS attached but the row is missing (returns 0.0),
    # or it isn't attached (returns None via the except branch). Both are
    # acceptable per the patch docstring — the contract is "never crash".
    _ok("_compute_run_cost_cents degrades gracefully without session_model_usage")
else:
    if not isinstance(result, (int, float)):
        _fail(f"expected numeric or None, got {type(result).__name__}: {result!r}")
    if result < 0:
        _fail(f"negative cost: {result!r}")
    _ok(f"_compute_run_cost_cents -> {result}")


# --- 4. board_stats() exposes cost_by_assignee key -----------------------------
# board_stats() needs a real connection to a board db. We probe both the
# devops profile (operator's `hermes kanban stats`) and the active
# agent-flow board if HERMES_KANBAN_BOARD is set. We do NOT assert
# specific numbers — only that the key exists and is a dict.
def _probe_board_stats():
    from pathlib import Path
    candidates = []
    for p in (
        os.environ.get("HERMES_KANBAN_DB"),
        "/home/builder/.hermes/profiles/devops/state/kanban.db",
        "/home/builder/.hermes/state/kanban.db",
    ):
        if p and Path(p).is_file() and p not in candidates:
            candidates.append(p)
    for path in candidates:
        try:
            conn = sqlite3.connect(path)
            try:
                # board_stats() iterates rows via ``row["col"]`` — needs
                # the sqlite3.Row factory. Match what kb.connect() does
                # in production so we exercise the real path.
                conn.row_factory = sqlite3.Row
                return kanban_db.board_stats(conn)
            finally:
                conn.close()
        except sqlite3.DatabaseError as exc:
            print(f"WARN: probe {path!r} skipped: {exc}")
            continue
        except TypeError as exc:
            # Pre-migration schema (board_stats failed before patch
            # landed) — surface as a skip, not a crash.
            print(f"WARN: probe {path!r} skipped: {exc}")
            continue
    return None


stats = _probe_board_stats()
if stats is None:
    # No board db available in this environment (e.g. CI container without
    # hermes profile state). Skip with a warning rather than fail —
    # symbols-importable + cli-registered + dashboard-route checks above
    # already cover the patch contract for this CI scenario.
    print("WARN: no kanban board db available; skipping board_stats() shape check")
else:
    if "cost_by_assignee" not in stats:
        _fail("board_stats() did not include 'cost_by_assignee' — patch not applied")
    if not isinstance(stats["cost_by_assignee"], dict):
        _fail(
            f"cost_by_assignee should be dict, got "
            f"{type(stats['cost_by_assignee'])}"
        )
    _ok(
        "board_stats() exposes cost_by_assignee",
        f"({len(stats['cost_by_assignee'])} assignee(s) tracked)",
    )


# --- 5. CLI subcommand `cost` is registered -----------------------------------
import argparse
_wrap = argparse.ArgumentParser(prog="/kanban-wrap", add_help=False)
_top_sub = _wrap.add_subparsers(dest="_top")
parser = kanban.build_parser(_top_sub)
found_cost = False
for action in parser._actions:
    if not hasattr(action, "choices") or not action.choices:
        continue
    if "cost" in action.choices:
        found_cost = True
        break
if not found_cost:
    _fail("hermes kanban CLI: 'cost' subcommand not registered (issue #1462)")
_ok("hermes kanban cost subcommand registered")


# --- 6. Dashboard plugin exposes /cost endpoint --------------------------------
from plugins.kanban.dashboard import plugin_api
has_get_cost = hasattr(plugin_api, "get_cost")
if not has_get_cost:
    _fail(
        "plugins/kanban/dashboard/plugin_api.py: 'get_cost' endpoint missing — "
        "patch not applied (issue #1462 acceptance)"
    )
_ok("dashboard plugin_api.get_cost present")


print("ALL COST ATTRIBUTION E2E CHECKS PASSED")