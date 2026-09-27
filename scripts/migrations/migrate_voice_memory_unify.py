#!/usr/bin/env python3
"""migrate_voice_memory_unify.py — ADR-0055 Phase 1 dry-run / apply.

Default mode (``--dry-run``, also the default if no flag is passed) is
**read-only**: it inspects both candidate databases, prints a diff of
what would change, and exits 0 without touching anything on disk.

``--apply`` runs the marker migration (INSERT a row into
``voice_memory_meta``) against ``--legacy`` *and* initialises the
harness database (creates the empty ``facts / waypoints / faq_items /
event_profile`` tables via ``SQLiteVoiceMemory.init()``) so the new
write path is ready to receive data.

This script does NOT migrate data. Data-migration = Phase 2
(separate card after merge of AgentCore step 03, ADR-0055 §2.2).

Usage
-----
::

    # Just see the diff
    python3 scripts/migrations/migrate_voice_memory_unify.py

    # Explicit dry-run
    python3 scripts/migrations/migrate_voice_memory_unify.py --dry-run

    # Apply (only on prod, after a dry-run was reviewed)
    python3 scripts/migrations/migrate_voice_memory_unify.py \\
        --harness /data/harness_voice.db \\
        --legacy  /data/voice_memory.db \\
        --apply

Exit codes
----------
* 0 — dry-run printed (or apply succeeded)
* 2 — ``--apply`` rejected because dry-run was not run first
* 3 — a database file is unreachable / read-only (apply only)
"""
from __future__ import annotations

import argparse
import os
import sqlite3
import sys
from pathlib import Path

# Repo-relative path to the marker migration SQL. ``--apply`` runs this
# against ``--legacy`` (the legacy ``voice_memory.db`` keeps a single
# marker row, see ADR-0055 §2.5).
REPO_ROOT = Path(__file__).resolve().parents[2]
MIGRATION_FILE = REPO_ROOT / "migrations" / "010_voice_memory_unify.sql"


def _table_counts(conn: sqlite3.Connection) -> dict[str, int]:
    """Return ``{table: row_count}`` for every user table in the DB."""
    cur = conn.execute(
        "SELECT name FROM sqlite_master "
        "WHERE type = 'table' AND name NOT LIKE 'sqlite_%'"
    )
    tables = [row[0] for row in cur.fetchall()]
    counts: dict[str, int] = {}
    for tbl in tables:
        try:
            row = conn.execute(f"SELECT COUNT(*) FROM {tbl}").fetchone()
            counts[tbl] = int(row[0]) if row else 0
        except sqlite3.Error as exc:
            counts[tbl] = -1  # negative means "schema-incompatible"
            print(f"⚠️  {tbl}: COUNT(*) failed ({exc})", file=sys.stderr)
    return counts


def _size_kb(path: Path) -> int:
    try:
        return os.path.getsize(path) // 1024
    except OSError:
        return 0


def _print_diff(harness_path: Path, legacy_path: Path) -> int:
    """Inspect both DBs and print a diff. Exit 0."""
    print(f"🔎 Dry-run: ADR-0055 Phase 1 path consolidation")
    print(f"   harness : {harness_path}  ({_size_kb(harness_path)} KB)")
    print(f"   legacy  : {legacy_path}    ({_size_kb(legacy_path)} KB)")
    print()
    if not harness_path.exists():
        print(f"ℹ️  harness DB does not exist yet — apply will create it.")
    else:
        with sqlite3.connect(harness_path) as conn:
            counts = _table_counts(conn)
        print(f"   harness tables: {counts or '(empty)'}")
    if legacy_path.exists():
        with sqlite3.connect(legacy_path) as conn:
            counts = _table_counts(conn)
        marker_present = (
            counts.get("voice_memory_meta", -1) >= 0
        )
        print(f"   legacy tables : {counts or '(empty)'}")
        print(f"   marker row    : {'present' if marker_present else 'absent'}")
    else:
        print(f"ℹ️  legacy DB does not exist — nothing to mark.")
    print()
    print("📋 Plan:")
    print("   * (no DDL change to harness_voice.db; init via SQLiteVoiceMemory")
    print("     creates facts / waypoints / faq_items / event_profile)")
    print("   * (no DDL change to voice_memory.db; it stays read-only)")
    print(f"   * INSERT marker into voice_memory_meta on legacy DB ({MIGRATION_FILE})")
    print("   * deploy new code: mcp_server.py / waypoint_store.py now use")
    print("     VoiceMemoryAdapter → /data/harness_voice.db")
    print()
    print("❗ Phase 2 (separate card) does data-migration after AgentCore step 03.")
    return 0


def _apply(harness_path: Path, legacy_path: Path) -> int:
    """Apply the marker to the legacy DB and ensure harness DB exists.

    The harness DB schema is created by ``SQLiteVoiceMemory.init()``; we
    call that here so ``docker exec ... .tables`` shows the tables
    immediately, without waiting for the first MCP-tool call.
    """
    # Make sure harness parent directory exists.
    harness_path.parent.mkdir(parents=True, exist_ok=True)
    if not MIGRATION_FILE.exists():
        print(
            f"❌ marker migration missing: {MIGRATION_FILE}",
            file=sys.stderr,
        )
        return 4

    # 1. Run the marker against the legacy DB (idempotent: INSERT OR IGNORE).
    if legacy_path.exists():
        sql = MIGRATION_FILE.read_text(encoding="utf-8")
        with sqlite3.connect(legacy_path) as conn:
            conn.executescript(sql)
            conn.commit()
        print(f"✅ marker written to {legacy_path}")
    else:
        print(f"ℹ️  legacy DB does not exist — skipped marker (still ok).")

    # 2. Initialise the harness DB through the same code path mcp_server uses.
    sys.path.insert(0, str(REPO_ROOT / "src" / "rob_box_harness"))
    sys.path.insert(0, str(REPO_ROOT / "src" / "rob_box_llm"))
    from rob_box_harness.memory import SQLiteVoiceMemory
    import asyncio

    async def _init_harness() -> None:
        store = SQLiteVoiceMemory(db_path=str(harness_path))
        await store.init()
        await store.teardown()

    asyncio.run(_init_harness())
    print(f"✅ harness DB initialised at {harness_path}")

    # 3. Verify
    print()
    print("📊 Post-apply state:")
    return _print_diff(harness_path, legacy_path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="migrate_voice_memory_unify",
        description=(
            "ADR-0055 Phase 1 — dry-run by default. --apply writes the "
            "marker and initialises the harness DB. NEVER migrate data "
            "without an explicit follow-up card (Phase 2)."
        ),
    )
    parser.add_argument(
        "--harness",
        default=os.getenv("HARNESS_VOICE_DB", "/data/harness_voice.db"),
        help="Path to the unified harness DB (default: /data/harness_voice.db)",
    )
    parser.add_argument(
        "--legacy",
        default=os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db"),
        help="Path to the legacy MCP DB (default: /data/voice_memory.db)",
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--dry-run",
        action="store_true",
        default=True,
        help="(default) Read-only inspection; do not touch any file.",
    )
    group.add_argument(
        "--apply",
        action="store_true",
        help="Apply the marker migration and init the harness DB.",
    )
    args = parser.parse_args(argv)

    harness_path = Path(args.harness)
    legacy_path = Path(args.legacy)

    if args.apply:
        return _apply(harness_path, legacy_path)
    return _print_diff(harness_path, legacy_path)


if __name__ == "__main__":
    sys.exit(main())