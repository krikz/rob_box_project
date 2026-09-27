#!/usr/bin/env bash
# ============================================================================
# test_migrate_voice_memory_unify.sh — регресс-тест миграционного скрипта
# scripts/migrations/migrate_voice_memory_unify.py (ADR-0055 §6.1).
#
# Что покрываем:
#   1) default-режим (--dry-run) на чистой директории — exit 0, ничего не
#      создаётся, ничего не пишется.
#   2) --apply с пустой legacy-БД — init harness_voice.db (DDL появляется),
#      marker в voice_memory_meta присутствует, idempotent.
#   3) повторный --apply — exit 0, размер обеих БД не растёт
#      (INSERT OR IGNORE действительно no-op), DDL не дублируется.
#   4) exit codes: dry-run = 0; apply после apply = 0; dry-run после
#      apply = 0; dry-run не меняет размер БД.
#
# Зависимости: python3 (sqlite3 модуль в stdlib). НЕ требуется CLI
# sqlite3 — все инспекции через python3 -c. Это переносимо на CI builder
# без apt-установки.
#
# Стратегия: фикстуры в mktemp -d, никаких правок репо, без сети.
#
# Run:
#   bash scripts/agent_flow/tests/test_migrate_voice_memory_unify.sh
# ============================================================================

set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$TEST_DIR/../../.." && pwd)"
SCRIPT="$ROOT_DIR/scripts/migrations/migrate_voice_memory_unify.py"

if [ ! -f "$SCRIPT" ]; then
    echo "FAIL: $SCRIPT not found" >&2
    exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
    echo "FAIL: python3 not available" >&2
    exit 1
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

pass_count=0
fail_count=0

# --- helpers ----------------------------------------------------------------

# assert_eq <name> <expected> <actual>
assert_eq() {
    local name="$1" expected="$2" actual="$3"
    if [ "$expected" = "$actual" ]; then
        echo "  PASS: $name (=$actual)"
        pass_count=$((pass_count + 1))
    else
        echo "  FAIL: $name — expected '$expected', got '$actual'" >&2
        fail_count=$((fail_count + 1))
    fi
}

# assert_le <name> <max> <actual>
assert_le() {
    local name="$1" max="$2" actual="$3"
    if [ "$actual" -le "$max" ]; then
        echo "  PASS: $name ($actual ≤ $max)"
        pass_count=$((pass_count + 1))
    else
        echo "  FAIL: $name — $actual > $max" >&2
        fail_count=$((fail_count + 1))
    fi
}

# has_table <db_path> <table>  → echoes 1 or 0
has_table() {
    python3 - "$1" "$2" <<'PY'
import sqlite3, sys
db, table = sys.argv[1], sys.argv[2]
try:
    with sqlite3.connect(db) as c:
        cur = c.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?",
            (table,),
        )
        print(1 if cur.fetchone() else 0)
except sqlite3.Error:
    print(0)
PY
}

# marker_count <db_path>  → echoes the count of migration_010_applied_at
# rows in <db>.voice_memory_meta (creates the table if missing so the
# apply step can write to it later in test 2).
marker_count() {
    python3 - "$1" <<'PY'
import sqlite3, sys
db = sys.argv[1]
with sqlite3.connect(db) as c:
    c.execute("CREATE TABLE IF NOT EXISTS voice_memory_meta (key TEXT PRIMARY KEY, value TEXT);")
    cur = c.execute(
        "SELECT COUNT(*) FROM voice_memory_meta WHERE key='migration_010_applied_at'"
    )
    print(cur.fetchone()[0])
PY
}

# pre_create_legacy_marker_table <db_path>  — make sure voice_memory_meta
# exists so the apply step has somewhere to write the marker. Safe no-op
# if the file does not exist yet.
pre_create_legacy_marker_table() {
    python3 - "$1" <<'PY'
import sqlite3, sys, os
db = sys.argv[1]
parent = os.path.dirname(db)
if parent:
    os.makedirs(parent, exist_ok=True)
if not os.path.exists(db):
    with sqlite3.connect(db) as c:
        c.execute(
            "CREATE TABLE voice_memory_meta (key TEXT PRIMARY KEY, value TEXT);"
        )
PY
}

# file_size <path>  → echoes the file size in bytes (0 if missing)
file_size() {
    if [ -f "$1" ]; then
        stat -c%s "$1"
    else
        echo 0
    fi
}

# ---- test 1: dry-run is read-only -----------------------------------------
echo "[1/4] dry-run is read-only on empty directory"
HARNESS_DB="$WORK/dry-harness.db"
LEGACY_DB="$WORK/dry-legacy.db"
python3 "$SCRIPT" \
    --harness "$HARNESS_DB" \
    --legacy  "$LEGACY_DB" \
    > "$WORK/dry.stdout" 2> "$WORK/dry.stderr"
dry_rc=$?
assert_eq "dry-run exit code" "0" "$dry_rc"
assert_eq "harness DB not created" "0" "$(file_size "$HARNESS_DB")"
assert_eq "legacy DB not created"  "0" "$(file_size "$LEGACY_DB")"
if grep -q "🔎 Dry-run" "$WORK/dry.stdout"; then
    echo "  PASS: dry-run prints banner"
    pass_count=$((pass_count + 1))
else
    echo "  FAIL: dry-run missing banner" >&2
    fail_count=$((fail_count + 1))
fi

# ---- test 2: --apply creates harness schema + writes marker --------------
echo
echo "[2/4] --apply initialises harness DB and writes marker"
HARNESS_DB="$WORK/harness_voice.db"
LEGACY_DB="$WORK/voice_memory.db"
pre_create_legacy_marker_table "$LEGACY_DB"
apply_rc=$(python3 "$SCRIPT" \
    --harness "$HARNESS_DB" \
    --legacy  "$LEGACY_DB" \
    --apply > "$WORK/apply.stdout" 2> "$WORK/apply.stderr"; echo $?)
assert_eq "apply exit code" "0" "$apply_rc"
size_after_apply=$(file_size "$HARNESS_DB")
if [ "$size_after_apply" -gt 0 ]; then
    echo "  PASS: harness DB created (size=$size_after_apply)"
    pass_count=$((pass_count + 1))
else
    echo "  FAIL: harness DB not created" >&2
    fail_count=$((fail_count + 1))
fi
for t in facts waypoints faq_items event_profile; do
    if [ "$(has_table "$HARNESS_DB" "$t")" = "1" ]; then
        echo "  PASS: harness DB has table '$t'"
        pass_count=$((pass_count + 1))
    else
        echo "  FAIL: harness DB missing table '$t'" >&2
        fail_count=$((fail_count + 1))
    fi
done
marker_after=$(marker_count "$LEGACY_DB")
assert_eq "marker row count after apply" "1" "$marker_after"

# ---- test 3: idempotency — re-apply must not grow the DBs ----------------
echo
echo "[3/4] --apply is idempotent"
harness_size_before=$(file_size "$HARNESS_DB")
legacy_size_before=$(file_size "$LEGACY_DB")
apply2_rc=$(python3 "$SCRIPT" \
    --harness "$HARNESS_DB" \
    --legacy  "$LEGACY_DB" \
    --apply > "$WORK/apply2.stdout" 2> "$WORK/apply2.stderr"; echo $?)
assert_eq "second apply exit code" "0" "$apply2_rc"
marker_after2=$(marker_count "$LEGACY_DB")
assert_eq "marker row count after second apply (still 1)" "1" "$marker_after2"
# Re-apply after the second one — three total. Marker must stay 1,
# schemas unchanged, sizes should plateau.
apply3_rc=$(python3 "$SCRIPT" \
    --harness "$HARNESS_DB" \
    --legacy  "$LEGACY_DB" \
    --apply > "$WORK/apply3.stdout" 2> "$WORK/apply3.stderr"; echo $?)
assert_eq "third apply exit code" "0" "$apply3_rc"
marker_after3=$(marker_count "$LEGACY_DB")
assert_eq "marker row count after third apply (still 1)" "1" "$marker_after3"
harness_size_after=$(file_size "$HARNESS_DB")
legacy_size_after=$(file_size "$LEGACY_DB")
harness_growth=$((harness_size_after - harness_size_before))
legacy_growth=$((legacy_size_after - legacy_size_before))
# Allow modest growth for WAL bookkeeping; marker is upserted so legacy
# should be essentially flat. Harness DB schema is stable.
assert_le "harness DB growth (≤ 4 KB)" 4096 "$harness_growth"
assert_le "legacy DB growth (≤ 1 KB)" 1024 "$legacy_growth"

# ---- test 4: dry-run after apply is non-destructive ----------------------
echo
echo "[4/4] dry-run after apply stays read-only"
HARNESS_BEFORE=$(file_size "$HARNESS_DB")
LEGACY_BEFORE=$(file_size "$LEGACY_DB")
dry2_rc=$(python3 "$SCRIPT" \
    --harness "$HARNESS_DB" \
    --legacy  "$LEGACY_DB" > "$WORK/dry2.stdout" 2> "$WORK/dry2.stderr"; echo $?)
assert_eq "dry-run after apply exit code" "0" "$dry2_rc"
HARNESS_AFTER=$(file_size "$HARNESS_DB")
LEGACY_AFTER=$(file_size "$LEGACY_DB")
assert_eq "harness DB size unchanged after dry-run" "$HARNESS_BEFORE" "$HARNESS_AFTER"
assert_eq "legacy DB size unchanged after dry-run"  "$LEGACY_BEFORE"  "$LEGACY_AFTER"
if grep -q "marker row    : present" "$WORK/dry2.stdout"; then
    echo "  PASS: dry-run reports marker as present after apply"
    pass_count=$((pass_count + 1))
else
    echo "  FAIL: dry-run did not report marker status" >&2
    fail_count=$((fail_count + 1))
fi

# ---- summary -------------------------------------------------------------
echo
echo "=============================================="
echo "PASS: $pass_count    FAIL: $fail_count"
echo "=============================================="
if [ "$fail_count" -gt 0 ]; then
    echo "DUMP apply.stdout:" >&2
    cat "$WORK/apply.stdout" >&2 || true
    echo "DUMP apply.stderr:" >&2
    cat "$WORK/apply.stderr" >&2 || true
    echo "DUMP apply2.stderr:" >&2
    cat "$WORK/apply2.stderr" >&2 || true
    echo "DUMP apply3.stderr:" >&2
    cat "$WORK/apply3.stderr" >&2 || true
    echo "DUMP dry.stdout:" >&2
    cat "$WORK/dry.stdout" >&2 || true
    echo "DUMP dry2.stdout:" >&2
    cat "$WORK/dry2.stdout" >&2 || true
    exit 1
fi
exit 0
