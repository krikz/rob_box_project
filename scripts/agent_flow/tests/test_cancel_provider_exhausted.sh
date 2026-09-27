#!/bin/bash
# ============================================================================
# test_cancel_provider_exhausted.sh — ретро 15.09 t_197de62a
#
# Регресс-гард для agent-flow-cancel-on-provider-exhausted.sh:
#   1. карточка с provider-marker в task_runs.summary → cancel-mode ловит
#   2. карточка с provider-marker ТОЛЬКО в tasks.last_failure_error →
#      cancel-mode ловит (signal=lfe)
#   3. карточка с provider-marker в ОБОИХ полях → cancel-mode ловит (signal=both)
#   4. здоровая карточка (status=running, чистые summary+lfe) → НЕТ cancel
#   5. карточка с sentinel-marker'ом → НЕТ cancel (idempotency)
#   6. карточка в статусе blocked (kind=capability) → НЕТ cancel
#   7. карточка с summary 'провайдер восстановлен' → НЕТ cancel (anti-pattern)
#   8. cancel → после повторного dry-run → 0 кандидатов (sentinel idempotency)
#
# Run:
#   bash scripts/agent_flow/tests/test_cancel_provider_exhausted.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_UNDER_TEST="${SCRIPT_UNDER_TEST:-$TEST_DIR/../agent-flow-cancel-on-provider-exhausted.sh}"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

# Sanity check
[ -x "$SCRIPT_UNDER_TEST" ] || fail "script not found: $SCRIPT_UNDER_TEST"

# --- фикстура: kanban.db + task_runs + task_comments -------------------------
BOARD_DIR="$WORK/boards/robbox"
mkdir -p "$BOARD_DIR"
python3 - "$BOARD_DIR" <<'PYEOF'
import sqlite3, sys, os, time
board_dir = sys.argv[1]
db = os.path.join(board_dir, "kanban.db")
con = sqlite3.connect(db)
con.executescript("""
CREATE TABLE tasks (
    id TEXT PRIMARY KEY,
    title TEXT DEFAULT '',
    body TEXT DEFAULT '',
    status TEXT DEFAULT 'todo',
    block_kind TEXT,
    block_recurrences INTEGER DEFAULT 0,
    consecutive_failures INTEGER DEFAULT 0,
    worker_pid INTEGER DEFAULT 0,
    last_failure_error TEXT DEFAULT '',
    created_at INTEGER DEFAULT 0,
    last_heartbeat_at INTEGER DEFAULT 0
);
CREATE TABLE task_runs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT,
    status TEXT,
    outcome TEXT,
    summary TEXT,
    error TEXT,
    started_at INTEGER,
    ended_at INTEGER
);
CREATE TABLE task_comments (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT,
    body TEXT,
    created_at INTEGER
);
""")
now = int(time.time())

# Test 1: marker в summary, status=todo → catch
con.execute("INSERT INTO tasks VALUES ('t_summary_catch','t1','body1','todo',NULL,0,0,'','',?,?)", (now, now))
con.execute("INSERT INTO task_runs (task_id,status,outcome,summary,started_at,ended_at) VALUES ('t_summary_catch','crashed','crashed','провайдер исчерпан, ждать (402/429)',?,?)", (now, now+1))

# Test 2: marker ТОЛЬКО в last_failure_error, status=running → catch (signal=lfe)
con.execute("INSERT INTO tasks VALUES ('t_lfe_catch','t2','body2','running',NULL,0,1,99999,'HTTP 429 Token Plan rate limit reached',?,?)", (now, now))
con.execute("INSERT INTO task_runs (task_id,status,outcome,summary,started_at,ended_at) VALUES ('t_lfe_catch','running','running','worker startup',?,?)", (now, now+1))

# Test 3: marker в ОБОИХ полях, status=ready → catch (signal=both)
con.execute("INSERT INTO tasks VALUES ('t_both_catch','t3','body3','ready',NULL,0,0,0,'Out of credits',?,?)", (now, now))
con.execute("INSERT INTO task_runs (task_id,status,outcome,summary,started_at,ended_at) VALUES ('t_both_catch','crashed','crashed','health-aware-fallback: all providers unavailable',?,?)", (now, now+1))

# Test 4: здоровая карточка → НЕТ cancel
con.execute("INSERT INTO tasks VALUES ('t_healthy','t4','body4','running',NULL,0,0,12345,'normal worker exit',?,?)", (now, now))
con.execute("INSERT INTO task_runs (task_id,status,outcome,summary,started_at,ended_at) VALUES ('t_healthy','running','running','work in progress',?,?)", (now, now+1))

# Test 5: marker в summary, но уже есть sentinel-marker → НЕТ cancel (idempotency)
con.execute("INSERT INTO tasks VALUES ('t_already_marked','t5','body5','todo',NULL,0,0,0,'HTTP 402: Insufficient Balance',?,?)", (now, now))
con.execute("INSERT INTO task_runs (task_id,status,outcome,summary,started_at,ended_at) VALUES ('t_already_marked','crashed','crashed','HTTP 402: Insufficient Balance',?,?)", (now, now+1))
con.execute("INSERT INTO task_comments (task_id,body,created_at) VALUES ('t_already_marked','<!-- agent-flow-cancel-on-provider-exhausted.sh:marker -->\n<!-- retro-key:foo -->\nprior block',?)", (now,))

# Test 6: blocked + block_kind=capability + marker → НЕТ cancel (его уже блокировали)
con.execute("INSERT INTO tasks VALUES ('t_blocked_already','t6','body6','blocked','capability',0,0,0,'HTTP 429',?,?)", (now, now))
con.execute("INSERT INTO task_runs (task_id,status,outcome,summary,started_at,ended_at) VALUES ('t_blocked_already','blocked','blocked','HTTP 429 rate limit',?,?)", (now, now+1))

# Test 7: anti-pattern — «провайдер восстановлен» → НЕТ cancel
con.execute("INSERT INTO tasks VALUES ('t_recovered','t7','body7','ready',NULL,0,0,0,?,?,?)", (now, now, now))
con.execute("INSERT INTO task_runs (task_id,status,outcome,summary,started_at,ended_at) VALUES ('t_recovered','unblocked','unblocked','провайдер восстановлен, MiniMax снова отвечает 200',?,?)", (now, now+1))

con.commit()
con.close()
PYEOF
[ -f "$BOARD_DIR/kanban.db" ] || fail "fixture db not created"

# --- прогон dry-run ----------------------------------------------------------
# ACTIONS_FILE внутри скрипта = $STATE_DIR/${SCRIPT_NAME}.actions.jsonl.
# SCRIPT_NAME = basename($0) = agent-flow-cancel-on-provider-exhausted.sh.
# Скрипт также делает mkdir -p на $STATE_DIR, но файл пишет прямо в
# $STATE_DIR/${SCRIPT_NAME}.actions.jsonl, без /state/ подкаталога.
ACTIONS_JSONL="$WORK/agent-flow-cancel-on-provider-exhausted.sh.actions.jsonl"
export KANBAN_BOARDS_DIR="$WORK/boards"
export HERMES_HOME="$WORK"
export STATE_DIR="$WORK"
mkdir -p "$STATE_DIR"

# Run dry-run. Use exec to avoid subshell issues with env vars.
OUT=$(cd "$WORK" && KANBAN_BOARDS_DIR="$WORK/boards" HERMES_HOME="$WORK" STATE_DIR="$WORK" \
    bash "$SCRIPT_UNDER_TEST" --dry-run 2>&1) || { echo "$OUT"; fail "dry-run crashed"; }
echo "$OUT"

# --- assertions --------------------------------------------------------------
echo "=== TEST ASSERTIONS ==="

# Test 1
grep -q '"task_id": "t_summary_catch"' "$ACTIONS_JSONL" || fail "T1: t_summary_catch (marker in summary) NOT caught"
echo "  T1 ok — t_summary_catch caught"

# Test 2 (signal=lfe)
LINE=$(grep '"task_id": "t_lfe_catch"' "$ACTIONS_JSONL" 2>/dev/null || true)
[ -n "$LINE" ] || fail "T2: t_lfe_catch (marker in last_failure_error only) NOT caught"
echo "$LINE" | grep -q '"signal": "lfe"' || fail "T2: signal should be 'lfe', got: $LINE"
echo "  T2 ok — t_lfe_catch caught with signal=lfe"

# Test 3 (signal=both)
LINE=$(grep '"task_id": "t_both_catch"' "$ACTIONS_JSONL" 2>/dev/null || true)
[ -n "$LINE" ] || fail "T3: t_both_catch NOT caught"
echo "$LINE" | grep -q '"signal": "both"' || fail "T3: signal should be 'both', got: $LINE"
echo "  T3 ok — t_both_catch caught with signal=both"

# Test 4
grep -q '"task_id": "t_healthy"' "$ACTIONS_JSONL" && fail "T4: t_healthy should NOT be caught"
echo "  T4 ok — t_healthy NOT caught (clean)"

# Test 5
grep -q '"task_id": "t_already_marked"' "$ACTIONS_JSONL" && fail "T5: t_already_marked (has sentinel) should NOT be caught"
echo "  T5 ok — t_already_marked NOT caught (idempotent sentinel)"

# Test 6
grep -q '"task_id": "t_blocked_already"' "$ACTIONS_JSONL" && fail "T6: t_blocked_already (status=blocked) should NOT be caught"
echo "  T6 ok — t_blocked_already NOT caught (already blocked)"

# Test 7
grep -q '"task_id": "t_recovered"' "$ACTIONS_JSONL" && fail "T7: t_recovered (anti-pattern 'провайдер восстановлен') should NOT be caught"
echo "  T7 ok — t_recovered NOT caught (recover anti-pattern)"

# --- idempotency: simulate cancel pass ---------------------------------------
# Add sentinel-marker to t_summary_catch and t_lfe_catch (as if cancel ran)
python3 - "$BOARD_DIR/kanban.db" <<PYEOF
import sqlite3, sys, time
db = sys.argv[1]
con = sqlite3.connect(db)
now = int(time.time())
for tid in ('t_summary_catch', 't_lfe_catch', 't_both_catch'):
    con.execute("INSERT INTO task_comments (task_id,body,created_at) VALUES (?,?,?)",
                (tid, '<!-- agent-flow-cancel-on-provider-exhausted.sh:marker -->\nsimulated', now))
con.commit()
con.close()
PYEOF

# Re-run dry-run
OUT2=$(cd "$WORK" && KANBAN_BOARDS_DIR="$WORK/boards" HERMES_HOME="$WORK" STATE_DIR="$WORK" \
    bash "$SCRIPT_UNDER_TEST" --dry-run 2>&1) || { echo "$OUT2"; fail "dry-run #2 crashed"; }

# Test 8: теперь все 3 должны быть пропущены (idempotency)
for tid in t_summary_catch t_lfe_catch t_both_catch; do
    if grep -q "\"task_id\": \"$tid\"" "$ACTIONS_JSONL"; then
        fail "T8: $tid should NOT be in actions after sentinel added (idempotency)"
    fi
done
echo "  T8 ok — re-run dry-run shows 0 candidates for previously-marked tasks"

pass "cancel-on-provider-exhausted: все 8 кейсов прошли (T1-T8)"