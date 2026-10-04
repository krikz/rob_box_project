#!/bin/bash
# ============================================================================
# test_cancel_provider_exhausted_auto_issue.sh — регресс-гард для auto-create-issue
# ветки в agent-flow-cancel-on-provider-exhausted.sh (ADR-0019, kanban t_4aaeeef6).
#
# Контекст (ночной ревью 2026-10-03, t_bf8216cb): 5-й рецидив MiniMax/DeepSeek
# исчерпания прошёл МОЛЧА — воркеры писали «провайдер исчерпан, ждать» в
# комментариях, но НИКТО не открыл incident-issue. Auto-issue закрывает этот
# process-gap: при PROVIDER_EXHAUST_AUTO_ISSUE=1 и наличии cancel-actions в тике
# скрипт создаёт ОДИН GitHub issue с лейблом `recurrent-incident` + root-cause
# ссылкой на #1193.
#
# Scenarios (PATH-hijack mock-gh, без сети):
#   S1. auto-issue OFF (default env) → NO gh issue create (safety).
#   S2. auto-issue ON + empty ACTIONS_FILE (silent tick) → NO gh issue create.
#   S3. auto-issue ON + ACTIONS_FILE non-empty + cooldown FRESH → SKIP (cooldown).
#   S4. auto-issue ON + ACTIONS_FILE non-empty + cooldown STALE + 0 OPEN issues
#       → 1 gh issue create call, with correct label/title/body, cooldown written.
#   S5. auto-issue ON + cooldown OK + 1 OPEN recurrent-incident issue → SKIP (gh-truth).
#   S6. auto-issue ON + cooldown OK + 2 OPEN recurrent-incident issues → STILL
#       SKIP (gh-truth; limit 1 — мы останавливаемся на первом).
#   S7. auto-issue ON + cooldown OK + cooldown NEVER existed + no OPEN issues
#       → 1 gh issue create call (cold start).
#   S8. gh issue create fails (rc=1) → cooldown NOT written, exit 0.
#   S9. multiple assignees via PROVIDER_EXHAUST_ISSUE_ASSIGNEES=alice,bob →
#       2× --assignee в args.
#
# Run:
#   bash scripts/agent_flow/tests/test_cancel_provider_exhausted_auto_issue.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_UNDER_TEST="${SCRIPT_UNDER_TEST:-$TEST_DIR/../agent-flow-cancel-on-provider-exhausted.sh}"

[ -f "$SCRIPT_UNDER_TEST" ] || { echo "FAIL: $SCRIPT_UNDER_TEST not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required"; exit 1; }

PASS=0
FAIL=0
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/bin"

# Mock hermes (для apply_cancel / kanban block / kanban comment) — без сети,
# реальных команд, no-op + логируем каждый вызов.
cat > "$WORK/bin/hermes" <<'EOF'
#!/bin/bash
if [ -n "${MOCK_HERMES_CALL_LOG:-}" ]; then
    echo "hermes_call: $*" >> "${MOCK_HERMES_CALL_LOG}"
fi
exit 0
EOF
chmod +x "$WORK/bin/hermes"

# ----------------------------------------------------------------------------
# Mock gh: программируемые ответы через env-переменные. Все env должны быть
# EXPORT'нуты в родителе → mock-gh их прочтёт через окружение.
# ----------------------------------------------------------------------------
cat > "$WORK/bin/gh" <<'EOF'
#!/bin/bash
_args=("$@")
_clean=()
_skip_next=0
for a in "${_args[@]}"; do
    if [ "$_skip_next" = "1" ]; then
        _skip_next=0
        continue
    fi
    case "$a" in
        --repo|--label|--state|--limit|--json) _skip_next=1 ;;
        --assignee|--title|--body) _skip_next=1 ;;
        *) _clean+=("$a") ;;
    esac
done

case "${_clean[0]} ${_clean[1]:-}" in
    "auth status")
        if [ "${MOCK_GH_AUTH_OK:-1}" = "1" ]; then
            echo "logged in"
            exit 0
        else
            echo "not logged in" >&2
            exit 1
        fi
        ;;
    "issue list")
        if [ -n "${MOCK_OPEN_LABEL_JSON:-}" ] && [ -f "${MOCK_OPEN_LABEL_JSON}" ]; then
            cat "${MOCK_OPEN_LABEL_JSON}"
        else
            echo '[]'
        fi
        exit 0
        ;;
    "issue create")
        if [ -n "${MOCK_GH_CALL_LOG:-}" ]; then
            echo "issue_create" >> "${MOCK_GH_CALL_LOG}"
        fi
        if [ -n "${MOCK_GH_ISSUE_CREATE_BODY_LOG:-}" ]; then
            for ((i=0; i<${#_args[@]}; i++)); do
                case "${_args[$i]}" in
                    --body) printf 'body<<%s\n' "${_args[$((i+1))]}" >> "${MOCK_GH_ISSUE_CREATE_BODY_LOG}" ;;
                    --label) printf 'label=%s\n' "${_args[$((i+1))]}" >> "${MOCK_GH_ISSUE_CREATE_BODY_LOG}" ;;
                    --title) printf 'title=%s\n' "${_args[$((i+1))]}" >> "${MOCK_GH_ISSUE_CREATE_BODY_LOG}" ;;
                    --assignee) printf 'assignee=%s\n' "${_args[$((i+1))]}" >> "${MOCK_GH_ISSUE_CREATE_BODY_LOG}" ;;
                esac
            done
        fi
        rc="${MOCK_GH_ISSUE_CREATE_RC:-0}"
        if [ "$rc" = "0" ]; then
            echo "${MOCK_GH_ISSUE_CREATE_OUT:-https://github.com/krikz/rob_box_project/issues/9999}"
        else
            echo "ERROR: gh issue create simulated failure (rc=$rc)" >&2
        fi
        exit "$rc"
        ;;
    *) echo "fake-gh: unknown command ${_clean[*]:-}" >&2; exit 2 ;;
esac
EOF
chmod +x "$WORK/bin/gh"

# ----------------------------------------------------------------------------
# Фикстура: kanban.db с одной провайдер-карточкой (cancel-кандидат)
# ----------------------------------------------------------------------------
BOARD_DIR="$WORK/boards/robbox"
mkdir -p "$BOARD_DIR"

make_fixture_db() {
    local db="$1"
    rm -f "$db"
    python3 - "$db" <<'PYEOF'
import sqlite3, sys, time
db = sys.argv[1]
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
con.execute("INSERT INTO tasks VALUES ('t_prov1','prov test','Source\n  repo: krikz/rob_box_project\n  issue: #1234','ready',NULL,0,0,0,'HTTP 429 rate limit',?,?)", (now, now))
con.execute("INSERT INTO task_runs (task_id,status,outcome,summary,started_at,ended_at) VALUES ('t_prov1','crashed','crashed','провайдер исчерпан (402/429)',?,?)", (now, now+1))
con.commit()
con.close()
PYEOF
}

# Globals set by run_scenario
SCENARIO_OUT=""
SCENARIO_COOLDOWN_FILE=""
SCENARIO_CALL_LOG=""
SCENARIO_BODY_LOG=""
SCENARIO_OPEN_PATH=""

# run_scenario <label> <cooldown_state> <open_issues_json> [extra_env...]
#   cooldown_state: "absent" | "fresh" | "stale"
#   open_issues_json: filename in $WORK, or "" for absent (creates [] file)
#   extra_env: arbitrary "KEY=VAL" pairs (each becomes an env entry)
run_scenario() {
    local label="$1"
    local cooldown_state="$2"
    local open_json="$3"
    shift 3
    local extra_env=("$@")

    # fresh fixture per scenario
    make_fixture_db "$BOARD_DIR/kanban.db"

    # Setup cooldown
    local cooldown_file="$WORK/cooldown-$label"
    case "$cooldown_state" in
        absent) rm -f "$cooldown_file" ;;
        fresh)  date -u +%s > "$cooldown_file" ;;
        stale)  touch -d "@$(($(date -u +%s) - 25*3600))" "$cooldown_file" ;;
        *) echo "unknown cooldown_state: $cooldown_state"; return 1 ;;
    esac

    # Setup open issues
    local open_path="$WORK/open-$label.json"
    if [ -n "$open_json" ]; then
        printf '%s' "$open_json" > "$open_path"
    else
        echo '[]' > "$open_path"
    fi

    local call_log="$WORK/gh_calls-$label.log"
    local body_log="$WORK/body-$label.log"
    rm -f "$call_log" "$body_log"

    local dry_run_flag="--dry-run"
    case "$label" in
        S4|S7|S8|S9) dry_run_flag="" ;;   # real run, чтобы попасть в gh call (DRY-RUN лишь печатает payload)
    esac

    # Собираем env в массив для env -i
    local env_args=(
        "KANBAN_BOARDS_DIR=$WORK/boards"
        "HERMES_HOME=$WORK"
        "STATE_DIR=$WORK"
        "ISSUE_COOLDOWN_FILE=$cooldown_file"
        "PATH=$WORK/bin:/usr/bin:/bin"
        "HOME=$WORK"
        "HERMES_BIN=$WORK/bin/hermes"  # override SOT-default absolute path
        "MOCK_GH_CALL_LOG=$call_log"
        "MOCK_GH_ISSUE_CREATE_BODY_LOG=$body_log"
        "MOCK_OPEN_LABEL_JSON=$open_path"
        "MOCK_GH_AUTH_OK=1"
        "MOCK_GH_ISSUE_CREATE_RC=0"
        "MOCK_HERMES_CALL_LOG=$WORK/hermes_calls-$label.log"
    )
    for kv in "${extra_env[@]}"; do
        env_args+=("$kv")
    done

    local out
    out=$(env -i "${env_args[@]}" bash "$SCRIPT_UNDER_TEST" $dry_run_flag 2>&1) || true

    SCENARIO_OUT="$out"
    SCENARIO_COOLDOWN_FILE="$cooldown_file"
    SCENARIO_CALL_LOG="$call_log"
    SCENARIO_BODY_LOG="$body_log"
    SCENARIO_OPEN_PATH="$open_path"
}

assert_contains() {
    local name="$1" needle="$2"
    if printf '%s' "$SCENARIO_OUT" | grep -qF -- "$needle"; then
        echo "  ok: $name"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: $name — output did not contain '$needle'"
        FAIL=$((FAIL + 1))
        printf '%s\n' "$SCENARIO_OUT" | tail -5
    fi
}

assert_call_count() {
    local name="$1" expected="$2"
    local got=0
    [ -f "$SCENARIO_CALL_LOG" ] && got=$(grep -c 'issue_create' "$SCENARIO_CALL_LOG" 2>/dev/null || echo 0)
    if [ "$got" = "$expected" ]; then
        echo "  ok: $name ($got)"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: $name — expected $expected gh issue_create call(s), got $got"
        FAIL=$((FAIL + 1))
    fi
}

assert_body_contains() {
    local name="$1" needle="$2"
    if [ -f "$SCENARIO_BODY_LOG" ] && grep -qF -- "$needle" "$SCENARIO_BODY_LOG"; then
        echo "  ok: $name"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: $name — body log did not contain '$needle'"
        FAIL=$((FAIL + 1))
        [ -f "$SCENARIO_BODY_LOG" ] && tail -3 "$SCENARIO_BODY_LOG"
    fi
}

assert_assignee_count() {
    local name="$1" expected="$2"
    local got=0
    [ -f "$SCENARIO_BODY_LOG" ] && got=$(grep -c '^assignee=' "$SCENARIO_BODY_LOG" 2>/dev/null || echo 0)
    if [ "$got" = "$expected" ]; then
        echo "  ok: $name ($got)"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: $name — expected $expected assignee(s), got $got"
        FAIL=$((FAIL + 1))
    fi
}

assert_cooldown_not_written() {
    local name="$1"
    if [ -f "$SCENARIO_COOLDOWN_FILE" ]; then
        local now_epoch file_epoch age_s
        now_epoch="$(date -u +%s)"
        file_epoch="$(stat -c %Y "$SCENARIO_COOLDOWN_FILE" 2>/dev/null || echo 0)"
        age_s=$(( now_epoch - file_epoch ))
        if [ "$age_s" -gt 60 ]; then
            echo "  ok: $name (cooldown file age=${age_s}s > 60s)"
            PASS=$((PASS + 1))
        else
            echo "  FAIL: $name — cooldown written (age=${age_s}s < 60s)"
            FAIL=$((FAIL + 1))
        fi
    else
        echo "  ok: $name (no cooldown file)"
        PASS=$((PASS + 1))
    fi
}

echo "=== S1: PROVIDER_EXHAUST_AUTO_ISSUE=0 (default) → no gh issue create ==="
make_fixture_db "$BOARD_DIR/kanban.db"
local_call_log="$WORK/gh_calls-S1.log"
local_open="$WORK/open-S1.json"
echo '[]' > "$local_open"
rm -f "$local_call_log"
SCENARIO_OUT=$(env -i \
    "KANBAN_BOARDS_DIR=$WORK/boards" "HERMES_HOME=$WORK" "STATE_DIR=$WORK" \
    "ISSUE_COOLDOWN_FILE=$WORK/cooldown-S1" \
    "PATH=$WORK/bin:/usr/bin:/bin" "HOME=$WORK" \
    "HERMES_BIN=$WORK/bin/hermes" \
    "MOCK_GH_CALL_LOG=$local_call_log" \
    "MOCK_OPEN_LABEL_JSON=$local_open" \
    bash "$SCRIPT_UNDER_TEST" --dry-run 2>&1) || true
SCENARIO_CALL_LOG="$local_call_log"
assert_contains "S1: off-by-default message" "AUTO_ISSUE: PROVIDER_EXHAUST_AUTO_ISSUE=0 (off) — skip"
assert_call_count "S1: zero gh issue_create calls" "0"

echo
echo "=== S2: auto-issue ON + empty ACTIONS_FILE (silent tick) → no create ==="
make_fixture_db "$BOARD_DIR/kanban.db"
python3 - "$BOARD_DIR/kanban.db" <<'PYEOF'
import sqlite3, sys
con = sqlite3.connect(sys.argv[1])
con.execute("DELETE FROM tasks WHERE id='t_prov1'")
con.execute("DELETE FROM task_runs WHERE task_id='t_prov1'")
con.commit(); con.close()
PYEOF
local_call_log="$WORK/gh_calls-S2.log"
echo '[]' > "$WORK/open-S2.json"
SCENARIO_OUT=$(env -i \
    "KANBAN_BOARDS_DIR=$WORK/boards" "HERMES_HOME=$WORK" "STATE_DIR=$WORK" \
    "ISSUE_COOLDOWN_FILE=$WORK/cooldown-S2" \
    "PATH=$WORK/bin:/usr/bin:/bin" "HOME=$WORK" \
    "HERMES_BIN=$WORK/bin/hermes" \
    "MOCK_GH_CALL_LOG=$local_call_log" \
    "MOCK_OPEN_LABEL_JSON=$WORK/open-S2.json" \
    "PROVIDER_EXHAUST_AUTO_ISSUE=1" \
    bash "$SCRIPT_UNDER_TEST" --dry-run 2>&1) || true
SCENARIO_CALL_LOG="$local_call_log"
assert_contains "S2: empty-AF message" "AUTO_ISSUE: empty ACTIONS_FILE — skip"
assert_call_count "S2: zero gh issue_create calls" "0"

echo
echo "=== S3: auto-issue ON + ACTIONS_FILE non-empty + cooldown FRESH → SKIP ==="
run_scenario "S3" "fresh" "" "PROVIDER_EXHAUST_AUTO_ISSUE=1"
assert_contains "S3: cooldown-active message" "AUTO_ISSUE: cooldown active"
assert_call_count "S3: zero gh issue_create calls" "0"

echo
echo "=== S4: auto-issue ON + cooldown STALE + 0 OPEN → 1 create + correct payload ==="
run_scenario "S4" "stale" "" "PROVIDER_EXHAUST_AUTO_ISSUE=1"
assert_contains "S4: real-run auto-created message" "AUTO-CREATED recurrent-incident issue"
assert_call_count "S4: exactly 1 gh issue_create call" "1"
assert_body_contains "S4: correct labels" "label=recurrent-incident,hermes,agent:devops"
assert_body_contains "S4: correct title prefix" "[recurrent-incident]"
assert_body_contains "S4: root cause issue ref in body" "#1193"

echo
echo "=== S5: auto-issue ON + cooldown absent + 1 OPEN recurrent-incident → SKIP (gh-truth) ==="
run_scenario "S5" "absent" '[{"number":555}]' "PROVIDER_EXHAUST_AUTO_ISSUE=1"
assert_contains "S5: gh-truth-skip message" "AUTO_ISSUE: open 'recurrent-incident' issues: 1 — skip"
assert_call_count "S5: zero gh issue_create calls" "0"

echo
echo "=== S6: 2 OPEN issues → STILL SKIP (gh-truth; >0 скипает) ==="
run_scenario "S6" "absent" '[{"number":555},{"number":556}]' "PROVIDER_EXHAUST_AUTO_ISSUE=1"
assert_contains "S6: gh-truth-skip with 2" "AUTO_ISSUE: open 'recurrent-incident' issues: 2 — skip"
assert_call_count "S6: zero gh issue_create calls" "0"

echo
echo "=== S7: cold start + no OPEN issues → 1 create (acceptance) ==="
run_scenario "S7" "absent" "" "PROVIDER_EXHAUST_AUTO_ISSUE=1"
assert_call_count "S7: exactly 1 gh issue_create call (cold start)" "1"

echo
echo "=== S8: gh issue create fails (rc=1) → cooldown NOT written ==="
make_fixture_db "$BOARD_DIR/kanban.db"
SCENARIO_COOLDOWN_FILE="$WORK/cooldown-S8"
rm -f "$SCENARIO_COOLDOWN_FILE"
SCENARIO_CALL_LOG="$WORK/gh_calls-S8.log"
SCENARIO_BODY_LOG="$WORK/body-S8.log"
SCENARIO_OPEN_PATH="$WORK/open-S8.json"
echo '[]' > "$SCENARIO_OPEN_PATH"
rm -f "$SCENARIO_CALL_LOG" "$SCENARIO_BODY_LOG"
SCENARIO_OUT=$(env -i \
    "KANBAN_BOARDS_DIR=$WORK/boards" "HERMES_HOME=$WORK" "STATE_DIR=$WORK" \
    "ISSUE_COOLDOWN_FILE=$SCENARIO_COOLDOWN_FILE" \
    "PATH=$WORK/bin:/usr/bin:/bin" "HOME=$WORK" \
    "HERMES_BIN=$WORK/bin/hermes" \
    "MOCK_GH_CALL_LOG=$SCENARIO_CALL_LOG" \
    "MOCK_GH_ISSUE_CREATE_BODY_LOG=$SCENARIO_BODY_LOG" \
    "MOCK_OPEN_LABEL_JSON=$SCENARIO_OPEN_PATH" \
    "PROVIDER_EXHAUST_AUTO_ISSUE=1" \
    "MOCK_GH_ISSUE_CREATE_RC=1" \
    bash "$SCRIPT_UNDER_TEST" 2>&1) || true
assert_contains "S8: gh-fail message" "AUTO_ISSUE: ERROR gh issue create failed"
assert_cooldown_not_written "S8: cooldown NOT written on gh failure"

echo
echo "=== S9: PROVIDER_EXHAUST_ISSUE_ASSIGNEES=alice,bob → 2× --assignee ==="
run_scenario "S9" "absent" "" \
    "PROVIDER_EXHAUST_AUTO_ISSUE=1" \
    "PROVIDER_EXHAUST_ISSUE_ASSIGNEES=alice,bob"
assert_call_count "S9: 1 gh issue_create call" "1"
assert_assignee_count "S9: 2 --assignee flags" "2"
assert_body_contains "S9: alice in args" "assignee=alice"
assert_body_contains "S9: bob in args" "assignee=bob"

echo
echo "=== S10: --dry-run + auto-issue ON → log payload, no gh call ==="
make_fixture_db "$BOARD_DIR/kanban.db"
local_call_log="$WORK/gh_calls-S10.log"
local_body_log="$WORK/body-S10.log"
local_open="$WORK/open-S10.json"
echo '[]' > "$local_open"
rm -f "$local_call_log" "$local_body_log"
SCENARIO_OUT=$(env -i \
    "KANBAN_BOARDS_DIR=$WORK/boards" "HERMES_HOME=$WORK" "STATE_DIR=$WORK" \
    "ISSUE_COOLDOWN_FILE=$WORK/cooldown-S10" \
    "PATH=$WORK/bin:/usr/bin:/bin" "HOME=$WORK" \
    "HERMES_BIN=$WORK/bin/hermes" \
    "MOCK_GH_CALL_LOG=$local_call_log" \
    "MOCK_GH_ISSUE_CREATE_BODY_LOG=$local_body_log" \
    "MOCK_OPEN_LABEL_JSON=$local_open" \
    "PROVIDER_EXHAUST_AUTO_ISSUE=1" \
    bash "$SCRIPT_UNDER_TEST" --dry-run 2>&1) || true
SCENARIO_CALL_LOG="$local_call_log"
assert_contains "S10: dry-run prints would-create" "AUTO_ISSUE [DRY-RUN] would: gh issue create"
assert_contains "S10: dry-run prints title" "AUTO_ISSUE [DRY-RUN] would: title="
assert_call_count "S10: zero gh issue_create calls" "0"

echo
echo "=== summary: PASS=$PASS FAIL=$FAIL ==="
[ "$FAIL" = "0" ] || exit 1
echo "ok: cancel-on-provider-exhausted auto-issue: все 10 сценариев прошли (S1-S10)"