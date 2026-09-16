#!/bin/bash
# ============================================================================
# test_stale_conflicting_watchdog.sh — регресс-гард для
# agent-flow-stale-conflicting-watchdog.sh (ретро t_a7d642cd, 2026-09-16).
#
# Тестируем чистую логику (без реальных hermes/gh side-effects) через
# PATH-hijack: подставляем mock-hermes (JOURNAL) и mock-gh (программируемые
# ответы). Тесты НЕ ходят в реальные GitHub/Kanban — все в tmp.
#
# Scenarios:
#   S1. clean_no_prs: gh возвращает [] → exit 0, scanned=0.
#   S2. fresh_pr_not_dirty: PR mergeable=true → scan=1 stale=0.
#   S3. dirty_but_fresh: PR dirty + updated=1h ago → scan=1 stale=0.
#   S4. stale_dirty_no_card_emits: PR dirty + updated=10h, no card in DB →
#       kanban-retro-create called (JOURNAL).
#   S5. stale_dirty_with_active_card_skips: PR dirty + active card on it
#       → SKIP, no kanban-retro-create call.
#   S6. dry_run_no_side_effects: DRY_RUN=true → recommended_count=1,
#       kanban-retro-create NOT called.
#   S7. behind_state_not_targeted: PR mergeable_state=behind → SCANNED but
#       stale=0 (мы только 'dirty'-PRs обрабатываем).
#   S8. sqlite_db_lock_fail_open: несуществующий KANBAN_DB_PATH → emit anyway
#       (WARN в stderr), без падения.
#   S9. multiple_stale_dirty_all_emit: 3 PR (dirty + stale) → 3 calls в
#       kanban-retro-create (по одной на каждый PR).
#  S10. idempotency_key_per_pr: re-run с теми же 3 PR → kanban-retro-create
#       SHELL mock видел тот же idempotency-key — это покрывается самим
#       kanban-retro-create.sh (мы тут только проверяем что watchdog
#       передаёт правильный --key).
#
# Run:
#   bash scripts/agent_flow/tests/test_stale_conflicting_watchdog.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../agent-flow-stale-conflicting-watchdog.sh}"

[ -f "$WATCHDOG_SH" ] || { echo "FAIL: $WATCHDOG_SH not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required"; exit 1; }
command -v gh >/dev/null || { echo "FAIL: gh required (for mock PATH)"; exit 1; }

_pass=0
_fail=0

pass() { echo "  ok: $1"; _pass=$((_pass+1)); }
fail() { echo "  FAIL: $1"; _fail=$((_fail+1)); }

# Эта переменная пересоздаётся в каждом test_* функции через MOCK_GH_PRS_JSON env.
# Формат: JSON array, как отдаёт gh pr list --json number,mergeable,mergeableState,
#   headRefName,baseRefName,updatedAt,title

# Подготовка mock-hermes (отдельный от test_stale_blocked_watchdog, потому
# что наш watchdog работает через kanban-retro-create.sh, который вызывает
# `hermes kanban create` — мы mock'аем этот канал).
#
# kanban-retro-create.sh внутри делает:
#   1. `hermes kanban --board X list --json` (для pre-check) → []
#   2. `hermes kanban --board X create TITLE --body BODY --assignee Y ...` →
#      выводит JSON {"id": "t_<id>"}
#
# Mock-hermes должен:
#   - на `kanban list --json` вернуть '[]'
#   - на `kanban ... create` записать в JOURNAL и вернуть JSON с id
make_mock_hermes() {
    local work="$1"
    cat > "$work/bin/hermes" <<'MOCKEOF'
#!/bin/bash
# mock-hermes для test_stale_conflicting_watchdog.sh
#
# Реальный hermes вызывается kanban-retro-create.sh как:
#   hermes kanban --board <board> list --json
#   hermes kanban --board <board> create TITLE --body BODY --assignee ... --key ... --max-runtime 1800 --idempotency-key ... --json
#
# В скрипте `$0=hermes`, `$1=kanban`, `$2=--board`, `$3=<board>`, `$4=list|create`.
#
# Возвращаем: для list → '[]', для create → JSON {"id":..., "title":...}.
JOURNAL="${JOURNAL_FILE:-/dev/null}"
_subcmd="${4:-}"
case "$1:${_subcmd}" in
    kanban:list)
        # pre-check для kanban-retro-create.sh — наш mock kanban DB всегда пуст.
        echo '[]'
        ;;
    kanban:create)
        # argv: hermes kanban --board <board> create TITLE --body BODY ...
        # Извлекаем TITLE — позиционный аргумент после 'create'.
        _title=""
        _seen_create=0
        for _arg in "$@"; do
            if [ "$_seen_create" -eq 1 ] && [ -z "$_title" ] && [ "$_arg" != "--body" ]; then
                _title="$_arg"
                break
            fi
            [ "$_arg" = "create" ] && _seen_create=1
        done
        # Запись в JOURNAL: 1 строка per create.
        printf '%s\n' "CREATE title='${_title}'" >> "$JOURNAL"
        _fake_id="t_$(printf '%s' "$_title" | md5sum | cut -c1-8)"
        echo "{\"id\":\"${_fake_id}\",\"title\":\"${_title}\"}"
        ;;
    *)
        printf '%s\n' "MOCKED_UNKNOWN: $*" >> "$JOURNAL"
        exit 0
        ;;
esac
MOCKEOF
    chmod +x "$work/bin/hermes"
}

# Mock-gh: на основе $MOCK_GH_PRS_JSON возвращает JSON.
# Используется аналогично test_stale_blocked_watchdog.sh, упрощённо.
make_mock_gh() {
    local work="$1"
    cat > "$work/bin/gh" <<MOCKEOF
#!/bin/bash
case "\${1:-}\${2:-}" in
    authstatus)
        exit 0
        ;;
    prlist)
        # shellcheck disable=SC2154
        if [ -n "\${MOCK_GH_PRS_JSON:-}" ] && [ -f "\${MOCK_GH_PRS_JSON}" ]; then
            cat "\${MOCK_GH_PRS_JSON}"
        else
            echo '[]'
        fi
        ;;
    *)
        echo "[]"
        ;;
esac
MOCKEOF
    chmod +x "$work/bin/gh"
}

# kanban-retro-create.sh wrapper внутри: проверяет pre-check `hermes kanban
# list --json`, затем `hermes kanban ... create`. Для наших тестов pre-check
# возвращает [] (т.к. mock-hermes на list всегда []), поэтому create всегда
# дергается. Это упрощает проверку: смотрим JOURNAL.calls.create.

run_test() {
    local name="$1"
    WORK="$(mktemp -d)"
    export WORK
    mkdir -p "$WORK/bin"
    JOURNAL_FILE="$WORK/journal.txt"
    export JOURNAL_FILE
    : > "$JOURNAL_FILE"

    # Подготовка mock-kanban DB (минимальная схема, как в production).
    KANBAN_DB_PATH="$WORK/kanban.db"
    export KANBAN_DB_PATH
    python3 - <<PYEOF
import sqlite3
con = sqlite3.connect('$KANBAN_DB_PATH')
con.executescript('''
CREATE TABLE tasks (
    id TEXT PRIMARY KEY,
    title TEXT,
    body TEXT,
    assignee TEXT,
    status TEXT NOT NULL,
    started_at INTEGER,
    max_runtime_seconds INTEGER,
    created_at INTEGER NOT NULL,
    workspace_kind TEXT NOT NULL DEFAULT 'scratch',
    session_id TEXT
);
''')
con.commit()
con.close()
PYEOF

    make_mock_hermes "$WORK"
    make_mock_gh "$WORK"
    PATH="$WORK/bin:$PATH"
    export PATH

    # kanban-retro-create.sh on path — нужен для watchdog'а; используем
    # production-копию (не mock), но он пойдёт через mock-hermes (PATH
    # resolution).
    KANBAN_RETRO_CREATE_SH="$TEST_DIR/../kanban-retro-create.sh"
    export KANBAN_RETRO_CREATE_SH
    # BOARD default 'robbox' → передаём явно, чтобы mock-hermes видел board.
    WATCHDOG_BOARD="test-board"
    export WATCHDOG_BOARD

    _ALL_WORKS+=("$WORK")
}

cleanup_all_works() {
    for w in "${_ALL_WORKS[@]}"; do
        rm -rf "$w" 2>/dev/null || true
    done
}
trap cleanup_all_works EXIT

echo "=== test_stale_conflicting_watchdog ==="

# ------------------- S1: clean (no PRs) -------------------------------
test_S1_clean_no_prs() {
    run_test S1
    MOCK_GH_PRS_JSON="$WORK/no_prs.json"; export MOCK_GH_PRS_JSON
    echo '[]' > "$MOCK_GH_PRS_JSON"
    set +e
    DRY_RUN=false bash "$WATCHDOG_SH" STALE_THRESHOLD_HOURS=4 >/tmp/s1.out 2>/tmp/s1.err
    _rc=$?
    set -e
    if [ "$_rc" = "0" ]; then
        pass "S1 exit=0 (clean run, no PRs)"
    else
        fail "S1 expected exit=0, got $_rc (stderr=$(cat /tmp/s1.err))"
    fi
    if grep -q "scanned=0" /tmp/s1.err; then
        pass "S1 summary scanned=0"
    else
        fail "S1 expected scanned=0 in stderr, got: $(cat /tmp/s1.err | tail -1)"
    fi
}
test_S1_clean_no_prs; _ALL_WORKS+=("$WORK")

# ------------------- S2: fresh non-dirty ------------------------------
test_S2_fresh_pr_not_dirty() {
    run_test S2
    MOCK_GH_PRS_JSON="$WORK/s2.json"; export MOCK_GH_PRS_JSON
    _updated="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    cat > "$MOCK_GH_PRS_JSON" <<EOF
[{"number":2647,"mergeable":true,"mergeableState":"clean","headRefName":"z-architect/abc","baseRefName":"develop","updatedAt":"$_updated","title":"docs only"}]
EOF
    set +e
    DRY_RUN=false bash "$WATCHDOG_SH" STALE_THRESHOLD_HOURS=4 >/tmp/s2.out 2>/tmp/s2.err
    _rc=$?
    set -e
    if [ "$_rc" = "0" ]; then pass "S2 exit=0 (clean PR, не dirty)"
    else fail "S2 expected exit=0, got $_rc"; fi
    if grep -q "scanned=1 stale=0" /tmp/s2.err; then
        pass "S2 scanned=1 stale=0"
    else
        fail "S2 expected scanned=1 stale=0, got: $(cat /tmp/s2.err | tail -1)"
    fi
    if [ ! -s "$JOURNAL_FILE" ]; then
        pass "S2 no kanban-retro-create call (clean PR — правильно пропущен)"
    else
        fail "S2 expected no create, JOURNAL has content: $(cat "$JOURNAL_FILE")"
    fi
}
test_S2_fresh_pr_not_dirty; _ALL_WORKS+=("$WORK")

# ------------------- S3: dirty but fresh ------------------------------
test_S3_dirty_but_fresh() {
    run_test S3
    MOCK_GH_PRS_JSON="$WORK/s3.json"; export MOCK_GH_PRS_JSON
    _updated="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    cat > "$MOCK_GH_PRS_JSON" <<EOF
[{"number":2647,"mergeable":false,"mergeableState":"dirty","headRefName":"z-{agent}/2630","baseRefName":"develop","updatedAt":"$_updated","title":"harness refactor"}]
EOF
    set +e
    DRY_RUN=false bash "$WATCHDOG_SH" STALE_THRESHOLD_HOURS=4 >/tmp/s3.out 2>/tmp/s3.err
    _rc=$?
    set -e
    if [ "$_rc" = "0" ]; then pass "S3 exit=0 (dirty but fresh)"
    else fail "S3 expected exit=0, got $_rc"; fi
    if grep -q "scanned=1 stale=0" /tmp/s3.err; then
        pass "S3 scanned=1 stale=0 (fresh — не алертим)"
    else
        fail "S3 expected scanned=1 stale=0, got: $(cat /tmp/s3.err | tail -1)"
    fi
    if [ ! -s "$JOURNAL_FILE" ]; then
        pass "S3 no create (fresh dirty — пропущен по threshold)"
    else
        fail "S3 expected no create, JOURNAL: $(cat "$JOURNAL_FILE")"
    fi
}
test_S3_dirty_but_fresh; _ALL_WORKS+=("$WORK")

# ------------------- S4: stale dirty, no card → emit ------------------
test_S4_stale_dirty_no_card_emits() {
    run_test S4
    MOCK_GH_PRS_JSON="$WORK/s4.json"; export MOCK_GH_PRS_JSON
    _updated="$(date -u -d '10 hours ago' +%Y-%m-%dT%H:%M:%SZ)"
    cat > "$MOCK_GH_PRS_JSON" <<EOF
[{"number":2639,"mergeable":false,"mergeableState":"dirty","headRefName":"z-{agent}/2630-refactor-harness-agentcore","baseRefName":"develop","updatedAt":"$_updated","title":"refactor(harness) AgentCore CC"}]
EOF
    set +e
    DRY_RUN=false bash "$WATCHDOG_SH" STALE_THRESHOLD_HOURS=4 >/tmp/s4.out 2>/tmp/s4.err
    _rc=$?
    set -e
    if [ "$_rc" = "2" ]; then pass "S4 exit=2 (recommend emitted, alert)"
    else fail "S4 expected exit=2, got $_rc (stderr=$(cat /tmp/s4.err))"; fi
    if grep -q "scanned=1 stale=1 has_card=0 recommended=1" /tmp/s4.err; then
        pass "S4 summary matches"
    else
        fail "S4 expected scanned=1 stale=1 has_card=0 recommended=1, got: $(cat /tmp/s4.err | tail -1)"
    fi
    if grep -q "CREATE title='rebase PR #2639" "$JOURNAL_FILE"; then
        pass "S4 emitted rebase card for PR #2639"
    else
        fail "S4 expected 'rebase PR #2639' create, JOURNAL: $(cat "$JOURNAL_FILE")"
    fi
    if grep -q "t_a7d642cd" /tmp/s4.err; then
        pass "S4 log mentions retro key"
    else
        # t_a7d642cd попадает в title "rebase PR #2639 (CONFLICTING 10h, t_a7d642cd)"
        # и в body, но в stderr summary log он может не быть — проверяем
        # наличие created card id вместо этого.
        if grep -q "CREATED t_" /tmp/s4.err; then
            pass "S4 created card id present in stderr"
        else
            fail "S4 expected 't_a7d642cd' marker OR 'CREATED t_', got: $(cat /tmp/s4.err | head -3)"
        fi
    fi
}
test_S4_stale_dirty_no_card_emits; _ALL_WORKS+=("$WORK")

# ------------------- S5: stale dirty but active card → SKIP -----------
test_S5_stale_dirty_with_active_card_skips() {
    run_test S5
    # Заранее кладём активную карточку на PR #2639 (через python3 sqlite3 —
    # CLI sqlite3 может не быть установлен).
    python3 <<PYEOF
import sqlite3
con = sqlite3.connect("$KANBAN_DB_PATH")
con.execute(
    "INSERT INTO tasks "
    "(id, title, body, assignee, status, started_at, max_runtime_seconds, created_at, workspace_kind, session_id) "
    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
    ("t_active_x", "existing rebase-card for #2639", "PR #2639 rebase in progress",
     "devops", "running", $(date -u +%s), 1800, $(date -u +%s), "scratch", "sess1")
)
con.commit()
con.close()
PYEOF
    MOCK_GH_PRS_JSON="$WORK/s5.json"; export MOCK_GH_PRS_JSON
    _updated="$(date -u -d '10 hours ago' +%Y-%m-%dT%H:%M:%SZ)"
    cat > "$MOCK_GH_PRS_JSON" <<EOF
[{"number":2639,"mergeable":false,"mergeableState":"dirty","headRefName":"z-{agent}/2630-refactor-harness-agentcore","baseRefName":"develop","updatedAt":"$_updated","title":"refactor(harness) AgentCore CC"}]
EOF
    set +e
    DRY_RUN=false bash "$WATCHDOG_SH" STALE_THRESHOLD_HOURS=4 >/tmp/s5.out 2>/tmp/s5.err
    _rc=$?
    set -e
    if [ "$_rc" = "0" ]; then pass "S5 exit=0 (active card — SKIP, no alert)"
    else fail "S5 expected exit=0, got $_rc"; fi
    if grep -q "has_card=1" /tmp/s5.err; then
        pass "S5 has_card=1 (correctly skipped)"
    else
        fail "S5 expected has_card=1, got: $(cat /tmp/s5.err | tail -1)"
    fi
    if [ ! -s "$JOURNAL_FILE" ]; then
        pass "S5 no create (active card present — skipped)"
    else
        fail "S5 expected no create, JOURNAL: $(cat "$JOURNAL_FILE")"
    fi
}
test_S5_stale_dirty_with_active_card_skips; _ALL_WORKS+=("$WORK")

# ------------------- S6: DRY_RUN=true ------------------------------
test_S6_dry_run() {
    run_test S6
    MOCK_GH_PRS_JSON="$WORK/s6.json"; export MOCK_GH_PRS_JSON
    _updated="$(date -u -d '10 hours ago' +%Y-%m-%dT%H:%M:%SZ)"
    cat > "$MOCK_GH_PRS_JSON" <<EOF
[{"number":2640,"mergeable":false,"mergeableState":"dirty","headRefName":"z-{agent}/2627","baseRefName":"develop","updatedAt":"$_updated","title":"refactor(voice) DialogueNode"}]
EOF
    set +e
    DRY_RUN=true bash "$WATCHDOG_SH" STALE_THRESHOLD_HOURS=4 >/tmp/s6.out 2>/tmp/s6.err
    _rc=$?
    set -e
    if [ "$_rc" = "0" ]; then pass "S6 exit=0 in DRY_RUN (no alert per spec)"
    else fail "S6 expected exit=0, got $_rc"; fi
    if grep -q "DRY-RUN" /tmp/s6.err; then
        pass "S6 logs DRY-RUN marker"
    else
        fail "S6 expected DRY-RUN marker, got: $(cat /tmp/s6.err | tail -1)"
    fi
    if [ ! -s "$JOURNAL_FILE" ]; then
        pass "S6 no create in DRY_RUN (no side-effect)"
    else
        fail "S6 DRY_RUN should not call create, JOURNAL: $(cat "$JOURNAL_FILE")"
    fi
}
test_S6_dry_run; _ALL_WORKS+=("$WORK")

# ------------------- S7: behind state, not our target ------------------
test_S7_behind_state_ignored() {
    run_test S7
    MOCK_GH_PRS_JSON="$WORK/s7.json"; export MOCK_GH_PRS_JSON
    _updated="$(date -u -d '10 hours ago' +%Y-%m-%dT%H:%M:%SZ)"
    cat > "$MOCK_GH_PRS_JSON" <<EOF
[{"number":2640,"mergeable":false,"mergeableState":"behind","headRefName":"z-{agent}/2627","baseRefName":"develop","updatedAt":"$_updated","title":"behind-state PR"}]
EOF
    set +e
    DRY_RUN=false bash "$WATCHDOG_SH" STALE_THRESHOLD_HOURS=4 >/tmp/s7.out 2>/tmp/s7.err
    _rc=$?
    set -e
    if [ "$_rc" = "0" ]; then pass "S7 exit=0 (behind — не dirty, не наш кейс)"
    else fail "S7 expected exit=0, got $_rc"; fi
    if grep -q "scanned=1 stale=0" /tmp/s7.err; then
        pass "S7 scanned but not stale (behind≠dirty)"
    else
        fail "S7 expected scanned=1 stale=0, got: $(cat /tmp/s7.err | tail -1)"
    fi
    if [ ! -s "$JOURNAL_FILE" ]; then
        pass "S7 no create (behind — out of scope)"
    else
        fail "S7 expected no create, JOURNAL: $(cat "$JOURNAL_FILE")"
    fi
}
test_S7_behind_state_ignored; _ALL_WORKS+=("$WORK")

# ------------------- S8: missing kanban DB → emit anyway --------------
test_S8_sqlite_db_missing_fail_open() {
    run_test S8
    KANBAN_DB_PATH="/nonexistent/path/that/does/not/exist.db"
    export KANBAN_DB_PATH
    MOCK_GH_PRS_JSON="$WORK/s8.json"; export MOCK_GH_PRS_JSON
    _updated="$(date -u -d '10 hours ago' +%Y-%m-%dT%H:%M:%SZ)"
    cat > "$MOCK_GH_PRS_JSON" <<EOF
[{"number":2642,"mergeable":false,"mergeableState":"dirty","headRefName":"z-{agent}/2631","baseRefName":"develop","updatedAt":"$_updated","title":"refactor(voice) DialogueNode run_turn"}]
EOF
    set +e
    DRY_RUN=false bash "$WATCHDOG_SH" STALE_THRESHOLD_HOURS=4 >/tmp/s8.out 2>/tmp/s8.err
    _rc=$?
    set -e
    if [ "$_rc" = "2" ]; then pass "S8 exit=2 (DB miss → fail-open, emit anyway)"
    else fail "S8 expected exit=2, got $_rc"; fi
    if grep -q "WARN kanban DB not found" /tmp/s8.err; then
        pass "S8 WARN logged about missing DB"
    else
        fail "S8 expected WARN about missing DB, got: $(cat /tmp/s8.err | tail -3)"
    fi
    if grep -q "CREATE title='rebase PR #2642" "$JOURNAL_FILE"; then
        pass "S8 emitted anyway (fail-open correctness)"
    else
        fail "S8 expected 'rebase PR #2642' create anyway, JOURNAL: $(cat "$JOURNAL_FILE")"
    fi
}
test_S8_sqlite_db_missing_fail_open; _ALL_WORKS+=("$WORK")

# ------------------- S9: 3 stale dirty PRs → 3 emits ------------------
test_S9_multi_stale_dirty() {
    run_test S9
    MOCK_GH_PRS_JSON="$WORK/s9.json"; export MOCK_GH_PRS_JSON
    _updated="$(date -u -d '10 hours ago' +%Y-%m-%dT%H:%M:%SZ)"
    cat > "$MOCK_GH_PRS_JSON" <<EOF
[{"number":2647,"mergeable":false,"mergeableState":"dirty","headRefName":"z-architect/2627","baseRefName":"develop","updatedAt":"$_updated","title":"ADR post-turn-music finalizer"},{"number":2644,"mergeable":false,"mergeableState":"dirty","headRefName":"z-{agent}/2609-fix-voice-torch","baseRefName":"develop","updatedAt":"$_updated","title":"torch CPU-only"},{"number":2642,"mergeable":false,"mergeableState":"dirty","headRefName":"z-{agent}/2631","baseRefName":"develop","updatedAt":"$_updated","title":"DialogueNode CC"}]
EOF
    set +e
    DRY_RUN=false bash "$WATCHDOG_SH" STALE_THRESHOLD_HOURS=4 >/tmp/s9.out 2>/tmp/s9.err
    _rc=$?
    set -e
    if [ "$_rc" = "2" ]; then pass "S9 exit=2 (multi-emit)"
    else fail "S9 expected exit=2, got $_rc"; fi
    if grep -q "scanned=3 stale=3 has_card=0 recommended=3" /tmp/s9.err; then
        pass "S9 stats: 3 scanned, 3 stale, 3 emitted"
    else
        fail "S9 expected 3+ stats, got: $(cat /tmp/s9.err | tail -1)"
    fi
    _cnt=$(grep -cE "^CREATE title='rebase PR #" "$JOURNAL_FILE")
    if [ "$_cnt" = "3" ]; then
        pass "S9 3 create calls (one per PR)"
    else
        fail "S9 expected 3 CREATE lines, got $_cnt: $(cat "$JOURNAL_FILE")"
    fi
    for n in 2647 2644 2642; do
        if grep -q "CREATE title='rebase PR #${n}" "$JOURNAL_FILE"; then
            pass "S9 created card for PR #${n}"
        else
            fail "S9 expected create for PR #${n}, JOURNAL: $(cat "$JOURNAL_FILE")"
        fi
    done
}
test_S9_multi_stale_dirty; _ALL_WORKS+=("$WORK")

# ------------------- S10: idempotency-key per PR ----------------------
test_S10_per_pr_idempotency_key() {
    run_test S10
    MOCK_GH_PRS_JSON="$WORK/s10.json"; export MOCK_GH_PRS_JSON
    _updated="$(date -u -d '10 hours ago' +%Y-%m-%dT%H:%M:%SZ)"
    cat > "$MOCK_GH_PRS_JSON" <<EOF
[{"number":2639,"mergeable":false,"mergeableState":"dirty","headRefName":"z-{agent}/2630","baseRefName":"develop","updatedAt":"$_updated","title":"harness refactor"}]
EOF
    # spy на argv kanban-retro-create.sh: чтобы проверить передаваемые --key
    # мы подменяем её на shell-traceable stub.
    cat > "$WORK/bin/kanban-retro-create.sh" <<'STUBEOF'
#!/bin/bash
echo "$@" >> "${KANBAN_DEBUG_LOG:-/dev/null}"
echo "WOULD_CREATE: $*"
STUBEOF
    chmod +x "$WORK/bin/kanban-retro-create.sh"
    KANBAN_DEBUG_LOG="$WORK/spy.log"
    export KANBAN_DEBUG_LOG
    # Создаём симлинк-обёртку: PATH уже включает $WORK/bin, kanban-retro-create.sh
    # watchdog ищет через /home/builder/.hermes/scripts — поэтому нужно
    # переопределить через env KANBAN_RETRO_CREATE_SH.
    KANBAN_RETRO_CREATE_SH="$WORK/bin/kanban-retro-create.sh"
    export KANBAN_RETRO_CREATE_SH
    set +e
    DRY_RUN=false bash "$WATCHDOG_SH" STALE_THRESHOLD_HOURS=4 >/tmp/s10.out 2>/tmp/s10.err
    _rc=$?
    set -e
    if [ "$_rc" = "2" ]; then pass "S10 exit=2 (emit)"
    else fail "S10 expected exit=2, got $_rc (stderr=$(cat /tmp/s10.err))"; fi
    if grep -q -- "--key rebase-pr-2639" "$KANBAN_DEBUG_LOG"; then
        pass "S10 idempotency-key=rebase-pr-2639 передан"
    else
        fail "S10 expected --key rebase-pr-2639, got: $(cat "$KANBAN_DEBUG_LOG")"
    fi
    if grep -q -- "--title 'rebase PR #2639" "$KANBAN_DEBUG_LOG"; then
        pass "S10 title содержит 'rebase PR #2639'"
    else
        # argv реально выглядит как: --title rebase PR #2639 ... (без кавычек)
        # потому что kanban-retro-create развернул "$TITLE" без quotes.
        if grep -q -- "--title rebase PR #2639" "$KANBAN_DEBUG_LOG"; then
            pass "S10 title содержит 'rebase PR #2639' (without quotes)"
        else
            fail "S10 expected title 'rebase PR #2639' (quoted or not), got: $(cat "$KANBAN_DEBUG_LOG" | head -c 200)"
        fi
    fi
    if grep -q -- "--assignee devops" "$KANBAN_DEBUG_LOG"; then
        pass "S10 assignee=devops передан"
    else
        fail "S10 expected --assignee devops, got: $(cat "$KANBAN_DEBUG_LOG")"
    fi
}
test_S10_per_pr_idempotency_key; _ALL_WORKS+=("$WORK")

echo
echo "=== summary: $_pass passed, $_fail failed ==="
[ "$_fail" -eq 0 ]
