#!/bin/bash
# ============================================================================
# test_e2e_process_deploy_recovery.sh — ретро 14.08 t_d01fe536
#
# Deploy-fail ветка e2e-process должна создавать recovery-карточку devops'у
# (🔧 re-deploy <round>), а не молча errored++ (было: issue #1229 закрылся БЕЗ
# e2e, re-round не создавался, recovery-карточки не было).
#
# Стратегия: НЕ гоним весь e2e-process (нужен полный round: build+deploy моки,
# тяжёлый harness). Вместо этого вырезаем из живого скрипта реальный блок
# «deploy fail → recovery card» по маркерам (без дублирования кода — тест
# читает тот же файл, что идёт в прод) и запускаем его с мокнутым `hermes`:
#   A. карточки нет            → kanban create вызван (assignee=devops)
#   B. активная карточка есть  → create НЕ вызван (skip)
#   C. карточка done есть      → create вызван заново (fresh)
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_deploy_recovery.sh
# ============================================================================
set -euo pipefail

TEST_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_LIB_DIR/lib/mock_env.sh"

E2E_PROCESS="$REPO_ROOT/agent-flow-e2e-process.sh"

# --- Извлечь deploy-fail recovery-блок из живого скрипта ---------------------
# Маркеры: строка с "# Ретро 14.08 (t_d01fe536): deploy fail" → до
# "errored=$((errored+1)); continue" (первое вхождение после блока).
# Последняя строка (errored++/continue) НЕ включается — в изолированном
# контексте теста нет цикла/переменной errored.
extract_recovery_block() {
    awk '
        /# Ретро 14.08 \(t_d01fe536\): deploy fail/ { inblock=1 }
        inblock && /errored=\$\(\(errored\+1\)\); continue/ { exit }
        inblock { print }
    ' "$E2E_PROCESS"
}

# --- Мок hermes: journal + state-driven kanban list -------------------------
install_hermes_mock() {
    local bin_dir="$TEST_TMP/bin"
    mkdir -p "$bin_dir"
    cat > "$bin_dir/hermes" <<'HERMES_MOCK_EOF'
#!/bin/bash
state="${KANBAN_STATE:-}"
journal="${KANBAN_JOURNAL:-/dev/null}"
ts="$(date -Iseconds 2>/dev/null || date)"
journal() { printf '%s\t%s\n' "$ts" "$*" >>"$journal"; }
if [ "${1:-}" = "kanban" ] && [ "${2:-}" = "--board" ]; then
    # hermes kanban --board <board> <subcmd> ... → сдвигаем kanban --board <board>
    shift 3 || true
    subcmd="${1:-}"; shift || true
    case "$subcmd" in
        list)
            journal "kanban list"
            [ -f "$state" ] && cat "$state" || printf '[]'
            exit 0
            ;;
        create)
            journal "kanban create $*"
            exit 0
            ;;
        *)
            journal "kanban $subcmd $*"
            exit 0
            ;;
    esac
fi
journal "hermes $*"
exit 0
HERMES_MOCK_EOF
    chmod +x "$bin_dir/hermes"
}

# --- Выполнить recovery-блок в изоляции --------------------------------------
run_recovery_block() {
    (
        export PATH="$TEST_TMP/bin:$PATH"
        export KANBAN_BOARD="robbox"
        export ROUND_BRANCH="z-{e2e}/test-round-109"
        export GH_REPO="krikz/rob_box_project"
        export DEPLOY_WORKFLOW="L-Deploy and Verify.yml"
        export KANBAN_STATE="$KANBAN_STATE"
        export KANBAN_JOURNAL="$KANBAN_JOURNAL"
        number=1229
        log() { printf 'LOG %s\n' "$*" >>"$KANBAN_JOURNAL"; }
        # shellcheck disable=SC1090
        eval "$(extract_recovery_block)"
    )
}

# ---------------------------------------------------------------------------
# A. Карточки нет → kanban create вызван (assignee=devops, re-deploy title).
# ---------------------------------------------------------------------------
test_A_no_card_creates_recovery() {
    new_test
    install_hermes_mock
    KANBAN_STATE="$TEST_TMP/state_A.json"
    KANBAN_JOURNAL="$TEST_TMP/journal_A.log"
    printf '[]' > "$KANBAN_STATE"
    : > "$KANBAN_JOURNAL"

    run_recovery_block

    local journal
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "kanban create" "$journal" "A: create вызван"
    assert_contains "re-deploy z-{e2e}/test-round-109" "$journal" "A: title содержит round-ветку"
    assert_contains "assignee devops" "$journal" "A: assignee=devops"
    assert_contains "deploy recovery card created" "$journal" "A: лог об успехе"
}

# ---------------------------------------------------------------------------
# B. Активная (running) карточка есть → create НЕ вызван (skip).
# ---------------------------------------------------------------------------
test_B_active_card_skips_create() {
    new_test
    install_hermes_mock
    KANBAN_STATE="$TEST_TMP/state_B.json"
    KANBAN_JOURNAL="$TEST_TMP/journal_B.log"
    cat > "$KANBAN_STATE" <<'EOF'
[{"id":"t_abc123","title":"🔧 re-deploy z-{e2e}/test-round-109 — deploy failed (issue #1229)","status":"running"}]
EOF
    : > "$KANBAN_JOURNAL"

    run_recovery_block

    local journal
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "kanban list" "$journal" "B: list вызван"
    assert_not_contains "kanban create" "$journal" "B: create НЕ вызван (активная карточка)"
    assert_contains "already active" "$journal" "B: лог о skip"
}

# ---------------------------------------------------------------------------
# C. Карточка done есть → create вызван заново (fresh ready-карточка).
# ---------------------------------------------------------------------------
test_C_done_card_creates_fresh() {
    new_test
    install_hermes_mock
    KANBAN_STATE="$TEST_TMP/state_C.json"
    KANBAN_JOURNAL="$TEST_TMP/journal_C.log"
    cat > "$KANBAN_STATE" <<'EOF'
[{"id":"t_done999","title":"🔧 re-deploy z-{e2e}/test-round-109 — deploy failed (issue #1229)","status":"done"}]
EOF
    : > "$KANBAN_JOURNAL"

    run_recovery_block

    local journal
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "kanban create" "$journal" "C: create вызван (старая done)"
    assert_contains "deploy recovery card created" "$journal" "C: лог об успехе"
}

run_test "A. нет карточки → recovery-карточка создана" test_A_no_card_creates_recovery
run_test "B. активная карточка → skip (без дублей)" test_B_active_card_skips_create
run_test "C. done-карточка → свежая ready-карточка" test_C_done_card_creates_fresh

# ---------------------------------------------------------------------------
summary
