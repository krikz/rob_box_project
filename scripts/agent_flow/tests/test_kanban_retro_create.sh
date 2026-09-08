#!/bin/bash
# ============================================================================
# test_kanban_retro_create.sh — acceptance-тесты dedup-guard
# scripts/agent_flow/kanban-retro-create.sh (ретро 13.08 t_35ff29f1).
#
# Стратегия (как в test_merge_gate_*.sh): мокаем `hermes` шелл-скриптом,
# который читает состояние из $TEST_TMP/kanban_list.json и пишет каждый
# вызов в journal. Тесты НЕ трогают реальный канбан.
#
# Проверяемые гарантии:
#   A. Нет существующей карточки → create c --idempotency-key "retro:<key>"
#      и маркером 'ретро-key: <key>' в body (вписан скриптом).
#   B. Есть не-archived карточка с маркером `ретро-key: <key>` → SKIP, create
#      не вызывается.
#   C. Есть не-archived карточка с тем же нормализованным title → SKIP.
#   D. Маркер только у ARCHIVED карточки → НЕ скипаем (мёртвая карточка),
#      create выполняется.
#   E. --dry-run без существующей → WOULD_CREATE, create не вызывается.
#   F. --dry-run с существующей → SKIP.
#   G. Без --key ключ берётся из title (slugify), единый для idempotency
#      и маркера.
#   H. Usage: без --assignee → exit 2.
#   I. `kanban list --json` падает → деградация: create с idempotency-key.
#   J. --skill с категорийным именем → pass-through (guard снят в ADR-0023).
#   --- ADR-0079 (issue #2159) — слой 4, issue-label guard ---------------
#   K. nightly-review-* + есть открытое issue за текущую ISO-неделю → SKIP.
#   L. nightly-review-* + issue из ПРОШЛОЙ недели → create проходит.
#   M. nightly-review-* + gh недоступен → fail-open, create проходит.
#   N. component-review-* тоже триггерит слой 4.
#   O. Не-prefix ключ → слой 4 НЕ включается, gh не дёргается.
#
# Invocation:
#   bash tests/test_kanban_retro_create.sh
# Возвращает 0 при всех pass, ненулевой при первом fail.
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/.." && pwd)"
RETRO_CREATE="$REPO_ROOT/kanban-retro-create.sh"

TEST_TMP="${TEST_TMP:-/tmp/agent-flow-retro-create-tests.$$}"
rm -rf "$TEST_TMP"
mkdir -p "$TEST_TMP/bin"

KANBAN_JOURNAL="$TEST_TMP/journal"
KANBAN_LIST_FILE="$TEST_TMP/kanban_list.json"
GH_ISSUE_FILE="$TEST_TMP/gh_issues.json"
GH_ISSUE_JOURNAL="$TEST_TMP/gh_journal"
GH_FAIL="${GH_FAIL:-}"  # если 1 — mock gh валится с exit 1
# Mock читает эти пути из ОКРУЖЕНИЯ (мок запускается в под-шеллах скрипта:
# "$(hermes kanban ...)"), поэтому переменные обязаны быть exported.
export KANBAN_JOURNAL KANBAN_LIST_FILE GH_ISSUE_FILE GH_ISSUE_JOURNAL GH_FAIL

# --- mock hermes ------------------------------------------------------------
cat > "$TEST_TMP/bin/hermes" <<'HERMES_MOCK_EOF'
#!/bin/bash
# Mock hermes: поддерживает только `kanban --board X list --json` и
# `kanban --board X create ... --json`. Все вызовы пишутся в journal.
journal="${KANBAN_JOURNAL:-/dev/null}"
printf 'hermes\t%s\n' "$*" >> "$journal"
sub="${4:-}"
case "$sub" in
    list)
        if [ -n "${KANBAN_LIST_FAIL:-}" ]; then
            echo "mock: kanban list failed (closed database)" >&2
            exit 1
        fi
        cat "${KANBAN_LIST_FILE:-/dev/null}"
        ;;
    create)
        # Имитируем реальный CLI: при совпавшем idempotency-key возвращает
        # существующий id (в тестах не используется — SKIP ловится раньше).
        cat <<'CREATE_EOF'
{"id": "t_testNEW", "title": "retro: test", "body": "b", "assignee": "devops", "status": "ready"}
CREATE_EOF
        ;;
    *)
        echo "mock: unexpected kanban subcommand: $sub" >&2
        exit 2
        ;;
esac
exit 0
HERMES_MOCK_EOF
chmod +x "$TEST_TMP/bin/hermes"

# --- mock gh ---------------------------------------------------------------
# Используется слоем 4 (ADR-0079): читает $GH_ISSUE_FILE — массив JSON
# [{number, createdAt}, ...] открытых issues с label nightly-review.
# Если $GH_FAIL=1 — возвращает exit 1 (имитирует недоступность gh).
cat > "$TEST_TMP/bin/gh" <<'GH_MOCK_EOF'
#!/bin/bash
gh_journal="${GH_ISSUE_JOURNAL:-/dev/null}"
printf 'gh\t%s\n' "$*" >> "$gh_journal"
if [ -n "${GH_FAIL:-}" ]; then
    echo "mock: gh unavailable" >&2
    exit 1
fi
cmd="${1:-}"
case "$cmd" in
    issue)
        sub="${2:-}"
        case "$sub" in
            list)
                if [ -f "${GH_ISSUE_FILE:-/dev/null}" ]; then
                    cat "$GH_ISSUE_FILE"
                else
                    echo '[]'
                fi
                ;;
            *)
                echo '[]'
                ;;
        esac
        ;;
    *)
        echo '[]'
        ;;
esac
exit 0
GH_MOCK_EOF
chmod +x "$TEST_TMP/bin/gh"

# --- registry ---------------------------------------------------------------
TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

run_test() {  # $1=name, $2=function
    local name="$1" fn="$2"
    TESTS_TOTAL=$((TESTS_TOTAL+1))
    printf '[ RUN     ] %s\n' "$name"
    if "$fn"; then
        TESTS_PASSED=$((TESTS_PASSED+1))
        printf '[   PASS  ] %s\n' "$name"
    else
        TESTS_FAILED=$((TESTS_FAILED+1))
        FAILED_NAMES+=("$name")
        printf '[   FAIL  ] %s\n' "$name"
    fi
}

assert_eq() {  # $1=expected $2=actual $3=msg
    if [ "$1" != "$2" ]; then
        printf '  assert fail: %s\n    expected: %q\n    actual:   %q\n' "$3" "$1" "$2" >&2
        return 1
    fi
}

assert_contains() {  # $1=needle $2=haystack $3=msg
    case "$2" in
        *"$1"*) return 0 ;;
        *)
            printf '  assert fail: %s\n    needle:   %q\n    haystack: %q\n' "$3" "$1" "$2" >&2
            return 1
            ;;
    esac
}

assert_not_contains() {  # $1=needle $2=haystack $3=msg
    case "$2" in
        *"$1"*)
            printf '  assert fail: %s\n    needle should NOT appear: %q\n    haystack: %q\n' "$3" "$1" "$2" >&2
            return 1
            ;;
    esac
    return 0
}

# --- helpers ----------------------------------------------------------------
reset_fixture() {
    : > "$KANBAN_JOURNAL"
    : > "$GH_ISSUE_JOURNAL"
    printf '[]\n' > "$KANBAN_LIST_FILE"
    printf '[]\n' > "$GH_ISSUE_FILE"
    unset KANBAN_LIST_FAIL GH_FAIL
}

run_retro_create() {
    PATH="$TEST_TMP/bin:$PATH" \
    GH_REPO="krikz/rob_box_project" \
    GH_BIN=gh \
    HERMES_BIN=hermes \
    KANBAN_BIN="$TEST_TMP/bin/hermes" \
        "$RETRO_CREATE" "$@"
}

# --- scenarios --------------------------------------------------------------
# A. Нет существующей карточки → create c idempotency-key и маркером в body.
test_A_create_with_idempotency_and_marker() {
    reset_fixture
    local out
    out="$(run_retro_create --title "ретро: e2e-стоп 13.08 — build-раннеры" \
        --body "## ФАКТЫ" --assignee devops --skill agent-flow-e2e-pipeline 2>&1)"
    assert_contains "CREATED t_testNEW" "$out" "create success reported" || return 1
    local journal
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "create" "$journal" "create was invoked" || return 1
    assert_contains "--idempotency-key retro:ретро-e2e-стоп-13-08-build-раннеры" \
        "$journal" "idempotency-key derived from title" || return 1
    assert_contains "ретро-key: ретро-e2e-стоп-13-08-build-раннеры" \
        "$journal" "body marker injected" || return 1
    assert_contains "--skill agent-flow-e2e-pipeline" "$journal" "skill passed through" || return 1
    assert_contains "--assignee devops" "$journal" "assignee passed through" || return 1
}

# B. Не-archived карточка с маркером `ретро-key: <key>` → SKIP, create НЕ зовём.
test_B_skip_existing_marker() {
    reset_fixture
    # ВАЖНО: в JSON-строке нужен ЭКРАНИРОВАННЫЙ \\n (printf '\\n' → backslash-n),
    # реальный перевод строки внутри JSON-строки невалиден → парсер бы не нашёл.
    printf '[{"id":"t_aaaa","status":"ready","title":"ретро: что-то","body":"## ФАКТЫ\\nретро-key: e2e-stop-build-runners"}]' \
        > "$KANBAN_LIST_FILE"
    local out
    out="$(run_retro_create --title "ретро: e2e-стоп 13.08 16:30Z — 3 L-Build" \
        --body "## ФАКТЫ" --assignee devops --key e2e-stop-build-runners 2>&1)"
    assert_contains "SKIP t_aaaa" "$out" "skip with existing id reported" || return 1
    local journal
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_not_contains "create" "$journal" "create NOT invoked when marker exists" || return 1
}

# C. Не-archived карточка с тем же нормализованным title (без маркера) → SKIP.
test_C_skip_same_title() {
    reset_fixture
    printf '[{"id":"t_bbbb","status":"todo","title":"ретро:  e2e-стоп  13.08 — build-раннеры","body":"старое тело"}]' \
        > "$KANBAN_LIST_FILE"
    local out
    out="$(run_retro_create --title "ретро: e2e-стоп 13.08 — build-раннеры" \
        --body "## ФАКТЫ" --assignee devops 2>&1)"
    assert_contains "SKIP t_bbbb" "$out" "skip by normalized title" || return 1
    local journal
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_not_contains "create" "$journal" "create NOT invoked on title match" || return 1
}

# D. Маркер только у ARCHIVED карточки → не скипаем, create выполняется.
test_D_archived_marker_not_blocking() {
    reset_fixture
    printf '[{"id":"t_cccc","status":"archived","title":"ретро: старая","body":"ретро-key: e2e-stop-build-runners"}]' \
        > "$KANBAN_LIST_FILE"
    local out
    out="$(run_retro_create --title "ретро: e2e-стоп повторно" \
        --body "## ФАКТЫ" --assignee devops --key e2e-stop-build-runners 2>&1)"
    assert_contains "CREATED t_testNEW" "$out" "archived card does not block new work" || return 1
    local journal
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "create" "$journal" "create invoked after archived-only match" || return 1
}

# E. --dry-run без существующей → WOULD_CREATE, create не вызывается.
test_E_dry_run_would_create() {
    reset_fixture
    local out
    out="$(run_retro_create --dry-run --title "ретро: тест dry-run" \
        --body "b" --assignee devops --key dry-test 2>&1)"
    assert_contains "WOULD_CREATE idempotency_key=retro:dry-test" "$out" "dry-run reports key" || return 1
    local journal
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_not_contains "create" "$journal" "dry-run must not invoke create" || return 1
}

# F. --dry-run с существующей → SKIP.
test_F_dry_run_skip() {
    reset_fixture
    printf '[{"id":"t_dddd","status":"ready","title":"ретро: тест","body":"ретро-key: dry-test"}]' \
        > "$KANBAN_LIST_FILE"
    local out
    out="$(run_retro_create --dry-run --title "ретро: тест другой" \
        --body "b" --assignee devops --key dry-test 2>&1)"
    assert_contains "SKIP t_dddd" "$out" "dry-run respects existing marker" || return 1
}

# G. Без --key ключ берётся из title (slugify) — один ключ для idempotency
#    и маркера.
test_G_key_fallback_from_title() {
    reset_fixture
    local out
    out="$(run_retro_create --title "ретро:  Баг  STT  #42 — fallback" \
        --body "b" --assignee backend 2>&1)"
    assert_contains "CREATED t_testNEW" "$out" "create success" || return 1
    local journal
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "--idempotency-key retro:ретро-баг-stt-42-fallback" \
        "$journal" "slugified key used for idempotency" || return 1
    assert_contains "ретро-key: ретро-баг-stt-42-fallback" \
        "$journal" "same slugified key used as body marker" || return 1
}

# H. Usage: без --assignee → exit 2.
test_H_usage_error() {
    reset_fixture
    local rc=0
    run_retro_create --title "ретро: без assignee" --body "b" >/dev/null 2>&1 || rc=$?
    assert_eq "2" "$rc" "missing --assignee exits 2"
}

# I. `kanban list --json` падает → деградация: create с idempotency-key.
test_I_list_failure_degradation() {
    reset_fixture
    export KANBAN_LIST_FAIL=1
    local out
    out="$(run_retro_create --title "ретро: деградация" \
        --body "b" --assignee devops --key degrade-test 2>&1)"
    assert_contains "CREATED t_testNEW" "$out" "create proceeds when list fails" || return 1
    local journal
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "--idempotency-key retro:degrade-test" "$journal" "idempotency still applied" || return 1
    unset KANBAN_LIST_FAIL
}

# J. --skill с категорийным (не плоским) именем больше не режется guard'ом
#    (ADR-0023: guard с плоской моделью удалён; fail-fast при unknown skill
#    делает vendor-патч на `kanban create`). Карточка создаётся, скилл
#    pass-through.
test_J_category_skill_not_rejected() {
    reset_fixture
    local out
    out="$(run_retro_create --title "ретро: sdlc категория" \
        --body "b" --assignee devops --skill sdlc-review 2>&1)"
    assert_contains "CREATED t_testNEW" "$out" "category-nested skill does not block create" || return 1
    local journal
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "--skill sdlc-review" "$journal" "skill passed through" || return 1
}

# ============================================================================
# Run
# ============================================================================
run_test "A. create + idempotency-key + body marker" test_A_create_with_idempotency_and_marker
run_test "B. existing marker → SKIP, no create" test_B_skip_existing_marker
run_test "C. same normalized title → SKIP" test_C_skip_same_title
run_test "D. archived-only marker → create proceeds" test_D_archived_marker_not_blocking
run_test "E. dry-run → WOULD_CREATE, no create" test_E_dry_run_would_create
run_test "F. dry-run + existing → SKIP" test_F_dry_run_skip
run_test "G. key fallback from title (slugify)" test_G_key_fallback_from_title
run_test "H. usage error without --assignee" test_H_usage_error
run_test "I. list failure degradation" test_I_list_failure_degradation
run_test "J. category-nested skill → create (guard removed)" test_J_category_skill_not_rejected
# --- ADR-0079 (issue #2159): слой 4 — issue-label guard --------------------
# K. nightly-review-* + есть открытое issue с label nightly-review за ISO-неделю
#    → SKIP, create НЕ вызывается (дайджест уже ушёл через issue).
test_K_issue_label_guard_skip() {
    reset_fixture
    # Текущая ISO-неделя (date -u +%G-W%V) — эмулируем issue, созданное сегодня.
    local iso_ts
    iso_ts="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf '[{"number": 2159, "createdAt": "%s", "state": "OPEN"}]' \
        "$iso_ts" > "$GH_ISSUE_FILE"
    local out journal
    out="$(run_retro_create --title "🌙 ночной ревью: voice-pipeline churn" \
        --body "## Сводка" --assignee devops --key "nightly-review-$(date -u +%G-W%V)" 2>&1)"
    assert_contains "SKIP (gh-issue-label guard" "$out" "K: skip when issue exists in week" || return 1
    assert_contains "#2159" "$out" "K: skip message names the issue" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_not_contains "create" "$journal" "K: create NOT invoked on guard hit" || return 1
}

# L. nightly-review-* + открытое issue с label nightly-review, НО создано
#    ДО текущей ISO-недели → слой 4 не срабатывает, create проходит.
test_L_issue_label_guard_old_issue() {
    reset_fixture
    # 2026-W20 (11 мая 2026) — точно не текущая неделя.
    printf '[{"number": 1888, "createdAt": "2026-05-11T10:00:00Z", "state": "OPEN"}]' \
        > "$GH_ISSUE_FILE"
    local out journal
    out="$(run_retro_create --title "🌙 ночной ревью: voice-pipeline churn" \
        --body "## Сводка" --assignee devops --key "nightly-review-$(date -u +%G-W%V)" 2>&1)"
    assert_contains "CREATED t_testNEW" "$out" "L: create proceeds for stale issue" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "create" "$journal" "L: create invoked when issue is from prior week" || return 1
}

# M. nightly-review-* + gh недоступен → fail-open, create проходит
#    (обратная сторона observability: лучше дубль, который поймает merge-gate,
#    чем false-negative в ночи).
test_M_issue_label_guard_gh_fail_open() {
    reset_fixture
    export GH_FAIL=1
    local out journal
    out="$(run_retro_create --title "🌙 ночной ревью: voice-pipeline churn" \
        --body "## Сводка" --assignee devops --key "nightly-review-$(date -u +%G-W%V)" 2>&1)"
    assert_contains "CREATED t_testNEW" "$out" "M: gh down → fail-open, create proceeds" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "create" "$journal" "M: create invoked despite gh failure" || return 1
}

# N. component-review-* тоже покрывается слоем 4 (любой ключ с префиксом
#    nightly-review- или component-review-).
test_N_component_review_key_triggers_guard() {
    reset_fixture
    local iso_ts
    iso_ts="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf '[{"number": 2160, "createdAt": "%s", "state": "OPEN"}]' \
        "$iso_ts" > "$GH_ISSUE_FILE"
    local out journal
    out="$(run_retro_create --title "🔍 ревью компонента: rob_box_voice" \
        --body "## Сводка" --assignee devops --key "component-review-rob_box_voice-$(date -u +%G-W%V)" 2>&1)"
    assert_contains "SKIP (gh-issue-label guard" "$out" "N: component-review-* also guarded" || return 1
    assert_contains "#2160" "$out" "N: skip message names the issue" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_not_contains "create" "$journal" "N: create NOT invoked for component-review" || return 1
}

# O. Не-prefix ключ (e2e-stop-build-runners) → слой 4 НЕ включается,
#    gh НЕ дёргается.
test_O_non_prefix_key_skips_guard() {
    reset_fixture
    printf '[{"number": 9999, "createdAt": "%sT00:00:00Z", "state": "OPEN"}]' \
        "$(date -u +%Y-%m-%d)" > "$GH_ISSUE_FILE"
    local out journal gh_journal
    out="$(run_retro_create --title "ретро: e2e-стоп 13.08 — build-раннеры" \
        --body "## ФАКТЫ" --assignee devops --key e2e-stop-build-runners 2>&1)"
    assert_contains "CREATED t_testNEW" "$out" "O: non-prefix key bypasses layer 4" || return 1
    journal="$(cat "$KANBAN_JOURNAL")"
    assert_contains "create" "$journal" "O: create invoked" || return 1
    gh_journal="$(cat "$GH_ISSUE_JOURNAL")"
    assert_not_contains "issue list" "$gh_journal" "O: gh issue list NOT called for non-prefix key" || return 1
}

run_test "K. layer-4 skip on open issue in current ISO week"  test_K_issue_label_guard_skip
run_test "L. layer-4 ignores issue from prior week"            test_L_issue_label_guard_old_issue
run_test "M. layer-4 fail-open when gh unavailable"            test_M_issue_label_guard_gh_fail_open
run_test "N. component-review-* also triggers layer-4"         test_N_component_review_key_triggers_guard
run_test "O. non-prefix key bypasses layer-4 (gh not called)"  test_O_non_prefix_key_skips_guard

echo
echo "TOTAL: $TESTS_TOTAL  PASS: $TESTS_PASSED  FAIL: $TESTS_FAILED"
if [ "$TESTS_FAILED" -gt 0 ]; then
    printf 'FAILED: %s\n' "${FAILED_NAMES[@]}"
    printf 'artifacts: %s\n' "$TEST_TMP"
    exit 1
fi
if [ -n "${KEEP_TEST_TMP:-}" ]; then
    printf 'KEEP_TEST_TMP set; artifacts: %s\n' "$TEST_TMP"
fi
rm -rf "$TEST_TMP"
exit 0
