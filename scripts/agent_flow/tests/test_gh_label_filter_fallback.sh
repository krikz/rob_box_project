#!/bin/bash
# ============================================================================
# test_gh_label_filter_fallback.sh — bug(process) #1457
#
# Регрессия: `gh issue list --label X` на некоторых версиях gh CLI (2.x.x)
# возвращает пустой массив даже когда есть открытые issues с меткой X.
# При этом `gh api repos/.../issues?labels=X&state=open` возвращает корректный
# список. gh_list_issues_by_label (ретро 19.08 #1457) делает fallback на REST
# API при пустом ответе gh-list.
#
# Сценарии (для каждой функции в каждом из 4 скриптов):
#   A. gh-list возвращает НЕпустой массив         → используется gh-list,
#                                                   fallback НЕ срабатывает
#   B. gh-list возвращает [] (баг CLI), REST — N  → fallback → N issues
#                                                   в выводе функции
#   C. gh-list [], REST [] (действительно пусто)  → функция возвращает []
#                                                   (для downstream это
#                                                   эквивалентно «нет issues»)
#
# Скрипты под проверкой:
#   - agent-flow-e2e-process.sh     (collect_issues_json needs-e2e)
#   - agent-flow-triage.sh          (issues_json ISSUE_LABEL=hermes)
#   - agent-flow-merge-gate.sh      (issues_json ISSUE_LABEL=hermes,
#                                    deploy_issue_reconcile_all)
#   - agent-flow-deploy-sweep.sh    (issues_json deployment)
#
# Run:
#   bash scripts/agent_flow/tests/test_gh_label_filter_fallback.sh
# ============================================================================
set -euo pipefail

TEST_FILE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_LIBS_DIR="$(cd "$TEST_FILE_DIR/lib" && pwd)"
# shellcheck source=lib/mock_env.sh
. "$TEST_FILE_DIR/lib/mock_env.sh"
# shellcheck source=lib/lib_eval_func.sh
. "$TEST_LIBS_DIR/lib_eval_func.sh"

# NOTE: mock_env.sh переустанавливает TEST_LIB_DIR → tests/lib, поэтому
# вычисляем пути ДО source (на базе TEST_FILE_DIR = tests/) и сохраняем
# в отдельные переменные, которые mock_env.sh не трогает.
REPO_ROOT_DIR="$(cd "$TEST_FILE_DIR/../.." && pwd)"
AGENT_FLOW_DIR="$(cd "$TEST_FILE_DIR/.." && pwd)"
E2E_PROCESS="$AGENT_FLOW_DIR/agent-flow-e2e-process.sh"
TRIAGE="$AGENT_FLOW_DIR/agent-flow-triage.sh"
MERGE_GATE="$AGENT_FLOW_DIR/agent-flow-merge-gate.sh"
DEPLOY_SWEEP="$AGENT_FLOW_DIR/agent-flow-deploy-sweep.sh"

# --- A. gh-list непустой → fallback НЕ срабатывает --------------------------
test_A_gh_list_nonempty_uses_gh_list() {
    new_test
    install_mocks

    # gh_list не пустой → helper использует gh-list и НЕ ходит в REST.
    local _fake_ghlist='[{"number":1392,"title":"issue 1392","labels":[{"name":"needs-e2e"}],"body":""}]'
    set_state ISSUE_LIST_JSON "$_fake_ghlist"

    local _out
    _out="$(eval_helper "$E2E_PROCESS" gh_list_issues_by_label needs-e2e open 20 2>/dev/null || true)"

    local _cnt
    _cnt="$(printf '%s' "$_out" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))')"
    assert_eq "1" "$_cnt" "A: gh-list непустой → 1 issue"

    local _journal
    _journal="$(cat "$GH_JOURNAL" 2>/dev/null || true)"
    assert_contains "gh issue list --label" "$_journal" "A: gh-list вызван"
    assert_not_contains "fallback на REST" "$_journal" "A: fallback НЕ сработал"
}

# --- B. gh-list [] (баг) + REST непустой → fallback ------------------------
test_B_gh_list_empty_falls_back_to_rest() {
    new_test
    install_mocks

    # gh_list возвращает []; REST — фикстура из 1 issue.
    set_state ISSUE_LIST_JSON '[]'
    set_state REST_ISSUES_BY_LABEL_JSON '[{"number":1392,"title":"issue 1392","labels":[{"name":"needs-e2e"}],"body":"","updatedAt":"2026-08-19T00:00:00Z"}]'

    # Мок gh пишет каждый вызов в GH_JOURNAL (см. install_mocks ниже). Это
    # надёжный способ проверить, что функция сначала попробовала gh-list,
    # и только потом свалилась в REST. Сохраняем stderr отдельно для
    # проверки log()-сообщения о fallback.
    local _out _err _journal
    _out="$(eval_helper "$E2E_PROCESS" gh_list_issues_by_label needs-e2e open 20 2>"$TEST_TMP/stderr.log" || true)"
    _err="$(cat "$TEST_TMP/stderr.log" 2>/dev/null || true)"
    _journal="$(cat "$GH_JOURNAL" 2>/dev/null || true)"

    local _cnt
    _cnt="$(printf '%s' "$_out" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))')"
    assert_eq "1" "$_cnt" "B: fallback → 1 issue (REST)"

    assert_contains "gh issue list --label" "$_journal" "B: gh-list вызван (первая попытка)"
    assert_contains "/issues?labels=needs-e2e" "$_journal" "B: REST API вызван в fallback"
    assert_contains "fallback на REST" "$_err" "B: log() сообщил о fallback"
}

# --- C. оба пусты → функция возвращает [] ------------------------------------
test_C_both_empty_returns_empty_array() {
    new_test
    install_mocks

    set_state ISSUE_LIST_JSON '[]'
    set_state REST_ISSUES_BY_LABEL_JSON '[]'

    local _out
    _out="$(eval_helper "$E2E_PROCESS" gh_list_issues_by_label needs-e2e open 20 2>/dev/null || true)"

    assert_eq "[]" "$_out" "C: оба пути пусты → []"

    local _journal
    _journal="$(cat "$GH_JOURNAL" 2>/dev/null || true)"
    assert_not_contains "fallback на REST" "$_journal" "C: fallback логируется только когда REST непустой"
}

# --- D. triage.sh helper ----------------------------------------------------
test_D_triage_helper_falls_back() {
    new_test
    install_mocks

    set_state ISSUE_LIST_JSON '[]'
    set_state REST_ISSUES_BY_LABEL_JSON '[{"number":1392,"title":"issue 1392","labels":[{"name":"hermes"}],"body":""}]'

    local _out
    _out="$(eval_helper "$TRIAGE" gh_list_issues_by_label hermes open 20 2>/dev/null || true)"

    local _cnt
    _cnt="$(printf '%s' "$_out" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))')"
    assert_eq "1" "$_cnt" "D: triage helper → 1 issue (REST fallback)"
}

# --- E. merge-gate.sh helper (с updatedAt) ----------------------------------
test_E_merge_gate_helper_falls_back() {
    new_test
    install_mocks

    set_state ISSUE_LIST_JSON '[]'
    set_state REST_ISSUES_BY_LABEL_JSON '[{"number":1392,"title":"issue 1392","labels":[{"name":"hermes"}],"body":"","updatedAt":"2026-08-19T00:00:00Z"}]'

    local _out
    _out="$(eval_helper "$MERGE_GATE" gh_list_issues_by_label hermes open 20 2>/dev/null || true)"

    local _has_updated
    _has_updated="$(printf '%s' "$_out" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
    print("1" if any("updatedAt" in it for it in d) else "0")
except Exception:
    print("0")')"
    assert_eq "1" "$_has_updated" "E: merge-gate helper → REST-fallback с updatedAt"
}

# --- F. deploy-sweep.sh helper -----------------------------------------------
test_F_deploy_sweep_helper_falls_back() {
    new_test
    install_mocks

    set_state ISSUE_LIST_JSON '[]'
    set_state REST_ISSUES_BY_LABEL_JSON '[{"number":1392,"title":"issue 1392","labels":[{"name":"deployment"}],"body":"","updatedAt":"2026-08-19T00:00:00Z"}]'

    local _out
    _out="$(eval_helper "$DEPLOY_SWEEP" gh_list_issues_by_label deployment open 20 2>/dev/null || true)"

    local _cnt
    _cnt="$(printf '%s' "$_out" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))')"
    assert_eq "1" "$_cnt" "F: deploy-sweep helper → 1 issue (REST fallback)"
}

# --- G. REST отдаёт updated_at (snake_case) → на выходе updatedAt ----------
# Баг, живший во всех четырёх копиях helper'а до дедупа 30.08: маппинг
# REST→gh-list делал `if "updatedAt" in it`, а GitHub REST отдаёт
# `updated_at`. Условие не срабатывало никогда, и на fallback-пути поле
# терялось. deploy-sweep читает его жёстко (`str(i["updatedAt"])`) внутри
# `< <(python3 ...)` → KeyError, ноль обработанных issue и НЕизменившийся
# exit-код тика, то есть отказ «в тишине».
test_G_rest_snake_case_updated_at() {
    new_test
    install_mocks

    set_state ISSUE_LIST_JSON '[]'
    set_state REST_ISSUES_BY_LABEL_JSON '[{"number":1392,"title":"issue 1392","labels":[{"name":"deployment"}],"body":"","updated_at":"2026-08-19T00:00:00Z"}]'

    local _out
    _out="$(eval_helper "$DEPLOY_SWEEP" gh_list_issues_by_label deployment open 20 2>/dev/null || true)"

    local _updated
    _updated="$(printf '%s' "$_out" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
    print(d[0].get("updatedAt", ""))
except Exception:
    print("")')"
    assert_eq "2026-08-19T00:00:00Z" "$_updated" "G: REST updated_at → updatedAt на выходе"
}

# ============================================================================
# Override install_mocks: добавим gh api обработку /repos/.../issues?labels=
# которая возвращает фикстуру REST_ISSUES_BY_LABEL_JSON.
# ============================================================================
install_mocks() {
    local bin_dir="$TEST_TMP/bin"
    mkdir -p "$bin_dir"

    # Mock `gh` ---------------------------------------------------------
    cat > "$bin_dir/gh" <<'GH_MOCK_EOF'
#!/bin/bash
state="${GH_STATE:-}"
journal="${GH_JOURNAL:-/dev/null}"
ts="$(date -Iseconds 2>/dev/null || date)"
journal() { printf '%s\t%s\n' "$ts" "$*" >>"$journal"; }
get_state() {
    local key="$1"
    if [ -f "$state" ]; then
        grep -E "^${key}=" "$state" | head -n1 | sed "s@^${key}=@@"
    fi
}

subcmd="${1:-}"; shift || true

case "$subcmd" in
    auth)
        journal "gh auth status"
        exit 0
        ;;
    issue)
        action="${1:-}"; shift || true
        case "$action" in
            list)
                if printf '%s' "$*" | grep -q -- '--label'; then
                    journal "gh issue list --label"
                    _data="$(get_state ISSUE_LIST_JSON)"
                    [ -n "$_data" ] || _data='[]'
                    printf '%s' "$_data"
                    exit 0
                fi
                journal "gh issue list (other)"
                printf '%s' '[]'
                exit 0
                ;;
            view|edit|comment|close)
                journal "gh issue $action $*"
                printf '%s' '{}'
                exit 0
                ;;
            *)
                journal "gh issue $action $*"
                printf '%s' '{}'
                exit 0
                ;;
        esac
        ;;
    api)
        journal "gh api $*"
        # Ретро 19.08 #1457: gh_list_issues_by_label ходит в REST API
        # /repos/.../issues?labels=X при пустом gh-list. Возвращаем
        # фикстуру REST_ISSUES_BY_LABEL_JSON.
        if printf '%s' "$*" | grep -qE '/issues\?labels='; then
            _data="$(get_state REST_ISSUES_BY_LABEL_JSON)"
            [ -n "$_data" ] || _data='[]'
            printf '%s' "$_data"
            exit 0
        fi
        if printf '%s' "$*" | grep -q 'rate_limit'; then
            printf '%s' '{"resources":{"core":{"remaining":5000},"graphql":{"remaining":5000}}}'
            exit 0
        fi
        printf '%s' '{}'
        exit 0
        ;;
    label|run|pr|workflow)
        journal "gh $subcmd $*"
        printf '%s' '[]'
        exit 0
        ;;
    *)
        journal "gh $subcmd $*"
        printf '%s' '{}'
        exit 0
        ;;
esac
GH_MOCK_EOF
    chmod +x "$bin_dir/gh"
    # Мок git/hermes для совместимости (многие скрипты делают source и
    # позднее могут позвать git/hermes — пусть не падают).
    cat > "$bin_dir/git" <<'GIT_MOCK_EOF'
#!/bin/bash
echo "{}"
GIT_MOCK_EOF
    chmod +x "$bin_dir/git"
    cat > "$bin_dir/hermes" <<'HERMES_MOCK_EOF'
#!/bin/bash
echo "{}"
HERMES_MOCK_EOF
    chmod +x "$bin_dir/hermes"
    export PATH="$bin_dir:$PATH"
}

new_test() {
    # Изолированный per-test temp + state.
    TEST_TMP="${TEST_TMP:-/tmp/gh-label-filter-fallback.$$}"
    TEST_TMP="${TEST_TMP}.${RANDOM}"
    mkdir -p "$TEST_TMP/bin"
    GH_STATE="$TEST_TMP/gh_state"
    GH_JOURNAL="$TEST_TMP/journal"
    : >"$GH_STATE"
    : >"$GH_JOURNAL"
    export TEST_TMP GH_STATE GH_JOURNAL
}

set_state() {
    local key="$1" val="$2"
    # Удалим старый ключ (если есть), добавим новый в конец (важно для
    # порядка — get_state берёт последний).
    if [ -f "$GH_STATE" ]; then
        grep -v "^${key}=" "$GH_STATE" >"$GH_STATE.tmp" || true
        mv "$GH_STATE.tmp" "$GH_STATE"
    fi
    printf '%s=%s\n' "$key" "$val" >>"$GH_STATE"
}

run_test "A. gh-list непустой → fallback НЕ сработал" test_A_gh_list_nonempty_uses_gh_list
run_test "B. gh-list [] (баг) → fallback на REST" test_B_gh_list_empty_falls_back_to_rest
run_test "C. оба пусты → []" test_C_both_empty_returns_empty_array
run_test "D. triage.sh helper → fallback" test_D_triage_helper_falls_back
run_test "E. merge-gate.sh helper (updatedAt) → fallback" test_E_merge_gate_helper_falls_back
run_test "F. deploy-sweep.sh helper → fallback" test_F_deploy_sweep_helper_falls_back
run_test "G. REST updated_at (snake_case) → updatedAt" test_G_rest_snake_case_updated_at

summary
