#!/bin/bash
# ============================================================================
# test_e2e_evidence_request_and_alert.sh — регресс-гард для двух helper'ов
# merge-gate, введённых ретро 2026-09-14 t_9580b71c (ADR-AF-0063+):
#
#   - pr_post_evidence_request(pr, source)
#       Постит «worker-evidence required» шаблон сразу после add-label
#       needs-review. Idempotent через EVIDENCE_REQUEST_DEDUP_HOURS (24h,
#       prefix-режим по маркеру «🤖 [merge-gate] **worker-evidence
#       required**»).
#
#   - needs_review_evidence_alert_pass_all()
#       Watchdog: сканирует OPEN PR с меткой needs-review старше
#       EVIDENCE_ALERT_AGE_HOURS (24h), у которых нет комментария с
#       EVIDENCE_REPORT_MARKER («worker-evidence report» — substring,
#       contains-режим). Для каждой такой PR постит alert-коммент
#       (🚨 [merge-gate watchdog] **evidence-missing**, prefix-режим,
#       24h dedup) + ставит label `evidence-missing` (idempotent).
#
# Сценарии (все через локальный mock-gh, без сети, без прод-файлов):
#   S1. alert fires — PR 25h, нет worker-evidence → 1 alert-comment + 1 add-label
#   S2. skip — evidence already present (worker-evidence report в комментах) → 0 alert, 0 add-label
#   S3. skip — under threshold (1h < 24h) → 0 alert, 0 add-label
#   S4. skip — dedup (alert-comment был < 24h назад) → 0 новых alert
#   S5. (мини) pr_post_evidence_request: первый вызов постит шаблон
#   S6. (мини) pr_post_evidence_request dedup: повторный в течение 24h → skip
#
# Контракт:
#   - EVIDENCE_* переменные экспортятся явно (тест не зависит от defaults).
#   - DRY_RUN=false (тестируем РЕАЛЬНЫЕ side-effects).
#   - mock-gh читает фикстуры из env (PR_LIST_NEEDS_REVIEW_JSON,
#     ISSUE_<n>_COMMENTS_JSON) и пишет лог вызовов (comment_log, edit_log).
#   - Все 6 сценариев прогоняются через sub-shell с PATH=$WORK/bin:...,
#     чтобы mock-gh перехватывал ВСЕ gh-вызовы (включая из _gh внутри
#     comment_recently_posted).
#   - Ассерты через assert_eq/assert_contains/assert_not_contains.
#
# Запуск (из корня репо):
#   bash scripts/agent_flow/tests/test_e2e_evidence_request_and_alert.sh
#
# Запуск из CI (см. .github/workflows/*):
#   bash scripts/agent_flow/tests/test_e2e_evidence_request_and_alert.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# tests/ → agent_flow/ → scripts/ → <repo>
MERGE_GATE_SH="${MERGE_GATE_SH:-$TEST_DIR/../agent-flow-merge-gate.sh}"
HERMES_GITHUB_SH="${HERMES_GITHUB_SH:-$TEST_DIR/../hermes_github.sh}"
LIB_COMMON_SH="${LIB_COMMON_SH:-$TEST_DIR/../lib_agent_flow_common.sh}"

[ -f "$MERGE_GATE_SH" ] || { echo "FAIL: $MERGE_GATE_SH not found"; exit 1; }
[ -f "$HERMES_GITHUB_SH" ] || { echo "FAIL: $HERMES_GITHUB_SH not found"; exit 1; }
[ -f "$LIB_COMMON_SH" ] || { echo "FAIL: $LIB_COMMON_SH not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required"; exit 1; }
command -v date >/dev/null || { echo "FAIL: date required"; exit 1; }

# ----------------------------------------------------------------------------
# Helpers (test-runner; не используют test_merge_gate_* helper, потому что
# этот тест полностью автономный — свой mock-gh, свой eval изолированных
# функций merge-gate). Ассерты заточены под стиль test_e2e_fail_streak_*.sh:
#   assert_eq expected actual msg
#   assert_contains needle haystack msg
#   assert_not_contains needle haystack msg
# ----------------------------------------------------------------------------
PASS=0
FAIL=0
FAILED_NAMES=()

assert_eq() {
    if [ "$1" != "$2" ]; then
        printf '  assert fail: %s\n    expected: %q\n    actual:   %q\n' "$3" "$1" "$2" >&2
        return 1
    fi
}

assert_contains() {
    case "$2" in
        *"$1"*) return 0 ;;
        *)
            printf '  assert fail: %s\n    needle:   %q\n    haystack: %q\n' "$3" "$1" "$2" >&2
            return 1
            ;;
    esac
}

assert_not_contains() {
    case "$2" in
        *"$1"*)
            printf '  assert fail: %s\n    needle should NOT appear: %q\n    haystack: %q\n' "$3" "$1" "$2" >&2
            return 1
            ;;
    esac
    return 0
}

# Счётчик ошибок внутри текущего run_test (обнуляется в начале).
_TEST_FAILS=0
_test_assert_failed() { _TEST_FAILS=$((_TEST_FAILS+1)); }

# Обёртки, которые накапливают fail-count вместо того, чтобы вернуть
# exit-code 1 (иначе bash-проблема: set -e внутри функции не срабатывает,
# а последний успешный echo перетирает exit code, и run_test видит PASS).
assert_eq_or_count() {
    assert_eq "$@" || _test_assert_failed
}
assert_contains_or_count() {
    assert_contains "$@" || _test_assert_failed
}
assert_not_contains_or_count() {
    assert_not_contains "$@" || _test_assert_failed
}

# run_test <name> <fn> — обёртка как в mock_env.sh; печатает RUN/PASS/FAIL.
# Запускает fn, после неё смотрит на _TEST_FAILS (накапливается через
# assert_*_or_count). Если > 0 — FAIL, иначе PASS.
run_test() {
    local name="$1" fn="$2"
    printf '── %s ──\n' "$name"
    _TEST_FAILS=0
    "$fn" || true
    if [ "$_TEST_FAILS" -gt 0 ]; then
        FAIL=$((FAIL+1))
        FAILED_NAMES+=("$name")
        printf '  FAIL (%d assertions)\n' "$_TEST_FAILS"
    else
        PASS=$((PASS+1))
        printf '  PASS\n'
    fi
}

# ----------------------------------------------------------------------------
# Извлечение нужных функций из прод-файлов.
# Используем тот же подход, что и test_merge_gate_evidence_alert_idempotent.sh:
# awk-блок от `^<name>()` до ближайшего `^}` (на верхнем уровне — функции
# однострочные, вроде has_label, либо многострочные с heredoc/python3 — все
# наши кандидаты укладываются в один блок без вложенных `^}`).
#
# ВАЖНО: extract_function ищет ПЕРВЫЙ блок, начинающийся с `^name()` (точное
# совпадение имени + открывающая скобка), и захватывает всё до первой
# standalone-строки `^}` (regex: строка начинается с `}` и не содержит других
# символов до конца). Это работает для всех наших целевых функций — ни одна
# из них не содержит блоков `}` на отдельной строке внутри.
# ----------------------------------------------------------------------------
extract_function() {
    local fname="$1" file="$2"
    # Захватываем функцию от `^fname() {` до первой строки `^}`.
    # awk ERE-quirk: `\\(` нужен для literal `(`, и `\\{` для literal `{`
    # (т.к. `{` в awk ERE — interval-operator).
    awk -v fn="$fname" '
        $0 ~ "^" fn "\\(\\) \\{" { p=1; print; next }
        p {
            print
            if ($0 ~ "^}$") { p=0 }
        }
    ' "$file"
}

LOG_BODY="$(extract_function 'log' "$MERGE_GATE_SH")"
HAS_LABEL_BODY="$(extract_function 'has_label' "$LIB_COMMON_SH")"
PR_POST_BODY="$(extract_function 'pr_post_evidence_request' "$MERGE_GATE_SH")"
NR_ALERT_BODY="$(extract_function 'needs_review_evidence_alert_pass_all' "$MERGE_GATE_SH")"
NOW_EPOCH_BODY="$(extract_function '_now_epoch' "$HERMES_GITHUB_SH")"
GH_API_WRAPPER_BODY="$(extract_function '_gh' "$HERMES_GITHUB_SH")"
COMMENT_RECENTLY_POSTED_BODY="$(extract_function 'comment_recently_posted' "$HERMES_GITHUB_SH")"

[ -n "$LOG_BODY" ] || { echo "FAIL: could not extract log() from $MERGE_GATE_SH"; exit 1; }
[ -n "$HAS_LABEL_BODY" ] || { echo "FAIL: could not extract has_label() from $LIB_COMMON_SH"; exit 1; }
[ -n "$PR_POST_BODY" ] || { echo "FAIL: could not extract pr_post_evidence_request() from $MERGE_GATE_SH"; exit 1; }
[ -n "$NR_ALERT_BODY" ] || { echo "FAIL: could not extract needs_review_evidence_alert_pass_all() from $MERGE_GATE_SH"; exit 1; }
[ -n "$NOW_EPOCH_BODY" ] || { echo "FAIL: could not extract _now_epoch() from $HERMES_GITHUB_SH"; exit 1; }
[ -n "$GH_API_WRAPPER_BODY" ] || { echo "FAIL: could not extract _gh() from $HERMES_GITHUB_SH"; exit 1; }
[ -n "$COMMENT_RECENTLY_POSTED_BODY" ] || { echo "FAIL: could not extract comment_recently_posted() from $HERMES_GITHUB_SH"; exit 1; }

# Eval в текущем shell + export -f, чтобы функции были доступны в sub-shell
# теста (run_watchdog / run_evidence_request запускаются в `(...)`, и без
# export -f они получат только env-переменные, но НЕ функции).
eval "$LOG_BODY"
eval "$HAS_LABEL_BODY"
eval "$NOW_EPOCH_BODY"
eval "$GH_API_WRAPPER_BODY"
eval "$COMMENT_RECENTLY_POSTED_BODY"
eval "$PR_POST_BODY"
eval "$NR_ALERT_BODY"
# shellcheck disable=SC2046
export -f log has_label _now_epoch _gh comment_recently_posted \
    pr_post_evidence_request needs_review_evidence_alert_pass_all 2>/dev/null || true

# ----------------------------------------------------------------------------
# WORK: tmp-директория под mock-gh + фикстуры.
# ----------------------------------------------------------------------------
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/bin"

# ----------------------------------------------------------------------------
# Mock-gh: понимает ТОЛЬКО команды, нужные двум тестируемым функциям.
# Дизайн — env-var based (как test_e2e_fail_streak_auto_issue.sh):
#   MOCK_GH_AUTH_OK=0        — auth status → ok
#   MOCK_PRS_FILE=<path>     — фикстура для `gh pr list --json ...`
#   MOCK_ISSUE_<N>_FILE=<path> — фикстура для `gh api .../issues/<N>/comments`
#
# Mock пишет в логи (для assert_eq):
#   $WORK/comment_log — каждая строка: "pr_comment <PR> <первая-строка-body>"
#   $WORK/edit_log   — каждая строка: "pr_edit <full args>"
#   $WORK/gh_call_log — каждая строка: "<subcmd> <args...>"
#
# Парсинг: helper `_strip_argv` собирает "логические" позиционные аргументы,
# пропуская пары --flag <value> (--repo, --label, --state, --limit,
# --json, --assignee, --title, --body, --base, --per-page, --jq, -q, -X).
# После strip'а gh --repo X api path → ["api", "path"].
# ----------------------------------------------------------------------------
cat > "$WORK/bin/gh" <<'GH_EOF'
#!/bin/bash
# Strip argv: убираем пары --flag value (используется только для определения
# subcommand'а; реальные значения берём из $@ когда нужно).
_args=("$@")
_clean=()
_skip=0
for a in "${_args[@]}"; do
    if [ "$_skip" = "1" ]; then
        _skip=0
        continue
    fi
    case "$a" in
        --repo|--label|--state|--limit|--json|--workflow|-w) _skip=1 ;;
        --assignee|--title|--body|--base|--per-page) _skip=1 ;;
        --jq|-q|-f|-X) _skip=1 ;;
        *) _clean+=("$a") ;;
    esac
done

_work="${WORK:-}"
_call_log="${_work}/gh_call_log"
_comment_log="${_work}/comment_log"
_edit_log="${_work}/edit_log"

_log_call() {
    [ -n "$_call_log" ] && printf '%s %s\n' "$1" "$*" >> "$_call_log"
}

case "${_clean[0]:-} ${_clean[1]:-}" in
    "auth status")
        if [ "${MOCK_GH_AUTH_OK:-0}" = "0" ]; then echo "logged in"; exit 0; fi
        echo "not logged in" >&2; exit 1 ;;

    "pr list"*)
        _log_call "pr_list"
        if [ -n "${MOCK_PRS_FILE:-}" ] && [ -f "${MOCK_PRS_FILE}" ]; then
            cat "${MOCK_PRS_FILE}"
        else
            echo '[]'
        fi
        exit 0 ;;

    "pr comment"*)
        _log_call "pr_comment"
        # PR-number — позиционный аргумент сразу после subcommand "comment".
        _pr=""
        for ((i=0; i<${#_clean[@]}; i++)); do
            if [ "${_clean[$i]}" = "pr" ] && [ "${_clean[$((i+1))]:-}" = "comment" ]; then
                _pr="${_clean[$((i+2))]:-}"
                break
            fi
        done
        # --body через _args (полный argv).
        _body=""
        for ((i=0; i<${#_args[@]}; i++)); do
            case "${_args[$i]}" in
                --body) _body="${_args[$((i+1))]:-}"; break ;;
            esac
        done
        _marker="$(printf '%s' "$_body" | head -n1)"
        [ -n "$_comment_log" ] && printf 'pr_comment %s %s\n' "$_pr" "$_marker" >> "$_comment_log"
        # Сохраняем полный body для детальной проверки.
        [ -n "$_pr" ] && printf '%s\n' "$_body" > "${_work}/comment_${_pr}.body"
        # Если есть фикстура комментов для этой PR — дополним её (имитация,
        # что GitHub теперь видит этот коммент). Это нужно для S6:
        # после первого pr_post_evidence_request второй вызов должен найти
        # свежий коммент и skip'нуть.
        _fixture_var="MOCK_ISSUE_${_pr}_FILE"
        _fixture="${!_fixture_var:-}"
        if [ -n "$_fixture" ] && [ -f "$_fixture" ] && [ -n "$_body" ]; then
            # Append через python с явным stdin (НЕ $_cur как скрипт — это баг).
            python3 - "$_fixture" "$_body" <<'PYEOF' 2>/dev/null || true
import json, sys
fp, body = sys.argv[1], sys.argv[2]
try:
    with open(fp) as f:
        data = json.load(f)
    if not isinstance(data, list):
        data = []
except Exception:
    data = []
data.append({"id": len(data) + 1, "body": body, "created_at": "1970-01-01T00:00:00Z", "user": {"login": "test"}})
with open(fp, "w") as f:
    json.dump(data, f)
PYEOF
        fi
        rc="${MOCK_GH_PR_COMMENT_RC:-0}"
        if [ "$rc" != "0" ]; then echo "ERROR: simulated gh pr comment failure (rc=$rc)" >&2; fi
        exit "$rc" ;;

    "pr edit"*)
        _log_call "pr_edit"
        [ -n "$_edit_log" ] && printf 'pr_edit %s\n' "$*" >> "$_edit_log"
        exit 0 ;;

    "api"*)
        _log_call "api"
        # Path — _clean[1] (после strip'а пары --repo X).
        _path="${_clean[1]:-}"
        # Извлекаем issue number из path вида
        # repos/<owner>/<repo>/issues/<N>/comments?since=...&per_page=100
        _n=""
        case "$_path" in
            repos/*/issues/*/comments*) _n="$(printf '%s' "$_path" | sed -nE 's#.*/issues/([0-9]+)/comments.*#\1#p')" ;;
        esac
        if [ -z "$_n" ]; then echo '[]'; exit 0; fi
        _fixture_var="MOCK_ISSUE_${_n}_FILE"
        _fixture="${!_fixture_var:-}"
        if [ -n "$_fixture" ] && [ -f "$_fixture" ]; then
            cat "$_fixture"
        else
            echo '[]'
        fi
        exit 0 ;;

    *)
        echo "{\"error\":\"unmocked gh subcommand: ${_clean[*]:-}\"}" >&2
        exit 2 ;;
esac
GH_EOF
chmod +x "$WORK/bin/gh"

# ----------------------------------------------------------------------------
# Helpers: фикстуры + runner.
# ----------------------------------------------------------------------------

# fixture_prs <prN> <age_hours> [labels_csv] — генерирует JSON-массив с одним
# PR, где updatedAt = (now - age_hours). labels = "$3" (lowercase CSV, по
# умолчанию "needs-review").
fixture_prs() {
    local pr_n="$1" age_h="$2" labels_csv="${3:-needs-review}"
    local out="${WORK}/prs_${pr_n}.json"
    local updated
    updated="$(date -u -d "@$(($(date -u +%s) - age_h * 3600))" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
    python3 - "$pr_n" "$updated" "$labels_csv" > "$out" <<'PYEOF'
import json, sys
pr_n, updated, labels_csv = sys.argv[1], sys.argv[2], sys.argv[3]
labels = [{"name": x.strip()} for x in labels_csv.split(",") if x.strip()]
pr = [{
    "number": int(pr_n),
    "createdAt": "2026-01-01T00:00:00Z",
    "updatedAt": updated,
    "title": "test PR #" + pr_n,
    "headRefName": "z-{agent}/t_test-pr-" + pr_n,
    "labels": labels,
}]
print(json.dumps(pr))
PYEOF
    export MOCK_PRS_FILE="$out"
}

# fixture_comments <prN> <age_hours> <body> — пишет массив из одного
# комментария с заданным body и createdAt = (now - age_hours).
fixture_comments() {
    local pr_n="$1" age_h="$2" body="$3"
    local out="${WORK}/comments_${pr_n}.json"
    local created
    created="$(date -u -d "@$(($(date -u +%s) - age_h * 3600))" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
    python3 - "$created" "$body" > "$out" <<'PYEOF'
import json, sys
created, body = sys.argv[1], sys.argv[2]
print(json.dumps([{"id": 1, "body": body, "created_at": created, "user": {"login": "test"}}]))
PYEOF
    local var_name="MOCK_ISSUE_${pr_n}_FILE"
    export "${var_name}=${out}"
}

# clear_calls — обнулить логи и tmp-файлы перед прогоном.
clear_calls() {
    : > "${WORK}/comment_log"
    : > "${WORK}/edit_log"
    : > "${WORK}/gh_call_log"
    rm -f "${WORK}"/comment_*.body
}

# Запускаем с export MOCK_* + EVIDENCE_* (тест не зависит от defaults).
run_watchdog() {
    local tag="$1" pr_n="$2"
    local stderr_f="${WORK}/stderr_${tag}.log"
    (
        PATH="${WORK}/bin:/usr/bin:/bin"
        GH_REPO="krikz/test-repo"
        GH_CONFIG_DIR="${WORK}/gh-config"
        DEVELOP_BRANCH="develop"
        NEEDS_REVIEW_LABEL="needs-review"
        EVIDENCE_MISSING_LABEL="evidence-missing"
        EVIDENCE_REPORT_MARKER="worker-evidence report"
        EVIDENCE_REQUEST_DEDUP_HOURS="24"
        EVIDENCE_ALERT_AGE_HOURS="24"
        EVIDENCE_ALERT_DEDUP_HOURS="24"
        DRY_RUN="false"
        LOG_PREFIX="[test]"
        # WORK нужен mock-gh для путей к логам; основной shell объявил,
        # но в sub-shell без export не пробрасывается. Экспортируем явно.
        export WORK
        export MOCK_PRS_FILE
        local v
        for v in $(env | grep -E '^MOCK_ISSUE_[0-9]+_FILE=' | cut -d= -f1); do
            export "$v"
        done
        export MOCK_GH_AUTH_OK MOCK_GH_PR_COMMENT_RC
        needs_review_evidence_alert_pass_all
    ) 2> "$stderr_f"
    echo "$stderr_f"
}

run_evidence_request() {
    local tag="$1" pr_n="$2" source="$3"
    local stderr_f="${WORK}/stderr_${tag}.log"
    (
        PATH="${WORK}/bin:/usr/bin:/bin"
        GH_REPO="krikz/test-repo"
        GH_CONFIG_DIR="${WORK}/gh-config"
        DEVELOP_BRANCH="develop"
        NEEDS_REVIEW_LABEL="needs-review"
        EVIDENCE_MISSING_LABEL="evidence-missing"
        EVIDENCE_REPORT_MARKER="worker-evidence report"
        EVIDENCE_REQUEST_DEDUP_HOURS="24"
        EVIDENCE_ALERT_AGE_HOURS="24"
        EVIDENCE_ALERT_DEDUP_HOURS="24"
        DRY_RUN="false"
        LOG_PREFIX="[test]"
        export WORK
        export MOCK_PRS_FILE
        local v
        for v in $(env | grep -E '^MOCK_ISSUE_[0-9]+_FILE=' | cut -d= -f1); do
            export "$v"
        done
        export MOCK_GH_AUTH_OK MOCK_GH_PR_COMMENT_RC
        pr_post_evidence_request "$pr_n" "$source"
    ) 2> "$stderr_f"
    echo "$stderr_f"
}

# Reset state for a clean run.
reset_run() {
    clear_calls
    rm -f "${WORK}"/stderr_*.log
}

# ----------------------------------------------------------------------------
# Сценарии (каждый — отдельная функция, вызывается через run_test).
# ----------------------------------------------------------------------------

# S1: alert fires
# PR open, needs-review label, updatedAt 25ч назад, нет комментов с
# worker-evidence → watchdog должен:
#   - запостить alert-comment (1 вызов gh pr comment)
#   - поставить label evidence-missing (1 вызов gh pr edit --add-label)
test_S1_alert_fires() {
    reset_run
    fixture_prs 1843 25
    # Нет fixture_comments → env не задан → mock вернёт '[]' (пусто).
    unset MOCK_ISSUE_1843_FILE
    local log
    log="$(run_watchdog s1 1843)"

    # 1 alert-comment ожидаем.
    local comments
    comments="$(grep -c '^pr_comment 1843 ' "${WORK}/comment_log" || true)"
    assert_eq_or_count "1" "$comments" "S1: ровно 1 pr_comment на PR #1843"

    # В теле должен быть маркер alert'а.
    local body
    body="$(cat "${WORK}/comment_1843.body" 2>/dev/null || echo '')"
    assert_contains_or_count "🚨 [merge-gate watchdog] **evidence-missing**" "$body" "S1: body содержит marker alert"

    # 1 add-label evidence-missing.
    local edits
    edits="$(grep -c '^pr_edit .* --add-label evidence-missing' "${WORK}/edit_log" || true)"
    assert_eq_or_count "1" "$edits" "S1: ровно 1 pr_edit --add-label evidence-missing"

    # В логе watchdog — "posted alert" + "+evidence-missing" сообщения.
    assert_contains_or_count "posted alert on PR #1843" "$(cat "$log")" "S1: лог содержит posted alert"
    assert_contains_or_count "+evidence-missing" "$(cat "$log")" "S1: лог содержит +evidence-missing"
}

# S2: skip — evidence already present
# PR open, needs-review, 25ч назад, есть комментарий с подстрокой
# «worker-evidence report» → watchdog НЕ должен постить alert и НЕ
# должен ставить label.
test_S2_skip_evidence_present() {
    reset_run
    fixture_prs 1844 25
    fixture_comments 1844 100 "## worker-evidence report
SHA: abcdef0
task: t_test
Acceptance: OK"
    local log
    log="$(run_watchdog s2 1844)"

    local comments
    comments="$(grep -c '^pr_comment ' "${WORK}/comment_log" || true)"
    assert_eq_or_count "0" "$comments" "S2: 0 alert-comment (evidence уже есть)"

    local edits
    edits="$(grep -c '^pr_edit ' "${WORK}/edit_log" || true)"
    assert_eq_or_count "0" "$edits" "S2: 0 add-label (evidence уже есть)"

    # В логе — сообщение о skip.
    assert_contains_or_count "worker-evidence уже рапортован" "$(cat "$log")" "S2: лог содержит skip-маркер"
    assert_not_contains_or_count "posted alert" "$(cat "$log")" "S2: лог НЕ должен содержать 'posted alert'"
}

# S3: skip — under threshold
# PR open, needs-review, 1ч назад (< EVIDENCE_ALERT_AGE_HOURS=24) →
# watchdog НЕ должен постить alert и НЕ должен ставить label.
test_S3_skip_under_threshold() {
    reset_run
    fixture_prs 1845 1
    unset MOCK_ISSUE_1845_FILE
    local log
    log="$(run_watchdog s3 1845)"

    local comments
    comments="$(grep -c '^pr_comment ' "${WORK}/comment_log" || true)"
    assert_eq_or_count "0" "$comments" "S3: 0 alert-comment (PR свежий)"

    local edits
    edits="$(grep -c '^pr_edit ' "${WORK}/edit_log" || true)"
    assert_eq_or_count "0" "$edits" "S3: 0 add-label (PR свежий)"

    # В логе — сообщение о scan + ничего про alert.
    assert_contains_or_count "scanning OPEN PRs with needs-review older than" "$(cat "$log")" "S3: лог содержит scan-message"
    assert_not_contains_or_count "posted alert" "$(cat "$log")" "S3: лог НЕ должен содержать 'posted alert'"
    assert_not_contains_or_count "+evidence-missing" "$(cat "$log")" "S3: лог НЕ должен содержать '+evidence-missing'"
}

# S4: skip — dedup (alert был < 24h назад)
# PR open, needs-review, 25ч назад, + в истории watchdog-alert-comment
# (свежий, < 24ч назад) + label evidence-missing уже стоит → watchdog
# должен skip новый alert AND не дублировать label.
test_S4_skip_dedup_recent_alert() {
    reset_run
    # PR уже имеет И needs-review И evidence-missing label (как после первого
    # watchdog-прохода). Watchdog должен skip новый alert-comment И не дублировать
    # label — он уже на месте.
    fixture_prs 1846 25 "needs-review,evidence-missing"
    # Свежий (1ч назад) alert-comment от watchdog'а — имитирует, что мы
    # ALREADY рапортовали watchdog'ом этот PR.
    fixture_comments 1846 1 "🚨 [merge-gate watchdog] **evidence-missing** (t_9580b71c)

PR \`#1846\` висит..."
    local log
    log="$(run_watchdog s4 1846)"

    local comments
    comments="$(grep -c '^pr_comment ' "${WORK}/comment_log" || true)"
    assert_eq_or_count "0" "$comments" "S4: 0 новых alert-comment (dedup)"

    local edits
    edits="$(grep -c '^pr_edit ' "${WORK}/edit_log" || true)"
    assert_eq_or_count "0" "$edits" "S4: 0 add-label (label уже стоит — dedup через has_label)"

    # В логе — сообщение о dedup-skip.
    assert_contains_or_count "alert уже был в" "$(cat "$log")" "S4: лог содержит dedup-skip маркер"
}

# S5 (мини): pr_post_evidence_request — первый вызов
# Нет предыдущих комментов → должен постить worker-evidence required.
test_S5_evidence_request_first_call() {
    reset_run
    unset MOCK_ISSUE_1847_FILE
    local log
    log="$(run_evidence_request s5 1847 "clean-pr-sweep lint")"

    local comments
    comments="$(grep -c '^pr_comment 1847 ' "${WORK}/comment_log" || true)"
    assert_eq_or_count "1" "$comments" "S5: 1 pr_comment на PR #1847"

    local body
    body="$(cat "${WORK}/comment_1847.body" 2>/dev/null || echo '')"
    assert_contains_or_count "🤖 [merge-gate] **worker-evidence required**" "$body" "S5: body содержит worker-evidence required marker"
    assert_contains_or_count "(clean-pr-sweep lint)" "$body" "S5: body содержит source в маркере"

    # В логе — posted.
    assert_contains_or_count "posted on PR #1847" "$(cat "$log")" "S5: лог содержит posted"
}

# S6 (мини): pr_post_evidence_request dedup
# Второй вызов в течение EVIDENCE_REQUEST_DEDUP_HOURS (24ч) — должен
# skip (comment_recently_posted с prefix-режимом находит свежий маркер).
test_S6_evidence_request_dedup() {
    reset_run
    unset MOCK_ISSUE_1848_FILE
    # Первый вызов — постит шаблон (как в S5).
    run_evidence_request s6a 1848 "clean-pr-sweep e2e-impossible" > /dev/null
    local first
    first="$(grep -c '^pr_comment 1848 ' "${WORK}/comment_log" || true)"
    assert_eq_or_count "1" "$first" "S6: первый вызов — 1 pr_comment"

    # mock-gh уже дополнил фикстуру записью (created_at=1970-01-01), но
    # comment_recently_posted смотрит на created_at и свежим считает только
    # >= cutoff (now - 24h). 1970-01-01 — за пределами окна.
    # Поэтому ставим свежий (1ч назад) комментарий с маркером поверх фикстуры.
    fixture_comments 1848 1 "🤖 [merge-gate] **worker-evidence required** (clean-pr-sweep e2e-impossible)

First request was posted. Please respond."

    clear_calls
    # Второй вызов — должен skip.
    local log
    log="$(run_evidence_request s6b 1848 "clean-pr-sweep e2e-impossible")"
    local second
    second="$(grep -c '^pr_comment ' "${WORK}/comment_log" || true)"
    assert_eq_or_count "0" "$second" "S6: второй вызов в течение 24h — skip"

    # В логе — сообщение о skip.
    assert_contains_or_count "worker-evidence request уже был в 24h, skip" "$(cat "$log")" "S6: лог содержит skip-маркер"
}

# ----------------------------------------------------------------------------
# Прогон
# ----------------------------------------------------------------------------
run_test "S1_alert_fires"               test_S1_alert_fires
run_test "S2_skip_evidence_present"      test_S2_skip_evidence_present
run_test "S3_skip_under_threshold"       test_S3_skip_under_threshold
run_test "S4_skip_dedup_recent_alert"    test_S4_skip_dedup_recent_alert
run_test "S5_evidence_request_first"    test_S5_evidence_request_first_call
run_test "S6_evidence_request_dedup"     test_S6_evidence_request_dedup

echo ""
echo "=== Total: $((PASS+FAIL)) / Passed: $PASS / Failed: $FAIL ==="
if [ "$FAIL" -gt 0 ]; then
    echo "Failed scenarios:"
    for n in "${FAILED_NAMES[@]}"; do echo "  - $n"; done
    exit 1
fi
exit 0
