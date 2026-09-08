#!/bin/bash
# ============================================================================
# test_kanban_report_write.sh — регресс-тест для kanban-report-write.sh
# (issue #2159, ADR-0077: kanban worker report file).
#
# Сценарии:
#   A) Генерация с дефолтами (без --title/--assignee) → exit 0, файл есть,
#      содержит task_id, assignee=devops, "Started: _<поставить..."
#   B) С --title и --assignee → exit 0, в шапке корректные значения
#   C) С --pr и --issue → exit 0, в шапке ссылки на PR/Issue
#   D) --output в нестандартный путь → exit 0, файл по этому пути
#   E) Без TASK_ID → exit 1 (usage error)
#   F) Не в git worktree (TMPDIR без .git) → exit 2
#   G) BASE_REF недоступен → exit 0, fallback на HEAD~1..HEAD, без краша
#   H) Идемпотентность: повторный запуск перезаписывает файл
#
# Exit codes:
#   0 — все сценарии прошли
#   N — номер первого упавшего сценария (с raw-выводом)
# ============================================================================
set -uo pipefail   # без -e (некоторые сценарии ждут non-zero exit)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/../.." && pwd)"
# SCRIPT is at <REPO_ROOT>/scripts/agent_flow/kanban-report-write.sh
SCRIPT="${REPO_ROOT}/agent_flow/kanban-report-write.sh"

# sandbox: копия worktree для тестов, чтобы не мусорить в основном
TMPBASE="$(mktemp -d /tmp/kanban-report-test.XXXXXX)"
trap 'rm -rf "$TMPBASE"' EXIT

_pass=0
_fail=0
_scenario=0
_fail_msg=""

_assert_eq() {
    # _assert_eq <name> <expected> <actual>
    if [ "$2" = "$3" ]; then
        _pass=$((_pass + 1))
    else
        _fail=$((_fail + 1))
        _fail_msg="${_fail_msg}ASSERT[$1] expected=[$2] actual=[$3] | "
        echo "  FAIL: $1 expected=[$2] got=[$3]" >&2
    fi
}

_assert_contains() {
    # _assert_contains <name> <expected_substring> <haystack>
    case "$3" in
        *"$2"*) _pass=$((_pass + 1)) ;;
        *) _fail=$((_fail + 1))
           _fail_msg="${_fail_msg}CONTAINS[$1] expected_sub=[$2] | "
           echo "  FAIL: $1 does not contain [$2]" >&2
           echo "  --- haystack (first 30 lines) ---" >&2
           echo "$3" | head -30 >&2
           ;;
    esac
}

_run_scenario() {
    _scenario=$((_scenario + 1))
    local name="$1"; shift
    echo "--- Scenario $_scenario: $name ---"
    if "$@"; then
        echo "  OK"
    else
        _fail=$((_fail + 1))
        _fail_msg="${_fail_msg}SCEN[$_scenario:$name exit=$?] | "
        echo "  FAIL: scenario $_scenario ($name) returned non-zero" >&2
    fi
}

# --- A) дефолты --------------------------------------------------------------
scenario_a() {
    local out="${TMPBASE}/A"
    mkdir -p "$out"
    ( cd "$REPO_ROOT" && \
      HERMES_KANBAN_ASSIGNEE="" \
      bash "$SCRIPT" "t_test_a" --output "$out/report.md" >/dev/null )
    local rc=$?
    [ $rc -eq 0 ] || { echo "  exit=$rc"; return 1; }
    [ -f "$out/report.md" ] || { echo "  no output file"; return 1; }
    local body; body="$(cat "$out/report.md")"
    _assert_eq "A.task_id" "t_test_a" "$(grep -E '^\*\*Task ID:\*\* t_test_a' "$out/report.md" >/dev/null && echo t_test_a || echo NO)"
    _assert_contains "A.assignee_default" "**Assignee:** devops" "$body"
    _assert_contains "A.completed_ts" "**Completed:**" "$body"
    _assert_contains "A.skill_section" "## Skill results" "$body"
    return 0
}

# --- B) с --title и --assignee -----------------------------------------------
scenario_b() {
    local out="${TMPBASE}/B"
    mkdir -p "$out"
    ( cd "$REPO_ROOT" && \
      bash "$SCRIPT" "t_test_b" --title "My fix" --assignee backend --output "$out/r.md" >/dev/null )
    local rc=$?
    [ $rc -eq 0 ] || { echo "  exit=$rc"; return 1; }
    local body; body="$(cat "$out/r.md")"
    _assert_contains "B.title" "# Отчёт: My fix" "$body"
    _assert_contains "B.assignee" "**Assignee:** backend" "$body"
    return 0
}

# --- C) с --pr и --issue -----------------------------------------------------
scenario_c() {
    local out="${TMPBASE}/C"
    mkdir -p "$out"
    ( cd "$REPO_ROOT" && \
      bash "$SCRIPT" "t_test_c" --pr 4242 --issue 2159 --output "$out/r.md" >/dev/null )
    local rc=$?
    [ $rc -eq 0 ] || { echo "  exit=$rc"; return 1; }
    local body; body="$(cat "$out/r.md")"
    _assert_contains "C.pr_line" "**PR:** #4242" "$body"
    _assert_contains "C.pr_url" "pull/4242" "$body"
    _assert_contains "C.issue_line" "**Issue:** #2159" "$body"
    return 0
}

# --- D) --output в нестандартный путь ----------------------------------------
scenario_d() {
    local custom="${TMPBASE}/D/deep/nested/path/r.md"
    mkdir -p "$(dirname "$custom")"
    ( cd "$REPO_ROOT" && \
      bash "$SCRIPT" "t_test_d" --output "$custom" >/dev/null )
    local rc=$?
    [ $rc -eq 0 ] || { echo "  exit=$rc"; return 1; }
    [ -f "$custom" ] || { echo "  no file at $custom"; return 1; }
    return 0
}

# --- E) без TASK_ID → exit 1 -------------------------------------------------
scenario_e() {
    local out
    out="$( cd "$REPO_ROOT" && bash "$SCRIPT" 2>&1 >/dev/null )"
    local rc=$?
    [ $rc -eq 1 ] || { echo "  expected exit=1, got $rc. stderr=$out"; return 1; }
    return 0
}

# --- F) не в git worktree → exit 2 -------------------------------------------
scenario_f() {
    local dir="${TMPBASE}/F"
    mkdir -p "$dir"
    # Передаём --output в TMPBASE, чтобы скрипт НЕ падал раньше времени на
    # определении worktree. Запускаем из TMPBASE (там нет .git).
    ( cd "$dir" && \
      bash "$SCRIPT" "t_test_f" --output "${TMPBASE}/F/r.md" >/dev/null 2>&1 )
    local rc=$?
    [ $rc -eq 2 ] || { echo "  expected exit=2, got $rc"; return 1; }
    return 0
}

# --- G) BASE_REF недоступен → exit 0, fallback -------------------------------
scenario_g() {
    local out="${TMPBASE}/G"
    mkdir -p "$out"
    ( cd "$REPO_ROOT" && \
      bash "$SCRIPT" "t_test_g" --base "refs/heads/__nonexistent__" --output "$out/r.md" >/dev/null )
    local rc=$?
    [ $rc -eq 0 ] || { echo "  exit=$rc"; return 1; }
    [ -f "$out/r.md" ] || { echo "  no output"; return 1; }
    _assert_contains "G.fallback_msg" "недоступен" "$(cat "$out/r.md")"
    return 0
}

# --- H) идемпотентность ------------------------------------------------------
scenario_h() {
    local out="${TMPBASE}/H"
    mkdir -p "$out"
    ( cd "$REPO_ROOT" && \
      bash "$SCRIPT" "t_test_h" --title "First" --output "$out/r.md" >/dev/null )
    ( cd "$REPO_ROOT" && \
      bash "$SCRIPT" "t_test_h" --title "Second" --output "$out/r.md" >/dev/null )
    local rc=$?
    [ $rc -eq 0 ] || { echo "  exit=$rc"; return 1; }
    _assert_contains "H.overwrite" "# Отчёт: Second" "$(cat "$out/r.md")"
    return 0
}

# --- main --------------------------------------------------------------------
echo "Running test_kanban_report_write.sh against $SCRIPT"
_run_scenario "A_defaults"               scenario_a
_run_scenario "B_title_assignee"         scenario_b
_run_scenario "C_pr_issue"               scenario_c
_run_scenario "D_custom_output"          scenario_d
_run_scenario "E_no_task_id"             scenario_e
_run_scenario "F_no_git_worktree"        scenario_f
_run_scenario "G_bad_base_ref"           scenario_g
_run_scenario "H_idempotent_overwrite"   scenario_h

echo ""
echo "===== Result: pass=$_pass fail=$_fail ====="
echo "fail_msg: $_fail_msg"
[ "$_fail" -eq 0 ]
