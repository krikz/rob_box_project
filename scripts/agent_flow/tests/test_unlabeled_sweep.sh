#!/bin/bash
# ============================================================================
# test_unlabeled_sweep.sh — unit-тест двушаговой автозакрывалки
#                           (ADR-0022 GATE-2).
#
# Тест не зовёт gh API напрямую. gh() подменён mock-функцией в test-env,
# скрипт работает в UNLABELED_SWEEP_TEST_MODE=1 (пропускает cli_args,
# flock, gh-auth) и обращается к нашему mock gh.
#
# Mock gh возвращает:
#   - gh issue list            → JSON с настраиваемыми issues
#   - gh api timeline          → JSON с настраиваемыми событиями (reopened, labeled)
#   - gh api comments          → JSON с настраиваемыми комментариями
#   - gh issue edit/comment/close → пишут вызовы в /tmp/log для анализа
#
# Кейсы:
#   T1:  OPEN, age=10h, без меток → nothing (fresh, age<STALE_HOURS_1)
#   T2:  OPEN, age=25h, без меток → label stale-candidate + comment
#   T3:  OPEN, age=30h, stale-candidate, без reopen → close
#   T4:  OPEN, age=30h, stale-candidate, reopen ДО метки → close
#   T5:  OPEN, age=30h, stale-candidate, reopen ПОСЛЕ метки → un-stale, не close
#   T6:  OPEN с меткой `hermes` → skip
#   T7:  idempotency: повторный тик не дублирует stale-комментарий
#   T8:  timeline API сдох → fail-closed (skip close)
#   T9:  OPEN с меткой `e2e-done` → skip
#   T10: shellcheck-clean + syntax-OK + required helpers exist
#
# Usage:
#   bash test_unlabeled_sweep.sh
# Env:
#   VERBOSE=1 — печатать mock calls во время теста
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT_UNDER_TEST="$TESTS_DIR/../agent-flow-unlabeled-sweep.sh"

PASS=0
FAIL=0
FAILED_CASES=()

if [ ! -f "$SCRIPT_UNDER_TEST" ]; then
  echo "FAIL: script not found: $SCRIPT_UNDER_TEST"
  exit 1
fi

echo "=== test_unlabeled_sweep.sh — agent-flow-unlabeled-sweep.sh unit tests ==="
echo

# ----------------------------------------------------------------------------
# Assertion helpers
# ----------------------------------------------------------------------------
assert_eq() {
  local got="$1" exp="$2" desc="$3"
  if [ "$got" = "$exp" ]; then
    PASS=$((PASS+1))
    echo "  ✓ $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (got=$got expected=$exp)")
    echo "  ✗ $desc — got='$got' expected='$exp'"
  fi
}

assert_greater() {
  local got="$1" min="$2" desc="$3"
  if [ "$got" -gt "$min" ] 2>/dev/null; then
    PASS=$((PASS+1))
    echo "  ✓ $desc (got=$got > $min)"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (got=$got not> $min)")
    echo "  ✗ $desc — got=$got not > $min"
  fi
}

assert_contains() {
  local file="$1" needle="$2" desc="$3"
  if grep -qF -- "$needle" "$file"; then
    PASS=$((PASS+1))
    echo "  ✓ $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (missing '$needle')")
    echo "  ✗ $desc — missing '$needle' in calls log"
  fi
}

assert_not_contains() {
  local file="$1" needle="$2" desc="$3"
  if ! grep -qF -- "$needle" "$file"; then
    PASS=$((PASS+1))
    echo "  ✓ $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (unexpected '$needle')")
    echo "  ✗ $desc — unexpected '$needle' in calls log"
  fi
}

# ----------------------------------------------------------------------------
# Mock-factory: build a wrapper script that overrides `gh` and exits
# ----------------------------------------------------------------------------
# Используем pattern «wrapper»: мокаем gh через `command_not_found_handle`
# НЕ работает в bash. Вместо этого запускаем скрипт в подоболочке с
# функцией gh() в env (export -f). Но `export -f` есть в bash 4.2+,
# и работает только если вызывающий bash запускает подоболочку.

# Лучший подход: создаём runner-скрипт на лету, который source'ит
# override-функции, source'ит SCRIPT и вызывает main. ИЛИ: запускаем
# в subshell через `bash -c` с inline-функциями.

# Чтобы тест был portable, делаем один большой runner-script:
# каждый test-case — это sub-shell с reset-state + new mocks + bash script.
#
# Переменные, которые runner "экспортирует" в sub-script через env.

WORK_DIR="$(mktemp -d -t test-unlabeled-sweep.XXXXXX)"
trap 'rm -rf "$WORK_DIR"' EXIT

# Mock helper function: writes a gh() override + lock + dir to a temp file.
# Called by every test case.
# Args: $1=test_name $2=issues_json $3=timeline_json $4=comments_json
emit_mock_runner() {
  local test_name="$1"
  local issues_json="$2"
  local timeline_json="$3"
  local comments_json="$4"

  cat > "$WORK_DIR/run_${test_name}.sh" <<EOF
#!/bin/bash
# mock runner: gh() overridden, script runs in test mode.
set -o pipefail
export UNLABELED_SWEEP_TEST_MODE=1
export GH_REPO='krikz/rob_box_project'
export STALE_HOURS_1='1'
export STALE_HOURS_2='1'
export DRY_RUN='false'
export LOCK_FILE='/tmp/test-unlabeled-sweep-runner.lock'
export SWEEP_LIMIT=200
export HOME=/tmp

# Mock gh
gh() {
  case "\$1" in
    auth)
      case "\${2:-}" in
        status) return 0 ;;
      esac
      ;;
    issue)
      case "\${2:-}" in
        list) printf '%s' '${issues_json}'; return 0 ;;
        edit)
          shift 2
          # side-effect log
          printf 'edit: %s\n' "\$*" >> '$WORK_DIR/${test_name}.calls'
          return 0
          ;;
        comment)
          shift 2
          printf 'comment: %s\n' "\$*" >> '$WORK_DIR/${test_name}.calls'
          return 0
          ;;
        close)
          shift 2
          printf 'close: %s\n' "\$*" >> '$WORK_DIR/${test_name}.calls'
          return 0
          ;;
      esac
      ;;
    api)
      # 'repos/owner/repo/issues/N/timeline?per_page=100'
      # Mock возвращает timeline как есть — скрипт фильтрует в python.
      if [[ "\${2:-}" =~ issues/[^/]+/timeline ]]; then
        printf '%s' '${timeline_json}'; return 0
      elif [[ "\${2:-}" =~ issues/[^/]+/comments ]]; then
        printf '%s' '${comments_json}'; return 0
      fi
      echo '[]'; return 0
      ;;
  esac
  return 1
}
export -f gh

# Делаем dates согласованными: подменяем now() в скрипте через патч
# (только в test mode). Скрипт использует \$(date +%s) и \$(date -u ...),
# но для изоляции мы просто даём ему реальное время — issue updatedAt
# мы задаём как ISO-время, а age рассчитывается из разности.

bash '${SCRIPT_UNDER_TEST}'

# Создаём файл вызовов даже если скрипт не делал side-effects —
# чтобы тест мог проверить "NO edit calls" без missing-file race.
touch '$WORK_DIR/${test_name}.calls'
EOF
  chmod +x "$WORK_DIR/run_${test_name}.sh"
}

# ----------------------------------------------------------------------------
# T1: helper presence + syntax + shellcheck
# ----------------------------------------------------------------------------
echo "--- T1: script syntax + shellcheck-clean + helpers exist ---"
if bash -n "$SCRIPT_UNDER_TEST"; then
  PASS=$((PASS+1)); echo "  ✓ bash syntax OK"
else
  FAIL=$((FAIL+1)); FAILED_CASES+=("bash syntax")
  echo "  ✗ bash syntax FAILED"
fi

SC_OUT="$(shellcheck -x "$SCRIPT_UNDER_TEST" 2>&1 || true)"
# Allowed: SC2016 (false positive on printf %s), SC2034 (declared unused
# with disable marker). Anything else = failure.
SC_ERR_COUNT=0
while IFS= read -r line; do
  case "$line" in
    ""|*"SC2016"*|*"SC2034"*|*"SC1090"*) : ;;  # allowed warnings
    *) SC_ERR_COUNT=$((SC_ERR_COUNT+1)) ;;
  esac
done < <(echo "$SC_OUT" | grep -E 'SC[0-9]+' || true)
assert_eq "$SC_ERR_COUNT" "0" "shellcheck has no errors (only allowed warnings)"

HELPERS=(has_label to_epoch last_reopen_at stale_labeled_at \
         has_recent_marker_comment now_minus_h_iso)
for fn in "${HELPERS[@]}"; do
  if grep -qE "^${fn}\(\)|^function ${fn} " "$SCRIPT_UNDER_TEST"; then
    PASS=$((PASS+1)); echo "  ✓ $fn() defined"
  else
    FAIL=$((FAIL+1)); FAILED_CASES+=("missing $fn()")
    echo "  ✗ $fn() missing"
  fi
done

# Двушаговая модель
if grep -q 'GATE-2\|двушаг' "$SCRIPT_UNDER_TEST"; then
  PASS=$((PASS+1)); echo "  ✓ references GATE-2 / двушаг in header"
else
  FAIL=$((FAIL+1)); echo "  ✗ no GATE-2/двушаг reference"
fi

# Поддержка --dry-run, --stale-hours-1/2, --limit.
for flag in '--dry-run' '--stale-hours-1' '--stale-hours-2' '--limit'; do
  if grep -qF -- "$flag" "$SCRIPT_UNDER_TEST"; then
    PASS=$((PASS+1)); echo "  ✓ flag $flag present"
  else
    FAIL=$((FAIL+1)); echo "  ✗ flag $flag missing"
  fi
done

# ----------------------------------------------------------------------------
# Behavior tests через mock-gh runner
# ----------------------------------------------------------------------------
NOW_ISO="$(date -u +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-18T20:00:00Z')"
NOW_MINUS_1H="$(date -u -d '1 hour ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-18T19:00:00Z')"
NOW_MINUS_30H="$(date -u -d '30 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-17T14:00:00Z')"
NOW_MINUS_25H="$(date -u -d '25 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-17T19:00:00Z')"

# T2: OPEN без меток, age ~30h → должно повесить stale-candidate
echo
echo "--- T2: OPEN age=30h без меток → mark stale-candidate (gating 'age>=STALE_HOURS_1') ---"
ISSUES_JSON='[
  {"number":1001,"title":"old unlabeled","updatedAt":"'"$NOW_MINUS_30H"'","createdAt":"'"$NOW_MINUS_30H"'","labels":[]},
  {"number":1002,"title":"fresh unlabeled","updatedAt":"'"$NOW_ISO"'","createdAt":"'"$NOW_ISO"'","labels":[]}
]'
TIMELINE_JSON='[]'
COMMENTS_JSON='[]'
emit_mock_runner "t2" "$ISSUES_JSON" "$TIMELINE_JSON" "$COMMENTS_JSON"
bash "$WORK_DIR/run_t2.sh" >/dev/null 2>&1 || true
if [ -f "$WORK_DIR/t2.calls" ]; then
  assert_contains "$WORK_DIR/t2.calls" "edit: 1001 --repo krikz/rob_box_project --add-label stale-candidate" "T2: 1001 → add-label stale-candidate"
  assert_not_contains "$WORK_DIR/t2.calls" "edit: 1002 --repo krikz/rob_box_project --add-label stale-candidate" "T2: 1002 (fresh) NO add-label"
  assert_contains "$WORK_DIR/t2.calls" "comment: 1001" "T2: 1001 → comment"
else
  FAIL=$((FAIL+1)); FAILED_CASES+=("T2 calls log missing")
  echo "  ✗ T2 calls log missing (mock runner broken?)"
fi

# T3: OPEN с stale-candidate, age 30h, без reopen → close
echo
echo "--- T3: OPEN с stale-candidate, age=30h, без reopen → close ---"
ISSUES_JSON='[
  {"number":2001,"title":"already stale","updatedAt":"'"$NOW_MINUS_30H"'","createdAt":"'"$NOW_MINUS_30H"'","labels":[{"name":"stale-candidate"}]}
]'
# Timeline показывает: stale-candidate поставлен 25h назад.
STALE_LABEL_TIME="$(date -u -d '25 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-17T19:00:00Z')"
TIMELINE_JSON='[
  {"event":"labeled","label":{"name":"stale-candidate"},"created_at":"'"$STALE_LABEL_TIME"'"}
]'
COMMENTS_JSON='[]'
emit_mock_runner "t3" "$ISSUES_JSON" "$TIMELINE_JSON" "$COMMENTS_JSON"
bash "$WORK_DIR/run_t3.sh" >/dev/null 2>&1 || true
if [ -f "$WORK_DIR/t3.calls" ]; then
  assert_contains "$WORK_DIR/t3.calls" "edit: 2001 --repo" "T3: 2001 → edit (any label op)"
  assert_contains "$WORK_DIR/t3.calls" "edit: 2001 --repo krikz/rob_box_project --remove-label stale-candidate" "T3: 2001 → remove stale-candidate before close"
  assert_contains "$WORK_DIR/t3.calls" "close: 2001" "T3: 2001 → close"
  assert_not_contains "$WORK_DIR/t3.calls" "edit: 2001 --repo krikz/rob_box_project --add-label stale-candidate" "T3: 2001 NO add-label (уже есть)"
else
  FAIL=$((FAIL+1)); FAILED_CASES+=("T3 calls log missing")
  echo "  ✗ T3 calls log missing"
fi

# T4: OPEN с stale-candidate + reopen ДО метки → close (reopen не подавляет)
echo
echo "--- T4: OPEN с stale-candidate + reopen ДО метки → close (fresh e2e-done equivalent) ---"
ISSUES_JSON='[
  {"number":3001,"title":"reopen before stale","updatedAt":"'"$NOW_MINUS_30H"'","createdAt":"'"$NOW_MINUS_30H"'","labels":[{"name":"stale-candidate"}]}
]'
# Reopen 30h ago, stale-метка 25h ago. Reopen ДО → close.
REOPEN_TIME="$(date -u -d '30 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-17T14:00:00Z')"
TIMELINE_JSON='[
  {"event":"reopened","created_at":"'"$REOPEN_TIME"'"},
  {"event":"labeled","label":{"name":"stale-candidate"},"created_at":"'"$STALE_LABEL_TIME"'"}
]'
COMMENTS_JSON='[]'
emit_mock_runner "t4" "$ISSUES_JSON" "$TIMELINE_JSON" "$COMMENTS_JSON"
bash "$WORK_DIR/run_t4.sh" >/dev/null 2>&1 || true
if [ -f "$WORK_DIR/t4.calls" ]; then
  assert_contains "$WORK_DIR/t4.calls" "close: 3001" "T4: 3001 → close (reopen ДО)"
else
  FAIL=$((FAIL+1)); FAILED_CASES+=("T4 calls log missing")
  echo "  ✗ T4 calls log missing"
fi

# T5: OPEN с stale-candidate + reopen ПОСЛЕ метки → un-stale + no close
echo
echo "--- T5: OPEN с stale-candidate + reopen ПОСЛЕ метки → un-stale (не close) ---"
ISSUES_JSON='[
  {"number":4001,"title":"reopen after stale","updatedAt":"'"$NOW_MINUS_30H"'","createdAt":"'"$NOW_MINUS_30H"'","labels":[{"name":"stale-candidate"}]}
]'
# Stale-метка 25h ago, reopen 1h ago. Reopen ПОСЛЕ → un-stale.
USER_REOPEN="$(date -u -d '1 hour ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-18T19:00:00Z')"
TIMELINE_JSON='[
  {"event":"labeled","label":{"name":"stale-candidate"},"created_at":"'"$STALE_LABEL_TIME"'"},
  {"event":"reopened","created_at":"'"$USER_REOPEN"'"}
]'
COMMENTS_JSON='[]'
emit_mock_runner "t5" "$ISSUES_JSON" "$TIMELINE_JSON" "$COMMENTS_JSON"
bash "$WORK_DIR/run_t5.sh" >/dev/null 2>&1 || true
if [ -f "$WORK_DIR/t5.calls" ]; then
  assert_contains "$WORK_DIR/t5.calls" "edit: 4001 --repo krikz/rob_box_project --remove-label stale-candidate" "T5: 4001 → remove stale-candidate (user-reopen)"
  assert_not_contains "$WORK_DIR/t5.calls" "close: 4001" "T5: 4001 НЕ close (user-reopen отменил)"
else
  FAIL=$((FAIL+1)); FAILED_CASES+=("T5 calls log missing")
  echo "  ✗ T5 calls log missing"
fi

# T6: OPEN с process-меткой → skip (no labels change)
echo
echo "--- T6: OPEN с process-метками → skip (нет side-effects) ---"
ISSUES_JSON='[
  {"number":5001,"title":"hermes tagged","updatedAt":"'"$NOW_MINUS_30H"'","createdAt":"'"$NOW_MINUS_30H"'","labels":[{"name":"hermes"}]},
  {"number":5002,"title":"needs-e2e tagged","updatedAt":"'"$NOW_MINUS_30H"'","createdAt":"'"$NOW_MINUS_30H"'","labels":[{"name":"needs-e2e"}]},
  {"number":5003,"title":"e2e-done tagged","updatedAt":"'"$NOW_MINUS_30H"'","createdAt":"'"$NOW_MINUS_30H"'","labels":[{"name":"e2e-done"}]},
  {"number":5004,"title":"e2e-rejected tagged","updatedAt":"'"$NOW_MINUS_30H"'","createdAt":"'"$NOW_MINUS_30H"'","labels":[{"name":"e2e:rejected"}]},
  {"number":5005,"title":"no-e2e-required tagged","updatedAt":"'"$NOW_MINUS_30H"'","createdAt":"'"$NOW_MINUS_30H"'","labels":[{"name":"no-e2e-required"}]}
]'
TIMELINE_JSON='[]'
COMMENTS_JSON='[]'
emit_mock_runner "t6" "$ISSUES_JSON" "$TIMELINE_JSON" "$COMMENTS_JSON"
bash "$WORK_DIR/run_t6.sh" >/dev/null 2>&1 || true
if [ -f "$WORK_DIR/t6.calls" ]; then
  assert_not_contains "$WORK_DIR/t6.calls" "edit: 5001" "T6: 5001 (hermes) NO edit"
  assert_not_contains "$WORK_DIR/t6.calls" "edit: 5002" "T6: 5002 (needs-e2e) NO edit"
  assert_not_contains "$WORK_DIR/t6.calls" "edit: 5003" "T6: 5003 (e2e-done) NO edit"
  assert_not_contains "$WORK_DIR/t6.calls" "edit: 5004" "T6: 5004 (e2e:rejected) NO edit"
  assert_not_contains "$WORK_DIR/t6.calls" "edit: 5005" "T6: 5005 (no-e2e-required) NO edit"
  assert_not_contains "$WORK_DIR/t6.calls" "close: 5" "T6: NO close calls"
else
  FAIL=$((FAIL+1)); FAILED_CASES+=("T6 calls log missing")
  echo "  ✗ T6 calls log missing"
fi

# T7: idempotency — повторный тик с тем же issue, в комментариях уже есть stale-marker.
# Скрипт не должен снова add-label (а должен пропустить, сказав "stale-комментарий уже").
echo
echo "--- T7: idempotency — 6h dedup, stale-комментарий уже есть ---"
ISSUES_JSON='[
  {"number":6001,"title":"already warned","updatedAt":"'"$NOW_MINUS_30H"'","createdAt":"'"$NOW_MINUS_30H"'","labels":[]}
]'
TIMELINE_JSON='[]'
# В комментариях уже есть stale-комментарий
COMMENTS_JSON='[
  {"body":"⚠️ auto-sweep pending stale-candidate (ADR-0022 GATE-2): ..."}
]'
emit_mock_runner "t7" "$ISSUES_JSON" "$TIMELINE_JSON" "$COMMENTS_JSON"
bash "$WORK_DIR/run_t7.sh" >/dev/null 2>&1 || true
if [ -f "$WORK_DIR/t7.calls" ]; then
  assert_not_contains "$WORK_DIR/t7.calls" "edit: 6001 --repo krikz/rob_box_project --add-label" "T7: 6001 NO add-label (idempotent)"
else
  FAIL=$((FAIL+1)); FAILED_CASES+=("T7 calls log missing")
  echo "  ✗ T7 calls log missing"
fi

# T8: timeline API сдох (empty timeline stale_labeled_at), issue stale-candidate
# → fail-closed: пропускаем, не закрываем.
echo
echo "--- T8: timeline API сдох (stale_labeled_at empty) → fail-closed, не close ---"
ISSUES_JSON='[
  {"number":7001,"title":"stale with broken timeline","updatedAt":"'"$NOW_MINUS_30H"'","createdAt":"'"$NOW_MINUS_30H"'","labels":[{"name":"stale-candidate"}]}
]'
TIMELINE_JSON='[]'  # пустой → labeled_at = '' → null → fail-closed
COMMENTS_JSON='[]'
emit_mock_runner "t8" "$ISSUES_JSON" "$TIMELINE_JSON" "$COMMENTS_JSON"
bash "$WORK_DIR/run_t8.sh" >/dev/null 2>&1 || true
if [ -f "$WORK_DIR/t8.calls" ]; then
  assert_not_contains "$WORK_DIR/t8.calls" "close: 7001" "T8: 7001 НЕ close (timeline dead)"
else
  FAIL=$((FAIL+1)); FAILED_CASES+=("T8 calls log missing")
  echo "  ✗ T8 calls log missing"
fi

# ----------------------------------------------------------------------------
# T9: ENV_FILE robustness — скрипт находит .env даже если $HERMES_HOME
#     указывает на нестандартный путь (например профильную папку).
#     Ретро 31.08 (t_9b0d60f7): ENV_FILE раньше вычислялся напрямую из
#     $HERMES_HOME и падал в no_agent cron-режиме когда переменная в env
#     указывала на /home/builder/.hermes/profiles/devops вместо /home/builder/.hermes.
# ----------------------------------------------------------------------------
echo
echo "--- T9: ENV_FILE robustness — HERMES_HOME=профиль, скрипт всё равно находит .env ---"

# Запустим script с подменой HERMES_HOME на битый путь И с mock-gh.
# Ожидание: ENV_FILE подхватится через fallback candidate, GH_REPO грузится,
# скрипт проходит до main (с dry-run не делает gh-вызовов).
HERMES_BROKEN="$WORK_DIR/profile-broken-home"
mkdir -p "$HERMES_BROKEN"
T9_RUNNER="$WORK_DIR/run_t9.sh"
cat > "$T9_RUNNER" <<EOF
#!/bin/bash
set -o pipefail
export UNLABELED_SWEEP_TEST_MODE=1
export STALE_HOURS_1='1'
export STALE_HOURS_2='1'
export DRY_RUN='true'   # dry-run чтобы не дёргать gh
export LOCK_FILE='/tmp/test-unlabeled-sweep-runner.lock'
export SWEEP_LIMIT=200
# Битый HERMES_HOME — НЕ содержит profiles/agent-flow/.env
export HERMES_HOME='$HERMES_BROKEN'
# HOME тоже битый (simulate profile scope)
export HOME='$HERMES_BROKEN/home'
# GH_REPO НЕ задан — пусть скрипт сам найдёт через .env fallback
unset GH_REPO
unset KANBAN_BOARD

gh() { return 0; }
export -f gh

bash '${SCRIPT_UNDER_TEST}'
EOF
chmod +x "$T9_RUNNER"
T9_OUT=$(bash "$T9_RUNNER" 2>&1 || true)
if echo "$T9_OUT" | grep -q "tick start: GH_REPO=krikz/rob_box_project"; then
  PASS=$((PASS+1)); echo "  ✓ T9: скрипт нашёл .env через fallback (HERMES_HOME=$HERMES_BROKEN)"
else
  FAIL=$((FAIL+1)); FAILED_CASES+=("T9: ENV_FILE fallback failed")
  echo "  ✗ T9: ENV_FILE fallback failed — output:"
  echo "$T9_OUT" | head -5
fi

# Также проверим что exit code != 1 (раньше был exit 1 на GH_REPO unset)
if echo "$T9_OUT" | grep -q "GH_REPO must be set"; then
  FAIL=$((FAIL+1)); FAILED_CASES+=("T9: GH_REPO unset message — fallback не сработал")
  echo "  ✗ T9: скрипт упал с 'GH_REPO must be set' (fallback не сработал)"
else
  PASS=$((PASS+1)); echo "  ✓ T9: скрипт НЕ падает с 'GH_REPO must be set' (fallback OK)"
fi

# ----------------------------------------------------------------------------
# T10: ENV_FILE robustness — HERMES_HOME вообще не задан (system-cron).
#      Должен подхватить через $HOME/.hermes/profiles/agent-flow/.env.
# ----------------------------------------------------------------------------
echo
echo "--- T10: ENV_FILE robustness — HERMES_HOME unset, HOME=/home/builder ---"
T10_RUNNER="$WORK_DIR/run_t10.sh"
cat > "$T10_RUNNER" <<EOF
#!/bin/bash
set -o pipefail
export UNLABELED_SWEEP_TEST_MODE=1
export STALE_HOURS_1='1'
export STALE_HOURS_2='1'
export DRY_RUN='true'
export LOCK_FILE='/tmp/test-unlabeled-sweep-runner.lock'
export SWEEP_LIMIT=200
unset HERMES_HOME
unset GH_REPO
unset KANBAN_BOARD
export HOME=/home/builder

gh() { return 0; }
export -f gh

bash '${SCRIPT_UNDER_TEST}'
EOF
chmod +x "$T10_RUNNER"
T10_OUT=$(bash "$T10_RUNNER" 2>&1 || true)
if echo "$T10_OUT" | grep -q "tick start: GH_REPO=krikz/rob_box_project"; then
  PASS=$((PASS+1)); echo "  ✓ T10: скрипт работает с HERMES_HOME=unset (HOME-fallback)"
else
  FAIL=$((FAIL+1)); FAILED_CASES+=("T10: HOME-fallback failed")
  echo "  ✗ T10: HOME-fallback failed — output:"
  echo "$T10_OUT" | head -5
fi

if echo "$T10_OUT" | grep -q "GH_REPO must be set"; then
  FAIL=$((FAIL+1)); FAILED_CASES+=("T10: GH_REPO unset — HOME-fallback не сработал")
  echo "  ✗ T10: скрипт упал с 'GH_REPO must be set' (HOME-fallback не сработал)"
else
  PASS=$((PASS+1)); echo "  ✓ T10: скрипт НЕ падает с 'GH_REPO must be set' (HOME-fallback OK)"
fi

# ----------------------------------------------------------------------------
# Summary
# ----------------------------------------------------------------------------
echo
echo "=== Summary: $PASS passed, $FAIL failed ==="
if [ "$FAIL" -gt 0 ]; then
  echo
  echo "FAILED CASES:"
  for c in "${FAILED_CASES[@]}"; do
    echo "  - $c"
  done
  exit 1
fi
echo "ALL TESTS PASSED"
exit 0
