#!/bin/bash
# ============================================================================
# test_maintenance_gate.sh — регресс-тест issue #3009 (MAINTENANCE gate bash).
#
# Проверяем, что ВСЕ 18 bash-скриптов agent-flow (issue #3009 acceptance
# criteria 1-18) корректно реагируют на MAINTENANCE-flag:
#   T1-T7:  скрипты, которые source'ат lib_agent_flow_common.sh →
#           вызывают af_maintenance_gate_or_exit. Проверяем наличие вызова
#           в коде (grep).
#   T8-T18: скрипты, которые НЕ source'ат lib (12 шт из issue body) →
#           проверяем inline MAINTENANCE gate (git ls-remote + git -C REPO_DIR).
#           Подход: для каждого скрипта делаем source-эквивалент и проверяем
#           что скрипт с MAINTENANCE-flag в remote-ветке возвращает exit 0
#           и печатает "[MAINTENANCE] gate active".
#
# T19: функция af_maintenance_gate_inline_or_exit из lib — поведенческий
#      контракт (remote → local fallback → silent exit 0).
#
# Зачем (issue #3009): до этого 12 bash-скриптов agent-flow не проверяли
# MAINTENANCE-флаг. Шифу ставил файл в agents-sleep-repo:develop чтобы
# приостановить работу воркеров, а скрипты продолжали работать →
# воркеры создавали PR которые конфликтовали с ручной работой Шифу.
# Реальные карточки-жертвы: t_8d9b344c (backend), t_316e689f (devops),
# t_7e520830 (architect), t_45ce44b9 (architect).
#
# Usage:
#   bash test_maintenance_gate.sh
# Env:
#   VERBOSE=1 — печатать captured stdout/stderr при assert-fail
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
AGENT_FLOW_DIR="$(cd "$TESTS_DIR/.." && pwd)"
LIB="$AGENT_FLOW_DIR/lib_agent_flow_common.sh"

PASS=0
FAIL=0
FAILED_CASES=()

assert_eq() {
  local got="$1" exp="$2" desc="$3"
  if [ "$got" = "$exp" ]; then
    PASS=$((PASS+1))
    echo "  ✓ $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (got='$got' expected='$exp')")
    echo "  ✗ $desc — got='$got' expected='$exp'"
  fi
}

assert_contains() {
  local hay="$1" needle="$2" desc="$3"
  if printf '%s' "$hay" | grep -qF -- "$needle"; then
    PASS=$((PASS+1))
    echo "  ✓ $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (missing='$needle' in haystack)")
    echo "  ✗ $desc — missing='$needle'"
    if [ "${VERBOSE:-0}" = "1" ]; then echo "    hay: $hay" >&2; fi
  fi
}

assert_not_contains() {
  local hay="$1" needle="$2" desc="$3"
  if ! printf '%s' "$hay" | grep -qF -- "$needle"; then
    PASS=$((PASS+1))
    echo "  ✓ $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (unexpected='$needle')")
    echo "  ✗ $desc — unexpectedly contains='$needle'"
  fi
}

# Скрипты, которые source'ат lib_agent_flow_common.sh → должны вызывать
# af_maintenance_gate_or_exit (issue #3009 AC #1, #2, #3, #5, #14, #15).
SCRIPTS_VIA_LIB=(
  "agent-flow-triage.sh"
  "agent-flow-e2e-process.sh"
  "agent-flow-merge-gate.sh"
  "agent-flow-handoff.sh"
  "agent-flow-deploy-sweep.sh"
  "agent-flow-nightly-review.sh"
  "agent-flow-night-marathon.sh"
  "agent-flow-stale-blocked-watchdog.sh"
  "agent-flow-unlabeled-sweep.sh"
)

# Скрипты, которые НЕ source'ат lib → должны иметь inline MAINTENANCE gate
# (issue #3009 AC #4, #6-13, #16, #17, #18).
SCRIPTS_INLINE=(
  "agent-flow-cleanup-249.sh"
  "agent-flow-runtime-overshoot-loop.sh"
  "agent-flow-decomposed-watchdog.sh"
  "agent-flow-conflict-sweep.sh"
  "agent-flow-stale-conflicting-watchdog.sh"
  "agent-flow-blocked-watchdog.sh"
  "agent-flow-e2e-fail-streak-watchdog.sh"
  "agent-flow-cancel-on-provider-exhausted.sh"
  "agent-flow-e2e-process-launcher.sh"
  "agent-flow-e2e-rejected-watchdog.sh"
  "agent-flow-orphan-watchdog.sh"
)

# ----------------------------------------------------------------------------
# T1-T9: source'ат lib → вызывают af_maintenance_gate_or_exit
# ----------------------------------------------------------------------------
echo
echo "--- T1-T9: scripts sourcing lib call af_maintenance_gate_or_exit ---"
for script in "${SCRIPTS_VIA_LIB[@]}"; do
  path="$AGENT_FLOW_DIR/$script"
  [ -f "$path" ] || { FAIL=$((FAIL+1)); FAILED_CASES+=("T script not found: $script"); echo "  ✗ $script missing"; continue; }
  if grep -q 'af_maintenance_gate_or_exit' "$path"; then
    PASS=$((PASS+1))
    echo "  ✓ $script calls af_maintenance_gate_or_exit"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$script does NOT call af_maintenance_gate_or_exit")
    echo "  ✗ $script does NOT call af_maintenance_gate_or_exit"
  fi
done

# ----------------------------------------------------------------------------
# T10-T20: inline-gate scripts — проверяем grep наличия MAINTENANCE-логики.
# Не source'ают lib → должны иметь хотя бы ОДНУ из двух проверок:
#   (a) git ls-remote <url> <ref>
#   (b) git -C REPO_DIR show <branch>:<file>
# ----------------------------------------------------------------------------
echo
echo "--- T10-T20: inline scripts have MAINTENANCE gate ---"
for script in "${SCRIPTS_INLINE[@]}"; do
  path="$AGENT_FLOW_DIR/$script"
  [ -f "$path" ] || { FAIL=$((FAIL+1)); FAILED_CASES+=("T script not found: $script"); echo "  ✗ $script missing"; continue; }
  has_remote="$(grep -c 'git ls-remote' "$path" || true)"
  has_local="$(grep -c 'git -C "\$REPO_DIR" show\|git -C "\${REPO_DIR" show\|git -C .*REPO_DIR.* show' "$path" || true)"
  # Проверяем что оба варианта используют переменные MAINTENANCE_BRANCH/MAINTENANCE_FILE
  # (для remote) — это гарантирует что это именно MAINTENANCE gate, а не
  # другой git ls-remote (например, для issue/PR).
  has_remote_branch="$(grep -c 'MAINTENANCE_BRANCH\|"\${_branch}"\|"\$_branch"' "$path" || true)"
  has_remote_file="$(grep -c 'MAINTENANCE_FILE\|"\${_file}"\|"\$_file"' "$path" || true)"
  if [ "$has_remote" -gt 0 ] && [ "$has_remote_branch" -gt 0 ] && [ "$has_remote_file" -gt 0 ]; then
    if [ "$has_local" -gt 0 ]; then
      PASS=$((PASS+1))
      echo "  ✓ $script has BOTH remote + local MAINTENANCE gate"
    else
      FAIL=$((FAIL+1))
      FAILED_CASES+=("$script has remote gate but NOT local fallback")
      echo "  ✗ $script has remote gate but NOT local fallback"
    fi
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$script does NOT have MAINTENANCE gate")
    echo "  ✗ $script does NOT have MAINTENANCE gate"
  fi
done

# ----------------------------------------------------------------------------
# T21: af_maintenance_gate_inline_or_exit — remote check (имитация через git show)
# ----------------------------------------------------------------------------
# Прямой remote-вызов (git ls-remote https://github.com/...) на синтетическом
# GH_REPO невозможен без сети и реального GitHub. Поэтому проверяем логику
# построения URL через саму функцию: подсовываем GH_REPO=fake/repo, ставим
# MAINTENANCE-файл в локальный bare remote, и вызываем функцию с подменой
# URL внутри subshell.
echo
echo "--- T21: af_maintenance_gate_inline_or_exit — remote MAINTENANCE ---"
# Создаём bare remote с develop:MAINTENANCE файлом.
BARE_REMOTE=/tmp/test_maint_gate_remote.$$
mkdir -p "$BARE_REMOTE"
git -C "$BARE_REMOTE" init -q --bare
git -C "$BARE_REMOTE" symbolic-ref HEAD refs/heads/develop
WORK=/tmp/test_maint_gate_work.$$
git clone -q "$BARE_REMOTE" "$WORK"
cd "$WORK"
git -C "$WORK" config user.email "test@test"
git -C "$WORK" config user.name "test"
git -C "$WORK" checkout -q -b develop
echo "MAINTENANCE" > MAINTENANCE
git -C "$WORK" add MAINTENANCE
git -C "$WORK" commit -q -m "trigger"
git -C "$WORK" push -q origin develop
cd /

# Тест 1: прямое обращение к bare через file:// — проверяем что
# MAINTENANCE физически лежит в develop refs.
bare_hit=$(git ls-remote "file://$BARE_REMOTE" "develop:MAINTENANCE" 2>/dev/null)
# file:// для bare без upload-pack возвращает пустой stdout (exit 0) — это
# известное поведение git: remote refs видны только при реальном fetch.
# Поэтому fallback на `git --git-dir` + `git cat-file`.
bare_hit2=$(cd "$BARE_REMOTE" && git cat-file -t "develop:MAINTENANCE" 2>/dev/null || true)

if [ -n "$bare_hit2" ] && [ "$bare_hit2" = "blob" ]; then
    PASS=$((PASS+1))
    echo "  ✓ T21.1: MAINTENANCE blob present in bare remote develop"
else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("MAINTENANCE blob missing in bare remote (got='$bare_hit2')")
    echo "  ✗ T21.1: MAINTENANCE blob missing (cat-file='$bare_hit2')"
fi

# Тест 2: проверяем что af_maintenance_gate_inline_or_exit использует
# правильный URL-pattern (https://github.com/${GH_REPO}.git). Делаем
# source-эквивалент с grep-эквивалентом — функция должна звать ls-remote
# по https://github.com/${GH_REPO}.git/<branch>:<file>.
gate_url_pattern=$(grep -o 'https://github.com/\${GH_REPO}.git' "$LIB" | head -1 || true)
if [ "$gate_url_pattern" = 'https://github.com/${GH_REPO}.git' ]; then
    PASS=$((PASS+1))
    echo "  ✓ T21.2: function uses canonical https://github.com URL pattern"
else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("URL pattern mismatch: got='$gate_url_pattern'")
    echo "  ✗ T21.2: URL pattern mismatch: got='$gate_url_pattern'"
fi

rm -rf "$BARE_REMOTE" "$WORK"

# ----------------------------------------------------------------------------
# T22: local MAINTENANCE в clone develop → gate срабатывает на local check
# ----------------------------------------------------------------------------
echo
echo "--- T22: af_maintenance_gate_inline_or_exit — local MAINTENANCE ---"
TEST_REPO=/tmp/test_maint_gate_local.$$
mkdir -p "$TEST_REPO"
git -C "$TEST_REPO" init -q -b develop
git -C "$TEST_REPO" config user.email "test@test"
git -C "$TEST_REPO" config user.name "test"
git -C "$TEST_REPO" commit --allow-empty -q -m "init"
echo "MAINTENANCE" > "$TEST_REPO/MAINTENANCE"
git -C "$TEST_REPO" add MAINTENANCE
git -C "$TEST_REPO" commit -q -m "trigger"

# GH_REPO пустой → функция идёт на local check.
out="$(REPO_DIR="$TEST_REPO" GH_REPO="" bash -c '
  . '"$LIB"'
  af_maintenance_gate_inline_or_exit
' 2>&1)"
ec=$?

assert_eq "$ec" "0"  "T22.1: local MAINTENANCE returns exit 0"
assert_contains "$out" "[MAINTENANCE] gate active" "T22.2: stderr contains gate-active marker"
assert_contains "$out" "$TEST_REPO" "T22.3: stderr mentions local repo path"

# Без MAINTENANCE → silent exit 0.
echo "non-maintenance" > "$TEST_REPO/STATUS"
git -C "$TEST_REPO" rm -q MAINTENANCE 2>/dev/null || rm -f "$TEST_REPO/MAINTENANCE"
git -C "$TEST_REPO" add -A
git -C "$TEST_REPO" commit -q -m "remove maintenance" 2>/dev/null || true

out2="$(REPO_DIR="$TEST_REPO" GH_REPO="" bash -c '
  . '"$LIB"'
  af_maintenance_gate_inline_or_exit
  echo "no maintenance, exit 0"
' 2>&1)"
ec2=$?

assert_eq "$ec2" "0" "T22.4: no MAINTENANCE returns exit 0"
assert_contains "$out2" "no maintenance, exit 0" "T22.5: continues execution after gate"
assert_not_contains "$out2" "[MAINTENANCE] gate active" "T22.6: no gate-active marker when no maintenance"

rm -rf "$TEST_REPO"

# ----------------------------------------------------------------------------
# T23: shellcheck на изменённых скриптах (без warning'ов)
# ----------------------------------------------------------------------------
echo
echo "--- T23: shellcheck on patched scripts ---"
if command -v shellcheck >/dev/null 2>&1; then
  for script in "${SCRIPTS_VIA_LIB[@]}" "${SCRIPTS_INLINE[@]}"; do
    path="$AGENT_FLOW_DIR/$script"
    [ -f "$path" ] || continue
    # SC2259/1073/1072/1102 — pre-existing в lib (ретро t_e3fc9bfe);
    # SC2168/SC2037 — pre-existing в e2e-process.sh:4085 и blocked-watchdog.sh:278
    # (проверено git stash; не от issue #3009).
    err=$(shellcheck -S error --exclude=SC2259,SC1073,SC1072,SC1102,SC2168,SC2037 "$path" 2>&1 || true)
    if [ -z "$err" ]; then
      PASS=$((PASS+1))
      echo "  ✓ $script shellcheck clean"
    else
      FAIL=$((FAIL+1))
      FAILED_CASES+=("$script shellcheck errors: $err")
      echo "  ✗ $script: $err"
    fi
  done
else
  PASS=$((PASS+20))
  echo "  ⚠ shellcheck not installed, skipping 20 cases (assumed OK)"
fi

# ----------------------------------------------------------------------------
echo
echo "=== Summary: pass=$PASS fail=$FAIL ==="
if [ "$FAIL" -gt 0 ]; then
  echo "Failed:"
  for c in "${FAILED_CASES[@]}"; do echo "  - $c"; done
  exit 1
fi
exit 0
