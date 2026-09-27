#!/bin/bash
# ============================================================================
# test_orphan_watchdog_scope.sh — agent-flow-orphan-watchdog cross-profile fix
# (ретро t_4c796522, 2026-09-26)
#
# Регресс-тест для fix'а "detector сканировал только devops/cron/jobs.json,
# ошибочно флагая watchdogs из agent-flow профиля как orphan".
#
# Scenarios:
#   C1. devops-only scan (старое поведение через USE_LEGACY_JOBS_FILE=1) →
#       needs-e2e-orphan-watchdog (в agent-flow профиле) ложно флагается
#       как orphan — регресс на ровно те issue, что watchdog зарепортил
#       25.09 (#3028 + #3029).
#   C2. cross-profile scan через PROFILES_GLOB → needs-e2e-orphan-watchdog
#       (в agent-flow профиле) найден, missing=1 (только not-registered-anywhere).
#   C3. NON_CRON_WATCHDOGS исключение — fail-streak-watchdog не флагается
#       даже при отсутствии cron-job (он инвокается из launcher'а).
#   C4. Реальный orphan (есть в EXPECTED[] install.sh, нет ни в одном
#       profiles/*/cron/jobs.json, НЕ в NON_CRON_WATCHDOGS) → missing=1.
#
# Run:
#   bash scripts/agent_flow/tests/test_orphan_watchdog_scope.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../agent-flow-orphan-watchdog.sh}"
SCRIPTS_DIR="${SCRIPTS_DIR:-$TEST_DIR/..}"

[ -f "$WATCHDOG_SH" ] || { echo "FAIL: $WATCHDOG_SH not found"; exit 1; }
[ -f "$SCRIPTS_DIR/install.sh" ] || { echo "FAIL: $SCRIPTS_DIR/install.sh not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required"; exit 1; }

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/bin"
mkdir -p "$WORK/hermes/profiles/devops/cron"
mkdir -p "$WORK/hermes/profiles/agent-flow/cron"

# Mock install.sh --list-files, чтобы изолировать тест от SOT-списка.
# Возвращаем набор watchdog'ов + fail-streak + «настоящий orphan».
# Watchdog ожидает $REPO_DIR/scripts/agent_flow/install.sh.
mkdir -p "$WORK/scripts/agent_flow"
cat > "$WORK/scripts/agent_flow/install.sh" <<'INSTALL_EOF'
#!/bin/bash
if [ "${1:-}" = "--list-files" ]; then
    cat <<'LIST'
agent-flow-orphan-watchdog.sh
agent-flow-blocked-watchdog.sh
agent-flow-stale-blocked-watchdog.sh
agent-flow-needs-e2e-orphan-watchdog.sh
agent-flow-e2e-fail-streak-watchdog.sh
agent-flow-not-registered-anywhere-watchdog.sh
LIST
fi
INSTALL_EOF
chmod +x "$WORK/scripts/agent_flow/install.sh"

# Helper: append a job entry to a profile's jobs.json (creates file on first call).
make_jobs() {
    local profile="$1"
    local script_name="$2"
    local jobs_file="$WORK/hermes/profiles/$profile/cron/jobs.json"
    python3 - "$jobs_file" "$script_name" <<'PYEOF'
import json, sys, os
jobs_file, script_name = sys.argv[1], sys.argv[2]
if os.path.exists(jobs_file):
    with open(jobs_file) as fh:
        d = json.load(fh)
else:
    d = {"jobs": []}
d.setdefault("jobs", []).append({
    "id": "aaaaaaaaaaaa",
    "name": "Mock " + script_name,
    "script": script_name,
    "enabled": True,
    "schedule": {"kind": "interval", "minutes": 60},
})
with open(jobs_file, "w") as fh:
    json.dump(d, fh)
PYEOF
}

# Reusable mock-gh — пишем пустышку (lockfile в начале не пускает, но
# на happy-path никогда не доходит до gh; на error-path через DRY_RUN).
cat > "$WORK/bin/gh" <<'EOF'
#!/bin/bash
exit 0
EOF
chmod +x "$WORK/bin/gh"

# Default mock для «успешного» сценария.
_log_marker() { echo "[TEST] $*"; }

run_watchdog() {
    # $1 = scenario_name, $@ = extra env
    local name="$1"
    shift
    local out
    out=$(env -i HOME="$WORK" PATH="$WORK/bin:$PATH" \
        HERMES_HOME="$WORK/hermes" \
        REPO_DIR="$WORK" \
        ALERT_LOG="$WORK/alert.log" \
        LOCK_FILE="$WORK/wd.lock" \
        LOG_FILE="$WORK/wd.log" \
        DRY_RUN=true \
        STALE_DEDUP_HOURS=999999 \
        GH_REPO=krikz/test \
        GH_CONFIG_DIR="$WORK" \
        bash "$WATCHDOG_SH" 2>&1)
    local rc=$?
    _log_marker "Scenario $name: exit=$rc, output:"
    echo "$out" | sed 's/^/    /'
    echo "$out" | grep -E '^watchdog-orphan-detector:'
    echo "$rc"
}

# === C1: только devops — нужен JOBS_FILE override (backcompat) ===
_log_marker "========== C1: devops-only scan (legacy JOBS_FILE) =========="
make_jobs devops "agent-flow-blocked-watchdog.sh"
make_jobs devops "agent-flow-stale-blocked-watchdog.sh"
# needs-e2e и audit — в agent-flow (НЕ в devops) → будут флагануты как orphan.
make_jobs agent-flow "agent-flow-needs-e2e-orphan-watchdog.sh"
make_jobs agent-flow "agent-flow-orphan-audit.sh"

out_c1=$(env -i HOME="$WORK" PATH="$WORK/bin:$PATH" \
    HERMES_HOME="$WORK/hermes" \
    REPO_DIR="$WORK" \
    ALERT_LOG="$WORK/alert.log" \
    LOCK_FILE="$WORK/wd.lock" \
    LOG_FILE="$WORK/wd.log" \
    DRY_RUN=true \
    STALE_DEDUP_HOURS=999999 \
    USE_LEGACY_JOBS_FILE=1 \
    JOBS_FILE="$WORK/hermes/profiles/devops/cron/jobs.json" \
    bash "$WATCHDOG_SH" 2>&1)
rc_c1=$?
echo "$out_c1" | grep '^watchdog-orphan-detector:' || true
_log_marker "C1 summary: $out_c1 (exit=$rc_c1)"

# Очищаем jobs.json перед C2 — убеждаемся, что PROFILES_GLOB работает с нуля.
rm -f "$WORK"/hermes/profiles/*/cron/jobs.json

# === C2: cross-profile scan — все 4 «легитимных» watchdog'а registered ===
_log_marker "========== C2: cross-profile scan (PROFILES_GLOB default) =========="
make_jobs devops "agent-flow-blocked-watchdog.sh"
make_jobs devops "agent-flow-stale-blocked-watchdog.sh"
make_jobs agent-flow "agent-flow-needs-e2e-orphan-watchdog.sh"
# fail-streak — НЕ регистрируем, но он в NON_CRON_WATCHDOGS → должен skip'нуться.
# not-registered-anywhere — НЕ регистрируем, и его нет в NON_CRON_WATCHDOGS
#   → должен остаться как orphan (для C4 проверки).

out_c2=$(env -i HOME="$WORK" PATH="$WORK/bin:$PATH" \
    HERMES_HOME="$WORK/hermes" \
    REPO_DIR="$WORK" \
    ALERT_LOG="$WORK/alert.log" \
    LOCK_FILE="$WORK/wd.lock" \
    LOG_FILE="$WORK/wd.log" \
    DRY_RUN=false \
    STALE_DEDUP_HOURS=999999 \
    bash "$WATCHDOG_SH" 2>&1)
rc_c2=$?
echo "$out_c2" | grep '^watchdog-orphan-detector:' || true
_log_marker "C2 summary: $out_c2 (exit=$rc_c2)"

# === Assertions ===

fail_count=0
pass_count=0

assert_eq() {
    # $1=name $2=expected $3=actual
    if [ "$2" = "$3" ]; then
        echo "  PASS  $1: expected=$2 got=$3"
        pass_count=$((pass_count + 1))
    else
        echo "  FAIL  $1: expected=$2 got=$3"
        fail_count=$((fail_count + 1))
    fi
}

# C1: legacy devops-only → needs-e2e + not-registered должны быть
#     флагануты = 2 missing (fail-streak и orphan-audit не в скоупе по glob-у).
#     blocked + stale-blocked зарегистрированы в devops → with_cron=2.
#     C1 НЕ пропускает NON_CRON (это проверка back-compat JOBS_FILE-режима —
#     в реальной жизни до t_4c796522 так и было: всё, чего нет в devops,
#     флагалось как orphan).
c1_missing=$(echo "$out_c1" | grep -oE 'missing=[0-9]+' | head -1 | cut -d= -f2)
assert_eq "C1 missing (legacy devops-only)" "2" "$c1_missing"

c1_with=$(echo "$out_c1" | grep -oE 'with_cron=[0-9]+' | head -1 | cut -d= -f2)
assert_eq "C1 with_cron (devops-only registered)" "2" "$c1_with"

# C2: cross-profile + NON_CRON_WATCHDOGS-skip →
#     - blocked (devops)         ✓
#     - stale-blocked (devops)   ✓
#     - needs-e2e (agent-flow)   ✓ (теперь видим)
#     - audit (agent-flow)       ✓ (теперь видим)
#     - fail-streak              skip (NON_CRON_WATCHDOGS)
#     - not-registered           missing (единственный настоящий orphan)
c2_missing=$(echo "$out_c2" | grep -oE 'missing=[0-9]+' | head -1 | cut -d= -f2)
assert_eq "C2 missing (cross-profile + NON_CRON)" "1" "$c2_missing"

c2_with=$(echo "$out_c2" | grep -oE 'with_cron=[0-9]+' | head -1 | cut -d= -f2)
assert_eq "C2 with_cron (4 registered: blocked, stale-blocked, needs-e2e + fail-streak skipped by NON_CRON)" "3" "$c2_with"

c2_scanned=$(echo "$out_c2" | grep -oE 'scanned=[0-9]+' | head -1 | cut -d= -f2)
# install.sh вернул 6 watchdog'ов: orphan-detector self-skip + fail-streak NON_CRON → scanned = 4.
assert_eq "C2 scanned (6 - self - fail-streak = 4)" "4" "$c2_scanned"

# C2 exit code: 2 (есть missing).
assert_eq "C2 exit code (1 missing → alert)" "2" "$rc_c2"

# C1 exit code: 2 (есть missing).
assert_eq "C1 exit code (2 missing → alert)" "2" "$rc_c1"

# Проверяем, что C2 не упомянул needs-e2e / audit / fail-streak в missing
# (они либо найдены, либо skip'нуты NON_CRON_WATCHDOGS).
echo
echo "  --- C2 alert.log content ---"
cat "$WORK/alert.log" 2>/dev/null | sed 's/^/    /' || echo "    (empty)"
echo "  --- C2 alert.log expected MISSING ---"
grep MISSING "$WORK/alert.log" 2>/dev/null | sed 's/^/    /' || echo "    (no MISSING — expected!)"
if grep -q "needs-e2e-orphan-watchdog" "$WORK/alert.log" 2>/dev/null; then
    echo "  FAIL  C2 alert.log should NOT contain needs-e2e-orphan-watchdog (it's in agent-flow profile)"
    fail_count=$((fail_count + 1))
else
    echo "  PASS  C2 alert.log clean of needs-e2e-orphan-watchdog"
    pass_count=$((pass_count + 1))
fi
if grep -q "e2e-fail-streak-watchdog" "$WORK/alert.log" 2>/dev/null; then
    echo "  FAIL  C2 alert.log should NOT contain e2e-fail-streak-watchdog (NON_CRON_WATCHDOGS)"
    fail_count=$((fail_count + 1))
else
    echo "  PASS  C2 alert.log clean of e2e-fail-streak-watchdog"
    pass_count=$((pass_count + 1))
fi
if grep -q "not-registered-anywhere-watchdog" "$WORK/alert.log" 2>/dev/null; then
    echo "  PASS  C2 alert.log DOES contain not-registered-anywhere (true orphan)"
    pass_count=$((pass_count + 1))
else
    echo "  FAIL  C2 alert.log missing not-registered-anywhere (should be flagged)"
    fail_count=$((fail_count + 1))
fi

echo
echo "=== test_orphan_watchdog_scope.sh: passed=$pass_count failed=$fail_count ==="
[ "$fail_count" -eq 0 ]