#!/bin/bash
# ============================================================================
# test_install_ensure_cleanup_cron.sh — ретро 13.08 t_04d73108
#
# Регресс-гард для ensure_cleanup_cron() в install.sh: cleanup-249.sh
# раскладывался на хост, но cron-job не создавался — stale round-ветки
# копились на origin (61-76/100-103). Функция должна идемпотентно
# регистрировать джоб в devops-профиле при каждом запуске install.sh.
#
# Проверяем:
#   1. джоб уже в jobs.json  -> OK (no-op), hermes НЕ вызывается;
#   2. джоба нет             -> ADD + hermes cron create вызывается;
#   3. --dry-run             -> [DRY], hermes НЕ вызывается.
#
# Run:
#   bash scripts/agent_flow/tests/test_install_ensure_cleanup_cron.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_SH="$TEST_DIR/../install.sh"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }

# --- фейковый hermes: ловим вызов cron create, НЕ трогаем реальный jobs.json
cat > "$WORK/hermes" <<'FAKE'
#!/bin/bash
echo "FAKE-HERMES cron create called with: $*" >> "$WORK/hermes-calls.log"
exit 0
FAKE
chmod +x "$WORK/hermes"

# --- извлекаем ensure_cleanup_cron() из install.sh --------------------------
awk '/^ensure_cleanup_cron\(\) \{/{f=1} f{print} f && /^\}/{exit}' \
    "$INSTALL_SH" > "$WORK/fn.sh"
[ -s "$WORK/fn.sh" ] || fail "ensure_cleanup_cron() not found in $INSTALL_SH"

# --- подменяем profile_dir на временный + PATH на фейковый hermes -----------
mkdir -p "$WORK/profiles/devops/cron"
make_runner() { # $1 = DRY_RUN (true/false)
    cat > "$WORK/run.sh" <<RUN
#!/bin/bash
export WORK="$WORK"
DRY_RUN=$1
REPO_DIR="/home/builder/hermes-share/rob_box_project"
PATH="$WORK:\$PATH"
sed -i 's#profile_dir="/home/builder/.hermes/profiles/devops"#profile_dir="$WORK/profiles/devops"#' "$WORK/fn.sh"
. "$WORK/fn.sh"
ensure_cleanup_cron
RUN
    chmod +x "$WORK/run.sh"
}

# --- TEST 1: джоб уже зарегистрирован -> OK, hermes НЕ вызывается -----------
cat > "$WORK/profiles/devops/cron/jobs.json" <<'JSON'
{
  "jobs": [
    {"id": "c999edf870aa", "name": "Agent Flow Cleanup 249", "script": "agent-flow-cleanup-249.sh", "no_agent": true}
  ],
  "updated_at": "2026-08-13T22:00:00+02:00"
}
JSON
make_runner false
OUT1=$("$WORK/run.sh" 2>&1)
echo "TEST1 (job exists):"; echo "$OUT1"
echo "$OUT1" | grep -q "already registered" || fail "expected 'already registered', got: $OUT1"
[ ! -f "$WORK/hermes-calls.log" ] || fail "hermes called although job exists"
echo "PASS: no-op, hermes not called"

# --- TEST 2: джоба нет -> ADD + hermes cron create вызывается ---------------
rm -f "$WORK/hermes-calls.log"
cat > "$WORK/profiles/devops/cron/jobs.json" <<'JSON'
{
  "jobs": [
    {"id": "49dd36041bdb", "name": "Agent Flow Scripts Drift", "script": "agent-flow-drift-detect.sh"}
  ],
  "updated_at": "2026-08-13T22:00:00+02:00"
}
JSON
OUT2=$("$WORK/run.sh" 2>&1)
echo "TEST2 (job missing):"; echo "$OUT2"
echo "$OUT2" | grep -q "cron job created" || fail "expected 'cron job created', got: $OUT2"
grep -q "Agent Flow Cleanup 249" "$WORK/hermes-calls.log" 2>/dev/null \
    || fail "hermes cron create not called with job name; log: $(cat "$WORK/hermes-calls.log" 2>/dev/null)"
echo "PASS: created, hermes called with job name"

# --- TEST 3: dry-run -> [DRY], hermes НЕ вызывается --------------------------
rm -f "$WORK/hermes-calls.log"
make_runner true
OUT3=$("$WORK/run.sh" 2>&1)
echo "TEST3 (dry-run):"; echo "$OUT3"
echo "$OUT3" | grep -q "\[DRY\]" || fail "expected [DRY] line, got: $OUT3"
[ ! -f "$WORK/hermes-calls.log" ] || fail "hermes called in dry-run"
echo "PASS: dry-run prints [DRY], hermes not called"

echo
echo "ALL 3 TESTS PASSED"
