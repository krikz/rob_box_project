#!/bin/bash
# ============================================================================
# test_install_ensure_install_daily_cron.sh — ретро 28.08 t_7ebdfce0
#
# Регресс-гард для ensure_install_daily_cron() в install.sh: после merge
# скриптов с cleanup-логикой (PR #1710) часть профилей отставала,
# пока оператор не запускал install.sh руками. Регрессия повторялась после
# каждого merge. Функция должна идемпотентно регистрировать interval-job
# (daily 03:00) в devops-профиле при каждом запуске install.sh.
#
# Проверяем:
#   1. interval-job уже в jobs.json + enabled  -> OK (no-op), hermes НЕ вызывается;
#   2. STALE once-job (state=completed, enabled=false) -> ADD + hermes вызывается;
#   3. джоба нет                              -> ADD + hermes cron create вызывается;
#   4. --dry-run                               -> [DRY], hermes НЕ вызывается;
#   5. wrapper-скрипт agent-flow-install-daily.sh создан в HERMES_SCRIPTS_DIR
#      при первом запуске (файл появляется на диске, executable);
#   6. второй запуск НЕ перезаписывает wrapper (idempotency wrapper'а).
#
# Run:
#   bash scripts/agent_flow/tests/test_install_ensure_install_daily_cron.sh
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

# --- извлекаем ensure_install_daily_cron() из install.sh -------------------
# Функция ссылается на SCRIPT_DIR (REPO_DIR/scripts/agent_flow), HERMES_SCRIPTS_DIR,
# REPO_DIR (для workdir) — пробрасываем минимальные стабы.
awk '/^ensure_install_daily_cron\(\) \{/{f=1} f{print} f && /^\}/{exit}' \
    "$INSTALL_SH" > "$WORK/fn.sh"
[ -s "$WORK/fn.sh" ] || fail "ensure_install_daily_cron() not found in $INSTALL_SH"

# --- подменяем profile_dir на временный + HERMES_SCRIPTS_DIR на tmp ---------
# В install.sh жёстко зашиты:
#   profile_dir="/home/builder/.hermes/profiles/devops"
#   cron_script_dst="/home/builder/.hermes/scripts/agent-flow-install-daily.sh"
# Подменим их через sed на пути в $WORK.
mkdir -p "$WORK/profiles/devops/cron"
mkdir -p "$WORK/hermes_scripts"
sed -i \
    -e 's#profile_dir="/home/builder/.hermes/profiles/devops"#profile_dir="$WORK/profiles/devops"#' \
    -e 's#"/home/builder/.hermes/scripts/$job_script"#"$WORK/hermes_scripts/$job_script"#' \
    "$WORK/fn.sh"

make_runner() { # $1 = DRY_RUN (true/false)
    cat > "$WORK/run.sh" <<RUN
#!/bin/bash
export WORK="$WORK"
DRY_RUN=$1
PATH="\$WORK:\$PATH"
REPO_DIR="/home/builder/hermes-share/rob_box_project"
SCRIPT_DIR="\$REPO_DIR/scripts/agent_flow"
HERMES_SCRIPTS_DIR="\$WORK/hermes_scripts"
. "\$WORK/fn.sh"
ensure_install_daily_cron
RUN
    chmod +x "$WORK/run.sh"
}

# --- TEST 1: interval-job уже зарегистрирован + enabled -> OK ---------------
cat > "$WORK/profiles/devops/cron/jobs.json" <<'JSON'
{
  "jobs": [
    {
      "id": "abc123",
      "name": "agent-flow-install-daily",
      "script": "agent-flow-install-daily.sh",
      "no_agent": true,
      "enabled": true,
      "state": "scheduled",
      "schedule": {"kind": "interval", "minutes": 1440, "display": "daily 03:00"}
    }
  ],
  "updated_at": "2026-08-28T13:00:00+02:00"
}
JSON
make_runner false
OUT1=$("$WORK/run.sh" 2>&1)
echo "TEST1 (interval-job exists, enabled):"; echo "$OUT1"
echo "$OUT1" | grep -q "already registered" || fail "expected 'already registered', got: $OUT1"
[ ! -f "$WORK/hermes-calls.log" ] || fail "hermes called although job exists"
echo "PASS: no-op, hermes not called"

# --- TEST 2: STALE once-job (state=completed, enabled=false) -> ADD ---------
rm -f "$WORK/hermes-calls.log"
rm -f "$WORK/hermes_scripts/agent-flow-install-daily.sh"
cat > "$WORK/profiles/devops/cron/jobs.json" <<'JSON'
{
  "jobs": [
    {
      "id": "deadbeef",
      "name": "agent-flow-install-daily",
      "script": "agent-flow-install-daily.sh",
      "no_agent": true,
      "enabled": false,
      "state": "completed",
      "schedule": {"kind": "once", "run_at": "2026-08-25T03:00:00+02:00", "display": "once"}
    }
  ],
  "updated_at": "2026-08-25T03:01:00+02:00"
}
JSON
make_runner false
OUT2=$("$WORK/run.sh" 2>&1)
echo "TEST2 (stale once-job only):"; echo "$OUT2"
echo "$OUT2" | grep -q "cron job created" || fail "expected 'cron job created', got: $OUT2"
grep -q "0 3 \* \* \*" "$WORK/hermes-calls.log" 2>/dev/null \
    || fail "hermes cron create not called with '0 3 * * *' schedule; log: $(cat "$WORK/hermes-calls.log" 2>/dev/null)"
echo "PASS: stale once-job coexists with new interval-job, hermes called with schedule"

# --- TEST 3: джоба нет вообще -> ADD + hermes cron create вызывается -------
rm -f "$WORK/hermes-calls.log"
rm -f "$WORK/hermes_scripts/agent-flow-install-daily.sh"
cat > "$WORK/profiles/devops/cron/jobs.json" <<'JSON'
{
  "jobs": [
    {"id": "other-job", "name": "Agent Flow Scripts Drift", "script": "agent-flow-drift-detect.sh"}
  ],
  "updated_at": "2026-08-28T13:00:00+02:00"
}
JSON
make_runner false
OUT3=$("$WORK/run.sh" 2>&1)
echo "TEST3 (no install-daily job):"; echo "$OUT3"
echo "$OUT3" | grep -q "cron job created" || fail "expected 'cron job created', got: $OUT3"
grep -q "agent-flow-install-daily.sh" "$WORK/hermes-calls.log" 2>/dev/null \
    || fail "hermes cron create not called with install-daily script; log: $(cat "$WORK/hermes-calls.log" 2>/dev/null)"
echo "PASS: created, hermes called with install-daily script"

# --- TEST 4: dry-run -> [DRY], hermes НЕ вызывается -------------------------
rm -f "$WORK/hermes-calls.log"
rm -f "$WORK/hermes_scripts/agent-flow-install-daily.sh"
cat > "$WORK/profiles/devops/cron/jobs.json" <<'JSON'
{
  "jobs": [],
  "updated_at": "2026-08-28T13:00:00+02:00"
}
JSON
make_runner true
OUT4=$("$WORK/run.sh" 2>&1)
echo "TEST4 (dry-run):"; echo "$OUT4"
echo "$OUT4" | grep -q "\[DRY\]" || fail "expected [DRY] line, got: $OUT4"
[ ! -f "$WORK/hermes-calls.log" ] || fail "hermes called in dry-run"
# wrapper НЕ должен создаваться в dry-run (он создаётся только в real-mode)
[ ! -f "$WORK/hermes_scripts/agent-flow-install-daily.sh" ] || fail "wrapper created in dry-run (must skip)"
echo "PASS: dry-run prints [DRY], hermes not called, wrapper not created"

# --- TEST 5: wrapper создан + executable + содержит 'install.sh' -----------
rm -f "$WORK/hermes_scripts/agent-flow-install-daily.sh"
cat > "$WORK/profiles/devops/cron/jobs.json" <<'JSON'
{
  "jobs": [],
  "updated_at": "2026-08-28T13:00:00+02:00"
}
JSON
make_runner false
"$WORK/run.sh" >/dev/null 2>&1
WRAPPER="$WORK/hermes_scripts/agent-flow-install-daily.sh"
[ -f "$WRAPPER" ] || fail "wrapper not created at $WRAPPER"
[ -x "$WRAPPER" ] || fail "wrapper not executable at $WRAPPER"
grep -q "install.sh" "$WRAPPER" || fail "wrapper does not invoke install.sh; got: $(cat "$WRAPPER")"
echo "PASS: wrapper exists, executable, calls install.sh"

# --- TEST 6: idempotency wrapper — второй запуск НЕ перезаписывает ---------
# Запишем «кастомный» wrapper (например, оператор добавил логирование), и
# убедимся что второй запуск install.sh его НЕ затёр.
cat > "$WRAPPER" <<'CUSTOM'
#!/bin/bash
# кастомный wrapper, который оператор сделал руками — НЕ ДОЛЖЕН перезаписаться
echo "CUSTOM WRAPPER (operator-touched)"
CUSTOM
chmod +x "$WRAPPER"
ORIG_MD5="$(md5sum "$WRAPPER" | awk '{print $1}')"
make_runner false
"$WORK/run.sh" >/dev/null 2>&1
NEW_MD5="$(md5sum "$WRAPPER" | awk '{print $1}')"
[ "$ORIG_MD5" = "$NEW_MD5" ] || fail "wrapper was overwritten on second run; orig=$ORIG_MD5 new=$NEW_MD5"
echo "PASS: wrapper not overwritten on second run (operator-edits preserved)"

echo
echo "ALL 6 TESTS PASSED"
