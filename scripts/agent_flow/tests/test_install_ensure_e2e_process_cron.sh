#!/bin/bash
# ============================================================================
# test_install_ensure_e2e_process_cron.sh — ретро 23.08+25.08 t_98bb3a1d/t_24e645e7
#
# Регресс-гард для ensure_e2e_process_cron() в install.sh: launcher
# agent-flow-e2e-process-launcher.sh раскладывался install.sh (commit
# bd7e509d), но cron-job не создавался — он создавался вручную через
# `hermes cron create 'once in 20m'`. После первого тика once-job
# переходил в state=completed и больше НЕ перезапускался (60ч+ простоя
# rotation). Функция должна идемпотентно регистрировать interval-job
# (every 20m) в devops-профиле при каждом запуске install.sh.
#
# Проверяем:
#   1. interval-job уже в jobs.json + enabled  -> OK (no-op), hermes НЕ вызывается;
#   2. только STALE once-job (state=completed, enabled=false) -> ADD + hermes вызывается;
#   3. джоба нет                              -> ADD + hermes cron create вызывается;
#   4. --dry-run                               -> [DRY], hermes НЕ вызывается.
#   5. md5sum verify (verify_three_copies_md5sum) для 4-х копий launcher'а:
#      - все совпадают -> OK
#      - drift         -> ERROR с понятным выводом
#
# Run:
#   bash scripts/agent_flow/tests/test_install_ensure_e2e_process_cron.sh
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

# --- извлекаем ensure_e2e_process_cron() из install.sh ----------------------
awk '/^ensure_e2e_process_cron\(\) \{/{f=1} f{print} f && /^\}/{exit}' \
    "$INSTALL_SH" > "$WORK/fn.sh"
[ -s "$WORK/fn.sh" ] || fail "ensure_e2e_process_cron() not found in $INSTALL_SH"

# --- подменяем profile_dir на временный + PATH на фейковый hermes -----------
mkdir -p "$WORK/profiles/devops/cron"
make_runner() { # $1 = DRY_RUN (true/false)
    cat > "$WORK/run.sh" <<RUN
#!/bin/bash
export WORK="$WORK"
DRY_RUN=$1
REPO_DIR="/home/builder/hermes-share/rob_box_project"
PATH="\$WORK:\$PATH"
sed -i 's#profile_dir="/home/builder/.hermes/profiles/devops"#profile_dir="\$WORK/profiles/devops"#' "\$WORK/fn.sh"
. "\$WORK/fn.sh"
ensure_e2e_process_cron
RUN
    chmod +x "$WORK/run.sh"
}

# --- TEST 1: interval-job уже зарегистрирован + enabled -> OK ---------------
cat > "$WORK/profiles/devops/cron/jobs.json" <<'JSON'
{
  "jobs": [
    {
      "id": "84864db04347",
      "name": "e2e-process auto-rotation",
      "script": "agent-flow-e2e-process-launcher.sh",
      "no_agent": true,
      "enabled": true,
      "state": "scheduled",
      "schedule": {"kind": "interval", "minutes": 20, "display": "every 20m"}
    }
  ],
  "updated_at": "2026-08-25T23:45:00+02:00"
}
JSON
make_runner false
OUT1=$("$WORK/run.sh" 2>&1)
echo "TEST1 (interval-job exists, enabled):"; echo "$OUT1"
echo "$OUT1" | grep -q "already registered" || fail "expected 'already registered', got: $OUT1"
[ ! -f "$WORK/hermes-calls.log" ] || fail "hermes called although job exists"
echo "PASS: no-op, hermes not called"

# --- TEST 2: STALE once-job (state=completed, enabled=false) -> ADD ---------
# Это критичный кейс: в реальной системе лежит b6706eaf432d (once, completed,
# enabled=false) — ensure_e2e_process_cron() должен зарегистрировать НОВЫЙ
# interval-job рядом со старым, не трогая последний.
rm -f "$WORK/hermes-calls.log"
cat > "$WORK/profiles/devops/cron/jobs.json" <<'JSON'
{
  "jobs": [
    {
      "id": "b6706eaf432d",
      "name": "e2e-process auto-rotation",
      "script": "agent-flow-e2e-process-launcher.sh",
      "no_agent": true,
      "enabled": false,
      "state": "completed",
      "schedule": {"kind": "once", "run_at": "2026-08-23T20:37:40+02:00", "display": "once in 20m"}
    }
  ],
  "updated_at": "2026-08-23T20:38:38+02:00"
}
JSON
make_runner false
OUT2=$("$WORK/run.sh" 2>&1)
echo "TEST2 (stale once-job only):"; echo "$OUT2"
echo "$OUT2" | grep -q "cron job created" || fail "expected 'cron job created', got: $OUT2"
grep -q "every 20m" "$WORK/hermes-calls.log" 2>/dev/null \
    || fail "hermes cron create not called with 'every 20m' schedule; log: $(cat "$WORK/hermes-calls.log" 2>/dev/null)"
echo "PASS: stale once-job coexists with new interval-job, hermes called with schedule"

# --- TEST 3: джоба нет вообще -> ADD + hermes cron create вызывается -------
rm -f "$WORK/hermes-calls.log"
cat > "$WORK/profiles/devops/cron/jobs.json" <<'JSON'
{
  "jobs": [
    {"id": "49dd36041bdb", "name": "Agent Flow Scripts Drift", "script": "agent-flow-drift-detect.sh"}
  ],
  "updated_at": "2026-08-12T05:38:00+02:00"
}
JSON
make_runner false
OUT3=$("$WORK/run.sh" 2>&1)
echo "TEST3 (no e2e job at all):"; echo "$OUT3"
echo "$OUT3" | grep -q "cron job created" || fail "expected 'cron job created', got: $OUT3"
grep -q "agent-flow-e2e-process-launcher.sh" "$WORK/hermes-calls.log" 2>/dev/null \
    || fail "hermes cron create not called with launcher script; log: $(cat "$WORK/hermes-calls.log" 2>/dev/null)"
echo "PASS: created, hermes called with launcher script"

# --- TEST 4: dry-run -> [DRY], hermes НЕ вызывается -------------------------
rm -f "$WORK/hermes-calls.log"
make_runner true
OUT4=$("$WORK/run.sh" 2>&1)
echo "TEST4 (dry-run):"; echo "$OUT4"
echo "$OUT4" | grep -q "\[DRY\]" || fail "expected [DRY] line, got: $OUT4"
[ ! -f "$WORK/hermes-calls.log" ] || fail "hermes called in dry-run"
echo "PASS: dry-run prints [DRY], hermes not called"

# --- TEST 5: md5sum verify для 4-х копий launcher'а -------------------------
# Извлекаем verify_three_copies_md5sum() из install.sh и подсовываем 4 пути
# на tmp-файлы с одинаковым/разным содержимым.
awk '/^verify_three_copies_md5sum\(\) \{/{f=1} f{print} f && /^\}/{exit}' \
    "$INSTALL_SH" > "$WORK/verify_fn.sh"
[ -s "$WORK/verify_fn.sh" ] || fail "verify_three_copies_md5sum() not found in $INSTALL_SH"

mkdir -p "$WORK/copies"
# 3 одинаковых копии + 1 отличающаяся (drift)
echo "identical content for launcher" > "$WORK/copies/agent-flow_e2e_process_launcher.sh"
cp "$WORK/copies/agent-flow_e2e_process_launcher.sh" "$WORK/copies/architect_e2e_process_launcher.sh"
cp "$WORK/copies/agent-flow_e2e_process_launcher.sh" "$WORK/copies/devops_e2e_process_launcher.sh"
echo "DRIFTED content" > "$WORK/copies/hermes_e2e_process_launcher.sh"

cat > "$WORK/verify_run.sh" <<RUN
#!/bin/bash
. "$WORK/verify_fn.sh"
verify_three_copies_md5sum "agent-flow-e2e-process-launcher.sh" \\
    "$WORK/copies/agent-flow_e2e_process_launcher.sh" \\
    "$WORK/copies/architect_e2e_process_launcher.sh" \\
    "$WORK/copies/devops_e2e_process_launcher.sh" \\
    "$WORK/copies/hermes_e2e_process_launcher.sh"
RC=\$?
exit \$RC
RUN
chmod +x "$WORK/verify_run.sh"

OUT5=$("$WORK/verify_run.sh" 2>&1)
RC5=$?
echo "TEST5 (md5sum drift):"; echo "$OUT5"
[ "$RC5" -ne 0 ] || fail "expected non-zero exit code on md5sum drift, got 0"
echo "$OUT5" | grep -q "drift detected" || fail "expected 'drift detected' message, got: $OUT5"
echo "PASS: md5sum drift detected, function returns non-zero"

# Теперь подменим 4-й файл чтобы все 4 совпадали — должен быть OK.
cp "$WORK/copies/agent-flow_e2e_process_launcher.sh" "$WORK/copies/hermes_e2e_process_launcher.sh"
OUT5B=$("$WORK/verify_run.sh" 2>&1)
RC5B=$?
echo "TEST5b (md5sum all match):"; echo "$OUT5B"
[ "$RC5B" -eq 0 ] || fail "expected zero exit code on all-match, got $RC5B"
echo "$OUT5B" | grep -q "across 4 copies" || fail "expected 'across 4 copies' message, got: $OUT5B"
echo "PASS: md5sum verify OK when all 4 copies match"

echo
echo "ALL 5 TESTS PASSED"
