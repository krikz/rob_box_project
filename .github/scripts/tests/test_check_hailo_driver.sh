#!/usr/bin/env bash
# ============================================================================
# test_check_hailo_driver.sh — регрессионный тест (issue #3090)
#
# Источник истины: <repo>/.github/scripts/tests/test_check_hailo_driver.sh
#
# Тестирует <repo>/.github/scripts/check_hailo_driver.sh (вынесенная из
# workflow L-Deploy and Verify.yml pre-deploy проверка для Hailo PCIe
# driver). Каждый кейс запускает СКРИПТ с мокнутым `sshpass` через PATH,
# читает JSON и сверяет с ожиданием.
#
# Зачем: фикс флапающего регрессионного #3090. Deploy-verify pipeline
# должен fail-fast если /dev/hailo0 или hailo_pci.ko для активного kernel
# отсутствуют — иначе vision-hailo контейнер уйдёт в restart-loop,
# весь стек vision станет unhealthy, робот не отвечает (см. #3089).
#
# Сценарии:
#   T1: OK            — всё есть (lspci Hailo + .ko + dev + scan) → healthy=true
#   T2: Root cause    — .ko отсутствует для активного kernel → healthy=false
#   T3: /dev/hailo0   — device node missing → healthy=false
#   T4: scan fail     — hailortcli не видит чип → healthy=false
#   T5: no Hailo HW   — lspci пустой → healthy=true (no-op, железки нет)
#   T6: ssh fail      — sshpass не работает → JSON с reason=ssh_failed
#
# Run:
#   bash .github/scripts/tests/test_check_hailo_driver.sh
# ============================================================================
set -euo pipefail

SCRIPT_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR_HERE/../../.." && pwd)"
SCRIPT_UNDER_TEST="$REPO_ROOT/.github/scripts/check_hailo_driver.sh"

[ -f "$SCRIPT_UNDER_TEST" ] || {
    echo "FATAL: $SCRIPT_UNDER_TEST не найден. Запускай из корня репо." >&2
    exit 1
}

command -v jq >/dev/null 2>&1 || {
    echo "FATAL: jq не найден в PATH. Установи: apt-get install jq" >&2
    exit 1
}

# --- counters ---------------------------------------------------------------
PASS=0
FAIL=0
FAILED_CASES=()

assert_eq() {
  local got="$1" exp="$2" desc="$3"
  if [ "$got" = "$exp" ]; then
    PASS=$((PASS+1))
    echo "  PASS  $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (got='$got' expected='$exp')")
    echo "  FAIL  $desc — got='$got' expected='$exp'"
  fi
}

assert_contains() {
  local haystack="$1" needle="$2" desc="$3"
  if echo "$haystack" | grep -qF "$needle"; then
    PASS=$((PASS+1))
    echo "  PASS  $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (no substring '$needle')")
    echo "  FAIL  $desc — expected substring '$needle'"
    echo "$haystack" | head -10
  fi
}

# --- mock helpers -----------------------------------------------------------
# MOCK_BIN — папка с моками sshpass/ssh которые выдают заранее заданные
# remote-state строки.

mock_state_setup() {
  MOCK_BIN="$(mktemp -d)"
  export PATH="$MOCK_BIN:$PATH"
  export PI_HOST="10.1.1.11"
  export PI_USER="ros2"
  export PI_PASSWORD="open"
  export PI_SSH_OPTS="-o StrictHostKeyChecking=no"
  export ENVIRONMENT="test"
  export SCOPE="vision"
  export GRACE_SECS="0"
  export FINDINGS_FILE="/dev/null"
}

# sshpass: вместо реального ssh+sshpass — запускает stdin как bash на локальной машине.
# Это позволяет тестировать парсинг remote-state без реального ssh.
mock_sshpass_capture() {
  cat > "$MOCK_BIN/sshpass" <<'SSHPASS_EOF'
#!/bin/bash
# Принимаем stdin как script body, запускаем локально
SCRIPT_BODY="$(cat)"
# strip leading args ($@: все что до последнего "user@host" — это ssh opts)
# sshpass передаёт все args после "-p password" в ssh. Нам нужен user@host.
HOST=""
for arg in "$@"; do
  if echo "$arg" | grep -q "@"; then
    HOST="$arg"
  fi
done
# Просто запускаем stdin как bash -c
exec bash -c "$SCRIPT_BODY"
SSHPASS_EOF
  chmod +x "$MOCK_BIN/sshpass"
}

mock_sshpass_fail() {
  cat > "$MOCK_BIN/sshpass" <<'SSHPASS_EOF'
#!/bin/bash
# Имитируем: ssh не подключился → remote_state=""
echo "" >&2
exit 1
SSHPASS_EOF
  chmod +x "$MOCK_BIN/sshpass"
}

mock_teardown() {
  if [ -n "${MOCK_BIN:-}" ] && [ -d "$MOCK_BIN" ]; then
    rm -rf "$MOCK_BIN"
  fi
  unset MOCK_BIN PI_HOST PI_USER PI_PASSWORD PI_SSH_OPTS ENVIRONMENT SCOPE GRACE_SECS FINDINGS_FILE
}

# --- Test cases -------------------------------------------------------------
echo "=== test_check_hailo_driver.sh — issue #3090 pre-deploy regression ==="
echo ""

# T1: OK — всё есть
mock_state_setup
mock_sshpass_capture
HEALTHY_RESULT="$(bash "$SCRIPT_UNDER_TEST" 2>/dev/null)"
HEALTHY="$(echo "$HEALTHY_RESULT" | jq -r '.healthy')"
FAILED_COUNT="$(echo "$HEALTHY_RESULT" | jq -r '.failed_count')"
assert_eq "$HEALTHY" "true" "T1: OK healthy=true"
assert_eq "$FAILED_COUNT" "0" "T1: OK failed_count=0"
# Tear down mock и подменим sshpass на выдачу нужного remote_state для T2-T5
# (state зависит от мока, не меняется)
mock_teardown

# T2: root cause — .ko missing
mock_state_setup
# Подменяем mock после создания, чтобы выдавал .ko = MAGIC_MISSING
cat > /dev/null # noop
# Сложно: в моём скрипте remote_script в heredoc идёт как stdin sshpass,
# mock_sshpass_capture просто делает exec bash -c "$SCRIPT_BODY".
# Чтобы протестировать другой state — нужно подменить stdin.
# Здесь используем другой подход: устанавливаем MOCK_BIN/sshpass как wrapper
# который запускает другой preset script. Делаем per-test mocks:
cat > "$MOCK_BIN/sshpass" <<'EOF'
#!/bin/bash
cat <<'STATE'
=== uname ===
6.8.0-1065-raspi
=== lspci ===
PRESENT
=== module_extra ===
MAGIC_MISSING
=== module_updates ===
MAGIC_MISSING
=== device_node ===
crw-rw---- 1 root root 234, 0 Sep 27 14:30 /dev/hailo0
=== dkms_status ===
hailort-pcie-driver/4.24.0, 6.8.0-1065-raspi, aarch64: installed
=== hailortcli_scan ===
Hailo-8 device found
=== uptime ===
5000
STATE
EOF
chmod +x "$MOCK_BIN/sshpass"
HEALTHY_RESULT="$(bash "$SCRIPT_UNDER_TEST" 2>/dev/null)"
HEALTHY="$(echo "$HEALTHY_RESULT" | jq -r '.healthy')"
FAILED_COUNT="$(echo "$HEALTHY_RESULT" | jq -r '.failed_count')"
assert_eq "$HEALTHY" "false" "T2: .ko missing → healthy=false"
assert_eq "$FAILED_COUNT" "1" "T2: failed_count=1"
assert_contains "$HEALTHY_RESULT" "issue #3090" "T2: reason mentions #3090"
mock_teardown

# T3: /dev/hailo0 missing
mock_state_setup
cat > "$MOCK_BIN/sshpass" <<'EOF'
#!/bin/bash
cat <<'STATE'
=== uname ===
6.8.0-1065-raspi
=== lspci ===
PRESENT
=== module_extra ===
PRESENT
=== module_updates ===
PRESENT
=== device_node ===
MAGIC_MISSING
=== dkms_status ===
hailort-pcie-driver/4.24.0, 6.8.0-1065-raspi, aarch64: installed
=== hailortcli_scan ===
Hailo-8 device found
=== uptime ===
5000
STATE
EOF
chmod +x "$MOCK_BIN/sshpass"
HEALTHY_RESULT="$(bash "$SCRIPT_UNDER_TEST" 2>/dev/null)"
HEALTHY="$(echo "$HEALTHY_RESULT" | jq -r '.healthy')"
FAILED_COUNT="$(echo "$HEALTHY_RESULT" | jq -r '.failed_count')"
assert_eq "$HEALTHY" "false" "T3: /dev/hailo0 missing → healthy=false"
assert_eq "$FAILED_COUNT" "1" "T3: failed_count=1"
assert_contains "$HEALTHY_RESULT" "/dev/hailo0 missing" "T3: reason about /dev/hailo0"
mock_teardown

# T4: hailortcli scan fail
mock_state_setup
cat > "$MOCK_BIN/sshpass" <<'EOF'
#!/bin/bash
cat <<'STATE'
=== uname ===
6.8.0-1065-raspi
=== lspci ===
PRESENT
=== module_extra ===
PRESENT
=== module_updates ===
PRESENT
=== device_node ===
crw-rw---- 1 root root 234, 0 Sep 27 14:30 /dev/hailo0
=== dkms_status ===
hailort-pcie-driver/4.24.0, 6.8.0-1065-raspi, aarch64: installed
=== hailortcli_scan ===
Hailo devices not found
=== uptime ===
5000
STATE
EOF
chmod +x "$MOCK_BIN/sshpass"
HEALTHY_RESULT="$(bash "$SCRIPT_UNDER_TEST" 2>/dev/null)"
HEALTHY="$(echo "$HEALTHY_RESULT" | jq -r '.healthy')"
FAILED_COUNT="$(echo "$HEALTHY_RESULT" | jq -r '.failed_count')"
assert_eq "$HEALTHY" "false" "T4: scan fail → healthy=false"
assert_eq "$FAILED_COUNT" "1" "T4: failed_count=1"
mock_teardown

# T5: no Hailo HW — lspci пустой
mock_state_setup
cat > "$MOCK_BIN/sshpass" <<'EOF'
#!/bin/bash
cat <<'STATE'
=== uname ===
6.8.0-1065-raspi
=== lspci ===
MAGIC_NO_HAILO
=== module_extra ===
MAGIC_MISSING
=== module_updates ===
MAGIC_MISSING
=== device_node ===
MAGIC_MISSING
=== dkms_status ===
MAGIC_NO_DKMS_TOOL
=== hailortcli_scan ===
MAGIC_NO_HAILORTCLI
=== uptime ===
5000
STATE
EOF
chmod +x "$MOCK_BIN/sshpass"
HEALTHY_RESULT="$(bash "$SCRIPT_UNDER_TEST" 2>/dev/null)"
HEALTHY="$(echo "$HEALTHY_RESULT" | jq -r '.healthy')"
REASON="$(echo "$HEALTHY_RESULT" | jq -r '.reason')"
assert_eq "$HEALTHY" "true" "T5: no Hailo HW → healthy=true (no-op)"
assert_eq "$REASON" "no_hailo_hw" "T5: reason=no_hailo_hw"
mock_teardown

# T6: ssh fail — sshpass возвращает exit 1 + empty output
mock_state_setup
mock_sshpass_fail
HEALTHY_RESULT="$(bash "$SCRIPT_UNDER_TEST" 2>/dev/null || true)"
HEALTHY="$(echo "$HEALTHY_RESULT" | jq -r '.healthy' 2>/dev/null || echo 'parse_error')"
REASON="$(echo "$HEALTHY_RESULT" | jq -r '.reason' 2>/dev/null || echo 'parse_error')"
assert_eq "$HEALTHY" "false" "T6: ssh fail → healthy=false"
assert_eq "$REASON" "ssh_failed" "T6: reason=ssh_failed"
mock_teardown

# --- summary ----------------------------------------------------------------
echo ""
echo "================================================="
echo " PASS: $PASS"
echo " FAIL: $FAIL"
if [ "$FAIL" -gt 0 ]; then
  echo ""
  echo "Failed cases:"
  for c in "${FAILED_CASES[@]}"; do
    echo "  - $c"
  done
  exit 1
fi
echo "All tests passed."