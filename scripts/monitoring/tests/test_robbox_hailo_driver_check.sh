#!/usr/bin/env bash
# ============================================================================
# test_robbox_hailo_driver_check.sh — регресс-тест watchdog'а
# scripts/monitoring/robbox_hailo_driver_check.sh (issue #3090).
#
# Тесты мокают зависимости через PATH:
#   * dkms       — script с заранее заданным выводом `dkms status`
#   * hailortcli — script, который возвращает 0 + строку "Hailo-8" или fail
#   * uname      — фиксированный kernel release
#
# Также подменяем /lib/modules/<kernel>/{extra,updates}/hailo_pci.ko через
# создание фикстуры в tmp-dir и ROBBOX_HAILO_DEVICE_PATH.
#
# Сценарии (issue #3090 acceptance):
#   T1  OK:           dev=1, kern_match=1, dkms=1, scan=1 → exit 0, alert=0
#   T2  Alert root:   dev=0, kern_match=0 → exit 1, alert=1, reason mentions "issue #3090 root cause"
#   T3  Grace:        dev=0, kern_match=0, uptime<GRACE → exit 0, verdict=grace
#   T4  Infra error:  dkms missing → verdict=infra_error, exit 0
#   T5  Scan fail:    dev=1, kern_match=1, scan=0, uptime>GRACE → alert (runtime)
#   T6  Dkms missing but APK ok:           dev=1, kern_match=1 → ok (если dkms register OK)
#                                                    scan=0 → grace/alert (зависит от uptime)
#
# Usage: bash scripts/monitoring/tests/test_robbox_hailo_driver_check.sh
# Env:   VERBOSE=1 — печатать captured stdout/stderr при assert-fail
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT_UNDER_TEST="$(cd "$TESTS_DIR/.." && pwd)/robbox_hailo_driver_check.sh"

PASS=0
FAIL=0
FAILED_CASES=()
# Для тестов, которые читают metrics file ещё до T7 (T1).
metrics_content=""

assert_eq() {
  local got="$1" exp="$2" desc="$3"
  if [ "$got" = "$exp" ]; then
    PASS=$((PASS+1))
    echo "  ✓ $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (got='$got' expected='$exp')")
    echo "  ✗ $desc — got='$got' expected='$exp'"
    if [ "${VERBOSE:-0}" = "1" ]; then
      echo "    --- captured (last 30 lines) ---"
      echo "$LAST_STDERR" | tail -30
    fi
  fi
}

assert_contains() {
  local haystack="$1" needle="$2" desc="$3"
  if echo "$haystack" | grep -qF "$needle"; then
    PASS=$((PASS+1))
    echo "  ✓ $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (no substring '$needle')")
    echo "  ✗ $desc — expected substring '$needle' in output"
    echo "    --- captured ---"
    echo "$haystack" | tail -20
  fi
}

# --------------------------------------------------------------------------- #
# Sandbox: создаём tmp-dir с моками и fake lib/modules/device.
# --------------------------------------------------------------------------- #
sandbox_setup() {
  local kernel="$1"
  SANDBOX_DIR="$(mktemp -d -t robbox-hailo-test.XXXXXX)"
  export PATH="$SANDBOX_DIR/bin:$PATH"
  export ROBBOX_HAILO_METRICS_FILE="$SANDBOX_DIR/metrics.prom"
  export ROBBOX_HAILO_ALERT_LOG="$SANDBOX_DIR/alerts.log"
  export ROBBOX_HAILO_BOOT_LOG="$SANDBOX_DIR/boot.log"
  # GRACE=0 по умолчанию, чтобы uptime>GRACE и не попасть в grace.
  export ROBBOX_HAILO_GRACE_SECS="${ROBBOX_HAILO_GRACE_SECS:-0}"
  export ROBBOX_HAILO_DEVICE_PATH="$SANDBOX_DIR/dev/hailo0"
  export ROBBOX_HAILO_EXPECTED_VERSION="4.24.0"
  export ROBBOX_HAILO_LIB_MODULES_FAKEROOT="$SANDBOX_DIR"

  mkdir -p "$SANDBOX_DIR/bin" \
           "$SANDBOX_DIR/lib/modules/$kernel/extra" \
           "$SANDBOX_DIR/lib/modules/$kernel/updates" \
           "$SANDBOX_DIR/dev"

  # uname mock: выдаёт заданный kernel release
  cat > "$SANDBOX_DIR/bin/uname" <<EOF
#!/bin/sh
case "\$1" in
  -r) echo "$kernel" ;;
  *)  echo "Linux mocked-host" ;;
esac
EOF
  chmod +x "$SANDBOX_DIR/bin/uname"
}

sandbox_fake_libmodules() {
  local kernel="$1" mode="$2"  # "present" or "absent"
  if [ "$mode" = "present" ]; then
    : > "$SANDBOX_DIR/lib/modules/$kernel/extra/hailo_pci.ko"
  else
    rm -f "$SANDBOX_DIR/lib/modules/$kernel/extra/hailo_pci.ko" \
      "$SANDBOX_DIR/lib/modules/$kernel/updates/hailo_pci.ko"
  fi
}

sandbox_fake_device() {
  local mode="$1"  # "present" or "absent"
  if [ "$mode" = "present" ]; then
    : > "$ROBBOX_HAILO_DEVICE_PATH"
  else
    rm -f "$ROBBOX_HAILO_DEVICE_PATH"
  fi
}

sandbox_fake_dkms_status() {
  local mode="$1"  # "registered", "missing", "absent_tool"
  if [ "$mode" = "absent_tool" ]; then
    cat > "$SANDBOX_DIR/bin/dkms" <<'EOF'
#!/bin/sh
echo "dkms: command not found" >&2
exit 127
EOF
    chmod +x "$SANDBOX_DIR/bin/dkms"
    return
  fi
  if [ "$mode" = "registered" ]; then
    cat > "$SANDBOX_DIR/bin/dkms" <<'EOF'
#!/bin/sh
if [ "$1" = "status" ]; then
  echo "hailort-pcie-driver/4.24.0, 6.8.0-138-generic, x86_64: installed"
  exit 0
fi
echo "dkms mock" >&2
exit 0
EOF
  else
    cat > "$SANDBOX_DIR/bin/dkms" <<'EOF'
#!/bin/sh
if [ "$1" = "status" ]; then
  echo "(no modules registered)"
  exit 0
fi
echo "dkms mock" >&2
exit 0
EOF
  fi
  chmod +x "$SANDBOX_DIR/bin/dkms"
}

sandbox_fake_hailortcli() {
  local mode="$1"  # "see_device", "not_see", "absent_tool"
  if [ "$mode" = "absent_tool" ]; then
    cat > "$SANDBOX_DIR/bin/hailortcli" <<'EOF'
#!/bin/sh
echo "hailortcli: command not found" >&2
exit 127
EOF
    chmod +x "$SANDBOX_DIR/bin/hailortcli"
    return
  fi
  if [ "$mode" = "see_device" ]; then
    cat > "$SANDBOX_DIR/bin/hailortcli" <<'EOF'
#!/bin/sh
if [ "$1" = "scan" ]; then
  echo "Hailo-8 device found"
  exit 0
fi
echo "hailortcli mock" >&2
exit 0
EOF
  else
    cat > "$SANDBOX_DIR/bin/hailortcli" <<'EOF'
#!/bin/sh
if [ "$1" = "scan" ]; then
  echo "Hailo devices not found"
  exit 1
fi
echo "hailortcli mock" >&2
exit 1
EOF
  fi
  chmod +x "$SANDBOX_DIR/bin/hailortcli"
}

# /proc/uptime НЕ мокается (читается реальный). Grace-period
# контролируется через ROBBOX_HAILO_GRACE_SECS=0 ("не в grace").
sandbox_teardown() {
  if [ -n "${SANDBOX_DIR:-}" ] && [ -d "$SANDBOX_DIR" ]; then
    rm -rf "$SANDBOX_DIR"
  fi
  unset SANDBOX_DIR ROBBOX_HAILO_METRICS_FILE ROBBOX_HAILO_ALERT_LOG \
    ROBBOX_HAILO_BOOT_LOG ROBBOX_HAILO_GRACE_SECS ROBBOX_HAILO_DEVICE_PATH \
    ROBBOX_HAILO_EXPECTED_VERSION ROBBOX_HAILO_LIB_MODULES_FAKEROOT
}

# Запуск: env UPTIME_SECS контролирует effective uptime через /proc/uptime
# в sandboxed tree (но скрипт читает реальный /proc, не sandbox). Для
# честного теста grace-period будем выставлять ROBBOX_HAILO_GRACE_SECS=0
# или использовать реальный uptime который всегда > grace.
run_check() {
  local extra_env="$1"
  local out stderr exit_code
  # shellcheck disable=SC2086  # extra_env — набор VAR=value пар, не quote'ить
  env $extra_env bash "$SCRIPT_UNDER_TEST" --json >"$SANDBOX_DIR/stdout.txt" 2>"$SANDBOX_DIR/stderr.txt"
  exit_code=$?
  out="$(cat "$SANDBOX_DIR/stdout.txt" 2>/dev/null || true)"
  stderr="$(cat "$SANDBOX_DIR/stderr.txt" 2>/dev/null || true)"
  JSON_OUT="$out"
  EXIT_CODE="$exit_code"
  LAST_STDERR="$stderr"
  metrics_content="$(cat "$ROBBOX_HAILO_METRICS_FILE" 2>/dev/null || true)"
}

# --------------------------------------------------------------------------- #
# Tests
# --------------------------------------------------------------------------- #
echo "test_robbox_hailo_driver_check.sh — issue #3090 watchdog regression"
echo "  (script: $SCRIPT_UNDER_TEST)"
echo ""

# T1: OK — device, kernel match, dkms ok, scan ok
sandbox_setup "6.8.0-1065-raspi"
sandbox_fake_libmodules "6.8.0-1065-raspi" "present"
sandbox_fake_device "present"
sandbox_fake_dkms_status "registered"
sandbox_fake_hailortcli "see_device"
# GRACE=0 → не в grace никогда
run_check "ROBBOX_HAILO_GRACE_SECS=0"
assert_eq "$EXIT_CODE" "0" "T1: OK exit=0"
assert_contains "$JSON_OUT" '"verdict":"ok"' "T1: verdict=ok"
assert_contains "$JSON_OUT" '"device_present":1' "T1: device_present=1"
assert_contains "$JSON_OUT" '"kernel_module_match":1' "T1: kern_match=1"
assert_contains "$JSON_OUT" '"dkms_registered":1' "T1: dkms=1"
assert_contains "$JSON_OUT" '"hailortcli_scan":1' "T1: scan=1"
assert_contains "$JSON_OUT" '"alert":0' "T1: alert metric=0 (in json output)"
assert_contains "$metrics_content" "robbox_hailo_driver_alert 0" "T1: prometheus metric alert=0"
sandbox_teardown

# T2: Alert root cause — issue #3090: kern_match=0, dev=0
sandbox_setup "6.8.0-1065-raspi"
sandbox_fake_libmodules "6.8.0-1065-raspi" "absent"
sandbox_fake_device "absent"
sandbox_fake_dkms_status "registered"
sandbox_fake_hailortcli "absent_tool"
run_check "ROBBOX_HAILO_GRACE_SECS=0"
assert_eq "$EXIT_CODE" "1" "T2: root cause alert exit=1"
assert_contains "$JSON_OUT" '"verdict":"alert"' "T2: verdict=alert"
assert_contains "$JSON_OUT" '"device_present":0' "T2: device_present=0"
assert_contains "$JSON_OUT" '"kernel_module_match":0' "T2: kern_match=0 (root cause)"
assert_contains "$LAST_STDERR" "issue #3090 root cause" "T2: reason mentions root cause"
# Alert log записан?
assert_eq "$(test -s "$SANDBOX_DIR/alerts.log" && echo yes || echo no)" "yes" "T2: alerts.log has entries"
sandbox_teardown

# T3: Grace — root cause, но uptime < GRACE
sandbox_setup "6.8.0-1065-raspi"
sandbox_fake_libmodules "6.8.0-1065-raspi" "absent"
sandbox_fake_device "absent"
sandbox_fake_dkms_status "registered"
sandbox_fake_hailortcli "absent_tool"
# GRACE_SECS должен быть больше реального uptime (более 2M сек на билдере).
run_check "ROBBOX_HAILO_GRACE_SECS=999999999"
assert_eq "$EXIT_CODE" "0" "T3: grace exit=0"
assert_contains "$JSON_OUT" '"verdict":"grace"' "T3: verdict=grace"
# Alert log НЕ записан
assert_eq "$(test -s "$SANDBOX_DIR/alerts.log" && echo yes || echo no)" "no" "T3: no alert in grace"
sandbox_teardown

# T4: Infra error — dkms missing
sandbox_setup "6.8.0-1065-raspi"
sandbox_fake_libmodules "6.8.0-1065-raspi" "present"
sandbox_fake_device "present"
sandbox_fake_dkms_status "absent_tool"
sandbox_fake_hailortcli "see_device"
run_check "ROBBOX_HAILO_GRACE_SECS=0"
assert_eq "$EXIT_CODE" "0" "T4: infra_error exit=0 (not alert)"
assert_contains "$JSON_OUT" '"verdict":"infra_error"' "T4: verdict=infra_error"
assert_contains "$JSON_OUT" '"dkms_registered":-1' "T4: dkms metric=-1 (signal infra)"
sandbox_teardown

# T5: Scan fail (runtime) — dev+kern_match ok, но hailortcli не видит
sandbox_setup "6.8.0-1065-raspi"
sandbox_fake_libmodules "6.8.0-1065-raspi" "present"
sandbox_fake_device "present"
sandbox_fake_dkms_status "registered"
sandbox_fake_hailortcli "not_see"
run_check "ROBBOX_HAILO_GRACE_SECS=0"
assert_eq "$EXIT_CODE" "1" "T5: scan fail alert exit=1"
assert_contains "$JSON_OUT" '"verdict":"alert"' "T5: verdict=alert"
assert_contains "$JSON_OUT" '"hailortcli_scan":0' "T5: scan metric=0"
assert_contains "$LAST_STDERR" "hailortcli scan не видит чип" "T5: reason about runtime"
sandbox_teardown

# T6: warn-only — alert → exit 0
sandbox_setup "6.8.0-1065-raspi"
sandbox_fake_libmodules "6.8.0-1065-raspi" "absent"
sandbox_fake_device "absent"
sandbox_fake_dkms_status "registered"
sandbox_fake_hailortcli "absent_tool"
run_check "ROBBOX_HAILO_GRACE_SECS=0"
# повторно с --warn-only
out_w="$(bash "$SCRIPT_UNDER_TEST" --warn-only --json 2>/dev/null || true)"
assert_contains "$out_w" '"verdict":"alert"' "T6: --warn-only still reports verdict=alert"
# Exit code в --warn-only — перехватываем отдельно
bash "$SCRIPT_UNDER_TEST" --warn-only >/dev/null 2>&1
assert_eq "$?" "0" "T6: --warn-only exit=0 even on alert"
sandbox_teardown

# T7: metrics file write
sandbox_setup "6.8.0-1065-raspi"
sandbox_fake_libmodules "6.8.0-1065-raspi" "present"
sandbox_fake_device "present"
sandbox_fake_dkms_status "registered"
sandbox_fake_hailortcli "see_device"
run_check "ROBBOX_HAILO_GRACE_SECS=0"
assert_eq "$(test -s "$ROBBOX_HAILO_METRICS_FILE" && echo yes || echo no)" "yes" "T7: metrics file written"
# Prometheus textfile: проверим что есть ожидаемые gauge'и
assert_contains "$metrics_content" "robbox_hailo_driver_device_present 1" "T7: prometheus metric device_present=1"
assert_contains "$metrics_content" "robbox_hailo_driver_alert 0" "T7: prometheus metric alert=0"
sandbox_teardown

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