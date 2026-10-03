#!/usr/bin/env bash
# ============================================================================
# robbox_hailo_driver_check.sh — watchdog для /dev/hailo0 + DKMS-состояния
# драйвера hailo_pci (issue #3090).
#
# Single-source-of-truth скрипт, вызывается из systemd timer
# `robbox-hailo-driver.timer` (см. setup_vision_pi.sh →
# setup_hailo_driver_monitor). Также может запускаться руками:
#
#   bash scripts/monitoring/robbox_hailo_driver_check.sh               # normal
#   bash scripts/monitoring/robbox_hailo_driver_check.sh --dry-run     # no side effects
#   bash scripts/monitoring/robbox_hailo_driver_check.sh --json        # machine-readable
#   bash scripts/monitoring/robbox_hailo_driver_check.sh --warn-only   # exit 0 always
#
# Acceptance criteria (родительская карточка t_3a5ddb1c, issue #3090):
#   - Алерт если /dev/hailo0 отсутствует (HailoRT PCIe driver не загружен
#     для активного kernel).
#   - Метрика `robbox_hailo_driver_ok` пишется в textfile-коллектор.
#   - Alert-event в ALERT_LOG.
#
# Env (опционально, см. defaults):
#   ROBBOX_HAILO_METRICS_FILE  — Prometheus textfile
#                                 (default $HOME/.local/state/robbox_hailo_driver.prom)
#   ROBBOX_HAILO_ALERT_LOG     — путь к alert-log
#                                 (default $HOME/.local/state/robbox_hailo_alerts.log)
#   ROBBOX_HAILO_GRACE_SECS    — «не алертить первые N секунд после boot»
#                                 default 120 (kernel init + DKMS build)
#   ROBBOX_HAILO_DEVICE_PATH   — путь к PCIe device node
#                                 (default /dev/hailo0)
#
# Exit codes:
#   0 — OK (Hailo device present, DKMS kernel→.ko match, hailortcli видит чип)
#   1 — ALERT (одно из условий нарушено вне grace-периода)
#   2 — usage / config error
# ============================================================================

set -euo pipefail

# --------------------------------------------------------------------------- #
# Defaults & flags
# --------------------------------------------------------------------------- #
GRACE_SECS="${ROBBOX_HAILO_GRACE_SECS:-120}"
DEVICE_PATH="${ROBBOX_HAILO_DEVICE_PATH:-/dev/hailo0}"
BOOT_LOG="${ROBBOX_HAILO_BOOT_LOG:-/var/log/robbox-hailo-driver-boot.log}"
METRICS_FILE="${ROBBOX_HAILO_METRICS_FILE:-$HOME/.local/state/robbox_hailo_driver.prom}"
ALERT_LOG="${ROBBOX_HAILO_ALERT_LOG:-$HOME/.local/state/robbox_hailo_alerts.log}"

DRY_RUN=0
JSON_OUT=0
WARN_ONLY=0

for arg in "$@"; do
  case "$arg" in
    --dry-run)   DRY_RUN=1 ;;
    --json)      JSON_OUT=1 ;;
    --warn-only) WARN_ONLY=1 ;;
    -h|--help)
      sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "Unknown arg: $arg" >&2
      exit 2
      ;;
  esac
done

# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
log_info()  { printf '[%s] INFO  %s\n'  "$(date -u +%FT%TZ)" "$*" >&2; }
log_warn()  { printf '[%s] WARN  %s\n'  "$(date -u +%FT%TZ)" "$*" >&2; }
log_error() { printf '[%s] ERROR %s\n' "$(date -u +%FT%TZ)" "$*" >&2; }

# elapsed_seconds_since_boot — секунды с момента boot (issue #3090 root
# cause завязан на kernel upgrade → fresh boot, поэтому grace-period
# считается от boot, а не от start unit'а как у vision_health_check).
# Без systemd/bootcmd → 0 (сохраняем совместимость с CI).
elapsed_seconds_since_boot() {
  if [ -r /proc/uptime ]; then
    awk '{printf "%d", $1}' /proc/uptime
  else
    echo "0"
  fi
}

# check_device_present — есть ли device node.
check_device_present() {
  if [ -e "$DEVICE_PATH" ]; then
    echo "1"
  else
    echo "0"
  fi
}

# check_kernel_module_match — собрана ли .ko для активного kernel?
# Возвращает "1" если /lib/modules/$(uname -r)/{extra,updates}/hailo_pci.ko
# существует. Это root-cause-check из issue #3090.
#
# Env override (для unit-тестов): ROBBOX_HAILO_LIB_MODULES_FAKEROOT —
# если задан, проверяем путь относительно него вместо реального /
# (например ROBBOX_HAILO_LIB_MODULES_FAKEROOT=/tmp/sandbox → смотрим
# /tmp/sandbox/lib/modules/<kernel>/).
check_kernel_module_match() {
  local kern root
  kern="$(uname -r)"
  root="${ROBBOX_HAILO_LIB_MODULES_FAKEROOT:-}"
  if [ -n "$root" ]; then
    if [ -f "${root}/lib/modules/${kern}/extra/hailo_pci.ko" ] || \
       [ -f "${root}/lib/modules/${kern}/updates/hailo_pci.ko" ]; then
      echo "1"
    else
      echo "0"
    fi
  else
    if [ -f "/lib/modules/${kern}/extra/hailo_pci.ko" ] || \
       [ -f "/lib/modules/${kern}/updates/hailo_pci.ko" ]; then
      echo "1"
    else
      echo "0"
    fi
  fi
}

# check_dkms_status — есть ли hailort-pcie-driver в `dkms status`.
# Возвращает:
#   -1  если dkms command missing/unusable (infrastructure problem)
#    1  если hailort-pcie-driver зарегистрирован
#    0  если dkms доступен, но hailort-pcie-driver НЕ зарегистрирован
check_dkms_status() {
  if ! command -v dkms >/dev/null 2>&1; then
    echo "-1"
    return 0
  fi
  local out
  out="$(dkms status 2>/dev/null || true)"
  if [ -z "$out" ] || echo "$out" | grep -qi "command not found"; then
    # dkms "есть" в PATH, но не запускается (broken install, mock absent_tool)
    echo "-1"
    return 0
  fi
  if echo "$out" | grep -q "hailort-pcie-driver"; then
    echo "1"
  else
    echo "0"
  fi
}

# check_hailortcli_scan — финальная проверка: hailortcli видит устройство.
# Возвращает "1" если `hailortcli scan` нашёл устройство. Используется как
# positive end-to-end check, отдельный от device node (т.к. /dev/hailo0
# может быть, но с firmware-проблемой).
check_hailortcli_scan() {
  if ! command -v hailortcli >/dev/null 2>&1; then
    echo "-1"
    return 0
  fi
  # `hailortcli scan` обычно возвращает 0 при успехе; в текстовом выводе
  # видим строку с device description. Timeout 10 сек — hailortcli может
  # зависнуть если firmware не отвечает.
  if timeout 10 hailortcli scan 2>/dev/null | grep -qi "hailo"; then
    echo "1"
  else
    echo "0"
  fi
}

# write_metrics — Prometheus textfile (SSoT-pattern, как
# robbox_vision_health_check.sh).
write_metrics() {
  local dev_present="$1" kern_match="$2" dkms_ok="$3" scan_ok="$4" \
        grace_elapsed="$5" verdict="$6" alert="$7"
  if [ "$DRY_RUN" = "1" ]; then
    log_info "DRY-RUN: would write metrics to $METRICS_FILE"
    return 0
  fi
  mkdir -p "$(dirname "$METRICS_FILE")" 2>/dev/null || true
  cat > "$METRICS_FILE" <<EOF
# HELP robbox_hailo_driver_device_present 1 if /dev/hailo0 exists on the host, 0 otherwise. The most common failure mode after a kernel upgrade (issue #3090).
# TYPE robbox_hailo_driver_device_present gauge
robbox_hailo_driver_device_present ${dev_present}
# HELP robbox_hailo_driver_kernel_module_match 1 if hailo_pci.ko is built for the active kernel, 0 otherwise. Core root-cause check for issue #3090.
# TYPE robbox_hailo_driver_kernel_module_match gauge
robbox_hailo_driver_kernel_module_match ${kern_match}
# HELP robbox_hailo_driver_dkms_registered 1 if hailort-pcie-driver is registered in dkms status, -1 if dkms not installed, 0 otherwise.
# TYPE robbox_hailo_driver_dkms_registered gauge
robbox_hailo_driver_dkms_registered ${dkms_ok}
# HELP robbox_hailo_driver_hailortcli_scan 1 if hailortcli scan reports a Hailo device, -1 if hailortcli missing, 0 otherwise. End-to-end smoke check.
# TYPE robbox_hailo_driver_hailortcli_scan gauge
robbox_hailo_driver_hailortcli_scan ${scan_ok}
# HELP robbox_hailo_driver_grace_elapsed_seconds Seconds elapsed since the last boot. Negative if /proc/uptime unavailable.
# TYPE robbox_hailo_driver_grace_elapsed_seconds gauge
robbox_hailo_driver_grace_elapsed_seconds ${grace_elapsed}
# HELP robbox_hailo_driver_alert 1 if the watchdog believes the driver is broken past grace, 0 otherwise.
# TYPE robbox_hailo_driver_alert gauge
robbox_hailo_driver_alert ${alert}
# HELP robbox_hailo_driver_check_timestamp_seconds Unix timestamp of the last watchdog run.
# TYPE robbox_hailo_driver_check_timestamp_seconds gauge
robbox_hailo_driver_check_timestamp_seconds $(date +%s)
EOF
}

# record_alert — append alert-event в ALERT_LOG (идемпотентно: каждое
# событие имеет timestamp; дедупликация — на промежутке).
record_alert() {
  local dev_present="$1" kern_match="$2" dkms_ok="$3" scan_ok="$4" \
        grace_elapsed="$5" reason="$6"
  if [ "$DRY_RUN" = "1" ]; then
    log_info "DRY-RUN: would alert (dev=${dev_present}, kern_match=${kern_match}, dkms=${dkms_ok}, scan=${scan_ok}, grace=${grace_elapsed}s, reason='$reason')"
    return 0
  fi
  mkdir -p "$(dirname "$ALERT_LOG")" 2>/dev/null || true
  {
    printf '[%s] ALERT robbox-hailo-driver: dev=%s kern_match=%s dkms=%s scan=%s grace=%ss reason=%s\n' \
      "$(date -u +%FT%TZ)" "$dev_present" "$kern_match" "$dkms_ok" "$scan_ok" "$grace_elapsed" "$reason"
  } >> "$ALERT_LOG" 2>/dev/null || log_warn "запись в $ALERT_LOG не удалась"
}

# append_boot_summary — короткий слепок для boot-log.
append_boot_summary() {
  local dev_present="$1" kern_match="$2" dkms_ok="$3" scan_ok="$4" \
        grace_elapsed="$5" verdict="$6"
  if [ "$DRY_RUN" = "1" ]; then
    log_info "DRY-RUN: would append to $BOOT_LOG"
    return 0
  fi
  if ! [ -w "$(dirname "$BOOT_LOG")" ] && ! [ -w "$BOOT_LOG" ]; then
    log_warn "BOOT_LOG ($BOOT_LOG) недоступен для записи — пропускаем"
    return 0
  fi
  {
    printf '==== robbox-hailo-driver boot summary @ %s ====\n' "$(date -u +%FT%TZ)"
    printf 'active_kernel=%s\n' "$(uname -r)"
    printf 'device_present=%s kernel_module_match=%s dkms_ok=%s hailortcli_scan=%s\n' \
      "$dev_present" "$kern_match" "$dkms_ok" "$scan_ok"
    printf 'grace_elapsed_secs=%s grace_threshold=%s verdict=%s\n' \
      "$grace_elapsed" "$GRACE_SECS" "$verdict"
    printf '\n'
  } >> "$BOOT_LOG" 2>/dev/null || log_warn "запись в $BOOT_LOG не удалась"
}

# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

# 1. Сбор всех сигналов
dev_present="$(check_device_present)"
kern_match="$(check_kernel_module_match)"
dkms_ok="$(check_dkms_status)"
scan_ok="$(check_hailortcli_scan)"
grace_elapsed="$(elapsed_seconds_since_boot)"

# 2. Verdict
verdict="ok"
reason=""

# Иерархия нарушений (от важного к минорному):
# - dkms_ok = -1 (dkms не установлен) → infra_error, не alert
# - kern_match = 0 → root-cause failure (issue #3090)
# - dev_present = 0 → /dev/hailo0 отсутствует (effect root-cause)
# - scan_ok = 0 → HailoRT не видит чип (firmware/runtime проблема)
if [ "$dkms_ok" = "-1" ]; then
  verdict="infra_error"
  reason="dkms не установлен"
elif [ "$kern_match" = "0" ] || [ "$dev_present" = "0" ]; then
  if [ "$grace_elapsed" -lt "$GRACE_SECS" ]; then
    verdict="grace"
    reason="в grace-периоде (${grace_elapsed}s < ${GRACE_SECS}s): kern_match=${kern_match}, dev_present=${dev_present}"
  else
    verdict="alert"
    if [ "$kern_match" = "0" ]; then
      reason="hailo_pci.ko отсутствует для активного kernel $(uname -r) (issue #3090 root cause)"
    else
      reason="/dev/hailo0 отсутствует (driver не загружен)"
    fi
  fi
elif [ "$scan_ok" = "0" ]; then
  if [ "$grace_elapsed" -lt "$GRACE_SECS" ]; then
    verdict="grace"
    reason="в grace-периоде: hailortcli scan не видит чип (firmware может ещё подгружаться)"
  else
    verdict="alert"
    reason="hailortcli scan не видит чип (dev_present=1, kern_match=1, но runtime проблема)"
  fi
fi

# 3. Side effects
alert_metric=0
case "$verdict" in
  alert)
    log_error "$reason"
    record_alert "$dev_present" "$kern_match" "$dkms_ok" "$scan_ok" "$grace_elapsed" "$reason"
    alert_metric=1
    ;;
  infra_error)
    log_warn "$reason"
    ;;
  grace)
    log_info "$reason"
    ;;
  ok)
    log_info "Hailo driver ok (dev=$dev_present, kern_match=$kern_match, dkms=$dkms_ok, scan=$scan_ok, grace=${grace_elapsed}s)"
    ;;
esac

append_boot_summary "$dev_present" "$kern_match" "$dkms_ok" "$scan_ok" "$grace_elapsed" "$verdict"
write_metrics "$dev_present" "$kern_match" "$dkms_ok" "$scan_ok" "$grace_elapsed" "$verdict" "$alert_metric"

# 4. JSON output
if [ "$JSON_OUT" = "1" ]; then
  printf '{"timestamp":"%s","active_kernel":"%s","device_present":%s,"kernel_module_match":%s,"dkms_registered":%s,"hailortcli_scan":%s,"grace_elapsed_seconds":%s,"grace_threshold_seconds":%s,"verdict":"%s","alert":%s,"reason":"%s"}\n' \
    "$(date -u +%FT%TZ)" "$(uname -r)" "$dev_present" "$kern_match" "$dkms_ok" "$scan_ok" \
    "$grace_elapsed" "$GRACE_SECS" "$verdict" "$alert_metric" "$reason"
fi

# 5. Exit code
if [ "$verdict" = "alert" ] && [ "$WARN_ONLY" = "0" ]; then
  exit 1
fi
exit 0