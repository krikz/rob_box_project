#!/usr/bin/env bash
# ============================================================================
# check_hailo_driver.sh — pre-deploy fail-fast для Hailo PCIe driver
# (issue #3090).
#
# Источник истины: <repo>/.github/scripts/check_hailo_driver.sh
# Копия на runner (self-hosted) обновляется через checkout — НЕ через
# hardlink/symlink. Шаблон дистрибуции совпадает с check_container_status.sh
# (вызывается из .github/workflows/L-Deploy and Verify.yml).
#
# Контракт:
#   - Проверяет на Vision Pi (10.1.1.11):
#       * /dev/hailo0 (через ssh + ls)
#       * hailo_pci.ko для активного kernel (через ssh + ls
#         /lib/modules/$(uname -r)/{extra,updates}/)
#       * dkms status hailort-pcie-driver
#       * hailortcli scan (end-to-end runtime)
#   - Если что-то не так вне grace → возвращает {healthy: false, reason: ...}
#     в JSON, workflow fail-fast и НЕ стартует docker compose up.
#   - Если hailortcli scan или .ko отсутствует — НЕ запускать контейнеры
#     vision-hailo (они будут fail-fast внутри, но это тратит время на
#     boot+image-pull+init+restart-loop).
#
# Env (required):
#   PI_HOST            Vision Pi IP (e.g. 10.1.1.11)
#   PI_USER            ssh user (e.g. ros2)
#   PI_PASSWORD        sshpass -p password
#   PI_SSH_OPTS        ssh options (set up by workflow)
#
# Env (optional):
#   CONFIRM_INTERVAL   delay between two checks (default 5 — без подтверждения)
#   FINDINGS_FILE      JSONL file для совместимости с deploy-verify pipeline
#                      (если не задан, находки НЕ пишутся)
#   ENVIRONMENT        production|staging|test — попадает в findings
#   SCOPE              vision|main — попадает в findings
#   GRACE_SECS         grace-период после boot (default 0 — на runner мы не знаем
#                      uptime Vision Pi, fail-fast без grace — но это conservative
#                      и ловит только hard red. Для soft checks — fail-fast всегда)
#
# Output (stdout, парсится workflow через jq):
#   {"healthy": true|false, "failed_count": N, "findings_count": N}
#
# Exit codes:
#   0 — script finished (healthy/false decision в JSON)
#   1 — ssh failed (real connectivity problem, не fail-fast на driver)
# ============================================================================
set -euo pipefail

CONFIRM_INTERVAL="${CONFIRM_INTERVAL:-5}"

# Required env
: "${PI_HOST:?PI_HOST must be set (e.g. 10.1.1.11)}"
: "${PI_USER:?PI_USER must be set (e.g. ros2)}"
: "${PI_PASSWORD:?PI_PASSWORD must be set (sshpass -p)}"
: "${PI_SSH_OPTS:?PI_SSH_OPTS must be set (ssh options string)}"

# Optional env
GRACE_SECS="${GRACE_SECS:-0}"
FINDINGS_FILE="${FINDINGS_FILE:-/dev/null}"
ENVIRONMENT="${ENVIRONMENT:-unknown}"
SCOPE="${SCOPE:-vision}"

log() { printf '[check_hailo_driver] %s %s\n' "$(date -Iseconds)" "$*" >&2; }

# --------------------------------------------------------------------------- #
# Step 1: gather remote state via SSH
# --------------------------------------------------------------------------- #
# Один SSH-вызов — собрать всё состояние Pi для атомарности (нет гонки
# между sample1/sample2 как в check_container_status.sh). Для pre-deploy
# достаточно single-snapshot.
remote_state_marker="" # unused placeholder reserved for future hooks
sentinel_magic="magic_sentinel_no_hailo_lspci" # unused placeholder for diagnostic
# Запускаем sshpass в под-оболочке чтобы перехватить exit code; внутри
# $() set -e/-o pipefail НЕ действуют, поэтому сохраняем код явно.
_ssh_rc=0
remote_state="$(sshpass -p "$PI_PASSWORD" ssh $PI_SSH_OPTS "${PI_USER}@${PI_HOST}" <<'REMOTE_SCRIPT'
echo "=== uname ==="
uname -r
echo "=== lspci ==="
if lspci 2>/dev/null | grep -i Hailo > /dev/null; then
  echo PRESENT
else
  echo MAGIC_NO_HAILO
fi
echo "=== module_extra ==="
if ls "/lib/modules/$(uname -r)/extra/hailo_pci.ko" > /dev/null 2>&1; then
  echo PRESENT
else
  echo MAGIC_MISSING
fi
echo "=== module_updates ==="
if ls "/lib/modules/$(uname -r)/updates/hailo_pci.ko" > /dev/null 2>&1; then
  echo PRESENT
else
  echo MAGIC_MISSING
fi
echo "=== device_node ==="
if ls -la /dev/hailo0 > /dev/null 2>&1; then
  ls -la /dev/hailo0 | awk '{for(i=6;i<=NF;i++)printf "%s ",$i;print ""}'
else
  echo MAGIC_MISSING
fi
echo "=== dkms_status ==="
if command -v dkms > /dev/null 2>&1; then
  dkms status 2>&1 | grep -i hailort || echo MAGIC_NO_HAILORT_DKMS
else
  echo MAGIC_NO_DKMS_TOOL
fi
echo "=== hailortcli_scan ==="
if command -v hailortcli > /dev/null 2>&1; then
  timeout 8 hailortcli scan 2>&1 || echo MAGIC_HAILORTCLI_FAILED
else
  echo MAGIC_NO_HAILORTCLI
fi
echo "=== uptime ==="
awk '{print int($1)}' /proc/uptime
REMOTE_SCRIPT
)"; _ssh_rc=$?

if [ "$_ssh_rc" -ne 0 ] || [ -z "$remote_state" ]; then
  log "ERROR: SSH to ${PI_HOST} failed rc=${_ssh_rc}, no state gathered"
  jq -nc '{healthy: false, failed_count: 1, findings_count: 0, reason: "ssh_failed"}'
  exit 1
fi

# --------------------------------------------------------------------------- #
# Step 2: parse remote state
# --------------------------------------------------------------------------- #
active_kernel="$(echo "$remote_state" | awk '/^=== uname ===/ {getline; print; exit}')"
lspci_line="$(echo "$remote_state" | awk '/^=== lspci ===/ {getline; print; exit}')"
module_extra="$(echo "$remote_state" | awk '/^=== module_extra ===/ {getline; print; exit}')"
module_updates="$(echo "$remote_state" | awk '/^=== module_updates ===/ {getline; print; exit}')"
device_node="$(echo "$remote_state" | awk '/^=== device_node ===/ {getline; print; exit}')"
dkms_status="$(echo "$remote_state" | awk '/^=== dkms_status ===/{f=1;next} /^=== /{f=0} f')"
hailortcli_scan="$(echo "$remote_state" | awk '/^=== hailortcli_scan ===/{f=1;next} /^=== /{f=0} f')"
uptime_secs="$(echo "$remote_state" | awk '/^=== uptime ===/ {getline; print; exit}')"

# --------------------------------------------------------------------------- #
# Step 3: verdict — 4 checks
# --------------------------------------------------------------------------- #
findings_jsonl=""
FAILED_COUNT=0
REASONS=()

# Check 1: lspci shows Hailo — если MAGIC_NO_HAILO, железки нет,
# pre-deploy check не наша забота.
if [ "${lspci_line}" = "MAGIC_NO_HAILO" ]; then
  log "WARN: Hailo AI HAT not detected via lspci, skipping pre-deploy check"
  jq -nc '{healthy: true, failed_count: 0, findings_count: 0, reason: "no_hailo_hw"}'
  exit 0
fi

# Check 2: .ko для активного kernel
if [ "$module_extra" = "MAGIC_MISSING" ] && [ "$module_updates" = "MAGIC_MISSING" ]; then
  REASONS+=("hailo_pci.ko missing for active kernel ${active_kernel}, issue #3090 root cause")
fi

# Check 3: /dev/hailo0
if [ "$device_node" = "MAGIC_MISSING" ]; then
  REASONS+=("/dev/hailo0 missing, driver not loaded for ${active_kernel}")
fi

# Check 4: hailortcli scan (end-to-end)
if echo "$hailortcli_scan" | grep -qi "hailo devices not found\|no devices\|magic_hailortcli_failed\|magic_no_hailortcli"; then
  REASONS+=("hailortcli scan: ${hailortcli_scan}")
fi

# Grace-period: если uptime < GRACE_SECS, не алертим (kernel init / DKMS build).
# На pre-deploy этат нормально fail-fast без grace — Шифу хочет видеть red.
if [ "${GRACE_SECS}" -gt 0 ] && [ -n "$uptime_secs" ] && [ "$uptime_secs" -lt "$GRACE_SECS" ]; then
  log "Vision Pi uptime ${uptime_secs}s < grace ${GRACE_SECS}s — issues будут silent"
  jq -nc --argjson up "$uptime_secs" --argjson grace "$GRACE_SECS" \
    '{healthy: true, failed_count: 0, findings_count: 0, reason: ("grace_period: " + (up|tostring) + "s < " + (grace|tostring) + "s")}'
  exit 0
fi

# --------------------------------------------------------------------------- #
# Step 4: emit findings + JSON output
# --------------------------------------------------------------------------- #
if [ "${#REASONS[@]}" -gt 0 ]; then
  FAILED_COUNT="${#REASONS[@]}"
  log "❌ Hailo driver check failed: ${FAILED_COUNT} issue(s)"
  for r in "${REASONS[@]}"; do
    log "  - $r"
  done

  # Findings → JSONL (если задан FINDINGS_FILE)
  if [ "$FINDINGS_FILE" != "/dev/null" ]; then
    {
      for r in "${REASONS[@]}"; do
        jq -nc \
          --arg environment "$ENVIRONMENT" \
          --arg scope "$SCOPE" \
          --arg kind "hailo_driver" \
          --arg severity "critical" \
          --arg summary "Hailo PCIe driver issue (issue #3090)" \
          --arg raw_text "$r" \
          '{environment:$environment, scope:$scope, kind:$kind, severity:$severity, summary:$summary, raw_text:$raw_text}'
      done
    } >> "$FINDINGS_FILE" 2>/dev/null || true
  fi
fi

# Final JSON (парсится workflow через jq)
HEALTHY_BOOL="true"
[ "$FAILED_COUNT" -gt 0 ] && HEALTHY_BOOL="false"

REASON_JSON=""
if [ "${#REASONS[@]}" -gt 0 ]; then
  REASON_JSON="$(printf '%s;' "${REASONS[@]}")"
fi

jq -nc \
  --argjson healthy "$( [ "$HEALTHY_BOOL" = "true" ] && echo true || echo false )" \
  --argjson failed_count "$FAILED_COUNT" \
  --arg active_kernel "$active_kernel" \
  --arg lspci_line "$lspci_line" \
  --arg module_extra "$module_extra" \
  --arg module_updates "$module_updates" \
  --arg device_node "${device_node:-(missing)}" \
  --arg dkms_status "$(echo "$dkms_status" | head -1)" \
  --arg hailortcli "$(echo "$hailortcli_scan" | head -1)" \
  --arg reason "$REASON_JSON" \
  '{healthy: $healthy, failed_count: $failed_count, active_kernel: $active_kernel, lspci: $lspci_line, module_extra: $module_extra, module_updates: $module_updates, device_node: $device_node, dkms_status: $dkms_status, hailortcli: $hailortcli, reason: $reason}'

exit 0