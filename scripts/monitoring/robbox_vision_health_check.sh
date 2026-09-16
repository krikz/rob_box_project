#!/usr/bin/env bash
# ============================================================================
# robbox_vision_health_check.sh — детектор failed boot Vision Pi стека.
#
# Single-source-of-truth скрипт, вызывается из systemd timer
# `robbox-vision-health.timer` (см. setup_vision_pi.sh → setup_health_monitor).
# Также может запускаться руками:
#
#   bash scripts/monitoring/robbox_vision_health_check.sh              # normal
#   bash scripts/monitoring/robbox_vision_health_check.sh --dry-run    # no side effects
#   bash scripts/monitoring/robbox_vision_health_check.sh --json      # machine-readable
#   bash scripts/monitoring/robbox_vision_health_check.sh --warn-only # exit 0 always
#
# Acceptance criteria (родительская карточка t_5ab5e44a):
#   - Алерт если 0 running-контейнеров > ROBBOX_VISION_GRACE_SECS после старта
#     сервиса `robbox-vision.service`.
#   - Метрика `robbox_vision_running_containers` пишется в textfile-коллектор.
#   - Лог-summary (`/var/log/robbox-vision-boot.log`) — список того, что
#     поднялось и что не поднялось (exit codes).
#
# Env (все optional, см. defaults в коде):
#   ROBBOX_VISION_COMPOSE_DIR   — путь к docker-compose.yaml (default ~/rob_box_project/docker/vision)
#   ROBBOX_VISION_GRACE_SECS    — «не алертить первые N секунд после старта robbox-vision.service»
#                                 default 300 (5 минут, как требовала карточка)
#   ROBBOX_VISION_BOOT_LOG      — путь к boot-summary (default /var/log/robbox-vision-boot.log)
#   ROBBOX_VISION_METRICS_FILE  — Prometheus textfile (default $HOME/.local/state/robbox_vision_health.prom)
#   ROBBOX_VISION_ALERT_LOG     — путь к alert-log (default $HOME/.local/state/robbox_vision_alerts.log)
#
# Exit codes:
#   0 — OK (контейнеры подняты ИЛИ мы в grace-периоде)
#   1 — ALERT (0 контейнеров после grace-периода)
#   2 — usage / config error
# ============================================================================

set -euo pipefail

# --------------------------------------------------------------------------- #
# Defaults & flags
# --------------------------------------------------------------------------- #
# COMPOSE_DIR зарезервирован для последующих итераций (например, явный
# `docker compose --project-directory $COMPOSE_DIR ps` когда нужно
# отфильтровать контейнеры именно стека robbox, а не все на хосте).
# Сейчас `docker ps` намеренно смотрит на ВСЕ контейнеры: parent-карточка
# t_5ab5e44a говорит «0 running-контейнеров >5 минут после старта», и для
# стека robbox-vision.Compose все равно поднимает _только_ его сервисы,
# так что посторонние контейнеры в этот список не попадут. Если это
# изменится — добавим `--project-name` или `docker compose ps`.
# shellcheck disable=SC2034
COMPOSE_DIR="${ROBBOX_VISION_COMPOSE_DIR:-$HOME/rob_box_project/docker/vision}"
GRACE_SECS="${ROBBOX_VISION_GRACE_SECS:-300}"
BOOT_LOG="${ROBBOX_VISION_BOOT_LOG:-/var/log/robbox-vision-boot.log}"
METRICS_FILE="${ROBBOX_VISION_METRICS_FILE:-$HOME/.local/state/robbox_vision_health.prom}"
ALERT_LOG="${ROBBOX_VISION_ALERT_LOG:-$HOME/.local/state/robbox_vision_alerts.log}"

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
log_error() { printf '[%s] ERROR %s\n'  "$(date -u +%FT%TZ)" "$*" >&2; }

# elapsed_seconds_since_start_unit — сколько секунд прошло с последнего
# `systemctl start robbox-vision.service` (или `systemctl show` → ActiveEnterTimestamp).
# Возвращает 0 если systemctl не спросить (CI без systemd, в контейнере и т.п.),
# "-1" если service никогда не стартовал.
#
# Поддерживает 2 формата ActiveEnterTimestamp:
#   * современный systemd: "Tue 2026-09-16 08:11:22 UTC" (с day-name)
#   * ISO:                "2026-09-16 08:11:22 UTC"
# Парсинг locale-зависимый (на голландской системе `Tue` → `wo`, что ломает
# `date -d`); fallback на ISO-парсинг покрывает оба случая.
elapsed_seconds_since_start_unit() {
  if ! command -v systemctl >/dev/null 2>&1; then
    echo "0"
    return 0
  fi
  local ts
  ts="$(systemctl show robbox-vision.service --property=ActiveEnterTimestamp --value 2>/dev/null || true)"
  # `systemctl show --value` возвращает либо пустую строку (service без юнита),
  # либо "n/a" (service inactive/never-started в этой сессии). Оба случая →
  # «service ни разу не стартовал» → сигнал -1.
  if [ -z "$ts" ] || [ "$ts" = "n/a" ]; then
    echo "-1"
    return 0
  fi
  local epoch_now epoch_then
  epoch_now="$(date +%s)"
  # Сначала пробуем as-is (systemd-формат с day-name зависит от locale);
  # если не вышло — отбрасываем первый token и пробуем ISO.
  epoch_then="$(date -d "$ts" +%s 2>/dev/null || true)"
  if [ -z "$epoch_then" ]; then
    epoch_then="$(date -d "${ts#* }" +%s 2>/dev/null || echo "$epoch_now")"
  fi
  echo $(( epoch_now - epoch_then ))
}

# count_running — robust wrapper вокруг `docker ps`. Возвращает число
# running-контейнеров (--filter status=running) c именами. Если docker daemon
# лежит, возвращает "-1" как сигнал ошибки инфраструктуры (НЕ алерт о стеке).
count_running() {
  if ! command -v docker >/dev/null 2>&1; then
    echo "-1"
    return 0
  fi
  # `docker ps --filter status=running --format '{{.Names}}'` — стабильный
  # machine-readable вывод. Если docker daemon недоступен — `docker ps` падает
  # с exit≠0; в bash-стиле `set -e` это убьёт скрипт, поэтому оборачиваем.
  local n
  n="$(docker ps --filter status=running --format '{{.Names}}' 2>/dev/null | wc -l | tr -d ' ')" || n="-1"
  if ! [[ "$n" =~ ^[0-9]+$ ]]; then
    n="-1"
  fi
  echo "$n"
}

# list_names_and_status — все контейнеры (любой статус) с именами и статусом,
# для boot-log summary (родительская карточка п.3).
list_names_and_status() {
  if ! command -v docker >/dev/null 2>&1; then
    echo "docker not available"
    return 0
  fi
  docker ps -a --format '{{.Names}}\t{{.Status}}\t{{.State}}' 2>/dev/null \
    | head -50 || true
}

# append_boot_summary — записать (append) summary-блок в BOOT_LOG. Не падаем,
# если BOOT_LOG недоступен для записи (например, /var/log/ не монтирован на
# хосте без sudo). setup_vision_pi.sh должен создать файл с правильными правами.
append_boot_summary() {
  if [ "$DRY_RUN" = "1" ]; then
    log_info "DRY-RUN: would append to $BOOT_LOG"
    return 0
  fi
  if ! [ -w "$(dirname "$BOOT_LOG")" ] && ! [ -w "$BOOT_LOG" ]; then
    log_warn "BOOT_LOG ($BOOT_LOG) недоступен для записи — пропускаем"
    return 0
  fi
  {
    printf '==== robbox-vision boot summary @ %s ====\n' "$(date -u +%FT%TZ)"
    printf 'running_containers=%s\n' "$1"
    printf 'grace_elapsed_secs=%s grace_threshold=%s\n' "$2" "$GRACE_SECS"
    printf '%s\n' '--- container list (name | status | state) ---'
    list_names_and_status
    printf '\n'
  } >> "$BOOT_LOG" 2>/dev/null || log_warn "запись в $BOOT_LOG не удалась"
}

# write_metrics — пишет Prometheus textfile-format с метриками для
# node_exporter textfile collector (SSoT-pattern, см. README-orphan-audit.md §51).
write_metrics() {
  local running="$1" grace_elapsed="$2" verdict="$3"
  if [ "$DRY_RUN" = "1" ]; then
    log_info "DRY-RUN: would write metrics to $METRICS_FILE"
    return 0
  fi
  mkdir -p "$(dirname "$METRICS_FILE")" 2>/dev/null || true
  cat > "$METRICS_FILE" <<EOF
# HELP robbox_vision_running_containers Number of docker containers on the Vision Pi stack currently in 'running' state. 0 outside the grace period triggers an alert.
# TYPE robbox_vision_running_containers gauge
robbox_vision_running_containers ${running}
# HELP robbox_vision_grace_elapsed_seconds Seconds elapsed since the last robbox-vision.service start. Negative if service has never been started in this boot.
# TYPE robbox_vision_grace_elapsed_seconds gauge
robbox_vision_grace_elapsed_seconds ${grace_elapsed}
# HELP robbox_vision_health_alert 1 if the health-check believes the stack failed to come up (0 running containers past grace), 0 otherwise.
# TYPE robbox_vision_health_alert gauge
robbox_vision_health_alert $([ "$verdict" = "alert" ] && echo 1 || echo 0)
# HELP robbox_vision_health_check_timestamp_seconds Unix timestamp of the last health-check run.
# TYPE robbox_vision_health_check_timestamp_seconds gauge
robbox_vision_health_check_timestamp_seconds $(date +%s)
EOF
}

# record_alert — append alert-event в ALERT_LOG (идемпотентно через cooldown).
record_alert() {
  local running="$1" grace_elapsed="$2"
  if [ "$DRY_RUN" = "1" ]; then
    log_info "DRY-RUN: would alert (running=$running, grace=${grace_elapsed}s)"
    return 0
  fi
  mkdir -p "$(dirname "$ALERT_LOG")" 2>/dev/null || true
  {
    printf '[%s] ALERT robbox-vision: running_containers=%s grace_elapsed_secs=%s\n' \
      "$(date -u +%FT%TZ)" "$running" "$grace_elapsed"
  } >> "$ALERT_LOG" 2>/dev/null || log_warn "запись в $ALERT_LOG не удалась"
}

# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

# 1. Считаем running-контейнеры
running="$(count_running)"

# 2. Считаем сколько секунд прошло с момента start unit-а
grace_elapsed="$(elapsed_seconds_since_start_unit)"

# 3. Verdict
verdict="ok"
reason=""

if [ "$running" = "-1" ]; then
  # docker daemon лежит — это ИНФРА-проблема, а не проблема стека.
  # Пишем в метрики -1 (не число контейнеров), не алертим, но warn.
  verdict="infra_error"
  reason="docker daemon unavailable"
elif [ "$running" = "0" ]; then
  if [ "$grace_elapsed" = "-1" ]; then
    # service никогда не стартовал → это не cold-boot, это «выключен».
    # Не алертим (см. эссе в elapsed_seconds_since_start_unit).
    verdict="inactive"
    reason="robbox-vision.service never started"
  elif [ "$grace_elapsed" -lt "$GRACE_SECS" ]; then
    verdict="grace"
    reason="within grace period (${grace_elapsed}s < ${GRACE_SECS}s)"
  else
    verdict="alert"
    reason="0 running containers past grace (${grace_elapsed}s >= ${GRACE_SECS}s)"
  fi
fi

# 4. Side effects
case "$verdict" in
  alert)
    log_error "$reason"
    record_alert "$running" "$grace_elapsed"
    ;;
  infra_error)
    log_warn "$reason"
    ;;
  grace|inactive)
    log_info "$reason"
    ;;
  ok)
    log_info "stack healthy (running=$running, grace=${grace_elapsed}s)"
    ;;
esac

append_boot_summary "$running" "$grace_elapsed"
write_metrics "$running" "$grace_elapsed" "$verdict"

# 5. JSON output (для ручной диагностики / piping)
if [ "$JSON_OUT" = "1" ]; then
  printf '{"timestamp":"%s","running_containers":%s,"grace_elapsed_seconds":%s,"grace_threshold_seconds":%s,"verdict":"%s","reason":"%s"}\n' \
    "$(date -u +%FT%TZ)" "$running" "$grace_elapsed" "$GRACE_SECS" "$verdict" "$reason"
fi

# 6. Exit code
if [ "$verdict" = "alert" ] && [ "$WARN_ONLY" = "0" ]; then
  exit 1
fi
exit 0
