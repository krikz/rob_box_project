#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# 🤖 РОББОКС Vision Pi — настройка zram-swap (issue #2621, ADR-0111)
# ═══════════════════════════════════════════════════════════════════════════
#
# Назначение:
#   Настроить zram-swap на Vision Pi, чтобы у ядра Linux была возможность
#   вытеснять анонимные страницы в сжатую область RAM. Без этого при
#   заполнении MemAvailable sshd и любая прикладная нагрузка становятся
#   неотзывчивыми, а оператор теряет доступ до жёсткой перезагрузки.
#
# Идемпотентность:
#   Если /dev/zram0 уже активен и параметры совпадают — пропускает.
#   Если параметры отличаются — пересоздаёт zram (swapoff → reset → mkswap → swapon).
#
# Использование:
#   bash scripts/setup/setup_vision_pi_swap.sh --auto          # default 4 GB zstd
#   bash scripts/setup/setup_vision_pi_swap.sh --dry-run        # показать план, ничего не менять
#   bash scripts/setup/setup_vision_pi_swap.sh --status          # текущее состояние
#   bash scripts/setup/setup_vision_pi_swap.sh --remove          # выключить и удалить
#   ROBBOX_ZRAM_SIZE_MB=2048 bash setup_vision_pi_swap.sh --auto # override размера
#
# Переменные окружения (override):
#   ROBBOX_ZRAM_SIZE_MB       default 4096  (4 GB на 8 GB Pi)
#   ROBBOX_ZRAM_ALGO          default zstd  (zstd|lzo|lz4)
#   ROBBOX_ZRAM_PRIORITY      default 5     (swapon -p, выше = приоритетнее)
#   ROBBOX_ZRAM_SWAPPINESS    default 180   (рекомендация для zram)
#   ROBBOX_ZRAM_DRY_RUN       default 0     (1 = только print, ничего не менять)
#
# Что НЕ делает:
#   - Не трогает существующий файловый swap (dphys-swapfile). Если он
#     включён и конфликтует — оператор должен отключить его вручную.
#   - Не пишет /etc/fstab (управляется через systemd unit).
#   - Не устанавливает zram-tools / systemd-zram-generator (зависимостей нет).
#
# Требования:
#   - Linux kernel ≥ 3.15 (zram поддерживается с 3.14, stable ≥ 3.15)
#   - Модуль zram доступен (modprobe zram)
#   - systemd ≥ 232 (ExecStartPre работает)
#   - root (для modprobe/swapon). Проверяется в самом начале.
#
# Автор: devops worker (Hermes Agent) по ADR-0111, kanban t_22362046, issue #2621
# Дата: 2026-09-15
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

# ─── Defaults ──────────────────────────────────────────────────────────────────
: "${ROBBOX_ZRAM_SIZE_MB:=4096}"
: "${ROBBOX_ZRAM_ALGO:=zstd}"
: "${ROBBOX_ZRAM_PRIORITY:=5}"
: "${ROBBOX_ZRAM_SWAPPINESS:=180}"
: "${ROBBOX_ZRAM_DRY_RUN:=0}"

ZRAM_DEV="/dev/zram0"
STATE_FILE="/run/robbox-zram.state"   # маркер, что наш скрипт уже поднимал zram
SERVICE_FILE="/etc/systemd/system/robbox-zram.service"
SERVICE_NAME="robbox-zram.service"

# ─── Logging ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'
log_info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }
log_step()  { echo -e "\n${BLUE}──── $* ────${NC}"; }

# ─── Argument parsing ─────────────────────────────────────────────────────────
MODE="auto"   # auto | status | remove (--dry-run = auto + ROBBOX_ZRAM_DRY_RUN=1)
while [[ $# -gt 0 ]]; do
  case "$1" in
    --auto)     MODE="auto"; shift ;;
    --dry-run)  MODE="auto"; ROBBOX_ZRAM_DRY_RUN=1; shift ;;
    --status)   MODE="status"; shift ;;
    --remove)   MODE="remove"; shift ;;
    -h|--help)
      sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      log_error "Unknown argument: $1 (use --help)"
      exit 64
      ;;
  esac
done

# ─── Helpers ───────────────────────────────────────────────────────────────────
require_root() {
  if [[ $EUID -ne 0 ]]; then
    log_error "Требуются права root (sudo). Перезапустите с sudo или от root."
    exit 77
  fi
}

bytes_to_mb() { echo $(( $1 / 1024 / 1024 )); }

mb_to_bytes() { echo $(( $1 * 1024 * 1024 )); }

run_cmd() {
  # Передаём КАК массив аргументов: run_cmd echo hello
  # В dry-run режиме печатаем как одну строку (отображаемое имя команды).
  if [[ "${ROBBOX_ZRAM_DRY_RUN}" == "1" ]]; then
    log_info "[DRY-RUN] $*"
  else
    "$@"
  fi
}

# ─── Status subcommand ─────────────────────────────────────────────────────────
cmd_status() {
  log_step "Статус zram на этом хосте"
  echo
  echo "── /proc/swaps ──"
  if [[ -s /proc/swaps ]]; then
    cat /proc/swaps
  else
    log_warn "/proc/swaps пуст — своп не активен"
  fi
  echo
  echo "── /sys/block/zram0 ──"
  if [[ -e /sys/block/zram0 ]]; then
    echo "disksize: $(bytes_to_mb "$(cat /sys/block/zram0/disksize)") MB"
    echo "comp_algorithm: $(cat /sys/block/zram0/comp_algorithm | tr ' ' '\n' | grep -v '^$')"
    echo "mem_used_total: $(bytes_to_mb "$(cat /sys/block/zram0/mm_stat 2>/dev/null | awk '{print $2}')") MB"
  else
    log_warn "/sys/block/zram0 не существует — модуль не загружен или устройство не создано"
  fi
  echo
  echo "── vm.swappiness ──"
  echo "swappiness: $(cat /proc/sys/vm/swappiness)"
  echo
  echo "── systemd unit ──"
  if [[ -f "$SERVICE_FILE" ]]; then
    log_ok "$SERVICE_FILE существует"
    if command -v systemctl &>/dev/null; then
      systemctl is-enabled "$SERVICE_NAME" 2>/dev/null || log_warn "не enabled"
    fi
  else
    log_warn "$SERVICE_FILE НЕ существует — после перезагрузки zram не поднимется"
  fi
  exit 0
}

# ─── Remove subcommand ─────────────────────────────────────────────────────────
cmd_remove() {
  require_root
  log_step "Удаление zram-swap"
  if [[ -e /sys/block/zram0 ]]; then
    if grep -q zram0 /proc/swaps; then
      run_cmd swapoff "$ZRAM_DEV"
      log_ok "swapoff $ZRAM_DEV"
    fi
    run_cmd bash -c "echo 1 > /sys/block/zram0/reset"
    log_ok "zram0 reset"
  else
    log_warn "zram0 не активен, нечего удалять"
  fi
  if [[ -f "$STATE_FILE" ]]; then
    run_cmd rm -f "$STATE_FILE"
  fi
  log_ok "zram-swap отключён"
  exit 0
}

# ─── Validation ────────────────────────────────────────────────────────────────
validate_config() {
  # size
  if ! [[ "$ROBBOX_ZRAM_SIZE_MB" =~ ^[0-9]+$ ]] || [[ "$ROBBOX_ZRAM_SIZE_MB" -lt 256 ]]; then
    log_error "ROBBOX_ZRAM_SIZE_MB=$ROBBOX_ZRAM_SIZE_MB — должно быть целое ≥ 256 МБ"
    exit 64
  fi
  if [[ "$ROBBOX_ZRAM_SIZE_MB" -gt 65536 ]]; then
    log_error "ROBBOX_ZRAM_SIZE_MB=$ROBBOX_ZRAM_SIZE_MB — больше 64 ГБ не имеет смысла"
    exit 64
  fi
  # algo
  case "$ROBBOX_ZRAM_ALGO" in
    zstd|lzo|lz4|lz4hc|deflate|842) ;;
    *)
      log_error "ROBBOX_ZRAM_ALGO=$ROBBOX_ZRAM_ALGO — неподдерживаемый алгоритм"
      log_error "Допустимо: zstd lzo lz4 lz4hc deflate 842"
      exit 64
      ;;
  esac
  # priority
  if ! [[ "$ROBBOX_ZRAM_PRIORITY" =~ ^-?[0-9]+$ ]] || [[ "$ROBBOX_ZRAM_PRIORITY" -lt -32768 ]] || [[ "$ROBBOX_ZRAM_PRIORITY" -gt 32767 ]]; then
    log_error "ROBBOX_ZRAM_PRIORITY=$ROBBOX_ZRAM_PRIORITY — вне диапазона [-32768, 32767]"
    exit 64
  fi
  # swappiness
  if ! [[ "$ROBBOX_ZRAM_SWAPPINESS" =~ ^[0-9]+$ ]] || [[ "$ROBBOX_ZRAM_SWAPPINESS" -gt 200 ]]; then
    log_error "ROBBOX_ZRAM_SWAPPINESS=$ROBBOX_ZRAM_SWAPPINESS — вне диапазона [0, 200]"
    exit 64
  fi
}

# ─── Detection of current state ────────────────────────────────────────────────
detect_existing_zram() {
  if [[ ! -e /sys/block/zram0 ]]; then
    return 1   # не существует
  fi
  local current_size_mb current_algo
  current_size_mb=$(bytes_to_mb "$(cat /sys/block/zram0/disksize)")
  current_algo=$(cat /sys/block/zram0/comp_algorithm | grep -oE '\[?[a-z0-9]+\]?' | head -1 | tr -d '[]')
  echo "current_size_mb=$current_size_mb current_algo=$current_algo"
}

# ─── Apply (idempotent) ────────────────────────────────────────────────────────
apply_zram() {
  validate_config
  # В --dry-run режиме root НЕ обязателен (только print). Иначе — требуем.
  if [[ "${ROBBOX_ZRAM_DRY_RUN}" != "1" ]]; then
    require_root
  fi

  log_step "Конфигурация"
  log_info "ROBBOX_ZRAM_SIZE_MB=$ROBBOX_ZRAM_SIZE_MB ($((ROBBOX_ZRAM_SIZE_MB / 1024)) GB)"
  log_info "ROBBOX_ZRAM_ALGO=$ROBBOX_ZRAM_ALGO"
  log_info "ROBBOX_ZRAM_PRIORITY=$ROBBOX_ZRAM_PRIORITY"
  log_info "ROBBOX_ZRAM_SWAPPINESS=$ROBBOX_ZRAM_SWAPPINESS"
  log_info "ROBBOX_ZRAM_DRY_RUN=$ROBBOX_ZRAM_DRY_RUN"

  log_step "Проверка текущего состояния"
  local current
  if current=$(detect_existing_zram); then
    log_info "Обнаружен существующий zram0: $current"
    local cur_size cur_algo
    cur_size=$(echo "$current" | sed -n 's/.*current_size_mb=\([0-9]*\).*/\1/p')
    cur_algo=$(echo "$current" | sed -n 's/.*current_algo=\([a-z0-9]*\).*/\1/p')
    if [[ "$cur_size" == "$ROBBOX_ZRAM_SIZE_MB" && "$cur_algo" == "$ROBBOX_ZRAM_ALGO" ]]; then
      log_ok "Параметры совпадают — пропускаю активацию (идемпотентность)"
    else
      log_info "Параметры отличаются (size $cur_size→$ROBBOX_ZRAM_SIZE_MB, algo $cur_algo→$ROBBOX_ZRAM_ALGO) — пересоздаю"
      if grep -q zram0 /proc/swaps; then
        run_cmd swapoff "$ZRAM_DEV"
        log_ok "swapoff выполнен"
      fi
      run_cmd bash -c "echo 1 > /sys/block/zram0/reset"
      log_ok "zram0 reset"
      _do_init
    fi
  else
    log_info "zram0 не активен — создаю"
    # модуль может быть вкомпилирован в ядро (нет /sys/module/zram), тогда modprobe вернёт ошибку — игнорируем
    if [[ -d /sys/module/zram ]]; then
      log_info "Модуль zram уже загружен"
    else
      log_info "Загружаю модуль zram"
      run_cmd modprobe zram num_devices=1 || log_warn "modprobe вернул ошибку — возможно, модуль вкомпилирован"
    fi
    _do_init
  fi

  log_step "vm.swappiness"
  local current_swappiness
  current_swappiness=$(cat /proc/sys/vm/swappiness)
  if [[ "$current_swappiness" != "$ROBBOX_ZRAM_SWAPPINESS" ]]; then
    log_info "swappiness: $current_swappiness → $ROBBOX_ZRAM_SWAPPINESS"
    run_cmd bash -c "echo '$ROBBOX_ZRAM_SWAPPINESS' > /proc/sys/vm/swappiness"
  else
    log_ok "swappiness уже =$ROBBOX_ZRAM_SWAPPINESS"
  fi

  log_step "systemd unit для автоподнятия при загрузке"
  install_systemd_unit

  log_step "Готово"
  log_ok "zram-swap настроен"
  cmd_status
}

_do_init() {
  log_info "Устанавливаю comp_algorithm=$ROBBOX_ZRAM_ALGO"
  if [[ "${ROBBOX_ZRAM_DRY_RUN}" == "1" ]]; then
    log_info "[DRY-RUN] echo $ROBBOX_ZRAM_ALGO > /sys/block/zram0/comp_algorithm"
  else
    # Сначала выбираем алгоритм, потом disksize (порядок важен для некоторых ядер)
    if grep -q "$ROBBOX_ZRAM_ALGO" /sys/block/zram0/comp_algorithm; then
      echo "$ROBBOX_ZRAM_ALGO" > /sys/block/zram0/comp_algorithm
      log_ok "comp_algorithm=$ROBBOX_ZRAM_ALGO"
    else
      log_warn "алгоритм $ROBBOX_ZRAM_ALGO не поддерживается ядром (доступно: $(cat /sys/block/zram0/comp_algorithm)) — использую lzo (безопасный fallback)"
      echo "lzo" > /sys/block/zram0/comp_algorithm || true
    fi
  fi
  log_info "Устанавливаю disksize=$ROBBOX_ZRAM_SIZE_MB MB"
  run_cmd bash -c "echo '$(mb_to_bytes "$ROBBOX_ZRAM_SIZE_MB")' > /sys/block/zram0/disksize"
  log_ok "disksize установлен"
  log_info "mkswap $ZRAM_DEV"
  run_cmd mkswap "$ZRAM_DEV"
  log_ok "mkswap выполнен"
  log_info "swapon -p $ROBBOX_ZRAM_PRIORITY $ZRAM_DEV"
  run_cmd swapon -p "$ROBBOX_ZRAM_PRIORITY" "$ZRAM_DEV"
  log_ok "swapon выполнен"
  run_cmd touch "$STATE_FILE"
}

install_systemd_unit() {
  if [[ -f "$SERVICE_FILE" ]]; then
    log_ok "systemd unit уже установлен: $SERVICE_FILE"
    return
  fi
  log_info "Создаю $SERVICE_FILE"
  if [[ "${ROBBOX_ZRAM_DRY_RUN}" == "1" ]]; then
    log_info "[DRY-RUN] tee $SERVICE_FILE <<EOF"
    cat <<UNIT
[Unit]
Description=ROBBOX Vision Pi — zram-swap (issue #2621, ADR-0111)
Documentation=https://github.com/krikz/rob_box_project/blob/develop/docs/adr/0111-vision-pi-zram-swap-and-container-limits.md
After=systemd-modules-load.service
Before=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
EnvironmentFile=-/etc/default/robbox-zram
ExecStart=/usr/local/bin/robbox-zram-setup.sh
ExecStop=/sbin/swapoff /dev/zram0
TimeoutSec=30

[Install]
WantedBy=multi-user.target
UNIT
    return
  fi
  cat > "$SERVICE_FILE" <<'UNIT'
[Unit]
Description=ROBBOX Vision Pi — zram-swap (issue #2621, ADR-0111)
Documentation=https://github.com/krikz/rob_box_project/blob/develop/docs/adr/0111-vision-pi-zram-swap-and-container-limits.md
After=systemd-modules-load.service
Before=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
EnvironmentFile=-/etc/default/robbox-zram
# Ссылаемся на сам setup_vision_pi_swap.sh с --auto (идемпотентно)
ExecStart=/bin/bash -c 'ROBBOX_ZRAM_SIZE_MB=${ROBBOX_ZRAM_SIZE_MB:-4096} ROBBOX_ZRAM_ALGO=${ROBBOX_ZRAM_ALGO:-zstd} exec /opt/rob_box_project/scripts/setup/setup_vision_pi_swap.sh --auto'
ExecStop=/sbin/swapoff /dev/zram0
TimeoutSec=30

[Install]
WantedBy=multi-user.target
UNIT
  log_ok "systemd unit создан: $SERVICE_FILE"

  log_info "systemctl daemon-reload + enable"
  run_cmd systemctl daemon-reload
  run_cmd systemctl enable "$SERVICE_NAME"
  log_ok "unit enabled"
}

# ─── Dispatch ──────────────────────────────────────────────────────────────────
case "$MODE" in
  auto)    apply_zram ;;
  status)  cmd_status ;;
  remove)  cmd_remove ;;
esac