#!/usr/bin/env bash
# Снятие фактического адреса падения controller_server на Main Pi (#5180).
#
# Сам на робота НЕ ходит и сеть не открывает.
# Печатает блок команд. Координатор вставляет его в уже открытое окно
# на Main Pi и приносит сырой вывод сюда.
# Разбор адреса — scripts/diagnostics/nav2_sigill_locate.py, ему робот не нужен.
#
# Что блок снимает, пока процесс ещё жив или только что умер:
#   * dmesg: traps: controller_server[pid] illegal instruction / general protection
#   * /proc/<pid>/maps того же pid (из строки dmesg, не «любой controller_server»)
#   * coredumpctl / core_pattern / ulimit -c — если процесса уже нет
#   * ldd librclcpp.so, версия пакета, sha образа контейнера nav2
set -u

TARGET="${1:-print}"
case "$TARGET" in
    print) ;;
    *)
        echo "nav2_sigill_capture: только print. SSH и docker к роботу этот скрипт не открывает." >&2
        echo "Команды ниже вставляет координатор в уже открытое окно на Main Pi." >&2
        exit 2
        ;;
esac

cat << 'BLOCK'
# --- nav2 SIGILL capture, вставить в окно на Main Pi. Не запускать с katana. ---
set -u
OUT="${NAV2_SIGILL_OUT:-/tmp/nav2-sigill-capture}"
mkdir -p "$OUT"
date -u +%Y-%m-%dT%H:%M:%SZ | tee "$OUT/when.txt"

# Образ и пакет — с хоста, до входа в контейнер.
{
    echo "hostname=$(hostname)"
    docker inspect nav2 --format 'Image={{.Image}} Id={{.Id}} Status={{.State.Status}}' 2>&1 || true
    docker exec nav2 bash -lc 'dpkg-query -W -f "rclcpp=${Version}\n" ros-${ROS_DISTRO:-humble}-rclcpp 2>/dev/null || dpkg-query -W ros-humble-rclcpp' 2>&1 || true
    docker exec nav2 bash -lc 'ldd $(find /opt/ros -name librclcpp.so -print -quit) 2>&1 | head -40' 2>&1 || true
} > "$OUT/image.txt" 2>&1

# dmesg хоста: traps пишет ядро хоста, не контейнера.
dmesg -T 2>/dev/null | grep -E 'controller_server|illegal instruction|general protection fault' | tail -40 > "$OUT/dmesg.txt" || true

# pid из последней строки traps, не из pgrep: pgrep поймает уже другой процесс.
PID=$(grep -oE 'controller_server\[[0-9]+\]' "$OUT/dmesg.txt" | tail -1 | grep -oE '[0-9]+' || true)
echo "pid=${PID:-none}" | tee "$OUT/pid.txt"

if [ -n "${PID:-}" ] && [ -r "/proc/$PID/maps" ]; then
    cat "/proc/$PID/maps" > "$OUT/maps.txt"
    echo "maps=host-proc" >> "$OUT/pid.txt"
else
    # Процесс в контейнере: /proc на хосте может быть другим pid (namespace).
    docker exec nav2 bash -lc "
        pid=\$(ls -d /proc/[0-9]* 2>/dev/null | while read d; do
            tr '\\0' ' ' < \$d/cmdline 2>/dev/null | grep -q controller_server && basename \$d && break
        done)
        echo container_pid=\${pid:-none}
        if [ -n \"\${pid:-}\" ] && [ -r /proc/\$pid/maps ]; then cat /proc/\$pid/maps; fi
    " > "$OUT/maps.txt" 2>&1 || true
    echo "maps=container-or-missing" >> "$OUT/pid.txt"
fi

{
    echo "ulimit_c=$(ulimit -c 2>/dev/null || echo unknown)"
    echo "core_pattern=$(cat /proc/sys/kernel/core_pattern 2>/dev/null || echo unreadable)"
    if command -v coredumpctl >/dev/null 2>&1; then
        coredumpctl list controller_server 2>&1 | tail -20 || true
    else
        echo "coredumpctl=absent"
    fi
} > "$OUT/core.txt" 2>&1

echo "capture dir=$OUT"
# Разбор без робота, уже на katana:
#   python3 scripts/diagnostics/nav2_sigill_locate.py --dmesg dmesg.txt --maps maps.txt --json
BLOCK
