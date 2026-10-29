#!/bin/sh
# Осмотр колонки по SSH перед тем, как поднимать на ней чат РОББОКСа (ADR-0160 §8). Только чтение: ничего не ставит,
# не пишет и не перезапускает. POSIX sh — на колонке может не быть bash (busybox, Android toolbox).
#
#   ssh <колонка> 'sh -s' < scripts/station/inventory.sh > station_inventory.txt
#
# Каждый раздел печатается, даже если команды нет: «нет команды X» — тоже ответ.

section() { printf '\n===== %s =====\n' "$1"; }
have() { command -v "$1" >/dev/null 2>&1; }
run() { if have "$1"; then "$@" 2>&1 | head -n "${LINES_MAX:-60}"; else echo "нет команды $1"; fi; }
show() { for f in "$@"; do [ -r "$f" ] && { echo "--- $f"; head -n "${LINES_MAX:-60}" "$f" 2>&1; }; done; }

section "железо и ОС"
run uname -a
show /proc/device-tree/model /etc/os-release /etc/issue
grep -m 8 -iE 'hardware|model name|processor|features|cpu architecture|revision' /proc/cpuinfo 2>/dev/null
have nproc && echo "ядер: $(nproc)"
if have getprop; then  # Android
  for p in ro.product.model ro.product.device ro.build.version.release ro.build.version.sdk ro.product.cpu.abi; do
    echo "$p = $(getprop "$p")"; done
fi

section "память, диск, нагрузка"
show /proc/meminfo
run df -h
show /proc/loadavg
for z in /sys/class/thermal/thermal_zone*/temp; do [ -r "$z" ] && echo "$z: $(cat "$z")"; done

section "что можно запустить"
for b in python3 python python3.12 python3.11 pip3 busybox toybox opkg apt apk curl wget openssl git gcc; do
  if have "$b"; then echo "есть $b: $(command -v "$b")"; else echo "нет $b"; fi
done
have python3 && python3 -c 'import sys, ssl, sqlite3; print(sys.version); print(ssl.OPENSSL_VERSION); print("sqlite", sqlite3.sqlite_version)' 2>&1
run ldd --version
ls /lib/ld-* /lib64/ld-* /system/bin/linker* 2>/dev/null
ls -d /etc/ssl/certs /system/etc/security/cacerts 2>/dev/null

section "звук"
show /proc/asound/cards /proc/asound/pcm /etc/asound.conf
ls -l /dev/snd 2>/dev/null
run arecord -l
run aplay -l
for b in tinycap tinyplay tinymix pactl pulseaudio pipewire wpctl; do have "$b" && echo "есть $b"; done

section "процессы (что уже крутится на колонке)"
if ps -ef >/dev/null 2>&1; then LINES_MAX=150 run ps -ef; else LINES_MAX=150 run ps -A; fi

section "сеть"
run ip addr
if have ss; then run ss -tlnp; else run netstat -tlnp; fi
for url in https://api.deepseek.com https://tts.api.cloud.yandex.net https://api.minimax.io; do
  if have curl; then echo "$url: $(curl -s -o /dev/null -m 5 -w '%{http_code} %{time_total}s' "$url" 2>&1)"; fi
done

section "свои пакеты колонки (имена, без содержимого)"
for d in / /system /vendor /data /opt /usr/lib /usr/bin; do
  [ -d "$d" ] && find "$d" -maxdepth 3 \( -iname '*quasar*' -o -iname '*alice*' -o -iname '*yandex*' -o -iname '*spotter*' \
    -o -iname '*wake*' -o -iname '*vad*' -o -iname '*aec*' -o -iname '*beamform*' -o -iname '*tts*' \) 2>/dev/null | head -n 40
done

section "куда можно писать"
run mount
ls -ld /data /tmp /opt /home/* 2>/dev/null

section "светодиоды и кнопки"
ls /sys/class/leds 2>/dev/null
ls /dev/input 2>/dev/null

echo
echo "осмотр закончен: $(date -u '+%Y-%m-%dT%H:%M:%SZ' 2>/dev/null)"
