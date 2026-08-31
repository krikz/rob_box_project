#!/bin/bash
# start_supercollider.sh — запуск SuperCollider synthesis server в headless-режиме
#
# Стратегия: JACK (no-realtime) → dmix_respeaker → ReSpeaker.
# dmix_respeaker определён в asound.conf, тот же шейринг что у voice-assistant TTS.
#
# ВАЖНО: period_size в jackd и scsynth ДОЛЖЕН совпадать с period_size в asound.conf (1024).
#
# Опции scsynth:
#   -u 57110   UDP OSC-порт (Renardo/FoxDot подключается сюда)
#   -D 0       Отключить realtime scheduling (необходимо в Docker)
#   -m 8192    Размер realtime-памяти в KB
#   -z 1024    Размер буфера = period_size dmix (JACK требует совпадения)
#   -S 16000   Частота дискретизации (ReSpeaker UAC1.0 поддерживает только 16000 Hz)
#   -H jack    JACK backend
#   -a 1024    Число аудио-шин
#   -l 32      maxLogins = 32 (issue #1363 — sclang по умолчанию ожидает ≤32
#              клиента; дефолт сервера 64 → client 32+ получают nodeID
#              0x80000001+ который как signed int32 = отрицательный
#              → "FAILURE IN SERVER /g_new negative node IDs are reserved"
#              в логах. Свист из динамика — побочный эффект: клиентский
#              Group 1 не создаётся, renardo Player-ы шлют /s_new в пустоту
#              и часть нод "зависает" активной → постоянный тон через JACK.)

set -euo pipefail

export JACK_NO_AUDIO_RESERVATION=1

# ── Clean up stale JACK SHM files from previous crashes ──────────────────────
# Without this, jackd fails with "default server already active" after
# an ungraceful shutdown that leaves orphaned SHM/semaphore files in /dev/shm/.
echo "[SuperCollider] Cleaning up stale JACK SHM files..."
rm -f /dev/shm/jack-0-0 /dev/shm/jack-0-1 /dev/shm/jack_default_0_0 2>/dev/null || true
rm -f /dev/shm/jack_sem.0_default_* 2>/dev/null || true
rm -f /dev/shm/jack-shm-registry 2>/dev/null || true
rm -rf /dev/shm/jack_db-* 2>/dev/null || true

# ── Prometheus metrics endpoint (issue #1160) ────────────────────────────────
# Лёгкий stdlib HTTP-сервер на 9102 (см. metrics_server.py и
# docker/monitoring/config/prometheus.yml target 10.1.1.11:9102).
# Не блокирует запуск JACK/scsynth: если python3 отсутствует — просто
# логируем предупреждение и продолжаем.
echo "[SuperCollider] Starting metrics server on :9102/metrics..."
python3 /scripts/metrics_server.py 2>&1 | sed 's/^/[metrics] /' &
METRICS_PID=$!
echo "[SuperCollider] Metrics server pid=$METRICS_PID"

echo "[SuperCollider] Starting JACK via dmix_respeaker (period=1024, rate=16000, nperiods=3)..."

jackd --no-realtime \
    -d alsa \
    -d dmix_respeaker \
    -r 16000 \
    -p 1024 \
    -n 3 \
    -P \
    2>&1 | sed 's/^/[jackd] /' &

JACK_PID=$!
echo "[SuperCollider] Waiting for JACK to start (pid=$JACK_PID)..."
sleep 3

if ! kill -0 $JACK_PID 2>/dev/null; then
    echo "[SuperCollider] ERROR: JACK failed to start!"
    exit 1
fi

echo "[SuperCollider] JACK running. Starting scsynth on UDP port 57110..."

scsynth \
    -u 57110 \
    -D 0 \
    -m 65536 \
    -z 1024 \
    -S 16000 \
    -H jack \
    -a 1024 \
    -l 32 &

SCSYNTH_PID=$!

# Ждём пока scsynth зарегистрируется в JACK как клиент 'jack'
echo "[SuperCollider] Waiting for scsynth JACK ports..."
for i in $(seq 1 20); do
    if jack_lsp 2>/dev/null | grep -q "^jack:out_1$"; then
        break
    fi
    sleep 0.5
done

# Подключаем выходы scsynth к физическому выходу ALSA
# scsynth регистрируется как клиент 'jack' (имя по умолчанию)
if jack_lsp 2>/dev/null | grep -q "^jack:out_1$"; then
    jack_connect jack:out_1 system:playback_1 && echo "[SuperCollider] Connected jack:out_1 -> system:playback_1"
    jack_connect jack:out_2 system:playback_2 && echo "[SuperCollider] Connected jack:out_2 -> system:playback_2"
else
    echo "[SuperCollider] WARNING: scsynth JACK ports not found, audio may be silent"
fi

# ── Группа 1 — дефолтная группа клиента ──────────────────────────────────────
#
# 🔴 FIX (live 31.08, робот молчал при полностью здоровых логах): Renardo
# заводит группу на каждый плеер как ДОЧЕРНЮЮ к группе 1
# (ServerManager.get_bundle: `/g_new [id, 1, 1]`) и все ноты кладёт внутрь неё.
# Саму группу 1 создаёт клиент при инициализации — один раз.
#
# Свежий scsynth поднимается только с RootNode. Если он рестартовал ПОСЛЕ
# того, как Renardo проинициализировался (перезапуск контейнера, краш,
# порядок подъёма при деплое), группы 1 больше нет, а пересоздать её некому:
#
#     FAILURE IN SERVER /g_new Group 1 not found
#     FAILURE IN SERVER /s_new Group 6469 not found
#
# То есть /g_new плеера падает, а следом отвергается КАЖДАЯ нота. Снаружи это
# выглядит идеально: контейнеры healthy, sclang рапортует прелоад, tool
# отвечает «успешно», в логе лежит корректный код композиции — и полная
# тишина. Диагностируется только опросом самого сервера.
#
# Поэтому группу заводит сам сервер при старте: тогда она есть с первой
# секунды жизни scsynth и переживает любой рестарт клиента.
if command -v python3 > /dev/null 2>&1; then
    python3 - <<'PYEOF'
import socket
import struct

ZERO = b"\x00"


def osc_string(s):
    # Строка OSC обязана иметь минимум один нулевой терминатор и длину,
    # кратную 4. Для строк длиной кратной 4 это ещё 4 нуля, а не ноль.
    b = s.encode() + ZERO
    while len(b) % 4:
        b += ZERO
    return b


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# /g_new <id> <addAction> <target>: группа 1, addToHead (0) от RootNode (0).
sock.sendto(
    osc_string("/g_new") + osc_string(",iii") + struct.pack(">iii", 1, 0, 0),
    ("127.0.0.1", 57110),
)
sock.close()
PYEOF
    echo "[SuperCollider] Default client group 1 created"
else
    echo "[SuperCollider] WARNING: python3 missing, group 1 not created — Renardo will be silent"
fi

wait $SCSYNTH_PID
