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
    -a 1024 &

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

wait $SCSYNTH_PID
