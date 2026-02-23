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

echo "[SuperCollider] Starting JACK via dmix_respeaker (period=1024, rate=16000)..."

jackd --no-realtime \
    -d alsa \
    -d dmix_respeaker \
    -r 16000 \
    -p 1024 \
    -n 2 \
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

exec scsynth \
    -u 57110 \
    -D 0 \
    -m 8192 \
    -z 1024 \
    -S 16000 \
    -H jack \
    -a 1024
