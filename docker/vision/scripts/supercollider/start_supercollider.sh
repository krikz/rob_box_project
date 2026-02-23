#!/bin/bash
# start_supercollider.sh — запуск SuperCollider synthesis server в headless-режиме
#
# Стратегия: запускаем JACK (no-realtime, ALSA → ReSpeaker), затем scsynth поверх него.
# Это позволяет SC микшироваться через dmix вместе с TTS и sound_node.
#
# Опции scsynth:
#   -u 57110   UDP OSC-порт (Renardo/FoxDot подключается сюда)
#   -D 0       Отключить realtime scheduling (необходимо в Docker)
#   -m 8192    Размер realtime-памяти в KB
#   -z 512     Размер буфера (block size, samples)
#   -S 16000   Частота дискретизации (ReSpeaker UAC1.0 поддерживает только 16000 Hz)
#   -H jack    JACK audio backend (JACK → ALSA → dmix → ReSpeaker)
#   -a 1024    Число аудио-шин

set -euo pipefail

echo "[SuperCollider] Starting JACK (no-realtime, ALSA backend → ReSpeaker)..."

# Start JACK with no-realtime to avoid memlock issues in Docker.
# Uses ALSA card by name so it works regardless of card number.
jackd --no-realtime \
    -d alsa \
    -d hw:CARD=ArrayUAC10,DEV=0 \
    -r 16000 \
    -p 512 \
    -n 2 \
    2>&1 | sed 's/^/[jackd] /' &

JACK_PID=$!
echo "[SuperCollider] Waiting for JACK to start (pid=$JACK_PID)..."
sleep 3

# Check JACK is alive
if ! kill -0 $JACK_PID 2>/dev/null; then
    echo "[SuperCollider] ERROR: JACK failed to start!"
    exit 1
fi

echo "[SuperCollider] JACK running. Starting scsynth on UDP port 57110..."

exec scsynth \
    -u 57110 \
    -D 0 \
    -m 8192 \
    -z 512 \
    -S 16000 \
    -H jack \
    -a 1024
