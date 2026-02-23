#!/bin/bash
# start_supercollider.sh — запуск SuperCollider synthesis server в headless-режиме
#
# Стратегия: запускаем JACK (no-realtime, ALSA → plug:dmix_respeaker), затем scsynth.
#
# plug:dmix_respeaker — виртуальное устройство из asound.conf:
#   plug:    = libasound plugin (rate/format conversion)
#   dmix:    = software mixer (позволяет шерить ReSpeaker с voice-assistant TTS)
#
# Опции scsynth:
#   -u 57110   UDP OSC-порт (Renardo/FoxDot подключается сюда)
#   -D 0       Отключить realtime scheduling (необходимо в Docker)
#   -m 8192    Размер realtime-памяти в KB
#   -z 512     Размер буфера (block size, samples)
#   -S 16000   Частота дискретизации (ReSpeaker UAC1.0 поддерживает только 16000 Hz)
#   -H jack    JACK backend (подключается к нашему jackd)
#   -a 1024    Число аудио-шин

set -euo pipefail

# Пропустить D-Bus device reservation (нет X11/dbus в контейнере)
export JACK_NO_AUDIO_RESERVATION=1

echo "[SuperCollider] Starting JACK via plug:dmix_respeaker (shared ALSA mixer)..."

# plug:dmix_respeaker → libasound → dmix → hw:ArrayUAC10
# -P = playback only (dmix is write-only mixer)
jackd --no-realtime \
    -d alsa \
    -d plug:dmix_respeaker \
    -r 16000 \
    -p 512 \
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
    -z 512 \
    -S 16000 \
    -H jack \
    -a 1024
