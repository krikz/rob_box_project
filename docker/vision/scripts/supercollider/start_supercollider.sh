#!/bin/bash
# start_supercollider.sh — запуск SuperCollider synthesis server в headless-режиме
#
# Запускает scsynth с ALSA-бэкендом (через dmix из /etc/asound.conf).
# Порт 57110 (UDP) — стандартный OSC-порт для Renardo/FoxDot.
#
# Опции scsynth:
#   -u 57110   UDP OSC-порт (Renardo/FoxDot подключается сюда)
#   -D 0       Отключить realtime scheduling (необходимо в Docker)
#   -m 8192    Размер realtime-памяти в KB
#   -z 512     Размер буфера (block size, samples)
#   -S 44100   Частота дискретизации
#   -H alsa    ALSA audio backend (использует 'default' device → dmix)
#   -a 1024    Число аудио-шин

set -euo pipefail

echo "[SuperCollider] Starting scsynth (headless) on UDP port 57110..."
echo "[SuperCollider] ALSA device: default (dmix via /etc/asound.conf)"

exec scsynth \
    -u 57110 \
    -D 0 \
    -m 8192 \
    -z 512 \
    -S 44100 \
    -H alsa \
    -a 1024
