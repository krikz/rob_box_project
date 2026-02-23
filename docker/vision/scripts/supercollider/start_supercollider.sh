#!/bin/bash
# start_supercollider.sh — запуск SuperCollider synthesis server в headless-режиме
#
# Стратегия: scsynth напрямую через ALSA dmix (без JACK).
# asound.conf монтируется в /etc/asound.conf и задаёт pcm.!default = dmix_respeaker,
# что позволяет SC воспроизводить звук одновременно с TTS и sound_node через dmix.
#
# Опции scsynth:
#   -u 57110   UDP OSC-порт (Renardo/FoxDot подключается сюда)
#   -D 0       Отключить realtime scheduling (необходимо в Docker)
#   -m 8192    Размер realtime-памяти в KB
#   -z 512     Размер буфера (block size, samples)
#   -S 16000   Частота дискретизации (ReSpeaker UAC1.0 поддерживает только 16000 Hz)
#   -H alsa    ALSA backend (использует pcm.!default из asound.conf → dmix_respeaker)
#   -a 1024    Число аудио-шин

set -euo pipefail

# Убираем JACK reservation через D-Bus (не нужен, используем dmix напрямую)
export JACK_NO_AUDIO_RESERVATION=1

echo "[SuperCollider] Starting scsynth with ALSA backend (dmix_respeaker)..."
echo "[SuperCollider] Audio config: rate=16000, blocksize=512, port=57110"

exec scsynth \
    -u 57110 \
    -D 0 \
    -m 8192 \
    -z 512 \
    -S 16000 \
    -H alsa \
    -a 1024
