#!/usr/bin/env bash
# healthcheck_audio.sh — issue #2701: «здоровый» контейнер с зависшим захватом.
#
# 17.09.2026, Vision Pi: audio_node потерял вызовы audio_callback без
# исключения (PortAudio молчал), процесс жил, ReSpeaker был виден в
# lsusb, а старый healthcheck (`pgrep -f 'python3.*voice'`) весь вечер
# отвечал `Up (healthy)`. VAD/DoA по USB HID продолжали работать —
# каждая фраза уходила в «Речь отклонена: 0.00с», STT ничего не получал.
#
# Проверяем ДВА факта, как в healthcheck_frame.sh (ADR-0104):
#   1) процесс жив (pgrep) — fast-fail без лишней работы;
#   2) heartbeat-файл audio_node обновлялся не позже AUDIO_HEARTBEAT_MAX_AGE_S
#      назад. Файл пишет AudioNode._touch_audio_heartbeat() (см.
#      src/rob_box_voice/rob_box_voice/audio_node.py, issue #2701) на
#      каждом здоровом тике watchdog-таймера (1Hz), пока захват жив.
#      Если захват завис — watchdog это обнаружит и переоткроет поток
#      сам (audio_stall_timeout_s, default 5s), но ДО переоткрытия файл
#      не обновляется, и это видно здесь.
#
# Специально НЕ через `ros2 topic echo /audio/audio` (как healthcheck_frame.sh
# делает для vision-hailo): /audio/audio — best-effort топик с периодом
# 16-256мс, topic echo для него дороже и шумнее простого stat() файла, а
# voice-assistant хостит 9 нод в одном контейнере — здесь и так тесно по CPU.
#
# Использование в docker-compose.yaml:
#   healthcheck:
#     test: ["CMD-SHELL", "/scripts/healthcheck_audio.sh"]
#     interval: 15s
#     timeout: 5s
#     start_period: 20s
#     retries: 3

set -eo pipefail

HEARTBEAT_FILE="${AUDIO_HEARTBEAT_FILE:-/tmp/rob_box_audio_heartbeat}"
# 3x дефолтный audio_stall_timeout_s (5s, см. audio_node.yaml) — запас на
# джиттер watchdog-таймера (1Hz tick) и время самого переоткрытия потока.
MAX_AGE_S="${AUDIO_HEARTBEAT_MAX_AGE_S:-15}"

if ! pgrep -f 'python3.*voice' > /dev/null; then
    echo "[healthcheck_audio] FAIL: процесс voice-нод не найден" >&2
    exit 1
fi

if [ ! -f "$HEARTBEAT_FILE" ]; then
    # Честный FAIL (AGENTS.md), не «поверим на слово»: без heartbeat-файла
    # мы ничего не знаем о состоянии захвата. start_period в docker-compose
    # (20s) даёт ноде время его создать при старте — до истечения
    # start_period провалы healthcheck не считаются в retries.
    echo "[healthcheck_audio] FAIL: heartbeat-файл $HEARTBEAT_FILE не найден — захват ещё не поднялся" >&2
    exit 1
fi

NOW=$(date +%s)
MTIME=$(stat -c %Y "$HEARTBEAT_FILE" 2>/dev/null || echo 0)
AGE=$((NOW - MTIME))

if [ "$AGE" -gt "$MAX_AGE_S" ]; then
    echo "[healthcheck_audio] FAIL: heartbeat старше ${AGE}s (порог ${MAX_AGE_S}s) — захват завис (issue #2701)" >&2
    exit 1
fi

exit 0
