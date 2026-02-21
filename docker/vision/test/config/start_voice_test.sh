#!/bin/bash
# start_voice_test.sh — запуск только dialogue_node в тестовом режиме
# (без hardware: нет audio_node, stt_node, led_node)
#
# Hardware замоканы:
#   - STT  → scenario-runner публикует в /voice/stt/result напрямую
#   - VAD  → scenario-runner публикует в /audio/vad напрямую
#   - TTS  → /voice/dialogue/response читает scenario-runner (не tts_node)
#   - LED  → /voice/animation/request читает scenario-runner (проверка publish)
#
# LLM → DeepSeek API (api.deepseek.com) — ключ через env DEEPSEEK_API_KEY

set -e

echo "=================================================="
echo "  Voice Assistant TEST mode (dialogue_node only)"
echo "=================================================="

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source /ws/install/setup.bash

# Ждём Zenoh router
echo "Ожидание Zenoh router (localhost:7447)..."
RETRY=0
while [ $RETRY -lt 30 ]; do
    if nc -z localhost 7447 2>/dev/null; then
        echo "✓ Zenoh router доступен"
        break
    fi
    sleep 2
    RETRY=$((RETRY + 1))
done

echo ""
echo "Запуск dialogue_node (test config)..."
echo "  LLM:       DeepSeek API (deepseek-chat)"
echo "  MCP tools: disabled"
echo "  Wake words: empty (прямой диалог)"
echo ""

exec ros2 run rob_box_voice dialogue_node \
    --ros-args \
    --params-file "${VOICE_TEST_CONFIG:-/test-config/voice_assistant_test.yaml}"
