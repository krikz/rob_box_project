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
echo "Ожидание Zenoh router (localhost:17447)..."
RETRY=0
while [ $RETRY -lt 30 ]; do
    if nc -z localhost 17447 2>/dev/null; then
        echo "✓ Zenoh router доступен"
        break
    fi
    sleep 2
    RETRY=$((RETRY + 1))
done

echo ""
echo "Запуск dialogue_node (test config)..."
echo "  LLM:       DeepSeek Chat (через LiteLLM proxy localhost:4000)"
echo "  MCP tools: enabled"
echo "  Wake words: empty (прямой диалог)"
echo ""

# Ждём DeepSeek proxy (на билд-машине)
echo "Ожидание DeepSeek proxy (localhost:4000)..."
RETRY=0
while [ $RETRY -lt 20 ]; do
    if wget -qO- http://localhost:4000/health 2>/dev/null | grep -q "healthy"; then
        echo "✓ DeepSeek proxy доступен"
        break
    fi
    sleep 3
    RETRY=$((RETRY + 1))
done

# ROS 2 Humble bug: --params-file не применяется для Python-нод
# (declare_parameter дефолт побеждает). Используем -p вместо.
exec ros2 run rob_box_voice dialogue_node \
    --ros-args \
    -p provider:=deepseek \
    -p api_key:="not-needed" \
    -p base_url:="http://localhost:4000/v1" \
    -p model:="deepseek-chat" \
    -p temperature:=0.5 \
    -p max_tokens:=150 \
    -p streaming:=true \
    -p enable_fallback:=false \
    -p enable_mcp_tools:=true \
    -p silence_words:="стоп,тихо,замолчи"
