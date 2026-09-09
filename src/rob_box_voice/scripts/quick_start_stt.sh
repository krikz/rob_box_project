#!/bin/bash
# Quick Start для тестирования STT с ReSpeaker

set -e

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║       🎤 QUICK START: STT Testing для ROBBOX                  ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Проверка ReSpeaker
echo "1️⃣  Проверка ReSpeaker..."
if lsusb | grep -q "2886:0018"; then
    echo "   ✅ ReSpeaker Mic Array найден"
else
    echo "   ❌ ReSpeaker НЕ найден!"
    echo "   Подключите ReSpeaker Mic Array v2.0"
    exit 1
fi

# Установка зависимостей
echo ""
echo "2️⃣  Установка системных зависимостей..."
# Проверяем portaudio (нужен для pyaudio)
if ! dpkg -l | grep -q portaudio19-dev; then
    echo "   📦 Устанавливаю portaudio19-dev..."
    sudo apt-get update -qq
    sudo apt-get install -y portaudio19-dev
else
    echo "   ✅ portaudio19-dev уже установлен"
fi

echo "3️⃣  Установка Python зависимостей..."
pip install vosk pyaudio sounddevice numpy

# Проверка Vosk модели
echo ""
echo "4️⃣  Проверка Vosk модели..."
if [ -d "/models/vosk-model-small-ru-0.22" ]; then
    echo "   ✅ Vosk модель найдена"
else
    echo "   ⏳ Скачивание Vosk модели (45 MB)..."
    sudo mkdir -p /models
    cd /tmp
    wget -q https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip
    unzip -q vosk-model-small-ru-0.22.zip
    sudo mv vosk-model-small-ru-0.22 /models/
    rm vosk-model-small-ru-0.22.zip
    cd - > /dev/null
    echo "   ✅ Vosk модель установлена"
fi

# Опционально: Whisper
echo ""
echo "5️⃣  Whisper (опционально, большой)..."
if python3 -c "import whisper" 2>/dev/null; then
    echo "   ✅ Whisper установлен"
else
    echo "   ⏳ Whisper НЕ установлен (для тестов можно установить позже)"
    echo "      pip install openai-whisper torch torchaudio"
fi

# Запуск теста
echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                   ✅ ГОТОВО К ЗАПУСКУ!                         ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "Запуск тестового скрипта..."
echo ""

cd /home/ros2/rob_box_project/src/rob_box_voice/scripts

# Загрузить environment если есть
if [ -f "../.env.secrets" ]; then
    set -a
    source ../.env.secrets
    set +a
fi

# ЗАПУСК
python3 test_stt_options.py
