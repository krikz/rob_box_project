#!/bin/bash
# ollama/entrypoint.sh — запуск Ollama и автоматическая загрузка модели
#
# Если модель уже есть в volume (ollama-models) — загрузка занимает секунды.
# При первом запуске — скачивает модель (qwen2.5:0.5b ~350MB).

set -e

MODEL="${LLM_MODEL:-qwen2.5:0.5b}"

echo "=================================================="
echo "  Ollama LLM Server"
echo "  Model: $MODEL"
echo "=================================================="

# Запускаем сервер в фоне
ollama serve &
OLLAMA_PID=$!

# Ждём пока сервер стартует (ollama list обращается к серверу через unix socket)
echo "Ожидание Ollama API..."
until ollama list > /dev/null 2>&1; do
    sleep 1
done
echo "✓ Ollama API доступен"

# Проверяем есть ли модель в кэше
if ollama list | grep -q "$MODEL"; then
    echo "✓ Модель $MODEL уже в кэше"
else
    echo "Загрузка модели $MODEL (первый запуск)..."
    ollama pull "$MODEL"
    echo "✓ Модель загружена"
fi

# Прогрев: загружаем модель в память заранее (первый inference медленный)
echo "Прогрев модели (first inference)..."
echo "hi" | ollama run "$MODEL" > /dev/null 2>&1 || true
echo "✓ Модель готова к работе"

echo ""
echo "Ollama готов: http://localhost:11435/v1 (OpenAI-compatible)"
echo "Доступные модели:"
ollama list

# Флаг-файл: healthcheck проверяет его, а не дёргает API tags
# (curl -sf api/tags | grep model может не работать пока идёт pull)
touch /tmp/ollama_ready
echo "✓ Ollama полностью готов: http://localhost:11435"

# Ждём пока основной процесс завершится
wait $OLLAMA_PID
