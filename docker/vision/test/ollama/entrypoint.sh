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

# Ждём пока сервер стартует
echo "Ожидание Ollama API..."
until curl -sf http://localhost:11434/api/tags > /dev/null 2>&1; do
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

# Прогрев: первый запрос иногда медленный — делаем cold start здесь
echo "Прогрев модели (first inference)..."
curl -s http://localhost:11434/api/generate \
    -d "{\"model\": \"$MODEL\", \"prompt\": \"hi\", \"stream\": false}" \
    > /dev/null
echo "✓ Модель готова к работе"

echo ""
echo "Ollama готов: http://localhost:11434/v1 (OpenAI-compatible)"
echo "Доступные модели:"
ollama list

# Ждём пока основной процесс завершится
wait $OLLAMA_PID
