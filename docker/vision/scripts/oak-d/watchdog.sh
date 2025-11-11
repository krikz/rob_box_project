#!/bin/bash
# OAK-D Camera Health Monitor Watchdog
# Monitors camera health and restarts container on failure

set -e

CONTAINER_NAME="oak-d"
CHECK_INTERVAL=30  # Проверка каждые 30 секунд
ERROR_THRESHOLD=5  # Перезапуск после 5 ошибок подряд
LOG_FILE="/tmp/oak-d-watchdog.log"

echo "🐕 OAK-D Watchdog запущен" | tee -a "$LOG_FILE"
echo "   Контейнер: $CONTAINER_NAME" | tee -a "$LOG_FILE"
echo "   Интервал проверки: ${CHECK_INTERVAL}s" | tee -a "$LOG_FILE"
echo "   Порог ошибок: $ERROR_THRESHOLD" | tee -a "$LOG_FILE"

error_count=0
last_check_ok=true

while true; do
    sleep "$CHECK_INTERVAL"
    
    # Проверяем, что контейнер запущен
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] ⚠️  Контейнер $CONTAINER_NAME не запущен" | tee -a "$LOG_FILE"
        error_count=0  # Сбрасываем счётчик, если контейнер остановлен вручную
        continue
    fi
    
    # Проверяем логи на наличие X_LINK_ERROR
    recent_logs=$(docker logs "$CONTAINER_NAME" --tail 50 2>&1)
    
    if echo "$recent_logs" | grep -q "X_LINK_ERROR"; then
        error_count=$((error_count + 1))
        
        if [ "$last_check_ok" = true ]; then
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] ❌ Обнаружена ошибка X_LINK_ERROR (${error_count}/${ERROR_THRESHOLD})" | tee -a "$LOG_FILE"
            last_check_ok=false
        fi
        
        # Если достигнут порог ошибок - перезапускаем контейнер
        if [ "$error_count" -ge "$ERROR_THRESHOLD" ]; then
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] 🔄 КРИТИЧНО: ${ERROR_THRESHOLD} ошибок подряд - перезапуск контейнера" | tee -a "$LOG_FILE"
            
            docker restart "$CONTAINER_NAME"
            
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] ✅ Контейнер перезапущен" | tee -a "$LOG_FILE"
            error_count=0
            last_check_ok=true
            
            # Ждём 60 секунд после перезапуска перед следующей проверкой
            sleep 60
        fi
    elif echo "$recent_logs" | grep -q "No data on logger queue"; then
        error_count=$((error_count + 1))
        
        if [ "$last_check_ok" = true ]; then
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] ⚠️  Обнаружена ошибка: No data on logger queue (${error_count}/${ERROR_THRESHOLD})" | tee -a "$LOG_FILE"
            last_check_ok=false
        fi
        
        if [ "$error_count" -ge "$ERROR_THRESHOLD" ]; then
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] 🔄 КРИТИЧНО: ${ERROR_THRESHOLD} ошибок подряд - перезапуск контейнера" | tee -a "$LOG_FILE"
            
            docker restart "$CONTAINER_NAME"
            
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] ✅ Контейнер перезапущен" | tee -a "$LOG_FILE"
            error_count=0
            last_check_ok=true
            
            sleep 60
        fi
    else
        # Нет ошибок - сбрасываем счётчик
        if [ "$error_count" -gt 0 ]; then
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] ✅ Ошибки устранены, счётчик сброшен" | tee -a "$LOG_FILE"
        fi
        error_count=0
        last_check_ok=true
    fi
done
