# Quick Reference: Zenoh Transport Error Fix

**Дата:** 2025-11-09  
**Обновлено:** 2025-11-10 (Максимальное исправление)  
**Проблема:** `ERROR: Unable to push non droppable network message. Closing transport!` + `ERROR: Cannot find link 1`  
**Статус:** ⚠️ Максимальное исправление применено (2025-11-10), требует тестирования

---

## 🎯 Что было сделано

### История исправлений

**2025-11-09 - Базовое исправление:**
1. Увеличен размер TX очередей с 2 до 4 batch (удвоение буфера)
2. Увеличен таймаут wait_before_close с 5 до 20 секунд

**2025-11-10 - Усиленное исправление:**
1. Приоритизация критичных очередей:
   - `control` и `real_time`: 4 → 8 batch (для камеры и LiDAR)
   - `data_high`: 4 → 6 batch (для TF и Nav2)
   - `data_low` и `background`: 4 → 2 batch (освобождение памяти)
2. Дополнительное увеличение таймаута: 20 → 30 секунд
3. **Результат:** ❌ Ошибки продолжались каждые 30 секунд

**2025-11-10 - МАКСИМАЛЬНОЕ исправление (текущее):**
1. Увеличение до максимальных значений:
   - `control` и `real_time`: 8 → **16 batch (МАКСИМУМ)** - 1024 KB буфер
   - `data_high`: 6 → **12 batch** - 768 KB буфер
2. Удвоение таймаута: 30 → **60 секунд**
3. Синхронизация `peers_failover_brokering`: Vision Pi false → **true**

### Затронутые файлы

- ✅ `docker/vision/config/zenoh_router_config.json5`
- ✅ `docker/main/config/zenoh_router_config.json5`

---

## 🚀 Применение исправлений

### На Vision Pi

```bash
# SSH в Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21

# Перейти в директорию проекта
cd ~/rob_box_project/docker/vision

# Обновить конфигурацию (если нужно)
git pull

# Перезапустить Zenoh router
docker compose restart zenoh-router

# Проверить логи
docker logs zenoh-router --tail 50
```

### На Main Pi

```bash
# SSH в Main Pi
sshpass -p 'open' ssh ros2@10.1.1.20

# Перейти в директорию проекта
cd ~/rob_box_project/docker/main

# Обновить конфигурацию (если нужно)
git pull

# Перезапустить Zenoh router
docker compose restart zenoh-router

# Проверить логи
docker logs zenoh-router --tail 50
```

---

## 🔍 Проверка результатов

### 1. Проверить отсутствие ошибок

```bash
# На Vision Pi
docker logs zenoh-router 2>&1 | grep "Unable to push"

# На Main Pi
docker logs zenoh-router 2>&1 | grep "Unable to push"

# Ожидаемый результат: пустой вывод (нет ошибок)
```

### 2. Проверить статус роутера

```bash
# На любом Pi
docker logs zenoh-router --tail 20

# Должны видеть:
# - Successful connection to peer
# - No transport closure errors
```

### 3. Проверить ROS 2 топики

```bash
# Проверить камеру
ros2 topic hz /camera/rgb/image_raw

# Проверить LiDAR
ros2 topic hz /scan

# Ожидаемый результат: стабильная частота без пропусков
```

---

## 📊 Технические детали

### Первоначальная конфигурация (до 2025-11-09)

```json5
queue: {
  size: {
    control: 2,        // 2 × 64KB = 128KB
    // ... все по 2
  },
  congestion_control: {
    block: {
      wait_before_close: 5000000,  // 5 секунд
    },
  },
}
```

### Базовое исправление (2025-11-09)

```json5
queue: {
  size: {
    control: 4,        // 4 × 64KB = 256KB
    // ... все по 4
  },
  congestion_control: {
    block: {
      wait_before_close: 20000000,  // 20 секунд
    },
  },
}
```

### Максимальное исправление (2025-11-10) ⭐ ТЕКУЩЕЕ

```json5
queue: {
  size: {
    control: 16,          // 16 × 64KB = 1024KB ⬆️⬆️⬆️ МАКСИМУМ
    real_time: 16,        // 16 × 64KB = 1024KB ⬆️⬆️⬆️ МАКСИМУМ
    interactive_high: 4,  // 4 × 64KB = 256KB
    interactive_low: 4,   // 4 × 64KB = 256KB
    data_high: 12,        // 12 × 64KB = 768KB ⬆️⬆️
    data: 4,              // 4 × 64KB = 256KB
    data_low: 2,          // 2 × 64KB = 128KB
    background: 2,        // 2 × 64KB = 128KB
  },
  congestion_control: {
    block: {
      wait_before_close: 60000000,  // 60 секунд ⬆️⬆️
    },
  },
}
```

### Сравнительная таблица

| Параметр | Было | Базовое | Усиленное | Максимальное | Улучшение |
|----------|------|---------|-----------|--------------|-----------|
| **Буфер control** | 128 KB | 256 KB | 512 KB | **1024 KB** | +700% |
| **Буфер real_time** | 128 KB | 256 KB | 512 KB | **1024 KB** | +700% |
| **Буфер data_high** | 128 KB | 256 KB | 384 KB | **768 KB** | +500% |
| **Таймаут** | 5 сек | 20 сек | 30 сек | **60 сек** | +1100% |
| **Использование RAM** | 1 MB | 2 MB | 3 MB | **~3.8 MB** | +2.8 MB |
| **Устойчивость к пикам** | Низкая | Средняя | Высокая | **Максимальная** | ✅✅✅✅ |

---

## 🔗 Дополнительная информация

**Полный анализ:**  
📄 [ZENOH_TRANSPORT_ERROR_ANALYSIS_2025-11-09.md](../reports/ZENOH_TRANSPORT_ERROR_ANALYSIS_2025-11-09.md)

**Максимальное исправление:**  
📄 [ZENOH_FIX_2025-11-10_MAXIMUM.md](ZENOH_FIX_2025-11-10_MAXIMUM.md)

**Связанные документы:**
- [SYSTEM_OVERVIEW.md](../architecture/SYSTEM_OVERVIEW.md)
- [SOFTWARE.md](../architecture/SOFTWARE.md)
- [DOCKER_STANDARDS.md](../development/DOCKER_STANDARDS.md)

---

## ❓ Если проблема сохраняется

Если после применения **МАКСИМАЛЬНОГО исправления** ошибки продолжают появляться:

⚠️ **КРИТИЧНО:** Вы уже используете максимальные значения Zenoh (control/real_time: 16 batches)!

Дальнейшие действия требуют фундаментальных изменений:

3. **Проверить сетевую инфраструктуру:**
   ```bash
   # Скорость Ethernet
   ethtool eth0
   
   # Проверка связи
   ping 10.1.1.10 -c 100
   
   # Статистика ошибок сети
   ip -s link show eth0
   ```

4. **Проверить нагрузку на процессор и память:**
   ```bash
   htop
   free -h
   ```

5. **Оптимизировать публикацию данных:**
   - Включить сжатие изображений камеры
   - Уменьшить частоту публикации некритичных топиков
   - Использовать downsampling для облаков точек

6. **Рассмотреть альтернативы:**
   - Обновление до более новой версии ROS 2 (если доступно)
   - Переход на Zenoh 1.0+ (требует тестирования)
   - Использование выделенного Ethernet для данных SLAM

7. **Создать issue** в репозитории с полными логами:
   ```bash
   # Main Pi
   docker logs zenoh-router > /tmp/main_zenoh.log 2>&1
   
   # Vision Pi
   docker logs zenoh-router > /tmp/vision_zenoh.log 2>&1
   ```

---

## ✅ Чеклист применения

### Базовое исправление (2025-11-09)
- [x] Обновлены конфиги на Vision Pi (очереди 2→4, таймаут 5→20с)
- [x] Обновлены конфиги на Main Pi (очереди 2→4, таймаут 5→20с)
- [x] Перезапущен zenoh-router на Vision Pi
- [x] Перезапущен zenoh-router на Main Pi
- [x] Проверены логи на отсутствие ошибок
- [x] Мониторинг показал - ошибки продолжаются под нагрузкой

### Максимальное исправление (2025-11-10) ⭐ ТЕКУЩЕЕ
- [ ] Pull изменений на Vision Pi (`git pull`)
- [ ] Pull изменений на Main Pi (`git pull`)
- [ ] Перезапущен zenoh-router на Vision Pi
- [ ] Перезапущен zenoh-router на Main Pi
- [ ] Проверены логи - нет "Unable to push" ошибок (10 минут)
- [ ] Проверены логи - нет "Cannot find link" ошибок (10 минут)
- [ ] Проверены логи - нет "Failed to terminate tasks" ошибок (10 минут)
- [ ] Проверена работа ROS 2 топиков (camera, lidar, scan, tf)
- [ ] Нагрузочный тест - все сенсоры одновременно (30 минут)
- [ ] Мониторинг в течение 1 часа под полной нагрузкой
- [ ] Мониторинг в течение 24 часов
- [ ] Если успешно - задокументировать результаты
- [ ] Если неуспешно - собрать данные для анализа сетевой инфраструктуры

### Команды для применения

```bash
# Vision Pi - полный процесс
sshpass -p 'open' ssh ros2@10.1.1.21 << 'EOF'
cd ~/rob_box_project
git pull
cd docker/vision
docker compose restart zenoh-router
sleep 5
docker logs zenoh-router --tail 50
EOF

# Main Pi - полный процесс
sshpass -p 'open' ssh ros2@10.1.1.20 << 'EOF'
cd ~/rob_box_project
git pull
cd docker/main
docker compose restart zenoh-router
sleep 5
docker logs zenoh-router --tail 50
EOF
```
