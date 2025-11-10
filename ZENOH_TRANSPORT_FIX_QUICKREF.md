# Quick Reference: Zenoh Transport Error Fix

**Дата:** 2025-11-09  
**Обновлено:** 2025-11-10  
**Проблема:** `ERROR: Unable to push non droppable network message. Closing transport!` + `ERROR: Cannot find link 1`  
**Статус:** ✅ Усиленное исправление применено (2025-11-10)

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

### Усиленное исправление (2025-11-10) ⭐ ТЕКУЩЕЕ

```json5
queue: {
  size: {
    control: 8,           // 8 × 64KB = 512KB ⬆️⬆️
    real_time: 8,         // 8 × 64KB = 512KB ⬆️⬆️
    interactive_high: 4,  // 4 × 64KB = 256KB
    interactive_low: 4,   // 4 × 64KB = 256KB
    data_high: 6,         // 6 × 64KB = 384KB ⬆️
    data: 4,              // 4 × 64KB = 256KB
    data_low: 2,          // 2 × 64KB = 128KB ⬇️
    background: 2,        // 2 × 64KB = 128KB ⬇️
  },
  congestion_control: {
    block: {
      wait_before_close: 30000000,  // 30 секунд ⬆️
    },
  },
}
```

### Сравнительная таблица

| Параметр | Было | Базовое | Усиленное | Улучшение |
|----------|------|---------|-----------|-----------|
| **Буфер control** | 128 KB | 256 KB | **512 KB** | +300% |
| **Буфер real_time** | 128 KB | 256 KB | **512 KB** | +300% |
| **Буфер data_high** | 128 KB | 256 KB | **384 KB** | +200% |
| **Таймаут** | 5 сек | 20 сек | **30 сек** | +500% |
| **Использование RAM** | 1 MB | 2 MB | **3 MB** | +2 MB |
| **Устойчивость к пикам** | Низкая | Средняя | **Высокая** | ✅✅✅ |

---

## 🔗 Дополнительная информация

**Полный анализ:**  
📄 [ZENOH_TRANSPORT_ERROR_ANALYSIS_2025-11-09.md](docs/reports/ZENOH_TRANSPORT_ERROR_ANALYSIS_2025-11-09.md)

**Связанные документы:**
- [SYSTEM_OVERVIEW.md](docs/architecture/SYSTEM_OVERVIEW.md)
- [SOFTWARE.md](docs/architecture/SOFTWARE.md)
- [DOCKER_STANDARDS.md](docs/development/DOCKER_STANDARDS.md)

---

## ❓ Если проблема сохраняется

Если после применения усиленного исправления ошибки продолжают появляться:

1. **Дальнейшее увеличение критичных очередей до максимума (16 batch):**
   ```json5
   size: { 
     control: 16,      // Максимально допустимое значение
     real_time: 16,    // Максимально допустимое значение
     data_high: 12,
     // остальное без изменений
   }
   ```

2. **Увеличить таймаут до 60 секунд:**
   ```json5
   wait_before_close: 60000000,  // 60 секунд
   ```

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

### Усиленное исправление (2025-11-10) ⭐
- [ ] Pull изменений на Vision Pi (`git pull`)
- [ ] Pull изменений на Main Pi (`git pull`)
- [ ] Перезапущен zenoh-router на Vision Pi
- [ ] Перезапущен zenoh-router на Main Pi
- [ ] Проверены логи - нет "Unable to push" ошибок
- [ ] Проверены логи - нет "Cannot find link" ошибок
- [ ] Проверена работа ROS 2 топиков (camera, lidar, scan)
- [ ] Мониторинг в течение 24 часов под нагрузкой
- [ ] Тест высоких нагрузок (одновременная работа всех сенсоров)

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
