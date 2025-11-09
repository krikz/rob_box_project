# Quick Reference: Zenoh Transport Error Fix

**Дата:** 2025-11-09  
**Проблема:** `ERROR: Unable to push non droppable network message. Closing transport!`  
**Статус:** ✅ Исправлено

---

## 🎯 Что было сделано

### Изменения в конфигурации

1. **Увеличен размер TX очередей** с 2 до 4 batch (удвоение буфера)
2. **Увеличен таймаут wait_before_close** с 5 до 20 секунд

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

### До исправления

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

### После исправления

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

### Эффект

| Параметр | Было | Стало | Улучшение |
|----------|------|-------|-----------|
| **Размер буфера** | 128 KB | 256 KB | +100% |
| **Таймаут** | 5 сек | 20 сек | +300% |
| **Использование RAM** | 1 MB | 2 MB | +1 MB |
| **Устойчивость к пикам** | Низкая | Высокая | ✅ |

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

Если после применения исправлений ошибки продолжают появляться:

1. **Увеличьте размер очередей до 8 batch:**
   ```json5
   size: { control: 8, real_time: 8, /* ... */ }
   ```

2. **Увеличьте таймаут до 30 секунд:**
   ```json5
   wait_before_close: 30000000,
   ```

3. **Проверьте сетевую инфраструктуру:**
   ```bash
   # Скорость Ethernet
   ethtool eth0
   
   # Проверка связи
   ping 10.1.1.10 -c 100
   ```

4. **Проверьте нагрузку на процессор:**
   ```bash
   htop
   ```

5. **Создайте issue** в репозитории с логами

---

## ✅ Чеклист применения

- [ ] Обновлены конфиги на Vision Pi
- [ ] Обновлены конфиги на Main Pi
- [ ] Перезапущен zenoh-router на Vision Pi
- [ ] Перезапущен zenoh-router на Main Pi
- [ ] Проверены логи на отсутствие ошибок
- [ ] Проверена работа ROS 2 топиков
- [ ] Мониторинг в течение 24 часов
