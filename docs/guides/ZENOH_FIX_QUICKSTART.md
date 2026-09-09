# Быстрая инструкция: Применение максимального исправления Zenoh

## 🎯 Что это исправляет

Ошибки в логах Zenoh роутера:
- ❌ `ERROR: Unable to push non droppable network message. Closing transport!`
- ❌ `ERROR: Cannot find link 1`
- ❌ `ERROR: Failed to terminate 1 tasks`
- ❌ `WARN: Query not found!`

## 📋 Что было изменено

### 1. Zenoh буферы увеличены до МАКСИМУМА
- control: 8 → **16 batches** (1024 KB)
- real_time: 8 → **16 batches** (1024 KB)
- data_high: 6 → **12 batches** (768 KB)
- Таймаут: 30с → **60с**

### 2. Компрессия изображений камеры ВКЛЮЧЕНА
- RGB: сжатие JPEG (экономия ~90%)
- Depth: сжатие PNG (экономия ~80%)
- **Трафик камеры: 18 MB/s → 3 MB/s**

### 3. Синхронизация роутеров
- peers_failover_brokering включён на обоих Pi

## 🚀 Как применить

### Автоматическое применение (РЕКОМЕНДУЕТСЯ)

Запустите эти команды **с локальной машины** (не на Pi):

```bash
# Vision Pi - обновление и перезапуск
sshpass -p 'open' ssh ros2@10.1.1.21 << 'EOF'
cd ~/rob_box_project
git pull
cd docker/vision
docker compose restart zenoh-router
docker compose restart oak-d
docker compose restart apriltag
sleep 10
echo "=== Vision Pi Status ==="
docker ps | grep -E "(zenoh|oak|apriltag)"
docker logs zenoh-router --tail 20
EOF

# Main Pi - обновление и перезапуск
sshpass -p 'open' ssh ros2@10.1.1.20 << 'EOF'
cd ~/rob_box_project
git pull
cd docker/main
docker compose restart zenoh-router
sleep 10
echo "=== Main Pi Status ==="
docker ps | grep zenoh
docker logs zenoh-router --tail 20
EOF
```

### Пошаговое применение

**Шаг 1: Vision Pi**
```bash
# SSH подключение
sshpass -p 'open' ssh ros2@10.1.1.21

# Обновление кода
cd ~/rob_box_project
git pull

# Перезапуск контейнеров
cd docker/vision
docker compose restart zenoh-router
docker compose restart oak-d
docker compose restart apriltag

# Проверка
docker logs zenoh-router --tail 30
docker logs oak-d --tail 30

# Выход
exit
```

**Шаг 2: Main Pi**
```bash
# SSH подключение
sshpass -p 'open' ssh ros2@10.1.1.20

# Обновление кода
cd ~/rob_box_project
git pull

# Перезапуск контейнеров
cd docker/main
docker compose restart zenoh-router

# Проверка
docker logs zenoh-router --tail 30

# Выход
exit
```

## ✅ Проверка результатов

### 1. Проверить отсутствие ошибок в логах (10 минут)

```bash
# Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21 \
  'docker logs zenoh-router --since 10m 2>&1 | grep -E "(ERROR|Unable to push|Cannot find link)"'

# Main Pi
sshpass -p 'open' ssh ros2@10.1.1.20 \
  'docker logs zenoh-router --since 10m 2>&1 | grep -E "(ERROR|Unable to push|Cannot find link)"'

# Ожидается: пустой вывод (нет ошибок)
```

### 2. Проверить компрессию изображений

```bash
# Подключиться к Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21

# Проверить что камера публикует сжатые топики
ros2 topic list | grep compressed

# Должны видеть:
# /camera/camera/color/image/compressed
# /camera/camera/depth/image/compressed

# Проверить частоту
ros2 topic hz /camera/camera/color/image/compressed

# Должно быть: average rate: ~5.0

exit
```

### 3. Мониторинг использования сети

```bash
# На любом Pi, смотреть трафик
sshpass -p 'open' ssh ros2@10.1.1.21 'iftop -i eth0 -t -s 10'

# Должно быть: ~5-10 MB/s (вместо ~30-60 MB/s)
```

### 4. Проверить работу AprilTag

```bash
# Подключиться к Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21

# Проверить что AprilTag работает
ros2 topic echo /tf --once | grep april

# Должны видеть TF трансформации от AprilTag если тег виден

exit
```

## 📊 Ожидаемые результаты

После применения исправления в течение **1 часа** должно быть:

- ✅ **0 ошибок** "Unable to push non droppable network message"
- ✅ **0 ошибок** "Cannot find link"
- ✅ **0 ошибок** "Failed to terminate tasks"
- ✅ **Трафик камеры снижен** с ~18 MB/s до ~3 MB/s
- ✅ **Топики камеры** публикуются стабильно на 5 FPS
- ✅ **AprilTag** работает с compressed изображениями
- ✅ **Использование памяти** увеличено на ~2 MB на роутер (приемлемо)

## ⚠️ Если проблемы сохраняются

Если после 1 часа работы ошибки продолжают появляться:

### 1. Собрать диагностику

```bash
# Логи роутеров
sshpass -p 'open' ssh ros2@10.1.1.21 'docker logs zenoh-router > /tmp/vision_zenoh.log 2>&1'
sshpass -p 'open' ssh ros2@10.1.1.20 'docker logs zenoh-router > /tmp/main_zenoh.log 2>&1'

# Скопировать логи на локальную машину
sshpass -p 'open' scp ros2@10.1.1.21:/tmp/vision_zenoh.log ./
sshpass -p 'open' scp ros2@10.1.1.20:/tmp/main_zenoh.log ./

# Проверить сеть
sshpass -p 'open' ssh ros2@10.1.1.21 'ethtool eth0'
sshpass -p 'open' ssh ros2@10.1.1.21 'ping 10.1.1.10 -c 100 -i 0.01'
sshpass -p 'open' ssh ros2@10.1.1.21 'ip -s link show eth0'
```

### 2. Создать issue в GitHub

Создайте issue с:
- Логами обоих роутеров
- Выводом команд диагностики сети
- Описанием как часто происходят ошибки
- Версиями: `dpkg -l | grep zenoh`

### 3. Рассмотреть дополнительные меры

См. раздел "Если проблема сохранится" в [ZENOH_FIX_2025-11-10_MAXIMUM.md](../fixes/ZENOH_FIX_2025-11-10_MAXIMUM.md):
- Проверка сетевой инфраструктуры (кабель, коммутатор)
- Дополнительная оптимизация данных (downsampling LiDAR)
- Выделенный Ethernet для SLAM данных
- Обновление версии Zenoh

## 📚 Дополнительная информация

- **Полная документация:** [ZENOH_FIX_2025-11-10_MAXIMUM.md](../fixes/ZENOH_FIX_2025-11-10_MAXIMUM.md)
- **Краткий справочник:** [ZENOH_TRANSPORT_FIX_QUICKREF.md](../fixes/ZENOH_TRANSPORT_FIX_QUICKREF.md)
- **Технический анализ:** [ZENOH_TRANSPORT_ERROR_ANALYSIS_2025-11-09.md](../reports/ZENOH_TRANSPORT_ERROR_ANALYSIS_2025-11-09.md)

---

**Дата:** 2025-11-10  
**Версия:** 1.0 (Максимальное исправление + Компрессия)
