# ✅ Чеклист запуска и проверки системы

## Перед запуском

### Raspberry Pi #1 (с OAK-D камерой)

- [ ] Камера OAK-D подключена к USB 3.0 порту
- [ ] Проверено: `lsusb | grep Movidius` показывает камеру
- [ ] Файлы конфигурации на месте:
  - [ ] `docker/vision/config/oak_d_config.yaml`
  - [ ] `docker/vision/config/cyclonedds.xml`
  - [ ] `docker/vision/config/start_oak_d.sh`
- [ ] IP адрес Pi известен (для проверки связи)
- [ ] Достаточно свободного места: `df -h` (минимум 2GB)

### Raspberry Pi #2 (с RTAB-Map)

- [ ] Файлы конфигурации на месте:
  - [ ] `docker/main/config/rtabmap_config.yaml`
  - [ ] `docker/main/config/cyclonedds.xml`
  - [ ] `docker/main/config/rtabmap.yaml` (симлинк на rtabmap_config.yaml)
- [ ] Папка для карт создана: `mkdir -p docker/main/maps`
- [ ] Достаточно свободного места: `df -h` (минимум 5GB для карт)
- [ ] Проверена связь с Pi #1: `ping <IP_Pi1>` (< 5ms)

### На обоих Raspberry Pi

- [ ] `ROS_DOMAIN_ID=0` в docker-compose.yaml
- [ ] Docker и docker-compose установлены
- [ ] Достаточно RAM (минимум 2GB, рекомендуется 4GB+)

---

## Запуск системы

### 1️⃣ Raspberry Pi #1 - Запуск камеры

```bash
cd ~/docker/vision  # или путь к вашему проекту

# Остановить старые контейнеры
docker-compose down

# Пересобрать образ (если изменили Dockerfile)
docker-compose build

# Запустить
docker-compose up -d

# Проверить статус
docker ps | grep oak-d
```

**Ожидаемый результат:**
- [ ] Контейнер `oak-d` запущен и в статусе `Up`
- [ ] Нет ошибок в логах (следующий шаг)

### 2️⃣ Проверка логов камеры

```bash
docker logs -f oak-d
```

**Что должно быть в логах:**
- [ ] "Camera created successfully"
- [ ] "Publishing on topics: /oak/rgb/image_raw/compressed"
- [ ] "Publishing on topics: /oak/stereo/image_raw/compressedDepth"
- [ ] НЕТ ошибок о USB или libusb
- [ ] НЕТ ошибок о нехватке памяти

**Если есть ошибки:**
- Проверить USB подключение
- Проверить права доступа к USB: `ls -l /dev/bus/usb`
- Перезапустить контейнер: `docker-compose restart oak-d`

### 3️⃣ Проверка топиков камеры (на Pi #1)

```bash
# Список топиков
ros2 topic list | grep oak

# Частота публикации RGB
ros2 topic hz /oak/rgb/image_raw/compressed

# Частота публикации depth
ros2 topic hz /oak/stereo/image_raw/compressedDepth

# Размер сообщений RGB
ros2 topic bw /oak/rgb/image_raw/compressed
```

**Ожидаемые значения:**
- [ ] Топики публикуются с частотой **~5 Hz**
- [ ] Bandwidth RGB: **1-3 MB/s** (было бы 10-15 MB/s без сжатия)
- [ ] Bandwidth depth: **0.5-1.5 MB/s**

### 4️⃣ Raspberry Pi #2 - Запуск RTAB-Map

```bash
cd ~/docker/main  # или путь к вашему проекту

# Остановить старые контейнеры
docker-compose down

# Запустить
docker-compose up -d

# Проверить статус
docker ps | grep rtabmap
```

**Ожидаемый результат:**
- [ ] Контейнер `rtabmap` запущен и в статусе `Up`

### 5️⃣ Проверка логов RTAB-Map

```bash
docker logs -f rtabmap
```

**Что должно быть в логах:**
- [ ] "RTAB-Map started"
- [ ] "Subscribed to /oak/rgb/image_raw"
- [ ] "Subscribed to /oak/stereo/image_raw"
- [ ] "Using GFTT/BRIEF feature detector"
- [ ] НЕТ ошибок "Waiting for images..."
- [ ] НЕТ ошибок о transform

**Если "Waiting for images...":**
- Проверить топики на Pi #2: `ros2 topic list | grep oak`
- Если топиков нет - проблема с CycloneDDS/сетью
- Проверить ping между Pi
- Перезапустить ROS daemon: `ros2 daemon stop && ros2 daemon start`

### 6️⃣ Проверка топиков на Pi #2

```bash
# На Raspberry Pi #2
ros2 topic list | grep oak
```

**Должны видеть:**
- [ ] `/oak/rgb/image_raw/compressed`
- [ ] `/oak/rgb/camera_info`
- [ ] `/oak/stereo/image_raw/compressedDepth`
- [ ] `/oak/stereo/camera_info`

**Если топиков нет:**
- Проблема с CycloneDDS или сетью
- См. секцию "Troubleshooting" ниже

---

## Проверка производительности

### CPU нагрузка

```bash
# На Pi #1
htop
# Найти процесс depthai или python
# Ожидаемая нагрузка: 30-45%
```

- [ ] CPU Pi #1: **< 50%** ✅

```bash
# На Pi #2
htop
# Найти процесс rtabmap
# Ожидаемая нагрузка: 40-60%
```

- [ ] CPU Pi #2: **< 65%** ✅

### Память

```bash
free -h
```

**Ожидаемые значения:**
- [ ] Pi #1 used memory: **< 1 GB** ✅
- [ ] Pi #2 used memory: **< 1.5 GB** ✅

### Сетевой трафик

```bash
# Установить iftop если нет
sudo apt install iftop

# Запустить
sudo iftop -i eth0  # или wlan0 для WiFi
```

**Ожидаемые значения:**
- [ ] Total bandwidth: **8-15 Mbps** ✅
- [ ] Без больших всплесков

### Температура (важно для Pi!)

```bash
vcgencmd measure_temp
```

- [ ] Температура **< 70°C** ✅ (в нагрузке)
- [ ] Температура **< 60°C** идеально
- Если > 75°C - добавить охлаждение!

---

## Функциональные проверки

### RTAB-Map получает данные

```bash
# На Pi #2
ros2 topic echo /rtabmap/info --once
```

- [ ] Видны данные о loop closures, locations, features
- [ ] `local_map_size` увеличивается при движении робота

### Visual Odometry работает

```bash
ros2 topic echo /rtabmap/odom --once
```

- [ ] Данные одометрии публикуются
- [ ] Position меняется при движении камеры

### TF дерево корректно

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

- [ ] Transform публикуется без ошибок
- [ ] Нет warning о missing transforms

---

## Troubleshooting

### ❌ Топики не видны на Pi #2

**Проверка:**

```bash
# На обоих Pi
echo $ROS_DOMAIN_ID
# Должно быть 0 на обоих!

echo $CYCLONEDDS_URI
# Должно указывать на правильный файл

# Перезапустить ROS daemon
ros2 daemon stop
ros2 daemon start

# Проверить топики снова
ros2 topic list
```

**Если не помогло:**

```bash
# Проверить multicast
ping -c 3 239.255.0.1
# Если не работает - проблема с сетью

# Проверить firewall
sudo ufw status
# Порты 7400-7600 должны быть открыты для UDP
```

### ❌ Высокая нагрузка CPU

**Дополнительная оптимизация:**

1. Снизить FPS до 3:
   ```yaml
   # В oak_d_config.yaml
   i_fps: 3.0
   ```

2. Уменьшить features RTAB-Map:
   ```yaml
   # В rtabmap_config.yaml
   Vis/MaxFeatures: "300"
   Kp/MaxFeatures: "150"
   ```

3. Отключить grid mapping:
   ```yaml
   Grid/FromDepth: "false"
   ```

### ❌ Задержка > 1 секунда

**Проверка сети:**

```bash
ping -c 10 <IP_другого_Pi>
# Средняя задержка должна быть < 5ms
```

**Если задержка большая:**
- Использовать проводное подключение (не WiFi)
- Проверить качество сетевого кабеля
- Убедиться что оба Pi в одной подсети

### ❌ "Waiting for images" в RTAB-Map

**Решение:**

```bash
# На Pi #2
# Проверить что compressed image transport установлен
docker exec -it rtabmap bash
ros2 pkg list | grep image_transport

# Должно быть:
# - image_transport
# - compressed_image_transport
# - compressed_depth_image_transport
```

**Если пакетов нет:**
- Пересобрать контейнер RTAB-Map
- Проверить Dockerfile на установку плагинов

---

## Финальная проверка ✅

Система работает корректно если:

- [x] Оба контейнера запущены и стабильны
- [x] Топики камеры видны на обоих Pi
- [x] FPS стабильные ~5 Hz
- [x] CPU < 50% на Pi #1, < 65% на Pi #2
- [x] Bandwidth < 20 Mbps
- [x] RTAB-Map строит карту (local_map_size растет)
- [x] Нет ошибок в логах
- [x] Температура < 70°C

## 📊 Метрики для мониторинга

Создать простой скрипт для периодической проверки:

```bash
#!/bin/bash
# monitor.sh

echo "=== CPU Usage ==="
top -bn1 | grep "Cpu(s)" | awk '{print "CPU: " $2}'

echo "=== Memory ==="
free -h | grep Mem | awk '{print "Used: " $3 " / " $2}'

echo "=== Temperature ==="
vcgencmd measure_temp

echo "=== ROS Topics ==="
ros2 topic hz /oak/rgb/image_raw/compressed --once --timeout 2

echo "=== Bandwidth ==="
ros2 topic bw /oak/rgb/image_raw/compressed --once --timeout 2
```

**Использование:**

```bash
chmod +x monitor.sh
watch -n 5 ./monitor.sh
```

---

**Следующие шаги:**
- [ ] Протестировать систему в движении
- [ ] Проверить качество построенной карты
- [ ] Настроить параметры навигации
- [ ] Добавить мониторинг в систему

**Документация:**
- Полная информация: `OPTIMIZATION_README.md`
- Быстрый старт: `QUICK_START_RU.md`
- Сводка изменений: `SUMMARY.md`
