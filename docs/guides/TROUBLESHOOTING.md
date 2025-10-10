# Диагностика и решение проблем

## Проблема: RTAB-Map не получает данные от камеры

### Симптомы
```
[WARN] rgbd_sync: Did not receive data since 5 seconds!
```

RTAB-Map запущен и ждёт данные, но камера на другой Pi не публикует топики или они не доходят.

---

## Пошаговая диагностика

### Шаг 1: Проверка камеры на Pi #1 (Vision)

```bash
# На Vision Pi проверьте статус контейнера
docker ps

# Проверьте логи камеры
docker logs oak-d

# Если есть ошибки - перезапустите
cd ~/rob_box_project/docker/vision
./update_and_restart.sh
```

### Шаг 2: Проверка публикации топиков на Pi #1

```bash
# Список всех топиков
ros2 topic list

# Должны быть:
# /oak/rgb/image_raw/compressed
# /oak/rgb/camera_info
# /oak/stereo/image_raw/compressedDepth

# Проверьте частоту публикации (должно быть ~5 Hz)
ros2 topic hz /oak/rgb/image_raw/compressed
ros2 topic hz /oak/stereo/image_raw/compressedDepth

# Проверьте размер сообщений
ros2 topic bw /oak/rgb/image_raw/compressed
```

### Шаг 3: Проверка сетевого обнаружения (DDS Discovery)

```bash
# На ОБЕИХ Pi проверьте переменную ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Должно быть одинаковое значение (0)

# Проверьте, видят ли Pi друг друга
# На Pi #1:
ros2 node list

# На Pi #2:
ros2 node list

# Обе должны видеть ноды с обеих машин
```

### Шаг 4: Проверка топиков на Pi #2 (Main)

```bash
# На Main Pi проверьте видимость топиков камеры
ros2 topic list | grep oak

# Должны быть видны:
# /oak/rgb/image_raw/compressed
# /oak/rgb/camera_info
# /oak/stereo/image_raw/compressedDepth

# Проверьте, приходят ли данные
ros2 topic hz /oak/rgb/image_raw/compressed
```

### Шаг 5: Проверка синхронизации времени

```bash
# На ОБЕИХ Pi проверьте время
date

# Разница должна быть < 1 секунды
# Если разница большая, синхронизируйте:
sudo apt install ntpdate -y
sudo ntpdate pool.ntp.org

# Или настройте NTP
sudo systemctl enable systemd-timesyncd
sudo systemctl start systemd-timesyncd
timedatectl set-ntp true
```

### Шаг 6: Проверка сети между Pi

```bash
# На Pi #1 узнайте IP адрес
hostname -I

# На Pi #2 пингуйте Pi #1
ping <IP_адрес_Pi1>

# Проверьте пропускную способность (опционально)
# На Pi #2:
iperf3 -s

# На Pi #1:
iperf3 -c <IP_адрес_Pi2>
```

---

## Частые проблемы и решения

### 1. Камера не запускается на Pi #1

**Симптом:** `docker logs oak-d` показывает ошибки CycloneDDS

**Решение:**
```bash
cd ~/rob_box_project
git pull origin main
cd docker/vision
docker-compose down --remove-orphans
docker-compose up -d
```

### 2. Разные ROS_DOMAIN_ID на Pi

**Симптом:** Ноды не видят друг друга

**Решение:**
```bash
# Проверьте в docker-compose.yaml обеих Pi
# Должно быть одинаковое значение:
environment:
  - ROS_DOMAIN_ID=0
```

### 3. Firewall блокирует DDS трафик

**Симптом:** Топики не проходят между Pi

**Решение:**
```bash
# Откройте UDP порты для DDS (7400-7500)
sudo ufw allow 7400:7500/udp

# Или временно отключите firewall для теста
sudo ufw disable

# После теста включите обратно
sudo ufw enable
```

### 4. Топики публикуются, но RTAB-Map не подписывается

**Симптом:** `ros2 topic list` показывает топики, но rgbd_sync не получает данные

**Проблема:** RTAB-Map ждёт **несжатые** топики (`image_raw_relay`), а камера публикует **сжатые** (`compressed`)

**Решение:** Используется нода `republish`, которая конвертирует compressed → raw. Проверьте её логи:
```bash
docker logs rtabmap | grep republish
```

Если `republish` упал, проблема в конфигурации docker-compose.yaml.

### 5. Несинхронизированное время

**Симптом:** RTAB-Map сообщает об ошибках синхронизации

**Решение:**
```bash
# На обеих Pi
sudo timedatectl set-ntp true
sudo systemctl restart systemd-timesyncd
```

---

## Команды мониторинга

### Общий статус системы

```bash
# Статус контейнеров
docker ps

# Использование CPU/RAM
htop

# Температура
vcgencmd measure_temp

# Сетевая нагрузка
iftop -i wlan0  # или eth0

# Топики и их частота
ros2 topic list
ros2 topic hz /oak/rgb/image_raw/compressed
```

### Мониторинг DDS трафика

```bash
# Статистика сети
netstat -s | grep -i "udp"

# Открытые порты
ss -tuln | grep 74

# Пропускная способность топиков
ros2 topic bw /oak/rgb/image_raw/compressed
ros2 topic bw /oak/stereo/image_raw/compressedDepth
```

### Проверка TF дерева

```bash
# Установите (если нет)
sudo apt install ros-humble-tf2-tools

# Просмотр дерева трансформаций
ros2 run tf2_tools view_frames

# Эхо конкретной трансформации
ros2 run tf2_ros tf2_echo base_link camera_link
```

---

## Ожидаемое поведение после запуска

### Pi #1 (Vision - OAK-D)

```bash
docker logs oak-d
# Должно быть:
# ✅ [INFO] Camera ready
# ✅ Publishing to /oak/rgb/image_raw/compressed
# ✅ Publishing to /oak/stereo/image_raw/compressedDepth
# ❌ НЕ должно быть ошибок CycloneDDS
```

### Pi #2 (Main - RTAB-Map)

```bash
docker logs rtabmap | tail -50
# Первые ~30 секунд:
# ⚠️ [WARN] Did not receive data since 5 seconds

# После подключения к камере:
# ✅ [INFO] rgbd_sync: receiving data
# ✅ [INFO] Odometry update rate: ~5Hz
# ✅ [INFO] RTAB-Map update rate: ~1Hz
```

### Сетевой трафик

```bash
# На Pi #1 (Vision):
ros2 topic bw /oak/rgb/image_raw/compressed
# Ожидается: ~8-15 Mbps (JPEG 720p @ 5fps)

# На Pi #2 (Main):
ros2 topic bw /oak/rgb/image_raw/compressed
# Должно совпадать с Pi #1
```

---

## Быстрая перезагрузка при проблемах

### На Pi #1 (Vision):
```bash
cd ~/rob_box_project/docker/vision
./update_and_restart.sh
```

### На Pi #2 (Main):
```bash
cd ~/rob_box_project/docker/main
./update_and_restart.sh
```

### Полная перезагрузка обеих систем:
```bash
# Сначала остановите на обеих
cd ~/rob_box_project/docker/vision  # или /main
docker-compose down

# Затем запустите сначала Vision, потом Main
# Pi #1:
cd ~/rob_box_project/docker/vision
docker-compose up -d

# Подождите 10 секунд, затем Pi #2:
cd ~/rob_box_project/docker/main
docker-compose up -d
```

---

## Получение помощи

При обращении за помощью предоставьте:

1. **Логи камеры:**
   ```bash
   docker logs oak-d > oak-d.log
   ```

2. **Логи RTAB-Map:**
   ```bash
   docker logs rtabmap > rtabmap.log
   ```

3. **Список топиков на обеих Pi:**
   ```bash
   ros2 topic list > topics.txt
   ```

4. **Системную информацию:**
   ```bash
   uname -a > sysinfo.txt
   ros2 doctor > ros2_doctor.txt
   ```

5. **Сетевую конфигурацию:**
   ```bash
   ifconfig > network.txt
   ```
