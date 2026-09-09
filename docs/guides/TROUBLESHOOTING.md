# Диагностика и решение проблем

> **Актуально для**: ROS 2 Humble + Zenoh DDS + Docker Compose (май 2026)

## Проблема: RTAB-Map не строит карту (нет данных от LiDAR)

### Симптомы
```
[WARN] rtabmap: Did not receive scan data since 5 seconds!
```
Или `/map` топик не обновляется, карта не растёт.

### Диагностика

```bash
# 1. Проверить что lslidar контейнер запущен
docker ps | grep lslidar

# 2. Проверить публикацию /scan
ros2 topic hz /scan  # должен быть ~10 Hz

# 3. Если /scan пустой — проверить логи lslidar
docker logs lslidar --tail 50

# 4. Убедиться что LiDAR подключён физически (USB)
ls /dev/ttyACM*

# 5. Перезапустить lslidar
cd ~/rob_box_project/docker/main
docker compose restart lslidar
```

---

## Проблема: Zenoh — Vision Pi не видит топики Main Pi

### Симптомы
`ros2 topic list` на Vision Pi не показывает топики Main Pi (или наоборот).

### Диагностика

```bash
# 1. Проверить что zenoh-router запущен на Main Pi
docker ps | grep zenoh-router

# 2. Проверить логи Zenoh роутера Vision Pi
docker logs zenoh-router-vision --tail 30

# 3. Убедиться в сетевой связи
ping 10.1.1.10  # с Vision Pi → должен пинговаться Main Pi

# 4. Проверить конфигурацию Zenoh
cat ~/rob_box_project/docker/vision/config/zenoh_session_config.json5

# 5. Перезапустить Zenoh роутеры
docker compose restart zenoh-router         # на Main Pi
docker compose restart zenoh-router-vision  # на Vision Pi
```

**Полная документация**: `docs/fixes/ZENOH_FIX_SUMMARY_2025-11-10.md`

---

## Проблема: VESC не отвечает (timeout)

### Симптомы
`ros2-control` контейнер `unhealthy`, `/cmd_vel` не отрабатывается, робот не едет.

### Диагностика

```bash
# 1. Проверить логи ros2-control
docker logs ros2-control --tail 50
# Искать: "VESC timeout", "connection refused", "serial error"

# 2. Проверить наличие устройства
ls /dev/ttyACM*
# Обычно VESC: /dev/ttyACM0 или /dev/ttyACM1

# 3. Убедиться что порт правильный в конфиге
grep "vesc\|serial" ~/rob_box_project/docker/main/config/ros2_control/vesc_config.yaml

# 4. Перезапустить ros2-control
docker compose restart ros2-control
```

---

## Проблема: Perception контейнер unhealthy

### Симптомы
`docker ps` показывает `perception` unhealthy, health_monitor сообщает об ошибках.

### Диагностика

```bash
# 1. Проверить логи perception
docker logs perception --tail 50

# 2. Часто причина — нет Zenoh-связи с Vision Pi
#    Решить Zenoh-проблему выше, затем перезапустить:
docker compose restart perception
```

---

## Пошаговая диагностика (общая)

### Шаг 1: Проверка статуса контейнеров

```bash
# На Main Pi
cd ~/rob_box_project/docker/main
docker compose ps

# На Vision Pi
cd ~/rob_box_project/docker/vision
docker compose ps
```

Все контейнеры должны быть `running` (или `healthy` если есть healthcheck). Любой `unhealthy` — смотреть `docker logs <name> --tail 50`.

### Шаг 2: Проверка ROS 2 топиков

```bash
ros2 topic list
ros2 topic hz /scan          # LiDAR: ~10 Hz
ros2 topic hz /odom          # Одометрия: ~50 Hz
ros2 topic hz /map           # SLAM карта: обновляется при движении
```

### Шаг 3: Проверка сети между Pi

```bash
ping 10.1.1.11  # с Main Pi → Vision Pi
ping 10.1.1.10  # с Vision Pi → Main Pi
```

---

## Частые проблемы и решения

### 1. Контейнер не запускается (Vision Pi)

**Симптом:** `docker logs oak-d` показывает ошибки подключения

**Решение:**
```bash
cd ~/rob_box_project
git pull origin develop
cd docker/vision
docker compose down --remove-orphans
docker compose up -d
```

### 2. Firewall блокирует Zenoh трафик

**Симптом:** Топики не проходят между Pi даже когда Zenoh роутеры запущены

**Решение:**
```bash
sudo ufw allow 7447/tcp
sudo ufw allow 7447/udp
sudo ufw allow 8000/tcp
```

### 3. RMW_IMPLEMENTATION не установлен

**Симптом:** Ноды не видят друг друга между Pi (при Zenoh discovery не работает если rmw не zenoh)

**Решение:**
```bash
grep "RMW_IMPLEMENTATION" ~/rob_box_project/docker/main/docker-compose.yaml
# Должно быть: RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

### 4. RTAB-Map строит плохую карту

**Симптом:** Карта "плывёт", накапливается drift

**Решение:**
```bash
docker exec -it rtabmap rm -rf /root/.ros/rtabmap.db
docker compose restart rtabmap
```

### 5. Несинхронизированное время

**Симптом:** TF-ошибки: `extrapolation into the future`; в логах zenoh-router:
`Error treating timestamp for received Data ... exceeding delta 500ms is rejected`

**Причина:** часы на Pi разошлись (NTP не синхронизирует). Дефолтный сервер
`ntp.ubuntu.com` из локальной сети 10.1.1.x часто недоступен (UDP 123 таймаутит),
поэтому systemd-timesyncd остаётся в состоянии `System clock synchronized: no`.

**Решение:**

```bash
# 1. Быстрый фикс (включить NTP + прописать рабочие серверы ru.pool.ntp.org)
sudo timedatectl set-ntp true
# Проверить, что NTP-серверы настроены на ru.pool.ntp.org:
grep -E "^(NTP|FallbackNTP)" /etc/systemd/timesyncd.conf
# Если там нет ru.pool.ntp.org — запустить идемпотентный скрипт из репозитория:
sudo bash ~/rob_box_project/scripts/maintenance/sync_time.sh

# 2. Либо вручную:
sudo tee /etc/systemd/timesyncd.conf > /dev/null << 'EOF'
[Time]
NTP=ru.pool.ntp.org 0.ru.pool.ntp.org 1.ru.pool.ntp.org
FallbackNTP=pool.ntp.org ntp.ubuntu.com
RootDistanceMaxSec=5
PollIntervalMinSec=32
PollIntervalMaxSec=2048
EOF
sudo systemctl restart systemd-timesyncd

# 3. Убедиться, что синхронизация прошла:
timedatectl  # должно быть: System clock synchronized: yes
```

> ⚠️ `ntp.ubuntu.com` может быть недоступен из локальной сети — это нормально,
> если в `FallbackNTP`/`NTP` есть рабочие серверы (ru.pool.ntp.org проверен).
> Деплой-воркфлоу автоматически запускает `sync_time.sh` на обоих Pi перед
> стартом контейнеров.

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

### Main Pi

```bash
docker logs lslidar | tail -20
# ✅ Publishing /scan at ~10 Hz

docker logs ros2-control | tail -20
# ✅ Robot hardware interface active

docker logs rtabmap | tail -20
# ✅ RTAB-Map mapping active, receiving /scan
```

### Vision Pi

```bash
docker logs oak-d | tail -20
# ✅ Camera ready, publishing /oak/rgb/image_raw

docker logs voice | tail -20
# ✅ STT/TTS initialized

docker logs led-matrix | tail -20
# ✅ LED matrix initialized (381 LEDs)
```

### Сетевой трафик

```bash
# Zenoh discovery:
ros2 topic list  # должен показывать топики с обеих Pi

# Ключевые топики:
ros2 topic hz /scan            # ~10 Hz (LiDAR)
ros2 topic hz /odom            # ~50 Hz
ros2 topic hz /map             # ~0.2-1 Hz (при движении)
```

---

## Быстрая перезагрузка при проблемах

### Vision Pi:
```bash
cd ~/rob_box_project/docker/vision
docker compose down --remove-orphans
docker compose up -d
```

### Main Pi:
```bash
cd ~/rob_box_project/docker/main
docker compose down --remove-orphans
docker compose up -d
```

### Полная перезагрузка (сначала Vision, потом Main):
```bash
# Сначала Vision Pi:
cd ~/rob_box_project/docker/vision
docker compose up -d

# Подождите ~10 секунд, затем Main Pi:
cd ~/rob_box_project/docker/main
docker compose up -d
```

---

## Получение помощи

При обращении за помощью предоставьте:

1. **Статус контейнеров:**
   ```bash
   docker compose ps > containers.txt
   ```

2. **Логи проблемного контейнера:**
   ```bash
   docker logs <container-name> --tail 100 > container.log
   ```

3. **Список топиков:**
   ```bash
   ros2 topic list > topics.txt
   ```

4. **Системная информация:**
   ```bash
   uname -a
   ros2 doctor
   ```

---

## Ложные срабатывания и безопасные предупреждения (False Positives)

Некоторые сообщения в логах выглядят как ошибки, но на самом деле являются **нормальным поведением** системы. Эти сообщения можно игнорировать:

### ✅ CAN Interface "ERROR-ACTIVE" State

**Лог сообщение:**
```
CAN controller state: ERROR-ACTIVE
```

**Объяснение:** ERROR-ACTIVE - это **нормальное рабочее состояние** CAN контроллера. В этом состоянии устройство может передавать и принимать сообщения без ограничений. Проблемой является только состояние **BUS-OFF** (полная потеря связи) или **ERROR-PASSIVE** (высокий уровень ошибок).

**Действия:** Игнорировать. Система работает корректно.

---

### ✅ micro-ros-agent Serial Port Error

**Лог сообщение:**
```
ERROR: Cannot open /dev/ttyUSB0
```

**Объяснение:** ESP32 sensor hub в настоящее время **не подключен** к роботу (hardware unavailable). Это ожидаемое поведение для тестовой/dev среды.

**Действия:** Игнорировать, если ESP32 не используется. Для production подключите ESP32 через USB.

---

### ✅ cAdvisor/Promtail Machine ID Missing

**Лог сообщение:**
```
open /etc/machine-id: no such file or directory
```

**Объяснение:** Контейнеры мониторинга (cAdvisor, Promtail) пытаются прочитать machine-id с хоста, но Docker контейнер не всегда имеет этот файл. Это **не влияет** на работу мониторинга - метрики и логи собираются корректно.

**Действия:** Игнорировать. Мониторинг работает без machine-id.

---

### ✅ LSLidar "Error" Messages

**Лог сообщение:**
```
error 0, all angle: xxxxx
error 1, all angle: xxxxx
```

**Объяснение:** Эти сообщения - **информационные логи инициализации** LiDAR сканера, а не настоящие ошибки. Слово "error" в данном случае означает "error_code" (код состояния), а не "error condition" (ошибка).

**Действия:** Игнорировать. LiDAR работает корректно.

---

### ✅ Nav2 "Sensor Out of Bounds"

**Лог сообщение:**
```
Sensor origin at (x, y) is out of map bounds
```

**Объяснение:** Это **ожидаемое поведение** до начала картографирования. Nav2 пытается использовать карту, но RTAB-Map еще не создал её. После начала SLAM-картографирования эти предупреждения исчезнут.

**Действия:** Игнорировать до начала картографирования.

---

### ✅ Zenoh Scouting Delay Warnings

**Лог сообщение:**
```
WARN zenoh::net::routing::hat: Scouting delay elapsed
```

**Объяснение:** Это **нормальные предупреждения при запуске** Zenoh роутера. Zenoh выполняет "scouting" (поиск других узлов в сети), и может занять несколько секунд. После запуска всех сервисов эти предупреждения прекращаются.

**Действия:** Игнорировать при старте системы. Если предупреждения продолжаются > 30 секунд, проверьте сетевое соединение между Pi.

---

### Как отличить настоящую ошибку от ложного срабатывания?

**Настоящая ошибка:**
- ❌ Контейнер падает и перезапускается в цикле
- ❌ Топики ROS 2 не публикуются (`ros2 topic list` пуст)
- ❌ Функциональность не работает (например, камера не даёт изображение)
- ❌ Уровень лога: **ERROR** или **FATAL**

**Ложное срабатывание:**
- ✅ Контейнер работает стабильно
- ✅ Топики публикуются и данные передаются
- ✅ Функциональность работает корректно
- ✅ Уровень лога: **INFO** или **WARN**

**Совет:** Используйте `docker ps` и `ros2 topic list` для проверки реального состояния системы, а не только логи.
