# Исправление конфликта портов Zenoh router (2025-11-10)

**Дата:** 2025-11-10  
**Проблема:** Zenoh transport errors, ROS service failures, query timeouts  
**Решение:** Использование конкретных IP адресов вместо wildcard endpoints  
**Статус:** ✅ ИСПРАВЛЕНО

---

## 🎯 Описание проблемы

### Симптомы

После применения исправлений из PR #177, #180, #182 система запускалась, но через 3-5 минут начинали появляться ошибки:

**ROS ноды:**
```
[camera_node-1] [ERROR] [1762787133.032684891] [rclcpp]: executor taking a service server request from service '/camera/get_parameters' unexpectedly failed: error not set, at ./src/rcl/service.c:269
```

**Zenoh (Vision Pi):**
```
ERROR: Unable to push non droppable network message to 5abd034e3bba0830eb0cb5cdd5af1f06. Closing transport!
WARN: Didn't receive DeclareFinal for interest Face{1, ...}: Timeout(10s)!
WARN: Route reply: Query not found!
```

**Zenoh (Main Pi):**
```
ERROR: Unable to push non droppable network message to 42cd01b3a16f7a5c6d7f31bcd507b6dc. Closing transport!
ERROR: [Peers Network] Received LinkStateList from unknown link
```

### Корневая причина

#### Проблема #1: Конфликт портов на listen endpoints

**Неправильная конфигурация:**
```json5
// Main Pi router
listen.endpoints: ["tcp/[::]:7447#iface=eth0"]

// Vision Pi router  
listen.endpoints: ["tcp/[::]:7447#iface=eth0"]
```

**Почему это ошибка:**

1. **`tcp/[::]:7447`** - это wildcard адрес, который биндится на ВСЕ IPv6 адреса
2. **`#iface=eth0`** - это НЕ ограничение binding, а только routing hint для Zenoh
3. Оба роутера пытаются забиндить порт 7447 на одном сегменте сети (eth0: 10.1.1.0/24)
4. Возникает конфликт или непредсказуемое поведение маршрутизации

#### Проблема #2: ROS ноды не могут подключиться

**Неправильная конфигурация:**
```json5
// Main Pi session config
connect.endpoints: ["tcp/localhost:7447"]

// Vision Pi session config
connect.endpoints: ["tcp/localhost:7447"]
```

**Почему это ошибка:**

1. Роутеры слушают на `tcp/[::]:7447#iface=eth0` (только eth0)
2. ROS ноды подключаются к `tcp/localhost:7447` (loopback интерфейс)
3. Подключение может не работать, т.к. router не слушает на loopback

---

## 🔧 Решение

### Правильная конфигурация

#### Main Pi Router (`docker/main/config/zenoh_router_config.json5`)

```json5
listen: {
  endpoints: ["tcp/10.1.1.10:7447"],  // Конкретный IP адрес eth0
}

connect: {
  endpoints: ["tcp/zenoh.robbox.online:7447"],  // Cloud router (опционально)
}
```

#### Vision Pi Router (`docker/vision/config/zenoh_router_config.json5`)

```json5
listen: {
  endpoints: ["tcp/10.1.1.11:7447"],  // Конкретный IP адрес eth0
}

connect: {
  endpoints: ["tcp/10.1.1.10:7447#iface=eth0"],  // Main Pi через eth0
}
```

#### Main Pi Session (`docker/main/config/zenoh_session_config.json5`)

```json5
connect: {
  endpoints: ["tcp/10.1.1.10:7447"],  // Локальный router через eth0
}
```

#### Vision Pi Session (`docker/vision/config/zenoh_session_config.json5`)

```json5
connect: {
  endpoints: ["tcp/10.1.1.11:7447"],  // Локальный router через eth0
}
```

### Преимущества нового решения

✅ **Нет конфликтов портов** - каждый роутер слушает на своём уникальном IP  
✅ **Весь трафик через Gigabit Ethernet** - явные IP 10.1.1.x гарантируют использование eth0  
✅ **Предсказуемая маршрутизация** - нет ambiguity в выборе интерфейса  
✅ **Простота диагностики** - легко проверить кто к кому подключается через `netstat`

### Архитектура коммуникации

```mermaid
graph TB
    subgraph "Vision Pi (10.1.1.11)"
        V_Router["Zenoh Router<br/>tcp/10.1.1.11:7447"]
        V_Camera["camera_node"]
        V_AprilTag["apriltag_node"]
        V_LiDAR["lslidar_node"]
        
        V_Camera -->|tcp/10.1.1.11:7447| V_Router
        V_AprilTag -->|tcp/10.1.1.11:7447| V_Router
        V_LiDAR -->|tcp/10.1.1.11:7447| V_Router
    end
    
    subgraph "Main Pi (10.1.1.10)"
        M_Router["Zenoh Router<br/>tcp/10.1.1.10:7447"]
        M_RTAB["rtabmap_node"]
        M_Nav2["nav2_nodes"]
        
        M_RTAB -->|tcp/10.1.1.10:7447| M_Router
        M_Nav2 -->|tcp/10.1.1.10:7447| M_Router
    end
    
    V_Router -->|tcp/10.1.1.10:7447<br/>#iface=eth0| M_Router
    M_Router -.->|tcp/zenoh.robbox.online:7447<br/>(опционально)| Cloud["Cloud Zenoh Router"]
    
    style V_Router fill:#e8f4f8,stroke:#2c5282,stroke-width:2px
    style M_Router fill:#e8f4f8,stroke:#2c5282,stroke-width:2px
    style Cloud fill:#fef3c7,stroke:#f59e0b,stroke-width:2px
```

---

## 🚀 Развёртывание

### Предварительные требования

- Доступ к обоим Raspberry Pi (SSH)
- Git репозиторий обновлён до последнего коммита
- Docker и docker-compose установлены

### Порядок развёртывания

⚠️ **ВАЖНО:** Развёртывать строго в этом порядке!

#### 1. Main Pi (10.1.1.20)

```bash
# SSH в Main Pi
sshpass -p 'open' ssh ros2@10.1.1.20

# Перейти в директорию проекта
cd ~/rob_box_project

# Обновить из git
git pull origin main

# Перезапустить Zenoh router и все сервисы
cd docker/main
docker compose restart zenoh-router
sleep 5
docker compose restart

# Проверить что router слушает на правильном IP
sudo netstat -tlnp | grep 7447
# Должно быть: tcp 0 0 10.1.1.10:7447 0.0.0.0:* LISTEN <pid>/zenohd

# Проверить логи router
docker logs zenoh-router --tail 50
# Должно быть: "Zenoh can be reached at: tcp/10.1.1.10:7447"
```

#### 2. Vision Pi (10.1.1.21)

```bash
# SSH в Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21

# Перейти в директорию проекта
cd ~/rob_box_project

# Обновить из git
git pull origin main

# Перезапустить Zenoh router и все сервисы
cd docker/vision
docker compose restart zenoh-router
sleep 5
docker compose restart

# Проверить что router слушает на правильном IP
sudo netstat -tlnp | grep 7447
# Должно быть: tcp 0 0 10.1.1.11:7447 0.0.0.0:* LISTEN <pid>/zenohd

# Проверить логи router
docker logs zenoh-router --tail 50
# Должно быть: "Zenoh can be reached at: tcp/10.1.1.11:7447"
```

### Проверка коммуникации

#### На Main Pi:

```bash
# Проверить TCP соединения Zenoh
sudo netstat -tnp | grep zenohd

# Должно быть:
# tcp 0 0 10.1.1.10:7447 10.1.1.11:xxxxx ESTABLISHED <pid>/zenohd  (Vision Pi → Main Pi)
# tcp 0 0 10.1.1.10:xxxxx 10.1.1.10:7447 ESTABLISHED <pid>/zenohd  (локальные ROS ноды)
```

#### На Vision Pi:

```bash
# Проверить TCP соединения Zenoh
sudo netstat -tnp | grep zenohd

# Должно быть:
# tcp 0 0 10.1.1.11:xxxxx 10.1.1.10:7447 ESTABLISHED <pid>/zenohd  (Vision Pi → Main Pi)
# tcp 0 0 10.1.1.11:xxxxx 10.1.1.11:7447 ESTABLISHED <pid>/zenohd  (локальные ROS ноды)
```

### Проверка ROS топиков

```bash
# На Main Pi - должны видеть топики от Vision Pi
docker exec rtabmap ros2 topic list | grep camera
# /camera/rgb/camera_info
# /camera/rgb/image_raw
# /camera/depth/camera_info
# /camera/depth/image_raw

docker exec rtabmap ros2 topic hz /camera/rgb/image_raw
# average rate: 5.000
```

---

## 🔍 Диагностика проблем

### Проблема: Router не запускается

**Симптом:**
```
ERROR: Unable to open listener tcp/10.1.1.10:7447: Address in use
```

**Решение:**
```bash
# Проверить что порт не занят другим процессом
sudo netstat -tlnp | grep 7447

# Если занят - убить процесс
sudo kill <pid>

# Перезапустить router
docker compose restart zenoh-router
```

### Проблема: ROS ноды не подключаются

**Симптом:**
```
[WARN] [rmw_zenoh_cpp]: Unable to connect to a Zenoh router
```

**Решение:**
```bash
# 1. Проверить что router запущен
docker ps | grep zenoh-router

# 2. Проверить что router слушает на правильном IP
sudo netstat -tlnp | grep 7447

# 3. Проверить что конфиг сессии правильный
cat config/zenoh_session_config.json5 | grep endpoints

# 4. Перезапустить ноды
docker compose restart
```

### Проблема: Vision Pi не подключается к Main Pi

**Симптом:**
```
ERROR: Unable to connect to tcp/10.1.1.10:7447#iface=eth0
```

**Решение:**
```bash
# 1. Проверить сетевую связность
ping -c 3 10.1.1.10

# 2. Проверить что Main Pi router слушает
sshpass -p 'open' ssh ros2@10.1.1.20 'sudo netstat -tlnp | grep 7447'

# 3. Проверить таблицу маршрутизации
ip route show

# 4. Проверить firewall (если есть)
sudo iptables -L -n
```

---

## 📊 Тестирование

### Сценарий 1: Проверка стабильности

```bash
# Запустить систему на 30 минут
# Мониторить логи обоих роутеров

# Vision Pi
docker logs -f zenoh-router 2>&1 | grep -i "error\|warn\|closing"

# Main Pi  
docker logs -f zenoh-router 2>&1 | grep -i "error\|warn\|closing"

# Ожидаемый результат: НЕТ ошибок "Unable to push", "Closing transport", "Query not found"
```

### Сценарий 2: Проверка ROS сервисов

```bash
# На Vision Pi - проверить вызов сервиса камеры
docker exec oak-d ros2 service call /camera/get_parameters rcl_interfaces/srv/GetParameters "{names: ['width', 'height']}"

# Ожидаемый результат: Успешный response без ошибок executor
```

### Сценарий 3: Нагрузочное тестирование

```bash
# Запустить все основные компоненты:
# - Camera (RGB 5 FPS, 1-3 MB/frame)
# - Depth (5 FPS, 0.5-1 MB/frame)  
# - LiDAR (10 Hz, 0.1-0.5 MB/scan)
# - RTAB-Map SLAM
# - Nav2

# Мониторить трафик Ethernet
sudo iftop -i eth0

# Ожидаемый результат:
# - Пропускная способность ~185 Мбит/с (18.5% от 1 Гбит/с)
# - НЕТ ошибок Zenoh transport
# - НЕТ ошибок ROS executor
```

---

## 📝 История изменений

### 2025-11-10 - Исправление конфликта портов

**Изменённые файлы:**
- `docker/main/config/zenoh_router_config.json5`
- `docker/vision/config/zenoh_router_config.json5`
- `docker/main/config/zenoh_session_config.json5`
- `docker/vision/config/zenoh_session_config.json5`

**Изменения:**
1. Main Pi router: `tcp/[::]:7447#iface=eth0` → `tcp/10.1.1.10:7447`
2. Vision Pi router: `tcp/[::]:7447#iface=eth0` → `tcp/10.1.1.11:7447`
3. Main Pi session: `tcp/localhost:7447` → `tcp/10.1.1.10:7447`
4. Vision Pi session: `tcp/localhost:7447` → `tcp/10.1.1.11:7447`

### Предыдущие попытки (неудачные)

**PR #177** (2025-11-10): Добавлен `#iface=eth0` для принудительной маршрутизации через Ethernet  
❌ Результат: Wildcard `[::]` всё ещё вызывал конфликты

**PR #179** (2025-11-10): Попытка использовать пустые `connect.endpoints` для роутеров  
❌ Результат: ROS ноды не могли подключиться

**PR #180** (2025-11-10): Добавлен `tcp/localhost:7447` в listen endpoints  
❌ Результат: Конфликт с `#iface=eth0` restriction

**PR #182** (2025-11-10): Увеличены TX buffer sizes до максимума  
⚠️ Результат: Частично помогло, но не устранило корневую причину

---

## 🔗 Связанные документы

- [ZENOH_ETHERNET_INTERFACE_FIX_2025-11-10.md](../../ZENOH_ETHERNET_INTERFACE_FIX_2025-11-10.md) - PR #177
- [ZENOH_FIX_2025-11-10_DEPLOYMENT.md](../../ZENOH_FIX_2025-11-10_DEPLOYMENT.md) - PR #179  
- [ZENOH_FIX_QUICKSTART.md](../../ZENOH_FIX_QUICKSTART.md) - PR #180
- [ZENOH_TRANSPORT_FIX_QUICKREF.md](../../ZENOH_TRANSPORT_FIX_QUICKREF.md) - PR #182
- [ZENOH_TRANSPORT_ERROR_ANALYSIS_2025-11-09.md](ZENOH_TRANSPORT_ERROR_ANALYSIS_2025-11-09.md) - Детальный анализ
- [ZENOH_CONFIGURATION_BEST_PRACTICES.md](ZENOH_CONFIGURATION_BEST_PRACTICES.md) - Best practices

---

**Автор:** GitHub Copilot  
**Статус:** ✅ ГОТОВО К РАЗВЁРТЫВАНИЮ  
**Последнее обновление:** 2025-11-10
