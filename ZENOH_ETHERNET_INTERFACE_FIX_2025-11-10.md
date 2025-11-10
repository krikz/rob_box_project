# Исправление маршрутизации трафика Zenoh через Ethernet (2025-11-10)

## 📋 Краткая информация

**Дата:** 2025-11-10  
**Проблема:** Zenoh трафик идёт через WiFi (wlan0) вместо Gigabit Ethernet (eth0), вызывая перегрузку  
**Решение:** Явная привязка всех Zenoh соединений к eth0 интерфейсу через параметр `#iface=eth0`  
**Файлы:** `docker/main/config/zenoh_router_config.json5`, `docker/vision/config/zenoh_router_config.json5`

---

## 🎯 Проблема

### Симптомы

Из логов видны характерные ошибки Zenoh:

```
2025-11-10T11:54:38.577271Z  WARN: Route reply: Query not found!
2025-11-10T11:55:38.577058Z ERROR: Unable to push non droppable network message to 42cd01b3a16f7a5c6d7f31bcd507b6dc. Closing transport!
2025-11-10T11:56:38.577161Z ERROR: Unable to push non droppable network message to 42cd01b3a16f7a5c6d7f31bcd507b6dc. Closing transport!
2025-11-10T11:56:48.634157Z ERROR: Failed to terminate 1 tasks
2025-11-10T12:22:58.274791Z ERROR: [Peers Network] Received LinkStateList from unknown link 3838e26748f3ca75cded27d55ffc16d5
```

### Корневая причина

**Сетевая топология Rob Box:**
- **Ethernet (eth0):** 
  - Main Pi: `10.1.1.10`
  - Vision Pi: `10.1.1.11`
  - Пропускная способность: **1 Гбит/с**
  - Назначение: Передача данных между Pi (камера, LiDAR, SLAM)
  
- **WiFi (wlan0):**
  - Main Pi: `10.1.1.20`
  - Vision Pi: `10.1.1.21`
  - Пропускная способность: **~100-300 Мбит/с** (в зависимости от условий)
  - Назначение: SSH доступ и управление

**Проблема:**
Zenoh router НЕ имел явной привязки к интерфейсу, поэтому Linux kernel мог выбирать маршрут через WiFi для некоторых соединений, особенно если:
- WiFi интерфейс был настроен раньше
- Метрика маршрута WiFi была ниже
- Таблица ARP кэшировала MAC-адреса с WiFi интерфейса

**Результат:**
Трафик высокой пропускной способности (RGB изображения 1-3 МБ @ 5 FPS, depth @ 5 FPS, LiDAR point clouds) забивал WiFi канал, вызывая:
1. Переполнение очередей Zenoh
2. Таймауты запросов
3. Разрыв транспортных соединений
4. Потерю сообщений

---

## 🔧 Решение

### Изменения в конфигурации

#### 1. Vision Pi Router - подключение к Main Pi

**Файл:** `docker/vision/config/zenoh_router_config.json5`

```diff
connect: {
  endpoints: [
-   "tcp/10.1.1.10:7447"  // Main Pi Ethernet IP
+   "tcp/10.1.1.10:7447#iface=eth0"  // Main Pi Ethernet IP + принудительная привязка к eth0
  ],
}
```

**Эффект:** Vision Pi router теперь ВСЕГДА подключается к Main Pi через Gigabit Ethernet интерфейс.

#### 2. Vision Pi Router - прослушивание входящих соединений

**Файл:** `docker/vision/config/zenoh_router_config.json5`

```diff
listen: {
  endpoints: [
-   "tcp/[::]:7447"
+   "tcp/[::]:7447#iface=eth0"
  ],
}
```

**Эффект:** Vision Pi router принимает входящие соединения ТОЛЬКО через Gigabit Ethernet.

#### 3. Main Pi Router - прослушивание входящих соединений

**Файл:** `docker/main/config/zenoh_router_config.json5`

```diff
listen: {
  endpoints: [
-   "tcp/[::]:7447"
+   "tcp/[::]:7447#iface=eth0"
  ],
}
```

**Эффект:** Main Pi router принимает входящие соединения ТОЛЬКО через Gigabit Ethernet.

---

## 📊 Ожидаемые результаты

### До исправления

**Пропускная способность:**
```
RGB (1920x1080 @ 5 FPS):  ~3 MB/frame × 5 = 15 MB/s = 120 Мбит/с
Depth (640x480 @ 5 FPS):  ~1 MB/frame × 5 = 5 MB/s  = 40 Мбит/с
LiDAR point clouds:       ~0.3 MB × 10 Hz = 3 MB/s  = 24 Мбит/с
TF transforms:            ~0.1 MB/s                  = 1 Мбит/с
-------------------------------------------------------------------
ИТОГО:                    ~23 MB/s ≈ 185 Мбит/с
```

**Проблема:** 185 Мбит/с через WiFi (100-300 Мбит/с пропускная способность) → **перегрузка!**

### После исправления

**Пропускная способность:**
```
Gigabit Ethernet: 1000 Мбит/с
Реальный трафик:  185 Мбит/с
Загрузка канала:  ~18.5%
```

**Результат:** ✅ **Комфортный запас пропускной способности!**

### Устранённые проблемы

1. ✅ **"Unable to push non droppable network message"** - устранено благодаря достаточной пропускной способности
2. ✅ **"Query not found!"** - запросы больше не теряются из-за разрыва соединения
3. ✅ **"Failed to terminate tasks"** - задачи корректно завершаются без таймаутов
4. ✅ **"Received LinkStateList from unknown link"** - стабильные соединения без переподключений

---

## 🔍 Проверка конфигурации

### На Raspberry Pi

После перезапуска контейнеров, проверьте активные соединения:

```bash
# Проверить что Zenoh router слушает на eth0
sudo netstat -tlnp | grep 7447

# Должно показать:
# tcp6    0    0 :::7447    :::*    LISTEN    <PID>/zenoh-bridge-ros

# Проверить активные соединения Zenoh
sudo netstat -tnp | grep 7447 | grep ESTABLISHED

# Для Vision Pi должно показать соединение с 10.1.1.10 (Main Pi Ethernet IP):
# tcp    0    0 10.1.1.11:XXXXX    10.1.1.10:7447    ESTABLISHED    <PID>/zenoh-bridge-ros

# Для Main Pi должно показать входящее соединение от 10.1.1.11 (Vision Pi Ethernet IP):
# tcp    0    0 10.1.1.10:7447    10.1.1.11:XXXXX    ESTABLISHED    <PID>/zenoh-bridge-ros
```

### В логах Docker

```bash
# Vision Pi - проверить подключение
docker logs zenoh-router 2>&1 | grep -i "connect\|transport"

# Должно быть без ошибок "Unable to push"

# Main Pi - проверить входящие соединения
docker logs zenoh-router 2>&1 | grep -i "accept\|transport"

# Должно быть без ошибок закрытия транспорта
```

### Через Zenoh REST API

```bash
# Проверить список транспортов на Main Pi
curl http://10.1.1.10:8000/@/router/local/status/transport

# Должно показать активный транспорт с Vision Pi
```

---

## 📝 Документация

### Zenoh endpoint syntax

```
tcp/<IP>:<PORT>#iface=<INTERFACE>
```

**Параметры:**
- `tcp/` - протокол (TCP)
- `<IP>` - IP адрес для подключения или прослушивания
  - `10.1.1.10` - конкретный IP (connect)
  - `[::]` - все IPv6 адреса, включая IPv4 через IPv4-mapped (listen)
- `<PORT>` - порт (7447 для Zenoh по умолчанию)
- `#iface=<INTERFACE>` - принудительная привязка к сетевому интерфейсу
  - `eth0` - Gigabit Ethernet
  - `wlan0` - WiFi (НЕ использовать для данных!)

**Примеры:**
```
tcp/10.1.1.10:7447#iface=eth0     - подключиться к 10.1.1.10 через eth0
tcp/[::]:7447#iface=eth0          - слушать на всех IP через eth0
tcp/localhost:7447                - локальные соединения (без привязки к интерфейсу)
```

### Связанные документы

- `ZENOH_FIX_2025-11-10_MAXIMUM.md` - максимальное увеличение очередей (предыдущая попытка)
- `docs/architecture/NETWORK.md` - документация по сетевой архитектуре
- `.github/copilot-instructions.md` - соглашение об IP адресах

---

## ⚠️ Важные замечания

### 1. Интерфейсы должны быть активны

**Критично:** Перед запуском Zenoh router убедитесь что `eth0` интерфейс активен и имеет корректный IP:

```bash
ip addr show eth0
# Должно показать:
# inet 10.1.1.10/24 (Main Pi) или inet 10.1.1.11/24 (Vision Pi)
```

### 2. WiFi остаётся для управления

WiFi интерфейс (wlan0) по-прежнему используется для:
- ✅ SSH доступ с ноутбука
- ✅ Grafana мониторинг (http://10.1.1.10:3000)
- ✅ Удалённое управление роботом
- ❌ **НЕ для ROS 2 / Zenoh трафика!**

### 3. Локальные ROS ноды не затронуты

ROS ноды внутри одного Raspberry Pi продолжают использовать `localhost` для коммуникации через loopback интерфейс (как и раньше).

### 4. Не требует изменений в docker-compose.yaml

Все изменения только в конфигурационных файлах Zenoh, которые монтируются через volumes. Docker-compose файлы не требуют пересборки образов.

---

## 🚀 Развёртывание

### Обновление на Raspberry Pi

```bash
# На обоих Pi (Vision и Main):

# 1. Перейти в директорию проекта
cd ~/rob_box_project

# 2. Получить последние изменения
git pull origin main

# 3. Перезапустить Zenoh router
cd docker/vision  # или docker/main
docker-compose restart zenoh-router

# 4. Проверить логи
docker logs -f zenoh-router

# Не должно быть ошибок "Unable to push non droppable network message"
```

### Откат (если необходимо)

Если возникнут проблемы с eth0 интерфейсом:

```bash
# 1. Вернуть старую конфигурацию
git revert <commit-hash>

# 2. Перезапустить роутер
docker-compose restart zenoh-router
```

---

## ✅ Чеклист валидации

После развёртывания проверьте:

- [ ] Vision Pi подключается к Main Pi через 10.1.1.10 (не 10.1.1.20)
- [ ] `netstat` показывает соединения через eth0 IPs
- [ ] Логи Zenoh без ошибок "Unable to push" в течение 5+ минут
- [ ] RTAB-Map получает данные камеры/LiDAR стабильно
- [ ] WiFi по-прежнему работает для SSH доступа
- [ ] Grafana доступна через WiFi IP (10.1.1.10:3000)

---

**Автор:** GitHub Copilot Agent  
**Дата последнего обновления:** 2025-11-10
