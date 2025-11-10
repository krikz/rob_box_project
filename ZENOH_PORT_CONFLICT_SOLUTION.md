# Решение проблемы "Address in use" на Main Pi и Vision Pi

## 📋 Проблемы

1. **Vision Pi**: Падал с ошибкой "Address in use (os error 98)" на порту 7447
2. **Main Pi**: Также падал с той же ошибкой после исправления Vision Pi

## 🔍 Корневая причина

**Конфликт портов в конфигурации Main Pi:**
```json5
endpoints: [
  "tcp/localhost:7447",           // Слушает на ::1:7447 (IPv6 localhost)
  "tcp/[::]:7447#iface=eth0"      // Пытается слушать на ВСЕХ IPv6, включая ::1
]
```

**Проблема:** `[::]:7447` означает "bind на ВСЕ IPv6 адреса", что включает `::1` (localhost).  
Даже с параметром `#iface=eth0`, Zenoh делает bind на всех адресах → конфликт с `localhost:7447`!

## ✅ Решение

**Используем ТОЛЬКО IP адреса eth0 (без localhost/[::]):**

### Main Pi
```json5
// Router config
listen: {
  endpoints: ["tcp/10.1.1.10:7447"]  // ТОЛЬКО eth0 IP
}

// Session config (для локальных ROS нод)
connect: {
  endpoints: ["tcp/10.1.1.10:7447"]  // Подключаться к eth0 IP
}
```

### Vision Pi
```json5
// Router config
listen: {
  endpoints: ["tcp/10.1.1.11:7447"]  // ТОЛЬКО eth0 IP
}
connect: {
  endpoints: ["tcp/10.1.1.10:7447#iface=eth0"]  // К Main Pi
}

// Session config (для локальных ROS нод)
connect: {
  endpoints: ["tcp/10.1.1.11:7447"]  // Подключаться к eth0 IP
}
```

## 🎯 Что это даёт

✅ **Нет конфликта портов** - каждый роутер слушает на своём IP  
✅ **ВЕСЬ трафик через Gigabit Ethernet** - включая локальные ROS ноды  
✅ **Гарантия** - невозможен fallback на WiFi  
✅ **Простота** - один endpoint на роутер  

## ⚠️ Trade-off

Локальные ROS ноды теперь подключаются через сетевой стек (10.1.1.10/11) вместо loopback (127.0.0.1).

**Влияние на производительность:** Минимальное (~1-2 микросекунды latency)  
**Преимущество:** Гарантирует что ВЕСЬ трафик идёт через eth0

## 🚀 Развертывание

### 1. Диагностика (опционально)
```bash
# На Main Pi
ssh ros2@10.1.1.20
cd ~/rob_box_project
./scripts/diagnose_zenoh_port_conflict.sh
```

### 2. Применение исправлений

**⚠️ ВАЖНО: Сначала Main Pi, потом Vision Pi!**

```bash
# Main Pi (сначала!)
ssh ros2@10.1.1.20
cd ~/rob_box_project/docker/main
git pull
./update_and_restart.sh

# Проверить что роутер запустился
docker logs zenoh-router --tail 20
```

Ожидаемый вывод:
```
INFO zenoh::net::runtime::orchestrator: Listener tcp/10.1.1.10:7447 started
INFO zenoh::api::session: Session opened
```

**НЕ должно быть:**
```
WARN Unable to open listener tcp/10.1.1.10:7447: Address in use
```

```bash
# Vision Pi (после Main Pi!)
ssh ros2@10.1.1.21
cd ~/rob_box_project/docker/vision
git pull
./update_and_restart.sh

# Проверить что роутер запустился
docker logs zenoh-router-vision --tail 20
```

### 3. Проверка связи

**На Main Pi:**
```bash
# Проверить подключение от Vision Pi
docker exec zenoh-router netstat -tnp 2>/dev/null | grep "10.1.1.11.*ESTABLISHED"

# Должно показать:
# tcp    0    0 10.1.1.10:7447    10.1.1.11:XXXXX    ESTABLISHED
```

**На Vision Pi:**
```bash
# Проверить подключение к Main Pi
docker exec zenoh-router-vision netstat -tnp 2>/dev/null | grep "10.1.1.10:7447.*ESTABLISHED"

# Должно показать:
# tcp    0    0 10.1.1.11:XXXXX    10.1.1.10:7447    ESTABLISHED
```

## 📊 Архитектура (финальная)

```
Cloud Router (zenoh.robbox.online:7447)
    ↑ tcp/zenoh.robbox.online:7447
Main Pi (10.1.1.10:7447) - HUB
    ↑ tcp/10.1.1.10:7447#iface=eth0
Vision Pi (10.1.1.11:7447)

Локальные ноды:
  Main Pi ROS nodes → tcp/10.1.1.10:7447
  Vision Pi ROS nodes → tcp/10.1.1.11:7447
  
Всё через Gigabit Ethernet! ✅
```

## 📝 Инструменты

**Валидация конфигурации:**
```bash
./scripts/validate_zenoh_config.sh
```

**Диагностика портов:**
```bash
./scripts/diagnose_zenoh_port_conflict.sh
```

**Устранение проблем:**
См. `TROUBLESHOOTING_MAIN_PI_PORT_CONFLICT.md`

---

**Дата:** 2025-11-10  
**Статус:** ✅ Исправлено и готово к развертыванию
