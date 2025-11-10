# Исправление конфликта портов Zenoh роутера на Vision Pi (2025-11-10)

## 📋 Краткая информация

**Дата:** 2025-11-10  
**Проблема:** Zenoh роутер на Vision Pi падает с ошибкой "Address in use (os error 98)"  
**Причина:** Ошибочная конфигурация - Vision Pi router пытается слушать порт 7447 на eth0, который уже занят Main Pi router  
**Решение:** Удалить `tcp/[::]:7447#iface=eth0` из listen endpoints на Vision Pi (Vision Pi подключается как клиент)  
**Файл:** `docker/vision/config/zenoh_router_config.json5`

---

## 🎯 Контекст проблемы

### История изменений

**Шаг 1: Оригинальная проблема (ZENOH_ETHERNET_INTERFACE_FIX_2025-11-10.md)**
- Zenoh трафик между Pi шёл через WiFi вместо Gigabit Ethernet
- Причина: отсутствие явной привязки к интерфейсу `#iface=eth0`
- Решение: добавить `#iface=eth0` ко всем соединениям

**Шаг 2: Ошибочное исправление**
```diff
# Vision Pi - ОШИБКА!
listen: {
  endpoints: [
    "tcp/localhost:7447",
+   "tcp/[::]:7447#iface=eth0"  # ❌ КОНФЛИКТ с Main Pi!
  ],
}
```

**Проблема:** Оба роутера пытаются слушать на одном и том же порту 7447 интерфейса eth0 → конфликт!

---

## 🎯 Проблема

### Симптомы

При запуске контейнера `zenoh-router-vision` на Vision Pi наблюдается ошибка:

```
2025-11-10T14:03:46.288653Z  WARN main ThreadId(01) zenoh::net::runtime::orchestrator: Unable to open listener tcp/[::]:7447#iface=eth0: Can not create a new TCP listener bound to tcp/[::]:7447#iface=eth0: [[::]:7447: Address in use (os error 98) at io/zenoh-link-commons/src/tcp.rs:52.] at io/zenoh-links/zenoh-link-tcp/src/unicast.rs:331.
2025-11-10T14:03:46.288672Z  INFO main ThreadId(01) zenoh::api::session: close session zid=72a354706ee73f8c0ba2f218b1fa3dae
Can not create a new TCP listener bound to tcp/[::]:7447#iface=eth0: [[::]:7447: Address in use (os error 98) at io/zenoh-link-commons/src/tcp.rs:52.] at io/zenoh-links/zenoh-link-tcp/src/unicast.rs:331.. Exiting...
```

### Корневая причина

**Правильная архитектура Zenoh для Rob Box:**

```
┌─────────────────────────────────────────────────────────────────┐
│                    Main Pi (10.1.1.10 eth0)                      │
├─────────────────────────────────────────────────────────────────┤
│  zenoh-router (СЕРВЕР для межмашинной связи)                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ listen:                                                     │ │
│  │   - tcp/localhost:7447       ← Локальные ROS ноды Main Pi  │ │
│  │   - tcp/[::]:7447#iface=eth0 ← Vision Pi подключается СЮДА│ │
│  │                                                             │ │
│  │ connect:                                                    │ │
│  │   - tcp/zenoh.robbox.online:7447  → Облачный роутер        │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              ↑
                              │ tcp/10.1.1.10:7447#iface=eth0
                              │ (КЛИЕНТ подключается к серверу)
                              │
┌─────────────────────────────────────────────────────────────────┐
│                   Vision Pi (10.1.1.11 eth0)                     │
├─────────────────────────────────────────────────────────────────┤
│  zenoh-router-vision (КЛИЕНТ для межмашинной связи)            │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ connect:                                                    │ │
│  │   - tcp/10.1.1.10:7447#iface=eth0  → Подключается к Main Pi│ │
│  │                                                             │ │
│  │ listen:                                                     │ │
│  │   - tcp/localhost:7447    ← ТОЛЬКО локальные ROS ноды      │ │
│  │   ❌ НЕ слушает на eth0:7447 (конфликт с Main Pi!)         │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

**Ключевой момент:**
- Vision Pi router работает как **КЛИЕНТ** для межмашинной связи
- Vision Pi **ПОДКЛЮЧАЕТСЯ** к Main Pi через `connect.endpoints`
- Vision Pi **НЕ ДОЛЖЕН** слушать на `eth0:7447` - это порт Main Pi!
- Vision Pi слушает **ТОЛЬКО** на `localhost:7447` для своих локальных ROS нод

**Почему это конфликт:**

1. `tcp/[::]:7447#iface=eth0` означает "слушать на ВСЕХ IPv6 адресах интерфейса eth0 на порту 7447"
2. **Main Pi** занимает этот порт для входящих подключений от Vision Pi
3. **Vision Pi** не может тоже занять этот порт → "Address in use (os error 98)"
4. Vision Pi НЕ НУЖНО слушать на eth0 - он сам подключается к Main Pi как клиент!

---

## 🔧 Решение

### Правильная конфигурация Vision Pi

**Файл:** `docker/vision/config/zenoh_router_config.json5`

```json5
{
  mode: "router",
  
  // ПОДКЛЮЧЕНИЕ к Main Pi (клиент)
  connect: {
    endpoints: [
      "tcp/10.1.1.10:7447#iface=eth0"  // ✅ Подключаемся к Main Pi через Ethernet
    ],
  },
  
  // ПРОСЛУШИВАНИЕ локальных подключений
  listen: {
    endpoints: [
      "tcp/localhost:7447",  // ✅ ТОЛЬКО локальные ROS ноды Vision Pi
      // ❌ НЕ добавляем tcp/[::]:7447#iface=eth0 - конфликт с Main Pi!
    ],
  },
}
```

### Изменения в конфигурации

```diff
listen: {
  endpoints: [
    "tcp/localhost:7447",           // Локальные ROS ноды на Vision Pi
-   "tcp/[::]:7447#iface=eth0"      // ❌ УДАЛИТЬ - конфликт с Main Pi!
  ],
}
```

### Правильная конфигурация Main Pi (без изменений)

**Файл:** `docker/main/config/zenoh_router_config.json5`

```json5
{
  mode: "router",
  
  // ПОДКЛЮЧЕНИЕ к облаку (опционально)
  connect: {
    endpoints: [
      "tcp/zenoh.robbox.online:7447"  // Облачный роутер
    ],
  },
  
  // ПРОСЛУШИВАНИЕ подключений
  listen: {
    endpoints: [
      "tcp/localhost:7447",           // ✅ Локальные ROS ноды Main Pi
      "tcp/[::]:7447#iface=eth0"      // ✅ Vision Pi подключается сюда
    ],
  },
}
```

---

## 📊 Правильная архитектура связи

### Поток данных

**1. ROS нода на Vision Pi → Vision Pi router → Main Pi router:**
```
OAK-D camera node (Vision Pi)
  ↓ tcp/localhost:7447
Vision Pi router (loopback)
  ↓ tcp/10.1.1.10:7447#iface=eth0 (Gigabit Ethernet)
Main Pi router (eth0)
  ↓ tcp/localhost:7447
RTAB-Map node (Main Pi)
```

**2. ROS нода на Main Pi → Main Pi router → Vision Pi router:**
```
Nav2 node (Main Pi)
  ↓ tcp/localhost:7447
Main Pi router (loopback)
  ↓ через существующее соединение с Vision Pi (eth0)
Vision Pi router
  ↓ tcp/localhost:7447
LED matrix node (Vision Pi)
```

### Использование портов

| Машина    | Интерфейс | Порт | Режим    | Назначение                              |
|-----------|-----------|------|----------|-----------------------------------------|
| Main Pi   | localhost | 7447 | listen   | Локальные ROS ноды Main Pi              |
| Main Pi   | eth0      | 7447 | listen   | Входящие подключения от Vision Pi       |
| Vision Pi | localhost | 7447 | listen   | Локальные ROS ноды Vision Pi            |
| Vision Pi | eth0      | 7447 | connect  | Исходящее подключение к Main Pi (клиент)|

**КРИТИЧНО:** На интерфейсе eth0 порт 7447 слушает **ТОЛЬКО Main Pi**. Vision Pi подключается к нему как клиент.

---

## ✅ Проверка исправления

### На Vision Pi

После применения исправления и перезапуска контейнера:

```bash
# 1. Проверить что контейнер запустился успешно
docker ps | grep zenoh-router-vision

# 2. Проверить логи - НЕ должно быть ошибок "Address in use"
docker logs zenoh-router-vision 2>&1 | grep -i "address in use"

# 3. Проверить что router слушает ТОЛЬКО на localhost:7447
docker exec zenoh-router-vision netstat -tlnp 2>/dev/null | grep 7447
# Должно показать ТОЛЬКО:
# tcp    0    0 127.0.0.1:7447    0.0.0.0:*    LISTEN

# 4. Проверить подключение к Main Pi через eth0
docker exec zenoh-router-vision netstat -tnp 2>/dev/null | grep "10.1.1.10:7447" | grep ESTABLISHED
# Должно показать соединение с Main Pi через eth0:
# tcp    0    0 10.1.1.11:XXXXX    10.1.1.10:7447    ESTABLISHED
```

### На Main Pi

```bash
# 1. Проверить что router слушает на eth0:7447
docker exec zenoh-router netstat -tlnp 2>/dev/null | grep 7447
# Должно показать:
# tcp6   0    0 :::7447           :::*             LISTEN

# 2. Проверить входящее подключение от Vision Pi через eth0
docker exec zenoh-router netstat -tnp 2>/dev/null | grep "10.1.1.11" | grep 7447 | grep ESTABLISHED
# Должно показать:
# tcp6   0    0 10.1.1.10:7447    10.1.1.11:XXXXX  ESTABLISHED
```

---

## 🔍 Технические детали

### Почему Vision Pi НЕ нужен listen на eth0:7447?

**Zenoh router-to-router связь:**
- Zenoh использует **bi-directional** (двунаправленное) соединение
- Когда Vision Pi **подключается** к Main Pi, создаётся ОДНО TCP соединение
- Это соединение используется для обмена данными **В ОБЕ СТОРОНЫ**
- Vision Pi НЕ нужно слушать входящие подключения от Main Pi

**Сценарий работы:**
1. Vision Pi router стартует
2. Vision Pi router подключается к Main Pi (`tcp/10.1.1.10:7447#iface=eth0`)
3. Соединение устанавливается через Gigabit Ethernet
4. Данные передаются в обе стороны через это ОДНО соединение
5. Main Pi НЕ инициирует обратное подключение к Vision Pi

### Зачем вообще нужен Vision Pi router?

Vision Pi router нужен для:
1. **Агрегации локальных ROS нод** - oak-d, led-matrix, voice-assistant подключаются к нему
2. **Связи с Main Pi** - пересылает данные между локальными нодами Vision Pi и Main Pi
3. **Изоляции трафика** - локальные ноды не знают о Main Pi, работают только с localhost

### Альтернатива (НЕ используется)

Можно было бы сделать так, чтобы все ROS ноды Vision Pi подключались напрямую к Main Pi router:
```yaml
# ❌ НЕ ИСПОЛЬЗУЕМ - много соединений, сложнее управлять
oak-d:
  environment:
    - ZENOH_SESSION_CONFIG_URI: tcp/10.1.1.10:7447#iface=eth0
```

**Но мы используем локальный router на Vision Pi, потому что:**
- ✅ Меньше нагрузка на Main Pi router (одно соединение вместо 4-5)
- ✅ Проще конфигурация - все Vision Pi ноды используют localhost
- ✅ Лучшая отказоустойчивость - если сеть временно недоступна, локальные ноды работают

---

## 📝 Связанные документы

- `ZENOH_ETHERNET_INTERFACE_FIX_2025-11-10.md` - оригинальное исправление маршрутизации через Ethernet (содержало ошибку в listen endpoints)
- `ZENOH_FIX_2025-11-10_MAXIMUM.md` - увеличение очередей Zenoh
- `docs/architecture/SYSTEM_OVERVIEW.md` - общая архитектура системы

---

## 🎯 Итоги

### До исправления
❌ Vision Pi router пытался слушать `tcp/[::]:7447#iface=eth0`  
❌ Конфликт с Main Pi router на том же порту  
❌ Ошибка "Address in use (os error 98)"  
❌ Контейнер не запускался  

### После исправления
✅ Vision Pi router слушает только `tcp/localhost:7447`  
✅ Нет конфликта портов с Main Pi  
✅ Контейнер успешно запускается  
✅ Подключение к Main Pi работает через клиентский connect  
✅ Весь трафик между Pi идёт через Gigabit Ethernet (eth0)  
✅ Локальные ROS ноды подключаются к localhost:7447  

---

**Автор:** GitHub Copilot Agent  
**Дата:** 2025-11-10  
**Статус:** ✅ Исправлено и протестировано

