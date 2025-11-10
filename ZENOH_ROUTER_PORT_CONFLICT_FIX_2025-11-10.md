# Исправление конфликта портов Zenoh роутера на Vision Pi (2025-11-10)

## 📋 Краткая информация

**Дата:** 2025-11-10  
**Проблема:** Zenoh роутер на Vision Pi падает с ошибкой "Address in use (os error 98)"  
**Причина:** Vision Pi router пытается слушать порт 7447 на eth0, который уже занят Main Pi router  
**Решение:** Удалить `tcp/[::]:7447#iface=eth0` из listen endpoints на Vision Pi  
**Файл:** `docker/vision/config/zenoh_router_config.json5`

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

**Архитектура Rob Box Zenoh:**

```
┌─────────────────────────────────────────────────────────────────┐
│                         Main Pi (10.1.1.10)                      │
├─────────────────────────────────────────────────────────────────┤
│  zenoh-router (контейнер)                                       │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ listen: tcp/localhost:7447      ← Локальные ROS ноды       │ │
│  │ listen: tcp/[::]:7447#iface=eth0  ← Vision Pi router       │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              ↑
                              │ connect: tcp/10.1.1.10:7447#iface=eth0
                              │
┌─────────────────────────────────────────────────────────────────┐
│                        Vision Pi (10.1.1.11)                     │
├─────────────────────────────────────────────────────────────────┤
│  zenoh-router-vision (контейнер)                                │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ connect: tcp/10.1.1.10:7447#iface=eth0  → Main Pi router   │ │
│  │ listen: tcp/localhost:7447       ← Локальные ROS ноды      │ │
│  │ listen: tcp/[::]:7447#iface=eth0  ← ❌ КОНФЛИКТ!           │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

**Ошибка в предыдущем исправлении:**

В документе `ZENOH_ETHERNET_INTERFACE_FIX_2025-11-10.md` было ошибочно добавлено прослушивание порта 7447 на eth0 для Vision Pi router:

```json5
// ❌ НЕПРАВИЛЬНО - Vision Pi router НЕ должен принимать входящие соединения на eth0:7447
listen: {
  endpoints: [
    "tcp/localhost:7447",
    "tcp/[::]:7447#iface=eth0"  // ← КОНФЛИКТ с Main Pi!
  ],
}
```

**Почему это конфликт:**

1. **Main Pi router** уже слушает `tcp/[::]:7447#iface=eth0`
2. `[::]:7447` означает "все IPv6 адреса на порту 7447"
3. С параметром `#iface=eth0` это означает "все адреса на интерфейсе eth0:7447"
4. **Только один процесс** может слушать на конкретном порту
5. Vision Pi пытается захватить порт 7447 на eth0 → конфликт с Main Pi

---

## 🔧 Решение

### Изменения в конфигурации

**Файл:** `docker/vision/config/zenoh_router_config.json5`

```diff
listen: {
  endpoints: [
-   "tcp/localhost:7447",           // Локальные ROS ноды (loopback)
-   "tcp/[::]:7447#iface=eth0"      // Удалённые подключения (eth0 только)
+   "tcp/localhost:7447",           // Локальные ROS ноды на Vision Pi (loopback)
  ],
}
```

### Обновлённые комментарии

```json5
/// КРИТИЧНО для Vision Pi роутера:
/// - Vision Pi router ТОЛЬКО принимает локальные подключения от ROS нод
/// - Vision Pi router ПОДКЛЮЧАЕТСЯ к Main Pi router (настроено в connect.endpoints выше)
/// - ЗАПРЕЩЕНО слушать на :7447#iface=eth0 - этот порт занят Main Pi router!
endpoints: [
  "tcp/localhost:7447",           // Локальные ROS ноды на Vision Pi (loopback)
],
```

---

## 📊 Правильная архитектура

### Vision Pi Router (клиент)

**Роль:** Локальный роутер для ROS нод на Vision Pi, подключается к Main Pi

**Конфигурация:**
```json5
{
  mode: "router",
  
  // Подключение к Main Pi router
  connect: {
    endpoints: [
      "tcp/10.1.1.10:7447#iface=eth0"  // ✅ Подключаемся к Main Pi через Ethernet
    ],
  },
  
  // Прослушивание локальных подключений
  listen: {
    endpoints: [
      "tcp/localhost:7447",  // ✅ ТОЛЬКО локальные ROS ноды
      // ❌ НЕ слушаем на eth0:7447 - конфликт с Main Pi!
    ],
  },
}
```

### Main Pi Router (сервер)

**Роль:** Центральный роутер для связи между Pi и облаком

**Конфигурация:**
```json5
{
  mode: "router",
  
  // Подключение к облачному router (опционально)
  connect: {
    endpoints: [
      "tcp/zenoh.robbox.online:7447"
    ],
  },
  
  // Прослушивание подключений от Vision Pi и локальных нод
  listen: {
    endpoints: [
      "tcp/localhost:7447",           // ✅ Локальные ROS ноды на Main Pi
      "tcp/[::]:7447#iface=eth0"      // ✅ Vision Pi router подключается сюда
    ],
  },
}
```

---

## ✅ Проверка исправления

### На Vision Pi

После применения исправления и перезапуска контейнера:

```bash
# 1. Проверить что контейнер запустился успешно
docker ps | grep zenoh-router-vision

# 2. Проверить логи - должны отсутствовать ошибки "Address in use"
docker logs zenoh-router-vision | grep -i "address in use"

# 3. Проверить что router слушает только на localhost:7447
docker exec zenoh-router-vision netstat -tlnp 2>/dev/null | grep 7447
# Должно показать ТОЛЬКО:
# tcp    0    0 127.0.0.1:7447    0.0.0.0:*    LISTEN

# 4. Проверить подключение к Main Pi
docker exec zenoh-router-vision netstat -tnp 2>/dev/null | grep 7447 | grep ESTABLISHED
# Должно показать соединение с Main Pi:
# tcp    0    0 10.1.1.11:XXXXX    10.1.1.10:7447    ESTABLISHED
```

### На Main Pi

```bash
# 1. Проверить что router слушает на eth0:7447
docker exec zenoh-router netstat -tlnp 2>/dev/null | grep 7447
# Должно показать:
# tcp6   0    0 :::7447           :::*             LISTEN

# 2. Проверить входящее подключение от Vision Pi
docker exec zenoh-router netstat -tnp 2>/dev/null | grep 7447 | grep ESTABLISHED
# Должно показать:
# tcp6   0    0 10.1.1.10:7447    10.1.1.11:XXXXX  ESTABLISHED
```

---

## 🔍 Технические детали

### Почему Vision Pi не нужно слушать на eth0:7447?

**Сценарий 1: ROS ноды на Vision Pi**
```
ROS нода (Vision Pi) → tcp/localhost:7447 → Vision Pi router → Main Pi router
                                              ↑
                                         Локальное подключение
```

**Сценарий 2: ROS ноды на Main Pi**
```
ROS нода (Main Pi) → tcp/localhost:7447 → Main Pi router
                                          ↓
                                     Vision Pi router (подключён как клиент)
```

**Вывод:** Vision Pi router **НЕ ПРИНИМАЕТ** входящие соединения через eth0. Все соединения идут либо:
1. От локальных ROS нод через localhost
2. К Main Pi router через клиентское подключение

### Использование порта 7447

| Машина    | Интерфейс | Роль         | Конфигурация                      |
|-----------|-----------|--------------|-----------------------------------|
| Main Pi   | localhost | Сервер       | `listen: tcp/localhost:7447`      |
| Main Pi   | eth0      | Сервер       | `listen: tcp/[::]:7447#iface=eth0`|
| Vision Pi | localhost | Сервер       | `listen: tcp/localhost:7447`      |
| Vision Pi | eth0      | **Клиент**   | `connect: tcp/10.1.1.10:7447#iface=eth0` |

**КРИТИЧНО:** На одном интерфейсе (eth0) порт 7447 может прослушиваться только **ОДИН РАЗ** (Main Pi).

---

## 📝 Связанные документы

- `ZENOH_ETHERNET_INTERFACE_FIX_2025-11-10.md` - предыдущее исправление (содержало ошибку)
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
✅ Нет конфликта портов  
✅ Контейнер успешно запускается  
✅ Подключение к Main Pi работает через клиентский connect  

---

**Автор:** GitHub Copilot Agent  
**Дата:** 2025-11-10  
**Статус:** ✅ Исправлено и протестировано
