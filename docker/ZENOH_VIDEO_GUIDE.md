# Zenoh Configuration - Best Practices from Articulated Robotics Video

## Видео источник
**"Zenoh: A Better Way to Communicate in ROS"**  
Ссылка: https://articulatedrobotics.xyz/

---

## 🎯 Основные выводы блогера

### Проблемы DDS (00:00:32)
- ❌ Нестабильность в Wi-Fi сетях
- ❌ Topics исчезают без объяснений
- ❌ Flooding сетей (два офиса парализуют друг друга)
- ❌ Криптичные XML конфигурации
- ❌ Переменные окружения непонятные

### Преимущества Zenoh
- ✅ Написан на Rust (высокая производительность)
- ✅ Масштабируемый протокол
- ✅ Работает локально И в облаке
- ✅ Поддержка интернет-передачи БЕЗ VPN
- ✅ Простая конфигурация JSON5

---

## 📊 3 режима работы Zenoh

### Режим 1: Router Mode ✅ РЕКОМЕНДУЕТСЯ (06:23)

**Как работает**:
```
Vision Pi Router ←→ Main Pi Router ←→ Cloud Router
       ↓                    ↓
   Camera (peer)      RTAB-Map (peer)
```

**Плюсы**:
- Роутеры делают discovery между собой
- Ноды общаются НАПРЯМУЮ после discovery
- Можно остановить роутер - ноды продолжат работать
- **Изоляция сетей** - разные офисы не мешают

**Конфигурация Vision Pi Router** (08:14):
```json5
{
  mode: "router",
  connect: {
    endpoints: ["tcp/10.1.1.10:7447"]  // ← ЯВНО указываем Main Pi Ethernet
  },
  listen: {
    endpoints: ["tcp/[::]:7447"]
  }
}
```

**Конфигурация Camera (peer session)**:
```json5
{
  mode: "peer",
  connect: {
    endpoints: ["tcp/localhost:7447"]  // ← К ЛОКАЛЬНОМУ роутеру
  }
}
```

---

### Режим 2: Session Direct Mode (10:38)

**Как работает**:
```
Vision Pi (NO router) ──TCP──► Main Pi Router
                                     ↓
                               RTAB-Map (peer)
```

**Когда использовать**:
- Raspberry Pi со слабым CPU (экономия ресурсов)
- Клиент-серверная архитектура

**Конфигурация Camera на Vision Pi** (11:14):
```json5
// NO router на Vision Pi!
// Camera session config:
{
  mode: "peer",
  connect: {
    endpoints: ["tcp/10.1.1.10:7447"]  // ← Напрямую к Main Pi router
  }
}
```

**Минусы**:
- Все ноды на Vision Pi через сеть к Main Pi
- Нет локального fast discovery

---

### Режим 3: Peer Multicast Mode ⚠️ НЕ РАБОТАЕТ (12:43)

**Как работает**:
```
Camera (peer) ←UDP multicast→ RTAB-Map (peer)
```

**Конфигурация** (13:08):
```json5
{
  mode: "peer",
  listen: {
    endpoints: ["tcp/0.0.0.0:0"]
  },
  scouting: {
    multicast: {
      enabled: true
    }
  }
}
```

**Environment variable** (13:35):
```bash
export ZENOH_ROUTER_CHECK_ATTEMPTS=0  # Игнорировать отсутствие роутера
```

**Блогер НЕ СМОГ заставить это работать стабильно!** ❌

---

## 🌐 Облачный роутер (15:01)

**Архитектура**:
```
Home PC Router ──► Cloud Router (DigitalOcean) ◄── Office PC Router
                         ↓
                   Видео через ROS topic
                   ЧЕРЕЗ ИНТЕРНЕТ
                   БЕЗ VPN! (17:38)
```

**Конфигурация**:
```json5
// Home PC router.json5
{
  mode: "router",
  connect: {
    endpoints: ["tcp/cloud-server-ip:7447"]
  }
}

// Office PC router.json5
{
  mode: "router",
  connect: {
    endpoints: ["tcp/cloud-server-ip:7447"]
  }
}
```

**Пример из видео** (17:08):
- USB camera на офисном PC
- Передача через облачный роутер
- Просмотр image_view на домашнем PC
- **БЕЗ VPN!**

---

## 🛠️ Рекомендации для Rob Box Project

### ✅ Что уже правильно

1. **Router mode** - как в видео ✅
2. **Peer sessions** - к localhost:7447 ✅
3. **Cloud router** - zenoh.robbox.online ✅

### ⚠️ Что нужно улучшить

#### 1. **Explicit Router Connect** (08:14)

**Сейчас** (Vision Pi router):
```json5
connect: {
  endpoints: []  // ← ПУСТО! Полагается на автообнаружение
}
```

**Нужно** (по рекомендациям блогера):
```json5
connect: {
  endpoints: ["tcp/10.1.1.10:7447"]  // ← ЯВНО к Main Pi Ethernet
}
```

**Почему важно**:
- Быстрее установка соединения
- Нет зависимости от multicast discovery
- Предсказуемое поведение

---

#### 2. **Listen на конкретном интерфейсе** (06:42)

**Сейчас**:
```json5
listen: {
  endpoints: ["tcp/[::]:7447"]  // ← Слушает ВСЕ интерфейсы
}
```

**Опционально** (для строгой изоляции):
```json5
listen: {
  endpoints: ["tcp/0.0.0.0:7447#iface=eth0"]  // ← ТОЛЬКО Ethernet
}
```

**Почему может быть полезно**:
- Исключает случайные подключения через WiFi
- Строгая изоляция data/management сетей

---

#### 3. **Environment Variables** (из видео 04:12)

**Нужно установить**:
```bash
# В docker-compose.yaml для всех Zenoh нод:
environment:
  - RMW_IMPLEMENTATION=rmw_zenoh_cpp  # ← УЖЕ ЕСТЬ ✅
  - ZENOH_CONFIG=/config/zenoh_session_config.json5  # ← УЖЕ ЕСТЬ ✅
```

У вас УЖЕ правильно! ✅

---

#### 4. **Router Healthcheck** (из кода блогера)

**У вас**:
```yaml
healthcheck:
  test: ["CMD-SHELL", "wget -qO- http://localhost:8000/@/local/router || exit 1"]
  interval: 5s
  timeout: 10s
  retries: 20
  start_period: 30s
```

✅ **ОТЛИЧНО!** Блогер этого не показал, у вас даже лучше!

---

## 🚀 Оптимизированные конфигурации

### Vision Pi Router (`zenoh_router_config.json5`)

```json5
{
  mode: "router",
  
  // Подключение к Main Pi router (ЯВНО по Ethernet)
  connect: {
    timeout_ms: { router: -1, peer: -1, client: 0 },
    endpoints: [
      "tcp/10.1.1.10:7447"  // ← Main Pi Ethernet ЯВНО
    ],
    exit_on_failure: { router: false, peer: false, client: true },
    retry: {
      period_init_ms: 1000,
      period_max_ms: 4000,
      period_increase_factor: 2,
    },
  },
  
  // Слушаем на порту 7447 (можно ограничить только eth0)
  listen: {
    timeout_ms: 0,
    endpoints: [
      "tcp/[::]:7447"  // Все интерфейсы
      // ИЛИ для строгой изоляции:
      // "tcp/0.0.0.0:7447#iface=eth0"
    ],
  },
  
  // ... остальное стандартное
}
```

---

### Main Pi Router (`zenoh_router_config.json5`)

```json5
{
  mode: "router",
  
  // Подключение к облачному роутеру
  connect: {
    timeout_ms: { router: -1, peer: -1, client: 0 },
    endpoints: [
      "tcp/zenoh.robbox.online:7447"  // ← Облачный router
    ],
    exit_on_failure: { router: false, peer: false, client: true },
    retry: {
      period_init_ms: 1000,
      period_max_ms: 4000,
      period_increase_factor: 2,
    },
  },
  
  // Слушаем для входящих от Vision Pi
  listen: {
    timeout_ms: 0,
    endpoints: [
      "tcp/[::]:7447"
    ],
  },
  
  // ... остальное стандартное
}
```

---

### Session Config (Camera, RTAB-Map)

```json5
{
  mode: "peer",
  
  // Подключение к ЛОКАЛЬНОМУ роутеру
  connect: {
    timeout_ms: { router: -1, peer: -1, client: 0 },
    endpoints: [
      "tcp/localhost:7447"  // ← К локальному router
    ],
  },
  
  // ... остальное стандартное
}
```

---

## 🎓 Ключевые цитаты блогера

**05:33** - "Каждая нода (peer) должна знать роутер для discovery других нод. Но после discovery ноды общаются НАПРЯМУЮ."

**06:23** - "Если убить роутеры, communication продолжается! Просто нет больше discovery."

**06:52** - "Эти две машины НЕ МОГУТ говорить друг с другом. Некоторые видят это как баг. Но для многих это ФИЧА - нет interference между офисами!"

**08:14** - "В конфиге роутера добавляем IP адрес другого роутера в endpoints."

**17:38** - "Видео transmission через ROS topic через интернет БЕЗ VPN!"

---

## 📝 Checklist оптимизации

- [ ] Добавить `tcp/10.1.1.10:7447` в Vision Pi router `connect.endpoints`
- [ ] Проверить что Main Pi router подключается к `zenoh.robbox.online:7447`
- [ ] Опционально: ограничить `listen` только на eth0 для строгой изоляции
- [ ] Проверить healthcheck работает (wget router status)
- [ ] Проверить прямую связь нод после discovery (netstat)
- [ ] Документировать в TROUBLESHOOTING.md

---

**Источник**: Articulated Robotics - "Zenoh: A Better Way to Communicate in ROS"  
**Дата анализа**: 2025-10-09  
**Применимо к**: Rob Box Project Zenoh architecture
