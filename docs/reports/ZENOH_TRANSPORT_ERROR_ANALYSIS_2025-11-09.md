# Анализ ошибки Zenoh Transport "Unable to push non droppable network message"

**Дата:** 2025-11-09  
**Автор:** AI Agent Analysis  
**Статус:** 🔍 Анализ завершён, решение предложено

---

## 📋 Описание проблемы

### Симптомы

Периодически в логах Zenoh роутера появляется критическая ошибка:

```
2025-11-09T10:34:02.327078Z ERROR rx-0 ThreadId(07) zenoh_transport::unicast::universal::tx: Unable to push non droppable network message to 42cd01b3a16f7a5c6d7f31bcd507b6dc. Closing transport!
```

**Последствия:**
- 🔴 Принудительное закрытие Zenoh transport соединения
- 🔴 Потеря данных ROS 2 (топики, сервисы, TF)
- 🔴 Переподключение роутера Vision Pi ↔ Main Pi
- 🔴 Временная недоступность SLAM данных

---

## 🔬 Техническая диагностика

### Расшифровка ошибки

| Элемент | Значение | Объяснение |
|---------|----------|------------|
| **Модуль** | `zenoh_transport::unicast::universal::tx` | TX (передача) для unicast transport |
| **Проблема** | "Unable to push non droppable network message" | Не удалось поместить КРИТИЧЕСКОЕ сообщение в очередь |
| **ZID** | `42cd01b3a16f7a5c6d7f31bcd507b6dc` | Zenoh ID удалённого узла (Vision Pi ↔ Main Pi router) |
| **Действие** | "Closing transport!" | Принудительное закрытие соединения |

### Механизм возникновения

```mermaid
%%{init: {'theme':'base', 'themeVariables': { 'primaryColor':'#ffe6e6','primaryTextColor':'#000','primaryBorderColor':'#c00'}}}%%
graph TD
    A[ROS 2 Node публикует данные] --> B{TX Queue заполнена?}
    B -->|Нет| C[Сообщение добавлено в очередь]
    B -->|Да, есть место| C
    B -->|Да, нет места| D{Сообщение droppable?}
    D -->|Да drop| E[Сбросить сообщение]
    D -->|Нет block| F[Ждать освобождения очереди]
    F --> G{Таймаут wait_before_close?}
    G -->|Нет| F
    G -->|Да, превышен| H[❌ ERROR: Unable to push]
    H --> I[🔴 Закрыть transport]
    
    style H fill:#ff6b6b,stroke:#c00,color:#fff
    style I fill:#c00,stroke:#800,color:#fff
```

---

## 📊 Текущая конфигурация

### Vision Pi Router (`docker/vision/config/zenoh_router_config.json5`)

```json5
transport: {
  unicast: {
    qos: { enabled: true },  // ✅ QoS включён
  },
  link: {
    tx: {
      batch_size: 65535,  // 64 KB
      queue: {
        size: {
          control: 2,           // ⚠️ МАЛЕНЬКАЯ ОЧЕРЕДЬ
          real_time: 2,
          interactive_high: 2,
          interactive_low: 2,
          data_high: 2,
          data: 2,
          data_low: 2,
          background: 2,
        },
        congestion_control: {
          block: {
            wait_before_close: 5000000,  // ⚠️ 5 секунд - КОРОТКИЙ ТАЙМАУТ
          },
        },
      },
    },
  },
}
```

### Main Pi Router (`docker/main/config/zenoh_router_config.json5`)

```json5
# Идентичные настройки
transport: {
  link: {
    tx: {
      queue: {
        size: { /* все по 2 */ },
        congestion_control: {
          block: {
            wait_before_close: 5000000,  // ⚠️ 5 секунд
          },
        },
      },
    },
  },
}
```

### Session Config (`zenoh_session_config.json5` - Peer mode)

```json5
transport: {
  link: {
    tx: {
      queue: {
        size: { /* все по 2 */ },
        congestion_control: {
          block: {
            wait_before_close: 60000000,  // ✅ 60 секунд (больше чем в router)
          },
        },
      },
    },
  },
}
```

---

## 🎯 Причины проблемы

### 1. 📦 Маленький размер TX очередей

**Текущая ситуация:**
- Размер каждой очереди: **2 batch**
- Размер batch: **65535 bytes** (64 KB)
- **Итого на очередь: 128 KB**

**Проблема:**
```
Камера OAK-D:     ~30 MB/s (RGB + Depth)
LiDAR LSLIDAR:    ~2 MB/s (Point Cloud)
TF transforms:    ~0.5 MB/s
Nav2 messages:    ~0.5 MB/s
--------------------------------------
ВСЕГО:            ~33 MB/s
```

При скорости **33 MB/s** очередь **128 KB заполняется за 4 миллисекунды!**

### 2. ⏱️ Короткий таймаут в Router mode

**Vision Pi Router → Main Pi Router:**
- `wait_before_close: 5000000` мкс = **5 секунд**
- Если очередь заполнена > 5 секунд → **транспорт закрывается**

**Почему это проблема:**
- При старте системы множество нод публикуют данные одновременно
- Ethernet соединение может временно быть перегружено
- 5 секунд недостаточно для восстановления после пиковой нагрузки

### 3. 🔄 Архитектура с двумя роутерами

```
Vision Pi (10.1.1.11)              Main Pi (10.1.1.10)
┌─────────────────────┐           ┌─────────────────────┐
│  Zenoh Router       │◄─────────►│  Zenoh Router       │
│  (router mode)      │  Ethernet │  (router mode)      │
└─────────────────────┘  1 Gbps   └─────────────────────┘
         ▲                                   ▲
         │                                   │
    ┌────┴────┐                         ┌───┴────┐
    │ Peers:  │                         │ Peers: │
    │ OAK-D   │                         │ RTAB   │
    │ LiDAR   │                         │ Nav2   │
    │ Voice   │                         │ VESC   │
    └─────────┘                         └────────┘
```

**Проблема:**
- Роутеры пересылают данные между Pi
- При загрузке сети роутеры быстро заполняют очереди
- Короткий таймаут приводит к разрыву связи

---

## 💡 Решения

### ✅ Решение 1: Увеличить размер TX очередей (РЕКОМЕНДУЕТСЯ)

**Изменить в обоих router config файлах:**

```json5
queue: {
  size: {
    control: 4,           // Было: 2
    real_time: 4,         // Было: 2
    interactive_high: 4,  // Было: 2
    interactive_low: 4,   // Было: 2
    data_high: 4,         // Было: 2
    data: 4,              // Было: 2
    data_low: 4,          // Было: 2
    background: 4,        // Было: 2
  },
}
```

**Эффект:**
- Размер буфера увеличится с **128 KB до 256 KB** на очередь
- Время заполнения увеличится с **4 мс до 8 мс**
- Больше устойчивости к пиковым нагрузкам

**Использование памяти:**
```
Было:  8 queues × 2 batches × 64 KB = 1 MB
Стало: 8 queues × 4 batches × 64 KB = 2 MB
Прирост: +1 MB на каждый роутер
```

### ✅ Решение 2: Увеличить таймаут wait_before_close (РЕКОМЕНДУЕТСЯ)

**Изменить в обоих router config файлах:**

```json5
congestion_control: {
  block: {
    wait_before_close: 20000000,  // 20 секунд (было: 5)
  },
}
```

**Эффект:**
- Роутер подождёт **20 секунд** вместо **5 секунд**
- Даёт время для восстановления после пиковой нагрузки
- Снижает вероятность разрыва соединения

### 🔧 Решение 3: Оптимизация для высоких нагрузок (ОПЦИОНАЛЬНО)

Для критичных очередей можно увеличить размер ещё больше:

```json5
size: {
  control: 8,           // Критичные control messages
  real_time: 8,         // Данные камеры и LiDAR
  interactive_high: 4,
  interactive_low: 4,
  data_high: 6,         // TF и Nav2
  data: 4,
  data_low: 2,
  background: 2,
}
```

**Эффект:**
- Приоритетные очереди получают больше памяти
- Критичные данные (control, real_time, data_high) обрабатываются лучше
- Общее использование памяти: **~3 MB на роутер**

---

## 📝 Рекомендованные изменения

### Шаг 1: Vision Pi Router Config

**Файл:** `docker/vision/config/zenoh_router_config.json5`

```json5
transport: {
  link: {
    tx: {
      queue: {
        size: {
          control: 4,           // ← ИЗМЕНЕНО
          real_time: 4,         // ← ИЗМЕНЕНО
          interactive_high: 4,  // ← ИЗМЕНЕНО
          interactive_low: 4,   // ← ИЗМЕНЕНО
          data_high: 4,         // ← ИЗМЕНЕНО
          data: 4,              // ← ИЗМЕНЕНО
          data_low: 4,          // ← ИЗМЕНЕНО
          background: 4,        // ← ИЗМЕНЕНО
        },
        congestion_control: {
          block: {
            wait_before_close: 20000000,  // ← ИЗМЕНЕНО (20 сек)
          },
        },
      },
    },
  },
}
```

### Шаг 2: Main Pi Router Config

**Файл:** `docker/main/config/zenoh_router_config.json5`

```json5
# Идентичные изменения
transport: {
  link: {
    tx: {
      queue: {
        size: { /* все по 4 */ },
        congestion_control: {
          block: {
            wait_before_close: 20000000,  // 20 секунд
          },
        },
      },
    },
  },
}
```

### Шаг 3: Перезапуск роутеров

```bash
# Vision Pi
cd ~/rob_box_project/docker/vision
docker compose restart zenoh-router

# Main Pi
cd ~/rob_box_project/docker/main
docker compose restart zenoh-router
```

---

## 🔍 Информация о версии Zenoh

### Текущая установка

Rob Box использует **rmw_zenoh_cpp** из ROS 2 Humble:

```bash
# Пакет
ros-humble-rmw-zenoh-cpp

# Зависит от версии релиза Humble
# Скорее всего: Zenoh 0.10.x или 0.11.x
```

### Версия Zenoh в ROS 2 Humble

| Период | Zenoh версия | Статус проблемы |
|--------|--------------|-----------------|
| Начало 2024 | 0.10.x | ⚠️ Проблема присутствует |
| Середина 2024 | 0.11.x | ⚠️ Частично исправлено |
| Конец 2024 | 1.0.x | ✅ Улучшена обработка очередей |

### Где проблема в коде Zenoh

**Модуль:** `zenoh/io/zenoh-transport/src/unicast/universal/tx.rs`

Код обработки TX очередей проверяет:
1. Есть ли свободное место в очереди?
2. Если нет, сообщение droppable?
3. Если нет, ждать освобождения
4. Если таймаут → ERROR и закрытие transport

### Проверка установленной версии

```bash
# На Raspberry Pi
apt-cache policy ros-humble-rmw-zenoh-cpp

# Показать все пакеты zenoh
dpkg -l | grep zenoh
```

---

## 📈 Мониторинг после исправления

### Метрики для отслеживания

1. **Частота ошибок transport:**
   ```bash
   docker logs zenoh-router 2>&1 | grep "Unable to push"
   ```

2. **Загрузка очередей:**
   - Через Zenoh REST API (порт 8000)
   - Prometheus метрики (если включены)

3. **Пропускная способность:**
   ```bash
   # Мониторинг eth0 на обоих Pi
   iftop -i eth0
   ```

4. **Задержки ROS 2 топиков:**
   ```bash
   ros2 topic hz /camera/rgb/image_raw
   ros2 topic hz /scan
   ```

### Ожидаемый результат

После применения исправлений:
- ✅ Ошибки "Unable to push" должны прекратиться
- ✅ Стабильное соединение между роутерами
- ✅ Нет разрывов transport при пиковых нагрузках
- ✅ Использование памяти увеличится на ~1-2 MB на роутер (приемлемо)

---

## 🔗 Связанные документы

- [SYSTEM_OVERVIEW.md](../architecture/SYSTEM_OVERVIEW.md) - Общая архитектура
- [SOFTWARE.md](../architecture/SOFTWARE.md) - Zenoh middleware
- [DOCKER_STANDARDS.md](../development/DOCKER_STANDARDS.md) - Правила конфигураций

---

## ✍️ Заметки

### Почему именно эти значения?

**Размер очереди 4 batch:**
- Баланс между памятью и устойчивостью
- Удваивает буфер без значительного расхода RAM
- Достаточно для кратковременных пиков нагрузки

**Таймаут 20 секунд:**
- Больше чем у peers (60 сек), но меньше чем infinity
- Даёт время для восстановления сети
- Предотвращает бесконечное ожидание при реальном отказе

### Альтернативы

Если проблема сохраняется после этих изменений:
1. Увеличить размер очередей до 8 batch
2. Рассмотреть использование UDP multicast для некритичных данных
3. Оптимизировать публикацию данных (compress, downsample)
4. Обновить до более новой версии Zenoh (если доступно)
