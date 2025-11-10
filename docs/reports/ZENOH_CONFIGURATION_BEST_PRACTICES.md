# Zenoh Configuration Best Practices (на основе официальной документации)

**Дата создания:** 2025-11-10  
**Источники:** 
- https://docs.rs/zenoh/latest/zenoh/
- https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5
- https://github.com/eclipse-zenoh/zenoh/blob/main/zenoh/src/net/protocol/network.rs

---

## 📚 Ключевые находки из официальной документации

### 1. Ограничения размера очередей

**Из DEFAULT_CONFIG.json5:**
```
/// NOTE: the number of batches in each priority must be included between 1 and 16. 
/// Different values will result in an error.
```

**Вывод:** ✅ Наша конфигурация (control=8, real_time=8, data_high=6) находится в допустимом диапазоне 1-16.

---

### 2. Расчёт использования памяти

**Из документации:**
```
/// The amount of memory being allocated for each queue is then SIZE_XXX * BATCH_SIZE.
/// In the case of the transport link MTU being smaller than the ZN_BATCH_SIZE,
/// then amount of memory being allocated for each queue is SIZE_XXX * LINK_MTU.
```

**Наши расчёты:**

| Очередь | Размер (batches) | BATCH_SIZE | Память на очередь | Комментарий |
|---------|-----------------|------------|-------------------|-------------|
| control | 8 | 64 KB | 512 KB | Критичные control сообщения |
| real_time | 8 | 64 KB | 512 KB | Данные камеры и LiDAR |
| interactive_high | 4 | 64 KB | 256 KB | Интерактивные данные высокого приоритета |
| interactive_low | 4 | 64 KB | 256 KB | Интерактивные данные низкого приоритета |
| data_high | 6 | 64 KB | 384 KB | TF и Nav2 |
| data | 4 | 64 KB | 256 KB | Обычные данные |
| data_low | 2 | 64 KB | 128 KB | Низкоприоритетные данные |
| background | 2 | 64 KB | 128 KB | Фоновые задачи |

**Итого:** 2.5 MB на один Zenoh router

**Для всей системы:**
- Main Pi router: ~2.5 MB
- Vision Pi router: ~2.5 MB
- **Общее увеличение:** ~5 MB (приемлемо для Raspberry Pi 4 с 4GB RAM)

---

### 3. Рекомендации для высокопропускных сценариев

**Из DEFAULT_CONFIG.json5:**
```
/// For very high throughput scenarios, the rx_buffer_size can be increased to accommodate
/// more in-flight data. This is particularly relevant when dealing with large messages.
/// E.g. for 16MiB rx_buffer_size set the value to: 16777216.
```

**Текущая конфигурация:**
```json5
rx: {
  buffer_size: 65535,  // 64 KB (дефолт)
}
```

**Рекомендация для Rob Box:**
Учитывая наши большие сообщения (RGB изображения ~1-3 MB, depth maps ~0.5-1 MB, point clouds ~0.1-0.5 MB), можно рассмотреть увеличение RX буфера:

```json5
rx: {
  buffer_size: 2097152,  // 2 MB (вместо 64 KB)
  // или даже
  buffer_size: 4194304,  // 4 MB для больших изображений
}
```

**⚠️ Важно:** Это дополнительная оптимизация, которую стоит применить, если текущее исправление недостаточно.

---

### 4. Congestion Control в высокопроизводительных приложениях

**Из примера z_pub_thr.rs:**
```rust
let publisher = session
    .declare_publisher("test/thr")
    .congestion_control(CongestionControl::Block)  // ← Блокирующий режим
    .priority(prio)
    .express(args.express)
    .wait()
    .unwrap();
```

**Вывод:** Для критичных данных (камера, LiDAR) правильно использовать `CongestionControl::Block`, что приводит к ошибкам "Unable to push" при переполнении очередей. Наше решение (увеличение очередей) - правильный подход.

---

### 5. Режимы выделения памяти

**Из DEFAULT_CONFIG.json5:**
```json5
allocation: {
  /// Mode for memory allocation of batches in the priority queues.
  /// - "init": batches are allocated at queue initialization time.
  /// - "lazy": batches are allocated when needed up to the maximum number 
  ///           of batches configured in the size configuration parameter.
  mode: "lazy",
}
```

**Текущая конфигурация:** `mode: "lazy"` ✅

**Преимущество "lazy":**
- Память выделяется по мере необходимости
- Снижается начальное потребление RAM
- Подходит для динамических нагрузок (Rob Box)

**Альтернатива "init":**
- Вся память выделяется сразу при старте
- Более предсказуемое поведение
- Подходит для систем с постоянной высокой нагрузкой

**Рекомендация для Rob Box:** Оставить "lazy" - наша нагрузка динамическая (робот может стоять на месте или активно двигаться).

---

### 6. Дефолтный таймаут wait_before_close

**Из DEFAULT_CONFIG.json5:**
```json5
block: {
  /// The maximum time in microseconds to wait for an available batch before 
  /// closing the transport session when sending a blocking message
  /// if still no batch is available.
  wait_before_close: 5000000,  // 5 секунд (дефолт)
}
```

**Наше значение:** 30000000 (30 секунд) - **в 6 раз больше дефолта** ✅

**Обоснование:** 
- Rob Box работает в условиях пиковых нагрузок (до 60 MB/s)
- Raspberry Pi может временно тормозить при нагрузке CPU
- 30 секунд даёт достаточный запас времени для восстановления

---

## 🎯 Сравнение с дефолтной конфигурацией Zenoh

| Параметр | Zenoh Default | Rob Box Vision Pi | Rob Box Main Pi | Обоснование |
|----------|---------------|-------------------|-----------------|-------------|
| **Queue: control** | 2 | 8 | 8 | Критичные сообщения управления |
| **Queue: real_time** | 2 | 8 | 8 | Данные сенсоров высокого потока |
| **Queue: data_high** | 2 | 6 | 6 | TF и навигация |
| **Queue: data** | 2 | 4 | 4 | Обычные данные |
| **Queue: data_low** | 2 | 2 | 2 | Низкоприоритетные данные |
| **Queue: background** | 2 | 2 | 2 | Фоновые задачи |
| **wait_before_close** | 5s | 30s | 30s | Больший запас при пиках |
| **rx buffer_size** | 64 KB | 64 KB | 64 KB | Можно увеличить до 2-4 MB |
| **batch_size** | 65535 | 65535 | 65535 | Максимальный размер batch |
| **allocation mode** | lazy | lazy | lazy | Динамическое выделение памяти |

---

## 🔧 Потенциальные дополнительные оптимизации

### 1. Увеличение RX buffer для больших сообщений

**Проблема:** RGB изображения от OAK-D могут быть 1-3 MB, что больше дефолтного RX буфера (64 KB).

**Решение:**
```json5
// В обоих router config файлах
rx: {
  buffer_size: 2097152,  // 2 MB (вместо 64 KB)
  max_message_size: 1073741824,  // 1 GB (дефолт, оставить)
}
```

**Эффект:**
- Лучшая обработка больших изображений
- Снижение фрагментации сообщений
- Меньше overhead на defragmentation

**Использование памяти:** +2 MB на каждый router (приемлемо)

---

### 2. Настройка TCP buffer sizes для Ethernet

**Из документации:**
```
/// For TCP and TLS links, it is possible to specify the TCP buffer sizes:
/// E.g. tcp/192.168.0.1:7447#so_sndbuf=65000;so_rcvbuf=65000
```

**Текущая конфигурация:** Используются дефолтные размеры TCP буферов ОС.

**Возможная оптимизация:**
```json5
// Увеличить TCP буферы для Gigabit Ethernet
tcp: {
  so_rcvbuf: 2097152,  // 2 MB receive buffer
  so_sndbuf: 2097152,  // 2 MB send buffer
}
```

**⚠️ Внимание:** Требует тестирования, так как может конфликтовать с настройками ОС.

---

### 3. Использование приоритетов в endpoint конфигурации

**Из документации:**
```
/// It is also possible to specify a priority range to be used on the link.
/// For example `tcp/localhost?prio=6-7;rel=0` assigns priorities 
/// "data_low" and "background" to the established link.
```

**Текущая конфигурация Vision Pi router:**
```json5
connect: {
  endpoints: [
    "tcp/10.1.1.10:7447"  // Main Pi Ethernet IP
  ],
}
```

**Возможная оптимизация (приоритет real_time + control):**
```json5
connect: {
  endpoints: [
    "tcp/10.1.1.10:7447?prio=0-1"  // Priorities: control (0) + real_time (1)
  ],
}
```

**⚠️ Внимание:** Требует глубокого понимания приоритетов Zenoh. Может ограничить передачу других приоритетов.

---

## 📊 Валидация нашей конфигурации

### ✅ Что мы делаем правильно

1. **Размеры очередей в допустимом диапазоне** (1-16) ✅
2. **Приоритизация критичных очередей** (control, real_time) ✅
3. **Использование "lazy" allocation** для динамических нагрузок ✅
4. **Увеличенный таймаут** для Raspberry Pi ✅
5. **QoS включён** для правильной работы приоритетов ✅
6. **Batch size максимальный** (65535 bytes) ✅

### ⚠️ Что можно улучшить в будущем

1. **RX buffer увеличить до 2-4 MB** для больших изображений
2. **TCP buffer sizes настроить** под Gigabit Ethernet
3. **Мониторинг метрик Zenoh** через REST API (порт 8000)
4. **Тестирование shared memory** (сейчас отключено)

---

## 🔍 Источник ошибки "Cannot find link" (подтверждение)

**Из network.rs:287:**
```rust
pub(crate) fn get_local_context(&self, context: NodeId, link_id: usize) -> NodeId {
    match self.get_link(link_id) {
        Some(link) => { /* ... */ },
        None => {
            tracing::error!("Cannot find link {}", link_id);  // ← Наша ошибка
            0
        }
    }
}
```

**Механизм:**
1. Transport закрывается из-за "Unable to push" (переполнение очереди)
2. Link удаляется из `VecMap<Link>` (коллекция активных links)
3. Другие части кода пытаются обратиться к удалённому link_id
4. Возникает "Cannot find link 1" ошибка

**Решение:** Предотвратить закрытие transport путём увеличения очередей и таймаута. ✅

---

## 📝 Рекомендации для мониторинга

### 1. Проверка метрик через Zenoh REST API

```bash
# Получить статус router
curl http://10.1.1.10:8000/@/router/local

# Получить информацию о транспортах
curl http://10.1.1.10:8000/@/router/local/transports

# Получить статистику
curl http://10.1.1.10:8000/@/router/local/stats
```

### 2. Мониторинг через логи

```bash
# Следить за критичными ошибками
docker logs -f zenoh-router 2>&1 | grep -E "(ERROR|Unable|Cannot find)"

# Статистика transport events
docker logs zenoh-router 2>&1 | grep -c "transport"
```

### 3. Проверка использования памяти

```bash
# Использование памяти контейнером
docker stats zenoh-router --no-stream

# Общая память системы
free -h
```

---

## ✅ Заключение

Наша конфигурация Zenoh для Rob Box:
- ✅ Соответствует лучшим практикам из официальной документации
- ✅ Находится в допустимых пределах (queue size 1-16)
- ✅ Оптимизирована для высокопроизводительных сценариев
- ✅ Использует правильные режимы (lazy allocation, QoS enabled)
- ⚠️ Может быть дополнительно улучшена (RX buffer, TCP buffers)

**Следующие шаги:**
1. Развернуть текущую конфигурацию на роботе
2. Мониторить 24-48 часов
3. Если ошибки исчезли - зафиксировать как стабильную конфигурацию
4. Если ошибки сохраняются - применить дополнительные оптимизации (RX buffer)

---

## 🔗 Полезные ссылки

- **Zenoh Documentation:** https://docs.rs/zenoh/latest/zenoh/
- **Default Config:** https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5
- **Network Protocol:** https://github.com/eclipse-zenoh/zenoh/blob/main/zenoh/src/net/protocol/network.rs
- **Performance Examples:** https://github.com/eclipse-zenoh/zenoh/tree/main/examples/examples

---

**Автор:** AI Agent Analysis  
**Дата:** 2025-11-10  
**Версия:** 1.0
