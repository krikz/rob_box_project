# Руководство по диагностике проблем Zenoh Transport

**Дата:** 2025-11-10  
**Цель:** Понять ПОЧЕМУ происходят ошибки "Unable to push non droppable network message"  
**Методы:** Увеличение verbosity, мониторинг метрик, анализ трафика

---

## 🔍 Способы диагностики

### 1. Увеличение verbosity Zenoh логов

#### Вариант A: Через переменную окружения RUST_LOG (РЕКОМЕНДУЕТСЯ)

**Уровни логирования Zenoh:**
```
error  - только критичные ошибки (по умолчанию)
warn   - предупреждения
info   - информационные сообщения
debug  - детальная отладка
trace  - максимальная детализация (ВСЁ)
```

**Текущая конфигурация** (`docker-compose.yaml`):
```yaml
environment:
  - RUST_LOG=zenoh=info  # Текущий уровень
```

**Для детальной диагностики** - изменить на:

**Файл:** `docker/vision/docker-compose.yaml` и `docker/main/docker-compose.yaml`

```yaml
zenoh-router:
  image: eclipse/zenoh:latest
  environment:
    # МАКСИМАЛЬНАЯ детализация для диагностики
    - RUST_LOG=zenoh=trace,zenoh_transport=trace
    
    # ИЛИ выборочная детализация (рекомендуется)
    - RUST_LOG=zenoh=info,zenoh_transport::unicast::universal::tx=trace,zenoh_transport::unicast::universal::rx=trace
```

**Применение:**
```bash
# Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21 'cd ~/rob_box_project/docker/vision && docker compose restart zenoh-router'

# Main Pi  
sshpass -p 'open' ssh ros2@10.1.1.20 'cd ~/rob_box_project/docker/main && docker compose restart zenoh-router'
```

#### Вариант B: Включить в zenoh_router_config.json5

**Файлы:** 
- `docker/main/config/zenoh_router_config.json5`
- `docker/vision/config/zenoh_router_config.json5`

**Добавить секцию adminspace для REST API:**
```json5
{
  // ... существующая конфигурация ...
  
  // Включить админ интерфейс для мониторинга
  adminspace: {
    // Разрешить доступ к статистике и метрикам
    permissions: {
      read: true,
      write: false,
    },
  },
  
  // Опционально: включить Prometheus плагин
  plugins: {
    // REST API для статистики
    rest: {
      enabled: true,
      http_port: 8000,  // Main Pi: 8000, Vision Pi: 8001
    },
    
    // Prometheus метрики (если установлен плагин)
    // prometheus: {
    //   enabled: true,
    //   http_port: 9090,
    // },
  },
}
```

**⚠️ Внимание:** Не все плагины могут быть доступны в `eclipse/zenoh:latest` образе!

---

## 📊 Что искать в логах

### 1. Паттерны ошибок

**Ошибка "Unable to push":**
```
ERROR zenoh_transport::unicast::universal::tx: Unable to push non droppable network message to 42cd01b3a16f7a5c6d7f31bcd507b6dc. Closing transport!
```

**Что смотреть:**
- ✅ **Peer ID** (42cd01b3a16f7a5c6d7f31bcd507b6dc) - идентификатор удалённого peer
- ✅ **ThreadId** - какой поток столкнулся с проблемой
- ✅ **Частота ошибок** - каждые 30с, 60с?
- ✅ **Паттерн** - всегда один и тот же peer или разные?

### 2. Дополнительные детали при trace уровне

**Что появится в trace логах:**

```
TRACE zenoh_transport::unicast::universal::tx: Pushing message to queue: control/real_time/data_high
TRACE zenoh_transport::unicast::universal::tx: Queue stats: used=15/16, pending_bytes=960KB/1024KB
TRACE zenoh_transport::unicast::universal::tx: Waiting for queue space... attempt 1
TRACE zenoh_transport::unicast::universal::tx: Waiting for queue space... attempt 2
...
TRACE zenoh_transport::unicast::universal::tx: Wait timeout exceeded (60000000 microseconds)
ERROR zenoh_transport::unicast::universal::tx: Unable to push non droppable network message...
```

**Полезная информация:**
- **Какая очередь переполнена**: control, real_time, data_high?
- **Сколько попыток было**: количество attempt
- **Размер очереди**: used/total
- **Bytes в очереди**: сколько данных застряло

### 3. Анализ причин переполнения

**Возможные причины (из trace логов):**

#### A. Сетевая проблема
```
TRACE zenoh_transport::unicast::universal::link: TCP send blocked, retry...
TRACE zenoh_transport::unicast::universal::link: TCP congestion detected
```
→ Проверить сеть: `iperf3`, `ping`, `ethtool`

#### B. Высокая нагрузка данных
```
TRACE zenoh_transport: Incoming message size: 3145728 bytes (3 MB)
TRACE zenoh_transport: Incoming message size: 3145728 bytes (3 MB)
TRACE zenoh_transport: Incoming message size: 3145728 bytes (3 MB)
```
→ Включить компрессию изображений

#### C. Медленный receiver
```
TRACE zenoh_transport::unicast::universal::rx: RX queue full, dropping messages
TRACE zenoh_transport: Receiver processing slow, backpressure
```
→ Оптимизировать обработку на Main Pi (RTAB-Map downsampling)

#### D. Проблемы с peer discovery
```
TRACE zenoh::net::routing: Peer 42cd01b3a16f7a5c6d7f31bcd507b6dc disconnected
TRACE zenoh::net::routing: Reconnecting to peer...
TRACE zenoh_transport: Transport establishment timeout
```
→ Проверить конфигурацию роутеров, connectivity

---

## 🛠️ Практическая диагностика

### Шаг 1: Включить trace логи

**Файл:** `docker/vision/docker-compose.yaml`
```yaml
zenoh-router:
  environment:
    # Детализированные логи ТОЛЬКО для transport модуля
    - RUST_LOG=zenoh=info,zenoh_transport=trace
```

**Файл:** `docker/main/docker-compose.yaml`
```yaml
zenoh-router:
  environment:
    # То же самое на Main Pi
    - RUST_LOG=zenoh=info,zenoh_transport=trace
```

### Шаг 2: Перезапустить и собрать логи

```bash
# Vision Pi - перезапуск с новыми логами
sshpass -p 'open' ssh ros2@10.1.1.21 << 'EOF'
cd ~/rob_box_project/docker/vision
docker compose restart zenoh-router
echo "Waiting 10 seconds for router to start..."
sleep 10
echo "=== Vision Pi Zenoh Router - Last 200 lines ==="
docker logs zenoh-router --tail 200
EOF

# Main Pi - перезапуск с новыми логами
sshpass -p 'open' ssh ros2@10.1.1.20 << 'EOF'
cd ~/rob_box_project/docker/main
docker compose restart zenoh-router
echo "Waiting 10 seconds for router to start..."
sleep 10
echo "=== Main Pi Zenoh Router - Last 200 lines ==="
docker logs zenoh-router --tail 200
EOF
```

### Шаг 3: Запустить нагрузку и наблюдать

```bash
# Vision Pi - следить за логами в реальном времени
sshpass -p 'open' ssh ros2@10.1.1.21 'docker logs -f zenoh-router 2>&1 | grep -E "(TRACE|ERROR|push|queue)"'
```

**В другом терминале - запустить камеру и лидар:**
```bash
# Запустить нагрузку
sshpass -p 'open' ssh ros2@10.1.1.21 'cd ~/rob_box_project/docker/vision && docker compose up -d oak-d lslidar'
```

**Наблюдать за логами 5-10 минут и анализировать паттерны**

### Шаг 4: Сохранить логи для анализа

```bash
# Vision Pi - сохранить полные логи
sshpass -p 'open' ssh ros2@10.1.1.21 'docker logs zenoh-router > /tmp/vision_zenoh_trace.log 2>&1'
sshpass -p 'open' scp ros2@10.1.1.21:/tmp/vision_zenoh_trace.log ./vision_zenoh_trace.log

# Main Pi - сохранить полные логи
sshpass -p 'open' ssh ros2@10.1.1.20 'docker logs zenoh-router > /tmp/main_zenoh_trace.log 2>&1'
sshpass -p 'open' scp ros2@10.1.1.20:/tmp/main_zenoh_trace.log ./main_zenoh_trace.log
```

---

## 📈 REST API для мониторинга (если включён)

### Проверка доступности REST API

```bash
# Main Pi (порт 8000)
curl http://10.1.1.10:8000/@/router/local

# Vision Pi (порт 8001)
curl http://10.1.1.11:8001/@/router/local
```

### Получение статистики транспортов

```bash
# Статистика всех транспортов
curl http://10.1.1.10:8000/@/router/local/transports

# Детали конкретного peer
curl http://10.1.1.10:8000/@/router/local/transports/<peer_id>
```

**Пример ответа:**
```json
{
  "peer_id": "42cd01b3a16f7a5c6d7f31bcd507b6dc",
  "links": [
    {
      "src": "tcp/10.1.1.11:7447",
      "dst": "tcp/10.1.1.10:7447",
      "mtu": 65535,
      "is_reliable": true,
      "is_streamed": true
    }
  ],
  "tx_stats": {
    "sent_bytes": 1234567890,
    "sent_msgs": 123456,
    "dropped_msgs": 42,  // ⚠️ ВАЖНО!
    "queue_full_errors": 15  // ⚠️ ВАЖНО!
  }
}
```

---

## 🔬 Дополнительные инструменты диагностики

### 1. Мониторинг сетевого трафика

**iftop - Real-time bandwidth monitor:**
```bash
# Vision Pi - мониторинг Ethernet трафика
sshpass -p 'open' ssh ros2@10.1.1.21 'sudo iftop -i eth0 -n -P'
```

**Что искать:**
- Пиковые значения bandwidth (должно быть < 800 Mbps для Gigabit)
- Направление трафика (TX vs RX)
- Наличие ретрансмиссий TCP

### 2. Проверка качества сети

**Ping test с высокой частотой:**
```bash
# С Vision Pi в Main Pi
sshpass -p 'open' ssh ros2@10.1.1.21 'ping 10.1.1.10 -c 1000 -i 0.01 -s 1400'
```

**Ожидаемые результаты:**
- ✅ **0% packet loss**
- ✅ **avg latency < 1ms**
- ✅ **max latency < 5ms**
- ❌ Если >1% loss или >10ms - проблема в сети!

**iperf3 - Тест пропускной способности:**
```bash
# Main Pi - запустить сервер
sshpass -p 'open' ssh ros2@10.1.1.20 'iperf3 -s'

# Vision Pi - запустить клиент
sshpass -p 'open' ssh ros2@10.1.1.21 'iperf3 -c 10.1.1.10 -t 60 -i 5'
```

**Ожидаемые результаты:**
- ✅ **>900 Mbits/sec** (для Gigabit Ethernet)
- ✅ **Нет ретрансмиссий** (Retr = 0)
- ❌ Если <500 Mbits/sec - проблема в кабеле/коммутаторе!

### 3. Мониторинг топиков ROS 2

**Проверка частоты публикации:**
```bash
# Подключиться к любому Pi
sshpass -p 'open' ssh ros2@10.1.1.21

# Проверить частоту камеры
ros2 topic hz /camera/rgb/image_raw

# Проверить размер сообщений
ros2 topic bw /camera/rgb/image_raw
```

**Что искать:**
- ✅ Стабильная частота (например, 5 Hz постоянно)
- ❌ Скачущая частота (5 → 3 → 7 → 2 Hz) - признак проблем
- ❌ Большой размер сообщений (>3 MB) - нужна компрессия

### 4. htop - CPU и память

```bash
# Vision Pi - мониторинг ресурсов
sshpass -p 'open' ssh ros2@10.1.1.21 'htop'
```

**Что искать:**
- ✅ CPU zenoh-router < 10%
- ✅ CPU общий < 80%
- ✅ Доступная память > 1 GB
- ❌ Swap использование > 0 - недостаточно RAM!

---

## 📋 Чеклист диагностики

### Базовая диагностика (30 минут)

- [ ] Включить `RUST_LOG=zenoh_transport=trace` на обоих Pi
- [ ] Перезапустить zenoh-router
- [ ] Запустить нагрузку (камера + лидар)
- [ ] Собрать логи через 10 минут
- [ ] Найти в логах:
  - [ ] Какая очередь переполняется (control/real_time/data_high)?
  - [ ] Размер сообщений (bytes)
  - [ ] Частота ошибок (каждые X секунд)
  - [ ] Peer ID который вызывает проблему

### Расширенная диагностика (1-2 часа)

- [ ] Тест пропускной способности `iperf3`
- [ ] Тест качества сети `ping` с большим пакетом
- [ ] Мониторинг трафика `iftop`
- [ ] Проверка частоты топиков `ros2 topic hz`
- [ ] Проверка размера сообщений `ros2 topic bw`
- [ ] Мониторинг CPU/RAM `htop`
- [ ] Анализ логов Zenoh trace
- [ ] Документирование findings

---

## 🎯 Интерпретация результатов

### Сценарий 1: Переполнение control/real_time очередей

**Признаки в логах:**
```
TRACE: Queue stats: control used=16/16 (100%)
ERROR: Unable to push non droppable network message
```

**Причина:** Высокоприоритетные данные (TF, команды) не успевают передаваться

**Решение:**
- ✅ Уже применены максимальные буферы (16 batch)
- ✅ Проверить сеть (`iperf3`, `ping`)
- ✅ Убедиться что таймаут 60 секунд применён

### Сценарий 2: Переполнение data_high очереди

**Признаки в логах:**
```
TRACE: Queue stats: data_high used=12/12 (100%)
TRACE: Incoming message size: 3145728 bytes (3 MB camera image)
ERROR: Unable to push non droppable network message
```

**Причина:** Большие данные камеры переполняют буфер

**Решение:**
- ✅ Включить компрессию изображений (JPEG/PNG)
- ✅ Снизить FPS камеры с 30 до 5-10 Hz
- ✅ Использовать `image_transport: compressed`

### Сценарий 3: Сетевые проблемы

**Признаки в логах:**
```
TRACE: TCP send blocked, retry...
TRACE: Link congestion detected
TRACE: Retransmission timeout
```

**Проверка:**
```bash
iperf3 -c 10.1.1.10 -t 60
# Результат: <500 Mbits/sec или Retr > 0
```

**Решение:**
- ❌ Заменить Ethernet кабель
- ❌ Проверить коммутатор
- ❌ Убедиться в 1000 Mbps link speed (`ethtool eth0`)

### Сценарий 4: Медленная обработка на receiver

**Признаки в логах:**
```
TRACE: RX queue full, backpressure
TRACE: Receiver processing slow
```

**Проверка:**
```bash
htop  # На Main Pi
# CPU RTAB-Map > 90% постоянно
```

**Решение:**
- ✅ Downsampling в RTAB-Map
- ✅ Снизить FPS камеры
- ✅ Увеличить `cloud_decimation`, `cloud_voxel_size`

---

## 🚀 Быстрый старт диагностики

### Для срочной диагностики ПРЯМО СЕЙЧАС:

```bash
# 1. Включить trace логи (Vision Pi)
sshpass -p 'open' ssh ros2@10.1.1.21 << 'EOF'
cd ~/rob_box_project/docker/vision
# Отредактировать docker-compose.yaml вручную или временно:
docker compose stop zenoh-router
docker run -d --name zenoh-router-debug \
  --network host \
  -v $(pwd)/config/zenoh_router_config.json5:/config.json5:ro \
  -e RUST_LOG=zenoh=info,zenoh_transport=trace \
  eclipse/zenoh:latest \
  -c /config.json5
EOF

# 2. Следить за логами
sshpass -p 'open' ssh ros2@10.1.1.21 'docker logs -f zenoh-router-debug 2>&1 | grep -E "(ERROR|TRACE.*queue|TRACE.*push)"'

# 3. В другом терминале - запустить нагрузку
sshpass -p 'open' ssh ros2@10.1.1.21 'cd ~/rob_box_project/docker/vision && docker compose up -d oak-d'

# 4. Наблюдать 5-10 минут, затем остановить
sshpass -p 'open' ssh ros2@10.1.1.21 'docker stop zenoh-router-debug && docker rm zenoh-router-debug && cd ~/rob_box_project/docker/vision && docker compose up -d zenoh-router'
```

---

## 📚 Полезные ссылки

- [Zenoh Troubleshooting Guide](https://zenoh.io/docs/getting-started/troubleshooting/)
- [Zenoh Configuration Reference](https://zenoh.io/docs/manual/configuration/)
- [Zenoh REST API Documentation](https://zenoh.io/docs/manual/plugin-storage-manager/#rest-api)
- [RUST_LOG Environment Variable](https://docs.rs/env_logger/latest/env_logger/)

---

**Автор:** AI Agent  
**Дата:** 2025-11-10  
**Статус:** ✅ Готово к использованию
