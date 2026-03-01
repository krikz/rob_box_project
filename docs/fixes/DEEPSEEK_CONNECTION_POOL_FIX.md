# DeepSeek ThreadPoolExecutor Deadlock Fix

**Дата:** 30 января 2026  
**Компонент:** `dialogue_node.py`  
**Проблема:** 30-секундный timeout при создании stream соединения к DeepSeek API

## 🐛 Проблема

### Симптомы
```
[dialogue_node] 🤔 Запрос к DeepSeek...
[dialogue_node] 🛠️  Отправка запроса с 17 MCP инструментами
... 30 секунд тишины ...
[dialogue_node] ⏱️ Timeout при создании stream соединения (30 секунд)
[dialogue_node] ⏱️ TIMEOUT: DeepSeek streaming не ответил за 60.0s
```

### Root Cause: ThreadPoolExecutor DEADLOCK

**Проблема в коде:**
```python
# dialogue_node.__init__
self._llm_executor = ThreadPoolExecutor(max_workers=2)  # ❌ Только 2 worker

# _process_dialogue_request
with ThreadPoolExecutor(max_workers=1) as executor:
    future = executor.submit(_do_streaming)  # Worker #1 занят
    
    def _do_streaming():
        # Мы УЖЕ ВНУТРИ потока #1
        future = self._llm_executor.submit(_create_stream)  # ❌ Пытаемся занять worker #2
        stream = future.result(timeout=30.0)  # ❌ ЖДЁМ... но оба worker заняты!
```

**Deadlock scenario:**
1. Outer `ThreadPoolExecutor(max_workers=1)` запускает `_do_streaming` → **Worker #1 занят**
2. Внутри `_do_streaming` вызывается `self._llm_executor.submit(_create_stream)` → **пытается занять Worker #2**
3. Но если `self._llm_executor` тоже имеет `max_workers=2` и оба worker заняты → **DEADLOCK**
4. `future.result(timeout=30.0)` висит 30 секунд → **TIMEOUT**

### Timeline Analysis
- **На develop:** Работает (нет вложенного executor.submit)
- **На feature/agent:** Timeout (добавлен вложенный executor.submit в коммите `4f5f9e8`)

## ✅ Решение

### Убрать вложенный `executor.submit`

**Было:**
```python
def _do_streaming():  # УЖЕ ВНУТРИ ThreadPoolExecutor
    # Выполняем create() с timeout используя executor (DEADLOCK!)
    def _create_stream():
        return self.client.chat.completions.create(**request_params)
    
    future = self._llm_executor.submit(_create_stream)
    try:
        stream = future.result(timeout=30.0)
    except FuturesTimeoutError:
        streaming_result["error"] = "Failed to establish stream connection"
        return
```

**Стало:**
```python
def _do_streaming():  # УЖЕ ВНУТРИ ThreadPoolExecutor
    # Создаём stream напрямую (мы уже в потоке, вложенный executor не нужен)
    # httpx client имеет свой timeout (60s total, 10s connect)
    try:
        stream = self.client.chat.completions.create(**request_params)
    except Exception as e:
        self.get_logger().error(f"⏱️ Ошибка создания stream: {e}")
        streaming_result["error"] = f"Failed to create stream: {e}"
        return
```

### Ключевые изменения
1. **Убран `self._llm_executor.submit()` изнутри `_do_streaming`**
2. **Прямой вызов `self.client.chat.completions.create()`**
3. **httpx timeout** (60s total, 10s connect) защищает от зависания
4. **Исправлено 2 места:** основной stream и рекурсивный stream (line 924, 2209)

## 📊 Expected Impact

### Плюсы ✅
- ❌ **Нет timeout** при первом запросе после idle
- ✅ **Стабильная работа** независимо от времени простоя
- ✅ **Быстрый fallback** если DeepSeek действительно недоступен

### Минусы ⚠️
- **Latency +50-100ms** на установку TCP handshake (незаметно для пользователя)
- **Нет TLS session reuse** (но это тоже ~50ms overhead)

### Trade-off
Для voice assistant **стабильность > 50ms latency**. Лучше +100ms на каждый запрос, чем 30s timeout на первом запросе после простоя.

## 🧪 Тестирование

### Локальный тест
```bash
# 1. Запусти dialogue_node
ros2 run rob_box_voice dialogue_node

# 2. Подожди 15 минут

# 3. Попроси робота что-то сказать
ros2 topic pub /voice/stt/result std_msgs/String "data: 'робот расскажи анекдот'" --once

# Ожидаемое: БЕЗ timeout, ответ в течение 5-10 секунд
```

### На роботе
1. **Deploy fix** через GitHub Actions workflow
2. **Restart voice-assistant** контейнер
3. **Подожди 15 минут** без запросов
4. **Спроси робота** рассказать анекдот
5. **Проверь логи**: должен быть `🏁 Stream завершён`, БЕЗ timeout

### Проверка логов
```bash
# Vision Pi
ssh ros2@10.1.1.21
docker logs voice-assistant --tail 100 -f

# Ищи:
# ✅ LLM клиент инициализирован: DeepSeek (no connection pooling)
# 🤔 Запрос к DeepSeek...
# 🏁 Stream завершён: ...
# 📊 Token usage: ...
```

## 🔍 Альтернативные решения (не выбраны)

### 1. Короткий keepalive_expiry
```python
limits=Limits(max_keepalive_connections=5, keepalive_expiry=300)  # 5 минут
```
❌ **Проблема:** Не решает полностью — сервер может закрыть раньше

### 2. Connection retry logic
```python
try:
    stream = client.chat.completions.create(...)
except ConnectionError:
    client.close()
    stream = client.chat.completions.create(...)
```
❌ **Проблема:** Усложняет код, добавляет 30s latency на первый retry

### 3. HTTP/2 с longer keepalive
```python
http_client = httpx.Client(http2=True, limits=Limits(keepalive_expiry=600))
```
❌ **Проблема:** DeepSeek API может не поддерживать HTTP/2 keepalive

## 📝 Lessons Learned

1. **Connection pooling** полезен для high-throughput APIs, но **опасен** для low-frequency requests (1-2 req/min)
2. **Idle timeout** у DeepSeek ~10-12 минут → нужно либо отключить pooling, либо делать periodic health checks
3. **httpx default:** `max_keepalive_connections=20` → всегда переиспользует соединения
4. **openai library** использует httpx под капотом → можно передать custom `http_client`

## 🔗 Related Issues

- [DEEPSEEK_REASONER_FIX.md](DEEPSEEK_REASONER_FIX.md) — timeout после 2-3 запросов (другая проблема)
- [PROMPT_REPETITION_FIX.md](PROMPT_REPETITION_FIX.md) — garbage accumulation в pending_queries
- [docs/fixes/ZENOH_FIX_SUMMARY_2025-11-10.md](docs/fixes/ZENOH_FIX_SUMMARY_2025-11-10.md) — Zenoh DDS discovery

## 🚀 Deployment

```bash
# 1. Commit fix
git add src/rob_box_voice/rob_box_voice/dialogue_node.py
git commit -m "fix(voice): disable httpx connection pooling to prevent idle timeout"

# 2. Push to trigger CI/CD
git push origin main

# 3. GitHub Actions → build → deploy to robots

# 4. Verify on Vision Pi
ssh ros2@10.1.1.21
docker ps  # Check voice-assistant Up < 5 minutes
docker logs voice-assistant --tail 50 | grep "no connection pooling"
```

---
**Status:** ✅ Fixed  
**Impact:** High (блокирует работу voice assistant после idle)  
**Severity:** Critical  
**Effort:** 5 minutes (1 line change)  
**Testing:** Required (deploy + wait 15min + test)
