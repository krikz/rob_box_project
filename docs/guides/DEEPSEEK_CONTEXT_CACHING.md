# DeepSeek Context Caching - Подробный разбор для Rob Box

## 📋 Что такое Context Caching on Disk?

**Context Caching on Disk** - технология DeepSeek, которая кэширует повторяющиеся части промптов на распределённом дисковом массиве. При обнаружении дубликатов входных данных, повторяющиеся части извлекаются из кэша, минуя необходимость повторных вычислений.

### 🎯 Ключевые характеристики:

1. **Автоматическое включение** - работает для всех пользователей, никаких изменений кода не требуется
2. **Изоляция данных** - кэш каждого пользователя изолирован и логически невидим для других
3. **Автоматическая очистка** - неиспользуемые записи удаляются через несколько часов/дней
4. **Минимальный размер** - кэширует только блоки >= 64 токенов
5. **Best-effort** - не гарантирует 100% попаданий в кэш

---

## 💰 Стоимость и экономия

### Цены (DeepSeek V3.2):

| Тип токенов | Цена (¥/1M токенов) | Цена ($/1M токенов) |
|-------------|---------------------|---------------------|
| **Cache Hit** (из кэша) | ¥0.1 | **$0.014** |
| **Cache Miss** (новые) | ¥1.0 | **$0.14** |
| **Output** (генерация) | ¥2.0 | **$0.28** |

### 📊 Экономия:

- **До 90%** при оптимизации под кэш (все промпты с одинаковым префиксом)
- **~50%** в среднем без оптимизации (по статистике DeepSeek)
- **Cache Hit в 10x дешевле** Cache Miss ($0.014 vs $0.14)

---

## 🔄 Как работает кэширование

### Принцип работы:

```
Запрос 1:
[System Prompt] + [User Message 1] → Создание кэша
                                    ↓
                                  Cache построен

Запрос 2:
[System Prompt] + [User Message 2]
       ↓
[System Prompt] - из КЭША (cache hit) ✅
[User Message 2] - новый (cache miss) ❌
```

### ⚠️ ВАЖНО: Кэш работает только с **идентичными префиксами**

**Условия Cache Hit:**
- ✅ Префикс должен быть **точно таким же** (с 0-го токена)
- ✅ Минимум **64 токена** в префиксе
- ✅ Последовательность сообщений **не изменилась**

**НЕ работает:**
- ❌ Частичные совпадения в середине промпта
- ❌ Измененный порядок сообщений
- ❌ Даже 1 символ отличия в префиксе

---

## 🎭 Примеры использования

### Пример 1: Мультираундовый диалог (Multi-turn conversation)

**Запрос 1:**
```python
messages = [
    {"role": "system", "content": "You are a helpful assistant"},
    {"role": "user", "content": "Привет робот"}
]
# Cache: создаётся для system + user
# Tokens: 0 hit, 20 miss
```

**Запрос 2:**
```python
messages = [
    {"role": "system", "content": "You are a helpful assistant"},
    {"role": "user", "content": "Привет робот"},
    {"role": "assistant", "content": "Привет! Чем помочь?"},
    {"role": "user", "content": "Какая температура процессора?"}
]
# Cache: system + первый user + assistant - ИЗ КЭША ✅
# Tokens: 40 hit, 15 miss
# Экономия: 40 * $0.014 вместо 40 * $0.14 = $0.00056 vs $0.0056 (10x дешевле!)
```

### Пример 2: Длинный System Prompt (ваш случай!)

**Rob Box использует большой system prompt** (~500 токенов):

```python
SYSTEM_PROMPT = """Ты — голосовой помощник робота Rob Box. 
Твоя задача: отвечать кратко, по делу, естественным разговорным языком.

ПРАВИЛА:
1. Ответы должны быть SHORT (1-2 фразы максимум для простых вопросов)
2. Используй разговорный стиль, без формальностей
...
(ещё ~450 токенов правил и примеров)
"""
```

**При каждом запросе:**
- System prompt **500 токенов** → **всегда из кэша** после первого запроса ✅
- Cache Hit: 500 * $0.014 = **$0.007 за запрос**
- Cache Miss: 500 * $0.14 = **$0.070 за запрос**
- **Экономия 90%** на system prompt!

### Пример 3: Контекст с датчиками (будущее использование)

```python
# Промпт с данными датчиков (обновляется каждую секунду)
messages = [
    {"role": "system", "content": LONG_SYSTEM_PROMPT},  # 500 токенов
    {"role": "user", "content": f"""
Текущее состояние робота:
- Батарея: {battery}%
- Температура CPU: {cpu_temp}°C
- Позиция: x={x}, y={y}, θ={theta}
- Скорость: {speed} м/с
...
Пользователь спросил: {user_question}
"""}
]
```

**Результат:**
- System prompt (500 токенов) → **Cache Hit** ✅
- Состояние робота (~200 токенов) → **Cache Miss** ❌ (данные меняются)
- Вопрос пользователя (~50 токенов) → **Cache Miss** ❌

**Экономия:** 500/(500+200+50) = **66% токенов из кэша!**

---

## 📈 Мониторинг Cache Hits

### В ответе API добавлены поля:

```python
response = client.chat.completions.create(...)

# В response.usage:
usage = {
    "prompt_tokens": 750,              # Всего входных токенов
    "completion_tokens": 120,          # Токенов в ответе
    "total_tokens": 870,               # Общее количество
    
    # 🆕 Новые поля для кэша:
    "prompt_cache_hit_tokens": 500,    # Взято из кэша ($0.014/1M)
    "prompt_cache_miss_tokens": 250,   # Новые токены ($0.14/1M)
}

# Проверка эффективности кэша:
cache_efficiency = 500 / (500 + 250) = 66.7%
```

### Пример логирования:

```python
def log_cache_stats(response):
    usage = response.usage
    hit = usage.prompt_cache_hit_tokens
    miss = usage.prompt_cache_miss_tokens
    total = hit + miss
    
    efficiency = (hit / total * 100) if total > 0 else 0
    cost_saved = hit * (0.14 - 0.014) / 1_000_000  # Экономия в $
    
    self.get_logger().info(
        f"💾 Cache: {hit}/{total} tokens ({efficiency:.1f}%), "
        f"saved ${cost_saved:.6f}"
    )
```

---

## 🚀 Оптимизация для Rob Box

### Текущая ситуация (dialogue_node.py):

```python
# Каждый запрос:
messages = [
    {"role": "system", "content": SYSTEM_PROMPT},  # ~500 токенов
    *self.conversation_history,  # История диалога
    {"role": "user", "content": user_input}  # Новый вопрос
]
```

**Как работает кэш:**

| Запрос | System Prompt | История | User Input | Cache Hit % |
|--------|---------------|---------|------------|-------------|
| 1 | ❌ Miss (500) | - | ❌ Miss (50) | 0% |
| 2 | ✅ Hit (500) | ❌ Miss (200) | ❌ Miss (40) | 67% |
| 3 | ✅ Hit (500) | ✅ Hit (200+100) | ❌ Miss (35) | 89% |
| 4 | ✅ Hit (500) | ✅ Hit (200+100+80) | ❌ Miss (60) | 88% |

### 💡 Рекомендации по оптимизации:

1. **Используйте стабильный system prompt**
   - ✅ Не изменяйте system prompt между запросами
   - ✅ Загружайте его один раз при инициализации
   - ❌ Не добавляйте динамические данные в system prompt

2. **Храните историю диалога**
   - ✅ Переиспользуйте предыдущие сообщения
   - ✅ Cache Hit на всю историю (кроме последнего вопроса)

3. **Мониторьте эффективность**
   - Добавьте логирование `prompt_cache_hit_tokens`
   - Отслеживайте cache efficiency %
   - Настройте алерты при падении ниже 50%

4. **Длина контекста**
   - Минимум 64 токена для кэширования
   - System prompt ~500 токенов → отлично кэшируется
   - История диалога растёт → больше cache hits

---

## 🔧 Реализация мониторинга для Rob Box

### Добавление в dialogue_node.py:

```python
def _handle_deepseek_response(self, response, user_input: str):
    """Обработка ответа от DeepSeek с мониторингом кэша"""
    
    # Логируем статистику кэша
    if hasattr(response, 'usage'):
        usage = response.usage
        
        total_prompt = usage.prompt_tokens
        cache_hit = getattr(usage, 'prompt_cache_hit_tokens', 0)
        cache_miss = getattr(usage, 'prompt_cache_miss_tokens', total_prompt)
        
        if total_prompt > 0:
            cache_efficiency = (cache_hit / total_prompt) * 100
            cost_saved = cache_hit * (0.14 - 0.014) / 1_000_000
            
            self.get_logger().info(
                f"💾 Cache: {cache_hit}/{total_prompt} tokens "
                f"({cache_efficiency:.1f}% hit), "
                f"saved ${cost_saved:.6f}"
            )
            
            # Алерт если кэш не работает
            if cache_hit == 0 and total_prompt > 100:
                self.get_logger().warning(
                    "⚠️ Cache Miss на весь промпт! Проверьте system prompt."
                )
    
    # Остальная обработка...
```

---

## 📊 Производительность

### Latency (задержка первого токена):

| Размер промпта | БЕЗ кэша | С кэшем | Ускорение |
|----------------|----------|---------|-----------|
| 1K токенов | ~500ms | ~200ms | 2.5x |
| 16K токенов | ~3s | ~400ms | 7.5x |
| 128K токенов | ~13s | ~500ms | **26x** |

### Для Rob Box (типичный запрос):

- System prompt: 500 токенов
- История: 0-2000 токенов (зависит от длины диалога)
- User input: 20-100 токенов

**Первый запрос (холодный старт):**
- Latency: ~500-800ms
- Cache: 0% hit

**Последующие запросы (тёплый кэш):**
- Latency: ~200-300ms (ускорение 2-3x)
- Cache: 60-90% hit

---

## 🎯 Выводы для Rob Box Voice Assistant

### ✅ Преимущества Context Caching:

1. **Автоматическая экономия 50-90%** на повторяющихся промптах
2. **Ускорение 2-3x** для последующих запросов
3. **Нулевые изменения кода** - работает out-of-the-box
4. **Безопасность** - изолированный кэш на пользователя

### 🎁 Бонусы для вашего робота:

- **Длинный system prompt** (~500 токенов) → всегда кэшируется
- **История диалога** → кэшируется всё, кроме последнего вопроса
- **Non-streaming режим** → полная совместимость с кэшем
- **Мультираундовые диалоги** → идеальный use case для кэша

### 💡 Практический эффект:

**Сценарий:** 100 запросов к Rob Box за день

**БЕЗ оптимизации:**
- Average: 50% cache hit (по статистике DeepSeek)
- Cost: (500 hit * $0.014 + 500 miss * $0.14) * 100 = **$7.70**

**С оптимизацией** (стабильный system prompt + история):
- Average: 80% cache hit
- Cost: (800 hit * $0.014 + 200 miss * $0.14) * 100 = **$3.92**

**Экономия: $3.78 в день = ~$115 в месяц** 💰

---

## 📚 Дополнительные материалы

- [DeepSeek Context Caching Announcement](https://api-docs.deepseek.com/news/news0802)
- [Official Cache Guide](https://api-docs.deepseek.com/guides/kv_cache)
- [DeepSeek V3.2 Tech Report](https://huggingface.co/deepseek-ai/DeepSeek-V3.2/resolve/main/assets/paper.pdf)

---

**Дата создания:** 20 декабря 2025  
**Автор:** AI Agent (GitHub Copilot)  
**Робот:** RBXU100001 (Rob Box)
