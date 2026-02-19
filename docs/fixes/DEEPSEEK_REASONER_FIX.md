# 🚨 КРИТИЧНО: DeepSeek автоматически переключается на Reasoner

## Проблема

С **23 января 2026** DeepSeek API начал **автоматически** использовать `deepseek-reasoner` вместо `deepseek-chat`, даже когда явно указан `model="deepseek-chat"`.

### Статистика расходов:

| Дата | deepseek-chat | deepseek-reasoner | Стоимость |
|------|---------------|-------------------|-----------|
| 1-22 янв | ✅ Только chat | ❌ Нет | ~0.0001-0.08 CNY/день |
| 23 янв | 99 req (chat) | **158 req (reasoner!)** | **6.04 CNY** (×200 рост!) |
| 24 янв | 88 req | **206 req** | **8.33 CNY** |
| 27 янв | 340 req | **876 req** | **12.16 CNY** |
| 28 янв | 2206 req | **5844 req** | **39.68 CNY + $8.45 USD** |
| 29 янв | - | - | **$2.75 USD** (за неполный день!) |

### Ключевые цифры 23 января:

**KEK_ROBBOX (тестовый ключ):**
- `deepseek-reasoner`: 139K output + 1.4M input = **6.04 CNY**

**ROBOT (ключ на роботе):**
- `deepseek-reasoner`: 112K output + 1.1M input = тоже дорого

### Причина:

DeepSeek **игнорирует** параметр `model` и **автоматически выбирает reasoner** если:
1. Промпт кажется "сложным"
2. Большой context window
3. Наличие function calling / tools (возможно)

## ✅ Решение

### Вариант 1: Явно отключить reasoner (рекомендуется)

Добавить параметр `reasoning_content="disabled"` в запросы:

```python
response = client.chat.completions.create(
    model="deepseek-chat",
    messages=messages,
    reasoning_content="disabled",  # ← ДОБАВИТЬ ЭТО!
    temperature=0.7,
    max_tokens=500
)
```

### Вариант 2: Переключиться на Qwen полностью

Qwen дешевле и стабильнее:
- Qwen: $0.05/1M input, $0.15/1M output
- DeepSeek Chat: $0.14/1M input, $0.28/1M output
- DeepSeek Reasoner: $0.55/1M input, $2.19/1M output (в 10 раз дороже!)

### Вариант 3: Использовать DeepSeek V3

Попробовать модель `deepseek-chat-v3` которая может не иметь этой проблемы.

## 📝 Где фиксить

### 1. Context Aggregator (суммаризация)
**Файл:** `src/rob_box_perception/rob_box_perception/context_aggregator_node.py:639`

```python
response = self.deepseek_client.chat.completions.create(
    model="deepseek-chat",
    messages=[{"role": "user", "content": prompt}],
    reasoning_content="disabled",  # ← ДОБАВИТЬ
    temperature=0.3,
    max_tokens=300
)
```

### 2. Reflection Node (рефлексия робота)
**Файл:** `src/rob_box_perception/rob_box_perception/reflection_node.py:~300+`

Найти вызовы `chat.completions.create` и добавить `reasoning_content="disabled"`.

### 3. Voice Assistant (диалог)
**Файл:** `src/rob_box_voice/rob_box_voice/llm/provider_manager.py`

Добавить в метод создания запроса или в wrapper функцию.

### 4. Локальные тесты
**Файл:** `local_test/hardware/test_another_key.py:32`

```python
response = client.chat.completions.create(
    model="deepseek-chat",
    messages=messages,
    reasoning_content="disabled",  # ← ДОБАВИТЬ
    temperature=0.7,
    max_tokens=500,
    stream=False
)
```

## 🔍 Проверка

После исправления:
1. Запустить локальный тест с новым ключом
2. Проверить billing dashboard DeepSeek - должен использоваться **только deepseek-chat**
3. Мониторить расходы следующие 24 часа

## 📚 Ссылки

- DeepSeek API Docs: https://api-docs.deepseek.com/
- Function Calling Guide: https://api-docs.deepseek.com/guides/function_calling
- Pricing: https://www.deepseek.com/pricing

## ⚠️ Важно

**НЕ ДЕПЛОИТЬ** на роботов до тестирования локально!

---
**Дата:** 29 января 2026  
**Статус:** 🔴 КРИТИЧНО - ФИКСИТЬ СРОЧНО  
**Потери:** ~$15+ USD за 6 дней (если продолжится - сотни долларов в месяц)
