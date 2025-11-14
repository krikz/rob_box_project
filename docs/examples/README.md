# Примеры кода Rob Box Project

Эта директория содержит примеры кода для различных компонентов проекта Rob Box.

## 📂 Содержание

### Интеграции LLM

- **`grok_api_integration_example.py`** — Пример интеграции xAI Grok API
  - Базовый чат с Grok
  - Streaming ответов
  - Function calling для управления роботом
  - Сравнение с DeepSeek API
  - Использование длинного контекста (1M токенов)

## 🚀 Использование примеров

### Предварительные требования

```bash
# Установить зависимости
pip install openai

# Установить API ключи
export XAI_API_KEY="xai-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
export DEEPSEEK_API_KEY="sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
```

### Запуск примеров

```bash
# Grok API интеграция
cd /path/to/rob_box_project/docs/examples
python3 grok_api_integration_example.py
```

## 📖 Связанная документация

- **Анализ Grok API:** `/docs/reports/GROK_VOICE_MODE_ANALYSIS.md`
- **Текущий голосовой ассистент:** `/src/rob_box_voice/README.md`
- **Dialogue Node:** `/src/rob_box_voice/rob_box_voice/dialogue_node.py`

## ⚠️ Важно

Эти примеры предназначены **только для тестирования и обучения**.

Для реальной интеграции в проект требуется:
1. Модификация `dialogue_node.py`
2. Обновление конфигурации `voice_assistant.yaml`
3. Добавление секретов в `.env.secrets`
4. Тестирование на реальном роботе

## 📝 Добавление новых примеров

При добавлении новых примеров:

1. Создайте описательное имя файла: `<component>_<feature>_example.py`
2. Добавьте docstring с описанием
3. Включите примеры использования
4. Обновите этот README
5. Добавьте ссылки на связанную документацию

---

**Последнее обновление:** 14 ноября 2025
