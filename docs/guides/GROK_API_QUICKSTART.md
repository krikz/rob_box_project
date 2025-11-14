# Быстрый старт: Тестирование Grok API

Краткое руководство для быстрого тестирования Grok API в проекте Rob Box.

## 🎯 Цель

Протестировать Grok API как альтернативу DeepSeek для dialogue_node и принять решение о целесообразности интеграции.

## ⏱️ Время выполнения

- **Быстрый тест:** 30 минут
- **Полное тестирование:** 1-2 недели

---

## 📋 Шаг 1: Получение API ключа (10 мин)

### 1.1 Регистрация

1. Перейдите на https://x.ai/api
2. Зарегистрируйтесь или войдите через аккаунт X (Twitter)
3. Создайте новый API ключ

### 1.2 Сохранение ключа

```bash
# Добавьте в .env.secrets
echo 'export XAI_API_KEY="xai-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"' >> /path/to/rob_box_project/docker/vision/.env.secrets
```

**⚠️ ВАЖНО:** Файл `.env.secrets` в `.gitignore` — не будет закоммичен!

---

## 🧪 Шаг 2: Запуск примеров (20 мин)

### 2.1 Установка зависимостей

```bash
# OpenAI SDK (совместим с Grok API)
pip install openai
```

### 2.2 Запуск тестового скрипта

```bash
cd /path/to/rob_box_project/docs/examples

# Загрузить API ключи
source ../../docker/vision/.env.secrets

# Запустить примеры
python3 grok_api_integration_example.py
```

### 2.3 Что тестируется

1. **Пример 1:** Базовый чат (3 простых вопроса)
2. **Пример 2:** Сравнение Grok vs DeepSeek (скорость, качество)
3. **Пример 3:** Function calling (навигационные команды)
4. **Пример 4:** Длинный контекст (1M токенов)

### 2.4 Ожидаемый результат

```
============================================================
 Примеры интеграции Grok API в Rob Box Voice Assistant
============================================================

============================================================
ПРИМЕР 1: Базовый чат с Grok API
============================================================

👤 Пользователь: Привет! Как дела?
🤖 Grok: Привет! У меня всё отлично, спасибо! Готов помочь. Как у тебя дела?

👤 Пользователь: Что ты умеешь делать?
🤖 Grok: Я умею навигировать по помещениям, строить карты, отвечать на вопросы и выполнять голосовые команды.

...
```

---

## 📊 Шаг 3: Оценка результатов (после тестирования)

### 3.1 Качество диалогов

Сравните ответы Grok vs DeepSeek:

| Критерий | DeepSeek | Grok | Победитель |
|----------|----------|------|------------|
| Понимание контекста | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ? |
| Естественность | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ? |
| Скорость ответа | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ? |
| Function calling | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ? |

### 3.2 Стоимость

После 1 недели тестирования:

```bash
# Проверьте биллинг
# xAI Dashboard: https://console.x.ai/billing
# DeepSeek Dashboard: https://platform.deepseek.com/usage

# Примерная оценка:
# - DeepSeek: $5-10/месяц
# - Grok: $???/месяц (требуется измерить!)
```

### 3.3 Критерии решения

**Переходить на Grok если:**
- ✅ Качество диалогов улучшилось **значительно** (>30%)
- ✅ Стоимость **приемлема** для проекта (<$50/месяц)
- ✅ Function calling работает **лучше** чем у DeepSeek

**Оставить DeepSeek если:**
- ❌ Улучшение качества **незначительное** (<10%)
- ❌ Стоимость **слишком высока** (>$100/месяц)
- ❌ Нет критичных новых функций

---

## 🔧 Шаг 4: Интеграция в dialogue_node (опционально)

Если результаты тестирования положительные:

### 4.1 Создать feature branch

```bash
cd /path/to/rob_box_project
git checkout develop
git pull origin develop
git checkout -b feature/grok-llm-integration
```

### 4.2 Модифицировать dialogue_node.py

```python
# В dialogue_node.py добавить поддержку Grok

def __init__(self):
    # ...
    
    # Параметр выбора провайдера
    self.declare_parameter("llm_provider", "deepseek")  # или "grok"
    
    llm_provider = self.get_parameter("llm_provider").value
    
    if llm_provider == "grok":
        base_url = "https://api.x.ai/v1"
        api_key = os.getenv("XAI_API_KEY")
        model = "grok-4"
    elif llm_provider == "deepseek":
        base_url = "https://api.deepseek.com"
        api_key = os.getenv("DEEPSEEK_API_KEY")
        model = "deepseek-chat"
    
    self.client = OpenAI(api_key=api_key, base_url=base_url)
    self.model = model
```

### 4.3 Обновить конфигурацию

```yaml
# docker/vision/config/voice_assistant.yaml

dialogue_node:
  ros__parameters:
    llm_provider: "grok"  # "deepseek" или "grok"
    api_key: ""  # Загружается из .env.secrets
```

### 4.4 Добавить секреты

```bash
# docker/vision/.env.secrets
export XAI_API_KEY="xai-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
export DEEPSEEK_API_KEY="sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
```

### 4.5 Пересобрать Docker образ

```bash
cd docker/vision/voice_assistant
docker build -t rob_box:voice-assistant-grok-test .

# Или через GitHub Actions (push в feature branch)
git add .
git commit -m "feat(voice): add Grok API integration"
git push origin feature/grok-llm-integration
```

### 4.6 Тестирование на роботе

```bash
# На Vision Pi
cd ~/rob_box_project/docker/vision
docker-compose down voice-assistant
docker-compose up -d voice-assistant

# Проверить логи
docker logs -f voice-assistant
```

---

## 📈 Метрики для A/B тестирования

Если интегрируете Grok, собирайте метрики:

### Качественные метрики

- Понимание контекста (сложные диалоги)
- Естественность ответов
- Корректность function calling
- Обработка ошибок

### Количественные метрики

```python
# Добавить в dialogue_node.py
metrics = {
    "llm_provider": "grok",  # или "deepseek"
    "response_time_ms": ...,
    "tokens_used": ...,
    "cost_usd": ...,
    "success_rate": ...
}
```

### Логирование

```python
# В dialogue_node.py
self.get_logger().info(
    f"LLM response: provider={self.llm_provider}, "
    f"time={response_time:.2f}s, tokens={tokens_used}, "
    f"cost=${cost:.4f}"
)
```

---

## 🎓 Рекомендации

### ✅ ДЕЛАЙТЕ:

1. **Тестируйте параллельно** — используйте оба API (Grok + DeepSeek)
2. **Измеряйте метрики** — собирайте данные для принятия решения
3. **A/B тестирование** — переключайте провайдера и сравнивайте
4. **Следите за бюджетом** — контролируйте расходы на API

### ❌ НЕ ДЕЛАЙТЕ:

1. **Не удаляйте DeepSeek** — оставьте как fallback
2. **Не коммитьте API ключи** — используйте `.env.secrets`
3. **Не тестируйте на production** — используйте feature branch
4. **Не принимайте решение без данных** — сначала соберите метрики

---

## 📚 Дополнительные ресурсы

### Документация проекта

- **Анализ Grok API:** `/docs/reports/GROK_VOICE_MODE_ANALYSIS.md`
- **Пример кода:** `/docs/examples/grok_api_integration_example.py`
- **Secrets Guide:** `/src/rob_box_voice/SECRETS_GUIDE.md`

### Официальная документация

- **xAI API:** https://x.ai/api
- **Grok Docs:** https://docs.x.ai/docs/guides/streaming-response
- **OpenAI SDK:** https://github.com/openai/openai-python

### Community

- **xAI Discord:** (если доступен)
- **GitHub Issues:** https://github.com/krikz/rob_box_project/issues

---

## ❓ FAQ

**Q: Нужно ли платить за Grok API?**
A: Да, это платный сервис. Точная стоимость не опубликована — требуется тестирование.

**Q: Заменит ли Grok весь voice assistant?**
A: Нет! Grok API — это только LLM. STT (Vosk/Whisper) и TTS (Yandex) остаются без изменений.

**Q: Можно ли использовать Grok offline?**
A: Нет. Grok API требует интернет-соединение. Для offline используйте локальные модели (Ollama, LLaMA).

**Q: Стоит ли переходить на Grok?**
A: Зависит от результатов тестирования! Соберите данные и примите обоснованное решение.

**Q: Поддерживает ли Grok русский язык?**
A: Да, Grok поддерживает русский язык (и многие другие).

---

## ✅ Чеклист быстрого старта

- [ ] Получен API ключ xAI
- [ ] Добавлен в `.env.secrets`
- [ ] Установлены зависимости (`pip install openai`)
- [ ] Запущены примеры (`grok_api_integration_example.py`)
- [ ] Сравнено качество ответов (Grok vs DeepSeek)
- [ ] Измерена стоимость API (1-2 недели тестирования)
- [ ] Принято решение о интеграции
- [ ] (Опционально) Интегрировано в `dialogue_node.py`
- [ ] (Опционально) Протестировано на реальном роботе

---

**Автор:** AI Agent (GitHub Copilot)  
**Дата:** 14 ноября 2025  
**Версия:** 1.0

---

**Готовы начать? Запустите примеры и посмотрите Grok в действии! 🚀**
