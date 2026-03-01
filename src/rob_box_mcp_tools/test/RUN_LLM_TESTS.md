# 🤖 Запуск тестов с реальными LLM

## Созданные тесты

В файле `test_llm_integration.py` создано **7 тестов** с реальными LLM API:

1. ✅ `test_llm_can_call_single_tool` - LLM вызывает один инструмент
2. ✅ `test_llm_can_call_multiple_tools` - LLM вызывает несколько инструментов
3. ✅ `test_llm_tool_call_with_validation` - Валидация параметров от LLM
4. ✅ `test_llm_understands_russian_commands` - Понимание русских команд
5. ✅ `test_llm_handles_tool_result_feedback` - Полный цикл с результатом
6. ✅ `test_llm_refuses_unavailable_tool` - Отказ от недоступных инструментов
7. ✅ `test_llm_response_time` - Измерение производительности

## Подготовка

### 1. Установите API ключ

**DeepSeek (рекомендуется):**
```bash
export DEEPSEEK_API_KEY="sk-your-key-here"
```

**Qwen:**
```bash
export QWEN_API_KEY="sk-your-key-here"
```

**OpenAI:**
```bash
export OPENAI_API_KEY="sk-your-key-here"
```

**Или универсальный:**
```bash
export LLM_API_KEY="your-key-here"
```

### 2. Установите зависимости

```bash
pip install openai
```

## Запуск тестов

### Все LLM тесты

```bash
cd /home/ros2/rob_box_project/src/rob_box_mcp_tools
pytest test/test_llm_integration.py -m llm_api -v -s
```

### Один конкретный тест

```bash
# Простой тест с одним tool call
pytest test/test_llm_integration.py::TestRealLLMIntegration::test_llm_can_call_single_tool -v -s

# Полный цикл с результатом
pytest test/test_llm_integration.py::TestRealLLMIntegration::test_llm_handles_tool_result_feedback -v -s

# Тест производительности
pytest test/test_llm_integration.py::test_llm_response_time -v -s
```

### Без медленных тестов

```bash
pytest test/test_llm_integration.py -m "llm_api and not slow" -v -s
```

## Что проверяют тесты

### 1. Базовый tool call
```python
Пользователь: "Покажи анимацию радости"
LLM → вызывает: play_animation(animation_name="happy")
✅ Проверка: LLM правильно понял и вызвал инструмент
```

### 2. Множественные tool calls
```python
Пользователь: "Покажи анимацию радости и поверни направо"
LLM → вызывает: play_animation() и move_direction()
✅ Проверка: LLM может вызвать несколько инструментов
```

### 3. Полный цикл
```python
1. Пользователь → "Покажи анимацию"
2. LLM → вызывает play_animation()
3. Tool → возвращает результат
4. LLM → получает результат и формирует ответ пользователю
✅ Проверка: Весь workflow работает end-to-end
```

### 4. Русские команды
```python
"Покажи анимацию радости" → play_animation(animation="happy")
"Покажи грустную анимацию" → play_animation(animation="sad")
✅ Проверка: LLM понимает русский язык
```

## Примеры вывода

### Успешный тест:
```
🤖 Тестируем с DeepSeek (deepseek-chat)
🛠️  Зарегистрировано 1 инструментов
📤 Отправляем запрос: 'Покажи анимацию радости'
📥 Получен ответ от DeepSeek
✅ LLM вызвал инструмент: play_animation
   Параметры: {'animation_name': 'happy'}
PASSED
```

### Если нет API ключа:
```
SKIPPED [1] No LLM API key found
```

## Стоимость тестов

**⚠️ ВНИМАНИЕ:** Эти тесты делают реальные запросы к API!

- **DeepSeek:** ~$0.001 за тест (очень дешево)
- **Qwen:** Зависит от тарифа
- **OpenAI GPT-4:** ~$0.01-0.03 за тест

**Примерная стоимость всех 7 тестов:**
- DeepSeek: ~$0.007 (меньше цента)
- OpenAI GPT-4o-mini: ~$0.07

## Troubleshooting

### ImportError: No module named 'openai'
```bash
pip install openai
```

### Тесты пропускаются (SKIPPED)
```bash
# Проверьте что API ключ установлен
echo $DEEPSEEK_API_KEY

# Если пусто - установите:
export DEEPSEEK_API_KEY="your-key"
```

### Тест падает с ошибкой API
```bash
# Проверьте корректность ключа
curl https://api.deepseek.com/v1/models \
  -H "Authorization: Bearer $DEEPSEEK_API_KEY"
```

### Timeout ошибки
```bash
# Увеличьте timeout в тесте или проверьте сеть
ping api.deepseek.com
```

## CI/CD

Эти тесты **НЕ** запускаются автоматически в CI/CD (требуют API ключ).

Для запуска в CI добавьте секрет:
```yaml
- name: Run LLM tests
  env:
    DEEPSEEK_API_KEY: ${{ secrets.DEEPSEEK_API_KEY }}
  run: pytest test/test_llm_integration.py -m llm_api
```

## Разработка новых тестов

Шаблон нового теста:
```python
@pytest.mark.llm_api
@pytest.mark.slow
def test_my_new_llm_test(skip_if_no_llm_api, mock_node):
    """Описание теста"""
    client, model, provider = get_llm_client_and_model()
    
    # Настройка инструментов
    registry = MCPToolRegistry()
    registry.register(MyTool(mock_node))
    tools = registry.get_openai_tools()
    
    # Запрос к LLM
    response = client.chat.completions.create(
        model=model,
        messages=[{"role": "user", "content": "..."}],
        tools=tools,
    )
    
    # Проверки
    assert response.choices[0].message.tool_calls is not None
```

## Полезные команды

```bash
# Только быстрые тесты
pytest test/test_llm_integration.py -m "llm_api and not slow"

# С подробным выводом
pytest test/test_llm_integration.py -vv -s

# Остановиться на первой ошибке
pytest test/test_llm_integration.py -x

# С coverage
pytest test/test_llm_integration.py --cov=rob_box_mcp_tools

# Запустить N раз
pytest test/test_llm_integration.py --count=3
```
