# System Prompts для rob_box_perception

Директория с system prompts для Internal Dialogue Agent.

## Файлы

### reflection_prompt.txt
**Назначение:** Основной system prompt для `reflection_node`

**Описание:**
- Определяет личность внутреннего голоса робота
- Правила когда говорить вслух
- Формат JSON ответа
- Мониторинг здоровья системы

**Используется по умолчанию** в `reflection_node`

## Использование

### По умолчанию
```bash
# Использует reflection_prompt.txt автоматически
ros2 run rob_box_perception reflection_node
```

### Кастомный промпт
```bash
# Создать свой промпт
echo "Ты робот-философ..." > /path/to/custom_prompt.txt

# Положить в prompts/ и пересобрать
cp custom_prompt.txt src/rob_box_perception/prompts/
colcon build --packages-select rob_box_perception

# Запустить с кастомным промптом
ros2 run rob_box_perception reflection_node \
  --ros-args -p system_prompt_file:=custom_prompt.txt
```

### В launch файле
```python
Node(
    package='rob_box_perception',
    executable='reflection_node',
    parameters=[{
        'system_prompt_file': 'reflection_prompt.txt',
        # или 'custom_prompt.txt'
    }]
)
```

### В Docker
```yaml
environment:
  SYSTEM_PROMPT_FILE: reflection_prompt.txt
```

## Формат промпта

System prompt должен содержать:

1. **Личность** - кто ты (внутренний голос робота)
2. **Задачи** - что нужно делать (анализ, рефлексия, решение)
3. **Правила** - когда говорить вслух
4. **Формат ответа** - JSON структура

### Пример:
```
Ты - внутренний голос робота РобБокс.

Твоя задача:
1. Анализировать контекст (датчики, камера, память)
2. Генерировать внутренние мысли
3. РЕШАТЬ: говорить вслух или молчать

Правила речи:
- Говори ТОЛЬКО если важно
- НЕ болтай просто так
...

Формат ответа JSON:
{
  "thought": "внутренняя мысль",
  "should_speak": true/false,
  "speech": "текст для произнесения"
}
```

## Редактирование

### При разработке (symlink)
```bash
# Редактируй напрямую (благодаря --symlink-install)
nano src/rob_box_perception/prompts/reflection_prompt.txt

# Перезапусти ноду - изменения применятся
ros2 run rob_box_perception reflection_node
```

### В Docker
Промпт встроен в образ при сборке. Для изменений:
1. Отредактировать `prompts/reflection_prompt.txt`
2. Пересобрать Docker образ
3. Передеплоить контейнер

## Fallback

Если файл не найден, используется встроенный промпт из кода:
```python
# В reflection_node.py
def _load_system_prompt(self) -> str:
    try:
        # Загрузить из файла
        ...
    except Exception as e:
        # Fallback на встроенный
        return """Ты - внутренний голос робота..."""
```

**Fallback гарантирует:** нода всегда запустится, даже если файл отсутствует.

## Best Practices

1. ✅ **Используй reflection_prompt.txt для production**
2. ✅ **Тестируй кастомные промпты локально**
3. ✅ **Версионируй промпты в Git**
4. ✅ **Документируй изменения в коммитах**
5. ⚠️ **Не удаляй fallback промпт из кода**

## Отладка

### Проверить загруженный промпт
```bash
# Запусти ноду - логи покажут размер
ros2 run rob_box_perception reflection_node

# Вывод:
# [INFO] [reflection_node]: ✅ Загружен prompt: reflection_prompt.txt (1004 байт)
```

### Проверить установленные промпты
```bash
ls -la install/rob_box_perception/share/rob_box_perception/prompts/
```

### Прочитать промпт
```bash
cat install/rob_box_perception/share/rob_box_perception/prompts/reflection_prompt.txt
```

## Сравнение с rob_box_voice

| Аспект | rob_box_voice | rob_box_perception |
|--------|---------------|-------------------|
| Файл | `master_prompt_simple.txt` | `reflection_prompt.txt` |
| Параметр | `system_prompt_file` | `system_prompt_file` |
| Env var | `SYSTEM_PROMPT_FILE` | `SYSTEM_PROMPT_FILE` |
| Fallback | ✅ Есть | ✅ Есть |
| Формат | SSML JSON | JSON (thought/speech) |

**Единый подход!** Оба пакета используют одинаковую архитектуру.

---

**Готово к использованию!** 🚀
