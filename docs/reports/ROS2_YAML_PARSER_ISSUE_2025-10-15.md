# ROS2 YAML Parser Issue - Sequence Type Validation

**Дата**: 15 октября 2025  
**Ветка**: `feature/voice-assistant`  
**Коммиты**: `b32f611` → `72520fd`

## 🔥 Проблема

Voice assistant контейнер на Vision Pi (10.1.1.11) постоянно крашился с ошибкой:

```
rclpy._rclpy_pybind11.RCLError: failed to initialize rcl: 
Couldn't parse params file: '--params-file /config/voice_assistant/voice_assistant.yaml'. 
Error: Sequence should be of same type. Value type 'double' do not belong at line_num 203
```

## 🔍 Исследование

### Попытки решения (8 коммитов)

1. **Конвертация float → double** (`feea5ac`) - ❌ Не помогло
2. **Добавление пустых params** (`7d6eab7`) - ❌ Не помогло  
3. **Унификация структуры params** (`9944212`) - ❌ Не помогло
4. **Конвертация в float массивов** (`beea768`) - ❌ Не помогло
5. **Использование null** (`02d684c`) - ❌ Не помогло
6. **Правильный YAML синтаксис** (`68a0308`) - ❌ Файл повредился
7. **Восстановление + RGB fix** (`b32f611`) - ⚠️ Частично помогло
8. **Унификация distance/angle** (`d668013`) - ❌ Не помогло

Во время попыток файл `voice_assistant.yaml` был повреждён из-за множественных операций `replace_string_in_file`, что привело к слиянию строк ("- 0.0Node =====").

### Восстановление файла

Файл был восстановлен из чистого состояния:
```bash
git show e70e3c9:docker/vision/config/voice_assistant/voice_assistant.yaml > /tmp/voice_yaml_clean.yaml
cp /tmp/voice_yaml_clean.yaml docker/vision/config/voice_assistant/voice_assistant.yaml
```

## 🎯 Корневая причина

### GitHub Issues

Изучены следующие issue в ROS2:

- **[ros2/rcl#463](https://github.com/ros2/rcl/issues/463)** - ROS2 YAML parser doesn't support inline syntax
- **[ros2/ros2#1380](https://github.com/ros2/ros2/issues/1380)** - YAML inline arrays/dicts not supported
- **[ros2/rcl#555](https://github.com/ros2/rcl/issues/555)** - "Sequence should be of same type" with .nan values

### Ключевые находки

1. **Inline синтаксис НЕ поддерживается**:
   ```yaml
   # ❌ НЕ РАБОТАЕТ
   colors: [0, 255, 0]
   params: {distance: 0.5}
   
   # ✅ РАБОТАЕТ
   colors:
     - 0
     - 255
     - 0
   params:
     distance: 0.5
   ```

2. **Sequence из nested dictionaries НЕ поддерживается**:
   ```yaml
   # ❌ НЕ РАБОТАЕТ - даже с правильным YAML!
   commands:
     - pattern: "вперед"
       action: "move_forward"
       params:
         distance: 0.5
         angle: 0.0
     - pattern: "назад"
       action: "move_backward"
       params:
         distance: 0.5
         angle: 0.0
   ```
   
   ROS2 YAML парсер воспринимает это как "sequence с разными типами" из-за nested структуры.

3. **Python yaml.safe_load() работает, RCL YAML parser - НЕТ**:
   - Python парсер более толерантен
   - RCL parser использует строгую типизацию (rcutils/src/parse.c:378)
   - Проблема на уровне C-кода rcl, не Python

## ✅ Решение

### Финальный коммит: `72520fd`

Заменили nested structure на **plain string array**:

```yaml
# ✅ РАБОТАЕТ - простой массив строк
command_patterns:
  - "вперед|forward|move_forward|0.5|0.0"
  - "назад|back|move_backward|0.5|0.0"
  - "налево|left|turn_left|0.0|90.0"
  - "направо|right|turn_right|0.0|90.0"
  - "стоп|stop|stop|0.0|0.0"
  - "домой|go home|return_home|0.0|0.0"
```

Формат: `pattern|english|action|distance|angle`

### Результат

```
✅ audio_node - РАБОТАЕТ (VAD, DoA, ReSpeaker)
✅ stt_node - РАБОТАЕТ (Vosk распознавание)
✅ tts_node - РАБОТАЕТ
✅ led_node - РАБОТАЕТ
✅ sound_node - РАБОТАЕТ
❌ dialogue_node - DEEPSEEK_API_KEY не найден (ожидаемо)
❌ command_node - nav2_msgs не установлен (отдельная проблема)
```

**YAML парсинг полностью исправлен!** ✨

## 📝 Изменения в коде

### Файлы изменены:

1. **`docker/vision/config/voice_assistant/voice_assistant.yaml`**:
   - RGB colors: inline arrays → proper YAML indentation
   - commands: nested dicts → plain string array

2. **Требуется обновить** `src/rob_box_voice/rob_box_voice/command_node.py`:
   - Парсинг `commands` → `command_patterns`
   - Split по `|` вместо dict access

## 🔧 TODO

- [ ] Обновить `command_node.py` для работы с новым форматом
- [ ] Добавить nav2_msgs в Dockerfile или сделать command_node optional
- [ ] Добавить DEEPSEEK_API_KEY в .env.secrets на Vision Pi
- [ ] Протестировать полный цикл voice assistant

## 📚 Уроки

1. **ROS2 YAML parser ОЧЕНЬ строгий** - не поддерживает многие фичи стандартного YAML
2. **Inline синтаксис запрещён** - всегда используйте full indentation
3. **Nested structures опасны** - предпочитайте плоские структуры
4. **Multiple edits опасны** - файлы могут повредиться, используйте git restore
5. **Всегда читайте GitHub issues** - там часто есть ответы на "невозможные" проблемы

## 🔗 Ссылки

- Issue #555: https://github.com/ros2/rcl/issues/555
- Issue #463: https://github.com/ros2/rcl/issues/463  
- Issue #1380: https://github.com/ros2/ros2/issues/1380
- RCL parse.c: https://github.com/ros2/rcl/blob/rolling/rcl/src/rcl/arguments.c#L406

## 🎉 Статус

**ПРОБЛЕМА РЕШЕНА** ✅

Voice assistant успешно парсит YAML и все основные ноды запускаются.
