# Отчёт статического анализа — Rob Box

**Дата:** 2026-05-15
**Scope:** 6 пакетов (rob_box_voice, rob_box_animations, rob_box_perception, rob_box_bringup, rob_box_mcp_tools, rob_box_teleop)
**Исключены:** src/ros2leds/ и src/vesc_nexus/ (git submodules)

---

## flake8

**Версия:** 4.0.1
**Параметры:** `--max-line-length=120 --extend-ignore=E203,W503`
**Итого нарушений:** 4288

### Нарушения по категории

| Код | Кол-во | Описание | Приоритет |
|-----|--------|----------|-----------|
| W293 | 3642 | Blank line contains whitespace | LOW (автоисправление) |
| F401 | 146 | Imported but unused | MEDIUM |
| E501 | 112 | Line too long (>120 chars) | MEDIUM |
| E128 | 63 | Continuation line under-indented | LOW |
| F541 | 51 | f-string missing placeholders | **HIGH** |
| F841 | 30 | Local variable assigned but never used | MEDIUM |
| E302 | 30 | Expected 2 blank lines, found 1 | LOW |
| F405 | 35 | May be undefined from star imports | MEDIUM |
| F811 | 35 | Redefinition of unused name | MEDIUM |
| E722 | 20 | Bare except (no exception type) | **HIGH** |
| W291 | 55 | Trailing whitespace | LOW |
| W292 | 15 | No newline at end of file | LOW |
| E402 | 8 | Module level import not at top | LOW |
| E305 | 7 | Expected 2 blank lines after function | LOW |
| E306 | 13 | Expected 1 blank line before nested def | LOW |
| E303 | 4 | Too many blank lines | LOW |
| F403 | 9 | Star import used | MEDIUM |
| F821 | 1 | Undefined name | **HIGH** |
| W391 | 4 | Blank line at end of file | LOW |
| E261 | 5 | At least 2 spaces before inline comment | LOW |
| E116 | 1 | Unexpected indentation (comment) | LOW |
| E127 | 1 | Continuation line over-indented | LOW |
| E228 | 1 | Missing whitespace around modulo | LOW |
| **ИТОГО** | **4288** | | |

### Топ файлов по числу нарушений

| Файл | Нарушений |
|------|-----------|
| src/rob_box_voice/scripts/silero_tts_gui.py | 196 |
| src/rob_box_perception/rob_box_perception/reflection_node.py | 167 |
| src/rob_box_perception/rob_box_perception/context_aggregator_node.py | 120 |
| src/rob_box_voice/test/unit/core/test_command_parser.py | 111 |
| src/rob_box_mcp_tools/test/test_llm_integration.py | 109 |
| src/rob_box_mcp_tools/rob_box_mcp_tools/async_executor.py | 101 |
| src/rob_box_perception/test/test_memory_integration.py | 86 |
| src/rob_box_voice/scripts/generate_phrases_deepseek.py | 83 |
| src/rob_box_voice/scripts/test_tts_voices.py | 79 |
| src/rob_box_voice/test/unit/llm/test_provider_manager.py | 75 |

> Примечание: 85% всех нарушений (3642/4288) — это W293 (whitespace в пустых строках), которые автоматически исправляются за секунды командой `autopep8 --in-place --select=W293,W291`. После автоисправления останется ~646 реальных нарушений.

### HIGH-priority нарушения (требуют внимания)

| Код | Кол-во | Почему важно |
|-----|--------|--------------|
| F541 | 51 | f-string без `{}` — строки выглядят как f-string но без подстановки; скрытые логические ошибки, не поймает runtime |
| E722 | 20 | `bare except:` глотает все исключения включая `KeyboardInterrupt`; маскирует реальные ошибки |
| F821 | 1 | `undefined name 'Command'` — потенциальный `NameError` в runtime при вызове соответствующего пути |
| F401 | 146 | Unused imports — загрязнение namespace; часть может указывать на удалённые зависимости |

---

## black

**Версия:** 26.3.1 (compiled: yes)
**Параметры:** `--check --line-length 120`
**Файлов требующих форматирования:** 159

```
Oh no! 💥 💔 💥
159 files would be reformatted, 29 files would be left unchanged.
```

Последние 7 файлов из вывода (наиболее критичные production-файлы):
- `src/rob_box_voice/rob_box_voice/dialogue_node.py`
- `src/rob_box_voice/test/unit/node/test_agent_loop.py`
- `src/rob_box_voice/test/unit/node/test_pure_methods.py`
- `src/rob_box_voice/test/unit/llm/test_tool_call_executor.py`
- `src/rob_box_voice/test/unit/llm/test_provider_manager.py`
- `src/rob_box_voice/training/train_piper.py`
- `src/rob_box_voice/training/check_system.py`

---

## isort

**Версия:** 8.0.1
**Параметры:** `--check-only --profile black`
**Файлов с неправильным порядком импортов:** 136

Пример вывода:
```
ERROR: src/rob_box_voice/setup.py Imports are incorrectly sorted and/or formatted.
ERROR: src/rob_box_voice/rob_box_voice/tts_node.py Imports are incorrectly sorted and/or formatted.
ERROR: src/rob_box_voice/rob_box_voice/audio_node.py Imports are incorrectly sorted and/or formatted.
ERROR: src/rob_box_voice/rob_box_voice/stt_node.py Imports are incorrectly sorted and/or formatted.
ERROR: src/rob_box_voice/rob_box_voice/command_node.py Imports are incorrectly sorted and/or formatted.
... (131 файлов далее)
```

---

## Сводка

| Инструмент | Нарушений / файлов | Статус |
|------------|-------------------|--------|
| flake8 | 4288 нарушений | ⚠️ Требует внимания |
| black | 159 файлов | ⚠️ Требует форматирования |
| isort | 136 файлов | ⚠️ Неправильный порядок импортов |

---

## Рекомендации

**Немедленно (HIGH priority):**
- **F541 (51):** Исправить f-strings без `{}` — это логические ошибки; строки не делают того, что выглядят
- **E722 (20):** Заменить `bare except:` на `except Exception as e:` или специфичный тип; логировать ошибку
- **F821 (1):** Исправить undefined name `Command` — найти правильный импорт (`from std_msgs.msg import String`?)

**Автоматическое исправление (LOW risk):**
```bash
# W293 + W291 (85% всех нарушений) — безопасно:
autopep8 --in-place --select=W293,W291 -r src/rob_box_voice/ src/rob_box_animations/ \
  src/rob_box_perception/ src/rob_box_bringup/ src/rob_box_mcp_tools/ src/rob_box_teleop/

# black форматирование (изменит whitespace/quotes, не логику):
python3 -m black --line-length 120 src/rob_box_voice/ ... (тщательно review перед commit!)

# isort (порядок импортов):
python3 -m isort --profile black src/rob_box_voice/ ...
```

**При следующем рефакторинге:**
- F401 (146): Очистить неиспользуемые импорты (часть может быть критична — проверить вручную)
- F811 (35): Убрать redefinition of `String` (обычно дубль импорта с разными алиасами)

---

## Конфигурационный файл (рекомендуется создать)

```ini
# setup.cfg (в корне репозитория)
[flake8]
max-line-length = 120
extend-ignore = E203,W503
exclude =
    src/ros2leds/,
    src/vesc_nexus/,
    build/,
    install/,
    log/

[isort]
profile = black
line_length = 120
```

---
*Отчёт создан: Phase 3 Milestone 1*
*Следующий запуск: перед Milestone 2 (после автофиксов W293/W291)*
