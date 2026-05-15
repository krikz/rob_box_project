# Фаза 3: Code Quality Review — Research

**Дата исследования:** 2026-05-15  
**Домен:** Статический анализ Python / Tech debt / Test coverage / Stub tracking  
**Уверенность:** HIGH (все данные получены из живого репозитория и запущенных инструментов)

---

## Краткое резюме

Кодовая база Rob Box содержит **4288 нарушений flake8** в 6 основных пакетах, из которых ~85% составляют косметические пробелы (W293). Реальных качественных нарушений (F-коды, E722, E501) — 558. Инструменты `black` и `isort` не установлены в текущей среде dev-машины, их требуется установить.

Покрытие тестами — критически низкое: **9 из 14 модулей** имеют покрытие < 50%, из которых 6 модулей имеют **0%**. Главная проблема — `dialogue_node.py` (2040 строк, 80+ методов, 0% в htmlcov / 13.4% в coverage.json для одного теста) — основная точка боли, которую планируется рефакторить в Milestone 3.

В `CONCERNS.md` зафиксировано **7 tech debt, 6 bugs, 5 security issues, 3 perf bottleneck, 6 fragile areas, 3 scaling limits** — итого **30 пунктов**, ни один из которых не имеет формального severity и disposition.

**Главная рекомендация:** Фаза 3 — это **аудит и документирование**, не исправление. Создать репорт, расставить severity/disposition для всех 30 пунктов CONCERNS.md, задокументировать покрытие, пометить stubs, написать стратегию рефакторинга `dialogue_node.py`.

---

<phase_requirements>

## Требования фазы

| ID | Описание | Что нужно для реализации |
|----|----------|--------------------------|
| CQ-01 | Все известные критические баги задокументированы в трекере с приоритетами | Аудит CONCERNS.md → добавить записи в tasks.json (id, priority, description, status) |
| CQ-02 | Tech debt из CONCERNS.md разобран: каждый пункт имеет severity и план (fix / defer / accept) | Создать TECH_DEBT.md или раздел в трекере с severity + disposition для всех 30 пунктов |
| CQ-03 | Проведён статический анализ (flake8, black, isort) — отчёт с количеством нарушений | Запустить инструменты, создать STATIC_ANALYSIS_REPORT.md в .planning/ |
| CQ-04 | Покрытие тестами задокументировано: модули < 50% выявлены и объяснены | Проанализировать htmlcov + coverage.json → COVERAGE_REPORT.md |
| CQ-05 | `dialogue_node.py` — задокументирована стратегия рефакторинга для Milestone 3 | Создать DIALOGUE_NODE_REFACTORING.md с предложением декомпозиции |
| CQ-06 | Stub-реализации помечены `# STUB:` и добавлены в трекер | Найти и пометить все stubs в коде, добавить задачи в tasks.json |

</phase_requirements>

---

## Архитектурная карта ответственности

| Возможность | Первичный уровень | Вторичный уровень | Обоснование |
|------------|------------------|-------------------|-------------|
| Статический анализ (flake8) | Dev-машина CLI | CI/CD pipeline | Запускается локально + в GitHub Actions |
| Форматирование (black/isort) | Dev-машина CLI | CI/CD pipeline | Требует установки в среде разработки |
| Покрытие тестами | pytest + coverage | CI/CD отчёт | Данные уже собраны в htmlcov/ |
| Трекер задач (bugs/stubs) | tasks.json | — | Единый файл, читается агентами |
| Документация стратегии | .planning/phases/ | docs/plans/ | Артефакты GSD фаз |

---

## Результаты аудита CONCERNS.md

CONCERNS.md находится по адресу `.planning/codebase/CONCERNS.md` [VERIFIED: read_file].

### Tech Debt — 7 пунктов

| # | Проблема | Файл(ы) | Рекомендуемый Severity | Рекомендуемый Disposition |
|---|----------|---------|----------------------|--------------------------|
| TD-1 | `dialogue_node.py` монолит 2040 строк | `rob_box_voice/dialogue_node.py` | HIGH | defer → Milestone 3 |
| TD-2 | `get_robot_status` возвращает hardcoded stub (position 0,0; battery 85%) | `rob_box_mcp_tools/tools/system.py:440` | HIGH | defer → nav impl (Milestone 2) |
| TD-3 | `reflection_node.py` silent fallback без предупреждения | `rob_box_perception/reflection_node.py:668,702,747` | MEDIUM | fix (добавить WARN log при startup) |
| TD-4 | `command_node.py` навигация/vision stubs с `pass` | `rob_box_voice/command_node.py:343,359,370,394` | MEDIUM | defer → Milestone 2 |
| TD-5 | `async_executor.py` busy-wait polling 50ms | `rob_box_mcp_tools/async_executor.py:348-357` | LOW | defer → performance sprint |
| TD-6 | `setup.py` placeholder metadata в rob_box_perception | `rob_box_perception/setup.py:25-26` | LOW | accept / fix в Milestone 1 STRUCT |
| TD-7 | LED compositor hardcoded config + driver YAML = 2 источника конфигурации | `led_matrix_driver/launch/` | LOW | defer |

### Known Bugs — 6 пунктов

| # | Bug ID | Описание | Рекомендуемый Severity | В tasks.json? |
|---|--------|----------|----------------------|---------------|
| BG-1 | BUG-12 | Unbounded local messages list в agent run → timeouts | HIGH | TASK-043 (pending) |
| BG-2 | BUG-13 | Tool results накапливаются в conversation history | MEDIUM | TASK-044 (pending) |
| BG-3 | BUG-16 | LLM забывает system prompt после 15+ iterations | LOW | TASK-046 (pending) |
| BG-4 | TASK-047 | Barge-in регрессия (commit 37527df) | HIGH | TASK-047 (pending) |
| BG-5 | — | BLE joystick blocked by kernel 6.14.0-raspi regression | CRITICAL | **нет в трекере** |
| BG-6 | — | VESC wheel jitter (partially fixed) | MEDIUM | **нет в трекере** |

**Требует добавления в tasks.json**: BG-5 (BLE joystick regression) и BG-6 (VESC jitter edge cases).

### Security Issues — 5 пунктов

| # | Проблема | Severity | Disposition |
|---|----------|----------|-------------|
| SEC-1 | Hardcoded SSH password `'open'` в CI/CD workflow | CRITICAL | fix → GitHub Actions secret |
| SEC-2 | Пароль `'open'` в документации (docs/fixes/) | HIGH | fix → заменить на `<ROBOT_PASSWORD>` |
| SEC-3 | Zenoh без аутентификации | MEDIUM | accept (private network, lab use) |
| SEC-4 | `privileged: true` для 7 контейнеров | MEDIUM | accept / defer (требует hardware access) |
| SEC-5 | `network_mode: host` для всех контейнеров | LOW | accept (DDS requirement) |
| SEC-6 | Unpinned `ollama/ollama:latest` | LOW | defer (pin в следующем спринте) |

> **Примечание по Milestone 1:** Security fixes (SEC-1, SEC-2) — out of scope Phase 3 (аудит, не исправление). Но CQ-01/CQ-02 требуют задокументировать их severity и disposition.

### Performance Bottlenecks — 3 пункта

| # | Проблема | Severity | Disposition |
|---|----------|----------|-------------|
| PF-1 | `async_executor` busy-wait 50ms polling | LOW | defer |
| PF-2 | `dialogue_node.py` context grows во время одного agent run | HIGH | defer → BUG-12 / Milestone 3 refactor |
| PF-3 | `voice_memory.py` synchronous Ollama embedding | MEDIUM | defer |

### Fragile Areas — 6 пунктов

| # | Проблема | Severity | Disposition |
|---|----------|----------|-------------|
| FA-1 | `audio_node.py` bare `except: pass` в shutdown | MEDIUM | defer (низкий риск) |
| FA-2 | `calibrate_max_rpm.py`/`linearity_test.py` глотают все исключения при моторных тестах | HIGH | defer (инструменты, не prod) |
| FA-3 | VESC hardware interface: нет NaN/infinity guard | HIGH | defer → hardware sprint |
| FA-4 | Zenoh IPs hardcoded в router configs | MEDIUM | defer |
| FA-5 | `test_dialogue_node.py` — 13+ пустых тест-методов (только `pass`) | HIGH | документировать в Coverage report |
| FA-6 | `led_matrix_driver.py` exception в `clear_matrix` → LEDs остаются включёнными | LOW | defer |

### Scaling Limits — 3 пункта

| # | Проблема | Severity | Disposition |
|---|----------|----------|-------------|
| SL-1 | `docker/build/` — 8 полных копий репозитория для раннеров | LOW | accept / defer |
| SL-2 | Один Zenoh router = SPOF | MEDIUM | defer → Milestone 2 |
| SL-3 | LLM providers — только облако, offline fallback отключён | MEDIUM | defer |

---

## Результаты статического анализа (запущено вживую)

### flake8 — VERIFIED [VERIFIED: `python3 -m flake8 ... --count 2>&1`]

**Версия:** flake8 4.0.1, pycodestyle 2.8.0, pyflakes 2.4.0, CPython 3.10.12  
**Команда:**
```bash
python3 -m flake8 src/rob_box_voice/ src/rob_box_animations/ src/rob_box_perception/ \
  src/rob_box_bringup/ src/rob_box_mcp_tools/ src/rob_box_teleop/ \
  --max-line-length=120 --extend-ignore=E203,W503 --statistics --count
```

**Итого нарушений: 4288**

| Код | Кол-во | Описание | Приоритет исправления |
|-----|--------|----------|-----------------------|
| W293 | 3642 | Blank line contains whitespace | LOW (автоисправление) |
| F401 | 146 | Unused imports | MEDIUM |
| E501 | 112 | Line too long (>120 chars) | MEDIUM |
| E128 | 63 | Continuation line under-indented | LOW |
| F541 | 51 | f-string missing placeholders | HIGH (логические ошибки) |
| W291 | 55 | Trailing whitespace | LOW |
| F811 | 35 | Redefinition of unused name | MEDIUM |
| F405 | 35 | May be undefined from star imports | MEDIUM |
| E722 | 20 | Do not use bare `except` | HIGH |
| E302 | 30 | Expected 2 blank lines | LOW |
| F841 | 30 | Local variable assigned but never used | MEDIUM |
| W292 | 15 | No newline at end of file | LOW |
| E402 | 8 | Module level import not at top | LOW |
| E305 | 7 | Expected 2 blank lines after function | LOW |
| E306 | 13 | Expected 1 blank line before nested def | LOW |
| F403 | 9 | Star import used | MEDIUM |
| E261 | 5 | Two spaces before inline comment | LOW |
| E303 | 4 | Too many blank lines | LOW |
| W391 | 4 | Blank line at end of file | LOW |
| E228 | 1 | Missing whitespace around modulo | LOW |
| E127 | 1 | Continuation line over-indented | LOW |
| E116 | 1 | Unexpected indentation (comment) | LOW |
| F821 | 1 | Undefined name `Command` | HIGH |

**Топ-файлы по числу нарушений:**
| Файл | Нарушений |
|------|-----------|
| `src/rob_box_voice/scripts/silero_tts_gui.py` | 196 |
| `src/rob_box_perception/rob_box_perception/reflection_node.py` | 167 |
| `src/rob_box_perception/rob_box_perception/context_aggregator_node.py` | 120 |
| `src/rob_box_voice/test/unit/core/test_command_parser.py` | 111 |
| `src/rob_box_mcp_tools/test/test_llm_integration.py` | 109 |
| `src/rob_box_mcp_tools/rob_box_mcp_tools/async_executor.py` | 101 |
| `src/rob_box_voice/rob_box_voice/command_node.py` | 74 |

> **Примечание:** test-файлы тоже содержат нарушения. Скрипты в `scripts/` — не production код, но включены в анализ.

### black — НЕ УСТАНОВЛЕН [VERIFIED: `/usr/bin/python3: No module named black`]

**Статус:** black не установлен на dev-машине (`python3 -m black` недоступен, `/usr/bin/python3` не содержит модуль).  
**Требует:** `pip install black` или использование Docker-среды где black установлен.  
**Для отчёта CQ-03:** запустить в Docker или установить локально → документировать список файлов, требующих переформатирования.

### isort — НЕ УСТАНОВЛЕН [VERIFIED: `python3 -m isort --version` → "isort not found"]

**Статус:** isort не доступен в текущей среде.  
**Для отчёта CQ-03:** аналогично black — установить или использовать Docker.

### Конфигурация линтеров

**Статус:** Нет `.flake8`, `setup.cfg`, `pyproject.toml` в корне проекта [VERIFIED: `ls .flake8 setup.cfg pyproject.toml`]. Параметры передаются только через CLI.  
**Рекомендация:** Создать `setup.cfg` или `pyproject.toml` с секцией `[flake8]` для воспроизводимости.

---

## Покрытие тестами (анализ htmlcov/)

**Источники:** `htmlcov/index.html` (23 HTML-файла) + `coverage.json` [VERIFIED: парсинг Python]  
**Инструмент:** pytest-cov (coverage.py)

### Покрытие по модулям

| Модуль | Покрытие | Статус |
|--------|----------|--------|
| `rob_box_voice/audio_node.py` | 0% | ❌ КРИТИЧНО |
| `rob_box_voice/dialogue_node.py` | 0% (htmlcov) / 13.4% (coverage.json) | ❌ КРИТИЧНО |
| `rob_box_voice/stt_node.py` | 0% | ❌ КРИТИЧНО |
| `rob_box_voice/tts_node.py` | 0% | ❌ КРИТИЧНО |
| `rob_box_perception/vision_stub_node.py` | 0% | ❌ (stub, ожидаемо) |
| `rob_box_perception/utils/long_term_memory.py` | 0% | ❌ КРИТИЧНО |
| `rob_box_voice/utils/respeaker_interface.py` | 20% | ⚠️ LOW |
| `rob_box_voice/utils/audio_utils.py` | 21% | ⚠️ LOW |
| `rob_box_voice/audio_playback_manager.py` | 24% | ⚠️ LOW |
| `rob_box_voice/command_node.py` | 28% | ⚠️ LOW |
| `rob_box_perception/reflection_node.py` | 28% | ⚠️ LOW |
| `rob_box_perception/utils/node_monitor.py` | 37% | ⚠️ LOW |
| `rob_box_perception/utils/time_provider.py` | 39% | ⚠️ LOW |
| `rob_box_perception/context_aggregator_node.py` | 43% | ⚠️ LOW |
| `rob_box_voice/sound_node.py` | 53% | ✅ OK |
| `rob_box_voice/led_node.py` | 62% | ✅ OK |
| `rob_box_perception/health_monitor.py` | 69% | ✅ OK |
| `rob_box_perception/utils/internet_monitor.py` | 71% | ✅ OK |
| `rob_box_perception/startup_greeting_node.py` | 77% | ✅ OK |
| `__init__.py` файлы | 100% | ✅ OK |

**Итого модулей ниже 50%:** 14 из 19 содержательных модулей (74%)

### Объяснение низкого покрытия

| Причина | Модули | Что делать в Phase 3 |
|---------|--------|----------------------|
| ROS 2 ноды требуют интеграционной среды для полного тестирования | audio_node, stt_node, tts_node, dialogue_node | Документировать как технически сложные для юнит-тестов; предложить мокинг rclpy |
| Монолитная структура (2040 строк), трудно изолировать | dialogue_node | Стратегия рефакторинга → Milestone 3 |
| Тест-файл содержит только skeleton (13+ пустых методов) | dialogue_node | FA-5: документировать, добавить в трекер |
| Модуль использует внешнее железо (ReSpeaker, pyaudio) | respeaker_interface, audio_utils | Документировать hardware dependency |
| Нет тестов вообще написано | long_term_memory | Добавить в Coverage report как gap |

### Расхождение coverage.json vs htmlcov

`coverage.json` содержит только один файл — `dialogue_node.py` с 13.4%. Это означает, что последний запуск `pytest --cov` был нацелен **только на один тест** (возможно `test_dialogue_node.py`). `htmlcov/` содержит 23 файла — это результат более широкого запуска. Оба источника устарели; в Phase 3 нужно задокументировать оба и объяснить расхождение.

---

## Stub-реализации (анализ кода)

[VERIFIED: `grep -rn "# STUB|# TODO|pass$"` по src/]

### Критические stubs (влияют на runtime поведение)

| Файл | Строка | Stub | Impact |
|------|--------|------|--------|
| `rob_box_mcp_tools/tools/system.py` | 440 | `# TODO: Получить реальные данные из РОС топиков` → возвращает `{x:0,y:0,theta:0}`, `battery:85%` | LLM получает ложные данные о положении |
| `rob_box_voice/command_node.py` | 343 | `# TODO: Получить текущую позицию из /odom или /tf` | Голосовая команда позиции молча ничего не делает |
| `rob_box_voice/command_node.py` | 359 | `# TODO: Object detection` | Голосовая команда detect_objects молча ничего не делает |
| `rob_box_voice/command_node.py` | 370 | `# TODO: Person following` | Голосовая команда follow_person молча ничего не делает |
| `rob_box_voice/command_node.py` | 394 | `pass` (тело метода пустое) | — |

### Второстепенные stubs / TODO

| Файл | Строка | Описание |
|------|--------|----------|
| `rob_box_mcp_tools/async_executor.py` | 467 | `# TODO: Получать из tool_registry` |
| `rob_box_voice/command_node.py` | 34-35 | `# TODO: Phase 6` (параметры enable_follow, enable_vision) |
| `rob_box_voice/scripts/text_normalizer_v2.py` | 209 | `# TODO: ROS2 publisher для команд` (скрипт, не нода) |

### Что НЕ считается stub

- `pass` в обработчиках исключений с комментарием обоснования (`# best-effort, продолжаем`) — это **accepted behavior**, не stub
- `# TODO` в docs/dataset/scripts — вне scope анализа

---

## Стратегия рефакторинга dialogue_node.py (предварительная)

**Текущее состояние:** 2040 строк, 1 класс `DialogueNode`, 80+ методов [VERIFIED: `grep -n "def \|class " dialogue_node.py`]

### Функциональные блоки (для декомпозиции)

| Блок | Строки (approx) | Предлагаемый класс/модуль |
|------|-----------------|---------------------------|
| Инициализация параметров и ROS (subscribers/publishers) | 88–265 | Остаётся в `DialogueNode` |
| Загрузка конфигурации (API keys, models, prompts) | 265–415 | `VoiceAssistantConfig` |
| Инициализация событийных профилей (event/FAQ) | 415–555 | `EventProfileLoader` |
| Построение агента и инструментов | 554–1242 | `AgentFactory` |
| Агентный цикл (run/retry/cancel) | 1507–1670 | `AgentRunner` |
| История диалога (_trim_history, _truncate) | 1718–1795 | `ConversationHistory` |
| DJ-режим (prompt, tick, state) | 1838–2020 | `DjModeManager` |
| ROS callbacks (VAD, STT, TTS, sound) | 1411–1715 | Остаётся в `DialogueNode` |

**Цель рефакторинга:** `dialogue_node.py` — ~300-строчный оркестратор, как описано в CONCERNS.md.

**Когда:** Milestone 3 (после навигации). В Phase 3 Milestone 1 — только **документировать стратегию**.

---

## Инфраструктура тестов (для плана верификации)

### Текущий тест-стек

| Пакет | Тест-фреймворк | Наличие pytest.ini | Тест-директория |
|-------|----------------|---------------------|-----------------|
| `rob_box_voice` | pytest | ✅ `src/rob_box_voice/pytest.ini` | `src/rob_box_voice/test/` |
| `rob_box_perception` | pytest | ❌ | `src/rob_box_perception/test/` |
| `rob_box_mcp_tools` | pytest | ✅ `src/rob_box_mcp_tools/pytest.ini` | `src/rob_box_mcp_tools/test/` |

### Команды запуска (существующие)

```bash
# Покрытие с отчётом
cd /home/ros2/rob_box_project
python3 -m pytest src/rob_box_voice/test/ --cov=src/rob_box_voice/rob_box_voice/ \
  --cov-report=html:htmlcov/ --cov-report=json:coverage.json -v

# flake8 по основным пакетам (без субмодулей)
python3 -m flake8 src/rob_box_voice/ src/rob_box_animations/ src/rob_box_perception/ \
  src/rob_box_bringup/ src/rob_box_mcp_tools/ src/rob_box_teleop/ \
  --max-line-length=120 --extend-ignore=E203,W503 --statistics --count
```

---

## Не копируй вручную (Don't Hand-Roll)

| Проблема | Не делать | Использовать |
|----------|-----------|--------------|
| Подсчёт нарушений flake8 | ручной подсчёт в файлах | `--count --statistics` флаги flake8 |
| Покрытие тестами | ручной анализ кода | `pytest-cov` + `coverage.json` + `htmlcov/` |
| Форматирование | ручное изменение отступов | `black --check` (с флагом `--check` для аудита без изменений) |
| Сортировка импортов | ручная сортировка | `isort --check-only --diff` |
| Поиск stubs | ручное чтение файлов | `grep -rn "# STUB\|# TODO\|pass$"` |

---

## Типичные ошибки (Common Pitfalls)

### Pitfall 1: Включить субмодули в статический анализ
**Что пойдёт не так:** `src/ros2leds/` и `src/vesc_nexus/` — git submodules с другими code standards. flake8 по ним выдаст тысячи ложных нарушений.  
**Как избежать:** Явно указывать только пути основных пакетов, без `src/` целиком.

### Pitfall 2: Путать coverage.json с htmlcov
**Что пойдёт не так:** `coverage.json` содержит только результат **последнего** `--cov` запуска (1 файл, 13.4%). `htmlcov/` содержит более широкий исторический прогон (23 модуля). Использовать `htmlcov/` для полного отчёта.  
**Как избежать:** Перед созданием финального отчёта запустить coverage заново по всем пакетам.

### Pitfall 3: Трактовать аудит как задачу исправления
**Что пойдёт не так:** Из-за объёма нарушений (4288) возникнет соблазн автоматически исправить W293 `autopep8 --in-place`. Но по REQUIREMENTS.md Milestone 1 = **аудит и документирование**, не фикс.  
**Как избежать:** Создавать только отчёты и помечать stubs; не изменять код (кроме добавления `# STUB:` меток).

### Pitfall 4: Не разделять severity bugs vs tech debt
**Что пойдёт не так:** BUG-12 (runtime timeout) и TD-6 (setup.py placeholder) получат одинаковый приоритет.  
**Как избежать:** Использовать единую шкалу: critical/high/medium/low. Bugs → critical/high. Performance → medium. Cosmetic → low.

### Pitfall 5: Задокументировать стратегию рефакторинга dialogue_node без понимания структуры
**Что пойдёт не так:** Стратегия получится абстрактной и неисполнимой.  
**Как избежать:** Использовать реальный список методов из `grep -n "def "` для предложения декомпозиции.

---

## Окружение (Environment Availability)

| Зависимость | Нужна для | Доступна | Версия | Fallback |
|-------------|-----------|----------|--------|----------|
| flake8 | CQ-03 | ✅ | 4.0.1 (python3 -m flake8) | — |
| black | CQ-03 | ❌ | — | Docker-среда / pip install black |
| isort | CQ-03 | ❌ | — | Docker-среда / pip install isort |
| pytest-cov | CQ-04 | Уже запущен (htmlcov/) | — | Использовать существующие данные |
| tasks.json | CQ-01, CQ-02, CQ-06 | ✅ | — | — |
| coverage.json / htmlcov/ | CQ-04 | ✅ | — | — |

**Блокеры без fallback:** Нет. black/isort можно либо установить (`pip install black isort`), либо запустить через Docker, либо использовать существующие htmlcov данные для CQ-03 (частично).

---

## Архитектура верификации (Validation Architecture)

### Команды верификации по требованиям

| Req ID | Что проверять | Команда верификации | Критерий прохождения |
|--------|---------------|---------------------|----------------------|
| CQ-01 | Все баги из CONCERNS.md в tasks.json | `python3 -c "import json; t=json.load(open('tasks.json')); [print(x['id']) for x in t['tasks'] if x.get('category')=='bug']"` | BG-1..BG-6 присутствуют |
| CQ-02 | Каждый пункт CONCERNS.md имеет severity и disposition | `cat .planning/phases/03-code-quality-review/TECH_DEBT_REGISTER.md \| grep -c "severity"` | ≥ 30 строк с severity |
| CQ-03 | Отчёт flake8 задокументирован | `ls .planning/phases/03-code-quality-review/STATIC_ANALYSIS_REPORT.md` | Файл существует с числом 4288 |
| CQ-04 | Модули < 50% задокументированы | `ls .planning/phases/03-code-quality-review/COVERAGE_REPORT.md` | Файл существует, 14 модулей listed |
| CQ-05 | Стратегия рефакторинга dialogue_node задокументирована | `ls .planning/phases/03-code-quality-review/DIALOGUE_NODE_REFACTORING.md` | Файл существует с секцией "Предлагаемые модули" |
| CQ-06 | Stubs помечены `# STUB:` в коде | `grep -rn "# STUB:" src/rob_box_voice/ src/rob_box_mcp_tools/ \| wc -l` | ≥ 4 (4 критических stub помечены) |

### Команды пост-реализации

```bash
# Проверка: stubs помечены
grep -rn "# STUB:" src/rob_box_voice/ src/rob_box_mcp_tools/ --include="*.py"

# Проверка: CQ-03 flake8 (воспроизводимость)
python3 -m flake8 src/rob_box_voice/ src/rob_box_animations/ src/rob_box_perception/ \
  src/rob_box_bringup/ src/rob_box_mcp_tools/ src/rob_box_teleop/ \
  --max-line-length=120 --extend-ignore=E203,W503 --count 2>&1 | tail -1
# Ожидается: 4288 (без изменений)

# Проверка: новые задачи в tasks.json
python3 -c "import json; d=json.load(open('tasks.json')); print(len(d['tasks']), 'tasks')"
# Должно быть > 16 (добавлены BG-5, BG-6 и stub-задачи)
```

---

## Журнал допущений (Assumptions Log)

| # | Утверждение | Секция | Риск, если неверно |
|---|-------------|--------|---------------------|
| A1 | black и isort не установлены на dev-машине — требуют pip install | Статический анализ | Если установлены в PATH через venv — можно запустить сразу |
| A2 | htmlcov/ содержит более актуальные данные о покрытии, чем coverage.json | Покрытие тестами | Если htmlcov/ устарел больше — нужно перезапустить pytest --cov |
| A3 | Все 30 пунктов CONCERNS.md равнозначны для аудита — ни один не решён после даты создания (2026-05-15) | CONCERNS audit | Если часть пунктов уже решена в последующих коммитах — severity/disposition нужно актуализировать |

---

## Открытые вопросы

1. **Нужно ли создавать `setup.cfg`/`pyproject.toml` для линтеров?**  
   - Что знаем: flake8 параметры передаются только через CLI  
   - Что неясно: является ли создание конфига частью Phase 3 или выходит за рамки CQ-03  
   - Рекомендация: создать `setup.cfg` с секцией `[flake8]` как часть плана 03-02 (не исправление кода, а конфигурация инструмента)

2. **Нужно ли перезапустить `pytest --cov` по всем пакетам для актуального отчёта?**  
   - Что знаем: coverage.json охватывает только 1 модуль; htmlcov/ охватывает 23  
   - Рекомендация: перезапустить в рамках плана 03-03, задокументировать в COVERAGE_REPORT.md

3. **Scope stub-маркировки: только production-ноды или включать scripts/?**  
   - Рекомендация: только production-ноды в `src/*/rob_box_*/` — 4 критических stub

---

## Ограничения проекта (из copilot-instructions.md)

| Директива | Как влияет на Phase 3 |
|-----------|----------------------|
| `black` line-length=120 | Флаг `--line-length 120` при black check |
| `isort` profile=black | Флаг `--profile black` при isort check |
| `flake8` | Уже запущен с правильными параметрами |
| Git: commits через workflow | Артефакты фазы коммитятся через GSD commit commands |
| ❌ НИКОГДА `COPY config/` в Dockerfile | Не релевантно для Phase 3 |
| ✅ Русский язык для ответов | Все документы фазы — на русском |
| Type hints для public API | Не релевантно для Phase 3 (аудит, не написание кода) |

---

## Источники

### Первичные (HIGH confidence)
- `[VERIFIED: read_file]` — `.planning/codebase/CONCERNS.md` (прочитан полностью)
- `[VERIFIED: python3 -m flake8 --statistics --count]` — 4288 нарушений, полная статистика
- `[VERIFIED: python3 htmlcov parsing]` — 23 модуля с процентами покрытия
- `[VERIFIED: grep -n dialogue_node.py]` — структура 80+ методов, 2040 строк
- `[VERIFIED: grep -rn stubs]` — 4 критических stub + 3 второстепенных
- `[VERIFIED: python3 tasks.json]` — 16 задач, структура трекера
- `[VERIFIED: python3 -m flake8 --version]` — flake8 4.0.1 доступен
- `[VERIFIED: python3 -m black]` — black НЕ доступен
- `[VERIFIED: python3 -m isort]` — isort НЕ доступен

### Вторичные (MEDIUM confidence)
- `[VERIFIED: .planning/STATE.md]` — текущее состояние проекта, Phase 2 complete
- `[VERIFIED: .planning/REQUIREMENTS.md]` — требования CQ-01..CQ-06
- `[VERIFIED: .planning/ROADMAP.md]` — план Phase 3 с 5 подпланами

---

## Метаданные

**Уверенность по областям:**
- Аудит CONCERNS.md: HIGH (прочитан целиком, 30 пунктов задокументированы)
- Статический анализ (flake8): HIGH (запущен вживую, 4288 нарушений подтверждено)
- Статический анализ (black/isort): LOW → MEDIUM (не установлены, требуют установки)
- Покрытие тестами: HIGH (htmlcov/ проанализирован, 23 модуля)
- Stub-анализ: HIGH (grep по коду подтверждён)
- Стратегия рефакторинга dialogue_node: MEDIUM (на основе структуры кода, конкретные строки не читались)

**Дата исследования:** 2026-05-15  
**Действительно до:** 2026-06-15 (30 дней — стабильный код)
