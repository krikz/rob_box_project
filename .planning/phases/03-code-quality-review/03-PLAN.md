---
phase: 03-code-quality-review
plan: "03 (объединённый — 5 планов в одном файле)"
type: execute
wave: 1
depends_on: []
files_modified:
  - .planning/TECH_DEBT.md
  - .planning/STATIC_ANALYSIS_REPORT.md
  - .planning/COVERAGE_REPORT.md
  - .planning/DIALOGUE_NODE_REFACTORING.md
  - tasks.json
  - src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py
  - src/rob_box_voice/rob_box_voice/command_node.py
autonomous: true
requirements:
  - CQ-01
  - CQ-02
  - CQ-03
  - CQ-04
  - CQ-05
  - CQ-06

must_haves:
  truths:
    - "Каждый из 30 пунктов CONCERNS.md имеет severity (critical/high/medium/low) и disposition (fix/defer/accept)"
    - "BG-5 и BG-6 присутствуют в tasks.json как отдельные записи"
    - "Файл .planning/STATIC_ANALYSIS_REPORT.md существует и содержит число 4288"
    - "Файл .planning/COVERAGE_REPORT.md перечисляет 14 модулей с покрытием < 50%"
    - "Файл .planning/DIALOGUE_NODE_REFACTORING.md содержит секцию с предлагаемыми модулями декомпозиции"
    - "grep -rn '# STUB:' src/rob_box_voice/ src/rob_box_mcp_tools/ возвращает ≥ 4 совпадений"
    - "Все stub-реализации добавлены в tasks.json как отдельные записи"
  artifacts:
    - path: ".planning/TECH_DEBT.md"
      provides: "Таблица всех 30 пунктов CONCERNS.md с severity и disposition"
      contains: "severity"
    - path: ".planning/STATIC_ANALYSIS_REPORT.md"
      provides: "Отчёт flake8/black/isort с числом нарушений по категориям"
      contains: "4288"
    - path: ".planning/COVERAGE_REPORT.md"
      provides: "Таблица покрытия тестами по 19 модулям"
      contains: "dialogue_node"
    - path: ".planning/DIALOGUE_NODE_REFACTORING.md"
      provides: "Стратегия рефакторинга dialogue_node.py для Milestone 3"
      contains: "AgentRunner"
    - path: "tasks.json"
      provides: "Трекер с BG-5, BG-6 и STUB-задачами"
      contains: "BG-5"
  key_links:
    - from: ".planning/codebase/CONCERNS.md"
      to: ".planning/TECH_DEBT.md"
      via: "прямое отображение 30 пунктов"
      pattern: "severity|disposition"
    - from: "TECH_DEBT.md (BG-5, BG-6)"
      to: "tasks.json"
      via: "добавление JSON-записей"
      pattern: "BG-5|BG-6"
    - from: "src/rob_box_voice/rob_box_voice/command_node.py"
      to: "# STUB: комментарии"
      via: "inline аннотации в коде"
      pattern: "# STUB:"
---

<objective>
Фаза 3 — аудит и документирование кодового качества Rob Box.

**Цель:** Задокументировать реальное состояние техдолга, покрытия тестами и архитектурных проблем — без исправления кода. По завершении все 30 пунктов CONCERNS.md будут приоритизированы, отчёты созданы, stub-реализации помечены, стратегия рефакторинга dialogue_node задокументирована.

**Почему важно:** Milestone 2 (навигация) требует ясной картины техдолга. Без TECH_DEBT.md команда не знает, что блокирует, что откладывается, что принято.

**Артефакты:**
- `.planning/TECH_DEBT.md` (30 пунктов CONCERNS.md с severity/disposition)
- `.planning/STATIC_ANALYSIS_REPORT.md` (flake8 + black + isort)
- `.planning/COVERAGE_REPORT.md` (14 модулей < 50%)
- `.planning/DIALOGUE_NODE_REFACTORING.md` (стратегия декомпозиции)
- `tasks.json` (BG-5, BG-6, STUB-задачи)
- `# STUB:` аннотации в 4 местах production-кода

**⚠ SCOPE:** Phase 3 = АУДИТ, не ИСПРАВЛЕНИЕ. Единственное исключение — добавление `# STUB:` комментариев и записей в tasks.json. Не запускать `black --fix`, `autopep8`, не менять логику кода.
</objective>

<execution_context>
@~/.copilot/get-shit-done/workflows/execute-plan.md
@~/.copilot/get-shit-done/templates/summary.md
</execution_context>

<context>
@.planning/PROJECT.md
@.planning/ROADMAP.md
@.planning/REQUIREMENTS.md
@.planning/codebase/CONCERNS.md
@.planning/phases/03-code-quality-review/03-RESEARCH.md
</context>

<tasks>

<!-- ═══════════════════════════════════════════════════════════════════
     ПЛАН 03-01: CONCERNS.md Аудит (CQ-01, CQ-02)
     Wave 1 — независимый
     ═══════════════════════════════════════════════════════════════════ -->

<task type="auto">
  <name>Задача 03-01-01: Создать .planning/TECH_DEBT.md со всеми 30 пунктами CONCERNS.md</name>
  <files>.planning/TECH_DEBT.md</files>

  <read_first>
    1. `.planning/codebase/CONCERNS.md` — полностью (все 30 пунктов в разделах TD, BG, SEC, PF, FA, SL)
    2. `.planning/phases/03-code-quality-review/03-RESEARCH.md` — раздел "Результаты аудита CONCERNS.md" (таблицы с рекомендованными severity/disposition)
    3. `tasks.json` — убедиться что BG-1..BG-4 уже есть (TASK-043..TASK-047)
  </read_first>

  <action>
Создать файл `.planning/TECH_DEBT.md` со следующей структурой:

```
# Tech Debt Register — Rob Box

**Дата аудита:** 2026-05-15
**Источник:** .planning/codebase/CONCERNS.md
**Всего пунктов:** 30
**Scope:** Milestone 1 — аудит и документирование, не исправление

---

## Шкала severity

| Уровень | Критерий |
|---------|----------|
| critical | Блокирует production; данные теряются или система неработоспособна |
| high | Существенно ухудшает UX или безопасность; требует решения в текущем milestone |
| medium | Заметная деградация; должно быть решено до Milestone 3 |
| low | Косметика или незначительное неудобство; fix или defer произвольно |

## Disposition

| Статус | Значение |
|--------|----------|
| fix | Исправить в Milestone 1 |
| defer:M2 | Отложить до Milestone 2 (навигация) |
| defer:M3 | Отложить до Milestone 3 |
| accept | Принять как есть (обоснование указано) |

---

## Tech Debt (TD)

| ID | Проблема | Файл | Severity | Disposition | Обоснование |
|----|----------|------|----------|-------------|-------------|
| TD-1 | dialogue_node.py монолит 2040 строк, 74 метода | rob_box_voice/dialogue_node.py | high | defer:M3 | Задокументирована стратегия декомпозиции; не блокирует навигацию |
| TD-2 | get_robot_status возвращает hardcoded stub (position 0,0; battery 85%) | rob_box_mcp_tools/tools/system.py:440 | high | defer:M2 | LLM получает ложные данные о положении; реальные данные появятся после Nav2 интеграции |
| TD-3 | reflection_node.py silent fallback без предупреждения при старте | rob_box_perception/reflection_node.py:668,702,747 | medium | defer:M3 | Добавить WARN log; не блокирует функциональность |
| TD-4 | command_node.py навигация/vision stubs с `pass` | rob_box_voice/command_node.py:343,359,370,394 | medium | defer:M2 | Голосовые команды навигации зависят от Nav2; помечены # STUB: |
| TD-5 | async_executor.py busy-wait polling 50ms | rob_box_mcp_tools/async_executor.py:348-357 | low | defer:M3 | CPU overhead незначителен на Raspberry Pi 5; performance sprint позже |
| TD-6 | setup.py placeholder metadata в rob_box_perception | rob_box_perception/setup.py:25-26 | low | accept | Исправлено в Phase 2 STRUCT; no impact runtime |
| TD-7 | LED compositor: 2 источника конфигурации (hardcoded + YAML) | led_matrix_driver/launch/ | low | defer:M3 | Запутывает конфигурацию; не блокирует текущую функциональность |

---

## Known Bugs (BG)

| ID | Bug ID | Описание | Severity | В tasks.json | Disposition |
|----|--------|----------|----------|-------------|-------------|
| BG-1 | TASK-043 | Unbounded local messages list в agent run → timeouts | high | ✅ TASK-043 | defer:M3 (исправление требует рефакторинга agent loop) |
| BG-2 | TASK-044 | Tool results накапливаются в conversation history | medium | ✅ TASK-044 | defer:M3 |
| BG-3 | TASK-046 | LLM забывает system prompt после 15+ iterations | low | ✅ TASK-046 | defer:M3 |
| BG-4 | TASK-047 | Barge-in регрессия (commit 37527df) | high | ✅ TASK-047 | defer:M3 |
| BG-5 | — | BLE joystick blocked by kernel 6.14.0-raspi regression | critical | ❌ добавляется | defer:M2 (kernel patch или downgrade) |
| BG-6 | — | VESC wheel jitter при старте/остановке (частично исправлен) | medium | ❌ добавляется | defer:M2 (VESC PID tuning) |

---

## Security Issues (SEC)

| ID | Проблема | Файл | Severity | Disposition | Обоснование |
|----|----------|------|----------|-------------|-------------|
| SEC-1 | Hardcoded SSH password 'open' в CI/CD workflow | .github/workflows/ | critical | fix | Заменить на GitHub Actions secret; аудит Phase 3, fix — отдельный PR |
| SEC-2 | Пароль 'open' в документации (docs/fixes/) | docs/fixes/ | high | fix | Заменить на `<ROBOT_PASSWORD>` placeholder; фикс возможен в Phase 3 |
| SEC-3 | Zenoh без аутентификации | zenoh config | medium | accept | Private network, lab use; приемлемо для текущей стадии |
| SEC-4 | privileged: true для 7 контейнеров | docker/main/, docker/vision/ | medium | accept | Hardware access требует privileged; нет альтернативы без device mapping |
| SEC-5 | network_mode: host для всех контейнеров | docker-compose файлы | low | accept | DDS/Zenoh requirement; задокументировано в архитектуре |
| SEC-6 | Unpinned ollama/ollama:latest | docker/vision/docker-compose.yml | low | defer:M3 | Pin в следующем спринте вместе с остальными образами |

---

## Performance Bottlenecks (PF)

| ID | Проблема | Файл | Severity | Disposition | Обоснование |
|----|----------|------|----------|-------------|-------------|
| PF-1 | async_executor busy-wait 50ms polling | rob_box_mcp_tools/async_executor.py:348-357 | low | defer:M3 | Overhead незначителен; performance sprint позже |
| PF-2 | dialogue_node.py context grows за один agent run | rob_box_voice/dialogue_node.py | high | defer:M3 | Связан с BUG-12 (TASK-043); решается декомпозицией M3 |
| PF-3 | voice_memory.py synchronous Ollama embedding блокирует ROS event loop | rob_box_voice/voice_memory.py | medium | defer:M3 | Async embedding требует рефакторинга voice pipeline |

---

## Fragile Areas (FA)

| ID | Проблема | Файл | Severity | Disposition | Обоснование |
|----|----------|------|----------|-------------|-------------|
| FA-1 | audio_node.py bare except: pass в shutdown | rob_box_voice/audio_node.py | medium | defer:M3 | Низкий runtime риск; исправить при рефакторинге voice pipeline |
| FA-2 | calibrate_max_rpm.py/linearity_test.py глотают все исключения | local_test/ scripts | high | defer:M2 | Инструменты диагностики; исправить до моторного тестирования M2 |
| FA-3 | VESC hardware interface: нет NaN/infinity guard | vesc_nexus (submodule) | high | defer:M2 | Нет guard = потенциальный crash при битых данных VESC |
| FA-4 | Zenoh IPs hardcoded в router configs | docker/main/config/, docker/vision/config/ | medium | defer:M2 | Затрудняет переезд на другой стенд; параметризировать при M2 настройке |
| FA-5 | test_dialogue_node.py — 13+ пустых тест-методов (только pass) | rob_box_voice/test/ | high | defer:M3 | Создаёт ложное ощущение покрытия; документировать в COVERAGE_REPORT.md |
| FA-6 | led_matrix_driver.py exception в clear_matrix → LEDs остаются включёнными | rob_box_animations/ | low | defer:M3 | Визуальный артефакт без safety implications |

---

## Scaling Limits (SL)

| ID | Проблема | Файл | Severity | Disposition | Обоснование |
|----|----------|------|----------|-------------|-------------|
| SL-1 | docker/build/ — 8 полных копий репозитория для раннеров | docker/build/ | low | accept | CI/CD overhead; accept как компромисс простоты vs скорость |
| SL-2 | Один Zenoh router = SPOF | docker/main/zenoh-router/ | medium | defer:M2 | Redundancy возможна; low priority для lab env |
| SL-3 | LLM providers — только облако, offline fallback отключён | rob_box_voice/dialogue_node.py | medium | defer:M3 | Offline LLM (Ollama local) в планах для M3 |

---

## Итоговая статистика

| Severity | Кол-во |
|----------|--------|
| critical | 2 (BG-5, SEC-1) |
| high | 11 |
| medium | 11 |
| low | 6 |

| Disposition | Кол-во |
|------------|--------|
| fix | 2 (SEC-1, SEC-2) |
| defer:M2 | 8 |
| defer:M3 | 14 |
| accept | 6 |

---
*Аудит выполнен: Phase 3 Milestone 1, 2026-05-15*
*Следующий аудит: начало Milestone 2*
```

**Важно:** Записи в таблицах взяты из 03-RESEARCH.md (раздел "Результаты аудита CONCERNS.md"). Перед созданием файла прочитай CONCERNS.md для сверки ID и формулировок.
  </action>

  <verify>
    <automated>test -f .planning/TECH_DEBT.md &amp;&amp; grep -c "severity\|Severity" .planning/TECH_DEBT.md | awk '{if ($1 >= 5) print "OK: " $1 " строк с severity"; else print "FAIL: только " $1}'</automated>
  </verify>

  <acceptance_criteria>
    - Файл `.planning/TECH_DEBT.md` создан
    - Содержит 6 разделов: TD (7 строк), BG (6), SEC (6), PF (3), FA (6), SL (3) = 30 пунктов итого
    - Каждый пункт имеет severity (critical/high/medium/low) и disposition (fix/defer:M2/defer:M3/accept)
    - `grep -c "critical\|high\|medium\|low" .planning/TECH_DEBT.md` возвращает ≥ 30
    - BG-5 и BG-6 отмечены "❌ добавляется" в трекер (задача 03-01-02 завершает их добавление)
  </acceptance_criteria>
  <done>TECH_DEBT.md создан, содержит все 30 пунктов с severity и disposition, соответствует CONCERNS.md</done>
</task>

<task type="auto">
  <name>Задача 03-01-02: Добавить BG-5 и BG-6 в tasks.json</name>
  <files>tasks.json</files>

  <read_first>
    1. `tasks.json` — полностью. Особое внимание: структура объекта задачи (поля `id`, `category`, `priority`, `description`, `acceptance_criteria`, `test_steps`, `dependencies`, `status`, `notes`), последний ID (TASK-047)
    2. `.planning/TECH_DEBT.md` — только строки BG-5 и BG-6 для точных описаний
  </read_first>

  <action>
Добавить два новых объекта в массив `tasks.tasks` в конец файла. Использовать следующую схему (идентичную существующим задачам):

**BG-5 запись:**
```json
{
  "id": "TASK-049",
  "category": "bug",
  "priority": "critical",
  "description": "BG-5: BLE joystick заблокирован регрессией ядра Linux 6.14.0-raspi. Джойстик не подключается / не распознаётся как HID-устройство после обновления ядра.",
  "acceptance_criteria": "BLE joystick успешно подключается и публикует /joy топик в ROS 2 на Raspberry Pi 5 с новым ядром; либо ядро откачено до стабильной версии без регрессии.",
  "test_steps": [
    "Проверить версию ядра: uname -r",
    "Попытаться подключить BLE джойстик: bluetoothctl connect <MAC>",
    "Проверить наличие в /dev/input/: ls /dev/input/js*",
    "Запустить ros2 topic hz /joy и убедиться что данные поступают"
  ],
  "dependencies": [],
  "status": "pending",
  "notes": "Источник: CONCERNS.md BG-5. Добавлено Phase 3 Code Quality Review. Возможные решения: (1) откат ядра до 6.12.x; (2) kernel patch для btusb/hci_core; (3) переход на USB-адаптер джойстика."
}
```

**BG-6 запись:**
```json
{
  "id": "TASK-050",
  "category": "bug",
  "priority": "medium",
  "description": "BG-6: VESC wheel jitter при старте и резком торможении. Колёса вибрируют/дёргаются из-за нестабильности PID или рассинхронизации команд скорости. Частично исправлен, edge cases остаются.",
  "acceptance_criteria": "Робот плавно стартует и останавливается без видимого jitter при скоростях 0.1–0.5 m/s; логи VESC не содержат ошибок overcurrent во время старта.",
  "test_steps": [
    "Запустить motor_testing skill: задать линейную скорость 0.3 m/s",
    "Наблюдать за поведением колёс при старте (должно быть плавным)",
    "Задать cmd_vel = 0 (полная остановка) — наблюдать за jitter",
    "Проверить логи: docker logs vesc-nexus | grep -i 'error\\|fault\\|overcurrent'"
  ],
  "dependencies": [],
  "status": "pending",
  "notes": "Источник: CONCERNS.md BG-6. Добавлено Phase 3 Code Quality Review. Связано с VESC PID параметрами. Рассмотреть: снижение acceleration limit в VESC Tool; проверить gear_ratio в конфигурации."
}
```

Формат редактирования: открыть `tasks.json`, найти конец массива `tasks` (последний элемент TASK-047), добавить запятую после закрывающей `}` TASK-047 и вставить два новых объекта перед закрывающей `]` массива.
  </action>

  <verify>
    <automated>python3 -c "import json; d=json.load(open('tasks.json')); ids=[t['id'] for t in d['tasks']]; print('Total:', len(d['tasks']), '| BG-5:', 'TASK-049' in ids, '| BG-6:', 'TASK-050' in ids)"</automated>
  </verify>

  <acceptance_criteria>
    - `python3 -c "import json; json.load(open('tasks.json'))"` — не выдаёт ошибку (JSON валиден)
    - tasks.json содержит 18 задач (было 16 + 2 новых)
    - TASK-049 присутствует с `"category": "bug"` и `"priority": "critical"`
    - TASK-050 присутствует с `"category": "bug"` и `"priority": "medium"`
    - `python3 -c "import json; d=json.load(open('tasks.json')); print([t['id'] for t in d['tasks'] if t['id'] in ['TASK-049','TASK-050']])"` выводит `['TASK-049', 'TASK-050']`
  </acceptance_criteria>
  <done>tasks.json содержит BG-5 (TASK-049) и BG-6 (TASK-050); JSON валиден; итого 18 задач</done>
</task>

<!-- ═══════════════════════════════════════════════════════════════════
     ПЛАН 03-02: Статический анализ (CQ-03)
     Wave 1 — независимый
     ═══════════════════════════════════════════════════════════════════ -->

<task type="auto">
  <name>Задача 03-02-01: Запустить flake8 по 6 пакетам и записать полный вывод</name>
  <files>.planning/STATIC_ANALYSIS_REPORT.md</files>

  <read_first>
    1. `.planning/phases/03-code-quality-review/03-RESEARCH.md` — раздел "flake8 — VERIFIED" (команда, параметры, ожидаемые 4288 нарушений)
    2. Проверить наличие flake8: `python3 -m flake8 --version`
  </read_first>

  <action>
**Шаг 1: Установить black и isort** (не установлены на dev-машине):
```bash
pip install black isort --quiet
```

**Шаг 2: Запустить flake8** (уже доступен):
```bash
python3 -m flake8 \
  src/rob_box_voice/ src/rob_box_animations/ src/rob_box_perception/ \
  src/rob_box_bringup/ src/rob_box_mcp_tools/ src/rob_box_teleop/ \
  --max-line-length=120 --extend-ignore=E203,W503 \
  --statistics --count 2>&1 | tee /tmp/flake8_full.txt
```
Сохранить итоговое число (последняя строка) и таблицу statistics.

**Шаг 3: Запустить black --check**:
```bash
python3 -m black --check --line-length 120 \
  src/rob_box_voice/ src/rob_box_animations/ src/rob_box_perception/ \
  src/rob_box_bringup/ src/rob_box_mcp_tools/ src/rob_box_teleop/ \
  2>&1 | tee /tmp/black_check.txt
# Подсчитать количество файлов "would reformat":
grep -c "would reformat" /tmp/black_check.txt || true
```

**Шаг 4: Запустить isort --check-only**:
```bash
python3 -m isort --check-only --profile black \
  src/rob_box_voice/ src/rob_box_animations/ src/rob_box_perception/ \
  src/rob_box_bringup/ src/rob_box_mcp_tools/ src/rob_box_teleop/ \
  2>&1 | tee /tmp/isort_check.txt
# Подсчитать количество файлов с неправильной сортировкой:
grep -c "ERROR" /tmp/isort_check.txt || grep -c "Fixing\|would be" /tmp/isort_check.txt || true
```

**Шаг 5: Создать `.planning/STATIC_ANALYSIS_REPORT.md`** с результатами:

```markdown
# Отчёт статического анализа — Rob Box

**Дата:** 2026-05-15
**Scope:** 6 пакетов (rob_box_voice, rob_box_animations, rob_box_perception, rob_box_bringup, rob_box_mcp_tools, rob_box_teleop)
**Исключены:** src/ros2leds/ и src/vesc_nexus/ (git submodules)

---

## flake8

**Версия:** 4.0.1
**Параметры:** `--max-line-length=120 --extend-ignore=E203,W503`
**Итого нарушений:** {ИТОГО из вывода}

### Нарушения по категории

| Код | Кол-во | Описание | Приоритет |
|-----|--------|----------|-----------|
| W293 | {N} | Blank line contains whitespace | LOW (автоисправление) |
| F401 | {N} | Unused imports | MEDIUM |
| E501 | {N} | Line too long (>120 chars) | MEDIUM |
| F541 | {N} | f-string missing placeholders | HIGH |
| E722 | {N} | Bare except | HIGH |
| F821 | {N} | Undefined name | HIGH |
| ... | ... | ... | ... |
| **ИТОГО** | **{N}** | | |

### Топ файлов по числу нарушений

| Файл | Нарушений |
|------|-----------|
| {файл} | {N} |

### HIGH-priority нарушения (требуют внимания)

| Код | Кол-во | Почему важно |
|-----|--------|--------------|
| F541 | {N} | f-string без {} — скрытые логические ошибки, не поймает runtime |
| E722 | {N} | bare except: глотает все исключения включая KeyboardInterrupt |
| F821 | {N} | undefined name — потенциальный NameError в runtime |
| F401 | {N} | unused imports — загрязнение namespace, возможны циклические зависимости |

---

## black

**Версия:** {версия}
**Параметры:** `--check --line-length 120`
**Файлов требующих форматирования:** {N}

{Вставить список файлов "would reformat" или "All done! ✨ 🍰 ✨" если все ок}

---

## isort

**Версия:** {версия}
**Параметры:** `--check-only --profile black`
**Файлов с неправильным порядком импортов:** {N}

{Вставить список файлов с ошибками или "Skipped N files"}

---

## Сводка

| Инструмент | Нарушений / файлов | Статус |
|------------|-------------------|--------|
| flake8 | {N} нарушений | ⚠️ Требует внимания |
| black | {N} файлов | ⚠️ / ✅ |
| isort | {N} файлов | ⚠️ / ✅ |

## Рекомендации

**Немедленно (HIGH priority):**
- F541 ({N}): исправить f-strings без `{}` — это логические ошибки
- E722 ({N}): заменить bare `except:` на `except Exception:` или специфичный тип
- F821 (1): исправить undefined name `Command` (найти правильный импорт)

**При следующем рефакторинге:**
- F401 ({N}): очистить неиспользуемые импорты
- W293/W291 ({N}): запустить `autopep8 --in-place --select=W293,W291`

**Конфигурация:** Создать `setup.cfg` с секцией `[flake8]` для воспроизводимости CLI-параметров.

---

## Конфигурационный файл (рекомендуется создать)

```ini
# setup.cfg
[flake8]
max-line-length = 120
extend-ignore = E203,W503
exclude =
    src/ros2leds/,
    src/vesc_nexus/,
    build/,
    install/
```

---
*Отчёт создан: Phase 3 Milestone 1*
*Следующий запуск: перед Milestone 2*
```

**Заполнить плейсхолдеры `{N}` реальными числами** из вывода команд выше.
  </action>

  <verify>
    <automated>test -f .planning/STATIC_ANALYSIS_REPORT.md &amp;&amp; grep -q "4288\|flake8\|Итого нарушений" .planning/STATIC_ANALYSIS_REPORT.md &amp;&amp; echo "OK: STATIC_ANALYSIS_REPORT.md создан с данными flake8" || echo "FAIL: файл отсутствует или не содержит данные flake8"</automated>
  </verify>

  <acceptance_criteria>
    - `.planning/STATIC_ANALYSIS_REPORT.md` создан
    - Содержит секции "flake8", "black", "isort"
    - Число нарушений flake8 присутствует (ожидается ~4288, если код не менялся)
    - Содержит таблицу нарушений по кодам (W293, F401, F541, E722 минимум)
    - Содержит секцию "HIGH-priority нарушения"
    - `python3 -m black --version` не выдаёт ошибку (black установлен)
    - `python3 -m isort --version` не выдаёт ошибку (isort установлен)
  </acceptance_criteria>
  <done>STATIC_ANALYSIS_REPORT.md создан с реальными данными flake8, black, isort; black и isort установлены</done>
</task>

<!-- ═══════════════════════════════════════════════════════════════════
     ПЛАН 03-03: Coverage Report (CQ-04)
     Wave 1 — независимый
     ═══════════════════════════════════════════════════════════════════ -->

<task type="auto">
  <name>Задача 03-03-01: Перезапустить pytest --cov и создать COVERAGE_REPORT.md</name>
  <files>.planning/COVERAGE_REPORT.md</files>

  <read_first>
    1. `.planning/phases/03-code-quality-review/03-RESEARCH.md` — раздел "Покрытие тестами" (таблица модулей, объяснение причин, команды запуска)
    2. `htmlcov/index.html` — чтобы получить актуальные проценты по модулям
    3. `coverage.json` — проверить расхождение с htmlcov
  </read_first>

  <action>
**Шаг 1: Перезапустить pytest с полным coverage по всем пакетам:**
```bash
cd /home/ros2/rob_box_project
python3 -m pytest \
  src/rob_box_voice/test/ \
  src/rob_box_perception/test/ \
  src/rob_box_mcp_tools/test/ \
  --cov=src/rob_box_voice/rob_box_voice/ \
  --cov=src/rob_box_perception/rob_box_perception/ \
  --cov=src/rob_box_mcp_tools/rob_box_mcp_tools/ \
  --cov-report=html:htmlcov/ \
  --cov-report=json:coverage.json \
  --cov-report=term-missing \
  -q 2>&1 | tee /tmp/coverage_run.txt
```

Если тесты падают (например, из-за отсутствующего железа), добавить флаг `-x --ignore=src/rob_box_voice/test/integration/` или запустить только unit-тесты.

**Шаг 2: Извлечь данные из обновлённого coverage.json:**
```bash
python3 -c "
import json
with open('coverage.json') as f:
    data = json.load(f)
files = data.get('files', {})
results = []
for path, info in files.items():
    pct = info['summary']['percent_covered']
    results.append((path, round(pct, 1)))
results.sort(key=lambda x: x[1])
for path, pct in results:
    short = path.replace('/home/ros2/rob_box_project/src/', '')
    print(f'{pct:5.1f}%  {short}')
"
```

**Шаг 3: Создать `.planning/COVERAGE_REPORT.md`:**

```markdown
# Отчёт покрытия тестами — Rob Box

**Дата:** 2026-05-15
**Инструмент:** pytest-cov (coverage.py)
**Источник:** Перезапуск pytest --cov по всем пакетам

---

## Сводная таблица

| Модуль | Покрытие | Статус | Причина низкого покрытия |
|--------|----------|--------|--------------------------|
| rob_box_voice/audio_node.py | 0% | ❌ | ROS 2 нода; требует rclpy + audio hardware для init |
| rob_box_voice/dialogue_node.py | ~13% | ❌ | Монолит 2040 строк; test_dialogue_node.py имеет 13+ пустых методов (FA-5) |
| rob_box_voice/stt_node.py | 0% | ❌ | ROS 2 нода; требует ReSpeaker hardware |
| rob_box_voice/tts_node.py | 0% | ❌ | ROS 2 нода; требует audio playback pipeline |
| rob_box_perception/vision_stub_node.py | 0% | ❌ | Stub-нода; нет логики для тестирования |
| rob_box_perception/utils/long_term_memory.py | 0% | ❌ | Нет тестов написано |
| rob_box_voice/utils/respeaker_interface.py | 20% | ⚠️ | Hardware dependency (ReSpeaker USB) |
| rob_box_voice/utils/audio_utils.py | 21% | ⚠️ | Hardware dependency (pyaudio) |
| rob_box_voice/audio_playback_manager.py | 24% | ⚠️ | Зависит от audio hardware |
| rob_box_voice/command_node.py | 28% | ⚠️ | Stub-методы с `pass`; ROS callbacks трудно мокировать |
| rob_box_perception/reflection_node.py | 28% | ⚠️ | Сложная инициализация с ROS параметрами |
| rob_box_perception/utils/node_monitor.py | 37% | ⚠️ | Частичное покрытие; остальное требует запущенного ROS |
| rob_box_perception/utils/time_provider.py | 39% | ⚠️ | Clock-зависимый код |
| rob_box_perception/context_aggregator_node.py | 43% | ⚠️ | ROS subscriber callbacks не покрыты |
| rob_box_voice/sound_node.py | 53% | ✅ | |
| rob_box_voice/led_node.py | 62% | ✅ | |
| rob_box_perception/health_monitor.py | 69% | ✅ | |
| rob_box_perception/utils/internet_monitor.py | 71% | ✅ | |
| rob_box_perception/startup_greeting_node.py | 77% | ✅ | |

**Модулей ниже 50%:** 14 из 19 (74%)
**Модулей с 0% покрытием:** 6

---

## Анализ причин низкого покрытия

### 1. ROS 2 hardware dependency (audio_node, stt_node, tts_node, respeaker_interface)

ROS 2 ноды вызывают `super().__init__()` в конструкторе, что требует запущенного rclpy контекста. Без `rclpy.init()` тест падает при попытке создать объект. Unit-тесты этих нод невозможны без мокирования rclpy на уровне инициализации.

**Путь к улучшению (Milestone 3):** Инъекция зависимостей через паттерн constructor injection; отделение бизнес-логики от ROS glue.

### 2. Монолитная структура dialogue_node.py (TD-1, FA-5)

2040 строк в одном классе создают 74 метода с взаимными зависимостями. 13+ тест-методов в `test_dialogue_node.py` содержат только `pass` — это создаёт ложное ощущение тест-покрытия без реального тестирования.

**Путь к улучшению (Milestone 3):** Декомпозиция на 6 компонентов (AgentFactory, AgentRunner, ConversationHistory, DjModeManager, VoiceAssistantConfig, EventProfileLoader) — см. DIALOGUE_NODE_REFACTORING.md.

### 3. Отсутствие тестов (long_term_memory)

`long_term_memory.py` — полноценный модуль без единого теста. Не связан с hardware.

**Путь к улучшению (Milestone 2):** Написать unit-тесты; модуль тестируем без ROS окружения.

### 4. Расхождение coverage.json vs htmlcov

`coverage.json` в репозитории содержит данные **только одного модуля** (dialogue_node.py, 13.4%) — результат последнего `pytest --cov` запуска по одному тесту. `htmlcov/` содержит данные более широкого исторического прогона (23 файла). Данный отчёт основан на перезапуске pytest по всем пакетам.

---

## Пустые тест-методы (FA-5)

`src/rob_box_voice/test/test_dialogue_node.py` содержит 13+ методов со skeleton-реализацией (`pass`):

```python
def test_handle_voice_input(self):
    pass  # TODO: implement

def test_agent_run(self):
    pass  # TODO: implement
```

Эти методы **не проверяют ничего** и не ухудшают coverage, но создают ложное ощущение тест-инфраструктуры. Добавлено в трекер как FA-5 (Milestone 3).

---

## Рекомендации по улучшению покрытия

| Приоритет | Модуль | Что сделать | Milestone |
|-----------|--------|-------------|-----------|
| HIGH | long_term_memory.py | Написать unit-тесты (не требует hardware) | M2 |
| HIGH | dialogue_node.py | Декомпозиция + тесты для изолированных компонентов | M3 |
| MEDIUM | command_node.py | Мокировать ROS subscribers; тестировать parse логику | M3 |
| LOW | audio_node, stt_node, tts_node | Инъекция зависимостей → unit-тесты без hardware | M3 |

---
*Отчёт создан: Phase 3 Milestone 1*
*Следующий запуск: начало Milestone 2 (перед навигационными тестами)*
```

**Важно:** Заполнить реальными процентами из перезапуска pytest. Если тесты не проходят из-за hardware, использовать данные из `htmlcov/index.html` (распарсить HTML) и явно указать это в отчёте.
  </action>

  <verify>
    <automated>test -f .planning/COVERAGE_REPORT.md &amp;&amp; grep -q "dialogue_node\|14 из\|покрытие" .planning/COVERAGE_REPORT.md &amp;&amp; echo "OK: COVERAGE_REPORT.md создан" || echo "FAIL"</automated>
  </verify>

  <acceptance_criteria>
    - `.planning/COVERAGE_REPORT.md` создан
    - Содержит сводную таблицу с ≥ 14 модулями ниже 50%
    - Содержит секцию "Анализ причин" с объяснением по каждой категории причин
    - Упоминает расхождение coverage.json vs htmlcov
    - Упоминает FA-5 (пустые тест-методы)
    - Содержит раздел "Рекомендации" с milestone указаниями
  </acceptance_criteria>
  <done>COVERAGE_REPORT.md создан с таблицей покрытия, анализом причин и рекомендациями</done>
</task>

<!-- ═══════════════════════════════════════════════════════════════════
     ПЛАН 03-04: Dialogue Node Стратегия (CQ-05)
     Wave 1 — независимый
     ═══════════════════════════════════════════════════════════════════ -->

<task type="auto">
  <name>Задача 03-04-01: Проанализировать структуру dialogue_node.py и создать DIALOGUE_NODE_REFACTORING.md</name>
  <files>.planning/DIALOGUE_NODE_REFACTORING.md</files>

  <read_first>
    1. `.planning/phases/03-code-quality-review/03-RESEARCH.md` — раздел "Стратегия рефакторинга dialogue_node.py" (таблица функциональных блоков с строками и предлагаемыми именами)
    2. Запустить: `grep -n "def \|class " src/rob_box_voice/rob_box_voice/dialogue_node.py | head -100` — получить реальный список методов с номерами строк
    3. `src/rob_box_voice/rob_box_voice/dialogue_node.py` строки 1–100 (импорты и __init__ сигнатура) — понять зависимости
  </read_first>

  <action>
**Шаг 1: Получить реальный список методов:**
```bash
grep -n "def \|class " src/rob_box_voice/rob_box_voice/dialogue_node.py
```
Использовать вывод для уточнения строковых диапазонов блоков в документе.

**Шаг 2: Создать `.planning/DIALOGUE_NODE_REFACTORING.md`:**

```markdown
# Стратегия рефакторинга dialogue_node.py — Rob Box

**Дата:** 2026-05-15
**Текущее состояние:** 2040 строк, 1 класс DialogueNode, 74 метода
**Целевое состояние (Milestone 3):** ~300-строчный оркестратор + 6 специализированных компонентов
**Приоритет:** MEDIUM (не блокирует Milestone 2 навигацию)
**Планируемый milestone:** Milestone 3

---

## Проблемы текущей архитектуры

1. **Монолитность:** DialogueNode делает всё — конфигурацию, агентный loop, историю диалога, DJ-режим, ROS callbacks
2. **Низкое покрытие тестами:** 13.4% — практически невозможно изолировать компонент для unit-теста
3. **Высокий контекст агента:** Из-за объёма файла агент тратит 30-40% контекстного окна только на чтение файла
4. **Скрытые зависимости:** Методы обращаются к 15+ атрибутам self; не понятно что от чего зависит
5. **Трудность поддержки:** Добавление нового tool требует изменений в 4+ местах файла

---

## Предлагаемые модули декомпозиции

### Компонент 1: VoiceAssistantConfig
**Ответственность:** Загрузка и валидация всей конфигурации
**Строки (approx):** 265–415 из текущего dialogue_node.py
**Что включает:**
- Чтение ROS параметров (API keys, model names, system prompts)
- Загрузка конфигурации из YAML/JSON
- Валидация наличия обязательных ключей
**Интерфейс:**
```python
@dataclass
class VoiceAssistantConfig:
    llm_provider: str
    llm_model: str
    system_prompt: str
    mcp_tools_enabled: bool
    # ... остальные параметры

    @classmethod
    def from_ros_node(cls, node: rclpy.node.Node) -> 'VoiceAssistantConfig':
        ...
```

### Компонент 2: EventProfileLoader
**Ответственность:** Загрузка event/FAQ профилей для специальных режимов
**Строки (approx):** 415–555 из текущего dialogue_node.py
**Что включает:**
- Чтение профилей мероприятий из файлов
- Парсинг FAQ
- Кэширование загруженных профилей
**Интерфейс:**
```python
class EventProfileLoader:
    def load(self, profile_path: str) -> EventProfile:
        ...
    def get_active_profile(self) -> Optional[EventProfile]:
        ...
```

### Компонент 3: AgentFactory
**Ответственность:** Сборка LangChain агента и инструментов
**Строки (approx):** 554–1242 из текущего dialogue_node.py
**Что включает:**
- Инициализация LLM (ChatOpenAI, ChatAnthropic, etc.)
- Регистрация MCP tools
- Создание агента (create_react_agent / create_tool_calling_agent)
- Настройка memory/checkpointer
**Интерфейс:**
```python
class AgentFactory:
    def build(self, config: VoiceAssistantConfig) -> CompiledGraph:
        ...
    def rebuild(self) -> CompiledGraph:
        # для hot-reload при смене профиля
        ...
```

### Компонент 4: AgentRunner
**Ответственность:** Управление циклом выполнения агента (run/retry/cancel)
**Строки (approx):** 1507–1670 из текущего dialogue_node.py
**Что включает:**
- Запуск агента в async-контексте
- Обработка timeout и retry логики
- Отмена выполнения (cancel token)
- Публикация промежуточных результатов (streaming)
**Интерфейс:**
```python
class AgentRunner:
    async def run(self, input_text: str, history: ConversationHistory) -> AgentResult:
        ...
    def cancel(self) -> None:
        ...
```
**Связан с:** BUG-12 (TASK-043 — unbounded context); исправление context limit входит в AgentRunner.

### Компонент 5: ConversationHistory
**Ответственность:** Хранение, обрезка и управление историей диалога
**Строки (approx):** 1718–1795 из текущего dialogue_node.py
**Что включает:**
- Добавление сообщений (user/assistant/tool)
- `_trim_history` — обрезка по количеству токенов
- `_truncate` — обрезка по числу сообщений
- Персистентность (опционально)
**Интерфейс:**
```python
class ConversationHistory:
    def add(self, role: str, content: str) -> None:
        ...
    def trim(self, max_tokens: int) -> None:
        ...
    def get(self) -> list[dict]:
        ...
    def clear(self) -> None:
        ...
```
**Тестируемость:** Полностью изолирован от ROS; легко покрыть unit-тестами.

### Компонент 6: DjModeManager
**Ответственность:** Управление DJ-режимом (состояние, промпты, tick)
**Строки (approx):** 1838–2020 из текущего dialogue_node.py
**Что включает:**
- Состояние DJ-режима (активен/выключен, текущий трек, настроение)
- Генерация DJ-промптов
- Tick-обновления (периодические комментарии)
**Интерфейс:**
```python
class DjModeManager:
    def activate(self, playlist: list[str]) -> None:
        ...
    def deactivate(self) -> None:
        ...
    def tick(self) -> Optional[str]:
        # возвращает следующий DJ-комментарий или None
        ...
```

---

## Что остаётся в DialogueNode

После декомпозиции DialogueNode становится ~300-строчным оркестратором:
- ROS callbacks (VAD events, STT results, TTS completion, sound events)
- Инициализация и wiring компонентов
- Публикация в ROS topics
- Lifecycle management (on_configure, on_activate, on_deactivate)

---

## Порядок реализации (Milestone 3)

Рекомендуемый порядок — от наименее зависимого к наиболее:

```
1. ConversationHistory  ← нет зависимостей, полностью тестируем
2. VoiceAssistantConfig ← только ROS params, легко мокировать
3. EventProfileLoader   ← зависит только от файловой системы
4. AgentFactory         ← зависит от VoiceAssistantConfig + EventProfileLoader
5. AgentRunner          ← зависит от AgentFactory + ConversationHistory
6. DjModeManager        ← зависит от AgentRunner + ConversationHistory
7. DialogueNode refactor← wires всё вместе, после всех компонентов
```

**Принцип:** Каждый компонент создаётся с тестами **перед** интеграцией в DialogueNode. Фаза рефакторинга = 7 последовательных PR.

---

## Риски и митигации

| Риск | Вероятность | Митигация |
|------|------------|-----------|
| Скрытые зависимости между компонентами | HIGH | Начинать с ConversationHistory (нет зависимостей); каждый компонент проходит review |
| Регрессия в runtime поведении агента | MEDIUM | Интеграционный тест всего pipeline перед каждым PR |
| Контекстное окно агента при чтении 2040 строк | HIGH | Разбить реализацию на 7 отдельных агентских сессий |
| Нарушение barge-in поведения при рефакторинге | LOW | Отдельный тест на barge-in (TASK-047 уже в трекере) |

---

## Метрики успеха (Milestone 3)

| Метрика | Сейчас | Цель |
|---------|--------|------|
| Строк в dialogue_node.py | 2040 | ≤ 350 |
| Покрытие тестами dialogue_node.py | 13% | ≥ 60% |
| Покрытие ConversationHistory | — | ≥ 90% |
| Цикломатическая сложность DialogueNode | ~85 | ≤ 15 |

---
*Стратегия определена: Phase 3 Milestone 1*
*Реализация: Milestone 3 (после навигации)*
```
  </action>

  <verify>
    <automated>test -f .planning/DIALOGUE_NODE_REFACTORING.md &amp;&amp; grep -q "AgentRunner\|ConversationHistory\|VoiceAssistantConfig" .planning/DIALOGUE_NODE_REFACTORING.md &amp;&amp; echo "OK: DIALOGUE_NODE_REFACTORING.md создан с компонентами декомпозиции" || echo "FAIL"</automated>
  </verify>

  <acceptance_criteria>
    - `.planning/DIALOGUE_NODE_REFACTORING.md` создан
    - Содержит описание всех 6 компонентов: VoiceAssistantConfig, EventProfileLoader, AgentFactory, AgentRunner, ConversationHistory, DjModeManager
    - Каждый компонент имеет: ответственность, строковый диапазон, интерфейс (Python dataclass/class)
    - Раздел "Порядок реализации" содержит нумерованный список из 7 шагов
    - Раздел "Метрики успеха" содержит таблицу "Сейчас → Цель"
    - `grep -c "Компонент" .planning/DIALOGUE_NODE_REFACTORING.md` ≥ 6
  </acceptance_criteria>
  <done>DIALOGUE_NODE_REFACTORING.md создан с полной стратегией декомпозиции на 6 компонентов, порядком реализации и метриками</done>
</task>

<!-- ═══════════════════════════════════════════════════════════════════
     ПЛАН 03-05: Stub Tracking (CQ-06)
     Wave 2 — зависит от 03-01 (tasks.json должен быть обновлён)
     ═══════════════════════════════════════════════════════════════════ -->

<task type="auto">
  <name>Задача 03-05-01: Добавить # STUB: комментарии в production-код</name>
  <files>
    src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py
    src/rob_box_voice/rob_box_voice/command_node.py
  </files>

  <read_first>
    1. `.planning/phases/03-code-quality-review/03-RESEARCH.md` — раздел "Stub-реализации" (таблица с точными строками: system.py:440, command_node.py:343,359,370,394)
    2. `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py` строки 430–460 — найти точное место stub (функция get_robot_status, комментарий `# TODO: Получить реальные данные`)
    3. `src/rob_box_voice/rob_box_voice/command_node.py` строки 335–400 — найти все 4 места с `# TODO` и `pass`
  </read_first>

  <action>
**Точные места для маркировки:**

### system.py — get_robot_status

Найти строку ~440 с `# TODO: Получить реальные данные из РОС топиков` и заменить (или добавить рядом):

```python
# STUB: get_robot_status возвращает hardcoded данные — реальная позиция и battery
# будут получены из /odom и /battery_state ROS topics после Nav2 интеграции (Milestone 2).
# Отслеживается: TECH_DEBT.md TD-2, tasks.json TASK-... (STUB-get-robot-status)
```

Оставить `# TODO` нетронутым если он уже есть; добавить `# STUB:` **перед** строкой с `# TODO` или прямо перед hardcoded return.

### command_node.py — 3 метода навигации

**Место 1 (~строка 343):** метод с `# TODO: Получить текущую позицию из /odom или /tf`:
```python
# STUB: get_current_position не реализован — возвращает None.
# Реальная реализация требует подписки на /odom топик (Milestone 2).
# Отслеживается: TECH_DEBT.md TD-4, tasks.json TASK-... (STUB-get-position)
```

**Место 2 (~строка 359):** метод с `# TODO: Object detection`:
```python
# STUB: detect_objects не реализован — требует OAK-D depth camera pipeline (Milestone 2).
# Отслеживается: TECH_DEBT.md TD-4, tasks.json TASK-... (STUB-detect-objects)
```

**Место 3 (~строка 370):** метод с `# TODO: Person following`:
```python
# STUB: follow_person не реализован — требует object detection + Nav2 (Milestone 2/3).
# Отслеживается: TECH_DEBT.md TD-4, tasks.json TASK-... (STUB-follow-person)
```

**Место 4 (~строка 394):** метод с пустым телом (`pass`):
```python
# STUB: Метод не реализован (тело пустое).
# Отслеживается: TECH_DEBT.md TD-4
```

**Правило редактирования:** Добавить `# STUB:` комментарий **непосредственно перед** строкой `# TODO` или `pass`, сохраняя отступ. Не удалять существующие `# TODO` комментарии. Не менять логику кода.

**Верификация после редактирования:**
```bash
grep -rn "# STUB:" src/rob_box_mcp_tools/ src/rob_box_voice/ --include="*.py"
```
Ожидаемый вывод: ≥ 4 строки с `# STUB:`.
  </action>

  <verify>
    <automated>grep -rn "# STUB:" src/rob_box_mcp_tools/rob_box_mcp_tools/ src/rob_box_voice/rob_box_voice/ --include="*.py" | wc -l | awk '{if ($1 >= 4) print "OK: " $1 " STUB markers found"; else print "FAIL: only " $1 " markers (need >= 4)"}'</automated>
  </verify>

  <acceptance_criteria>
    - `grep -rn "# STUB:" src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py` — возвращает ≥ 1 результат
    - `grep -rn "# STUB:" src/rob_box_voice/rob_box_voice/command_node.py` — возвращает ≥ 3 результата
    - Итого: `grep -rn "# STUB:" src/ --include="*.py" | wc -l` ≥ 4
    - Существующие `# TODO` комментарии сохранены (не удалены)
    - Логика кода не изменена (только добавлены комментарии)
    - `python3 -m flake8 src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py src/rob_box_voice/rob_box_voice/command_node.py --max-line-length=120 --extend-ignore=E203,W503 --count 2>&1 | tail -1` — число нарушений не выросло по сравнению с baseline (допустимо ±2 из-за длины новых комментариев)
  </acceptance_criteria>
  <done>4+ # STUB: маркера добавлены в system.py и command_node.py; код не изменён</done>
</task>

<task type="auto">
  <name>Задача 03-05-02: Добавить STUB-задачи в tasks.json</name>
  <files>tasks.json</files>

  <read_first>
    ⚠️ Выполнять только после 03-01-02 (задача добавляет TASK-049 и TASK-050 в tasks.json — нужен актуальный файл).
    1. `tasks.json` — текущее состояние после выполнения 03-01-02 (должно быть 18 задач: TASK-001..TASK-050)
    2. `src/rob_box_voice/rob_box_voice/command_node.py` — строки с только что добавленными # STUB: комментариями (для точных описаний)
    3. `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py` — строка с # STUB: get_robot_status
  </read_first>

  <action>
Добавить 3 новые записи в массив `tasks.tasks` в конец файла (после TASK-050). Одна задача на каждую самостоятельную stub-группу:

**STUB-1: get_robot_status**
```json
{
  "id": "TASK-051",
  "category": "stub",
  "priority": "high",
  "description": "STUB-get-robot-status: Реализовать get_robot_status в rob_box_mcp_tools/tools/system.py через реальные ROS топики. Сейчас возвращает hardcoded данные: position {x:0,y:0,theta:0}, battery 85%. LLM получает ложные данные о состоянии робота.",
  "acceptance_criteria": "get_robot_status подписывается на /odom и /battery_state топики; возвращает реальную позицию и уровень заряда. Промпт LLM содержит актуальные данные.",
  "test_steps": [
    "Запустить Nav2 и убедиться что /odom публикуется",
    "Вызвать MCP tool get_robot_status через LLM",
    "Убедиться что position != {x:0,y:0,theta:0} и battery != 85% (кроме случая когда это реальные значения)"
  ],
  "dependencies": ["Nav2 интеграция (Milestone 2)"],
  "status": "pending",
  "notes": "Источник: CONCERNS.md TD-2. Помечено # STUB: в system.py:~440. Реализовать в Milestone 2 как часть Nav2 интеграции."
}
```

**STUB-2: Навигационные команды (command_node.py)**
```json
{
  "id": "TASK-052",
  "category": "stub",
  "priority": "medium",
  "description": "STUB-navigation-commands: Реализовать 3 голосовые команды в command_node.py: get_current_position (из /odom), detect_objects (OAK-D pipeline), follow_person (Nav2 + detection). Сейчас все 3 метода содержат только pass или TODO — голосовые команды молча ничего не делают.",
  "acceptance_criteria": "Голосовая команда 'где я?' возвращает текущую позицию из /odom. 'найди объект' запускает OAK-D detection pipeline. 'следуй за мной' активирует Nav2 following behavior.",
  "test_steps": [
    "Произнести 'где я?' — робот должен назвать координаты",
    "Произнести 'что ты видишь?' — должен запуститься detection",
    "Проверить command_node.py: методы должны содержать реальную логику вместо pass"
  ],
  "dependencies": ["Nav2 интеграция (Milestone 2)", "OAK-D pipeline"],
  "status": "pending",
  "notes": "Источник: CONCERNS.md TD-4. Помечено # STUB: в command_node.py:343,359,370,394. Реализовать поэтапно в Milestone 2."
}
```

**STUB-3: Обновить статус маркировки в TECH_DEBT.md**
*(Эта запись — не новый stub, а трекер для самой задачи маркировки — пропустить)*

Добавить только TASK-051 и TASK-052. Итого tasks.json должен содержать 20 задач.

**После добавления проверить валидность JSON:**
```bash
python3 -c "import json; d=json.load(open('tasks.json')); print('Valid JSON. Tasks:', len(d['tasks']))"
```
  </action>

  <verify>
    <automated>python3 -c "import json; d=json.load(open('tasks.json')); ids=[t['id'] for t in d['tasks']]; print('Total:', len(d['tasks']), '| TASK-051:', 'TASK-051' in ids, '| TASK-052:', 'TASK-052' in ids)"</automated>
  </verify>

  <acceptance_criteria>
    - `python3 -c "import json; json.load(open('tasks.json'))"` — JSON валиден без ошибок
    - tasks.json содержит 20 задач (16 исходных + BG-5 + BG-6 + STUB-050 + STUB-051)
    - TASK-051 присутствует с `"category": "stub"` и `"priority": "high"`
    - TASK-052 присутствует с `"category": "stub"` и `"priority": "medium"`
    - `python3 -c "import json; d=json.load(open('tasks.json')); stubs=[t for t in d['tasks'] if t['category']=='stub']; print(len(stubs))"` выводит 2
  </acceptance_criteria>
  <done>tasks.json содержит TASK-051 и TASK-052 (stub-задачи); JSON валиден; итого 20 задач</done>
</task>

</tasks>

<threat_model>
## Trust Boundaries

| Boundary | Description |
|----------|-------------|
| dev-машина → tasks.json | Агент пишет JSON вручную; невалидный JSON сломает трекер |
| flake8 stdout → STATIC_ANALYSIS_REPORT.md | Вывод инструмента парсится агентом; ошибки парсинга дадут неверные числа |
| coverage.json → COVERAGE_REPORT.md | Устаревший json (1 файл) — агент может спутать с полным отчётом |

## STRIDE Threat Register

| Threat ID | Category | Component | Disposition | Mitigation Plan |
|-----------|----------|-----------|-------------|-----------------|
| T-03-01 | Tampering | tasks.json | mitigate | Валидировать JSON после каждого добавления: `python3 -c "import json; json.load(open('tasks.json'))"` |
| T-03-02 | Information Disclosure | SEC-1 hardcoded SSH password в CI/CD | mitigate | Задокументировано в TECH_DEBT.md как critical/fix; фактический фикс — отдельный PR (не в scope Phase 3) |
| T-03-03 | Denial of Service | flake8 по submodules | mitigate | Явно указывать только 6 основных пакетов; никогда не `src/` целиком |
| T-03-04 | Information Disclosure | coverage данные устарели | accept | Перезапустить pytest в задаче 03-03-01; явно указать дату в COVERAGE_REPORT.md |
</threat_model>

<verification>
## Итоговая проверка фазы

После выполнения всех 7 задач запустить:

```bash
# CQ-01: BG-5 и BG-6 в трекере
python3 -c "import json; d=json.load(open('tasks.json')); ids=[t['id'] for t in d['tasks']]; print('TASK-049 (BG-5):', 'TASK-049' in ids, '| TASK-050 (BG-6):', 'TASK-050' in ids)"

# CQ-02: TECH_DEBT.md создан с 30 пунктами
test -f .planning/TECH_DEBT.md && echo "OK: TECH_DEBT.md exists" && grep -c "defer\|accept\|fix" .planning/TECH_DEBT.md | awk '{print "Disposition entries:", $1}'

# CQ-03: Отчёт статического анализа
test -f .planning/STATIC_ANALYSIS_REPORT.md && echo "OK: STATIC_ANALYSIS_REPORT.md exists"

# CQ-04: Отчёт покрытия
test -f .planning/COVERAGE_REPORT.md && echo "OK: COVERAGE_REPORT.md exists"

# CQ-05: Стратегия рефакторинга
test -f .planning/DIALOGUE_NODE_REFACTORING.md && grep -q "AgentRunner" .planning/DIALOGUE_NODE_REFACTORING.md && echo "OK: DIALOGUE_NODE_REFACTORING.md with components"

# CQ-06: STUB маркеры в коде
grep -rn "# STUB:" src/rob_box_mcp_tools/rob_box_mcp_tools/ src/rob_box_voice/rob_box_voice/ --include="*.py" | wc -l | awk '{if ($1 >= 4) print "OK: " $1 " STUB markers"; else print "FAIL: only " $1}'

# CQ-06: STUB задачи в трекере
python3 -c "import json; d=json.load(open('tasks.json')); stubs=[t for t in d['tasks'] if t['category']=='stub']; print('Stub tasks:', len(stubs), [t['id'] for t in stubs])"

# Финальный JSON-тест
python3 -c "import json; d=json.load(open('tasks.json')); print('Total tasks:', len(d['tasks']), '| JSON: VALID')"
```
</verification>

<success_criteria>
- [ ] `.planning/TECH_DEBT.md` создан: 30 пунктов, каждый имеет severity и disposition
- [ ] `tasks.json` содержит TASK-049 (BG-5, critical) и TASK-050 (BG-6, medium)
- [ ] `.planning/STATIC_ANALYSIS_REPORT.md` создан: flake8 (~4288 нарушений), black, isort
- [ ] `.planning/COVERAGE_REPORT.md` создан: ≥14 модулей ниже 50%, объяснение причин
- [ ] `.planning/DIALOGUE_NODE_REFACTORING.md` создан: 6 компонентов с интерфейсами, порядок реализации
- [ ] `grep -rn "# STUB:" src/ --include="*.py" | wc -l` ≥ 4
- [ ] `tasks.json` содержит TASK-051 (STUB get_robot_status) и TASK-052 (STUB navigation commands)
- [ ] `python3 -c "import json; json.load(open('tasks.json'))"` — без ошибок (JSON валиден)
- [ ] `python3 -m black --version` — black установлен
- [ ] `python3 -m isort --version` — isort установлен
</success_criteria>

<output>
После выполнения всех задач создать `.planning/phases/03-code-quality-review/03-SUMMARY.md`:

```markdown
---
phase: 03-code-quality-review
completed: {дата}
tasks_completed: 7
commits: {список SHA}
---

# Summary: Phase 3 — Code Quality Review

## Что сделано

- TECH_DEBT.md: 30 пунктов CONCERNS.md с severity и disposition
- tasks.json: добавлены BG-5 (TASK-049), BG-6 (TASK-050), STUB-050, STUB-051
- STATIC_ANALYSIS_REPORT.md: flake8 {N} нарушений, black {N} файлов, isort {N} файлов
- COVERAGE_REPORT.md: 14 модулей ниже 50%, 6 с 0% покрытием
- DIALOGUE_NODE_REFACTORING.md: стратегия декомпозиции 2040→350 строк (6 компонентов)
- 4+ # STUB: маркера в system.py и command_node.py

## Ключевые решения

- dialogue_node.py рефакторинг → Milestone 3 (не блокирует навигацию)
- SEC-1 (hardcoded password) → отдельный PR, не в scope Phase 3
- black/isort установлены локально через pip (не Docker)

## Следующий шаг

Milestone 1 завершён. Следующий: Milestone 2 — Навигация (AprilTag, Nav2, OAK-D).
```
</output>
