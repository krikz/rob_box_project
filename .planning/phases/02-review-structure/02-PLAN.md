---
phase: 02-review-structure
plan: 02
type: execute
wave: 1
depends_on: []
files_modified:
  - docker/vision/docker-compose.yaml
  - docker/main/docker-compose.yaml
  - src/rob_box_perception/setup.py
  - src/rob_box_bringup/setup.py
  - src/rob_box_mcp_tools/setup.py
  - src/rob_box_description/URDF_EXPORT/package.xml
  - src/rob_box_description/URDF_EXPORT/COLCON_IGNORE
  - src/led_matrix_driver/  # удаляется целиком
  - .github/workflows/G-Lint Code.yml
autonomous: true
requirements:
  - STRUCT-01
  - STRUCT-02
  - STRUCT-03
  - STRUCT-04
  - STRUCT-05

must_haves:
  truths:
    - "grep -r 'COPY config' docker/ --include='Dockerfile*' возвращает 0 результатов"
    - "grep -r 'COPY scripts' docker/ --include='Dockerfile*' возвращает 0 результатов"
    - "Все ROS-сервисы в compose-файлах имеют network_mode: host"
    - "Все ROS-сервисы имеют depends_on: zenoh-router; не-ROS исключения задокументированы комментариями"
    - "grep 'TODO' src/rob_box_perception/setup.py возвращает 0 результатов"
    - "src/led_matrix_driver/ не существует"
    - ".github/workflows/G-Lint Code.yml не содержит ссылок на src/led_matrix_driver/"
  artifacts:
    - path: "src/rob_box_description/URDF_EXPORT/COLCON_IGNORE"
      provides: "Исключение ROS 1 пакета из colcon workspace"
  key_links:
    - from: ".github/workflows/G-Lint Code.yml"
      to: "src/ros2leds/"
      via: "lint path inclusion"
      pattern: "src/ros2leds/"
---

# Phase 2: Ревью структуры — Plan

**Goal:** Docker layout и пакеты следуют единому стандарту проекта  
**Requirements:** STRUCT-01, STRUCT-02, STRUCT-03, STRUCT-04, STRUCT-05  
**Wave count:** 1  
**Estimated tasks:** 7  

---

<objective>
Фаза содержит точечные структурные правки: документирование обоснованных исключений в docker-compose,
устранение TODO из setup.py/package.xml, и удаление устаревшего дублирующего пакета src/led_matrix_driver/.

Аудит (02-RESEARCH.md) показал: STRUCT-02 уже выполнен (нарушений нет), большинство
STRUCT-01 уже соответствует стандарту. Реальный объём работ — ~10 небольших правок.

Purpose: Привести репозиторий в соответствие с задокументированными стандартами проекта.
Output: Чистый workspace без TODO, без дублирующих пакетов, с задокументированными исключениями.
</objective>

<execution_context>
@.planning/phases/02-review-structure/02-RESEARCH.md
@.planning/phases/02-review-structure/02-VALIDATION.md
@docs/development/DOCKER_STANDARDS.md
</execution_context>

<context>
@.planning/ROADMAP.md
@.planning/phases/02-review-structure/02-RESEARCH.md

**Ключевые выводы из research:**
- STRUCT-02 (COPY config/scripts) — нарушений нет, 0 результатов grep ✅
- STRUCT-01 (network_mode/depends_on) — все ROS-сервисы соответствуют, не-ROS исключения обоснованны ✅
- Исключения требуют только документирования через комментарии в compose-файлах
- Критические нарушения STRUCT-03: только rob_box_perception/setup.py (TODO) и URDF_EXPORT/package.xml
- STRUCT-05: src/led_matrix_driver/ — устаревший дубликат, Docker использует src/ros2leds/led_matrix_driver/

**Git submodule ограничения:**
- src/ros2leds/ — git submodule, файлы внутри не редактируются
- src/vesc_nexus/ — git submodule, аналогично
- placeholder emails в src/ros2leds/*/setup.py — вне scope, задокументировать в STATE.md
</context>

<tasks>

---

## Wave 1 — Structure Fixes

Все планы независимы, выполняются последовательно в рамках одной волны.

---

### Plan 02-01: Docker Audit Documentation

**Тип:** docs  
**Autonomous:** true  
**Покрывает:** STRUCT-01, STRUCT-02  

---

<task type="auto">
  <name>Task 02-01-01: Задокументировать исключения в docker/vision/docker-compose.yaml</name>
  <files>docker/vision/docker-compose.yaml</files>
  <action>
Добавить inline-комментарии над сервисами-исключениями в docker/vision/docker-compose.yaml,
объясняя почему они не имеют depends_on: zenoh-router (или network_mode).

Точные исправления:

1. Перед секцией `voice-resources-init:` добавить комментарий:
   ```
   # non-ROS service: одноразовый init-контейнер для копирования renardo samples в volume.
   # Не использует ROS/Zenoh, network_mode не требуется.
   ```

2. Перед секцией `supercollider:` добавить комментарий:
   ```
   # non-ROS service: аудиосервер SuperCollider (scsynth).
   # Не публикует/подписывается на ROS-топики, depends_on: zenoh-router не требуется.
   ```

3. Перед секцией `cadvisor:` (profile: monitoring) добавить комментарий:
   ```
   # non-ROS service: мониторинговый агент (profile: monitoring).
   # Не использует ROS/Zenoh, depends_on: zenoh-router не требуется.
   ```

4. Перед секцией `promtail:` (profile: monitoring) добавить аналогичный комментарий:
   ```
   # non-ROS service: Loki log shipper (profile: monitoring).
   # Не использует ROS/Zenoh, depends_on: zenoh-router не требуется.
   ```

5. Перед секцией `ollama:` (profile: ai) добавить комментарий:
   ```
   # non-ROS service: LLM inference server, HTTP API на localhost:11434 (profile: ai).
   # Не использует ROS/Zenoh, depends_on: zenoh-router не требуется.
   ```
  </action>
  <verify>`grep -c "non-ROS service" docker/vision/docker-compose.yaml`
  Ожидаемый результат: 5</verify>
  <done>В docker/vision/docker-compose.yaml все 5 не-ROS сервисов имеют объясняющий комментарий.</done>
</task>

<task type="auto">
  <name>Task 02-01-02: Задокументировать исключения в docker/main/docker-compose.yaml</name>
  <files>docker/main/docker-compose.yaml</files>
  <action>
Добавить inline-комментарии над сервисами-исключениями в docker/main/docker-compose.yaml.

Точные исправления:

1. Перед секцией `cadvisor:` (profile: monitoring) добавить комментарий:
   ```
   # non-ROS service: мониторинговый агент (profile: monitoring).
   # Не использует ROS/Zenoh, depends_on: zenoh-router не требуется.
   ```

2. Перед секцией `promtail:` (profile: monitoring) добавить комментарий:
   ```
   # non-ROS service: Loki log shipper (profile: monitoring).
   # Не использует ROS/Zenoh, depends_on: zenoh-router не требуется.
   ```
  </action>
  <verify>`grep -c "non-ROS service" docker/main/docker-compose.yaml`
  Ожидаемый результат: 2</verify>
  <done>В docker/main/docker-compose.yaml оба не-ROS сервиса (cadvisor, promtail) имеют объясняющий комментарий.</done>
</task>

**Коммит после 02-01:**
```
git commit -m "docs(docker): document non-ROS service exceptions in compose files"
```

---

### Plan 02-02: Python Package Fixes

**Тип:** fix  
**Autonomous:** true  
**Покрывает:** STRUCT-03, STRUCT-04  

---

<task type="auto">
  <name>Task 02-02-01: Исправить TODO в src/rob_box_perception/setup.py</name>
  <files>src/rob_box_perception/setup.py</files>
  <action>
Заменить TODO-плейсхолдеры реальными значениями, синхронизируя с package.xml.

Строки 25-26 в src/rob_box_perception/setup.py:

Заменить:
```python
    description='TODO: Package description',
    license='TODO: License declaration',
```

На:
```python
    description='Internal Dialogue Agent - Perception and Reflection for Rob Box',
    license='MIT',
```

Источник значений: src/rob_box_perception/package.xml (уже содержит корректные description/license).
  </action>
  <verify>`grep -c "TODO" src/rob_box_perception/setup.py`
  Ожидаемый результат: 0</verify>
  <done>src/rob_box_perception/setup.py не содержит ни одной строки с "TODO".</done>
</task>

<task type="auto">
  <name>Task 02-02-02: Заменить placeholder emails в setup.py</name>
  <files>
    src/rob_box_bringup/setup.py
    src/rob_box_mcp_tools/setup.py
  </files>
  <action>
Заменить placeholder email во всех редактируемых пакетах (не submodule) на `ros2@rob-box.local`.

**ВАЖНО:** src/ros2leds/ и src/vesc_nexus/ — git submodules. Их файлы НЕ редактировать.
src/led_matrix_driver/ — будет удалён в 02-03, не редактировать.

1. src/rob_box_bringup/setup.py — заменить:
   ```python
   maintainer='Your Name',
   maintainer_email='your_email@example.com',
   ```
   На:
   ```python
   maintainer='Rob Box Team',
   maintainer_email='ros2@rob-box.local',
   ```

2. src/rob_box_mcp_tools/setup.py — заменить только email (maintainer уже 'Rob Box Team'):
   ```python
   maintainer_email='your_email@example.com',
   ```
   На:
   ```python
   maintainer_email='ros2@rob-box.local',
   ```
  </action>
  <verify>`grep -r "your_email@example.com\|user@example.com" src/rob_box_bringup/ src/rob_box_mcp_tools/`
  Ожидаемый результат: пусто (0 совпадений)</verify>
  <done>
  src/rob_box_bringup/setup.py и src/rob_box_mcp_tools/setup.py содержат
  maintainer_email='ros2@rob-box.local' вместо плейсхолдеров.
  </done>
</task>

<task type="auto">
  <name>Task 02-02-03: Исправить URDF_EXPORT/package.xml и добавить COLCON_IGNORE</name>
  <files>
    src/rob_box_description/URDF_EXPORT/package.xml
    src/rob_box_description/URDF_EXPORT/COLCON_IGNORE
  </files>
  <action>
Контекст: src/rob_box_description/URDF_EXPORT/ — auto-generated Fusion 360 URDF export
с ROS 1 зависимостями (catkin, rospy). Должен быть исключён из colcon workspace.

**Шаг 1:** Исправить src/rob_box_description/URDF_EXPORT/package.xml —
заменить `<license>TODO</license>` на `<license>MIT</license>`.

Текущее содержимое строки: `  <license>TODO</license>`
Новое содержимое строки:   `  <license>MIT</license>`

**Шаг 2:** Создать пустой файл src/rob_box_description/URDF_EXPORT/COLCON_IGNORE:
```bash
touch src/rob_box_description/URDF_EXPORT/COLCON_IGNORE
```

Этот файл сигнализирует colcon о необходимости пропустить директорию при сборке workspace,
предотвращая предупреждения о ROS 1 зависимостях (catkin, rospy).
  </action>
  <verify>
  `grep "TODO" src/rob_box_description/URDF_EXPORT/package.xml; echo "exit:$?"`
  Ожидаемый результат: exit:1 (grep ничего не нашёл)

  `test -f src/rob_box_description/URDF_EXPORT/COLCON_IGNORE && echo "COLCON_IGNORE: OK"`
  Ожидаемый результат: COLCON_IGNORE: OK
  </verify>
  <done>
  URDF_EXPORT/package.xml содержит `&lt;license&gt;MIT&lt;/license&gt;`.
  URDF_EXPORT/COLCON_IGNORE существует.
  </done>
</task>

**Коммит после 02-02:**
```
git commit -m "fix(packages): fix TODO placeholders in setup.py and package.xml, add COLCON_IGNORE for URDF_EXPORT"
```

---

### Plan 02-03: Structural Cleanup

**Тип:** cleanup  
**Autonomous:** true  
**Покрывает:** STRUCT-05  

**Предварительная проверка (выполнить перед удалением):**
```bash
grep -r "ByteMultiArray\|Int8MultiArray" src/rob_box_animations/ src/rob_box_voice/
```
Ожидаемый результат: `Int8MultiArray` — это подтверждает, что rob_box_animations использует
канонический тип из src/ros2leds/led_matrix_driver/, а не из удаляемой копии.
Если grep показывает `ByteMultiArray` — задача требует уточнения перед удалением.

---

<task type="auto">
  <name>Task 02-03-01: Удалить src/led_matrix_driver/ (устаревший дубликат)</name>
  <files>src/led_matrix_driver/  (удаляется)</files>
  <action>
src/led_matrix_driver/ — standalone автономная копия LED драйвера, НЕ используется Docker.
Канонический пакет: src/ros2leds/led_matrix_driver/ (используется docker/vision/led_matrix/Dockerfile).

**Шаг 1: Финальная верификация перед удалением**
```bash
grep -r "ByteMultiArray\|Int8MultiArray" src/rob_box_animations/ src/rob_box_voice/
```
Убедиться что используется `Int8MultiArray` (из канонического ros2leds пакета).

**Шаг 2: Проверить что Docker не ссылается на standalone копию**
```bash
grep -r "led_matrix_driver" docker/ --include="Dockerfile*" | grep -v "ros2leds"
```
Ожидаемый результат: пусто (ни один Dockerfile не копирует standalone пакет).

**Шаг 3: Удалить директорию**
```bash
rm -rf src/led_matrix_driver/
```

Это удаляет: led_matrix_driver.py, setup.py, package.xml, resource/, launch/,
config/, README.md — всё содержимое standalone копии.
  </action>
  <verify>`test ! -d src/led_matrix_driver && echo "OK: removed" || echo "ERROR: still exists"`
  Ожидаемый результат: OK: removed</verify>
  <done>Директория src/led_matrix_driver/ не существует.</done>
</task>

<task type="auto">
  <name>Task 02-03-02: Обновить .github/workflows/G-Lint Code.yml</name>
  <files>.github/workflows/G-Lint Code.yml</files>
  <action>
Удалить все ссылки на удалённый src/led_matrix_driver/ из CI workflow.
Строки для удаления расположены в трёх шагах линтинга (black, flake8, isort):
строки 55, 69, 87 содержат `            src/led_matrix_driver/ \`

**Примечание:** src/ros2leds/ уже присутствует в тех же lint-шагах — она покрывает
src/ros2leds/led_matrix_driver/ и src/ros2leds/led_matrix_compositor/ автоматически.
Добавлять отдельные пути для submodule не нужно.

В каждом из трёх шагов (black --check, flake8, isort --check-only) удалить строку:
```
            src/led_matrix_driver/ \
```

Итого: удалить 3 строки из файла (по одной на каждый lint-шаг).
  </action>
  <verify>`grep "led_matrix_driver" ".github/workflows/G-Lint Code.yml" | grep -v "ros2leds"`
  Ожидаемый результат: пусто (0 совпадений — нет ссылок на удалённый путь)

  `grep "ros2leds" ".github/workflows/G-Lint Code.yml" | wc -l`
  Ожидаемый результат: ≥ 3 (src/ros2leds/ остаётся в каждом lint-шаге)</verify>
  <done>
  .github/workflows/G-Lint Code.yml не содержит строк с src/led_matrix_driver/.
  src/ros2leds/ сохранена в lint-шагах.
  </done>
</task>

**Коммит после 02-03:**
```
git commit -m "chore(src): remove obsolete src/led_matrix_driver/ duplicate, update CI lint paths"
```

</tasks>

---

<threat_model>
## Trust Boundaries

| Boundary | Description |
|----------|-------------|
| CI workflow | Изменения в .github/workflows/ влияют на все сборки в репозитории |
| git submodule | src/ros2leds/ — внешний репозиторий, нельзя изменять без PR |

## STRIDE Threat Register

| Threat ID | Category | Component | Disposition | Mitigation Plan |
|-----------|----------|-----------|-------------|-----------------|
| T-02-01 | Tampering | .github/workflows/G-Lint Code.yml | mitigate | Проверить grep после правки: убедиться что src/ros2leds/ не была случайно удалена |
| T-02-02 | Tampering | src/ros2leds/ (submodule) | accept | Не редактировать файлы внутри submodule; только удаление standalone копии |
| T-02-03 | Denial of Service | Docker compose исключения | accept | Комментарии документируют намерение — не меняют поведение runtime |
</threat_model>

<verification>

## Финальные проверки Success Criteria

```bash
# SC-1: Нет COPY config в Dockerfile-ах
grep -r "COPY config" docker/ --include="Dockerfile*" | wc -l
# Ожидаемый результат: 0

# SC-2: Нет COPY scripts в Dockerfile-ах  
grep -r "COPY scripts" docker/ --include="Dockerfile*" | wc -l
# Ожидаемый результат: 0

# SC-3a: Все ROS-сервисы имеют network_mode (семантическая проверка в RESEARCH подтверждена)
grep -c "network_mode: host" docker/main/docker-compose.yaml docker/vision/docker-compose.yaml

# SC-3b: Исключения задокументированы
grep -c "non-ROS service" docker/main/docker-compose.yaml docker/vision/docker-compose.yaml
# Ожидаемый результат: main=2, vision=5

# SC-4: Нет TODO в setup.py не-submodule пакетов
grep -r "TODO" src/ --include="setup.py" \
  --exclude-dir="ros2leds" --exclude-dir="vesc_nexus" --exclude-dir="robot_sensor_hub_msg"
# Ожидаемый результат: пусто

# SC-4b: COLCON_IGNORE создан
test -f src/rob_box_description/URDF_EXPORT/COLCON_IGNORE && echo "OK"

# SC-5: Устаревший пакет удалён
test ! -d src/led_matrix_driver && echo "OK: removed"

# SC-5b: CI не ссылается на удалённый путь
grep "led_matrix_driver" ".github/workflows/G-Lint Code.yml" | grep -v "ros2leds"
# Ожидаемый результат: пусто
```

**Submodule scope note:** placeholder emails в src/ros2leds/*/setup.py и src/vesc_nexus/*/setup.py
находятся вне scope этой фазы. Задокументировать в STATE.md: "STRUCT-03 submodule exception:
src/ros2leds/ and src/vesc_nexus/ email placeholders require PR to respective upstream repos."

</verification>

<success_criteria>

## Success Criteria Checklist

- [ ] `grep -r "COPY config" docker/ --include="Dockerfile*"` возвращает 0 результатов
- [ ] `grep -r "COPY scripts" docker/ --include="Dockerfile*"` возвращает 0 результатов
- [ ] Все ROS-сервисы в compose-файлах имеют `network_mode: host`
- [ ] Все ROS-сервисы в compose-файлах имеют `depends_on: zenoh-router`
- [ ] Не-ROS исключения задокументированы комментариями `# non-ROS service` (main: 2, vision: 5)
- [ ] `grep "TODO" src/rob_box_perception/setup.py` возвращает 0 результатов
- [ ] `grep -r "your_email@example.com\|user@example.com" src/rob_box_bringup/ src/rob_box_mcp_tools/` возвращает 0 результатов
- [ ] `src/rob_box_description/URDF_EXPORT/COLCON_IGNORE` существует
- [ ] `src/rob_box_description/URDF_EXPORT/package.xml` содержит `<license>MIT</license>`
- [ ] `src/led_matrix_driver/` не существует
- [ ] `.github/workflows/G-Lint Code.yml` не содержит ссылок на `src/led_matrix_driver/` (кроме ros2leds)
- [ ] STATE.md содержит запись об исключении submodule emails из scope

</success_criteria>

<output>
После завершения всех задач создать `.planning/phases/02-review-structure/02-SUMMARY.md`
со статусом выполнения каждого Success Criterion.
</output>
