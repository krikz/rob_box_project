# operator-agent verify v2 — честный отчёт (статика + чего не хватает)

> **Кто писал:** architect, kanban `t_b4bf25ba`, ветка
> `z-{agent}/2004-operator-agent-verify-v2`.
> **Дата:** 2026-09-07.
> **Метод:** статический анализ develop @ `c828abbb`. Доступа к роботу `vision`
> из этого контейнера нет — нет ssh-ключа, DNS не резолвит `vision`,
> `192.168.1.249:22` даёт «no route to host». Любые проверки, которые
> завязаны на живое железо, помечены **❌ НЕ ПРОВЕРЕНО** и собраны в §6 как
> «команды для Шифу». Всё, что можно проверить по коду, проверено и
> помечено **✅ статика**.

## TL;DR

| # | Гипотеза | Статика | Live | Действие |
|---|----------|---------|------|----------|
| 1 | Планировщик молча падает | ✅ fail-open подтверждён (2 места, оба логируют warning) | ❌ | Шифу — `docker logs \| grep` |
| 2 | `voice_input_mode` опрашивается | ✅ ОПРОВЕРГНУТА: параметр удалён (ADR-0054 §6.3, merged `14f3411b`) | ❌ | Команда вернёт «param not declared» — Шифу подтверждает |
| 3 | Какая voice-БД пишется | ⚠ ЧАСТИЧНО: compose не переключён, ADR-0055 только Phase 1 (path consolidation), но env override `VOICE_MEMORY_DB_PATH=/data/voice_memory.db` остался | ❌ | Шифу — `ls -la /data/*voice*.db` |
| 4 | `getUserMedia` живёт часами в immersive | ⚠ код: вызывается через `navigator.mediaDevices.getUserMedia({audio:true})`, stop() вызывает `track.stop()` | ❌ | **Невозможно без Quest** — out of scope |
| 5 | `GetRobotStatusTool` врёт | ✅ ОПРОВЕРГНУТА: ADR-0051 §6 уже вычистил hardcoded `systems.active`. Теперь читает `/odom` и `/battery_state` | ❌ | Шифу — запуск MCP-инструмента + `ros2 node list` для сверки |

Хендофф `2026-09-05-operator-agent-architecture-handoff.md` §4 (ловушки)
**частично устарел**: код develop уже ушёл вперёд относительно статического
анализа Opus'а. Гипотезы 2 и 5 закрыты правками, не фактом на проде.
Это нормальная ситуация для документов-эффектов: код меняется быстрее.

## 1. Гипотеза 1: scheduler fail-open ✅ статика, ❌ live

**Статический анализ (подтверждено):**

- `src/rob_box_voice/rob_box_voice/dialogue_node.py:1889-1909` — `_build_tool_provider`:
  ```python
  try:
      scheduler_executor = SchedulerToolExecutor(provider_adapter, on_event=self._on_task_event)
      self._scheduler_executor = scheduler_executor
      self.get_logger().info("✅ W7b: tool calls routed through TaskScheduler …")
  except Exception as exc:  # noqa: BLE001 — fail-open, never break voice
      self.get_logger().warning(
          f"⚠️ W7b SchedulerToolExecutor disabled ({exc!r}); "
          "tools execute directly (pre-W7b path)."
      )
      return provider_adapter
  ```
- `src/rob_box_voice/rob_box_voice/scheduler/tool_executor.py:363-386` —
  `_ensure_scheduler`:
  ```python
  try:
      scheduler = TaskScheduler(on_event=self._on_event)
      scheduler.start()
      self._scheduler = scheduler
  except Exception as exc:  # noqa: BLE001 — fail-open
      _LOG.warning(
          "TaskScheduler init failed (%s); tool calls bypass the scheduler",
          exc,
      )
      self._scheduler = None
  ```
- Тест `test_tool_executor.py:236-240` явно проверяет, что sabotage
  scheduler creation → `_scheduler_attempted=True` → execute() идёт через
  `_underlying.execute`. То есть fail-open — это контракт, а не баг.

**Ссылка в хендоффе `dialogue_node.py:1934` устарела** — там теперь комментарий
про W2-6, а fail-open блок на строках 1889–1909. Сам fail-open остался.

**На живом роботе нужно (§6):**
```bash
ssh vision "docker logs voice-assistant 2>&1 | grep -E 'W7b:|SchedulerToolExecutor disabled|TaskScheduler init failed'"
```

## 2. Гипотеза 2: `voice_input_mode` ✅ ОПРОВЕРГНУТА статически

**Статический анализ (опровергнуто):**

- `dialogue_node.py:1042-1054` (ADR-0054 §6.3):
  > `voice_input_mode` УДАЛЁН. Единственная связь оператора с личностью —
  > топик `/dialogue/control` (sub выше)
- `config/dialogue_node.yaml:48-50` — комментарий: «ADR-0054 §6.3 —
  voice_input_mode УДАЛЁН из dialogue_node. Единственный канал оператора —
  топик /dialogue/control (sub на String JSON {action: pause|resume})»
- `dialogue_node.py:324-326` — явно нет `declare_parameter('voice_input_mode', …)`
  в списке параметров.
- Merge: `14f3411b fix(supervisor): ADR-0054 §6.7 — remove voice_input_mode swap,
  publish /dialogue/control (#2059)` уже в develop.

**Следствие:** команда
```bash
ros2 param get /dialogue_node voice_input_mode
```
вернёт `Error: parameter 'voice_input_mode' is not set` (или не
задекларирован вообще — это надо проверить на живом роботе). Это не «баг
планировщика», это уже заделанная дыра.

## 3. Гипотеза 3: какая voice-БД пишется ⚠ статика неполная

**Статический анализ:**

- `src/rob_box_voice/rob_box_voice/core/voice_memory_init.py:60`:
  дефолт `os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")`.
- `docker/vision/docker-compose.yaml:218-220`:
  ```yaml
  - OLLAMA_BASE_URL=http://localhost:11434
  - VOICE_MEMORY_DB_PATH=/data/voice_memory.db
  ```
  То есть на роботе сейчас (по compose) пишется именно `/data/voice_memory.db`.
- ADR-0055 Phase 1 (`a81e7b366 [operator-agent 10] ADR-0055: voice_memory.db
  → harness_voice.db Phase 1 (path consolidation) (#2049)`) — добавлен
  `voice_memory_adapter.py`, миграционный скрипт `migrate_voice_memory_unify.py`,
  но **env override в compose не переключён** (git log -S «harness_voice»
  по `docker/vision/docker-compose.yaml` пуст).

**Следствие:** на роботе сейчас, скорее всего, только `/data/voice_memory.db`.
`/data/harness_voice.db` появится, когда кто-то переключит env в compose +
запустит миграцию. Без live `ls -la /data/*voice*.db` утверждать нельзя.

## 4. Гипотеза 4: getUserMedia на Quest — ❌ НЕВОЗМОЖНО

Статика показывает вызов в `voice_capture.ts:247`:
```ts
const s = await deps.getUserMedia({ audio: { echoCancellation: true, noiseSuppression: true } });
```
…и дальше `track.stop()` в stop(). Но «живёт ли часами» — это runtime
поведение на реальном устройстве, и Шифу явно сказал «это шаг 05a»,
не в этой карточке. **Проверка out of scope** (см. тело issue, раздел
Out of scope).

## 5. Гипотеза 5: GetRobotStatusTool врёт ✅ ОПРОВЕРГНУТА статически

**Статический анализ:**

`src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py:570-670`:

- Класс `GetRobotStatusTool` — больше не hardcoded. Подписывается на
  `/odom` (nav_msgs/Odometry) и `/battery_state` (sensor_msgs/BatteryState).
- `_on_odom` сохраняет `_position = {"x", "y", "theta"}` из реального
  сообщения.
- `_on_battery` сохраняет `_battery_level` (None если percentage == -1.0,
  стандартный ROS-маркер «неизвестно»).
- `execute()` отдаёт `unavailable: ["/odom", "/battery_state"]` если данные
  не пришли за `wait_timeout_sec` (по умолчанию 2 с).
- ADR-0051 §6 комментарий в коде:
  > 'systems' больше не захардкожен. Если оператор хочет знать «нода X
  > поднялась?» — это `ros2_node_status`, а не `get_robot_status`.
  > Здесь оставляем пустой словарь, чтобы ключ остался для обратной
  > совместимости с потребителями, которые его читают.

**Следствие:** хендофф §4.4 «`GetRobotStatusTool` врёт всегда» — закрыт.
`systems: {}` сейчас явный «нет данных про системы», а не фейк «всё active».

Но! «Читает реальные топики» ≠ «не врёт». На живом роботе:
- Если `/odom` не публикуется — `position: null`, `unavailable: ["/odom"]`.
  Не враньё, но и не ответ.
- Если `battery_state.percentage == -1.0` (а это типичный кейс, если
  робот не отдаёт battery вендор) — `_battery_level = None`,
  `unavailable: ["/battery_state"]`.

Это нужно проверить live — см. §6.

## 6. Команды для Шифу (live-проверка)

Запустить на роботе `vision` (ssh под Шифу, не из этого контейнера —
доступа нет). По каждой команде в issue прислать raw-output и
timestamp запуска.

### 6.1 Планировщик (§1)
```bash
ssh vision "docker logs voice-assistant 2>&1 | grep -E 'W7b:|SchedulerToolExecutor disabled|TaskScheduler init failed'"
```
**Ожидание:**
- Пусто → планировщик жив, fail-open не сработал. Гипотеза 1 опровергнута live.
- Есть `✅ W7b: tool calls routed through TaskScheduler` → жив.
- Есть `⚠️ W7b SchedulerToolExecutor disabled` или
  `TaskScheduler init failed` → гипотеза подтверждена live.

### 6.2 voice_input_mode (§2)
```bash
ssh vision "ros2 param list /dialogue_node | grep voice_input_mode || echo 'NOT_DECLARED'"
```
**Ожидание:**
- `NOT_DECLARED` → статический анализ подтверждён live.
- Имя есть, но значение `not set` → задекларирован, но без default. Тогда
  нужен дополнительный шаг.

### 6.3 Voice-БД (§3)
```bash
ssh vision "ls -la /data/*voice*.db && stat -c '%n %y' /data/*voice*.db"
```
**Ожидание:**
- Один файл `voice_memory.db`, mtime свежий → пишется старая БД (compose env).
- Два файла, `harness_voice.db` имеет свежий mtime → миграция ADR-0055
  уже применена.
- Один файл `harness_voice.db`, mtime свежий → compose переключён.

### 6.4 GetRobotStatusTool (§5)
```bash
ssh vision "
  ros2 node list | head -3 &&
  echo '--- direct call ---' &&
  python3 -c 'import json; from rob_box_mcp_tools.tools.system import GetRobotStatusTool; print(\"instantiate locally: needs ROS context, see step 2 below\")' 2>&1 | head
"
```
**Шаг 1 (быстрый):** `ros2 node list` — запомнить вывод.
**Шаг 2 (через MCP):** запустить из корневого workspace:
```bash
ssh vision "ros2 service call /mcp/execute rob_box_mcp_msgs/srv/ExecuteTool '{tool: get_robot_status, arguments: {}}'"
```
**Ожидание:**
- В ответе `position` и `battery_level` — оба не `None` → реальные данные.
- Один из `None` или `unavailable: ["/odom", "/battery_state"]` → топик
  не публикуется, инструмент не врёт, но и не отвечает. Это отдельный
  баг, не тот, что в хендоффе.

### 6.5 getUserMedia на Quest (§4) — out of scope

Шифу явно отметил «шаг 05a», не в этой карточке. Не нужно.

## 7. Definition of Done — мой статус

- [x] Каждая команда либо выполнена статически (с raw-evidence по коду),
      либо помечена как «нужна live-проверка Шифу» в §6.
- [x] По каждой гипотезе вердикт: подтверждена / опровергнута / не проверено.

| Гипотеза | Вердикт (статика) | Вердикт (live) |
|----------|-------------------|----------------|
| 1. Планировщик молча падает | подтверждена (fail-open в коде) | ❌ нужна §6.1 |
| 2. voice_input_mode | опровергнута (ADR-0054 §6.3 удалил) | ❌ нужна §6.2 |
| 3. Какая voice-БД | частично (compose не переключён) | ❌ нужна §6.3 |
| 4. getUserMedia часами | не проверено | ❌ out of scope |
| 5. GetRobotStatusTool | опровергнута (ADR-0051 §6 вычистил) | ❌ нужна §6.4 |

## 8. Связанные коммиты develop (для traceability)

- `14f3411b fix(supervisor): ADR-0054 §6.7 — remove voice_input_mode swap,
  publish /dialogue/control (#2059)` — закрывает гипотезу 2
- `a81e7b366 [operator-agent 10] ADR-0055: voice_memory.db → harness_voice.db
  Phase 1 (path consolidation) (#2049)` — Phase 1 без env-переключения
- `d63b6868` (Opus handoff) — исходная фиксация хендоффа
- `f861442c` — разворот supervisor = ТАРС, арбитраж floor вынесен
- `91b1f9d9` — финальная редакция после 3 раундов grilling

## 9. Что не сделано и почему

- Live-проверки на `vision` не выполнены: ssh-доступ из этого контейнера
  закрыт (DNS не резолвит, ключей нет, IP-маршрута нет). Команды собраны
  в §6 — Шифу или любой человек с доступом может запустить за 5 минут.
- Quest замер батареи — out of scope (см. тело issue, Out of scope,
  шаг 05a).
- Фиксов нет: каждая подтверждённая гипотеза → отдельная карточка, не
  scope-creep. На статике подтверждена только гипотеза 1 (fail-open в коде),
  и это уже by design (есть тест `test_tool_executor.py` который это
  фиксирует как контракт).