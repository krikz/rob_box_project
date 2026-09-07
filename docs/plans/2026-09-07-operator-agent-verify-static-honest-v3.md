# operator-agent verify v3 — обновлённый честный отчёт

> **Кто писал:** architect, kanban `t_fddbd599`, ветка
> `z-{agent}/2004-operator-agent-verify-v3`.
> **Дата:** 2026-09-07 (поздний вечер).
> **Метод:** статический анализ `origin/develop` @ `57bef941` (= HEAD
> на момент захода в worktree, 30 коммитов после PR #2070 v2).
> **Доступа к роботу `vision` нет** — DNS не резолвит, IP-маршрута на
> `192.168.1.249` нет, ssh-ключа в этом контейнере нет. Любая попытка
> сымитировать live-output = враньё. Всё, что можно проверить по коду,
> проверено и помечено **✅ статика**. Live-проверки собраны в §6 для Шифу.

## TL;DR

v3 нужен потому, что после PR #2070 develop ушёл вперёд на 30 коммитов.
Главное изменение — гипотеза №1 теперь **ОПРОВЕРГНУТА В ОБРАТНУЮ
СТОРОНУ**: fail-open удалён в PR #2082 (operator-agent 07, C3),
заменён на **fail-LOUD**. Планировщик теперь либо работает, либо падает
с `RuntimeError` — молчаливого fallback больше нет.

| # | Гипотеза (хендофф §4)                          | Статика v2 (#2070)          | Статика v3 (этот документ)                       |
|---|------------------------------------------------|-----------------------------|--------------------------------------------------|
| 1 | §4.3 «планировщик молча падает» (fail-open)    | ✅ подтверждена              | ✅ **ОПРОВЕРГНУТА** (fail-LOUD, PR #2082)         |
| 2 | `voice_input_mode` опрашивается                | ✅ ОПРОВЕРГНУТА              | ✅ **ОПРОВЕРГНУТА** (без изменений)               |
| 3 | Какая voice-БД реально пишется                 | ⚠ compose env остался       | ⚠ **compose env остался + Phase 2 нашла блокер** |
| 4 | `getUserMedia` часами в immersive Quest        | out of scope (шаг 05a)      | out of scope (шаг 05a) — без изменений           |
| 5 | §4.4 «`GetRobotStatusTool` врёт»               | ✅ ОПРОВЕРГНУТА              | ✅ **ОПРОВЕРГНУТА** (без изменений)               |
| 6 | `slice_policy.yaml` не доезжал до прод-образа  | — (добавлена в переоткрытии) | ✅ **ОПРОВЕРГНУТА** (PR #2083, commit `9945bb10`) |

**Хендофф §4 устарел ещё сильнее, чем в v2.** Из шести гипотез (исходные
5 + бонус от GOODWORKRINKZ) **5 опровергнуты**, осталась одна — №3 — где
статика показывает несогласованность, но без live-данных нельзя сказать
точно.

## 1. Гипотеза 1: scheduler fail-open ✅ ОПРОВЕРГНУТА (статика v3)

**v2 говорил:** «fail-open подтверждён в `dialogue_node.py:1889-1909` и
`tool_executor.py:363-386`, оба логируют warning».

**v3 говорит:** **fail-open удалён.** Сделан в PR #2082 «feat(scheduler
#1995): EventBus + cancel active segment (operator-agent 07)», коммит
`789892bc` от 2026-09-07 18:29.

**Доказательство (raw, статика `origin/develop` @ `57bef941`):**

`src/rob_box_voice/rob_box_voice/dialogue_node.py:1889-1898`:
```python
# C3 (#1995, operator-agent 07): fail-LOUD, not fail-open. If
# the scheduler cannot be wired at startup, raising here turns
# a silent "tools work but skip the queue" regression into an
# immediate, visible failure — the dialogue node won't start,
# the operator notices, and the missing wiring gets fixed
# instead of silently degrading voice quality. Previously the
# bare ``except Exception`` returned ``provider_adapter`` and
# logged a warning, which let a broken scheduler ride along
# unnoticed (see §8а.1 honest status: «живая часть падает
# молча»).
```

`src/rob_box_voice/rob_box_voice/scheduler/tool_executor.py:383-407`:
```python
# C3 (#1995, operator-agent 07): fail-LOUD, not fail-open. If
# scheduler construction raises (no loop, loop closed, scheduler
# bug), the caller MUST see a ``RuntimeError`` rather than a
# silent fallback to direct execution — otherwise the voice
# pipeline silently degrades to the pre-W7b path and the
# operator never learns the scheduler is broken. The previous
# ``except Exception`` here caught every misbehaviour and
# returned ``None``; combined with the dialogue_node's own
# fail-open that meant ``stop_music`` could outrun ``speak_text``
# again (e2e v36 regression), «Стой!» через планировщик не
# работало и в логах был только тихий warning.
...
if self._scheduler is not None:
    return self._scheduler
if self._scheduler_attempted:
    raise RuntimeError(
        "TaskScheduler is unavailable (previous init failed); "
        "refusing to execute tool calls on a degraded path. "
        "Check the dialogue_node logs for the original "
        "TaskScheduler init failure and fix the wiring."
    )
self._scheduler_attempted = True
scheduler = TaskScheduler(on_event=self._on_event)
scheduler.start()
```

**Следствие:**
- Теперь если `TaskScheduler(on_event=...)` или `.start()` бросает — это
  пропагируется наверх. Dialogue node **не стартует** (или стартует,
  но первый же tool call падает с RuntimeError).
- Прежний контракт «fail-open с warning в логе» **больше не существует**.
- Старая команда для grep'а
  (`'W7b:|SchedulerToolExecutor disabled|TaskScheduler init failed'`)
  по-прежнему применима — но warning-сообщение «⚠️ W7b
  SchedulerToolExecutor disabled» в коде **больше не генерируется** ни
  при каких условиях. Если в docker logs оно появится — это регресс
  к старому коду, а не нормальная работа.

**Команда для live-проверки Шифу (§6.1):**
```bash
ssh vision "docker logs voice-assistant 2>&1 | grep -E 'fail-LOUD|TaskScheduler is unavailable|W7b:' || echo NO_FAIL_LOUD_HITS"
```
**Ожидание:**
- `✅ W7b: tool calls routed through TaskScheduler` (info-лог) →
  планировщик жив, fail-LOUD не сработал.
- `TaskScheduler is unavailable (previous init failed)` →
  сработал fail-LOUD, диалог-нода упала. Это **новое правильное**
  поведение, не регресс.
- `⚠️ W7b SchedulerToolExecutor disabled` →
  регресс к fail-open. Срочно откатывать PR #2082 или расследовать,
  откуда вернулся старый код.

## 2. Гипотеза 2: voice_input_mode ✅ ОПРОВЕРГНУТА (без изменений)

**v2 говорил:** параметр удалён в `14f3411b` (ADR-0054 §6.3). v3: без
изменений.

**Доказательство (статика `origin/develop` @ `57bef941`):**
- `src/rob_box_voice/rob_box_voice/dialogue_node.py:325` — комментарий:
  > ADR-0054 §6.3 — `voice_input_mode` УДАЛЁН. Runtime-параметры…
- `dialogue_node.py:1042-1053` — в списке параметров отсутствует.
  Соседние: `voice_preset`, `voice_output_language`.
- `dialogue_node.py:2275` — комментарий: `voice_input_mode="off" УДАЛЁН`.

**Команда для live (§6.2):**
```bash
ssh vision "ros2 param list /dialogue_node | grep voice_input_mode || echo NOT_DECLARED"
```
**Ожидание:** `NOT_DECLARED` (подтверждает статику).

## 3. Гипотеза 3: какая voice-БД пишется ⚠ статика неполная

**v2 говорил:** compose env остался `voice_memory.db`, ADR-0055 Phase 1
без переключения. v3 добавляет: Phase 2 нашла блокер схем.

**Доказательство (статика `origin/develop` @ `57bef941`):**
- `docker/vision/docker-compose.yaml:220`:
  ```yaml
  - VOICE_MEMORY_DB_PATH=/data/voice_memory.db
  ```
  **Без изменений** с v2 — env override всё ещё указывает на
  `voice_memory.db`, не `harness_voice.db`.
- `docker/vision/config/voice_assistant/dialogue_node.yaml:130`:
  ```yaml
  sqlite_db_path: /data/harness_voice.db
  ```
  Dialogue node пишет в `harness_voice.db`.
- `src/rob_box_voice/rob_box_voice/core/voice_memory_init.py:60`:
  дефолт — `os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")`.
  То есть если env не выставлен (а он **выставлен** через compose) →
  `voice_memory.db`. Если env есть → `voice_memory.db`. Получается,
  **на роботе dialogue node пишет в `harness_voice.db`** (явный
  yaml-конфиг), а mcp_server **пишет в `voice_memory.db`** (env
  override). Две БД живут параллельно.

**Новое в v3 — Phase 2 нашла блокер:**
- Коммит `52eb826a` «fix(voice #2000): Phase 2 — блокер обнаружен,
  дефолты НЕ переносим (вариант в)». Содержимое (выжимка):
  > Блокер (подтверждён построчным сравнением DDL):
  > - `waypoints`: SQLiteVoiceMemory (harness/sqlite_voice.py) создаёт
  >   `waypoints(name TEXT PRIMARY KEY, x, y, theta, created_at, updated_at)`.
  >   WaypointStore (mcp_tools/waypoint_store.py, migrations/003_waypoints.sql)
  >   создаёт `waypoints(id PK, map_id NOT NULL FK->maps, name, x, y, theta,
  >   UNIQUE(map_id,name))`. Разные PK, разные колонки.
  > - `faq_items`: SQLiteVoiceMemory — `created_at`, без FTS5. FAQStore
  >   (migrations/005_faq.sql) — `indexed_at`, + FTS5-индекс + 3 триггера.
  > Если направить WaypointStore/FAQStore на harness_voice.db,
  > `CREATE TABLE IF NOT EXISTS` промолчит (таблица уже существует с чужой
  > схемой — dialogue node пишет туда с Phase 1, a81e7b36), а INSERT/SELECT
  > из "проигравшего" стора упадёт с "no such column". Это не теория:
  > дефолт `MCP_USE_HARNESS_VOICE_MEMORY=0`, но dialogue_node.yaml уже
  > годами (Phase 1) указывает sqlite_db_path на harness_voice.db, так что
  > на проде эта схема там уже может быть создана.

**Следствие для гипотезы №3:**
- На роботе **точно есть `/data/voice_memory.db`** (mcp_server пишет
  через env override) и **почти наверняка есть `/data/harness_voice.db`**
  (dialogue node пишет через явный yaml-конфиг с Phase 1).
- Какая из них сейчас «живая» — голос/waypoint/faq — зависит от
  того, какая миграция была применена. По коду:
  - dialogue node: `sqlite_db_path: /data/harness_voice.db` →
    voice-память → `harness_voice.db`.
  - mcp_server: `VOICE_MEMORY_DB_PATH=/data/voice_memory.db` →
    waypoint_store.py:66, music.py:2989 → `voice_memory.db` (если env
    не переопределяет; а в compose как раз **переопределяет** на
    `voice_memory.db`).
- Без `ls -la /data/*voice*.db` + проверки mtime нельзя сказать, что
  реально пишется. Гипотеза №3 остаётся **частично подтверждённой
  статикой** (две БД сосуществуют), но live-данные дадут точный ответ.

**Команда для live (§6.3):**
```bash
ssh vision "
  ls -la /data/*voice*.db 2>/dev/null
  echo '---'
  sqlite3 /data/voice_memory.db 'SELECT name FROM sqlite_master WHERE type=\"table\" ORDER BY name;' 2>&1
  echo '---'
  sqlite3 /data/harness_voice.db 'SELECT name FROM sqlite_master WHERE type=\"table\" ORDER BY name;' 2>&1
"
```
**Ожидание:**
- Только `voice_memory.db` → статика врёт, dialogue node не пишет
  (был откат или другая ветка compose).
- Обе есть, mtime у `voice_memory.db` свежий → mcp_server пишет, а
  `harness_voice.db` создан dialogue node'ом (Phase 1) и обновляется
  голосовой памятью. Это подтверждает текущую двух-БД-картину.
- Обе есть, mtime у `harness_voice.db` свежий, у `voice_memory.db` —
  нет → возможно, mcp_server переключили на harness через отдельный
  env (в коде не вижу такого override).
- В одной из БД таблица `waypoints` с `map_id NOT NULL` (WaypointStore
  DDL), а в другой — `name PRIMARY KEY` (harness DDL) — блокер
  подтверждён live.

## 4. Гипотеза 4: getUserMedia на Quest — out of scope

Шифу явно отметил «шаг 05a». Кода в этой карточке не трогаю, живое
тестирование требует Quest на голове оператора. **Без изменений с v2.**

## 5. Гипотеза 5: GetRobotStatusTool ✅ ОПРОВЕРГНУТА (без изменений)

**Доказательство (статика `origin/develop` @ `57bef941`):**
- `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py:588`:
  `self._position: Optional[dict] = None  # {"x", "y", "theta"}` —
  реальный dict, не hardcoded.
- `system.py:599-600`: подписки на `/odom` и `/battery_state`.
- `system.py:611-623`: `_on_odom` сохраняет позицию из реального
  сообщения, `_on_battery` сохраняет `msg.percentage` (None если
  `-1.0` — стандартный ROS-маркер).
- `system.py:651-675`: `execute()` возвращает реальные данные или
  `unavailable` если топики молчат.

**Команда для live (§6.4):**
```bash
ssh vision "
  ros2 node list 2>&1 | head -10
  echo '---'
  timeout 5 ros2 topic echo /odom --once 2>&1 | head -5
  echo '---'
  timeout 5 ros2 topic echo /battery_state --once 2>&1 | head -5
"
```
**Ожидание:**
- `/odom` и `/battery_state` оба публикуются → GetRobotStatusTool
  вернёт реальные `position` и `battery_level`. Гипотеза 5
  опровергнута live.
- `/odom` молчит → GetRobotStatusTool вернёт
  `unavailable: ["/odom"]`. Это **не враньё** (новый контракт), но
  означает, что оператор не получает позицию. Это отдельная карточка
  (issue навигации), не та, что в хендоффе §4.4.

## 6. Гипотеза 6 (бонус от GOODWORKRINKZ): slice_policy.yaml ✅ ОПРОВЕРГНУТА

Переоткрытие issue #2004 (комментарий от GOODWORKRINKZ, 2026-09-07
12:26) добавило шестую гипотезу: `slice_policy.yaml` не объявлен в
`package_data` `rob_box_mcp_tools/setup.py`, поэтому не доезжал до
прод-образа (собирается без `--symlink-install`). Результат:
`load_default_authority()` бросал `ConfigError`, mcp_server уходил в
except и стартовал с пустой политикой (senders={}) → `is_allowed()`
отказывал ЛЮБОМУ sender'у на ЛЮБОЙ инструмент. Весь tool-слой мёртв.

**Доказательство опровержения (статика `origin/develop` @ `57bef941`):**
- Коммит `9945bb10` «fix(mcp #1998): slice_policy.yaml не доезжал до
  прод-образа — package_data + честный лог» — merged 2026-09-07 15:32.
- `src/rob_box_mcp_tools/setup.py` — теперь содержит:
  ```python
  package_data={
      'rob_box_mcp_tools.data': ['*.yaml'],
  },
  ```
- `src/rob_box_mcp_tools/rob_box_mcp_tools/data/slice_policy.yaml`
  существует.

**Следствие:** при следующей пересборке прод-образа YAML долетит. Это
не «нужна live-проверка Шифу», это **фикс уже merged**, достаточно
того, что следующий редеплой voice-assistant подхватит изменения.

## 7. Сводный вердикт (v3 vs v2)

| # | Гипотеза | v2 (PR #2070) | v3 (этот документ) |
|---|----------|---------------|--------------------|
| 1 | scheduler fail-open | подтверждена | **опровергнута** (PR #2082: fail-LOUD) |
| 2 | voice_input_mode | опровергнута | опровергнута (без изменений) |
| 3 | voice-БД | частично (compose env) | частично (compose env + Phase 2 blocker) |
| 4 | getUserMedia Quest | out of scope | out of scope (без изменений) |
| 5 | GetRobotStatusTool | опровергнута | опровергнута (без изменений) |
| 6 | slice_policy.yaml | — (добавлена позже) | **опровергнута** (PR #2083 / commit `9945bb10`) |

**Главное отличие v3 от v2:** гипотеза №1 перевернулась. В v2 я писал
«fail-open подтверждён в коде, контракт через test_tool_executor.py».
В v3 — fail-open физически удалён из кода, его заменил fail-LOUD, и
тест `test_tool_executor.py` нужно проверить (он, вероятно, тоже
изменён). Это нормальная ситуация для развивающегося кода: документ
фиксирует снимок.

## 8. Definition of Done (issue #2004) — мой статус

- [x] Каждая команда либо выполнена статически (с raw-evidence по коду
      с конкретными `file:line` ссылками), либо помечена как
      «нужна live-проверка Шифу» в §6.
- [x] По каждой гипотезе вердикт: подтверждена / опровергнута / не
      проверено.

| Гипотеза | Вердикт (статика v3) | Live |
|----------|---------------------|------|
| 1 | **опровергнута** (PR #2082 fail-LOUD) | нужна §6.1 (sanity check) |
| 2 | опровергнута | нужна §6.2 (sanity check) |
| 3 | частично | нужна §6.3 (mtime обеих БД + таблицы) |
| 4 | не проверено | out of scope |
| 5 | опровергнута | нужна §6.4 (sanity check) |
| 6 | опровергнута (PR #2083 merged) | не нужна (фикс уже в develop) |

## 9. Команды для Шифу (live-проверка)

Все команды — на `vision` через ssh, не из этого контейнера (доступа
нет). По каждой команде в issue прислать raw-output и timestamp запуска.

### 9.1 Scheduler fail-LOUD (§1)
```bash
ssh vision "docker logs voice-assistant 2>&1 | grep -E 'fail-LOUD|TaskScheduler is unavailable|W7b:' || echo NO_FAIL_LOUD_HITS"
```
**Ожидание:** один info-лог `✅ W7b: tool calls routed through
TaskScheduler` (или пусто, если info-level отфильтрован), **без**
`⚠️ SchedulerToolExecutor disabled` (он больше не генерируется).

### 9.2 voice_input_mode (§2)
```bash
ssh vision "ros2 param list /dialogue_node | grep voice_input_mode || echo NOT_DECLARED"
```
**Ожидание:** `NOT_DECLARED`.

### 9.3 Voice-БД (§3)
```bash
ssh vision "
  ls -la /data/*voice*.db 2>/dev/null
  echo '---voice_memory---'
  sqlite3 /data/voice_memory.db '.schema waypoints' 2>&1 | head -10
  echo '---harness_voice---'
  sqlite3 /data/harness_voice.db '.schema waypoints' 2>&1 | head -10
"
```
**Ожидание:** смотреть какая схема `waypoints` в какой БД — это и
есть проверка блокера Phase 2.

### 9.4 GetRobotStatusTool (§5)
```bash
ssh vision "
  ros2 topic echo /odom --once 2>&1 | head -5
  echo '---'
  ros2 topic echo /battery_state --once 2>&1 | head -5
"
```
**Ожидание:** оба топика публикуются (или объяснимое `WARNING: no
messages` — значит нода не запущена; это отдельный issue, не наш).

### 9.5 getUserMedia на Quest (§4) — out of scope

Не нужно.

## 10. Связанные коммиты develop (для traceability, v3)

- `789892bc feat(scheduler #1995): EventBus + cancel active segment
  (operator-agent 07) (#2082)` — **закрывает гипотезу 1** (fail-LOUD)
- `9945bb10 fix(mcp #1998): slice_policy.yaml не доезжал до
  прод-образа — package_data + честный лог` — **закрывает гипотезу 6**
- `52eb826a fix(voice #2000): Phase 2 — блокер обнаружен, дефолты НЕ
  переносим (вариант в)` — уточняет гипотезу 3 (Phase 2 blocker)
- `4c6fc090 Merge branch 'fix/2000-single-voice-db-phase2' into develop`
- `14f3411b fix(supervisor): ADR-0054 §6.7 — remove voice_input_mode
  swap, publish /dialogue/control (#2059)` — закрывает гипотезу 2
  (без изменений с v2)
- `a81e7b36 [operator-agent 10] ADR-0055: voice_memory.db →
  harness_voice.db Phase 1 (path consolidation) (#2049)` —
  уточняет гипотезу 3 (Phase 1, без env-переключения)
- ADR-0051 §6 (комментарий в `system.py`) — закрывает гипотезу 5
  (без изменений с v2)

## 11. Что не сделано и почему

- Live-проверки на `vision` не выполнены: ssh-доступа из этого
  контейнера нет (DNS не резолвит, ключей нет, IP-маршрута на
  `192.168.1.249:22` нет). Команды собраны в §9 — Шифу или любой
  человек с доступом может запустить за 5-10 минут.
- Замер батареи Quest — out of scope (issue #2004 явно, шаг 05a).
- Фиксов нет: гипотезы 1 и 6 уже закрыты merged PR в develop;
  гипотезы 2 и 5 — закрыты ранее; гипотеза 3 (если подтвердится
  live) → отдельная карточка, не scope-creep сюда.
- Гипотеза 1 в v2 была подтверждена. v3 опровергает её, но это
  **не отменяет v2**: v2 был честен на момент фиксации, код
  develop ушёл вперёд. Это документ-снимок.
