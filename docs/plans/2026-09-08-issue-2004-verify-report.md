# Issue #2004 — отчёт верификации гипотез на живом роботе

> **Когда:** 2026-09-08, ~07:40 UTC (build host)
> **Метод:** прямые SSH-команды + пробник `GetRobotStatusTool` через Python в контейнере `voice-assistant`
> **Базис:** develop @ `3e9f72f4` (post PR #2133 sender LLMToolCallAdapter fix)
> **Контекст:** карточка-верификация для оператора-agent архитектуры, раздел §6C хендоффа 2026-09-05.

Каждой гипотезе — одна команда из карточки, raw-вывод (фрагмент, не полный дамп) и вердикт.

---

## H1. Статус планировщика (`scheduler` skill)

**Команда** (хендофф §4.3):
```
ssh vision "docker logs voice-assistant 2>&1 | grep -E 'W7b:|SchedulerToolExecutor disabled|TaskScheduler init failed'"
```

**Результат** — пусто по всем трём паттернам. Расширил поиск:
```
$ docker logs voice-assistant --tail 5000 2>&1 | grep -iE "scheduler|tool_executor|ensure_scheduler|W7b|EventBus|priority"
[dialogue_node-4] [INFO] 🧩 Загружено фрагментов скиллов: 12 (..., scheduler, ...)
[dialogue_node-4] [INFO] 🧭 Пред-роутер скиллов включён: ..., scheduler, ...
[dialogue_node-4] [INFO] ✅ W7b: tool calls routed through TaskScheduler (voice/music/anim channels; stop_music deferred).
```

**Вердикт: ОПРОВЕРГНУТА.**
Skill scheduler загружен, W7b-роутер активен, маршрут через TaskScheduler подтверждён. Нет ни `SchedulerToolExecutor disabled`, ни `TaskScheduler init failed`. Fail-open в `dialogue_node.py:1934` / `tool_executor.py::_ensure_scheduler` на этой ревизии не сработал. Гипотеза handoff §4.3 (написанная по состоянию на 91b1f9d9, 4 дня назад) уже не актуальна — судя по коммиту `d63b6868`/`f861442c`/`91b1f9d9` в handoff §1, scheduler был починен одним из последующих PR (Phase 1 PR #2070/2075 или последующих).

---

## H2. Реальное значение `voice_input_mode`

**Команды**:
```
$ ros2 param get /dialogue_node voice_input_mode
... zenoh init ...
Parameter not set

$ ros2 param list /dialogue_node | grep -i voice_input_mode
(пусто)
```

Также из логов:
```
[dialogue_node] [WARN] Failed to get parameters: ('Invalid access to undeclared parameter(s)', 'voice_input_mode')
[dialogue_node] [WARN] Failed to describe parameters: ('Invalid access to undeclared parameter(s)', 'voice_input_mode')
```

**Вердикт: НЕПРИМЕНИМА.**
Параметр `voice_input_mode` **не существует** в `dialogue_node` — ни объявлен, ни имеет значения. Это согласуется с целевой архитектурой §7.3, которая предписывает удалить `voice_input_mode` со всеми шестью значениями. Похоже, удаление уже произошло в одном из PR между develop @ `91b1f9d9` и текущим HEAD @ `3e9f72f4` (30 коммитов). Гипотеза «значение по умолчанию = respeaker» из handoff §4.1 (как риск) **уже неактуальна** — параметра нет в принципе, гонки не возникают.

---

## H3. Какая voice-БД реально пишется

**Команды**:
```
$ ls -la --time-style=full-iso /data/*.db
-rw-r--r-- 1 root root  303104 2026-09-08 04:37:17.282603576 +0300 /data/harness_voice.db
-rw-r--r-- 1 root root       0 2026-08-09 19:04:09.588817585 +0300 /data/memory.db
-rw-r--r-- 1 root root    4096 2026-09-08 02:10:35.461995910 +0300 /data/operator_memory.db
-rw-r--r-- 1 root root       0 2026-04-13 19:11:36.579531046 +0300 /data/rob_box_voice.db
-rw-r--r-- 1 root root   94208 2026-09-07 19:32:07.725378870 +0300 /data/speakers.db
-rw-r--r-- 1 root root       0 2026-06-12 11:51:38.374958058 +0300 /data/voice_assistant.db
-rw-r--r-- 1 root root 8577024 2026-09-01 15:31:46.041936155 +0300 /data/voice_memory.db
-rw-r--r-- 1 root root       0 2026-03-01 02:29:06.515618940 +0300 /data/waypoints.db
```

Карточка говорила про **две voice-БД** — на роботе **пять БД** в `/data`. Уточнённая проверка через FD процессов:

```
$ docker exec voice-assistant bash -c "ls -la /proc/\$(pgrep -f dialogue_node | head -1)/fd | grep -iE '\.db|sqlite'"
lrwx------ 1 root root 64 Sep  8 10:40 40 -> /data/harness_voice.db
lrwx------ 1 root root 64 Sep  8 10:40 41 -> /data/harness_voice.db-wal
lrwx------ 1 root root 64 Sep  8 10:40 42 -> /data/harness_voice.db-shm

$ docker exec voice-assistant bash -c "ls -la /proc/\$(pgrep -f mcp_server | head -1)/fd | grep -iE '\.db|sqlite'"
... /data/voice_memory.db (+ -wal, -shm) ...   # 4 fd на одну БД — WAL активен
... /data/music_library/index.db
```

**Вердикт: УТОЧНЕНО (гипотеза частично верна, но картина шире).**

| БД | Кто открыл | mtime | Статус |
|---|---|---|---|
| `harness_voice.db` | dialogue_node | сегодня 04:37 | **пишется** (303 KB, WAL активен) |
| `operator_memory.db` | (не виден в fd выборке) | сегодня 02:10 | пишется (4 KB) |
| `voice_memory.db` | mcp_server | 2026-09-01 (неделю назад) | **мёртвая**, размер 8.5 MB |
| `voice_assistant.db` | никто | 2026-06-12 (3 мес) | мёртвая (0 байт) |
| `rob_box_voice.db` | никто | 2026-04-13 (5 мес) | мёртвая (0 байт) |
| `speakers.db` | (вероятно speaker_id_node) | 2026-09-07 | пишется (94 KB) |
| `music_library/index.db` | mcp_server | не измерено | пишется |

Гипотеза «пишется только одна из двух voice-БД» в целом **подтверждается**, но не «voice_assistant», а **`harness_voice.db`**, открытая именно `dialogue_node` (а не mcp_server). ADR-0055 (voice-memory-db-unify-with-harness) уже применён — старая `voice_memory.db` (mcp_server) не пишется, её заменил `harness_voice.db`.

**Рекомендация (не правка, а замечание):** три нулевых файла (`voice_assistant.db`, `rob_box_voice.db`, `memory.db`, `waypoints.db`) — мёртвые артефакты после миграций. Очистка — отдельная карточка вне scope верификации.

---

## H4. `getUserMedia` в immersive-сессии Quest

**Статус: НЕ ПРОВЕРЕНО (out of scope).**

Карточка явно выводит замер за пределы: «Out of scope: Замер батареи Quest (это шаг 05а)». Проверка требует физического Quest-устройства и многочасовой immersive-сессии.

Косвенные сигналы (статический анализ кода):
- `src/rob_box_quest/webxr_client/src/input/voice_capture.ts:247` — `navigator.mediaDevices.getUserMedia({audio: ...})` создаёт стрим на старте voice capture.
- `voice_capture.ts:12` — комментарий «getUserMedia + AudioWorklet (real-time audio thread) → resample 48k→16k».
- Issue #1992 (зафиксирован в `main.ts:525`) — ранее был fail-silent при отказе getUserMedia/AudioWorklet, починен до текущего HEAD.
- Шаг 5а архитектурного плана остаётся единственным путём верификации.

**Вердикт: НЕ ПРОВЕРЕНО** (требует физического Quest, отдельная карточка шага 5а).

---

## H5. `GetRobotStatusTool` врёт

**Метод.** MCP-server не выставляет инструменты как ROS-сервисы. Подключился через пробник: инициализация rclpy-ноды внутри контейнера `voice-assistant`, создание `GetRobotStatusTool(node)`, spin 6 секунд (wait_timeout=3.0s + буфер), `tool.execute()`.

```
$ python3 /tmp/probe_status_tool.py   # внутри voice-assistant
[INFO] [get_robot_status] Запрос статуса робота
=== GetRobotStatusTool result ===
success: True
message: Статус робота получен (недоступны топики: /battery_state)
error:   None
data:    {'position': {'x': -0.0003585621016100049, 'y': 0.0014758831821382046, 'theta': -0.0025273570071640025},
          'battery_level': None,
          'systems': {},
          'unavailable_topics': ['/battery_state']}
```

Кросс-проверка состояния топиков:
```
$ ros2 topic info /odom           # Type: nav_msgs/msg/Odometry, 1 publisher, 8 subs
$ ros2 topic info /battery_state  # Type: sensor_msgs/msg/BatteryState, **0 publisher**, 1 sub
```

**Вердикт: ОПРОВЕРГНУТА.**

- `position` — реальные данные из `/odom` (x≈0, y≈0, theta≈0 — робот стоит).
- `battery_level` — `None` с честным `unavailable_topics=['/battery_state']`. Причина: на роботе **0 publisher** для `/battery_state` — это **инфраструктурный долг** (никто не публикует battery state), а не баг инструмента.
- `systems` — **пустой словарь**, не `{navigation: active, vision: active, tts: active}` как в старой версии.

В коде (`src/rob_box_mcp_tools/.../tools/system.py:660-668`) явно закомментировано: «ADR-0051 §6: 'systems' больше не захардкожен. Если оператор хочет знать «нода X поднялась?» — это ros2_node_status, а не get_robot_status.»

Гипотеза handoff §4.4 (написанная 5 сентября) уже **не соответствует коду** — инструмент починен ADR-0051, фикс залит в develop. Подтверждено: и в коде, и на живом роботе.

**Замечание (вне scope правок):** ноль publisher для `/battery_state` — отдельный инфраструктурный долг. Карточка-кандидат: «battery_state: добавить publisher в power-мониторинг».

---

## Итоговая таблица

| # | Гипотеза (handoff / карточка) | Вердикт | Артефакт-источник |
|---|---|---|---|
| H1 | Планировщик падает молча (fail-open в dialogue_node:1934 + tool_executor::_ensure_scheduler) | **ОПРОВЕРГНУТА** — W7b routed, scheduler skill активен | docker logs (см. выше) |
| H2 | `voice_input_mode` имеет значение по умолчанию, не совпадающее с желаемым | **НЕПРИМЕНИМА** — параметр удалён | ros2 param get/list |
| H3 | Из двух voice-БД пишется только одна | **УТОЧНЕНО** — пять БД, пишется `harness_voice.db` (dialogue_node), мёртвые: `voice_memory.db`, `voice_assistant.db`, `rob_box_voice.db` | ls -la + /proc/<pid>/fd |
| H4 | `getUserMedia` живёт часами в immersive-сессии | **НЕ ПРОВЕРЕНО** — out of scope, нужен физический Quest | (стат. анализ кода — косвенно) |
| H5 | `GetRobotStatusTool` врёт всегда (hardcoded "active") | **ОПРОВЕРГНУТА** — читает /odom и /battery_state, systems={}, ADR-0051 §6 применён | пробник + ros2 topic info |

---

## Что НЕ делалось в этой карточке

- Правок кода нет (только отчёт-документ).
- Чистка мёртвых `.db` — отдельная карточка.
- Publisher для `/battery_state` — отдельная карточка.
- Замер `getUserMedia` часами — карточка шага 5а архитектурного плана.

> ADR-0018: весь raw-вывод собран выше; не «работает», а показано конкретное состояние робота @ develop `3e9f72f4`.