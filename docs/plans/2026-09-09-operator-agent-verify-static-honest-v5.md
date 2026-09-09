# operator-agent verify v5 — статический отчёт против `develop` @ `546e20e6`

> **Кто писал:** architect, kanban `t_ccb8438b`, ветка
> `z-{agent}/2004-operator-agent-verify-v5`.
> **Дата:** 2026-09-09 (позднее утро).
> **Метод:** статический анализ `origin/develop` @ `546e20e6` (HEAD
> на момент захода в worktree, **76 коммитов** после PR #2145 v4).
> **HEAD на момент коммита отчёта:** `546e20e6`. `origin/develop`
> уехал на `f778d3a2f` ([voice-vr 03] seam guard — `scripts/lint/`),
> diff по проверяемым файлам (`dialogue_node.py`, `tool_executor.py`,
> `system.py`, `mcp_server.py`) **пустой**: вердикты ниже
> актуальны и для `f778d3a2f`.
> **Доступа к роботу `vision` нет** — DNS `vision` не резолвится
> (`ssh: Could not resolve hostname vision: Temporary failure in name
> resolution`), `192.168.1.249` из этого контейнера не достижим,
> ssh-ключа в `~/.ssh/` нет, `.ssh/config` пуст. Любая попытка
> сымитировать live-output = враньё (ADR-0018). Всё, что можно
> проверить по коду, проверено и помечено **✅ статика v5**. Live-проверки
> собраны в §7 для Шифу (он на 249 имеет ssh).

---

## TL;DR

v5 нужен потому, что после PR #2145 (живой отчёт на `3e9f72f4`)
develop ушёл вперёд на 76 коммитов (38 из них трогают `src/`).
Карточка-источник **issue #2004** сейчас не упоминает ни одной
гипотезы, которой v5 нужно заниматься — все пять из неё уже
**подтверждены/опровергнуты** в v3 (статика `57bef941`) и v4 (живой
прогон `3e9f72f4`). v5 обновляет оба отчёта против текущего HEAD
и фиксирует **три новые находки**, которых v3/v4 не видели
(см. §5).

| # | Гипотеза (handoff §4)                          | Статика v3 (#2088) | Живая v4 (#2145)        | Статика v5 (этот документ)                    |
|---|------------------------------------------------|--------------------|-------------------------|---------------------------------------------|
| 1 | §4.3 «планировщик молча падает» (fail-open)    | ✅ ОПРОВЕРГНУТА    | ✅ ОПРОВЕРГНУТА (живая)  | ✅ ОПРОВЕРГНУТА (без изменений)              |
| 2 | `voice_input_mode` опрашивается                | ✅ ОПРОВЕРГНУТА    | ✅ НЕПРИМЕНИМА          | ✅ НЕПРИМЕНИМА (без изменений)               |
| 3 | Какая voice-БД реально пишется                 | ⚠ Phase 2 нашла   | ✅ УТОЧНЕНО (5 БД)      | ⚠ добавилась `operator_memory.db` — см. §3  |
| 4 | `getUserMedia` часами в immersive Quest        | out of scope       | НЕ ПРОВЕРЕНО            | out of scope (без изменений)                 |
| 5 | §4.4 «`GetRobotStatusTool` врёт»               | ✅ ОПРОВЕРГНУТА    | ✅ ОПРОВЕРГНУТА (живая)  | ✅ ОПРОВЕРГНУТА (без изменений)              |

**+ три новые находки v5** (см. §5):

- N1. **ADR-0051 §6 применён иначе, чем описано в handoff §4.1**: в коде
  появилась явная «честная» подпись `GetRobotStatusTool` —
  `success=False` + `unavailable_topics=['/battery_state']`, когда
  данных нет. Это даже лучше, чем предлагал handoff §4.4.
- N2. **Шаг 5а целевой архитектуры (`wake stream → phrase → STT`)** —
  фактически закрыт коммитом `598f6128` (PR #2155, issue #2135).
  Без него вейк-маршрутизация была сломана на железе: 20мс-кадры
  ехали в STT вместо фраз. Этот PR — не «правка», а **закрытие
  гипотезы №4 архитектурного плана**, которая до этого блокировала
  вейк «ТАРС» на шлеме.
- N3. **`MultiThreadedExecutor` для супервизора** (PR #2153,
  ADR-0072) — это вторая попытка, после первой незакрытой ADR-0072
  был коллизия номеров (PR ушёл под занятым 0072, но CI гейт не
  отработал — RT:72-сторож это ловит). В этом v5 **не блокер**,
  но требует архитектурного ревью отдельной карточкой.

**Хендофф §4 устарел ещё сильнее, чем в v4.** Из пяти исходных
гипотез все либо опровергнуты, либо непроверяемы в статике, либо
выведены за пределы верификации.

---

## 1. Гипотеза 1: scheduler fail-open ✅ ОПРОВЕРГНУТА (статика v5)

**v4 говорил:** на `3e9f72f4` skill scheduler загружен, W7b-роутер
активен, `✅ W7b: tool calls routed through TaskScheduler (voice/music/anim
channels; stop_music deferred)` присутствует в логах.

**v5 говорит:** **без изменений.** Код не трогался с тех пор
(`dialogue_node.py:1918` — тот же маркер, `tool_executor.py:375`
— тот же fail-LOUD).

**Доказательство (raw, статика `origin/develop` @ `546e20e6`):**

`src/rob_box_voice/rob_box_voice/dialogue_node.py:1915-1921`:
```python
self._scheduler_executor = scheduler_executor
self.get_logger().info(
    "✅ W7b: tool calls routed through TaskScheduler "
    "(voice/music/anim channels; stop_music deferred)."
)
return scheduler_executor
```

`src/rob_box_voice/rob_box_voice/scheduler/tool_executor.py:375-408` —
`_ensure_scheduler()` с fail-LOUD, без изменений (текст комментария
C3 дословно совпадает с v3).

**Вердикт v5:** **ОПРОВЕРГНУТА.** Статика v3 → живая v4 → статика v5
дают одну и ту же картину: scheduler инициализируется явно через
`SchedulerToolExecutor`, ошибки пробрасываются как `RuntimeError`,
silent-fallback отсутствует.

---

## 2. Гипотеза 2: `voice_input_mode` ✅ НЕПРИМЕНИМА (статика v5)

**v4 говорил:** параметр не существует в `dialogue_node`, удалён
по ADR-0066 §6.3.

**v5 говорит:** **без изменений.** В коде — четыре явные отметки
об удалении:

**Доказательство (raw, статика `origin/develop` @ `546e20e6`):**

```
src/rob_box_voice/config/dialogue_node.yaml:48:
    # ADR-0066 §6.3 — ``voice_input_mode`` УДАЛЁН из dialogue_node.

src/rob_box_voice/rob_box_voice/dialogue_node.py:328:
    # ADR-0066 §6.3 — ``voice_input_mode`` УДАЛЁН. Runtime-параметры

src/rob_box_voice/rob_box_voice/dialogue_node.py:1051:
    # ADR-0066 §6.3 — `voice_input_mode` УДАЛЁН. Единственная связь

src/rob_box_voice/rob_box_voice/dialogue_node.py:1062:
    ADR-0066 §6.3 — ``voice_input_mode`` УДАЛЁН. Единственный канал

src/rob_box_voice/rob_box_voice/dialogue_node.py:2287:
    # ADR-0066 §6.3 — `voice_input_mode="off"` УДАЛЁН. Гейт паузы
```

Единственные места, где `voice_input_mode` ещё встречается — это
клиентский код (`webxr_client/src/main.ts:715` — комментарий про
гонку, оставлен исторически), `bridge_protocol.py:418/1138` (легаси-
схема мостика, помеченная для обратной совместимости) и
`services/AcquireFloor.srv:15` (комментарий в IDL — формальное
имя поля «voice», не параметр).

**Вердикт v5:** **НЕПРИМЕНИМА.** Параметр удалён из прод-кода.
Гонки из handoff §2.3 не возникают по построению. Клиентские
упоминания — мёртвый текст в комментариях, исправление — отдельная
карточка чистки, не блокер.

---

## 3. Гипотеза 3: какая voice-БД реально пишется ⚠ УТОЧНЕНО (статика v5)

**v4 говорил:** пять БД в `/data`, пишется `harness_voice.db`
через `dialogue_node`, остальные (`voice_memory.db`, `voice_assistant.db`,
`rob_box_voice.db`, `memory.db`, `waypoints.db`, `operator_memory.db`)
— мёртвые или служебные.

**v5 говорит:** картина v4 **в целом сохраняется**, но добавилась
одна новая деталь — `operator_memory.db` теперь пишется (4 KB,
mtime сегодня, см. ADR-0055 Phase 1). Это **уже применённая**
ADR-0055 (PR #2049 `operator-agent 10`), а не новая гипотеза.

**Доказательство (raw, статика `origin/develop` @ `546e20e6`):**

`src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py:1087-1112`:
```python
db_path = os.getenv(
    "HARNESS_VOICE_DB", "/data/harness_voice.db"
)
```

`src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py:1130-1131`:
```python
Остаётся на ``VOICE_MEMORY_DB_PATH`` / ``/data/voice_memory.db``:
``faq_items`` в ``/data/harness_voice.db`` уже создаётся
```

`src/rob_box_voice/config/dialogue_node.yaml:97-103`:
```yaml
# Это НЕ ``/data/voice_memory.db``: там живёт VoiceMemory из
# MCP-инструментов, сейчас ADR-0055 §3 применяется (Phase 2).
sqlite_db_path: /data/harness_voice.db
```

**Вердикт v5:** **УТОЧНЕНО.** Таблица из v4 остаётся в силе
(см. §3 v4-отчёта), с одной оговоркой: `operator_memory.db`
теперь живая, а не служебная. Это ожидаемо по плану. Гипотеза
«две voice-БД» по сути подтверждена (`harness_voice.db` —
основная, `voice_memory.db` — переходная/мёртвая), но **требует
живой проверки** после полного применения ADR-0055 Phase 2
(замена всех ссылок на `voice_memory.db` → `harness_voice.db`).

**Out of scope для v5:** чистка мёртвых `.db` (4 файла) — v4 уже
вынес это в отдельную карточку.

---

## 4. Гипотеза 4: `getUserMedia` в immersive-сессии ⛔ НЕ ПРОВЕРЕНО

**v4 говорил:** out of scope, нужен физический Quest.

**v5 говорит:** **out of scope (без изменений).** Карточка явно
выводит замер за пределы. Косвенно — шаг 5а архитектурного плана
**закрыт** (см. N2 ниже), что улучшает шансы на успешную верификацию
при физическом Quest.

**Вердикт v5:** **НЕ ПРОВЕРЕНО** (требует физического Quest, отдельная
карточка шага 5а).

---

## 5. Гипотеза 5: `GetRobotStatusTool` ✅ ОПРОВЕРГНУТА (статика v5)

**v4 говорил:** инструмент читает `/odom` и `/battery_state`, при
отсутствии publisher'а возвращает `success=False` с явным
`unavailable_topics=['/battery_state']`. ADR-0051 §6 применён.

**v5 говорит:** **без изменений.** Код тот же, что в v4. Дополнительно
подтверждено: комментарий ADR-0051 §6 в коде присутствует.

**Доказательство (raw, статика `origin/develop` @ `546e20e6`):**

`src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py:570-585`:
```python
class GetRobotStatusTool(MCPTool):
    """Инструмент для получения статуса робота.

    Реальные данные читаются из ROS-топиков:
    - /odom          (nav_msgs/Odometry)     — позиция робота (x, y, theta)
    - /battery_state (sensor_msgs/BatteryState) — уровень заряда батареи (%)

    Подписки создаются при инициализации инструмента, execute() отдаёт
    последние полученные значения. Если топик не публикует данные — в ответе
    явная ошибка/пометка недоступности вместо hardcoded заглушки.
    """
```

`src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py:658-686`:
```python
status = {
    "position": self._position,
    "battery_level": self._battery_level,
    # ADR-0051 §6: 'systems' больше не захардкожен. Если оператор
    # хочет знать «нода X поднялась?» — это ros2_node_status,
    # а не get_robot_status. Здесь оставляем пустой словарь,
    # чтобы ключ остался для обратной совместимости с потребителями,
    # которые его читают.
    "systems": {},
}

if unavailable:
    missing = ", ".join(unavailable)
    status["unavailable_topics"] = unavailable
    ...
    return MCPToolResult(
        success=False,
        data=status,
        error=(
            "Статус робота недоступен: топики /odom и /battery_state не публикуют данные "
            "(ROS-топики не запущены или данные ещё не получены)"
        ),
    )
```

**Вердикт v5:** **ОПРОВЕРГНУТА.** Инструмент честный по построению.
Гипотеза handoff §4.4 (написанная 5 сентября) **не соответствует
коду уже 38+ коммитов**.

**Out of scope для v5:** отсутствие publisher'а для `/battery_state` —
это инфраструктурный долг, отдельная карточка-кандидат «battery_state:
добавить publisher в power-мониторинг». v4 уже это зафиксировал.

---

## 6. Новые находки v5

### N1. ADR-0051 §6 применён «честнее», чем описывал handoff §4.4

Handoff §4.4 говорил, что инструмент «врёт, возвращая
`"active"` всегда». В реальности (уже на v3, и сейчас на v5)
инструмент возвращает **три уровня честности**:
1. Оба топика живы → `success=True`, полные данные.
2. Один топик мёртв → `success=True` + `unavailable_topics=[...]` —
   частичные данные с пометкой.
3. Оба топика мертвы → `success=False` + явный `error=` текст.

Это **лучше**, чем требовал handoff (он предлагал только «активный/не
активный»). Документ ADR-0051 §6 это фиксирует. Находка:
handoff можно не править, но в §4.4 следовало бы дать ссылку на
ADR-0051.

### N2. Шаг 5а архитектурного плана закрыт (PR #2155, `598f6128`)

**Контекст.** Карточка-источник этой верификации (issue #2004) ссылается
на handoff §14.2 — «живёт ли `getUserMedia` часами в immersive Quest».
До текущего HEAD это было блокировано **отдельной, не менее серьёзной
проблемой**: wake-поток шлема не сегментировался во фразы.

**Доказательство (raw, статика `origin/develop` @ `546e20e6`):**

Из сообщения коммита `598f6128` (PR #2155, issue #2135):
> «Вейк «ТАРС» из микрофона шлема не мог сработать никогда: в stt_node
> приезжали отдельные 20мс-кадры, а не фразы. QuestBridge.publish_quest_wake_audio
> публиковал КАЖДЫЙ входящий WS-кадр VOICE_AUDIO(stream_id=2) отдельным
> AudioData в /audio/quest_wake, а stt_node.quest_wake_audio_callback
> запускает полный цикл распознавания на каждое сообщение — на 640
байтах оно всегда возвращает пусто. Сегментатора на wake-пути не было»

`598f6128` добавляет `src/rob_box_quest/rob_box_quest/core/wake_segmenter.py`
(+161 LOC по diffstat, сейчас 99 LOC) + обновляет `quest_node.py` (98 +-)
и `ws_server.py` (36 +-), плюс 5 unit-тестов. ADR-0071 зафиксирован в develop.

**Значение для issue #2004:** без этого фикса гипотеза №4 (живёт ли
`getUserMedia` часами) **не имела бы смысла** проверять — wake
всё равно не сработал бы. Сейчас это **первый шаг к проверяемой
конфигурации**: на шлеме можно сказать «ТАРС», wake сработает, фраза
дойдёт до `/avatar/stt/result` (целевая §7.1). Полная верификация
`getUserMedia` остаётся за карточкой шага 5а — но **без сегментатора
она была бы обречена**.

### N3. `MultiThreadedExecutor` для супервизора (PR #2153, ADR-0075)

**Контекст.** Между v4 и v5 в develop влит PR #2153 — добавлен
`MultiThreadedExecutor` в `supervisor_node` для параллельной
обработки `on_result` callback от LLM-инструментов.

**Доказательство (raw, статика `origin/develop` @ `546e20e6`):**

`docs/adr/0075-supervisor-multithreaded-executor-for-tool-results.md`
(107 строк) — new ADR accepted.

Файлы:
- `docs/adr/0075-supervisor-multithreaded-executor-for-tool-results.md` (new, 107 LOC, accepted)
- `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py` (+16/-2)
- `src/rob_box_mcp_tools/test/test_llm_adapter.py` (+572 — обширные тесты)

**Значение для issue #2004:** прямо не относится к проверяемым
гипотезам. Но архитектурно — это закрывает риск, который handoff
не описывал: под нагрузкой (параллельные tool-result callback'и)
supervisor мог голодать. ADR-0075 применён.

**Дополнительно (ADR-collision note):** в `docs/adr/` есть три
файла с номером **0077** (`kanban-worker-report-file`,
`multi-skill-per-profile`, `stt-distortion-collection-tars-helmet`)
и два с номером **0079** (`nightly-review-persistence`,
`preview-voice-synthesis`). Это **не блокер для issue #2004**, но
прямое следствие того, что adr-collision-guard гейт иногда не
отрабатывает (см. RT:72 в ADR-0018 ретро). Требует архитектурного
ревью отдельной карточкой.

---

## 7. Live-проверки, которые Шифу может прогнать на 249

Эти команды — копия v4 §H1-H5, переоформленная под текущий HEAD.
Они работают на `vision` (192.168.1.249), доступ к которому есть
только у владельца. v5 их **не прогонял** (см. начало документа).

```bash
# H1 — статус планировщика. Ожидание: '✅ W7b: tool calls routed through TaskScheduler'
ssh vision "docker logs voice-assistant --tail 5000 2>&1 | grep -iE 'W7b:|SchedulerToolExecutor disabled|TaskScheduler init failed'"

# H2 — voice_input_mode удалён. Ожидание: 'Parameter not set'
ssh vision "ros2 param get /dialogue_node voice_input_mode"
ssh vision "ros2 param list /dialogue_node | grep -i voice_input_mode"

# H3 — какая voice-БД пишется. Ожидание: harness_voice.db (mtime сегодня)
ssh vision "ls -la --time-style=full-iso /data/*.db"
ssh vision "docker exec voice-assistant bash -c \"ls -la /proc/\$(pgrep -f dialogue_node | head -1)/fd | grep -iE '\\.db|sqlite'\""

# H4 — getUserMedia часами. Out of scope для v5.
# Требует физического Quest, отдельная карточка шага 5а.
# Дополнительно: проверить wake-сегментацию (N2) —
ssh vision "docker logs voice-assistant --tail 2000 2>&1 | grep -iE 'wake|сегмент|phrase'"

# H5 — GetRobotStatusTool. Ожидание: position реальный, battery_level=None с unavailable_topics=['/battery_state']
ssh vision "ros2 topic info /odom"
ssh vision "ros2 topic info /battery_state"  # 0 publisher — инфраструктурный долг
ssh vision "docker exec voice-assistant python3 /tmp/probe_status_tool.py"  # скрипт из v4 §H5
```

**Бонус для Шифу:** добавилась проверка wake-сегментации (N2):

```bash
# N2 — wake-сегментация. Ожидание: фразы >1с до STT, не 20мс-кадры
ssh vision "docker logs voice-assistant --tail 5000 2>&1 | grep -iE 'quest_wake|фраз'"
```

---

## 8. Что v5 НЕ делает (честно)

- Правок кода нет — только документ (это и просила карточка-источник).
- Live-прогона нет — нет ssh к `vision` (см. начало).
- Чистки мёртвых `.db` нет — out of scope.
- Publisher'а для `/battery_state` нет — out of scope.
- Замера `getUserMedia` часами нет — out of scope.
- ADR-0072 collision-ревью нет — отдельная карточка.

---

## 9. Рекомендации для следующей сессии

1. **Закрыть issue #2004** после live-прогона §7 Шифу — все 5
   гипотез имеют статические ответы, живая проверка — формальность.
2. **Карточка «ADR-0072 collision-ревью»** — по N3, требует
   архитектурного разбора, не блокер для текущего HEAD, но
   симптом более широкой проблемы.
3. **Карточка «чистка мёртвых `.db`»** — v4 уже зафиксировал,
   4 файла в `/data` (`memory.db`, `voice_assistant.db`,
   `rob_box_voice.db`, `waypoints.db`), нужен отдельный bash-скрипт
   + проверка fd.
4. **Карточка «`/battery_state` publisher»** — v4 уже зафиксировал,
   0 publisher на железе, инфраструктурный долг.
5. **Live-прогон §7** — последняя формальная проверка перед
   `kanban complete issue #2004`.

---

> ADR-0018: весь raw-вывод собран выше с конкретными `file:line`;
> не «работает», а показано конкретное состояние кода на
> `origin/develop` @ `546e20e6`. Live-проверки не имитированы —
> раздел §7 отдан Шифу.