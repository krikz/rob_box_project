# Verify: гипотезы из operator-agent handoff — статический разбор, 2026-09-07

> **Задача:** issue #2004, kanban `t_eeddc361`.
> **Автор:** architect (статический разбор, нет доступа к железу).
> **Источники:** `docs/plans/2026-09-05-operator-agent-architecture-handoff.md` §3/§4,
> `docs/architecture/target-operator-agent-and-dialogue.md` §8а.1,
> `docs/adr/0051-supervisor-operator-agent-arbiter-split.md`.
> **Базовый коммит:** `d5d378ab` (HEAD ветки `z-{agent}/2004-operator-agent-verify`,
> отстаёт от `origin/develop` на 13 коммитов — намеренно, чтобы не ломать base PR).

## 0. Что в этой карточке вообще просили

Карточка собирает **пять команд проверки на живом роботе** и помечает каждую
гипотезу вердиктом: подтверждена / опровергнута / не проверено. Команды требуют:

1. `ssh vision "docker logs voice-assistant 2>&1 | grep -E 'W7b:|SchedulerToolExecutor disabled|TaskScheduler init failed'"`
2. `ros2 param get /dialogue_node voice_input_mode`
3. `ls -la /data/*voice*.db` (mtime обеих)
4. Замер `getUserMedia` в immersive-сессии на Quest
5. Вызов `GetRobotStatusTool` + сравнение с `ros2 node list`

## 1. Проверка доступа — что доступно из worktree, что нет

```bash
$ timeout 8 ssh -o ConnectTimeout=4 -o BatchMode=yes vision "echo CONNECTED_OK; docker ps ..."
ssh: Could not resolve hostname vision: Temporary failure in name resolution

$ timeout 5 nslookup vision
;; Got SERVFAIL reply from 127.0.0.53
** server can't find vision: SERVFAIL

$ getent hosts vision 249 rob_box_quest
0.0.0.249       249
```

`vision` не резолвится из worktree (DNS отдаёт SERVFAIL, конфиг `~/.ssh/config`
отсутствует — `cat ~/.ssh/config: No such file or directory`).
`249` резолвится как `0.0.0.249`, но `ssh 249` валится в `Connection timed out`
(порт 22 за фильтром).

**Вердикт доступа:** из этой сессии **ни одна** из пяти проверочных команд не
выполнима. Карточка честно помечена как «информационная, не блокирует» —
это нормально, она нужна для следующего воркера/владельца с ssh-ключом.
Здесь я делаю только **статическую** часть (что в коде) — это полезно, но
не закрывает DoD.

## 2. Статический разбор: что гипотезы подтверждаются в коде (worktree @ `d5d378ab`)

### §4.3 «Планировщик падает молча» — fail-open в коде подтверждён

Два независимых fail-open (по обоим лог-сообщениям из команды верификации):

**a) `dialogue_node.py:1533-1553`** — обёртывание tool-provider в SchedulerToolExecutor:
```python
# dialogue_node.py:1528-1552
# W7b (issue #968): route channel tools (speak_text / music /
# anim) through the TaskScheduler. stop_music is deferred until
# the VOICE channel drains, so it can no longer outrun the TTS
# chunk (e2e v36). Fail-open: if the scheduler cannot start,
# the adapter is returned unwrapped and tools execute directly.
try:
    from rob_box_voice.scheduler.tool_executor import (
        SchedulerToolExecutor,
    )

    scheduler_executor = SchedulerToolExecutor(
        provider_adapter,
        on_event=self._on_task_event,
    )
    self._scheduler_executor = scheduler_executor
    self.get_logger().info(
        "✅ W7b: tool calls routed through TaskScheduler "
        "(voice/music/anim channels; stop_music deferred)."
    )
    return scheduler_executor
except Exception as exc:  # noqa: BLE001 — fail-open, never break voice
    self.get_logger().warning(
        f"⚠️ W7b SchedulerToolExecutor disabled ({exc!r}); "
        "tools execute directly (pre-W7b path)."
    )
    return provider_adapter
```

> **Замечание.** В хендоффе указана строка `dialogue_node.py:1934`. В текущем
> коде (HEAD `d5d378ab`) — это **строка 1549-1552**. Строка сместилась из-за
> последующих коммитов — на суть ловушки не влияет.

**b) `scheduler/tool_executor.py:363-386`** — внутренний fail-open конструктора:
```python
# rob_box_voice/scheduler/tool_executor.py
def _ensure_scheduler(self) -> Optional[TaskScheduler]:
    """Lazy-create the scheduler on first use (running loop available).
    ...
    Idempotent and fail-soft: if creation fails, ``None`` is returned and
    the caller executes directly.
    """
    if self._scheduler is not None or self._scheduler_attempted:
        return self._scheduler
    self._scheduler_attempted = True
    try:
        scheduler = TaskScheduler(on_event=self._on_event)
        scheduler.start()
        self._scheduler = scheduler
    except Exception as exc:  # noqa: BLE001 — fail-open
        _LOG.warning(
            "TaskScheduler init failed (%s); tool calls bypass the "
            "scheduler",
            exc,
        )
        self._scheduler = None
    return self._scheduler
```

`caller.execute()` (tool_executor.py:208-210 и 275-277) при `scheduler is None`
вызывает `self._underlying.execute(call)` — прямой путь без планировщика.

**Гипотеза §4.3 «падает молча» — статически подтверждена.** Внешне это
выглядит как «W7b landed» (есть success-лог), но при любом исключении
SchedulerToolExecutor → TaskScheduler → инициализация падает, и в логах
остаётся только `warning`. На живом роботе надо убедиться, что warning'ов
нет — для этого команда 1 из карточки.

### §4.4 «`GetRobotStatusTool` врёт всегда» — захардкоженное «active» подтверждено

`src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py:645-649`:
```python
status = {
    "position": self._position,
    "battery_level": self._battery_level,
    "systems": {"navigation": "active", "vision": "active", "tts": "active"},
}
```

Реальные данные подписки `/odom` (nav_msgs/Odometry) и `/battery_state`
(sensor_msgs/BatteryState) есть и обрабатываются (см. `_on_odom`/`_on_battery`
выше). Но раздел `systems` — статика, **никаких проверок** поднятия нод
`/navigation`, `/vision`, `/tts` не делается. Это видно и по `name == "get_robot_status"`
и `description` — инструмент выдаёт позицию и батарею, но «systems» — муляж.

**Гипотеза §4.4 — статически подтверждена.** Команда 5 из карточки это
продублирует на железе (сравнить `tools.execute()` с `ros2 node list`).

### Гипотеза 2 — реальное значение `voice_input_mode`

Дефолт зафиксирован в `dialogue_node.py:916-919`:
```python
self.declare_parameter("voice_input_mode", "respeaker")
self._voice_input_mode: str = "respeaker"
```

Изменяется через SetParameters (см. `dialogue_node.py:971-973` — кто-то
вроде супервизора зовёт `/dialogue_node/set_parameters`). Дефолтное
значение `respeaker` означает, что без активного супервизора/quest-клиента
Quest-STT игнорируется — это **нормальное** поведение для «робот без шлема»,
а не баг.

**Гипотеза 2 — статически:** дефолт известен, проверить можно только
на железе (команда 2). Без ssh не проверить.

### Гипотеза 3 — «две voice-БД»

Из `docker/vision/config/voice_assistant/dialogue_node.yaml:96-107`:
```yaml
# ``~/.rob_box/voice.db`` — это /root внутри контейнера, том туда не
# смонтирован, поэтому весь контекст разговора умирал вместе с
# контейнером (проверено на vision 29.08: файл создан в 15:29 вместе с
# контейнером, 69 ходов, ни одного старше).
#
# Это НЕ ``/data/voice_memory.db``: там живёт VoiceMemory из
# mcp_server, и схемы конфликтуют — у harness'а ``waypoints.name``
# PRIMARY KEY, у VoiceMemory ``waypoints`` с ``map_id NOT NULL`` и FK
# на maps; ``faq_items`` расходятся так же (created_at vs indexed_at).
# ``CREATE TABLE IF NOT EXISTS`` промолчал бы, а вставки бы падали.
# Два стора остаются двумя сторами — но оба переживают рестарт.
sqlite_db_path: /data/harness_voice.db
speaker_id_enabled: true
speaker_db_path: /data/speakers.db
```

Из `docker/vision/docker-compose.yaml:220`:
```yaml
- VOICE_MEMORY_DB_PATH=/data/voice_memory.db
```

Итого в `/data` минимум три файла с `voice`/речью:
- `/data/voice_memory.db` — VoiceMemory из mcp_server;
- `/data/harness_voice.db` — новый harness-стор;
- `/data/speakers.db` — speaker_id (не voice-БД в строгом смысле, но тот же том).

**Гипотеза 3 — статически:** понятно какие файлы должны быть. Проверить
mtime обеих и кто реально пишется — без ssh не проверить. ADR-0037
«Memory layers» прямо говорит, что `voice_memory.py` — старая БД, а
`MemoryStore` — новая; «migration `voice_memory.db` → `SQLiteVoiceMemory`,
но это ADR-0038+» — то есть статус миграции неизвестен.

### Гипотеза 4 — `getUserMedia` в immersive-сессии на Quest

Фронт-логика в `src/rob_box_quest/webxr_client/` (TypeScript), есть
тест `voice_capture.test.ts:1-3` («захват микрофона → int16 PCM 16 kHz mono
(рация). Проверяем: resample 48k→16k, нарезка на ~20мс чанки, освобождение
getUserMedia-трека на stop()»). Тест проверяет `stop()`-семантику, но **не**
проверяет многочасовую устойчивость. Хендофф §14.2 это явно вынес в
«открытое».

**Гипотеза 4 — статически:** без замера на устройстве не ответить.

## 3. Сводная таблица вердиктов (карточка #2004, DoD)

| # | гипотеза | статика (worktree) | на железе (ssh/Quest) | вердикт |
|---|---|---|---|---|
| 1 | Планировщик падает молча (§4.3) | подтверждено — fail-open в `dialogue_node.py:1548-1552` и `tool_executor.py:379-385` | **не проверено** (нет ssh vision) | подтверждена статически; на железе остаётся **не проверено** — нужен grep из карточки |
| 2 | Реальное `voice_input_mode` | default `respeaker` (`dialogue_node.py:918`) | **не проверено** | **не проверено** (нужен `ros2 param get`) |
| 3 | Какая voice-БД реально пишется | две БД известны: `voice_memory.db` и `harness_voice.db` (yaml) | **не проверено** | **не проверено** (нужен `ls -la /data/*voice*.db`) |
| 4 | `getUserMedia` часами на Quest | тест на `stop()` есть, многочасового нет | **не проверено** | **не проверено** (только замер на устройстве) |
| 5 | `GetRobotStatusTool` врёт | подтверждено — `systems` захардкожен `active` (`system.py:648`) | **не проверено** | подтверждена статически; на железе остаётся **не проверено** — нужен вызов + сравнение с `ros2 node list` |

## 4. Что осталось сделать следующему воркеру с ssh-доступом

1. `ssh vision "docker logs voice-assistant 2>&1 | grep -E 'W7b:|SchedulerToolExecutor disabled|TaskScheduler init failed'"` — если только success-лог `✅ W7b: tool calls routed...`, гипотеза 1 опровергнута; если есть `⚠️ W7b SchedulerToolExecutor disabled` или `TaskScheduler init failed`, подтверждена.
2. `ros2 param get /dialogue_node voice_input_mode` — дефолт `respeaker`, любое другое значение = активный супервизор/quest.
3. `ls -la /data/*voice*.db` (на vision-хосте внутри контейнера или на bind-mount) — `voice_memory.db` и `harness_voice.db`, сравнить mtime с реальным трафиком (см. `docker logs voice-assistant | grep -E "VoiceMemory|harness_voice"`).
4. Запустить immersive-сессию на Quest на ≥1 час, периодически опрашивать `getUserMedia`/`MediaStreamTrack.readyState` через CDP/console или watch `battery_level` при всегда-включённом микрофоне.
5. Дёрнуть `GetRobotStatusTool.execute()` через любой фронт (Telegram `/status`, `/avatar` или прямой ROS) и сравнить раздел `systems` с `ros2 node list | grep -E 'navigation|vision|tts'`.

## 5. Связь с уже идущими шагами

- Шаги operator-agent (PR #2008 #2009 #2010 #2011 #2012 уже в `origin/develop`)
  реализуют ADR-0051, но **не проверяют** на железе ни одной гипотезы из этой
  карточки. После их merge имеет смысл сделать «live-верификацию» одним прогоном.
- ADR-0037 «Memory layers» уже фиксирует расхождение двух БД; если миграция
  по ADR-0038+ ещё не сделана, на железе будут обе, и mtime покажет, какая
  реально пишется.
- ADR-0028 §4.5 (set_parameters на dialogue_node) формально отменён в целевой
  архитектуре (см. ADR-0051), но **код ещё держит** этот путь
  (`dialogue_node.py:971-973`). Это отдельная задача — не блокер этой карточки.

## 6. Честный итог

Карточка просит прогнать 5 команд на живом роботе. Из worktree архитектора
**ни одна не выполнима** — DNS `vision` не резолвится, ssh-конфиг пустой,
Quest физически недоступен. Статически подтверждены только те гипотезы,
которые читаются из исходников (§4.3, §4.4). Остальные три гипотезы (2, 3, 4)
остаются в статусе «не проверено» — их закрытие требует владельца или
e2e-воркера с ssh-доступом.

Сам факт статического подтверждения §4.3 и §4.4 — это **не «готово»**, а
**«ещё одна причина для live-чека»**: код честно fail-open'ит, статика
подтверждает, что ловушка существует; единственный способ убедиться, что
на проде она не сработала — это запустить проверочную команду из карточки
на работающем роботе.
