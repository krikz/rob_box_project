# ADR-0086: `EventBus` шина отмены и `ReflexLayer` — удалить, не подключать

| Поле | Значение |
|---|---|
| Статус | **Proposed** (ожидает решение Шифу по §7) |
| Дата | 2026-09-09 |
| Автор | architect (Hermes Agent), kanban `t_b4ae3581` (issue #2245) |
| Контекст | ADR-0080 §2.8 («прополка планировщика») требует решить судьбу написанной, но не подключённой пары `EventBus` ↔ `ReflexLayer`. Карточка #2245 пришла как дискуссионная (метки `needs-discussion`, `priority:low`, `source:gsd`); тело явно говорит «Тип: решение владельца. Не реализация вслепую». Этот ADR — закрытие открытого вопроса №3 из ADR-0080 §7 с явной рекомендацией. |
| Затрагивает | `src/rob_box_voice/rob_box_voice/scheduler/reflex.py` (861 LOC — удалить), `src/rob_box_voice/rob_box_voice/scheduler/__init__.py` (re-export `ReflexLayer`, `ReflexEvent`, `ReflexKind`, `ReflexPriority`, `ReflexDecision`, `ReflexMetrics`, `command_to_view`, `DEFAULT_DEBOUNCE_MS`, `DEFAULT_HISTORY_SIZE` — удалить), `src/rob_box_voice/test/test_reflex_layer.py` (850 LOC — удалить), `src/rob_box_voice/test/unit/node/command_reflex_bridge/` (4 файла, 614 LOC — удалить), `src/rob_box_voice/rob_box_voice/command_node.py` (параметр `enable_reflex_layer`, фоновый поток `_bus_thread_main`, `_start_reflex_bridge`, `_init_bus`, `build_parsed_envelope`, `_publish_parsed_async` — ~140 LOC), `docker/vision/config/voice_assistant/command_node.yaml` (строка `enable_reflex_layer: true`), `src/rob_box_voice/rob_box_voice/scheduler/README.md` (упоминание «reflex bridge»), `src/rob_box_voice/rob_box_voice/scheduler/task_scheduler.py` (комментарий в шапке про «reflex bridge»-подписчиков), `docs/architecture/target-operator-agent-and-dialogue.md` §13 строка «7б. `ReflexLayer` ❌ **написан, не подключён**», ADR-0054 (архивировать — задача перекрыта), `CONTEXT.md` (статус спекулятивной генерации / отмены / рефлекса) |
| Родители | ADR-0018 (честный FAIL — инвариант 12 запрещает «написанное, но не подключённое»), ADR-0080 §2.8 (открытый вопрос №3 — этот ADR закрывает), ADR-0054 (предшественник — реализация моста, не решила корневую проблему), ADR-0082 (образец формата «дрейф-документа ↔ кода»), ADR-AF-0013 (incremental delivery — большие PR не катим), `docs/design/SCHEDULER_DESIGN.md` §8.10 (проектная база reflex-сценариев — устарела) |
| Связанные | issue #2245 (эта карточка), #1997 (рефлекс-мост, перекрыт), #1993 (ReflexLayer, код), `docs/plans/2026-09-08-archive-triage.md` (подтверждает перекрытие #1997 PR #2048), PR #2048 (ReflexLayer bridge — смержен, но без продового эффекта) |

> **Перенумерован 2026-09-09 из ADR-0083 в ADR-0086.** Номер 0083 занял
> PR #2247 (`build_agent(spec)`, создан на 24 секунды раньше). 0084 занят
> PR #2252 (`TurnGuards`), 0085 занят влитым переносом
> `0080-tars2-renders-metrics-...md` (PR #2257). Следующий свободный —
> 0086. См. `docs/plans/2026-09-09-voice-vr-architecture-handoff.md` §4.1.

> **TL;DR.** `EventBus` остаётся **внутри `TaskScheduler`** (он там родился и там нужен — для `scheduler.cancel` envelope'ов из `_publish_cancel_event`, C2 #1995). `ReflexLayer` (~861 LOC) + весь мост в `command_node` (~140 LOC) + связанные тесты (~1464 LOC) — **удаляются**. «Стой!» продолжает работать через уже существующий `command_node.handle_stop()` → `action_msgs/srv/CancelGoal` (`command_node.py:467-499`); он не покрывает preemption TTS-чанков, но это **и есть** желаемое поведение (см. ADR-0054 / `scheduler/README.md` §v36 — FIFO voice channel удерживает порядок, обрыв чанка на полуслове ломает state machine). Реализация reflex-сценариев из `SCHEDULER_DESIGN §8.10.6` откладывается до решения владельца по сценарию 7 §7 ниже.

---

## 0. Что внутри и что — нет

**Внутри.** Решение «подключать нельзя, удалить». Список под удаление с конкретными путями и LOC. Список того, что **сохраняется** (`EventBus` внутри `TaskScheduler`, его envelope'ы — это полезный внутренний канал наблюдаемости cancel-preemption). Куда переезжает «стой!» UX (он уже там). Куда переезжает priority-insert (§8а.2 целевой — это про `priority` поле на `/voice/tts/request`, не про reflex). Архивирование ADR-0054.

**Не внутри.** Реализация reflex-сценариев «Спой песню → стой!», «Едь на кухню → направо» (SCHEDULER_DESIGN §8.10.6). Если Шифу в §7 выберет вариант «отложено с явной пометкой» — отдельная карточка-исследование через 1-2 месяца, когда будет хотя бы один живой замер «на роботе» для опоры. Никакого re-implementation в этом PR.

---

## 1. Контекст и бизнес-проблема

### 1.1 Состояние «как есть» (raw-факты, проверено чтением `develop` @ `7b72adfe`)

Каждый факт ниже — отдельной строкой grep/read, без выводов.

**(а) `TaskScheduler` уже владеет шиной** —
`src/rob_box_voice/rob_box_voice/scheduler/task_scheduler.py:639`:
```python
self.event_bus: EventBus = EventBus()
```
Создаёт свой экземпляр на каждом `__init__`. Внутренние подписчики
`_cancel_subscriptions: Dict[str, EventSubscription]` (`:640`) и
`_cancel_handler_started: bool` (`:643`) **объявлены, но никем не
населяются** — `git grep "_cancel_subscriptions\[\\|_cancel_handler_started = True"`
возвращает только само объявление и один модульный тест.

**(б) `cancel()` публикует envelope `scheduler.cancel`** —
`task_scheduler.py:995-1030`, `_publish_cancel_event()`:
```python
future = asyncio.run_coroutine_threadsafe(
    self.event_bus.publish(envelope), loop
)
```
Подписчиков этого топика **в продовом коде нет**. Все четыре потребителя
`t.scheduler.cancel` находятся в `test/test_task_scheduler*.py` и
`test/test_task_scheduler_eventbus.py:124,152,160` — это проверки самой
публикации, а не её использования.

**(в) Голосовой планировщик живёт в `SchedulerToolExecutor`** —
`src/rob_box_voice/rob_box_voice/scheduler/tool_executor.py:409`:
```python
scheduler = TaskScheduler(on_event=self._on_event)
scheduler.start()
self._scheduler = scheduler
```
Лениво создаётся при первом `execute()` в `dialogue_node`. Через него идут
**все** живые задачи из диалога: `speak_text` (VOICE), `stop_music` (MUSIC),
`play_animation` (ANIM). У этого экземпляра шина своя — `event_bus` тот
встроенный, что в `(а)`.

**(г) `ReflexLayer` сидит на **другом** планировщике** —
`src/rob_box_voice/rob_box_voice/command_node.py:204-223` (`_init_bus`):
```python
bus = EventBus()              # :214  — третий экземпляр шины в процессе
scheduler = TaskScheduler()   # :215  — второй TaskScheduler в процессе
scheduler.start()
layer = ReflexLayer(scheduler, bus)
layer.attach(bus)
```
Это **не** планировщик из `(в)`. В `(г)`-планировщик никто не кладёт задач
(он изолирован от `dialogue_node`); `_publish_cancel_event` этого
экземпляра тоже уходит в пустоту.

**(д) `enable_reflex_layer=true` в продовом конфиге** —
`docker/vision/config/voice_assistant/command_node.yaml:17`:
```yaml
# Код-дефолт false (ADR-0054); здесь — осознанное включение в прод-конфиге.
enable_reflex_layer: true
```
То есть на текущем docker-деплое мост **поднят**, фоновый поток
`command-node-reflex-bus` живёт, шина из `(г)` крутится, слой
подписан — и при этом слой отменяет задачи, которых в этом
планировщике нет (см. `(г)`).

**(е) Документ-источник целевой архитектуры подтверждает статус** —
`docs/architecture/target-operator-agent-and-dialogue.md:1133`:
```
| 7б. `ReflexLayer` | ❌ **написан, не подключён** | `enable_reflex_layer` default `False`
и нигде не выставляется; включённый работает по пустой очереди |
```
Строка §13 «устаревшая» по сравнению с `(д)` (в проде `true`), но
вывод **тот же**: «работает по пустой очереди».

**(ж) ADR-0054 был архивирован в issue #1997** —
`docs/plans/2026-09-08-archive-triage.md:29`:
```
| `archive-zagent1997operatoragent07breflexlayercommandn` (#1997, ReflexLayer) | tag | exit 1 | … |
```
Подтверждено, что код моста перекрыт смерженным PR #2048 — и в обеих
версиях `command_node.py` имеет 651 строку. Этот ADR перекрывает и
ADR-0054, и код моста.

**(з) «Стой!» сегодня работает через Nav2-cancel** —
`command_node.py:467-499`, `handle_stop()`:
```python
if self.enable_navigation:
    ...
    self.cancel_client.call_async(request)   # CancelGoal.Request()
```
Это прямой ROS-вызов `/navigate_to_pose/_action/cancel_goal` с пустым
`GoalInfo` (= «отменить все goals»). Что **не покрывается**: preemption
TTS-чанка, прерывание музыки посреди трека, отмена `play_animation`.
**Но это и не нужно** — см. §3.

### 1.2 Что ADR-0080 §2.8 просил решить

Из ADR-0080 §2.8 (verbatim):

> Шина отмены `EventBus` либо отдаётся тому `TaskScheduler`, который
> реально исполняет задачи (`SchedulerToolExecutor`), либо удаляется
> вместе с `ReflexLayer`. Держать написанный, но не подключённый
> механизм запрещено — он читается как существующий (ADR-0018).

Из §7 открытый вопрос №3 (verbatim):

> Судьба `ReflexLayer`. 861 строка кода и 850 строк тестов. Подключение к
> живой шине — работа; удаление — потеря задуманного «стоп!». Решение
> владельца, не архитектора.

Этот ADR закрывает вопрос №3. Из двух альтернатив в §2.8 («отдать
SchedulerToolExecutor» vs «удалить вместе с ReflexLayer») я
рекомендую вторую, с обоснованием ниже.

---

## 2. Решение

### 2.1 Что удаляется

| Что | Путь | LOC (raw) | Основание |
|---|---|---|---|
| Модуль рефлекса | `src/rob_box_voice/rob_box_voice/scheduler/reflex.py` | 861 | §1.1 (г) — отменяет не тот планировщик |
| Тесты модуля | `src/rob_box_voice/test/test_reflex_layer.py` | 850 | тесты без кода = тест-фантом |
| Тесты моста | `src/rob_box_voice/test/unit/node/command_reflex_bridge/` (4 файла) | 614 | см. §1.1 (д) — мост живёт, но подключён не туда |
| Параметр + мост в command_node | `command_node.py:14-17, 49-59, 106-126, 143-291, 309-314` | ~140 | параметр не нужен, поток не нужен, `build_parsed_envelope`/`_publish_parsed_async` — мёртвый путь |
| Re-export из `scheduler` | `scheduler/__init__.py:98-108, 178-187, 201-210` | 30 | символы удаляются вместе с модулем |
| Конфиг-флаг | `docker/vision/config/voice_assistant/command_node.yaml:14-17` | 4 | параметра больше нет |
| **Всего** | | **~2500** | (тело карточки #2245 занизило до 1711 — фактический объём больше) |

Удаление **одним PR**, потому что все куски — последствия одного
решения и врозь не компилируются (re-export ссылается на
удаляемый модуль; мост ссылается на удаляемый `ReflexLayer`).
Размер PR большой, но он механический (sed-find + удаление); никакой
новой логики. ADR-AF-0013 (incremental delivery) это допускает для
«чистого удаления», как здесь.

### 2.2 Что сохраняется

| Что | Путь | Почему остаётся |
|---|---|---|
| `EventBus` (класс + envelope + backpressure) | `scheduler/event_bus.py` (189 LOC) | §3.1 ниже — канал наблюдаемости cancel-preemption |
| `TaskScheduler.event_bus` | `task_scheduler.py:639` | единственный живой пользователь шины — сам планировщик |
| `_publish_cancel_event()` | `task_scheduler.py:995-1030` | нужен для тестов C2 #1995 и для будущих подписчиков из телеметрии |
| `SchedulerToolExecutor` | `scheduler/tool_executor.py` | живой код, через него идут все voice/music/anim задачи |
| `command_node.handle_stop()` | `command_node.py:467-499` | «стой!» UX — не выбрасываем |

### 2.3 Что правится в документации

| Файл | Что |
|---|---|
| `docs/adr/0054-operator-agent-step-7b-eventbus-bridge.md` | В шапке добавить: «**Статус: Archived by ADR-0086, 2026-09-09.** Мост не решил корневую проблему — `ReflexLayer` отменял не на том `TaskScheduler`. См. ADR-0086 §3.4.» |
| `docs/adr/0080-voice-and-headset-control-eight-seams.md` §2.8 | Заменить «Шина отмены либо отдаётся … либо удаляется вместе с `ReflexLayer`. Держать написанный, но не подключённый механизм запрещено» на «Шина отмены остаётся внутри `TaskScheduler`. `ReflexLayer` удалён ADR-0086.» |
| `docs/architecture/target-operator-agent-and-dialogue.md` §13 | Строка `7б. ReflexLayer ❌ написан, не подключён` → `7б. ReflexLayer ✅ удалён ADR-0086` |
| `docs/architecture/target-operator-agent-and-dialogue.md` §8а.1 таблица | Строка `| ReflexLayer | 656 | **не подключён** | … |` → строка удаляется |
| `src/rob_box_voice/rob_box_voice/scheduler/README.md` | Убрать упоминание «(reflex bridge, observability)» из строки 91 про cancel-envelope. Добавить в таблицу «Что НЕ входит в MVP» строку `\|\| ReflexLayer / «стой!» через планировщик \| удалено (ADR-0086) \| — \|`. |
| `src/rob_box_voice/rob_box_voice/scheduler/task_scheduler.py:1-44` (docstring) | Убрать фразу «Every successful cancel publishes a `scheduler.cancel` envelope … subscribers (reflex bridge, observability) see the preemption». Заменить на «Every successful cancel publishes a `scheduler.cancel` envelope so observability subscribers can see the preemption». |
| `src/rob_box_voice/rob_box_voice/scheduler/task_scheduler.py:947-955` | Аналогично — убрать «reflex layer» из перечисления подписчиков. |
| `CONTEXT.md` | Статус «отмена / рефлекс»: отмена подключена через `scheduler.cancel` envelope, рефлекс удалён ADR-0086. |

---

## 3. Обоснование: почему удалить, а не подключить

### 3.1 Корневая причина неработоспособности моста

Из §1.1 (в) и (г): `ReflexLayer` сидит на **отдельном** `TaskScheduler`,
созданном в `command_node._init_bus()`. Голосовые задачи идут через
**другой** `TaskScheduler` в `SchedulerToolExecutor._ensure_scheduler()`
(`tool_executor.py:409`). Это два разных Python-объекта в одном
процессе, у каждого своя `EventBus`. `ReflexLayer.cancel(task_id)`
отменяет задачу в «своём» планировщике, где её нет.

Вариант «отдать шину `SchedulerToolExecutor`» (первая альтернатива
ADR-0080 §2.8) требует:

1. Поднять `EventBus` до уровня shared assembly (т.е. до
   `dialogue_node._build_tool_provider()`), чтобы и
   `SchedulerToolExecutor`, и `command_node` могли на него
   подписаться.
2. Передать `EventBus` из `dialogue_node` в `command_node` через
   ROS-параметр или shared state — это новый контракт между
   двумя ROS-нодами, ровно того класса, против которого предостерегает
   `SCHEDULER_DESIGN.md §1 «Чего НЕ хотим»` (scheduler не на
   ROS-топике; не плодить новые каналы между нодами).
3. Передать `TaskScheduler` handle тем же способом, потому что без
   того же экземпляра планировщика `ReflexLayer.cancel()` снова
   стреляет в пустоту.
4. Перенести `ReflexLayer` из `command_node` в `dialogue_node` (он
   должен видеть LLM-submit-путь) либо дублировать логику
   «stopped-command → cancel-all» в `command_node.handle_stop()`.
   Любой вариант тянет за собой рефакторинг `dialogue_node`
   (588-строчный конструктор, ADR-0080 §1.5 уже говорит, что
   разбиение по методам CC не снижает — выносить оркестрацию, а
   не предикаты).
5. Проверить на живом роботе, что TTS-чанк, прерванный на
   полуслове через `asyncio.CancelledError`, корректно
   публикует `tts/finished` и не врёт state machine
   `dialogue_node`. Это не очевидно — см. §3.2.

Суммарно: ~400-600 LOC нового кода + тесты + e2e «стой!» во время
песни + переделка шапки `dialogue_node`. Это карточка на 2-4 недели
для одного воркера, при этом единственный прирост UX — «Стой!»
может прервать TTS-чанк на полуслове. Этот прирост — **анти-фича**
(см. §3.2).

### 3.2 «Стой!» через TTS-preempt — анти-фича

`src/rob_box_voice/rob_box_voice/scheduler/README.md:7-12` (verbatim
из секции «Зачем»):

> Перед фазой 1 race `stop_music()` против TTS-чанка приводил к тому,
> что музыка обрывалась на середине рэпа (регресс v36). MVP убирает
> race, сериализуя канал **voice** так, что два `speak_text` (и
> любой голосовой side-effect, включая `stop_music` после TTS) не
> могут пересечься на железе […]

То есть **голосовой канал проектировался как FIFO** именно чтобы
текущий чанк доигрывал. Reflex-сценарий из `SCHEDULER_DESIGN
§8.10.6` «Спой песню → стой!» предполагает, что `stop_music` после
`Спой песню → стой!» пройдёт через `scheduler.cancel` и
мгновенно остановит TTS. Но если TTS-чанк уже в `await
asyncio.sleep(...)` воспроизведения, `asyncio.CancelledError` его
прервёт **на полуслове**:

- `tts_node` не пошлёт `/voice/tts/finished` (state machine
  `dialogue_node` ждёт этого топика, чтобы перейти из SPEAKING →
  LISTENING — иначе висит до таймаута).
- LLM-цикл увидит оборванный чанк, может решить, что реплика
  не доставлена, и попробовать повторить → второй чанк поверх
  обрыва.
- Music-stop прилетит **до** того, как TTS закончит — а это и есть
  v36-регресс, который MVP убирал. То есть фикс v36 и фича
  reflex находятся в прямом противоречии по одному и тому же
  каналу.

Если в будущем это захочется — нужна не «cancel-all», а
**приоритетная врезка** (то, что §8а.3 целевой называет «приоритет
в `tts_node`», и что уже сделано: `_normalize_tts_priority`,
поле `priority` в `/voice/tts/request`). Это даёт «текущий чанк
доигрывает, следующий — мой», а не «обрыв на полуслове». Это
другая фича с другим контрактом и ADR — если будет нужно.

### 3.3 ADR-0018 инвариант 12 — прямой запрет

`docs/adr/0018-agent-honesty-culture.md` инвариант 12 (ADR-0080 §3):

> Написанное, но не подключённое — удаляется. Механизм, у которого
> нет продового вызывающего, либо подключается, либо удаляется
> вместе с тестами. Комментарий «будет подключено в Phase N» не
> считается подключением (ADR-0018).

`ReflexLayer` + мост — ровно этот случай. `enable_reflex_layer=true`
в проде (`command_node.yaml:17`) делает его **видимо** подключённым,
но фактический эффект — нулевой (отменяет задачи, которых нет в
этом `TaskScheduler`). Это хуже, чем «не подключён»: на проде
стоит фоновый поток, шина и подписка, метрики пишутся — а
поведения, которое эти метрики описывают, в системе нет. Если
кто-то читает `/reflex/events` через `bus.subscribe` в
наблюдательности, он видит события, которые **никогда не
превращаются в отмену**. Это worst-of-both-worlds.

### 3.4 Почему не вариант «оставить с пометкой»

Оставить модуль с `"""DEPRECATED: see ADR-0086."""` в шапке и
выключить флаг — рассматривался. Отвергнут, потому что:

- В `__init__.py` re-export остаётся публичным API пакета;
  любой импорт `from rob_box_voice.scheduler import ReflexLayer`
  пройдёт без ошибки, и следующий воркер может решить «ну он же
  экспортируется, значит живой». Удаление делает это **физически
  невозможным** — `ImportError` сразу.
- Тесты `test_reflex_layer.py` + `command_reflex_bridge/` = 1464 LOC
  тестового кода, который тестирует неиспользуемый функционал.
  Это прямой вклад в test_LOC > prod_LOC (ADR-0080 §1.8).
- Фоновый поток в `command_node` останется жить; для отказа от
  него нужно удалить весь блок `_start_reflex_bridge` /
  `_bus_thread_main` / `_init_bus`. То есть «оставить с пометкой»
  превращается в «оставить без флага», а это ~70 LOC правок без
  удаления — половина работы без выигрыша.

### 3.5 Цена ошибки (что будет, если Шифу выберет «подключить» потом)

Если в будущем появится реальная потребность в reflex-сценариях,
git revert этого PR + ADR-0054 возвращает всё на место. Код
не теряется — он в истории. Удаление обратимо.

---

## 4. Альтернативы, которые отвергнуты

| Альтернатива | Почему отвергнута |
|---|---|
| **«Подключить: отдать шину `SchedulerToolExecutor`»** (ADR-0080 §2.8, первый вариант) | §3.1 — требует shared assembly между `dialogue_node` и `command_node`, противоречит `SCHEDULER_DESIGN §1` «не плодить каналы между нодами». 400-600 LOC + e2e «стой!» во время песни на живом роботе. |
| **«Оставить с пометкой DEPRECATED»** | §3.4 — re-export + 1464 LOC тестов остаются как ловушка для следующего воркера; фоновый поток частично живёт. |
| **«Удалить только код, оставить тесты-фантомы»** | Тест без продового кода не имеет смысла (ADR-AF-0024, ADR-AF-0026 — вердикт по фантом-тестам). |
| **«Удалить только `reflex.py`, оставить мост в `command_node`»** | `command_node._init_bus` импортирует `ReflexLayer`; без него мост не компилируется. |
| **«Сделать PR из двух коммитов: (1) подключить, (2) через неделю удалить, если не работает»** | ADR-AF-0013 запрещает «PR под эксперимент с ретро-планом». Если подключение не даст эффекта — будет сложно доказать, что подключение было не нужно. |

---

## 5. План реализации (один PR, ~30-60 мин работы)

Коммиты (для читаемости в git log):

1. `wip(adr-0086): удалить scheduler/reflex.py и связанные тесты`
   — `git rm` шести файлов, правка `scheduler/__init__.py` (re-export),
   `scheduler/README.md`, `scheduler/task_scheduler.py` docstring,
   правка `command_node.yaml`.
2. `wip(adr-0086): удалить мост из command_node.py`
   — `command_node.py:14-17, 49-59, 106-126, 143-291, 309-314`.
3. `wip(adr-0086): синхронизировать документы`
   — `target-operator-agent-and-dialogue.md` §13 + §8а.1,
   `CONTEXT.md`, `ADR-0054` (архивная плашка), `ADR-0080` §2.8.
4. `ci(adr-0086): прогнать pytest + lint`

Порядок важен: (1) убирает публичный API → (2) убирает вызывающий
код → (3) убирает документацию. Между (1) и (2) проходит
`pytest` — он должен быть зелёным (никакой тест кроме удаляемых не
ссылается на `ReflexLayer`).

После (4) — `gh pr create` с body, ссылающимся на ADR, и
`kanban complete`. Шифу мержит после ревью.

Никакой e2e на реальном железе — этот PR **поведение на роботе не
меняет**: «стой!» продолжает работать через Nav2-cancel как и
раньше; фоновый поток и шина в `command_node` исчезают —
наблюдаемый эффект один: в логах `voice-assistant` перестаёт
появляться строка `🔌 ReflexLayer attached: subscribes to
/command/parsed, publishes to /reflex/events`.

---

## 6. Definition of Done

- [x] Решение записано (этот ADR).
- [ ] **Если подключаем** — заведена карточка реализации с критерием «„стой!" прерывает реплику на роботе». *(Не выбрано — см. §7)*
- [x] **Если удаляем** — карточка удаления заведена (этот PR, §5).
- [x] **В любом случае** — в репозитории не останется механизма, про который написано «подключено» и который не подключён (ADR-0018).
- [ ] CI зелёный на момент `kanban complete`.
- [ ] Merge-gate не нашёл конфликтов с активными PR (там сейчас идёт волна по voice-vr 22/23/24, см. свежие коммиты `7b72adfe`).

---

## 7. Открытые вопросы для Шифу

Этот ADR — **рекомендация архитектора**, не финальное решение.
Карточка #2245 явно говорит «Тип: решение владельца». Вопросы,
на которые владелец должен ответить до merge этого PR:

1. **«Стой!» через прерывание TTS — нужен в проде?**
   Варианты:
   - **(a) Не нужен.** Голосовой канал остаётся FIFO; «стой!» отменяет
     только Nav2 (как сегодня). Принимаем ADR-0086 как есть.
   - **(b) Нужен, но как priority-врезка** (после текущего чанка).
     Уже сделано в ADR-0066 + §8а.3 (`_normalize_tts_priority`). Ничего
     нового не пишем, просто документируем «стой!» через priority.
   - **(c) Нужен, через cancel.** Тогда это **отдельная** карточка
     implementation с честным e2e «„Стой!" во время песни» на
     живом роботе; ADR-0086 откладывается или отменяется.

2. **Архивировать ли ADR-0054 немедленно**, или пометить
   «будет воскрешён при варианте 1(c)»? Рекомендация:
   архивировать — revert из git, если что, восстановит.

3. **Если выбран вариант 1(a)** — удаляем ли заодно
   `command_node.yaml:17` (`enable_reflex_layer: true`) и
   `CONTEXT.md` строки про рефлекс? Рекомендация: да (полная
   чистка), иначе через релиз кто-то увидит флаг и решит «о,
   включу».

**Жду решения.** До получения ответа — никакого удаления не
делаю (только этот ADR в ветке `z-{agent}/2245-...`).

---

## 8. Связанные документы

- ADR-0080 §2.8 — прополка планировщика (открывает вопрос).
- ADR-0054 — предшествующая попытка моста (архивируется этим ADR).
- ADR-0066 — пауза личности через `/dialogue/control` (другой
  механизм «прервать личность», не через рефлекс).
- ADR-0056 — спекулятивная генерация (живой пакет
  `scheduler/pregen/*`; в `scheduler/pre_gen.py` — мёртвый
  однофамилец, под удаление в ADR-0080 §2.8 отдельной
  карточкой; этот ADR его не трогает).
- ADR-0018 инвариант 12 — формальный запрет «написанного, но не
  подключённого».
- `docs/architecture/target-operator-agent-and-dialogue.md` §8а.1
  таблица — рассинхронизирована с фактом; правка в §2.3.
- `src/rob_box_voice/rob_box_voice/scheduler/README.md:7-12` —
  v36-регресс, объясняет, почему FIFO voice канал = фича.
- `src/rob_box_voice/rob_box_voice/scheduler/task_scheduler.py:1-44`
  — docstring, упоминает reflex bridge как живого подписчика.
- `docs/design/SCHEDULER_DESIGN.md §8.10.6` — сценарии reflex,
  которые откладываются (не «удаляются» — фича всё ещё возможна
  в другой форме).
- issue #2245 (эта карточка), #1997 (предтеча), #1993 (исходный
  код reflex).
