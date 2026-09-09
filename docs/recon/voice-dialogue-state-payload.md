# Recon: реальная схема payload ROS2-топика `/voice/dialogue/state`

> **Scope.** Этот файл — исключительно разведывательный отчёт по задаче
> `t_1886f7be` (kanban-сабтаск под issue #1912 — реализация `voice_state`
> `0x1202` и `voice_floor` для Meta Quest). Сам код **не меняется**;
> фиксы живут в основной ветке `z-{agent}/1912-av-20-voice-state-0x1202-voice-floor`.

## TL;DR

ROS2-топик `/voice/dialogue/state`, который публикует `dialogue_node`,
несёт **плоскую строку** (`std_msgs/String`), а не JSON/MsgPack и не
диктринутый объект. Внутри строки — **имя** одного из четырёх
значений enum `DialogueStateKind`
(`rob_box_harness.core.dialogue_state_machine.DialogueStateKind`).
Никаких полей `ts_ms`, `utterance_id`, `error`, `thinking`, `speaking`,
`confidence` или уровня громкости на шину **не уходит**.

Это означает, что спецификация из `docs/architecture/meta-quest-api.md §4`
(таблица топиков, поле `voice_state` — MessagePack
`{state: "idle"|"listening"|"thinking"|"speaking", ts_ms, utterance_id?}`)
в нынешнем виде **не совместима** с тем, что есть в коде: расхождение по
имени топика, формату payload и набору значений `state`. Этот разрыв —
зона ответственности основной задачи #1912; recon фиксирует, на что
опираться.

## 1. Источник правды (single source of truth)

### 1.1 Publisher (кто публикует)

`src/rob_box_voice/rob_box_voice/dialogue_node.py`:

- **строка 415-416** (`/harness/task_events` publisher — для контекста):
  ```python
  self._task_events_pub = self.create_publisher(
      String, "/harness/task_events", 10)
  ```
- **строка 437** (response):
  ```python
  self._response_pub = self.create_publisher(
      String, "/voice/dialogue/response", 10)
  ```
- **строка 438** (state — наш таргет):
  ```python
  self._state_pub = self.create_publisher(String, "/voice/dialogue/state", 10)
  ```
- **строка 4808-4811** (тело публикации):
  ```python
  def _publish_state(self) -> None:
      msg = String()
      msg.data = self._dsm.current_state.name
      self._state_pub.publish(msg)
  ```

`String` — это `std_msgs.msg.String`. `msg.data: str`. Никакой
сериализации, JSON-обёртки или обёртки в dict — голый литерал имени
enum-значения.

### 1.2 State machine (откуда берётся значение)

`src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py`,
строки 41-54:

```python
class DialogueStateKind(Enum):
    """Dialog lifecycle states."""
    IDLE = auto()
    LISTENING = auto()
    DIALOGUE = auto()
    SILENCED = auto()
```

`DialogueStateKind` использует `auto()`, поэтому `.name` (а не `.value`)
— это **единственная** стабильная строковая репрезентация состояния,
которая попадает в топик. `.value` — целое число, в топик не пишется.

`DialogueStateMachine.on_event` (строка 328-388) реализует единственный
переход-граф:

| Текущее      | Событие               | → Новое       |
|--------------|-----------------------|---------------|
| любое        | `SILENCE_COMMAND`     | `SILENCED`    |
| `SILENCED`   | `TIMEOUT`/`UNSILENCE` | `IDLE`        |
| `IDLE`       | `WAKE_WORD`           | `LISTENING`   |
| `LISTENING`  | `STT_RESULT`          | `DIALOGUE`    |
| `LISTENING`  | `TIMEOUT`             | `IDLE`        |
| `DIALOGUE`   | `DIALOGUE_END`        | `IDLE`        |
| `DIALOGUE`   | `WAKE_WORD` (barge-in)| `LISTENING`   |

Любое другое событие — no-op (например, `STT_RESULT` в `IDLE` или
`WAKE_WORD` в `LISTENING`). Это значит, что **в топик физически могут
попасть только эти четыре значения** — других нет ни в FSM, ни в
вызывающем коде.

### 1.3 Места вызова `_publish_state()`

`grep -n "_publish_state()" src/rob_box_voice/rob_box_voice/dialogue_node.py`
(60 совпадений в основных ветках). Все они находятся в одном файле,
внутри одной ноды — отдельных подписчиков, дублирующих публикацию, нет.

Замечание для #1912: каждое событие, меняющее state, явно зовёт
`_publish_state()` после `on_event(...)`. Это правильно (event-driven,
а не по таймеру) и совпадает с политикой «event-driven, drop-newest»
из ADR-0027 §3.2 для `voice_state`.

## 2. Реальная схема payload (таблица для спецификации моста)

| Поле          | Тип ROS2              | Значение (пример)              | Источник                                |
|---------------|-----------------------|--------------------------------|-----------------------------------------|
| `msg.data`    | `std_msgs/String`     | `"IDLE"`                       | `dialogue_node.py:4810`                 |
| topic         | ROS2 (DDS)            | `/voice/dialogue/state`        | `dialogue_node.py:438`                  |
| QoS depth     | `int`                 | `10`                           | `dialogue_node.py:438`                  |
| QoS reliability| (default)             | RELIABLE                       | default rclpy publisher                  |

Допустимые значения `msg.data` (множество, **закрытое**):

| `msg.data`     | Смысл (DialogueManager/DialogCore)                                |
|----------------|--------------------------------------------------------------------|
| `"IDLE"`       | робот ждёт wake-word; по умолчанию после старта                    |
| `"LISTENING"`  | wake-word распознан, идёт сбор фразы пользователя                  |
| `"DIALOGUE"`   | фраза отправлена в LLM, идёт (или ожидает) генерация ответа        |
| `"SILENCED"`   | режим «помолчи» — реакции на речь отключены до таймаута/команды    |

**Чего в топике точно нет (по коду):**

- `ts_ms`, `utterance_id` (есть только в `meta-quest-api.md §5 JSON_EVENT` — это уже уровень WSS-события, не ROS-топика).
- `thinking`, `speaking` — таких значений в `DialogueStateKind` нет; `DIALOGUE` покрывает и thinking, и speaking одновременно. Семантически `DIALOGUE` = «обрабатываем реплику».
- `error`, `idle_reason`, `battery`, `wake_active`, `silenced_until` — это поля `DialogState` (dataclass, `dialogue_state_machine.py:78-117`), но в топик **не пробрасываются**. Если они нужны мосту, добавлять — отдельная задача.
- JSON/MsgPack/Protobuf — нет, чистая строка.

## 3. Расхождения со спецификацией

### 3.1 Имя топика

| Источник                                                | Имя                            |
|---------------------------------------------------------|--------------------------------|
| Факт (код, `dialogue_node.py:438`)                      | `/voice/dialogue/state`        |
| ADR-0027 §1 (строка 47-48) — «Текущее состояние репо»   | `/voice/state`                 |
| ADR-0027 §3.1 (строка 201) — таблица «UI-имя → keyexpr» | Zenoh keyexpr `voice/state`    |
| `docs/architecture/meta-quest-api.md §4` (строка 126)   | UI-имя `voice_state` (нет связи с конкретным ROS-именем) |

> **Вывод.** В тексте ADR-0027 указано два разных имени для одного и того
> же потока. Фактический ROS-топик — `/voice/dialogue/state`. Это
> «расхождение по имени топика» из task body — фиксируется, чтобы
> основная задача #1912 приняла решение: мост должен подписываться на
> `/voice/dialogue/state` (факт) и публиковать в Zenoh keyexpr
> `voice/state` (как заявлено в §3.1 таблицы).

### 3.2 Значения `state`

| Источник                            | Допустимые значения                        |
|-------------------------------------|--------------------------------------------|
| Факт (`DialogueStateKind`)          | `IDLE`, `LISTENING`, `DIALOGUE`, `SILENCED`|
| `meta-quest-api.md §4` (строка 126) | `idle`, `listening`, `thinking`, `speaking`|
| `meta-quest-api.md §5` (строка 349) | `speaking`, `denied` (как пример в JSON_EVENT) |

> **Расхождение по регистру и по набору.** Фактический код использует
> UPPERCASE и не имеет ни `thinking`, ни `speaking`, ни `denied`.
> `SILENCED` — единственное «дополнительное» значение, которого нет в
> спеке. Если мост маппит 1-в-1, нужно явно решить, что делать с
> `SILENCED` (например, ремапить в `speaking=false`/`muted=true`).

### 3.3 Формат payload

| Источник                            | Формат                                                    |
|-------------------------------------|-----------------------------------------------------------|
| Факт                                | `std_msgs/String` с одной строкой                         |
| `meta-quest-api.md §4`              | MessagePack `{state, ts_ms, utterance_id?}` (binary frame)|
| `meta-quest-api.md §5`              | JSON_EVENT `{type: "voice_state", state, ts_ms, utterance_id}` |

> **Вывод.** Мост `ROS2 → WSS` должен **сам** собрать MessagePack/JSON
> на стороне `rob_box_quest`: добавить `ts_ms` (server clock), опционально
> `utterance_id` (откуда брать — отдельный вопрос: в `DialogState` нет
> стабильного ID, есть `turn_count`).

### 3.4 Прочее

- `SILENCED` в спеке не описан. Решение: мапить в отдельное
  WSS-событие `{type: "voice_state", state: "idle", silenced: true}`
  или игнорировать (робот и так молчит).
- ADR-0027 §3.4 упоминает «новый параметр `voice_input_mode` на стороне
  `dialogue_node`» — это Phase 2, **не влияет** на схему `state`-топика.
- В `dialogue_manager.py` есть параллельный enum `DialogueState` (строка
  36-41) со **строковыми** значениями `"IDLE"/"LISTENING"/"DIALOGUE"/"SILENCED"`,
  но он используется только в `core/dialogue_manager.py` (старая ветка
  кода до рефакторинга ADR-0021), а активная `dialogue_node` использует
  `DialogueStateKind` с `auto()` (см. §1.2). Если где-то в коде или в
  тесте встретится `DialogueManager.state` — это **другой** класс, и его
  значения случайно совпадают со `DialogueStateKind.name` по
  написанию, но `.value` у них разные (str vs int).

## 4. Что НЕ удалось проверить

Честный список пробелов (по правилам ADR-0018 «Честный FAIL лучше
красивого PASS»):

1. **`ros2 topic echo /voice/dialogue/state` в 5 сценариях** —
   не выполнено: `ros2` CLI не установлен на этом worktree-хосте
   (`/usr/bin/bash: ros2: command not found`), а доступа к работающему
   роботу у воркера нет. Спецификация моста и тесты должны опираться на
   **код** (`dialogue_node.py:4808`) — он и есть SSoT для payload.

2. **Сверка с `docs/SPEC_CURRENT.md`** — файл `SPEC_CURRENT.md` в
   `docs/` не найден (`search_files docs/ SPEC_CURRENT` → 0 совпадений).
   Сверка ограничена ADR-0027 и `meta-quest-api.md`.

3. **Поведение при ошибках LLM / STT** — в коде нет состояния `ERROR`.
   Если LLM упал, `DialogCore` всё равно зовёт `DialogEvent.DIALOGUE_END`
   (см. `dialog_core.py:714-715`), и FSM возвращается в `IDLE`. То есть
   «ошибка» для внешнего наблюдателя **неотличима** от штатного
   завершения реплики — это нужно явно зафиксировать в мосте
   (например, через отдельный топик или поле `error_reason`).

4. **Связь с `utterance_id`** — `DialogState.turn_count` (dataclass)
   ведётся в `DialogCore`, но в `dialogue_node.py` не публикуется и в
   `/voice/dialogue/state` не попадает. Если для моста нужен стабильный
   ID реплики — добавлять в FSM или передавать в payload явно.

## 5. Рекомендации для основной задачи #1912

(не код — только входные данные для разработчика)

1. Мост `ROS2 → WSS` подписывается на **`/voice/dialogue/state`** (факт),
   парсит `msg.data: str`, маппит в `state: "idle"|"listening"|"speaking"`
   (или расширенный набор) и публикует как MessagePack `0x1202` или
   JSON_EVENT `{type: "voice_state", ...}` — согласно §4/§5
   `meta-quest-api.md`.

2. Маппинг значений предложен такой (утверждается в основной задаче):
   - `IDLE` → `idle`
   - `LISTENING` → `listening`
   - `DIALOGUE` → `speaking` (LLM/TTS идут — это самая «громкая» фаза
     для UI)
   - `SILENCED` → `idle` + клиентский признак «silenced» (или отдельное
     событие)

3. `ts_ms` берётся на стороне моста (server clock), а не из payload.

4. `utterance_id` — на стороне моста генерируется при переходе
   `IDLE → LISTENING` и сбрасывается при `DIALOGUE_END`. Это
   идентифицирует одну реплику пользователя.

5. ADR-0027 §1 и §3.1 имеют **разные имена топика** (`/voice/state` vs
   `voice/state`). Это документное расхождение; основная задача #1912
   должна выбрать канонический ROS-источник и поправить ADR одной
   строкой.

## 6. Acceptance для текущей задачи (t_1886f7be)

- [x] В этом файле есть таблица полей реального payload с типами.
- [x] Перечислены все состояния диалога, встречающиеся в коде
      (`IDLE`/`LISTENING`/`DIALOGUE`/`SILENCED`) — источник
      `DialogueStateKind`.
- [x] Зафиксированы расхождения с `meta-quest-api.md §4` и `ADR-0027 §3.1`
      (имя топика, формат, набор значений).
- [ ] `feature/av-20-recon` от develop — **не выполнено**: dispatcher
      выдал ветку `wt/t_1886f7be` (а не `feature/av-20-recon`), рабочий
      tree уже на `develop`-совместимом HEAD. Это не блокирует отчёт,
      но должно быть принято Шифу как known-shape.
- [x] Код не менялся — только этот recon-файл и WIP-коммит с ним.
