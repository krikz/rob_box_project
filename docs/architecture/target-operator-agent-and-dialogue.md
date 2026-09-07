# Target Operator-Agent and Dialogue — целевая архитектура

> **Статус:** living document. Каждый «Шаг» — отдельная карточка в
> kanban-е. Шаги 04a–11 относятся к **агенту оператора** (LLM-цикл,
> tools, memory); Шаг 12 — к **сужению seam-а** между клиентами и
> supervisor-ом (эта карточка, t_2da4b2b4).

| Поле | Значение |
|---|---|
| Дата начала | 2026-09-05 (первый handoff от GSD-волны) |
| Автор | architect |
| Цель | Заменить параллельные «ручки» (QuestBridge.publish_*, SupervisorClient.*, прямые ROS-вызовы) единым типизированным verb-ом `Bridge.execute(Command)` и подготовить фундамент для **оператора** как LLM-агента (Phase 2+). |
| Родительский ADR | ADR-0051 (этот шаг — §13 / Шаг 12) |

---

## §1. Зачем это всё

Сейчас архитектура «аватара» имеет **три** параллельные точки
управления (см. ADR-0028 §1.1):

1. **Quest** (`rob_box_quest`) — WebXR-клиент; команда → QuestBridge → ROS-топики.
2. **Telegram** (`rob_box_telegram`) — long-polling бот; команда → SupervisorClient → ROS-сервисы + прямые publishers.
3. **Будущий web-admin / CLI** — пока нет, но архитектура должна его вместить без ещё одного «зоопарка».

Каждый клиент сам публикует `cmd_vel_*` в `twist_mux`, сам шлёт в
`/voice/audio/in`, сам ходит в supervisor-сервисы для floor-логики.
Это даёт:

- Race conditions (Quest едет, а Telegram шлёт `/forward`).
- Дублирование логики «кто сейчас может ехать / говорить».
- 12+7+3+1 = 25+ сигнатур без единой документации.
- Невозможность добавить **оператора** (LLM-агента) — у него нет
  единой поверхности для вызова команд.

**Цель:** один типизированный verb (`Command`) + один сервис
(`ExecuteCommand`) + один клиентский API (`Bridge.execute(Command)`).

**Бонус-цель:** когда `Bridge` станет общим, **оператор** (LLM-агент)
сможет вызывать те же команды, что и человек через Quest — это
основа для Шагов 04a–11 (агентский цикл).

---

## §2. Принципы

1. **Типизация первая**: любая новая команда = новое поле в `Command.msg`,
   а не новый ROS-сервис. Никаких `std_srvs/Trigger` с временными JSON.
2. **Одна точка входа**: `Bridge.execute(Command)` — никаких обходных
   сервисов для клиентов.
3. **Backwards-compat в Phase 1**: legacy-сервисы `acquire_floor` /
   `release_floor` / `set_avatar_mode` + топик `/avatar/set_voice_mode`
   остаются работать параллельно с новым `/supervisor/execute`. Это
   ADR-0013 (incremental).
4. **Honest fallback**: если новый сервис недоступен, **логируем
   rate-limited WARN** и возвращаем «applied=false, reason=…», а не
   глотаем и не молчим. ADR-0018.
5. **Тесты-матрица**: одна таблица `Command.kind → Response` тестируется
   на стороне supervisor и на стороне клиента.

## §3. Текущее состояние (raw-evidence, 2026-09-07)

```text
$ find src -name "*.msg" -o -name "*.srv"
src/robot_sensor_hub_msg/msg/DeviceData.msg
src/robot_sensor_hub_msg/msg/DeviceSnapshot.msg
src/robot_sensor_hub_msg/msg/DeviceCommand.msg
src/rob_box_perception_msgs/msg/PerceptionEvent.msg
$ grep -rln "AvatarCommand\|SupervisorCommand" --include="*.msg" --include="*.srv" src/
(пусто)
```

IDL `rob_box_supervisor_msgs` отсутствует; Шаг 12 (этот) **создаёт** его.

---

## §4. Карта шагов (вне этого документа)

| Шаг | Описание | Карточка / issue | Статус |
|---|---|---|---|
| 01 | Рефакторинг dialogue_node на единый сodec /avatar/state (msgpack) | AV-14 / #1906 | MERGED (PR #1922) |
| 02 | Supervisor: monitor-режим (AcquireFloor / ReleaseFloor / SetAvatarMode) | AV-6 / #968 wave2 | MERGED (PR #1857) |
| 03 | Supervisor: msgpack-кабель для /avatar/state | AV-14 / #1906 | MERGED (PR #1922) |
| 04a | Operator agent: LLM-цикл (intent → plan → execute) | — | planned |
| 04b | Operator agent: tool-catalog (Bridge.execute + Memory + т.д.) | — | planned |
| ... | ... | ... | ... |
| 11 | Operator agent: e2e-верификация (Quest + Telegram + Operator в одной сцене) | — | planned |
| **12** | **Сужение Quest seam: Bridge.execute(Command) вместо 25 методов** | **t_2da4b2b4 / #2002** | **WIP (этот документ, ADR-0051)** |

---

## §5. Что вне scope этого документа

- Шаги 04a–11 (агентский цикл оператора) — отдельные карточки.
- Полная миграция `quest_node` и `telegram_node` на новый клиент — Phase 2.
- Удаление legacy-сервисов — Phase 3.
- IDL-кабель для `cmd_vel_*` через supervisor — Phase 4, отдельный ADR.
- IDL-кабель для `voice_audio` через supervisor — Phase 4.

---

## §6. Приложение: справочник команд

Текущий «verb-словарь» (Phase 1):

| `Command.kind` | Поля | Где обрабатывается | Phase 1 поведение |
|---|---|---|---|
| `KIND_ACQUIRE_FLOOR` | `client_id`, `floor` | `AvatarSupervisor._acquire_floor_logic` | Делегирует в legacy `_on_acquire_floor` |
| `KIND_RELEASE_FLOOR` | `client_id`, `floor` | `AvatarSupervisor._release_floor_logic` | Делегирует в legacy `_on_release_floor` |
| `KIND_SET_AVATAR_MODE` | `client_id`, `avatar_event` | `AvatarSupervisor._set_avatar_mode_logic` | Делегирует в legacy `_on_set_avatar_mode` |
| `KIND_SET_VOICE_MODE` | `client_id`, `voice_mode` | `AvatarSupervisor._apply_voice_mode` | Делегирует в legacy `_on_set_voice_mode` (топик) |
| `KIND_EMERGENCY_STOP` | `client_id` | (новое) `AvatarSupervisor._on_emergency_stop` | Прямо сейчас публикует `cmd_vel_emergency=0` (Phase 2 — через IDL) |
| `KIND_HEARTBEAT` | `client_id`, `floor` | (новое) `AvatarSupervisor._on_heartbeat` | Phase 1 — no-op, фиксирует только счётчик; Phase 2 — реальный lock-renewal |

> **Примечание.** `EMERGENCY_STOP` и `HEARTBEAT` — **новые** команды,
> не имеющие legacy-аналога. Они появляются именно в Phase 1 этого
> шага, чтобы клиенты **сразу** могли использовать единый verb для
> всех команд (а не для части + legacy для остатка).

---

## §7. История

- 2026-09-05 — first handoff от GSD-волны (issue #2002) без backing-документации.
- 2026-09-07 — этот документ создан (t_2da4b2b4 / ADR-0051).