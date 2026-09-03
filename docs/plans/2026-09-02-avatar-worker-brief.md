# Avatar-эпик: бриф воркера — замороженные контракты и правила

> **Читать ПЕРВЫМ, до того как открывать свою карточку AV-12…AV-28.**
>
> Семнадцать карточек делаются параллельно разными воркерами. Если каждый
> придумает своё имя топика, свой формат payload и свой код ошибки — мы
> получим ещё один месяц зелёных тестов при мёртвом проводе (ровно это и
> нашёл аудит `docs/plans/2026-09-02-avatar-epic-state-audit.md`).
>
> Поэтому всё, что пересекает границу между нодами, **заморожено ниже**.
> Отступление от таблиц §1–§4 — не «улучшение», а поломка чужой карточки.
> Если считаешь, что в таблице ошибка — **не меняй молча**: напиши в свой
> issue комментарий с обоснованием и жди решения товарища Шифу.

| Поле | Значение |
|---|---|
| Эпик | Avatar (Quest + Telegram + supervisor) |
| Milestone | M4: Avatar (Quest + Telegram) |
| Карточки | #1904–#1920 (AV-12 … AV-28) |
| Аудит-основание | `docs/plans/2026-09-02-avatar-epic-state-audit.md` |
| Базовая ветка | `origin/develop` (ADR-0045). `feature/avatar` **удалена** — не ссылаться (#1902) |

---

## 0. Пять разрывов, из-за которых всё это заведено

Чтобы не повторить: каждый из них жил под зелёными тестами, потому что
**каждая сторона тестировалась своим моком, а стык — ничем**.

| # | Разрыв | Чинит |
|---|---|---|
| G1 | Сервисы floor на `std_srvs/Trigger` — у Request нет полей, `client_id`/`floor` передать нечем | AV-12 |
| G2 | `/teleop_heartbeat` — один издатель (Telegram), ноль подписчиков; dead-man не питается | AV-13 |
| G3 | `/avatar/state` — msgpack на издателе, `json.loads` на потребителе, ошибка глотается молча | AV-14 |
| G4 | `SupervisorClient._acquire_via_service` — заглушка, всегда локальный grant | AV-15 |
| **G25** | **Имена сервисов не совпадают**: супервизор объявляет относительные `acquire_floor` (→ `/acquire_floor`), Telegram зовёт `/supervisor/acquire_floor`. Zenoh-wrapper (`ros_with_namespace.sh`) задаёт **только** Zenoh-namespace, ROS-namespace ноды не меняет | AV-12 |

**Вывод, который обязателен к применению во всех карточках:** любой PR,
который меняет что-то на границе двух нод, обязан содержать **тест на
стык** — сериализатор/издатель одной стороны кормит десериализатор/
потребитель другой. Тест «моя сторона умеет паковать» не считается.

---

## 1. Замороженные ROS-имена

### 1.1. Сервисы супервизора — **абсолютные**

| Имя | Тип (после AV-12) | Кто сервер | Кто клиенты |
|---|---|---|---|
| `/supervisor/acquire_floor` | `rob_box_supervisor_msgs/srv/AcquireFloor` | `avatar_supervisor` | Telegram, quest-сервер |
| `/supervisor/release_floor` | `rob_box_supervisor_msgs/srv/ReleaseFloor` | `avatar_supervisor` | Telegram, quest-сервер |
| `/supervisor/set_avatar_mode` | `rob_box_supervisor_msgs/srv/SetAvatarMode` | `avatar_supervisor` | Telegram, quest-сервер |

**Решение по G25:** источник истины — **абсолютные** имена с префиксом
`/supervisor/` (так уже написано в `supervisor_client.py:112-114`).
AV-12 обязан поменять **супервизор** (`ACQUIRE_FLOOR_SERVICE = "acquire_floor"`
→ `"/supervisor/acquire_floor"`, `supervisor_node.py:108-110`), а не клиентов.
Причина: клиентов трое (Telegram + quest-сервер + будущий web), сервер один.

### 1.2. Топики

| Топик | Тип | Издатель | Потребители |
|---|---|---|---|
| `/avatar/state` | `std_msgs/String` (msgpack, см. §3.1), QoS `transient_local` depth 1 | `avatar_supervisor` | Telegram, quest-сервер |
| `/teleop_heartbeat` | `rob_box_supervisor_msgs/msg/TeleopHeartbeat`, best-effort depth 10 | Telegram, quest-сервер | `avatar_supervisor` |
| `/avatar/set_voice_mode` | `std_msgs/String` | quest-сервер | `avatar_supervisor` |
| `/avatar/voice_in` | `audio_common_msgs/AudioData`, **int16 PCM 16 kHz mono**, best-effort + volatile | quest-сервер, Telegram (AV-23) | `sound_node` |
| `/audio/quest_in` | `audio_common_msgs/AudioData` | quest-сервер | `stt_node` |
| `/voice/stt/quest` | `std_msgs/String` | `stt_node` | `dialogue_node` |
| `/voice/dialogue/state` | `std_msgs/String` | `dialogue_node` | `avatar_supervisor`, quest-сервер (AV-20) |
| `/avatar/command` | `std_msgs/String` (JSON, §3.3) | `dialogue_node` (режим команды), Telegram | супервизор-агент |
| `/avatar/command_result` | `std_msgs/String` (JSON, §3.3) | супервизор-агент | Telegram, quest-сервер |

⚠️ Топик состояния диалога — **`/voice/dialogue/state`**, НЕ `/voice/state`.
Это зафиксированное расхождение с ADR-0027 (#2). Не «исправлять».

### 1.3. Формат `client_id`

```
quest:<session_id>        # формирует quest-СЕРВЕР, не клиент
telegram:<chat_id>        # формирует telegram-нода
```

**Инвариант:** `client_id` всегда формирует серверная сторона по факту
сессии. Значение, присланное клиентом в payload, — не источник истины;
расхождение логировать. Иначе клиент в очках сможет представиться
Telegram'ом и отобрать чужой floor.

---

## 2. Замороженный wire-протокол квеста

### 2.1. Фреймы

Формат кадра не меняется: `[1 byte type][4 bytes stream_id LE][LEB128 len][payload]`.

| Байт | Имя | Направление | Payload | Карточка |
|---|---|---|---|---|
| `0x30` | `SET_MODE` | client → server | msgpack `{client_id, mode}` | AV-16 |
| `0x31` | `ACQUIRE_FLOOR` | client → server | msgpack `{client_id, floor}` | AV-16 |
| `0x32` | `RELEASE_FLOOR` | client → server | msgpack `{client_id, floor}` | AV-16 |
| `0x33` | `STATE_UPDATE` | server → client | msgpack `{state: <AvatarState §3.1>}` | AV-16 |

`stream_id` для `0x30`–`0x33` = `0`. Payload — **msgpack**, не JSON.
`STATE_UPDATE` — не стрим: подписки (`SUBSCRIBE`) он не требует.

### 2.2. Subprotocol

| Значение | Что даёт |
|---|---|
| `robbox-quest-v1` | всё, что было; фреймы `0x30`–`0x33` недоступны |
| `robbox-quest-v2` | v1 + `0x30`–`0x33` + рассылка `STATE_UPDATE` |

Сервер анонсирует `("robbox-quest-v2", "robbox-quest-v1")`, версию выбирает
клиент. **v1-сессия, приславшая `0x30`–`0x32`, получает `ERROR{PROTOCOL_VERSION}`**
(не «молча игнорируем» — тихое игнорирование и есть механизм, спрятавший
G1–G4). v1-сессии `STATE_UPDATE` не рассылается.

### 2.3. Стримы (topic_id) — существующие, не переопределять

`camera_rear 0x1001`, `camera_front 0x1002`, `camera_oak_color 0x1003`,
`lidar_2d 0x1101`, `lidar_3d 0x1102`, `robot_status 0x1201`,
`voice_state 0x1202`, `person_detections 0x1301`.

### 2.4. Коды ошибок

| Код | Когда |
|---|---|
| `FLOOR_HELD` | floor занят другим `client_id`; либо release чужого floor |
| `MODE_CONFLICT` | FSM отклонила переход `SET_MODE` |
| `PROTOCOL_VERSION` | v1-сессия прислала v2-фрейм |
| `BAD_PAYLOAD` | невалидный msgpack/JSON, неизвестный `floor`/`mode` |
| `TOPIC_UNKNOWN` | топика нет в реестре |

---

## 3. Замороженные схемы данных

### 3.1. `AvatarState` и его транспорт

Поля — как в `src/rob_box_supervisor/rob_box_supervisor/core/state.py`
(не добавлять новых в рамках этих карточек):

```
mode: str                  # off | telegram_active | avatar_present | mixed | teleop_only | voice_only
teleop_floor: FloorState   # {client_id: str|"", since_ms: int, last_heartbeat_ms: int}
voice_floor:  FloorState
last_event: str
since_ms: int
version: int
```

**Транспорт — ровно один путь, и он живёт в одном модуле** (AV-14):

```
core/state.py:
    encode_for_ros_string(state) -> str   # msgpack.packb → .decode("latin-1")
    decode_from_ros_string(data) -> AvatarState
```

Все читатели `/avatar/state` — Telegram (AV-14/15/24), quest-сервер
(AV-16) — обязаны звать `decode_from_ros_string`. **Свой парсер писать
запрещено.** `json.loads` на этом топике — то, что мы чиним.

**Запрещён тихий fallback на другой кодек.** Нет msgpack → ERROR и не
публикуем. Именно «тихо переключимся на JSON» спрятало G3.

### 3.2. Поля srv/msg (AV-12) — точный список

```
AcquireFloor.srv    req: string client_id, string floor
                    res: bool granted, string held_by, string reason, bool applied
ReleaseFloor.srv    req: string client_id, string floor
                    res: bool success, string reason, bool applied
SetAvatarMode.srv   req: string client_id, string mode
                    res: bool success, string mode, string reason, bool applied
TeleopHeartbeat.msg string client_id, uint64 ts_ms, uint32 seq
FloorState.msg      string client_id, uint64 since_ms, uint64 last_heartbeat_ms
AvatarStateMsg.msg  string mode, FloorState teleop_floor, FloorState voice_floor,
                    string last_event, uint64 since_ms, uint32 version
```

`reason` ∈ `""` | `held_by_other` | `mode_conflict` | `bad_request` | `monitor` | `supervisor_unavailable`.
`floor` ∈ `teleop` | `voice`.

### 3.3. Вход/выход супервизор-агента (AV-21/AV-22)

```
/avatar/command         {request_id, source: "quest"|"telegram", client_id, text, ts_ms}
/avatar/command_result  {request_id, ok: bool, summary: str, tool_calls: [str]}
```

---

## 4. Замороженные константы и параметры

| Параметр | Нода | Default | Карточка |
|---|---|---|---|
| `dead_man_timeout_ms` | `avatar_supervisor` | `500` | AV-13 |
| `require_teleop_floor` | `rob_box_quest` | `false` | AV-19 |
| `require_voice_floor` | `rob_box_quest` | `false` | AV-20 |
| `supervisor_required` | `rob_box_telegram` | `false` | AV-15 |
| `enabled` (агент) | супервизор-агент | `false` | AV-21 |
| `radio_max_duration_s` | `rob_box_telegram` | `30` | AV-23 |
| `mode` | `avatar_supervisor` | `monitor` (в compose — `active`) | — |

Частота heartbeat — **10 Гц**. Порог dead-man — **500 мс**.
Keep-alive `STATE_UPDATE` — **1 Гц** плюс на каждое изменение.

**Все новые гейты по умолчанию выключены.** Причина: мостик и бот сегодня
работают на роботе, и карточка, которая их выключит «потому что так
правильнее», сломает демонстрацию. Включение — отдельным коммитом после
e2e, решением товарища Шифу.

**Ключ localStorage раскладки** — `rob_box_quest.panel_layout.v1` (AV-25),
имя зафиксировано ещё в аудите 30.08.

---

## 5. Порядок и что можно делать параллельно

```
W1  AV-12 ──┬── AV-13
   (IDL)    ├── AV-14 ──┬── AV-15 ──┬── AV-24
            └───────────┘           │
W2  AV-16 ──┬── AV-17 ── AV-18      │
   (wire)   ├── AV-19 ──────────────┘
            └── AV-20
W3  AV-21 ── AV-22
W4  AV-23 (независима, но voice-floor берёт из AV-15)
W5  AV-25, AV-26, AV-27, AV-28 — независимы от W1–W3
```

**Что это значит на практике.** Контракты §1–§4 заморожены здесь, поэтому
писать код можно **параллельно**, не дожидаясь мержа предшественника:
воркер AV-17 знает точный формат `STATE_UPDATE`, не заглядывая в AV-16.

Но **мержить** в этом порядке. Если твоя карточка помечена «блокируется
AV-N» и AV-N ещё не влит:

1. Пиши код и тесты против таблиц этого брифа.
2. Импорты чужого пакета (`rob_box_supervisor_msgs`) делай **ленивыми
   try-import** с деградацией — так уже сделано для `vesc_msgs`
   (`quest_node.py:490`) и `rclpy.qos` (`supervisor_node.py:155`).
3. В описании PR первой строкой: `Блокируется #<N> — не мержить раньше`.
4. **Не** копируй к себе куски чужой карточки «чтобы заработало». Два
   определения одного контракта — это ровно тот дефект, который мы чиним.

**AV-25, AV-26, AV-27, AV-28 не зависят ни от кого** — если ищешь, с чего
начать без ожидания, бери их.

---

## 6. Инварианты проекта (нарушение = PR не принимается)

1. **Честный FAIL лучше красивого PASS** (ADR-0018, AGENTS.md). Не
   прогнал — не пиши «прогнал». Raw-вывод обязателен: `pytest -v`,
   `npm test`, `ros2 topic echo`, `docker logs`, ссылка на CI-run.
2. **Никакого тихого fallback.** Не удалось декодировать / вызвать сервис /
   получить ответ LLM — шуми (WARN с rate-limit, счётчик, видимое
   состояние в UI). Молчаливая деградация — причина всех пяти разрывов §0.
3. **«Неизвестно» ≠ «свободно».** Пока `STATE_UPDATE`/`/avatar/state` не
   пришёл ни разу — показывать `?`, а не «floor свободен».
4. **Инструменты LLM — только из `rob_box_core.tool_catalog`.** Ни одного
   параллельного объявления схемы тула (убило Compositor, `e96b912d`).
5. **Чужие параметры — только через супервизор.** `SetParameters` на
   `dialogue_node`/`tts_node` из `rob_box_quest` или Telegram запрещён
   (ADR-0028 S5/S12). Путь: клиент → супервизор → параметр.
6. **Логика отдельно от рендера и от ROS.** В `webxr_client` — чистая
   функция (`parseX`, `formatX`, `hitTest`) + тонкий слой three.js; в
   Python — чистый модуль в `core/` + тонкая нода. Так устроены
   `status_hud.ts`, `lidar_payload.ts`, `core/fsm.py`; повторяй.
7. **Время подменяемо.** Никаких прямых `time.time()` / `Date.now()` в
   новой логике — часы инжектятся, иначе dead-man и троттлинг не
   протестировать.
8. **Документация правится тем же PR.** Меняешь список режимов —
   обнови ADR-0027 §3.4. Меняешь фреймы — обнови `meta-quest-api.md`.
   Разъехавшиеся док и код — ADR-0043/ADR-0044, у нас это уже классика.
9. **Аварийная остановка вне любых гейтов.** `stop_emergency` работает
   всегда, у кого бы ни был floor.

---

## 7. OpenSpec — обязательная часть работы (ADR-0038, ADR-0039)

Change-folder создаёт триаж автоматически при постановке карточки. Руками
не создавай. Твоя работа — **расширить скелет**:

- `proposal.md` — Why / What Changes / Impact конкретным текстом (не HTML-комменты).
- `specs/<capability>/spec.md` — ADDED Requirements + сценарии.
- `design.md` — Context, Goals/Non-Goals, Decisions, Risks.
- `tasks.md` — реальные шаги с чекбоксами (≤ 2 ч каждый).
- `openspec validate <change>` — PASS до мержа.

Capability для этого эпика — одна из шести, **не заводить новых**:

| Capability | Карточки |
|---|---|
| `avatar-floor-protocol` | AV-12, AV-13, AV-14, AV-19 |
| `captain-bridge-supervisor-api` | AV-16, AV-17, AV-18, AV-20 |
| `supervisor-agent` | AV-21, AV-22 |
| `telegram-avatar-client` | AV-15, AV-23, AV-24 |
| `captain-bridge-layout` | AV-25 |
| `captain-bridge-telemetry` | AV-26 |
| `captain-bridge-tts-picker` | AV-27 |
| `quest-voice-presets` | AV-28 |

Скилл с полным жизненным циклом — `.agents/skills/openspec-workflow/SKILL.md`.

---

## 8. Чек-лист перед «готово»

- [ ] Ветка от `origin/develop`, не от `feature/avatar` (её нет).
- [ ] Есть **тест на стык**, если PR трогает границу двух нод.
- [ ] Ни одного тихого `except: pass` на пути декодирования/вызова.
- [ ] Часы инжектятся; новых `time.time()`/`Date.now()` в логике нет.
- [ ] Контракты §1–§4 не изменены (или изменение согласовано в issue).
- [ ] OpenSpec change-folder расширен, `openspec validate` PASS.
- [ ] Доки, которых касается изменение, поправлены тем же PR.
- [ ] Raw-вывод приложен: `pytest -v` / `npm test` / `typecheck` / CI-run.
- [ ] `black --check --line-length 120` + `flake8 --max-line-length 120` чисто (Python).
- [ ] Если `needs-e2e` — e2e реально прогнан, вывод приложен. Не прогнан — метку не ставить.
