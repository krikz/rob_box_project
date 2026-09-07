# ADR-0051: Bridge.execute(Command) — единая типизированная точка входа в avatar_supervisor

| Поле | Значение |
|---|---|
| Статус | Proposed (Phase 1: IDL + сервис-заглушка; Phase 2: миграция клиентов; Phase 3: удаление legacy-сервисов) |
| Дата | 2026-09-07 |
| Автор | architect (по карточке t_2da4b2b4, issue #2002) |
| Контекст | Шов между Quest / Telegram / будущими web-клиентами и avatar_supervisor сейчас распылён по 3 ROS-сервисам (`acquire_floor`, `release_floor`, `set_avatar_mode`), 1 топику (`/avatar/set_voice_mode`), плюс прямой `cmd_vel_*` и `voice_audio` в обход супервизора. На стороне клиента это даёт 12 методов `QuestBridge` + 7 методов `SupervisorClient` ≈ 25 публичных сигнатур, что плохо тестируется и плохо документируется (issue #2002). |
| Затрагивает | новый docker-сервис `rob_box_supervisor_msgs` (IDL-пакет); supervisor_node (`AvatarSupervisor`); ws_server (Quest); supervisor_client (Telegram); QuestBridge |
| Родители | ADR-0028 (avatar_supervisor), ADR-0027 (Meta Quest / WebXR), ADR-0021 (dialogue_node discipline), ADR-0013 (incremental delivery), ADR-0018 (honesty) |
| Связанные | issue #2002; ADR-0028 §4.3 (ROS 2 API); `docs/architecture/target-operator-agent-and-dialogue.md` §13 Шаг 12 |

> **TL;DR.** Вводим типизированный IDL-пакет `rob_box_supervisor_msgs` с
> `Command` (msg union) и сервис `ExecuteCommand` (srv). Реализуем единую
> точку входа `Bridge.execute(Command)` — клиентский класс, который
> скрывает детали транспорта (Phase 1: facade поверх существующих
> сервисов; Phase 2: реальный сервис-вызов через IDL). Миграция
> `quest_node` и `telegram_node` — отдельными карточками (ADR-0013
> incremental). Полное удаление legacy-сервисов — Phase 3, только когда
> все клиенты переведены и есть 1 неделю стабильности.

---

## 1. Контекст и проблема

### 1.1. Текущее состояние (raw-evidence, 2026-09-07)

Карточка t_2da4b2b4 (issue #2002) утверждает: «типизированный IDL
`rob_box_supervisor_msgs` уже есть». Это **не соответствует
действительности** (см. raw в комментарии к карточке):

```text
$ find src -name "*.msg" -o -name "*.srv"
src/robot_sensor_hub_msg/msg/DeviceData.msg
src/robot_sensor_hub_msg/msg/DeviceSnapshot.msg
src/robot_sensor_hub_msg/msg/DeviceCommand.msg
src/rob_box_perception_msgs/msg/PerceptionEvent.msg
$ grep -rln "AvatarCommand\|SupervisorCommand" --include="*.msg" --include="*.srv" src/
(пусто)
```

IDL **отсутствует**. Аналогично отсутствуют
`docs/adr/0051-supervisor-operator-agent-arbiter-split.md` (этот ADR) и
`docs/architecture/target-operator-agent-and-dialogue.md` §13 Шаг 12.
Issue #2002 был сгенерирован GSD-волной как forward-reference без
backing-документации; этот ADR закрывает разрыв.

### 1.2. Что болит

Текущий API supervisor-а распылён:

- **3 ROS 2 service** (`/supervisor/acquire_floor`, `/supervisor/release_floor`, `/supervisor/set_avatar_mode`) — реализованы через `std_srvs/Trigger` с временными JSON-полями `client_id`/`floor`/`event` (техдолг, явно задокументирован в ADR-0028 §4.2 и docstring `supervisor_node.py:24-25`).
- **1 топик** (`/avatar/set_voice_mode`) — единственная Phase 1 точка для смены voice-mode.
- **Прямой `cmd_vel_*`** — и Quest, и Telegram **сами** публикуют в `twist_mux`, минуя супервизор (ADR-0028 §1.1, откуда race conditions).
- **Прямой `voice_audio`** — и Quest, и Telegram **сами** шлют в `/voice/audio/in`, минуя супервизор.

Со стороны клиентов это даёт:

- `QuestBridge` (`src/rob_box_quest/rob_box_quest/quest_node.py`) — 12 публичных клиентских методов: `publish_quest`, `publish_emergency`, `feed_client_alive`, `emergency_stop`, `publish_frame`, `available_streams`, `publish_voice_barge_in`, `publish_voice_audio`, `publish_voice_stop`, `publish_voice_robot_start`, `publish_voice_robot_stop`, `set_voice_mode`, `reset`.
- `SupervisorClient` (`src/rob_box_telegram/.../supervisor_client.py`) — 7 публичных методов: `acquire_floor`, `release_floor`, `with_floor`, `start_heartbeat`, `stop_heartbeat`, `subscribe_state`, `state`.

Суммарно ≈ 25 сигнатур (отсюда «25 методов» в issue). Каждая — точка
расширения, каждая требует отдельной документации, отдельного теста, и
отдельного решения «можно ли её дёргать из этого режима».

### 1.3. Чего мы хотим достичь

- **Один типизированный verb** для всех команд: `Command` (msg union).
- **Один сервис** для всех запросов: `ExecuteCommand` (srv).
- **Один клиентский API**: `Bridge.execute(Command) → Response`, без
  разнобоя сервисов.
- **Сохранение совместимости** в Phase 1 (legacy-сервисы
  параллельны с новым `ExecuteCommand`); миграция клиентов —
  Phase 2; удаление legacy — Phase 3.
- **Тестируемость**: матрица команд (какой `Command` → какой
  результат) проверяется **одним** unit-тестом, без тачков разных
  сервисов.

## 2. Решение

### 2.1. IDL: пакет `rob_box_supervisor_msgs`

Новый ROS 2 пакет (по образцу `rob_box_perception_msgs`):

```
src/rob_box_supervisor_msgs/
  CMakeLists.txt
  package.xml
  msg/
    Command.msg        # variant: kind + oneof { ... }
    Response.msg       # accepted + reason + held_by + mode + ...
  srv/
    ExecuteCommand.srv # Command → Response
```

**`Command.msg`** — типизированный union через `oneof`:

```
uint8 KIND_ACQUIRE_FLOOR    = 1
uint8 KIND_RELEASE_FLOOR    = 2
uint8 KIND_SET_AVATAR_MODE  = 3
uint8 KIND_SET_VOICE_MODE   = 4
uint8 KIND_EMERGENCY_STOP   = 5
uint8 KIND_HEARTBEAT        = 6

uint8 kind
string client_id            # кто отправил (для floor/mode)
string floor                # teleop | voice | "" (для других команд)
string avatar_event         # off | telegram_active | avatar_present | mixed | ... (для KIND_SET_AVATAR_MODE)
string voice_mode           # respeaker | quest_passthrough | quest_ttts | quest_stt | quest_llm_formalize | off
bool   emergency            # для KIND_EMERGENCY_STOP
```

> **Примечание.** ROS 2 msg-sintaks не имеет настоящих enum-union
> типов — `oneof` запрещён в msg (только в service/event). Поэтому
> `kind` (uint8) + много «пустых» строковых полей, как в существующем
> пакете `robot_sensor_hub_msg/msg/DeviceCommand.msg` (см. пример
> ниже). Это нормальная практика в rob_box.

Пример аналогии — `DeviceCommand.msg`:
```
uint8 KIND_UNKNOWN=0
uint8 KIND_PING=1
...
uint8 kind
string payload
```

**`Response.msg`**:

```
bool   accepted              # сервис принял запрос (≠ applied!)
bool   applied               # состояние реально изменилось
string reason                # human-readable: "held_by_other", "supervisor_in_monitor_mode", ...
string held_by               # для floor-команд: client_id текущего держателя
string actual_mode           # для SET_AVATAR_MODE: текущий режим после применения
bool   contacted_service     # для fallback-логики
```

**`ExecuteCommand.srv`**:

```
rob_box_supervisor_msgs/Command  command
---
rob_box_supervisor_msgs/Response response
```

### 2.2. Серверная сторона: `AvatarSupervisor.execute_command`

В `AvatarSupervisor` (`src/rob_box_supervisor/.../supervisor_node.py`)
добавляется:

```python
from rob_box_supervisor_msgs.srv import ExecuteCommand
from rob_box_supervisor_msgs.msg import Command, Response

# В __init__:
self._srv_execute = self.create_service(
    ExecuteCommand,
    "/supervisor/execute",
    self._on_execute_command,
)

def _on_execute_command(self, request, response):
    body = self._dispatch(request.command)
    response.response.accepted = body.get("accepted", False)
    response.response.applied = body.get("applied", False)
    response.response.reason = body.get("reason", "")
    response.response.held_by = body.get("held_by", "")
    response.response.actual_mode = body.get("actual_mode", "")
    response.response.contacted_service = body.get("contacted_service", True)
    return response
```

`_dispatch(Command) → dict` — единая таблица маршрутизации по `kind`,
которая в Phase 1 **делегирует** в существующие `_on_acquire_floor` /
`_on_release_floor` / `_on_set_avatar_mode` / `_on_set_voice_mode`. Это
даёт работающий IDL-канал **без** поломки legacy.

### 2.3. Клиентская сторона: `Bridge`

Новый общий модуль (предлагаемое место:
`src/rob_box_supervisor_msgs/.../bridge.py` или отдельный пакет
`rob_box_supervisor_client`). Минимальная реализация в этом PR:

```python
class Bridge:
    """Клиент avatar_supervisor (ADR-0051).

    Phase 1 (этот PR): facade — вызывает /supervisor/execute,
    маршрутизация делегирует в legacy-сервисы. Клиенты
    (quest_node, telegram_node) пока НЕ переведены (Phase 2).
    """

    SERVICE = "/supervisor/execute"

    def __init__(self, node, client_id: str, mode: str = "monitor"):
        self._node = node
        self._client_id = client_id
        self._mode = mode
        self._client = node.create_client(ExecuteCommand, self.SERVICE)

    def execute(self, command: Command) -> Response:
        if self._mode == "monitor":
            # grant locally, не дёргаем сервис
            return Response(accepted=True, applied=False,
                            reason="supervisor_in_monitor_mode")
        # Active: real service call
        ...
```

### 2.4. Scope этой карточки (Phase 1)

Что **входит** в PR карточки t_2da4b2b4:

- [x] Этот ADR (0051).
- [x] `docs/architecture/target-operator-agent-and-dialogue.md` §13 Шаг 12 — целевая архитектура.
- [x] Пакет `rob_box_supervisor_msgs` с IDL.
- [x] Сервер `/supervisor/execute` в `AvatarSupervisor` (фасад над legacy).
- [x] Класс `Bridge` с `execute(Command)` (фасад; в Phase 1 ходит через `/supervisor/execute`).
- [x] Тест-матрица команд: для каждого `kind` есть test_*.py, проверяющий «мост работает».
- [x] **НЕ** миграция `quest_node` и `telegram_node` (Phase 2).
- [x] **НЕ** удаление legacy-сервисов (Phase 3).

Что **не входит** (явно):

- Полное удаление `QuestBridge.publish_*` / `SupervisorClient.acquire_floor` — Phase 2/3.
- Полное удаление legacy-сервисов `acquire_floor`/`release_floor`/`set_avatar_mode` — Phase 3.
- Полное удаление топика `/avatar/set_voice_mode` — Phase 3.
- Реальный IDL-кабель для `cmd_vel_*` (всё ещё прямой `twist_mux`) — Phase 4, требует отдельного ADR.
- Реальный IDL-кабель для `voice_audio` — Phase 4.

### 2.5. Альтернативы (отклонённые)

| Альтернатива | Почему отклонена |
|---|---|
| Использовать существующие 3 сервиса, добавить 4-й, «без IDL union» | Не решает основную боль: клиентский API по-прежнему разнобой (12+7 сигнатур), и **невозможно добавить новую команду** без смены сигнатуры сервиса. |
| Сделать **один** топик `/supervisor/command` вместо сервиса | Теряем синхронный ответ (`granted`/`applied`/`reason`/`held_by`). Floor-логика требует синхронного ответа (race-safe). |
| Сделать `Bridge.execute(Command)` **сразу** с миграцией клиентов (Phase 2+3 в одном PR) | ADR-0013 incremental delivery; DoD-issue «git grep старых сигнатур → пусто» невозможно проверить в одном 30-минутном запуске. Разбиваем на инкременты. |
| Не вводить IDL-пакет, использовать `std_msgs/String` с JSON | Возвращает баг #1906 (silent JSON-decode-fallback). ADR-0018 запрещает silent fallback. |

## 3. План миграции (Phase 2/3/4)

### Phase 2 (отдельная карточка, ~1 неделя после стабилизации Phase 1)

- Перевод `quest_node` на `Bridge.execute(Command)` — все 12 публичных
  методов `QuestBridge` становятся тонкими обёртками, делегирующими в
  `bridge.execute(...)`. WSS-протокол клиента **не меняется** (только
  внутренний код ноды).
- Перевод `telegram_node` на `Bridge.execute(Command)` — `SupervisorClient`
  остаётся как совместимость-shim, новый код идёт через `bridge`.

### Phase 3 (отдельная карточка, ~2 недели после Phase 2)

- Удаление legacy-сервисов `acquire_floor`/`release_floor`/`set_avatar_mode`.
- Удаление топика `/avatar/set_voice_mode`.
- Удаление публичных методов `QuestBridge` (12) — оставляем только
  `execute(Command)`.
- Удаление публичных методов `SupervisorClient` (7) — оставляем только
  тонкий shim вокруг `bridge.execute`.

### Phase 4 (после Phase 3, требует отдельного ADR)

- IDL-кабель для `cmd_vel_*` через supervisor (а не прямой `twist_mux`).
- IDL-кабель для `voice_audio` через supervisor.

## 4. Trade-offs

**Плюсы:**

- Типизированный verb: новые команды добавляются без смены сигнатуры
  сервиса (только новое поле в `Command.msg`).
- Один клиентский API (`Bridge.execute`) для всех клиентов —
  тестируется одной куда.
- Сохраняем backward-compat: Phase 1 не ломает ни Quest, ни Telegram.
- ADR-0013 соблюдён: incremental, проверяемый в каждом PR.

**Минусы:**

- Промежуточная сложность: на время Phase 1+2 в репо живут и новый
  `/supervisor/execute`, и legacy `/supervisor/*` сервисы.
- Тесты растут: матрица команд × (quest, telegram) клиентов — нужно
  поддерживать.
- Один лишний слой (`_dispatch` в supervisor) — overhead, но
  минимальный (Python-dict lookup).

**Почему не альтернативы:** см. §2.5.

## 5. Verification (Phase 1)

- [ ] `colcon build --packages-select rob_box_supervisor_msgs` зелёный.
- [ ] `pytest src/rob_box_supervisor_msgs/test/ -v` зелёный (тест-матрица
      `Command.kind` → ожидаемый `Response`).
- [ ] `pytest src/rob_box_supervisor/test/ -v` зелёный (legacy-тесты
      supervisor не сломаны).
- [ ] `pytest src/rob_box_quest/test/ -v` зелёный (Quest-тесты не
      сломаны).
- [ ] `pytest src/rob_box_telegram/test/ -v` зелёный (Telegram-тесты не
      сломаны).
- [ ] `git grep "supervisor_msgs::Command\|ExecuteCommand"` показывает
      только новые файлы (нет референсов в legacy-коде, кроме
      `_dispatch` в supervisor).

Phase 2/3 verification — в их собственных карточках.

## 6. Связанные документы

- ADR-0028 §4.3 (текущий ROS 2 API supervisor-а) — будет обновлён в Phase 3, когда legacy-сервисы удаляются.
- ADR-0027 §3.4 (voice modes) — `voice_mode` остаётся тем же enum, мигрирует в `Command.voice_mode`.
- `docs/architecture/target-operator-agent-and-dialogue.md` §13 Шаг 12 — целевая архитектура, которая закладывает этот ADR.

## 7. История

- 2026-09-07 — Proposed (этот ADR), Phase 1 в работе (t_2da4b2b4 / issue #2002).