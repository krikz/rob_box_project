# ADR-0010: Perception architecture — UART bridge + ROS2 aggregator as two complementary nodes

| Поле         | Значение                                                                |
|--------------|-------------------------------------------------------------------------|
| Статус       | **Accepted** (retroactive)                                              |
| Дата         | 2026-07-28                                                              |
| Автор        | architect (Hermes Agent)                                                |
| Формат       | MADR (Markdown Any Decision Record)                                     |
| Контекст     | Phase 6 v2 (harness-p0-finalization), W11 wave; Kanban `t_2ff2e9ad`     |
| Родители     | `t_988e4052` (architect review PASS-WITH-FIXES), ADR-0001 §2.7           |
| Связанные ADR | ADR-0001 (harness architecture), 06-CONTEXT.md §D-05/D-06               |
| Закрывает    | child-блокер `t_988e4052` §G2 / §PHASE-STATUS.md W11 row                |

---

## 1. Контекст и проблема

### 1.1 План vs реальность

Фаза 6 v2 план (`06-CONTEXT.md` §D-05/D-06, `06-04-PLAN.md` W11) предписывал:

> **D-05**: Perception = UART-мост, без LLM. micro-ROS контейнер выкидываем. Сенсор-борд получает свою прошивку (вне скоупа, не Python). Perception нода = UART → ROS2 топики (~200 строк).
> **D-06**: Старые ноды удаляются полностью. `context_aggregator_node.py` (745 строк) + 4 другие perception-ноды → заменяются UART-мостом.

В `06-04-PLAN.md` W11 также явно сказано:

> **Output**: 1 new perception_bridge.py, **4 old nodes deleted**, 1 node (health_monitor) optionally kept, integration tests.

То есть ожидалась **консолидация 5 perception-нод в 1 UART-мост**.

### 1.2 Что реально произошло (W10 + W11)

| Коммит       | Действие                                                                                          |
|--------------|---------------------------------------------------------------------------------------------------|
| `7552418a` (W10) | Удалены `reflection_node.py`, `startup_greeting_node.py`, `vision_stub_node.py` (LLM-зависимые). context_aggregator + health_monitor **оставлены** |
| `85cfd62e` (W11) | Создан `perception_bridge.py` (198 строк, UART → `/sensors/data` + `/perception/health`). Launch-файлы обновлены, `setup.py` entry_points исправлен |

После W10+W11:
- ✅ 3 LLM-зависимые ноды удалены
- ✅ `perception_bridge.py` создан и работает (198 строк, ≤200 target)
- ✅ Launch-файлы (internal + docker) стартуют `perception_bridge`
- ✅ LLM-код отсутствует в perception
- ⚠️  **`context_aggregator_node.py` (523 строки) и `health_monitor.py` (163 строки) НЕ удалены** — расхождение с планом «5 → 1»

### 1.3 Почему план не выполнен буквально

При ревью (`t_988e4052` §A.2.3, §A.2.14, mermaid §6.2) выяснилось, что:

1. **`mcp_server.py` (harness-side) подписан на `/perception/context_update`** через `PerceptionEvent` message. Без `context_aggregator` этот топик не существует → harness теряет perception-context для LLM.
2. **`context_aggregator` после W10 — это НЕ legacy-LLM-нода**. Это **ROS2-topic-агрегатор**: подписан на 9 живых топиков (`/perception/vision_context`, `/rtabmap/localization_pose`, `/odom`, `/dynamic_joint_states`, `/rosout`, `/voice/stt/result`, `/voice/dialogue/response`, `/voice/command/intent`, `/voice/command/feedback`) и публикует нормализованный `PerceptionEvent`.
3. **Источники данных принципиально разные**:
   - `perception_bridge` = hardware-attached sensor board (UART) — нет ROS2-топика-источника, есть только serial port
   - `context_aggregator` = ROS2-топики от других нод (vision, odometry, voice, commands) — нет serial port
4. **Объединение двух нод в одну создало бы over-coupling**: UART-код и ROS2-агрегация имеют разные lifecycle, разные failure modes (hardware-absent vs topic-missing), разные QoS.

### 1.4 Конфликт плана и реализации

Архитектурное ревью `t_988e4052` зафиксировало W11 как **«done (broader scope than plan)»**, но **PHASE-STATUS.md** помечает W11 как расхождение с планом (G8/G9), и дочерняя задача `t_2ff2e9ad` была создана именно для разрешения этого конфликта.

---

## 2. Рассмотренные варианты

### Вариант А: «Доследовать плану» — удалить context_aggregator

- **Плюс**: 100% соответствие `06-04-PLAN.md` (5 → 1); −523 строки legacy-кода.
- **Минус**:
  - Ломает downstream consumer `mcp_server.py` (harness MCP-bridge) — подписка на `/perception/context_update` мертвеет.
  - Теряется агрегация 9 живых ROS2-источников (vision, pose, odom, joint_states, rosout, voice/*, command/*).
  - DialogueNode получает «слепой» perception — нет vision_context, нет is_moving, нет battery/health.
  - Это **регрессия функциональности**, не cleanup.

**Вердикт**: отклонён — перевешивает минус.

### Вариант Б: «Двух-нодовая архитектура» — оставить обе ноды ✅

- **Плюс**:
  - Сохраняет функциональность (9 ROS2-источников + UART).
  - Разделение concerns: hardware bridge vs ROS2 aggregator — каждая нода имеет одну ответственность.
  - Соответствует KISS: каждая нода ≤ 250 строк, читается за один проход.
  - Не ломает downstream consumer (mcp_server подписан на агрегатор).
  - Уже валидирован архитектурным ревью (`t_988e4052` §A.2.3 + mermaid §6.2 — явный двух-нодовый diagram).
- **Минус**:
  - 523 строки legacy-кода в `context_aggregator_node.py` живут дальше (но LLM-код уже вычищен в W10).
  - Расхождение с `06-04-PLAN.md` «5 → 1» (требует ретроактивного ADR — этот документ).
  - `/sensors/data` от `perception_bridge` пока не подписан `context_aggregator` (старая подписка `/device/snapshot` закомментирована — wiring diagram в launch-файле аспирационный, не актуальный).

**Вердикт**: принят — лучший trade-off для текущего состояния firmware (UART-драйвер ещё не написан).

### Вариант В: «Разделить context_aggregator» — вынести sensor/vision в отдельные модули

- **Плюс**: чище domain decomposition.
- **Минус**: overengineering — 523 строки делят между двумя нодами, появляется ещё один ROS2 hop, дополнительный failure mode. KISS-нарушение.

**Вердикт**: отклонён — преждевременное усложнение.

---

## 3. Решение

Принимается **Вариант Б**: perception-пайплайн состоит из **двух комплементарных ROS2-нод** с разными зонами ответственности:

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                          PERCEPTION PIPELINE                                │
└──────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────┐                ┌──────────────────────────────┐
│  perception_bridge      │                │  context_aggregator          │
│  ───────────────────    │                │  ──────────────────────      │
│  Type: UART bridge      │                │  Type: ROS2-topic aggregator │
│  Source: /dev/ttyAMA0   │                │  Source: 9 ROS2 topics       │
│       (sensor board)    │                │   (vision, pose, odom,       │
│  Output: /sensors/data  │                │    joint_states, rosout,     │
│       /perception/      │                │    voice/stt/result,         │
│       health            │                │    voice/dialogue/response,  │
│  LOC: 198               │                │    voice/command/intent,     │
│  Fail mode: stub (no    │                │    voice/command/feedback)   │
│       hardware)         │                │  Output: /perception/        │
│                         │                │       context_update         │
│                         │                │       (PerceptionEvent)      │
│                         │                │  LOC: 523 (post-W10)         │
│                         │                │  Fail mode: graceful         │
│                         │                │       degradation            │
└─────────────────────────┘                └──────────────────────────────┘
         │                                              │
         ▼                                              ▼
   /sensors/data (String/JSON)              /perception/context_update
   /perception/health (String/JSON)         (PerceptionEvent.msg)
         │                                              │
         ▼                                              ▼
   (no subscriber yet —                    mcp_server.py (harness MCP-bridge)
    pending Phase 7+                       → DialogCore perception context
    firmware readiness,                     → LLM input
    см. §4.2)

**Ключевые принципы:**

1. **Single Responsibility**: каждая нода имеет ОДНУ зону ответственности (hardware bridge vs ROS2 fusion).
2. **No LLM**: обе ноды — чистый data-flow без `import openai` / `from agents` (после W10).
3. **Graceful degradation**:
   - `perception_bridge` → stub-режим когда UART-порт недоступен (dev hosts).
   - `context_aggregator` → отсутствующие топики дают `null`-поля в `PerceptionEvent`, не крашат.
4. **Future extensibility**: `/sensors/data` от `perception_bridge` БУДЕТ подписан `context_aggregator` после того, как firmware-сенсор-борд будет готов и тип `DeviceSnapshot` будет собран (сейчас `/device/snapshot` subscription закомментирован в `context_aggregator_node.py:130-135`).
5. **Backward compatibility**: `mcp_server.py` продолжает получать `/perception/context_update` без изменений.

---

## 4. Последствия

### 4.1 Положительные

- Perception-пайплайн полностью функционален: UART + 9 ROS2-источников → harness MCP-bridge → LLM.
- Никаких регрессий относительно pre-W10 состояния (все источники сохранились).
- Launch-файлы (`internal_dialogue.launch.py`, `internal_dialogue_docker.launch.py`) корректно стартуют **обе** ноды.
- Тесты покрывают оба компонента: `test_perception_bridge.py` (484 LOC, 13 кейсов), `test_context_aggregator.py` (705 LOC, comprehensive).

### 4.2 Отрицательные / компромиссы

- **523 строки legacy в `context_aggregator_node.py`** живут дальше. Это допустимо: код LLM-free (после W10), хорошо покрыт тестами, имеет активного downstream consumer (mcp_server.py).
- **Расхождение с `06-04-PLAN.md` «5 → 1»**: ADR-0010 фиксирует это как осознанное решение. План был overly-ambitious — он не учитывал наличие downstream consumer'а `mcp_server.py`.
- **`/sensors/data` → context_aggregator wiring gap (KNOWN, NON-BLOCKING)**: текущее состояние:
  - `perception_bridge` публикует `String(JSON)` на `/sensors/data` (10 Hz) и `/perception/health` (1 Hz)
  - **Никто не подписан** на эти топики в текущей кодовой базе (grep подтверждает: только publisher и test-self-assertion)
  - Подписка `/device/snapshot` в `context_aggregator_node.py:130-135` закомментирована (требовала бы `robot_sensor_hub_msg/DeviceSnapshot`, который не входит в текущий build scope)
  - ASCII-диаграмма в `launch/internal_dialogue.launch.py:14-20` и `launch/internal_dialogue_docker.launch.py:14-20` рисует связь `perception_bridge → /sensors/data → context_aggregator`, что соответствует **целевому** состоянию Phase 7+, а не текущему
  - **Это known gap, не bug.** Root cause: firmware сенсор-борда ещё не готово → нет стабильного JSON-схемы данных → нельзя делать `context_aggregator` жёстко-зависимым от `/sensors/data`. В текущей архитектуре `perception_bridge` пишет в `/sensors/data` «в пустоту» — это безопасно (orphan publisher не ломает систему).
  - **Закрытие в Phase 7+**: после готовности firmware добавить `self._sensor_data_sub = self.create_subscription(String, '/sensors/data', self._on_sensor_data, 10)` в `context_aggregator_node.py.__init__()` + реализовать `on_sensor_data()` для добавления в `current_sensors` dict (battery/temperature/etc.).
  - **Не блокирует** `t_988e4052` (PASS-WITH-FIXES), потому что `mcp_server.py` подписан на `/perception/context_update` (агрегатор работает), и `/sensors/data` орфан-publisher не ломает запуск системы.

### 4.3 Что НЕ меняется этим ADR

- `health_monitor.py` (163 строки) — остаётся отдельной нодой, не агрегируется ни в одну из двух. Источник: `/rosout`. Потребитель: `/voice/sound/trigger`. Отдельная зона ответственности (system health → audio feedback).
- `perception_bridge` остаётся **stub-friendly**: при отсутствии `/dev/ttyAMA0` работает в no-op режиме (это намеренно для dev hosts и CI).

---

## 5. План реализации (Phase 6 close-out)

### 5.1 Этот ADR (W11a, `t_2ff2e9ad`)

- ✅ Зафиксировать решение в `docs/adr/0010-perception-bridge-and-aggregator.md` (этот документ).
- ✅ Создать `06-04-SUMMARY.md` (close-out doc для W10/W11/W12) — **уже существует** (commit `48312c7f`, 249 строк), PHASE-STATUS.md устарел.
- ✅ Обновить `PHASE-STATUS.md`: W11 row → `done (retroactive, broader scope, two-node architecture per ADR-0010)`; gap-table W11-related rows (G2, G8, G9) → `resolved (post-ADR-0010)`.
- ✅ Исправить ASCII-диаграмму в `launch/internal_dialogue.launch.py:14-20` и `launch/internal_dialogue_docker.launch.py:14-20` — отразить, что `/sensors/data → context_aggregator` это **целевое состояние** (Phase 7+), а не текущее.

### 5.2 Phase 7+ (deferred, вне scope этого ADR)

- Когда firmware сенсор-борда будет готово → включить `/device/snapshot` subscription в `context_aggregator_node.py:130-135` (или переименовать топик в `/sensors/data` и подписать на него).
- Возможный follow-up: выделить memory-tracking (`add_to_memory`, `get_memory_summary`) из `context_aggregator` в отдельный `perception_memory.py` если выяснится, что эта функциональность нужна другим нодам.

---

## 6. Ссылки

- `06-CONTEXT.md` §D-05 (Perception = UART-мост), §D-06 (Старые ноды удаляются)
- `06-04-PLAN.md` W11 (целевой план)
- `.planning/phases/06-harness-p0-finalization/06-ARCHITECT-REVIEW.md` §A.2.3, §A.2.14, mermaid §6.2
- `.planning/phases/06-harness-p0-finalization/PHASE-STATUS.md` строки 43 (W11), 77 (G8), 78 (G9), 79 (G10)
- `t_988e4052` (architect review, PASS-WITH-FIXES)
- `t_2ff2e9ad` (W11a, эта задача)
- Commits: `7552418a` (W10), `85cfd62e` (W11), `1200304c` (W12)
- Downstream consumer: `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py:144` (perception_sub on `/perception/context_update`)
- Launch wiring: `src/rob_box_perception/launch/internal_dialogue.launch.py:47-65` (запускает обе ноды)
