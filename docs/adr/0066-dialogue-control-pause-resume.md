# ADR-0066: `/dialogue/control` pause/resume — единственная связь оператора и личности

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-07 |
| Автор | architect по карточке `#1994` (operator-agent 06, шаг 6 из целевой §13) |
| Контекст | Закрывает шаг 6 целевой миграции `docs/architecture/target-operator-agent-and-dialogue.md` §13: добавить топик `/dialogue/control {pause\|resume}` на стороне `dialogue_node` без TTL, с подтверждением `/dialogue/control_ack`. Удалить `voice_input_mode` со всеми шестью значениями и quest-маршруты из `dialogue_node` (`_on_quest_stt`, `_publish_avatar_command_from_quest`, `_formalize_with_llm`, упоминания `/voice/stt/quest` в коде). Подписка на `/voice/stt/quest` уже удалена в рамках карточки #1990 (PR #2011). |
| Затрагивает | `src/rob_box_voice/rob_box_voice/dialogue_node.py` (новые sub/pub + удаление); `src/rob_box_voice/test/unit/node/test_dialogue_node.py` + `test_quest_stt_source.py` + `test_voice_presets_formalize.py` (тесты на удалённый код); связанные тесты `test_voice_input_mode_param.py` если остались; ADR-0027 §3.4 (отменяется в части quest-режимов `dialogue_node`); ADR-0028 §4.5 (отменяется, как уже зафиксировано в ADR-0051 §3.1) |
| Родители | ADR-0051 (агентский цикл оператора, инвариант 3 «одна связь агентов», инвариант 8 «пауза без авто-resume»), ADR-0064 (CC-бюджет), ADR-0018 (честность), `docs/architecture/target-operator-agent-and-dialogue.md` §7.3, §8, §10.3, §13 |
| Блок-зависимости | Зависит от шага 01 (сторож CC — **зелёный**, `scripts/lint/cc_budget.py` существует, baseline актуален) и шага 03 (`AgentCore` — ADR-0051 §2.4, код `agent_core` уже в репо через `DialogCore` переименование). Разблокирует шаг 07a (приоритет в `tts_node`) и e2e-сценарий паузы. |

> **TL;DR.** Агент оператора (`avatar_supervisor`) влияет на личность **ровно одним** способом: топиком `/dialogue/control` со значением `pause` или `resume`. На стороне `dialogue_node` это означает sub на `/dialogue/control`, pub на `/dialogue/control_ack`, переход `DSM → SILENCED` (уже существует) и **отсутствие** авто-возврата. Из `dialogue_node` удаляются `voice_input_mode`, `_on_quest_stt`, `_publish_avatar_command_from_quest`, `_formalize_with_llm` — личность больше не знает о Quest.

---

## 1. Контекст и бизнес-проблема

Сейчас в `dialogue_node.py` маршрутизация речи оператора проходит через параметр `voice_input_mode` с шестью значениями (`respeaker`, `quest_ttts`, `quest_llm_formalize`, `quest_stt`, `quest_command`, `off`). Параметр выставляет супервизор через `/dialogue_node/set_parameters` (ADR-0028 S5), что приводит к:

- пять маршрутов на одном микрофоне (`dialogue_node.py:2665` `_on_quest_stt`);
- гонки, зафиксированные в клиенте (`main.ts:590` — «voice_input_mode=respeaker и dialogue_node его игнорирует»);
- 6765-строчный `dialogue_node.py` со знанием про Quest, которого у личности быть не должно.

Целевая архитектура (`docs/architecture/target-operator-agent-and-dialogue.md` §2.3, §7.3, §8) требует **двух** изменений одновременно:

1. **Удалить** маршрутизатор (`voice_input_mode` + методы). Три из шести значений уже уехали в другие компоненты (`quest_ttts` → пайплайн грипа в супервизоре, `quest_stt` → не нужен, `quest_command` → `/avatar/stt/result` в агенте оператора). Оставшиеся `off` и `respeaker` — это и есть `pause`/`resume` через `/dialogue/control`.
2. **Добавить** единственный канал управления — топик `/dialogue/control` с двумя значениями и ack-канал `/dialogue/control_ack`. Без TTL: личность выходит из паузы только по явному `resume`. Напоминание голосом в шлем раз в 300 с и индикатор на мостике — **ответственность супервизора** (`avatar_supervisor`), а не `dialogue_node`.

### 1.1 Что уже сделано в смежных карточках

| Что сделано | Где | Карточка |
|---|---|---|
| Подписка на `/voice/stt/quest` удалена из `dialogue_node` и `stt_node` | `dialogue_node.py:594` (комментарий «#1990 — УДАЛЕНА»), `stt_node.py:313,348` | #1990 / PR #2011 (operator-agent 05) — уже merged в `develop` |
| Wake-роутер + `wake_words.yaml` (namespace per source) | `stt_node.py:348` | #1990 |
| Топики `/avatar/stt/result`, `/avatar/ptt/result`, `/avatar/voice_pipeline`, `/avatar/command` | `supervisor_node.py` | #1988 / #1989 (operator-agent 04a, 04b) |
| `avatar_supervisor` стал ТАРС (`AgentCore` + журнал) | `supervisor_node.py` | #1988 |
| `operator.control.txt` промпт ожидает инструменты паузы (сейчас говорит «инструменты появятся позже») | `src/rob_box_supervisor/prompts/skills/operator.control.txt:5` | #1988 — нужен апдейт после этого ADR |

Это означает, что наша карточка — **не пионер**: провод и каналы есть, нам нужно в `dialogue_node` подключить один sub/pub и снять мёртвый код.

### 1.2 Что осталось (эта карточка)

1. Sub `/dialogue/control` (String JSON) в `dialogue_node` — `pause` / `resume`.
2. Pub `/dialogue/control_ack` (String JSON) с полями `{state, since_ms, reason}`.
3. Удалить `voice_input_mode`, `_on_quest_stt`, `_publish_avatar_command_from_quest`, `_formalize_with_llm`, связанные параметры и кэш.
4. Удалить тесты на удалённый код.
5. Обновить `operator.control.txt`: снять оговорку «инструменты появятся позже» — теперь у супервизора есть инструменты паузы, реализованные поверх `/dialogue/control`.

---

## 2. Принятое решение

### 2.1 Wire-контракт

| Топик | Тип | Publisher | Subscriber | QoS | Payload |
|---|---|---|---|---|---|
| `/dialogue/control` | `std_msgs/String` (JSON внутри) | `avatar_supervisor` | `dialogue_node` | RELIABLE, KEEP_LAST depth=10 | `{"action": "pause"\|"resume", "reason": str, "ts_s": float}` |
| `/dialogue/control_ack` | `std_msgs/String` (JSON внутри) | `dialogue_node` | `avatar_supervisor` | RELIABLE, KEEP_LAST depth=10 | `{"state": "idle"\|"paused"\|"dialogue"\|"listening", "since_ms": int, "ts_s": float}` |

- `state` — текущее состояние DSM (`DialogueStateKind.name`). Поле `paused` совпадает с `SILENCED` (см. §2.2 — алиасы).
- `since_ms` — `int(time.monotonic()*1000)` момента входа в текущее состояние. Агенту оператора нужно для тика «напоминание раз в 300 с».
- `reason` — проброс от супервизора (например, `"operator"`); логируется, но не влияет на FSM.
- **TTL отсутствует** на стороне `dialogue_node`. Если `pause` пришёл без последующего `resume`, личность молчит до явного `resume`. Это инвариант 8 (ADR-0051 §5).

#### 2.1.1 Совместимость с `ttl_s`

В целевой §9.4 поле `ttl_s` помечено как часть контракта `{action, reason, ttl_s}`. **В этой карточке `ttl_s` не используется** — личность не может сама себя разбудить. Если супервизор захочет ограничить паузу, он сам шлёт `resume` через заданное время (например, через `TaskScheduler`). Это согласуется с §8 целевой: «Срока у паузы нет».

### 2.2 Маппинг на `DialogueStateMachine`

Текущая FSM (`src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py`) уже умеет:

| Событие | Из состояния | В состояние |
|---|---|---|
| `SILENCE_COMMAND` | любое | `SILENCED` |
| `UNSILENCE` | `SILENCED` | `IDLE` |
| `TIMEOUT` | `SILENCED` | `IDLE` (TTL — **не вызывается** из `dialogue_node`, см. §2.3) |

Решение: ввести **новые события** `OPERATOR_PAUSE` и `OPERATOR_RESUME`, которые:

- `OPERATOR_PAUSE` (из любого состояния) → `SILENCED` (аналог `SILENCE_COMMAND`, но без прохождения через `on_user_input`, т.к. команда приходит не текстом).
- `OPERATOR_RESUME` (из `SILENCED`) → `IDLE`. Из **любого другого** — no-op (защита от гонок, когда `resume` приходит, но `pause` уже не активен; ack всё равно отправляется).

Почему не переиспользовать `SILENCE_COMMAND` / `UNSILENCE`:

- `SILENCE_COMMAND` завязан на текстовое событие от `on_user_input`. Маршрут оператора — не текст, это структурированный JSON. Смешивать каналы = снова реестр из шести значений.
- `UNSILENCE` допускает также текстовое «робот, говори» (см. `dialogue_node.py:2851`, `is_unsilence_command`). Это публичная команда для людей рядом с роботом. Не надо делать её единственным путём выхода из `pause` оператора — иначе любой прохожий может снять паузу.

Новые события — чистая декомпозиция: оператор управляет через топик, прохожие управляют голосом, FSM трактует их одинаково через переход в `SILENCED` ↔ `IDLE`. Никакой новой state — только новые event-имена.

### 2.3 Таймаут тишины не вызывается

В `dialogue_node._on_inactivity_check` (`dialogue_node.py:6710`) вызывается **только** `check_inactivity_timeout` (LISTENING → IDLE). Метод `check_silence_timeout` (SILENCED → IDLE по TTL) в `dialogue_node` не вызывается нигде — подтверждено `search_files` по `src/`. Это автоматически удовлетворяет инвариант 8 «пауза без авто-resume». **Поведение не меняется**, дополнительный код не нужен.

### 2.4 Удаляемый код

В `dialogue_node.py` под нож идёт:

| Сущность | Строки | Что делаем |
|---|---|---|
| `self.declare_parameter("voice_input_mode", "respeaker")` | `1048` | Удалить |
| `self._voice_input_mode: str = "respeaker"` | `1049` | Удалить |
| Ветка в `parameters_callback` (`param.name == "voice_input_mode"`) | `1102-1106` | Удалить |
| Комментарии про `voice_input_mode` в docstrings | `329, 519, 1039-1058, 1077-1090, 2071, 2134, 2665-2747, 2795-2818, 598-600` | Удалить/обновить |
| Метод `_on_quest_stt` | `2665-2745` | Удалить целиком |
| Метод `_publish_avatar_command_from_quest` | `2747-2770` | Удалить целиком |
| Метод `_formalize_with_llm` | `2378+` (async coroutine) | Удалить целиком |
| Поля `_quest_session_id`, `_avatar_command_pub` (если только для `_publish_avatar_command_from_quest`) | проверить | Удалить, если только для этого |
| Тесты: `test_quest_stt_source.py`, `test_voice_presets_formalize.py`, `test_voice_input_mode_param.py` | в `test/unit/node/` | Удалить целиком |

Подписка `create_subscription(String, "/voice/stt/quest", self._on_quest_stt, ...)` уже удалена в #1990 (`dialogue_node.py:594` комментарий), трогать не надо.

Pub `/avatar/command` (`self._avatar_command_pub`) **сохраняется** — он используется супервизором (через свой собственный publisher) и не связан с `_publish_avatar_command_from_quest`. Перепроверить при имплементации.

### 2.5 Идемпотентность и edge-cases

| Сценарий | Поведение |
|---|---|
| `pause` пришёл, FSM уже в `SILENCED` | No-op, ack `{state:"paused", since_ms:<исходный>}` — обновлять `since_ms` не надо, чтобы напоминание в супервизоре не сбивалось |
| `pause` пришёл, FSM в `DIALOGUE` | FSM → `SILENCED`, ack `{state:"paused", since_ms:<новый>}`. Текущий LLM-тур продолжается (не отменяется), но следующий user input будет проигнорирован (текущее поведение `_on_stt:2849`). История не теряется |
| `resume` пришёл, FSM в `IDLE`/`LISTENING`/`DIALOGUE` | No-op, ack `{state:<текущий>, since_ms:<since текущего состояния>}`. Защита от гонок |
| `resume` пришёл, FSM в `SILENCED` | FSM → `IDLE`, ack `{state:"idle", since_ms:<новый>}`, `_publish_state()` |
| Невалидный JSON или отсутствующий `action` | WARNING в лог, ack НЕ отправляется (или отправляется `{state:"<current>", error:"invalid_action"}` — выбрать на этапе имплементации, по умолчанию ack НЕ шлём) |
| `pause` без `reason` | `reason=""`, не ошибка |

### 2.6 Что НЕ делает `pause`

Согласно §8 целевой:

- не глушит ReSpeaker — `voice_input_mode` больше нет, этот гейт не существует;
- не меняет маршрут аудио — `stt_node` уже сам решает;
- не чистит историю — `MemoryStore` не трогаем;
- не отменяет текущий TTS/музыку — это ответственность супервизора/планировщика (шаг 7).

Это **ровно** переход `DSM → SILENCED`, который уже реализован в FSM.

---

## 3. Что закрывается и отменяется

### 3.1 ADR-0027 §3.4 — закрывается в части quest-режимов `dialogue_node`

Маршрутизация `voice_input_mode` в `dialogue_node` отменяется. Wire-режим из `meta-quest-api.md §5` (см. `quest_node.py:475`, `ws_server.py:1395`) больше **не маппится** в параметр ноды — он маппится в выбор пайплайна (`/avatar/ptt/result` vs `/avatar/voice_pipeline`), что уже сделано в #1990.

### 3.2 ADR-0028 §4.5 — отменяется (уже зафиксировано в ADR-0051 §3.1)

Запись `dialogue_node` через `/dialogue_node/set_parameters` отменяется. Этот ADR закрывает отмену **де-факто**: `voice_input_mode` параметра больше нет, единственный канал — топик.

---

## 4. Последствия

### 4.1 Положительные

- `dialogue_node.py` теряет ~80–120 LOC (методы, параметр, кэш, ветки) и одно знание о Quest.
- Личность больше не подвержена гонке `voice_input_mode` (issue #1252 в миниатюре — параметр и текст).
- Канал оператора становится **публичным контрактом** — добавить второй инструмент (`/dialogue/control{mode:"mute_tts"}` например) = тривиально расширить JSON.
- Инвариант 3 (ADR-0051 §5) формально выполняется: единственная связь агентов — топик.

### 4.2 Отрицательные и риски

- `operator.control.txt` нужно обновить: убрать «инструменты появятся позже» и описать фактическое поведение (`dialogue_pause` / `dialogue_resume` шлют в `/dialogue/control`). Это **в этой же карточке** — иначе ADR-0051 §5 «честный FAIL» нарушается (модель будет говорить «не умею», хотя умеет).
- `supervisor_node.py` всё ещё содержит код `_voice_input_mode_before_swap`, `_set_dialogue_param("voice_input_mode", ...)`, `_capture_current_voice_mode()` (`supervisor_node.py:347, 352, 433, 443, 452, 835-886, 1544-1556`). Это **сломанный код** после удаления параметра: попытка `set_parameters` на несуществующее поле упадёт в rclpy runtime. **Эту работу делает developer-карточка**, не архитектор. Зафиксировать как **явный follow-up** в §7.
- Тесты `test_voice_presets_formalize.py`, `test_quest_stt_source.py` завязаны на `_on_quest_stt`/`_formalize_with_llm`. Их удаление сломает покрытие AV-28 (формализация пресета). Но AV-28 **сам по себе уехал** в пайплайн грипа супервизора (целевая §7.5) — тесты больше не нужны в `dialogue_node`. Подтверждение через grep: после #2011 (operator-agent 05) формализация пресета идёт в `grip_pipeline.py`, а не в `dialogue_node._formalize_with_llm`.

### 4.3 Что НЕ меняется

- Поведение личности для обычных пользователей (ReSpeaker, wake-word) — байт в байт.
- `DialogueStateMachine`, `MemoryStore`, `AgentCore` — не трогаем.
- Все остальные топики `/voice/*`, `/avatar/*`, `/mcp/*` — не трогаем.

---

## 5. Инварианты

Правила, нарушение которых означает откат ADR:

1. **Никаких `set_parameters` в `dialogue_node` из супервизора.** Единственный канал — `/dialogue/control`. Тест: grep `_set_dialogue_param.*dialogue_node` в `supervisor_node.py` после имплементации developer-карточкой → пусто.
2. **Никакого TTL на стороне `dialogue_node`.** `check_silence_timeout` не вызывается. Тест: unit-тест «pause без resume на 10 минут → FSM всё ещё SILENCED».
3. **`/dialogue/control_ack` синхронен с изменением FSM.** Изменение состояния → публикация ack **до** возврата из callback. Тест: подписчик на `/dialogue/control_ack` получает ack **раньше**, чем следующий вызов `_publish_state()`.
4. **История не теряется при pause/resume.** `MemoryStore` не модифицируется, `_active_dialogue` сохраняется. Тест: e2e «до pause → после resume → контекст цел».
5. **Текущий LLM-тур не отменяется pause.** Если LLM уже в `_run_turn`, он доезжает до `DIALOGUE_END`; только следующий user_input будет проигнорирован (текущее поведение SILENCED-гейта в `_on_stt:2849`).

---

## 6. План реализации (для developer-карточки)

Шаги строго последовательны; каждый — отдельный коммит в той же ветке.

### Шаг 6.1 — Подписка и публикация

В `dialogue_node.DialogueNode.__init__` после блока про `create_subscription(String, "/voice/tts/provider_state", ...)` (около строки 658) добавить:

```python
# ADR-0054 — единственная связь с агентом оператора: /dialogue/control.
# Sub: pause/resume от avatar_supervisor (String JSON).
# Pub: ack с текущим состоянием DSM после применения команды.
self._dialogue_control_pub = self.create_publisher(
    String, "/dialogue/control_ack", qos_r)
self.create_subscription(
    String, "/dialogue/control", self._on_dialogue_control,
    qos_r, callback_group=cbg)
self._paused_at_ms: Optional[int] = None  # monotonic() * 1000 в момент pause
self._pause_reason: str = ""
```

### Шаг 6.2 — Метод `_on_dialogue_control`

Новый метод (после `_on_command_feedback` или в конце класса):

```python
def _on_dialogue_control(self, msg: String) -> None:
    """ADR-0066 — pause/resume от avatar_supervisor.

    JSON: {"action": "pause"|"resume", "reason": str, "ts_s": float}.
    Любое другое значение action — warning + no-op (ack НЕ шлём,
    см. §2.5 «невалидный JSON»).
    """
    try:
        payload = json.loads(msg.data or "{}")
        action = str(payload.get("action") or "").strip().lower()
        reason = str(payload.get("reason") or "")
    except (json.JSONDecodeError, TypeError):
        self.get_logger().warning(
            f"⚠️ [ADR-0066] /dialogue/control: invalid JSON {msg.data!r}")
        return
    if action == "pause":
        self._apply_operator_pause(reason)
    elif action == "resume":
        self._apply_operator_resume()
    else:
        self.get_logger().warning(
            f"⚠️ [ADR-0066] /dialogue/control: unknown action {action!r}")
        return
    self._publish_control_ack()

def _apply_operator_pause(self, reason: str) -> None:
    state = self._dsm.current_state
    if state == DialogueStateKind.SILENCED:
        # идемпотентно: since_ms не обновляем (см. §2.5)
        return
    self._dsm.on_event(DialogueEvent.SILENCE_COMMAND)
    self._paused_at_ms = int(time.monotonic() * 1000)
    self._pause_reason = reason
    self._publish_state()
    self.get_logger().info(
        f"⏸️ [ADR-0066] pause reason={reason!r} (was {state.name})")

def _apply_operator_resume(self) -> None:
    state = self._dsm.current_state
    if state != DialogueStateKind.SILENCED:
        # no-op + ack с текущим состоянием (защита от гонок, §2.5)
        return
    self._dsm.on_event(DialogueEvent.UNSILENCE)
    self._paused_at_ms = None
    self._pause_reason = ""
    self._publish_state()
    self.get_logger().info("▶️ [ADR-0066] resume")

def _publish_control_ack(self) -> None:
    state = self._dsm.current_state
    payload = {
        "state": "paused" if state == DialogueStateKind.SILENCED
                 else state.name.lower(),
        "since_ms": self._paused_at_ms if state == DialogueStateKind.SILENCED
                    else int(time.monotonic() * 1000),
        "ts_s": time.time(),
        "reason": self._pause_reason if state == DialogueStateKind.SILENCED
                  else "",
    }
    out = String(); out.data = json.dumps(payload, ensure_ascii=False)
    self._dialogue_control_pub.publish(out)
```

CC-бюджет: `_on_dialogue_control` ≤ 7, `_apply_operator_pause` ≤ 4, `_apply_operator_resume` ≤ 4, `_publish_control_ack` ≤ 4. Все укладываются в лимит 15 без `update-baseline`.

### Шаг 6.3 — Удаление `voice_input_mode` и quest-маршрутов

1. Удалить `self.declare_parameter("voice_input_mode", "respeaker")` и `self._voice_input_mode` (`dialogue_node.py:1048-1049`).
2. Удалить ветку `if param.name == "voice_input_mode":` в `parameters_callback` (`dialogue_node.py:1102-1106`).
3. Удалить методы `_on_quest_stt` (`2665-2745`), `_publish_avatar_command_from_quest` (`2747-2770`), `_formalize_with_llm` (`2378+`).
4. Если `_quest_session_id` использовался только в `_publish_avatar_command_from_quest` — удалить.
5. Удалить мёртвые docstring-комментарии: `598-600`, `1039-1058`, `1077-1090`, `2665-2688`, `2738-2740`, `2794-2811`.
6. Удалить файлы тестов:
   - `src/rob_box_voice/test/unit/node/test_quest_stt_source.py`
   - `src/rob_box_voice/test/unit/node/test_voice_presets_formalize.py`
   - `src/rob_box_voice/test/unit/node/test_voice_input_mode_param.py` (если остался)

Проверить `git grep "voice_input_mode" src/rob_box_voice` → **пусто** (DoD §7.3 карточки).

### Шаг 6.4 — Обновить `operator.control.txt`

Снять оговорку «инструменты появятся позже». Новая версия:

```
Срез оператора: УПРАВЛЕНИЕ (пауза личности, floor/арбитраж).

Этот срез отвечает за контроль над личностью и голосовым каналом
(арбитраж floor).

Доступные действия:
- `dialogue_pause(reason="...")` — публикует в /dialogue/control
  {"action":"pause",...}; личность переходит в SILENCED и молчит до
  явного resume. Без TTL.
- `dialogue_resume()` — публикует в /dialogue/control
  {"action":"resume"}; личность выходит из SILENCED.
- Арбитраж floor — отдельная нода /avatar_arbiter/*; инструменты
  появятся в следующих шагах.

Если оператор просит «поставь личность на паузу» или «пусть робот
помолчит» — вызови dialogue_pause(reason="operator").
Если просит «продолжай» или «говори» — вызови dialogue_resume().
Если просит «дай мне floor» или «кто сейчас говорит» — сообщи, что
арбитраж floor переезжает на отдельную ноду.

НЕ выдумывай инструменты для этих действий — если инструмента нет,
скажи об этом прямо.
```

(Этот текст — рекомендация; точную формулировку утверждает супервизор-разработчик в его карточке. Здесь архитектор фиксирует только **факт** обновления.)

### Шаг 6.5 — Юнит-тесты

Новые тесты в `test_dialogue_node.py` (или отдельный файл `test_dialogue_control.py`):

| Сценарий | Ожидание |
|---|---|
| `pause` → FSM=SILENCED → ack `{state:"paused", since_ms:>0, reason:"<reason>"}` | ✓ |
| `pause` повторный → ack `{state:"paused", since_ms:<исходный>}` (не обновляется) | ✓ |
| `resume` → FSM=IDLE → ack `{state:"idle", since_ms:<новый>, reason:""}` | ✓ |
| `resume` без предшествующего `pause` → ack `{state:"<current>"}` (no-op) | ✓ |
| `pause` в DIALOGUE → LLM-тур продолжается, FSM → SILENCED | ✓ |
| `pause` невалидный JSON → WARNING, FSM не меняется, ack не шлётся | ✓ |
| `pause` неизвестный action → WARNING, FSM не меняется, ack не шлётся | ✓ |
| После 5 минут pause без resume → FSM всё ещё SILENCED (инвариант 8) | ✓ (time-mock) |
| `_publish_state` вызывается **до** `_publish_control_ack` (инвариант 3) | ✓ (mock на publisher, проверка порядка вызовов) |

### Шаг 6.6 — e2e контракт для e2e-process

Тест на живом роботе (делает e2e-process):

```yaml
## e2e
scenario: pause_resume_no_TTL
voice_text: "Робот, поставь личность на паузу"
voice_file: .github/e2e/voice_commands/rabot_pause_lichnost.ogg
volume: 150
record_seconds: 90
preconditions:
  - dialogue_node запущен, FSM=IDLE
  - avatar_supervisor запущен, TARS-агент активен
assertions:
  - ros2 topic echo /dialogue/control_ack --once → {"state":"paused",...}
  - ros2 topic echo /voice/dialogue/state --once → содержит "SILENCED"
  - сказать «робот, расскажи анекдот» в ReSpeaker → личность молчит (no TTS)
  - через 60 с (без resume) — личность всё ещё молчит
followup:
  voice_text: "Робот, продолжай"
  expected: ros2 topic echo /dialogue/control_ack → {"state":"idle"}
            сказать «робот, расскажи анекдот» → личность отвечает
```

Этот контракт будет запущен e2e-process **после** merge-gate по этому PR.

### Шаг 6.7 — Удаление в `supervisor_node.py` (ОТДЕЛЬНАЯ карточка)

Это **не входит** в эту карточку. Зафиксировать как follow-up:

> Карточка `[operator-agent 06-followup]`: убрать `_set_dialogue_param("voice_input_mode", ...)`,
> `_voice_input_mode_before_swap`, `_capture_current_voice_mode` из `supervisor_node.py` —
> после merge этой карточки параметр `voice_input_mode` больше не существует, любая попытка
> `set_parameters` упадёт в rclpy runtime. Assignee: backend (или developer с доступом к
> supervisor_node). Блок-зависимость: `parents=[t_d058dc6f]`.

Архитектор не имплементирует supervisor_node. Это явно за рамками роли.

---

## 7. Trade-offs / открытые вопросы

### 7.1 Почему не переиспользовать `SILENCE_COMMAND` / `UNSILENCE`

Альтернатива: расширить существующие события, чтобы они принимали и текст, и JSON. **Отклонено**: смешивание каналов = новая точка гонки. Чистая декомпозиция (новые события только для топика) проще и явнее.

### 7.2 Почему `paused`, а не `silenced` в поле `state` ack

`SILENCED` — внутреннее имя FSM. Внешний контракт использует **понятное** оператору слово «paused». Маппинг однозначный, делается в `_publish_control_ack`. Люди-тестеры сразу читают поле.

### 7.3 Почему не хранить `ttl_s` в FSM

Альтернатива: `pause(ttl_s=N)` → FSM сама выходит через N секунд. **Отклонено**: ADR-0051 §5 инвариант 8 явно запрещает авто-resume. Если оператор захочет ограниченную паузу, он делает это в `avatar_supervisor` (например, через `TaskScheduler`), а не в личности.

### 7.4 Открытые вопросы

- Точная формулировка инструментов `dialogue_pause` / `dialogue_resume` в `mcp_server` — отдельная карточка (шаг 6 + mcp интеграция).
- Напоминание голосом в шлем раз в 300 с — реализуется супервизором **отдельно** (карточка `[operator-agent 06-reminder]`). Здесь только контракт топика.
- Индикатор на мостике — `docs/architecture/target-operator-agent-and-dialogue.md` §13 явно out of scope для шага 6.

---

## 8. Связанные документы

- `docs/architecture/target-operator-agent-and-dialogue.md` §7.3, §8, §9.4, §10.3, §11 (инвариант 3, 8), §13 шаг 6
- `docs/adr/0051-supervisor-operator-agent-arbiter-split.md` §2.10, §3.1, §5 инвариант 3, 8
- `docs/adr/0028-avatar-supervisor.md` §4.5 (отменяется этим ADR)
- `docs/adr/0027-dialogue-quest-mode.md` §3.4 (отменяется в части quest-режимов `dialogue_node`)
- `docs/adr/0021-dialogue-node-decomposition-discipline.md` (CC-бюджет — лимит 15)
- `src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py` (`DialogueEvent`, `DialogueStateKind`)
- `src/rob_box_voice/rob_box_voice/dialogue_node.py` (целевой файл — 6765 LOC, после выпила ~6645)
- Карточка `#1990` (operator-agent 05, wake router — merged в `develop` через PR #2011)
- Карточка `#1988` (operator-agent 04a — supervisor → ТАРС)
- Карточка `#1989` (operator-agent 04b — grip pipeline)