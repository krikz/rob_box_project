# Scheduler в `dialogue_node` целиком — сегменты, MERGE/REPLACE, правка на лету

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.
> **Сначала прочитай `docs/plans/2026-08-28-scheduler-segments-merge-BRIEF.md`** —
> там окружение, рабочие команды pytest и красная базовая линия.

**Goal:** Закрыть блокер П1 (`SCHEDULER_DESIGN.md:1676`) и сценарий «комар + енот».
Новая команда во время речи **не глушит робота**: текущий сегмент доигрывает до
естественной границы, ещё не начатые — переписываются.

**Architecture:** Сегменты уже существуют де-факто — очередь `speak_text` в FIFO
канала VOICE (см. §0). Добавляем к `SchedulerTask` группировку (`group_id`/`seg_idx`),
метод `TaskScheduler.update()` с инвариантом «не трогать RUNNING», модуль правил
`quick_decide` (без второй LLM — ревизия v5), блок `[SEGMENT PLAN]` в контекст LLM
и MCP-тул `task_delta`. Врезка в `dialogue_node` — под флагом `barge_in_policy`,
дефолт `replace` = сегодняшнее поведение.

**Tech Stack:** Python 3.10+, asyncio, ROS 2 Humble (rclpy), pytest, YAML-конфиги.

**Design:** `docs/design/SCHEDULER_DESIGN.md` (§2 сегменты, §4 вердикты, §4.7 ревизия v5, §6.5 FROZEN, §7 feedback)
**Предшественник:** `docs/design/W7_INTEGRATION_PLAN.md` (W7a–c приземлены, W7d открыт)
**Спек-видение:** `docs/design/dialogue-mode-spec-2026-08-28.md` §2.5 + §5
**Аудит-источник:** `docs/plans/2026-08-28-dialogue-mode-gap-audit-plan.md` (фаза Ф2)

## ⚠️ Честность (ADR-0018)

Анализ кода — **статический**, по `develop` (head `610bf10e`); все ссылки
`файл:строка` даны на этот коммит. Ни одна строка плана в коде не реализована.

Что **прогонялось** при подготовке (raw — в BRIEF §2–3):

- `test/test_task_scheduler.py test/test_tool_executor.py test/test_speculative_pre_gen.py test/test_segment_estimator.py test/test_scheduler_event_bus.py` → **80 passed**;
- `test/unit` (без 2 каталогов с collection-ошибками) → **50 failed, 1408 passed, 11 skipped** — красная базовая линия ДО правок.

E2E (фаза S8) на воркстейшене **не прогонялся и прогнан быть не может** — нужен стенд.

---

## 0. Ключевой вывод разведки: сегменты уже есть де-факто

Это меняет оценку работы. **Не нужно строить сегментную машину с нуля.**

`master_prompt_compact.txt:219-222` уже требует от LLM: «for multi-part content
(songs, stories, poems, rap) — **MULTIPLE `speak_text`, one per item**;
per stanza, rap ≥ 6». То есть песня уже приходит как **N отдельных вызовов
`speak_text`**, а `SchedulerToolExecutor` (`scheduler/tool_executor.py:47`)
кладёт каждый в FIFO-очередь канала VOICE.

Соответствие с §2.1 дизайна:

| Дизайн §2.1 | Что уже есть в коде |
|---|---|
| `Segment` | `SchedulerTask` (`task_scheduler.py:143`) |
| `Segment.status = ACTIVE` | `TaskStatus.RUNNING` (`:105`) |
| `Segment.status = PENDING` | `TaskStatus.QUEUED` / `SCHEDULED` (`:103-104`) |
| `Segment.kind` | `SchedulerTask.channel` (`ChannelKind.VOICE/MUSIC/ANIM`) |
| `Segment.payload` | `SchedulerTask.args` |
| «удалить PENDING» | `_Channel.remove(task_id)` (`:359`) — уже rebuild-from-snapshot |
| `PENDING_LIVE` / `PENDING_FROZEN` граница | `SpeculativePreGenerator.build_plan → PreGenPlan.boundary_idx` (`pre_gen.py:97-115`) |
| `eta_ms` | `SegmentEstimate` (`estimator.py:54`) + `set_eta_provider` (`task_scheduler.py:716`) |
| task-группировка сегментов | ❌ **нет** — `SchedulerTask` не знает, к какой задаче принадлежит |

**Чего реально не хватает — четыре вещи, не двадцать:**

1. **Группировка**: у `SchedulerTask` нет `group_id`/`seg_idx` → нельзя сказать
   «перепиши сегменты 3 и 4 задачи t_001».
2. **`TaskScheduler.update(group_id, delta)`** — только `submit`/`cancel`.
3. **Классификация ввода** (`quick_decide`) — вердикта нет ни одного.
4. **Врезка в `dialogue_node`**: `:1809` безусловно зовёт `_cancel_run`, который
   публикует `STOP` в `/voice/tts/control` (`:4555-4566`).

### 0.1 Второй важный вывод: гонки турна почти нет

`SchedulerToolExecutor.execute` для `speak_text` возвращает
`{"status": "queued", "task_id": ...}` **немедленно** (`tool_executor.py:13-16`).
Значит LLM-ход заканчивается быстро, а песня продолжает играть из очереди
канала VOICE **уже без активного турна**.

Следствие: в типичном «комар + енот» на момент второй фразы **турн не в полёте**,
и весь вред наносит именно `_cancel_run` → `STOP` в TTS. Убрать этот `STOP` —
и половина сценария начинает работать без всякой машины вердиктов.

**Поэтому план начинается с самого дешёвого шага, дающего наибольший эффект (S1),
а сегментная машина строится следом.**

### 0.2 Риск, который надо решить до старта

Музыкальные стартеры (`execute_music_code` / `set_vibe_preset` / `load_track`)
**намеренно обходят** планировщик и исполняются блокирующе
(`tool_executor.py:49-52`, комментарий «party regression, live 19.08»: LLM должна
увидеть реальный результат, иначе DJ-сет разваливается).

Значит бит песни — вне сегментной модели, в ней только вокал (`speak_text`).
Для «комар + енот» этого достаточно (переписываются куплеты, бит играет). Но
**инвариант «MERGE не трогает ACTIVE» распространяется только на канал VOICE**.
Это надо зафиксировать явно, а не обнаружить на e2e.

---

## 1. Порядок фаз

```
S1  снять безусловный STOP (флаг)        ← самый дешёвый, самый большой эффект
S2  group_id / seg_idx у SchedulerTask
S3  TaskScheduler.update(group_id, delta)
S4  quick_decide — правила (level 1)
S5  [SEGMENT PLAN] в контекст LLM
S6  MCP-тул task_delta
S7  очередь pending user-messages в dialogue_node
S8  e2e «комар + енот» + переписать 00_barge_in   ← ЗЕЛЁНЫЙ = цель достигнута
─────────────────────────────────────────────── (можно останавливаться здесь)
S9  FROZEN/LIVE через build_plan
S10 auto-trigger LLM-после-MERGE (§4.5)
S11 W7d — снять костыли
S12 метрики + документация
```

S1–S8 — минимальный путь до рабочего сценария. S9–S12 — доводка.

Каждый шаг = один PR (ADR-0013). Всё под флагом из S1, дефолт — старое поведение.

---

## Фаза S1 — Флаг `barge_in_policy` и снятие безусловного STOP

> Самая рискованная правка во всём плане делается первой и отдельно, чтобы
> её можно было откатить одной строкой конфига.

### Task 1.1: Параметр `barge_in_policy`

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/dialogue_node.py` (рядом с `:708 declare_parameter("dialogue_timeout", ...)`)
- Modify: `docker/vision/config/voice_assistant/dialogue_node.yaml`
- Test: `src/rob_box_voice/test/unit/node/test_dialogue_node.py`

**Step 1: Тест** — нода объявляет `barge_in_policy` со значением по умолчанию
`"replace"`; допустимые значения `{"replace", "classify"}`; мусорное значение
логирует warning и падает в `"replace"`.

**Step 2: Реализовать** — `declare_parameter("barge_in_policy", "replace")`,
валидация в `__init__`, сохранение в `self._barge_in_policy`.

**Step 3:** `cd src/rob_box_voice && PYTHONUTF8=1 python -m pytest test/unit/node -q` → без НОВЫХ падений (база: 7 упавших, см. BRIEF). Commit:
```bash
git commit -m "feat(voice): barge_in_policy parameter (replace|classify)"
```

### Task 1.2: Разделить `_cancel_run` на «отменить турн» и «заглушить TTS»

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/dialogue_node.py:4555-4566`
- Test: `src/rob_box_voice/test/unit/node/test_dialogue_node.py`

Сейчас `_cancel_run` делает четыре разных вещи разом:
```python
self._run_cancelled = True
task.cancel()                                    # (1) отменить LLM-турн
self._tts_control_pub.publish("STOP")            # (2) заглушить TTS
self._effects.release_all_tts()                  # (3) отпустить awaiter'ы
self._effects.clear_sound_event()                # (4) сбросить sound-событие
```

**Step 1: Тест** — новый `_cancel_run(reason, *, stop_tts: bool)`:
- `stop_tts=True` → публикует `STOP` (текущее поведение, дефолт для совместимости);
- `stop_tts=False` → **не** публикует `STOP`, но всё ещё отменяет турн и
  отпускает awaiter'ы (иначе `speak_helpers._tts_events` залипнут навсегда —
  см. `core/speak_helpers.py:320 release_all_tts`).

⚠️ Тест обязан проверить именно «awaiter'ы отпущены, STOP не отправлен» —
это та самая ловушка, из-за которой наивная правка вешает диалог.

**Step 2: Реализовать** — параметр `stop_tts`, все существующие вызовы
(`:1789`, `:4448`, `:4469`) передают `stop_tts=True` явно.

**Step 3:** Commit:
```bash
git commit -m "refactor(voice): split _cancel_run into turn-cancel and tts-stop"
```

### Task 1.3: При `policy=classify` не глушить TTS на новом вводе

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/dialogue_node.py:1809`
- Test: `src/rob_box_voice/test/unit/node/test_barge_in_policy.py` (NEW; образец — `test/unit/node/test_speech_backlog_accumulator.py`). ⚠️ НЕ `test/test_dialogue_shell.py` — он требует настоящий ROS 2 и локально не собирается.

**Step 1: Тест** — при `barge_in_policy="classify"` приход второй STT-фразы
во время активного VOICE-канала **не** публикует `STOP` в `/voice/tts/control`;
при `"replace"` — публикует (регресс старого поведения).

**Step 2: Реализовать:**
```python
stop_tts = self._barge_in_policy == "replace"
self._cancel_run("new STT input", stop_tts=stop_tts)
```

**Step 3:** `cd src/rob_box_voice && PYTHONUTF8=1 python -m pytest test/unit/node -q` → без НОВЫХ падений. Commit.

**Acceptance S1:** с `barge_in_policy=classify` робот договаривает начатый
`speak_text`, когда пользователь произносит новую фразу. Ещё **не** правит план —
просто не замолкает.

---

## Фаза S2 — Группировка сегментов

### Task 2.1: `group_id` / `seg_idx` в `SchedulerTask`

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/scheduler/task_scheduler.py:143-181`
- Test: `src/rob_box_voice/test/test_task_scheduler.py`

**Step 1: Тест** — `SchedulerTask(group_id="t_001", seg_idx=2)`; оба поля
опциональны (`None` = одиночная задача, обратная совместимость); попадают в
`snapshot()`.

**Step 2: Реализовать** — два новых поля с дефолтом `None`, добавить в
`snapshot()` (`:182-201`). `TaskScheduler.submit` (`:535`) прокидывает их.

**Step 3:** Commit:
```bash
git commit -m "feat(scheduler): group_id/seg_idx on SchedulerTask"
```

### Task 2.2: `TaskScheduler.segments(group_id)`

**Files:**
- Modify: `scheduler/task_scheduler.py`
- Test: `src/rob_box_voice/test/test_task_scheduler.py`

**Step 1: Тест** — возвращает список сегментов группы в порядке `seg_idx` со
статусами; ровно один может быть `RUNNING`; завершённые остаются в списке до
завершения группы.

**Step 2: Реализовать** — реестр `self._groups: Dict[str, List[str]]`,
заполняется в `submit`, чистится в терминальных переходах.

**Step 3:** Commit.

### Task 2.3: `SchedulerToolExecutor` проставляет группу

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/scheduler/tool_executor.py`
- Test: `src/rob_box_voice/test/test_tool_executor.py`

**Step 1: Тест** — несколько `speak_text` из **одного** LLM-батча получают общий
`group_id` и возрастающий `seg_idx`; следующий батч — новый `group_id`.

**Step 2: Реализовать** — `SchedulerToolExecutor.begin_group()` вызывается из
`dialog_core` перед обработкой батча tool_calls; счётчик `seg_idx` внутри группы.
Точка врезки — тот же ре-ордеринг батча, что W7a уже сделал в
`rob_box_harness/core/dialog_core.py`.

**Step 3:** Commit.

---

## Фаза S3 — `TaskScheduler.update()` с инвариантом §2.3

### Task 3.1: Модель дельты

**Files:**
- Create: `src/rob_box_voice/rob_box_voice/scheduler/delta.py`
- Test: `src/rob_box_voice/test/test_scheduler_delta.py`

**Step 1: Тест** — `TaskDelta` с операциями `rewrite(seg_idx, args)`,
`replace(seg_idx, args)`, `append(args)`, `drop(seg_idx)`. Валидация: ссылка на
несуществующий `seg_idx` → `ValueError`; пустая дельта → `ValueError`.

**Step 2: Реализовать** — frozen dataclass, чистый Python, по образцу
`decision.py:19-60` (там уже есть валидация плана в `__post_init__`).

**Step 3:** Commit.

### Task 3.2: `update()` — правит только PENDING

**Files:**
- Modify: `scheduler/task_scheduler.py` (`_Channel.remove:359` — тот же
  rebuild-from-snapshot приём переиспользуется для замены)
- Test: `src/rob_box_voice/test/test_task_scheduler.py`

**Step 1: Тест — это и есть главный инвариант плана (§2.3):**
- `update()` на `RUNNING`-сегмент → он **не меняется** и **не отменяется**;
- `update()` на `QUEUED`/`SCHEDULED` → payload заменён, порядок сохранён;
- `append` добавляет в хвост очереди канала;
- `drop` снимает PENDING (переиспользует `remove`);
- группа с уже завершёнными сегментами → `update` их игнорирует;
- `update` несуществующей группы → `TaskNotFoundError`.

**Step 2: Реализовать** — `update(group_id, delta) -> UpdateReport`. Внутри —
снапшот очереди канала, применение дельты, обратная заливка. Обязательно эмитить
`task.updated` (`_emit`, `:272`) → уедет в `/harness/task_events` через уже
существующий W7c-publisher (`dialogue_node.py:1331`).

**Step 3:** `cd src/rob_box_voice && PYTHONUTF8=1 python -m pytest test/test_task_scheduler.py -q` → PASS. Commit:
```bash
git commit -m "feat(scheduler): TaskScheduler.update() honouring the ACTIVE invariant"
```

⚠️ **Гонка**: между снятием снапшота и заливкой `_pump` может забрать голову
очереди. Тест на это обязателен (`update` конкурентно с `_pump`) — иначе
сегмент исполнится дважды или потеряется.

---

## Фаза S4 — `quick_decide`: уровень правил

Строго по ревизии v5 (`SCHEDULER_DESIGN.md:554-604`): **никакой второй LLM**.
Правила ловят только явный мусор, всё остальное отдают основному циклу.

### Task 4.1: Модуль правил

**Files:**
- Create: `src/rob_box_voice/rob_box_voice/scheduler/quick_decide.py`
- Test: `src/rob_box_voice/test/test_quick_decide.py`

**Step 1: Тест** (acceptance из §4.7.4):
- `IGNORE`: междометия («угу», «ага», «мхм»), < 2 слов, дословный повтор
  предыдущей фразы, `confidence < 0.4`;
- `REPLACE`: явные императивы («хватит», «стоп», «замолчи», «отставить») —
  **кроме** music-stop, который уже разруливается `_MUSIC_STOP_OVERRIDES`
  (`dialogue_node.py:1778-1784`);
- `PENDING_LLM`: **всё остальное**, и отдельным тестом — правила
  **НИКОГДА** не решают «и ещё про енота», «а потом Y», «про него»;
- решение за < 50 мс (замер в тесте).

**Step 2: Реализовать** — чистый модуль без ROS, по образцу
`core/dialogue_text.py`. Сигнатура из §4.7.3:
```python
def quick_decide(text, *, source, active_group, clock) -> QuickVerdict
# QuickVerdict ∈ {IGNORE, REPLACE, PENDING_LLM}
```

**Step 3:** Commit.

### Task 4.2: Врезка вердикта в `_on_stt`

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/dialogue_node.py:1809`
- Test: `src/rob_box_voice/test/unit/node/test_barge_in_policy.py` (NEW; образец — `test/unit/node/test_speech_backlog_accumulator.py`). ⚠️ НЕ `test/test_dialogue_shell.py` — он требует настоящий ROS 2 и локально не собирается.

**Step 1: Тест** — при `policy=classify`: `IGNORE` → турн не запускается вовсе;
`REPLACE` → `_cancel_run(stop_tts=True)` как раньше; `PENDING_LLM` → турн
запускается **без** `STOP`.

**Step 2: Реализовать** — заменить безусловный вызов из Task 1.3 на диспетч по
вердикту. При `policy=replace` — `quick_decide` не вызывается вообще.

**Step 3:** Commit.

---

## Фаза S5 — `[SEGMENT PLAN]` в контекст LLM

Без этого блока LLM физически не может выдать осмысленную дельту — она не знает,
что играет и что можно править (§7.1).

### Task 5.1: Рендерер блока

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/scheduler/tool_executor.py:223`
  (рядом с существующим `active_tasks_block`)
- Test: `src/rob_box_voice/test/test_tool_executor.py`

**Step 1: Тест** — формат по §7.1: строки `ACTIVE:` / `PENDING:` с `seg_idx`,
кратким payload и `remaining`; строка `REWRITEABLE_SEGMENTS: [...]`; строка
`AT_RISK_ON_REPLACE: [...]`. Блок **пустой**, когда активной группы нет
(в idle его быть не должно).

**Step 2: Реализовать** — `segment_plan_block()` поверх
`TaskScheduler.segments(group_id)` из Task 2.2. `FROZEN` пока не различаем —
это S9; на этом шаге все PENDING считаются `LIVE`/rewriteable.

**Step 3:** Commit.

### Task 5.2: Подключить к системному контексту

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/dialogue_node.py:2470-2487`
- Test: `src/rob_box_voice/test/unit/node/test_dialogue_node.py`

**Step 1: Тест** — `_build_dynamic_system_context` содержит `[SEGMENT PLAN]`
при активной группе и не содержит в idle. Ошибка рендера не роняет контекст
(тот же `try/except`, что уже стоит вокруг `active_tasks_block`).

**Step 2: Реализовать** — добавить вызов рядом с существующим блоком W7c.

**Step 3:** Commit.

### Task 5.3: Правила для LLM в мастер-промпте

**Files:**
- Modify: `src/rob_box_voice/prompts/master_prompt_compact.txt`
- Test: `src/rob_box_voice/test/unit/test_master_prompt_*.py` (по образцу
  `test_issue_1709_prompt_unicode_speech.py`)

**Step 1: Тест** — промпт содержит правило: «видишь `[SEGMENT PLAN]` с
`REWRITEABLE_SEGMENTS` → правь их через `task_delta`, **не** начинай заново и
**не** зови `stop_music`».

**Step 2: Реализовать** — короткая секция. Держать в тонусе бюджет промпта:
`master_prompt_compact.txt` уже 415 строк.

**Step 3:** Commit.

---

## Фаза S6 — MCP-тул `task_delta`

### Task 6.1: Тул

**Files:**
- Create/Modify: `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/scheduler.py`
- Modify: `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py` (регистрация)
- Test: `src/rob_box_mcp_tools/test/test_tools/test_scheduler_delta.py`

**Step 1: Тест** — `task_delta(group_id, ops[])` валидирует операции,
недоступный scheduler → честная ошибка (`success=False`), не молчаливая
деградация (`docs/architecture/tts-extension-points.md`, `capability-honest`).
`execution_type = FAST`, `destructive = False`.

**Step 2: Реализовать** — тонкая обёртка над `TaskScheduler.update`.
Регистрация — рядом с прочими в `mcp_server.py`.

**Step 3:** `cd src/rob_box_mcp_tools && PYTHONUTF8=1 python -m pytest test -q` → PASS. Commit.

### Task 6.2: Маршрутизация тула мимо очереди

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/scheduler/tool_executor.py:47-52`
- Test: `src/rob_box_voice/test/test_tool_executor.py`

**Step 1: Тест** — `task_delta` исполняется **напрямую** (bypass), как
музыкальные стартеры: он правит очередь и обязан вернуть LLM реальный результат,
а не `{"status": "queued"}`.

**Step 2: Реализовать** — добавить в bypass-набор.

**Step 3:** Commit.

---

## Фаза S7 — Очередь pending user-messages

Закрывает дыру, которую открывает S1: если турн **всё-таки** в полёте (LLM ещё
думает), а мы больше не отменяем — нельзя запускать второй турн параллельно.
§4.7.3: «фраза ставится в очередь `user_messages` для следующего хода».

### Task 7.1: Очередь и дренаж

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/dialogue_node.py` (`_dispatch_turn`
  и `finally`-блок `_run_turn` около `:2790`)
- Test: `src/rob_box_voice/test/unit/node/test_barge_in_policy.py` (NEW; образец — `test/unit/node/test_speech_backlog_accumulator.py`). ⚠️ НЕ `test/test_dialogue_shell.py` — он требует настоящий ROS 2 и локально не собирается.

**Step 1: Тест**
- вердикт `PENDING_LLM` при живом `_run_task` → фраза кладётся в очередь,
  второй турн **не** стартует;
- по завершении турна очередь дренится **одним** турном (несколько накопленных
  фраз склеиваются, а не порождают N турнов);
- очередь чистится в `_reset_dialogue_session` (`:4460`) вместе с аккумулятором;
- очередь ограничена сверху (защита от залипания), переполнение логируется.

**Step 2: Реализовать** — `self._pending_user_messages: deque`, дренаж в
`finally` `_run_turn`, очистка в reset.

**Step 3:** Commit.

**Acceptance S7 (§4.7.4):** pending-фраза попадает в LLM-цикл за ≤ 200 мс после
освобождения турна — метрика в лог.

---

## Фаза S8 — e2e: «комар + енот» и переписанный barge-in

> **Это точка, где цель считается достигнутой.** Без зелёного прогона тут
> задача не закрывается (ADR-0018).

### Task 8.1: Сценарий MERGE

**Files:**
- Create: `docker/vision/test/scenario_runner/scenarios/05_merge_song.yaml`
- Modify: `docker/vision/test/config/voice_assistant_test.yaml`
  (`barge_in_policy: classify`)
- Modify: scenario_runner — новые ассерты

**Step 1:** Новые ассерты в раннере:
- `assert_tts_not_stopped` — между двумя инъекциями `STOP` в
  `/voice/tts/control` не приходил;
- `assert_task_event` — в `/harness/task_events` (топик уже публикуется,
  `dialogue_node.py:415`) есть `task.updated` для активной группы.

**Step 2:** Сценарий дословно по §2.5 спека:
```yaml
- inject_stt_no_wait: "робок спой песню про комара"
- wait_for_speaking: true
- inject_stt: "робок и ещё про енота"
  assert_tts_not_stopped: true
  assert_task_event: "task.updated"
```

**Step 3:** Прогон на стенде, **raw-вывод обязателен**: `docker logs`,
`ros2 topic echo /harness/task_events`, вывод раннера. Commit.

### Task 8.2: Переписать `00_barge_in.yaml`

**Files:**
- Modify: `docker/vision/test/scenario_runner/scenarios/00_barge_in.yaml`

Сценарий `barge_in_joke_then_memory` (`:28-31`) сегодня утверждает «Робот
**должен остановиться**» — это ровно то поведение, которое спек отменяет.

**Step 1:** Разделить на два:
- `barge_in_replace` — императив («хватит, расскажи про память») → робот
  останавливается. Проверяет ветку `REPLACE`, остаётся как есть.
- `barge_in_merge` — не-императив → робот **не** останавливается.

**Step 2:** Прогон обоих + raw. Commit.

### Task 8.3: Включить `classify` по умолчанию

**Files:**
- Modify: `docker/vision/config/voice_assistant/dialogue_node.yaml`

Только **после** зелёных 8.1 и 8.2 с приложенным raw-выводом. Флаг `replace`
остаётся как аварийный откат.

---

## Фаза S9 — FROZEN / LIVE (§6.5)

Оптимизация: не давать LLM переписывать то, до чего она не успеет.

### Task 9.1: Пометка границы

**Files:**
- Modify: `scheduler/task_scheduler.py`, `scheduler/pre_gen.py`
- Test: `src/rob_box_voice/test/test_speculative_pre_gen.py`

**Step 1: Тест** — `PreGenPlan.boundary_idx` (`pre_gen.py:97-115`, уже написан)
делит PENDING на `PENDING_FROZEN` (до границы) и `PENDING_LIVE` (после);
пересчёт при старте сегмента, при `LLMEstimator.record` и при `update()`
(три триггера из §2.1).

**Step 2:** Реализовать. `update()` по `FROZEN`-сегменту → отказ +
`CANCEL_REASON_MERGE_TOUCHED_FROZEN` (константа уже есть,
`speculative_executor.py:89`) → отмена пре-гена и перегенерация.

**Step 3:** Commit.

### Task 9.2: Отразить в `[SEGMENT PLAN]`

**Files:** `scheduler/tool_executor.py`, промпт.

`REWRITEABLE_SEGMENTS` = только `LIVE`. `AT_RISK_ON_REPLACE` = все PENDING.

---

## Фаза S10 — Авто-триггер LLM-после-MERGE (§4.5)

Нужен для **системных** событий (`battery_critical`), где пользователь молчит и
некому принести `task_delta`. Для «комар + енот» не требуется — там фраза
пользователя сама поднимает турн.

### Task 10.1: Триггер по трём условиям

**Files:**
- Modify: `scheduler/task_scheduler.py`, `scheduler/event_bus.py`
- Test: новый

**Step 1: Тест** — триггер срабатывает **только** при одновременном:
(1) есть группа с PENDING, (2) есть неприменённый event, (3) канал выглядит
требующим продолжения. Анти-паттерны из §4.5: не срабатывает во время ACTIVE
voice-сегмента, не срабатывает на `priority=low`, не срабатывает без PENDING.

**Step 2:** Реализовать `llm_continue_hook`. По ревизии v5 — зовёт **основную**
LLM, не лёгкую.

**Step 3:** Commit.

**Acceptance:** ложных срабатываний ≤ 1 на прогон при ≥ 5 истинных (§4.5).

---

## Фаза S11 — W7d: снять костыли

Разрешено только после зелёного S8 — это прямое указание
`W7_INTEGRATION_PLAN.md` («W7d последним, после зелёного e2e»).

**Files:** `dialogue_node.py` — `_pending_music_cleanup` (`:593, 2054-2060,
2167-2176, 2340-2342, 2804-2871`), `_publish_music_cleanup`, остатки debounce.

**Acceptance (из W7d):** grep по `dialogue_node.py` — `_pending_music_cleanup`
пуст; `music_cleanup` публикуется только со стороны scheduler.

⚠️ Это самая крупная чистка: восемь мест, завязанных на issue #980/#992.
Отдельным PR, с прогоном всего голосового e2e-набора.

---

## Фаза S12 — Метрики и документация

- **12.1** Prometheus: счётчик вердиктов `IGNORE/REPLACE/PENDING_LLM`, счётчик
  `task.updated`, гистограмма queue-latency pending-сообщений. Рядом с
  существующим `record_barge_in` (`dialogue_node.py:2755`,
  `observability/metrics.py`).
- **12.2** `SCHEDULER_DESIGN.md §11.6` — таблица статусов фаз; §12 — вычеркнуть
  закрытые OPEN issue (блокер П1, сегментная модель).
- **12.3** `dialogue-mode-spec-2026-08-28.md §2.5` — снять пометку «инвариант»
  как цели, сослаться на реализацию.
- **12.4** ADR на «MERGE не распространяется на музыкальный канал» (решение из
  §0.2 этого документа) — либо запись в `SCHEDULER_DESIGN.md §13`.

---

## 2. Acceptance целиком (что считаем «сделано»)

Из issue #968 + §2.5 спека. Каждый пункт требует raw-вывода (ADR-0018).

- [ ] `update()` **не трогает** `RUNNING`-сегмент — unit-тест (Task 3.2)
- [ ] `update()` правит `QUEUED`/`SCHEDULED` — unit-тест
- [ ] Правила решают мусор за < 50 мс, и **никогда** не решают «и ещё про X» — unit-тест (Task 4.1)
- [ ] Pending-фраза попадает в LLM-цикл за ≤ 200 мс — метрика (Task 7.1)
- [ ] LLM видит `[SEGMENT PLAN]` и не добавляет пост-амбл при активном voice — e2e (INSIGHT #8, §7.3)
- [ ] **«Комар + енот»**: куплет 1 допевается до конца, куплет 2 уже про енота, паузы нет — e2e (Task 8.1)
- [ ] Императив «хватит» по-прежнему останавливает — e2e (Task 8.2)
- [ ] `barge_in_policy=replace` полностью воспроизводит сегодняшнее поведение — регресс

---

## 3. Риски

| # | Риск | Митигация |
|---|---|---|
| **R1** | Снятие `STOP` вешает TTS-awaiter'ы (`speak_helpers._tts_events`) → диалог замолкает навсегда | Task 1.2 разделяет отмену и глушение; тест «awaiter'ы отпущены, STOP не отправлен» — обязателен |
| **R2** | Гонка `update()` vs `_Channel._pump` — сегмент исполнится дважды или потеряется | Явный конкурентный тест в Task 3.2 |
| **R3** | Музыкальные стартеры вне планировщика (`tool_executor.py:49-52`, party-регрессия 19.08) → MERGE не работает для бита | Зафиксировано в §0.2 как осознанное ограничение; ADR в Task 12.4 |
| **R4** | LLM игнорирует `[SEGMENT PLAN]` и начинает песню заново | Task 5.3 (правило в промпте) + e2e-ассерт на `task.updated`, а не только на звук |
| **R5** | Промпт распухает (уже 415 строк `compact`) | Блок рендерится только при активной группе; правило — короткое |
| **R6** | W7d ломает issue #980/#992 (обрезанный рэп, cleanup на первом чанке) | S11 строго после зелёного S8, отдельным PR, полный голосовой e2e-набор |
| **R7** | `classify` меняет поведение на живых мероприятиях | Флаг с дефолтом `replace` до Task 8.3; откат — одна строка YAML |

---

## 4. Что НЕ входит

- Acceptance-подтверждения (`AWAITING_CONFIRMATION`, §8 дизайна) — фаза 1.5,
  уже частично приземлена в `rob_box_harness/core/acceptance.py`, отдельная тема.
- Reflex-слой (§8.10) — уже приземлён (`scheduler/reflex.py`), не трогаем.
- Вердикты `QUEUE` и `CLARIFY`. В S4 правила их не выдают: всё, что не мусор и
  не императив, идёт в LLM как `PENDING_LLM`, а дальше LLM сама решает
  `task_delta` (MERGE) либо обычный ответ. Отдельные вердикты `QUEUE`/`CLARIFY`
  — следующая итерация, после того как MERGE поедет на живых прогонах.
- Всё из фаз Ф3–Ф9 аудита (память, профиль голоса, супервизор, панели).
