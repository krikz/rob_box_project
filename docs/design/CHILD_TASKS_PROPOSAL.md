# План нарезки scheduler на дочерние карточки — для согласования

Контекст: issue #968, SCHEDULER_DESIGN.md v5 (1796 строк, коммит 5cd5445a) на ветке feature/harness-p0-foundation. План — по §11 SCHEDULER_DESIGN.md (фазы 1–4).

## Блокер ревью #17 — должен быть исправлен ПЕРВЫМ (иначе разработчик не найдёт "уже существующий" механизм)

Карточка #0 (developer, ~30 мин): **Поправить ссылки на несуществующий код в SCHEDULER_DESIGN.md**

В §3.3, §8.2, §9.2, §12, §15 SCHEDULER_DESIGN.md указаны ссылки на `dialogue_node.py:672–702` (блокирующий `speak_text` через `_output_lock`). Эти строки НЕ СУЩЕСТВУЮТ — файл 578 строк, `_output_lock` нигде в `src/` нет. Реальный код — в `src/rob_box_voice/rob_box_voice/core/speak_helpers.py:110–142` (`_tts_events` dict + `register_tts` + ожидание finished).

**Что сделать:**
- Заменить все вхождения `dialogue_node.py:672–702` / `_output_lock` на `speak_helpers.py:110–142` (5+ мест).
- Проверить инвариант "один finished на speech_id" на текущем HEAD (был зафиксирован f3b58ef1, но e2e v38 показал 3 finished на 1 speech_id).
- Коммит: `fix 968 phase 0 step 0 doc-links`

**Acceptance:** grep по `dialogue_node.py:672` пуст; e2e v38 без дублированных finished.

---

## Фаза 1 — MVP

Карточка #1 (backend, ~2–3 дня): **TaskScheduler MVP с каналами voice/music/anim + интеграция в speak_text/execute_music_code/stop_music через dialogue_node**

Шаги:
1. `TaskScheduler` (FIFO-очереди по каналам voice/music/anim) в `src/rob_box_voice/rob_box_voice/scheduler/task_scheduler.py`
2. `Channel` (базовый класс с `enqueue`, `_channel_lock`, `cancel`)
3. Интеграция в `dialogue_node.py`: `speak_text`, `execute_music_code`, `stop_music` — через scheduler, не напрямую
4. **Удалить блокирующий `await tts/finished` из `speak_text`** (см. §3.3 SCHEDULER_DESIGN.md; реальный код в `speak_helpers.py` уже неблокирующий — проверить, что scheduler наследует это)
5. Quick-decide уровень 1 (правила + IGNORE для шума) — pure-data конфиг в YAML/JSON
6. Feedback events в LLM-контекст — `[CHANNELS]` блок (queue depth, current, eta) в форматтере

**Acceptance (полный список из §11.1):**
- [ ] `stop_music()` не доходит до железа раньше конца TTS-чанка
- [ ] e2e v36, v38 перестают падать
- [ ] L1 quick-decide < 50мс на типовых фразах
- [ ] LLM видит блок `[CHANNELS]` в каждом ходе
- [ ] Пост-амбл "Готово! Зачитал тебе..." не появляется при активном voice-канале
- [ ] Инвариант сегментов (unit): `update()` модифицирует только PENDING, ACTIVE не затрагивается
- [ ] `speak_text` НЕ делает `await tts/finished` (grep пуст)
- [ ] Два подряд `speak_text` возвращают за < 50мс каждый

Коммит: `fix 968 phase 1 step 1 mvp-scheduler`

---

## Фаза 1.5 — Acceptance tool-calling (расширение §8)

Карточка #2 (developer, ~2–3 дня, можно параллельно с #1): **Сегменты nav/mapping/mutate через AWAITING_CONFIRMATION + ToolConfirmationPolicy**

Шаги:
1. `ToolConfirmationPolicy` — pure data, загрузка из YAML, регистр из `mcp_server.py:312–375` (без хардкода в коде)
2. `nav_channel` с `safe_boundary_policy=soft` по умолчанию
3. `quick_decide` расширен: 6-й исход `CONFIRM`/`REJECT` при наличии AWAITING-сегмента
4. Voice-канал для формулировки плана: `speak_text` генерируется LLM по feedback event `awaiting_confirmation(plan, eta)`
5. Таймаут 20с + автоозвучка "отменяю" через `SchedulerConfig.confirmation_timeout_ms`
6. EventBus события: `awaiting_confirmation`, `confirmation_accepted`, `confirmation_rejected`, `navigation_at_safe_boundary`
7. Блок `[AWAITING]` в feedback events

**Acceptance (полный список из §11.2):**
- [ ] Сегмент `navigate_to_waypoint(кухня)` создаётся в AWAITING (unit test)
- [ ] Сегмент `stop_navigation` — сразу ACTIVE, минуя AWAITING (🟢 аварийный)
- [ ] Сегмент `speak_text` — PENDING → ACTIVE без подтверждения (🟢 безопасный)
- [ ] "да" → AWAITING → ACTIVE, таймер отменяется (e2e сценарий 1)
- [ ] "нет" → AWAITING → REJECTED + feedback event (e2e сценарий 8)
- [ ] Таймаут 20с → REJECTED + "отменяю" (e2e сценарий 7)
- [ ] safe_boundary: ACTIVE-навигация + новый план → CANCELLED + новый AWAITING (e2e сценарий 3)
- [ ] `stop_navigation` во время AWAITING исполняется мгновенно (e2e сценарий 4)
- [ ] Race-protection: ввод во время AWAITING интерпретируется как ответ на confirm (unit test)
- [ ] Блок `[AWAITING]` присутствует в feedback (unit test)
- [ ] Property-based тест: классификатор НИКОГДА не возвращает `requires_confirm=true` для `stop_navigation`

Коммит: `fix 968 phase 1 step 5 awaiting-confirmation`

---

## Фаза 2 — двухуровневое решение + SchedulerEventBus

Карточка #3 (backend, ~3–4 дня, ПОСЛЕ #1): **Quick-decide уровень 2 + SchedulerEventBus + разрешение блокера П1**

Шаги:
1. Quick-decide уровень 2 (лёгкая LLM, < 800мс) — `quick_decide()` (§4.6.2)
2. **`SchedulerEventBus`** для системных событий (`/battery/*`, `/obstacle/*`, `/hermes/*`) — НЕ путать с ADR-0001 `SideEffectBus`
3. Приоритеты событий (critical > high > normal > low)
4. Сценарий "батарея": `battery_critical` вплетается в PENDING-сегмент
5. Решение блокера П1: MERGE/QUEUE не отменяют LLM-цикл
6. Авто-триггер внеочередного LLM-после-MERGE (§4.5) — scheduler сам инициирует LLM-ход для `task_delta`

**Acceptance (полный список из §11.3):**
- [ ] Решение по новому вводу (любое из 5) < 800мс
- [ ] "и ещё про енота" во время куплета 1 → куплет 2 без паузы и рестарта
- [ ] `battery_critical` вплетается в PENDING без прерывания ACTIVE
- [ ] Два MERGE подряд за < 2с (unit test)
- [ ] Приоритеты: `obstacle` (critical) прерывает песню на границе такта, `battery` (high) вплетается в PENDING
- [ ] e2e "батарея" без паузы и рестарта
- [ ] Авто-триггер: `battery_critical` без юзер-ввода провоцирует LLM-ход через scheduler
- [ ] Класс шины именован `SchedulerEventBus`, не `EventBus` глобально

Коммит: `fix 968 phase 2 step 1 l2-decide-and-eventbus`

Карточка #4 (документатор/developer, ~30 мин, ПОСЛЕ #3): **ADR-0001 follow-up: разграничение SchedulerEventBus vs SideEffectBus**

Шаги:
1. Добавить запись в ADR-0001 §5: scheduler владеет `SchedulerEventBus` (фаза 2), `SideEffectBus` (ADR-0001) — в фазе 5; они НЕ пересекаются по scope
2. Зафиксировать, что scheduler и `dialogue_node` НЕ импортируют `rob_box_harness.side_effect_bus` до фазы 5

**Acceptance:**
- [ ] В ADR-0001 §5 есть явный абзац о разделении
- [ ] grep `rob_box_harness.side_effect_bus` в `src/rob_box_voice/` пуст

Коммит: `fix 968 phase 2 step 2 adr-followup-eventbus`

---

## Фаза 2.5 — Reflex-слой

Карточка #5 (backend, ~1–2 дня, можно параллельно с #3): **Reflex-слой: прямые команды без LLM через SchedulerEventBus**

Шаги:
1. Интеграция `command_node` ↔ `SchedulerEventBus`: публикация `/reflex/events` с `source="reflex"` и правильным приоритетом
2. Подписка scheduler'а на `/reflex/events`
3. Публикация scheduler'ом `task.cancelled(reason=reflex_stop)` в `/harness/task_events` для подтверждения command_node
4. Контекстно-зависимый роутер (§8.10): reflex-«стой» во время AWAITING отменяет без штрафа; reflex-«направо» во время песни — параллельно

**Acceptance (полный список из §8.10.6–8.10.8):**
- [ ] Reflex-команды попадают в `SchedulerEventBus` с правильным приоритетом (unit test)
- [ ] Reflex-«стой» отменяет ВСЕ активные задачи синхронно, latency < 500мс (e2e сценарий 1)
- [ ] Reflex-«направо» во время песни исполняется параллельно (e2e сценарий 3)
- [ ] Reflex-«стой» во время AWAITING_CONFIRMATION отменяет без штрафа (e2e)
- [ ] Полная двусторонняя интеграция: нет повторных cancel от command_node после feedback `task.cancelled` (лог-инспекция)

Коммит: `fix 968 phase 2 step 5 reflex-layer`

---

## Фаза 3 — эстиматоры + speculative pre-generation

Карточка #6 (developer, ~2–3 дня, ПОСЛЕ #1): **SegmentEstimator + LLMEstimator + speculative pre-gen**

Шаги:
1. `SegmentEstimator` (обёртка над `estimate_tts_duration` + music/anim метриками)
2. `LLMEstimator` (EMA latency)
3. Speculative pre-gen: пред-синтез TTS сегмента N+1
4. `InterruptibleTask.cancel()` при правке во время пред-генерации
5. Fill-сегменты (ambient music) при опоздании LLM

**Acceptance (полный список из §11.4):**
- [ ] `SegmentEstimator`: точность ±15% (unit test)
- [ ] `LLMEstimator`: EMA обновляется после каждого хода, видна в feedback events
- [ ] Пауза между сегментами при MERGE < 300мс
- [ ] При опоздании LLM — музыка/ambient продолжается, тишины нет
- [ ] Пред-генерация N+1 отменяется при правке без артефактов (unit test)

Коммит: `fix 968 phase 3 step 1 estimators-and-pregen`

---

## Фаза 4 (будущее) — action server + PASTE

Карточка #7 (devops/backend, ~3–5 дней, последняя): **tts_node → ROS2 Action Server + PASTE speculative execution**

Шаги:
1. Перевести `tts_node` на ROS2 Action Server (goal/feedback/result/cancel) — честный сигнал TTS готов из коробки
2. Speculative execution с shadow queue (Microsoft PASTE, arxiv 2603.18897) для prefetch результатов тулов
3. Полная замена debounce/таймеров на state-машины каналов

**Acceptance (полный список из §11.5):**
- [ ] `tts_node` принимает goal через Action interface, возвращает feedback по прогрессу
- [ ] cancel TTS через Action Server прерывает синтез до окончания (метрика latency от cancel до тишины < 200мс)
- [ ] Speculative execution: тул N+1 пред-исполняется, при правке отменяется без побочных эффектов
- [ ] Нет debounce/таймеров в `dialogue_node.py` (grep пуст)

Коммит: `fix 968 phase 4 step 1 action-server-and-paste`

---

## Итого карточек

| # | Название | Assignee | Блокируется от | Параллельно с | Объём |
|---|----------|----------|----------------|---------------|-------|
| 0 | Поправить ссылки на несуществующий код в SCHEDULER_DESIGN.md | developer | — | — | ~30 мин |
| 1 | Фаза 1 MVP (TaskScheduler + каналы + L1) | backend | #0 | #2, #5 | 2–3 дня |
| 2 | Фаза 1.5 Acceptance tool-calling | developer | #0 | #1, #5 | 2–3 дня |
| 3 | Фаза 2 L2-decide + SchedulerEventBus + блокер П1 | backend | #1 | — | 3–4 дня |
| 4 | ADR-0001 follow-up (SchedulerEventBus vs SideEffectBus) | developer | #3 | — | ~30 мин |
| 5 | Фаза 2.5 Reflex-слой | backend | #0 | #1, #2 | 1–2 дня |
| 6 | Фаза 3 эстиматоры + speculative pre-gen | developer | #1 | — | 2–3 дня |
| 7 | Фаза 4 action server + PASTE | devops | #3, #6 | — | 3–5 дней |

**Всего: 8 карточек (минимум по требованию — 6).**

Все коммиты — отдельные, атомарные, с префиксом `fix 968 phase N step M`.

## Открытые вопросы, требующие решения PM/Author (по §14)

Прежде чем стартовать карточки #1, #3, #5 — желательно закрыть:
- **Q5.1** (§14 #7): какой вариант A'1/A'2/A'3 рендера AWAITING-вопроса во время песни
- **Q3** (§14 #3): владелец ROS-топика `/harness/task_events` — отдельный node или publisher из dialogue_node
- **Q10** (§14 #10): ROS-топик vs прямой вызов для reflex-events (затрагивает #5)

Эти вопросы блокируют реализацию, не проектирование. Можно стартовать карточки #0, #2, #6 без них.
