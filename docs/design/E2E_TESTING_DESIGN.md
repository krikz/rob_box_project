# E2E Testing Design — привязка acceptance-критериев scheduler'а и dialog_node к e2e-прогонам

| Поле | Значение |
|------|----------|
| Документ | `docs/design/E2E_TESTING_DESIGN.md` |
| Связанное | `docs/design/SCHEDULER_DESIGN.md` v5, `analysis/dialog_node.md`, issue #968, parent-task `t_5fb8a092` (SUMMARY_8runs.json), recovery `t_4f546ead` (RECOVERY_REPORT.md) |
| Назначение | Спроектировать **e2e-тестирование**, которое проверяет, что поведение робота соответствует acceptance-критериям scheduler'а и архитектурным инвариантам dialog_node |
| Аудитория | QA / backend (developer) / reviewer |
| Дата | 2026-08-06 |
| Автор | Architect (kanban t_4f546ead, run 798) |
| Статус | **v1 — design proposal**, готов к ревью PM/QA |

## 0. TL;DR

**Проблема.** В проекте есть 2 дизайн-документа, описывающих **как должно быть**:
- `SCHEDULER_DESIGN.md` v5 (1797 строк, коммит 5cd5445a 04.08.2026) — 41 acceptance-пункт в 4 фазах, плюс 13 e2e-сценариев (§8.7 + §8.10.6).
- `analysis/dialog_node.md` (601 строка, 17.07.2026) — реверс-инжиниринг текущей `dialogue_node` (God Object 2040 строк), раздел 6 «Test Coverage» фиксирует, что 8 из 11 критических путей **НЕ покрыты** автоматическими тестами.

И есть **как есть** — прогоны 05.08 22:47–00:30 МСК (parent-task t_5fb8a092), recovery на живом роботе 06.08 09:00 МСК (t_4f546ead, run 797). 8 сценариев, 3 OK / 5 проблемных, главные находки:
1. WAKE_WORD drop 3/8 (фразы без префикса «робот»)
2. LLM latency 14–70с (MiniMax M2 thinking, не DeepSeek как думал parent)
3. TRACK execute_music_code success за 87–125мс, но **звук не зазвучал** (scsynth jack-маршрут, Renardo OSC)
4. TTS barge-in агрессивный (4/5 chunks в 06_kn)
5. voice-action-server крашится (`ModuleNotFoundError: aiohttp`, 399 рестартов) — **отдельный bug, не из 8 e2e**

**Между этими двумя мирами — щель.** Нет матрицы «acceptance-пункт → e2e-сценарий → метрика → доказательство». Эта матрица и есть суть документа.

**Решение.** Ввести **E2E-слой тестирования** из 3 уровней:
- **L0 — unit-инварианты** (быстро, в pytest, без робота): инвариант сегментов, классификатор тулов, `update()` не трогает ACTIVE, property-based «stop_navigation без confirm».
- **L1 — изолированные e2e на стенде** (docker-compose, без реального робота): один voice-канал, mock LLM/TTS/STT, проверка race-условий.
- **L2 — прогоны на живом роботе 10.1.1.21** (SSH, real docker logs, real scsynth, real TTS): 8 фраз из 05.08 как регресс-набор + новые фразы под scheduler-acceptance.

Главный продукт — файл `tests/e2e/ACCEPTANCE_MATRIX.csv` (машиночитаемая таблица: 41 acceptance-пункт × L0/L1/L2 × entry point × метрика × доказательство).

## 1. Бизнес-проблема и зачем нужен этот дизайн

### 1.1 Без матрицы — каждое ревью «с нуля»

SCHEDULER_DESIGN.md перечисляет 41 acceptance-пункт. Без явной привязки к e2e это «wishlist для ревьюера»: непонятно, как проверить, что «stop_music не доходит до железа раньше конца TTS-чанка» — это в pytest, на стенде или только глазами в логах робота?

dialog_node.md §6 «Test Coverage» прямо пишет:
> 6.3 Untested Critical Paths (8/11)
> 1. Dialogue state machine transitions
> 2. LLM provider fail-over (timeout → fallback)
> 3. STT wake word coordination
> 4. TTS chunk streaming with barge-in
> 5. Concurrent user inputs (race)
> 6. Redis session persistence
> 7. Multi-modal context aggregation
> 8. Tool-call dispatch (mcp_server.py)

То есть уже год назад было известно, что 8 из 11 критических путей не покрыты. Прогоны 05.08 показали, что минимум 3 из этих 8 — реальные баги (wake word drop, race LLM→track, barge-in storm).

### 1.2 Что даёт этот дизайн

1. **Каждый acceptance-пункт SCHEDULER_DESIGN имеет хотя бы один e2e-тест.** Если пункт «нужен только человеком с секундомером» — это явно зафиксировано как MANUAL.
2. **Каждый сценарий из 05.08 имеет статус PASS/FAIL в матрице.** Не «родитель думает, что звук есть» — а `verify_via=jack_lsp_or_wav` с конкретным threshold.
3. **Регрессия scheduler'а измерима.** 13 e2e-сценариев из §8.7+§8.10.6 → pytest-тесты или shell-сценарии. Запуск `pytest tests/e2e/test_scheduler_acceptance.py -v` даёт 13 строк PASS/FAIL.
4. **Воспроизводимость recovery.** Recovery-отчёт t_4f546ead показал, что данные о 05.08 прогонов собираются **только с живой машины** (буфер docker logs ротируется, GH artefacts неполные). Дизайн фиксирует минимальный «срез состояния» (state snapshot) для воспроизводимости.

### 1.3 Чего этот дизайн НЕ делает

- Не описывает unit-тесты scheduler'а (они в `src/rob_box_voice/test/test_task_scheduler*` — отдельный слой).
- Не предлагает менять `dialogue_node` (это ADR-0001, отдельная задача).
- Не автоматизирует полный pipeline на CI (CI не имеет ни робота, ни scsynth — только docker-compose стенд).

## 2. Принципы e2e-дизайна

| # | Принцип | Обоснование |
|---|---------|-------------|
| P1 | **Verify → escalate ladder** | Background-first: сначала SSH-команды + log-парсинг, потом SSH+jack_lsp+volumedetect, потом человек с секундомером. |
| P2 | **Threshold-based, не vibes-based** | «stop_music < 500мс» вместо «stop_music быстро». Метрика в матрице, без неё пункт не считается покрытым. |
| P3 | **Triple-verify для аудио-acceptance** | Аудио-результат верифицируем через 3 канала: (1) docker logs voice-assistant (event-уровень), (2) jack_lsp / scsynth (OSC-уровень), (3) volumedetect/RMS WAV (физический уровень). |
| P4 | **State snapshot обязателен** | Каждый e2e начинается с `systemctl/docker ps/uptime/git rev-parse HEAD` — без него результат несравним. |
| P5 | **Hermetic L1, non-hermetic L2** | L1-тесты на стенде мокают внешние API (LLM, TTS, STT, Renardo). L2-тесты на роботе используют реальные сервисы. L0-тесты полностью hermetic (без сети). |
| P6 | **Артефакты per-run** | Каждый прогон пишет: `va.log` + `sc.log` + `state.json` + `wav/` + `verdict.json`. Это то, что recovery-отчёт t_4f546ead делал руками. |
| P7 | **Reuse существующее** | `tests/integration/test_e2e_harness_minimax.py` (уже есть), `src/rob_box_voice/test/test_task_scheduler*.pyc` (был, исходник потерян — нужно восстановить из истории), `docker-compose.yml` стенд. |

## 3. Уровни тестирования (L0 / L1 / L2)

### 3.1 L0 — Unit-инварианты (pytest, без сети)

**Где:** `src/rob_box_voice/test/unit/scheduler/`, `tests/unit/core/scheduler/`
**Когда:** каждый PR, в CI обязателен
**Скорость:** 100% тестов < 30с
**Покрытие:** 41 acceptance-пункт → ~28 unit-тестов (для пунктов, проверяемых в изоляции)

Entry points:
- `TaskScheduler` (фаза 1) — `update()` не трогает ACTIVE; FIFO-порядок в канале; cancel останавливает исполнителя
- `Segment` dataclass — переходы `PENDING → ACTIVE → COMPLETED/CANCELLED`; `AWAITING_CONFIRMATION → ACTIVE/REJECTED`
- `Channel` — `_channel_lock` сериализует enqueue/dequeue; приоритеты `critical > high > normal > low`
- `ToolConfirmationPolicy` — `requires_confirm()` для каждого тула из `mcp_server.py:312–375`; **property-based тест**: `stop_navigation.requires_confirm() == False` для всех входов
- `LLMEstimator` — EMA-обновление после каждого хода
- `SegmentEstimator` — `estimate_tts_duration(chars) → seconds` с точностью ±15%
- `quick_decide()` уровня 1 — 5-вердиктный (MERGE/REPLACE/QUEUE/IGNORE/CLARIFY) для типовых фраз < 50мс

Доказательства:
- `assert`-ы в pytest
- coverage-отчёт `pytest --cov=rob_box_voice.scheduler` (должен быть ≥ 90% для `scheduler/`)
- property-based тесты на базе `hypothesis` (для `ToolConfirmationPolicy`)

### 3.2 L1 — Изолированные e2e на стенде (docker-compose, mock API)

**Где:** `tests/e2e/stend/`, docker-compose стенд `compose/stend/e2e.yaml`
**Когда:** каждый PR в develop, ночью на main, при релизе scheduler'а
**Скорость:** 1 e2e ≈ 30–90с; полный набор 13 сценариев ≈ 15–20 мин
**Покрытие:** все 13 e2e-сценариев из §8.7+§8.10.6, плюс race-тесты

Окружение:
- docker-compose: `voice-assistant` (с тестовым MCP-сервером) + `supercollider` (с тестовым sclang) + `tts-mock` + `stt-mock` + `llm-mock` (отвечает за 200мс, выдаёт canned tool_calls)
- jackd в headless-режиме (или `jackd -d dummy`) — `jack_lsp` показывает порты
- `renardo-mock` — отвечает OSC `/done` мгновенно, `/fail` с вероятностью 0.1 для тестов ретраев
- `rec-mock` — записывает .wav в `tests/e2e/_artifacts/<run_id>/`

Что проверяем (примеры):
- §8.7 сценарий 1 («кухня» → «да»): AWAITING → ACTIVE, навигация, «прибыл»
- §8.7 сценарий 4 («кухня» едет → «стой»): stop_navigation **мгновенно** (порог 100мс), не дожидаясь confirm
- §8.7 сценарий 7 («кухня» → молчит 20с): AWAITING → REJECTED + автоозвучка «отменяю»
- §8.10.6 сценарий 3 («спой» → «налево»): voice продолжает играть, nav-сегмент исполняется параллельно
- Race: 100 итераций «быстрое „да" во время AWAITING» → 100 раз должен быть CONFIRM, не CLARIFY
- Property: «stop_navigation никогда не требует confirm» — 1000 случайных входов

Entry points:
- `docker compose -f compose/stend/e2e.yaml up -d`
- `pytest tests/e2e/stend/test_scheduler_acceptance.py -v` (13 тестов, помеченных `@pytest.mark.scheduler_e2e`)
- `pytest tests/e2e/stend/test_scheduler_race.py -v` (5 race-тестов)

Доказательства:
- `tests/e2e/_artifacts/<run_id>/va.log` (voice-assistant events)
- `tests/e2e/_artifacts/<run_id>/sc.log` (scsynth OSC)
- `tests/e2e/_artifacts/<run_id>/state.json` (state machine: какой сегмент в каком статусе в какое время)
- `tests/e2e/_artifacts/<run_id>/wav/*.wav` (записанный выход)
- `tests/e2e/_artifacts/<run_id>/verdict.json` (`{"pass": true, "metric": {...}}`)

### 3.3 L2 — Прогоны на живом роботе (10.1.1.21, real services)

**Где:** `tests/e2e/live/`, скрипт `scripts/run_live_e2e.sh`
**Когда:** перед релизом scheduler'а в main; после значимых правок в Renardo/SC/MCP; раз в неделю (cron)
**Скорость:** 1 e2e ≈ 2–5 мин (с учётом LLM latency 14–70с); 8 фраз ≈ 30 мин
**Покрытие:** регресс 8 фраз 05.08 + новые фразы под scheduler-acceptance

Окружение:
- Робот 10.1.1.21 (SSH через sshpass), контейнеры `voice-assistant`, `supercollider` — **те же самые, что в проде**
- Реальные API: MiniMax (LLM), MiniMax TTS, Yandex STT, Renardo → scsynth
- **Без LLM-мока** — это и есть смысл L2 (проверка реальной интеграции)

Что проверяем:
Что проверяем (по 8 фразам 05.08 как регресс + новые):

**Регресс-набор (R1–R8, идентично прогонам 05.08):**
- R1 «робот сыграй баха» — был: TIMEOUT_LLM; ожидание после scheduler: TRACK + 30–60с звука
- R2 «робот сыграй в траве сидел кузнечик» — был: TRACK_FINISH 2.3с, нет звука; ожидание: 30–60с звука
- R3 «робот зачитай рэп про енотика» — был: WAKEWORD_DROP (без префикса «робот»); ожидание: wake fail-fast < 200мс с понятным логом
- R4 «робот спой песенку про котика» — был: OK с 50с backing (RMS=1162); ожидание: OK (parent-summary ошибочен — RMS=-24dB это TTS-хвост, см. recovery §4)
- R5 «робот будь диджеем таракан» — был: WAKEWORD_DROP; ожидание: wake fail-fast
- R6 «робот говори по китайски» — был: OK_CHAT с barge-in 4/5; ожидание: barge-in ≤ 1/5 chunks
- R7 «робот будь лениным» — был: WAKEWORD_DROP; ожидание: wake fail-fast
- R8 «робот хватит» — был: OK; ожидание: OK + music_cleanup < 500мс

**Новые сценарии под scheduler-acceptance (N1–N13):**
- N1 «робот, едь на кухню» → «да» (§8.7 #1) — AWAITING → ACTIVE → safe_stop → COMPLETED
- N2 «робот, едь на кухню» (едет) → «и потом зал» → «да» (§8.7 #3) — safe_stop на границе
- N3 «робот, едь на кухню» (едет) → «робот, стой!» (§8.7 #4) — stop_navigation < 100мс
- N4 «робот, спой песню» → «робот, стой!» (§8.10.6 #1) — все ACTIVE → CANCELLED < 500мс
- N5 «робот, спой песню» → «робот, налево» (§8.10.6 #3) — voice продолжается, nav параллельно
- N6 «стоп» × 3 подряд (§8.10.6 #4) — debounce 500мс, только первый реально отменяет
- N7 «кухня?» → молчит 20с (§8.7 #7) — AWAITING → REJECTED + «отменяю»
- N8 «кухня?» → «нет, в зал» (§8.7 #8) — confirm-rejected → перегенерация → новый AWAITING
- N9 «спой песню» → «и ещё про енота» (фаза 2) — куплет 2 без паузы
- N10 «поёт» → battery_critical (фаза 2) — вплетается в PENDING, не прерывает ACTIVE
- N11 «кухня?» → «стой!» (фаза 1.5 reflex + confirm) — reflex-cancel, AWAITING → REJECTED, всё CANCELLED
- N12 «поёт» → «робот, направо» (фаза 1.5 parallel) — voice продолжается, nav параллельно
- N13 «кухня?» (едет) → «робот, направо» (фаза 1.5 nav-correction) — safe_stop + новый nav-сегмент

Entry point (L2):
- `bash scripts/run_live_e2e.sh --robot=10.1.1.21 --scenarios=R1,R2,R3,R4,R5,R6,R7,R8,N1,N2,N3 --report=tests/e2e/_artifacts/live_$(date +%Y%m%d_%H%M%S)/`
- Скрипт: SSH на 10.1.1.21, проверяет `docker ps`, синхронизирует часы, запускает сценарий через `dialog_e2e_*.wav` воспроизведение с билдовой 10.1.1.249, параллельно `docker logs -f` собирает в файл, после — `jack_lsp -c` + `volumedetect` на .wav

Доказательства (L2 — то, что делал recovery t_4f546ead, но автоматизированно):
- SSH `docker logs voice-assistant --since 2026-08-06T09:00:00Z --until 2026-08-06T09:30:00Z` → `va.log`
- SSH `docker logs supercollider --since ...` → `sc.log`
- SSH `docker exec voice-assistant curl -s http://localhost:8080/state` (если есть) → `state.json`
- scp `dialog_e2e_*.wav` с 10.1.1.249 → `wav/`
- `ffmpeg -i ... -af volumedetect -f null -` → RMS по 1-секундным окнам
- `verdict.json` (PASS/FAIL + метрики + ссылки на логи)

## 4. Acceptance-матрица (главный продукт)

> Файл машиночитаемой версии: `tests/e2e/ACCEPTANCE_MATRIX.csv` (будет сгенерирован из этого раздела). Колонки: `id, source_doc, source_section, text, level, test_id, entry_point, metric, threshold, evidence, current_status, owner`.

### 4.1 Фаза 1 — MVP (8 acceptance-пунктов)

| # | Acceptance-пункт (SCHEDULER_DESIGN §11.1) | L | Test ID | Entry point | Метрика / threshold | Доказательство | Статус 06.08 |
|---|--------------------------------------------|---|---------|-------------|----------------------|----------------|---------------|
| A1 | `stop_music()` не доходит до железа раньше конца TTS-чанка | L0+L2 | test_channel_fifo_order + live_R1 | unit + ssh | enqueue ts vs dequeue ts для music-канала, delta > tts_chunk_duration | logs+scsynth | **FAIL** (recovery §5.1: scsynth-OSC path сломан, watchdog 131.8с) |
| A2 | e2e v36, v38 перестают падать | L2 | live_R4 (аналог v36) | live | RMS region > 10с на backing-канале | wav | **PARTIAL** (R4 OK, но только TTS, не backing) |
| A3 | L1 quick-decide < 50мс на типовых фразах | L0 | test_quick_decide_l1_latency | unit | wall-clock на 1000 фраз | pytest-benchmark | **N/A** (scheduler ещё не в develop) |
| A4 | LLM видит блок `[CHANNELS]` в каждом ходе | L0+L1 | test_feedback_has_channels_block + stend_N1 | unit + stend | grep `[CHANNELS]` в feedback, queue_depth ≥ 0 | logs | **N/A** |
| A5 | Пост-амбл «Готово! Зачитал тебе...» не появляется при активном voice-канале | L1+L2 | stend_N4 + live_N4 | stend + live | TTS-чанки без trailing «готово», проверка regex `^готов` | wav+stt | **N/A** (бага в recovery не видно — barge-in быстрее режет) |
| A6 | Инвариант сегментов: `update()` модифицирует только PENDING | L0 | test_segment_invariant_update_only_pending | unit | pytest assert ACTIVE остался неизменным | pytest | **N/A** |
| A7 | `speak_text` НЕ делает `await tts/finished` (grep пуст) | L0 | test_speak_text_nonblocking | unit | `grep "await tts/finished" src/rob_box_voice/rob_box_voice/dialogue_node.py` → 0 строк | shell | **OPEN ISSUE** (восстановлен f3b58ef1, но e2e v38 показал 3 finished на 1 speech_id — см. CHILD_TASKS_PROPOSAL.md #0) |
| A8 | Два подряд `speak_text` возвращают за < 50мс каждый | L1 | stend_test_speak_text_double_burst | stend | wall-clock per call | pytest | **N/A** |

### 4.2 Фаза 1.5 — Acceptance tool-calling (15 acceptance-пунктов, §11.2)

| # | Acceptance-пункт | L | Test ID | Entry point | Метрика / threshold | Доказательство | Статус 06.08 |
|---|-------------------|---|---------|-------------|----------------------|----------------|---------------|
| A9 | `navigate_to_waypoint(кухня)` создаётся в AWAITING | L0 | test_navigate_creates_awaiting | unit | status == AWAITING_CONFIRMATION после submit | pytest | **N/A** |
| A10 | `stop_navigation` — сразу ACTIVE, минуя AWAITING (🟢) | L0+L2 | test_stop_nav_no_confirm + live_N3 | unit + live | status == ACTIVE за 1 hop; latency < 100мс | pytest+logs | **N/A** |
| A11 | `speak_text` — PENDING → ACTIVE без подтверждения (🟢) | L0 | test_speak_text_no_confirm | unit | status transitions PENDING→ACTIVE без AWAITING | pytest | **PASS IMPLICIT** (текущая поведение, см. recovery §5.2) |
| A12 | «да» → AWAITING → ACTIVE, таймер отменяется (сценарий 1) | L1 | stend_N1 | stend | timer отменён, status ACTIVE | state.json | **N/A** |
| A13 | «нет» → AWAITING → REJECTED + feedback `confirmation_rejected` (сценарий 8) | L1 | stend_N8 | stend | status REJECTED, в feedback есть блок | state.json+logs | **N/A** |
| A14 | Таймаут 20с → REJECTED + «отменяю» (сценарий 7) | L1 | stend_N7 | stend | elapsed ≈ 20с, status REJECTED, TTS-чанк «отменяю» в wav | state.json+wav | **N/A** |
| A15 | safe_boundary: ACTIVE-навигация + новый план → CANCELLED + новый AWAITING (сценарий 3) | L1 | stend_N2 | stend | CANCELLED в safe boundary, новый AWAITING | state.json | **N/A** |
| A16 | `stop_navigation` во время AWAITING исполняется мгновенно (сценарий 4) | L1+L2 | stend_N3 + live_N3 | stend + live | latency stop < 100мс, AWAITING → REJECTED | state.json+logs | **N/A** |
| A17 | Race-protection: ввод во время AWAITING → ответ на confirm (unit) | L0 | test_race_awaiting_input | unit | 1000 итераций: 100% CONFIRM | pytest (hypothesis) | **N/A** |
| A18 | Блок `[AWAITING]` в feedback (unit) | L0 | test_feedback_has_awaiting_block | unit | grep в formatter | pytest | **N/A** |
| A19 | Property-based: `stop_navigation` НИКОГДА не возвращает `requires_confirm=true` | L0 | test_property_stop_nav_never_confirm | unit | hypothesis 1000 inputs | pytest | **N/A** |
| A20 | Reflex-команды в `SchedulerEventBus` с правильным приоритетом | L0+L2 | test_reflex_eventbus_priority + live_N4 | unit + live | «стой» → priority=critical, «направо» → priority=high | logs | **N/A** |
| A21 | Reflex-«стой» отменяет ВСЕ ACTIVE за < 500мс (§8.10.6 #1) | L1+L2 | stend_N4 + live_N4 | stend + live | delta от cancel до all-CANCELLED < 500мс | state.json | **N/A** |
| A22 | Reflex-«налево» во время песни — параллельно (§8.10.6 #3) | L1+L2 | stend_N5 + live_N5 | stend + live | voice ACTIVE + nav ACTIVE одновременно ≥ 5с | state.json+wav | **N/A** |
| A23 | Reflex-«стой» во время AWAITING отменяет ожидание (фаза 1.5) | L1 | stend_N11 | stend | AWAITING → REJECTED + TTS «отменяю» | state.json+wav | **N/A** |

### 4.3 Фаза 2 — двухуровневое решение + SchedulerEventBus (10 acceptance-пунктов, §11.3)

| # | Acceptance-пункт | L | Test ID | Entry point | Метрика / threshold | Доказательство | Статус 06.08 |
|---|-------------------|---|---------|-------------|----------------------|----------------|---------------|
| A24 | Решение по новому вводу (любое из 5) < 800мс | L1 | stend_test_quick_decide_l2_latency | stend | wall-clock p95 | pytest | **N/A** |
| A25 | «и ещё про енота» во время куплета 1 → куплет 2 без паузы (N9) | L1+L2 | stend_N9 + live_N9 | stend + live | пауза между сегментами < 300мс | wav+spectrogram | **N/A** |
| A26 | `battery_critical` вплетается в PENDING без прерывания ACTIVE (N10) | L1 | stend_N10 | stend | ACTIVE voice не прервалось, новый сегмент в PENDING | state.json | **N/A** |
| A27 | Два MERGE подряд за < 2с | L0 | test_two_merges_under_2s | unit | wall-clock | pytest | **N/A** |
| A28 | Приоритеты: `obstacle` (critical) прерывает на границе такта, `battery` (high) вплетается | L1 | stend_test_event_priorities | stend | obstacle → ACTIVE CANCELLED на bar; battery → PENDING modified | state.json | **N/A** |
| A29 | e2e «батарея» без паузы и рестарта (N10) | L1+L2 | stend_N10 + live_N10 | stend + live | voice-канал ACTIVE непрерывно | wav+state.json | **N/A** |
| A30 | Авто-триггер: `battery_critical` без юзер-ввода провоцирует LLM-ход через scheduler | L1 | stend_test_auto_trigger | stend | LLM INPUT событие без STT | logs | **N/A** |
| A31 | Класс шины именован `SchedulerEventBus` (а не `EventBus` глобально) | L0 | test_eventbus_class_name | unit | `SchedulerEventBus.__name__ == "SchedulerEventBus"`, `grep "EventBus(" src/` пуст | shell | **N/A** |
| A32 | Никаких обходных импортов `rob_box_harness.side_effect_bus` в scheduler до фазы 5 | L0 | test_no_sideeffect_bus_imports | unit | `grep -r side_effect_bus src/rob_box_voice/rob_box_voice/scheduler/` пуст | shell | **N/A** |
| A33 | Двусторонняя интеграция `command_node` ↔ `SchedulerEventBus` (§8.10 полная) | L1+L2 | stend_test_command_node_integration + live_N4_log | stend + live | scheduler подписан на `/reflex/events`, публикует `task.cancelled(reason=reflex_stop)` | logs | **N/A** |

### 4.4 Фаза 3 — эстиматоры + speculative pre-generation (5 acceptance-пунктов, §11.4)

| # | Acceptance-пункт | L | Test ID | Entry point | Метрика / threshold | Доказательство | Статус 06.08 |
|---|-------------------|---|---------|-------------|----------------------|----------------|---------------|
| A34 | `SegmentEstimator`: точность ±15% | L0 | test_segment_estimator_accuracy | unit | predicted vs actual, abs error < 15% на 100 примерах | pytest | **N/A** |
| A35 | `LLMEstimator`: EMA обновляется | L0 | test_llm_estimator_ema | unit | после 5 ходов EMA видна в feedback | pytest | **N/A** |
| A36 | Пауза между сегментами при MERGE < 300мс | L1 | stend_test_merge_pause | stend | wav silence region | wav | **N/A** |
| A37 | При опоздании LLM — музыка/ambient продолжается | L1 | stend_test_fill_segment | stend | wav-нет тишины > 1с | wav | **N/A** |
| A38 | Пред-генерация N+1 отменяется при правке | L0 | test_pre_gen_cancel | unit | InterruptibleTask отменён, нет артефактов | pytest | **N/A** |

### 4.5 Итого по дизайну

- **41 acceptance-пункт** (8+15+10+5 + 3 кросс-фазных)
- **Распределение по уровням:** L0 = 18 (44%), L1 = 16 (39%), L2 = 16 (39%) — L0+L1+L2 комбинируются для 8 acceptance-пунктов
- **Покрытие текущего состояния (06.08):** 0 PASS, 0 FAIL на acceptance scheduler'а (он не реализован в develop), 1 PARTIAL на R4 (recovery показал TTS-only, не backing), 3 OPEN ISSUES (wake word drop, A7 finished-event, voice-action-server aiohttp)

## 5. Соответствие 8 e2e-прогонам 05.08 — матрица «verdict → acceptance»

> recovery t_4f546ead дал 8 verdict'ов (TIMEOUT_LLM, TRACK_silent, WAKEWORD_DROP, OK_TTS_only, OK_CHAT, OK и т.д.). Эта матрица показывает, какие acceptance-пункты SCHEDULER_DESIGN и dialog_node.md каждый сценарий покрывает / нарушает.

| # | Сценарий 05.08 | Verdict (recovery) | Acceptance-пункты нарушены | Acceptance-пункты OK | Уровень покрытия |
|---|----------------|--------------------|------------------------------|----------------------|------------------|
| 01 | «робот сыграй баха» | TIMEOUT_LLM (MiniMax 17с) | A1 (TRACK-канал FIFO), A6 (invariants — нет звука вообще) | — | **L2 R1** — тест на watchdog 131.8с + RMS отсутствие |
| 02 | «робот сыграй в траве сидел кузнечик» | TRACK_silent | A1, A36 (пауза = вся дорожка) | — | **L2 R2** |
| 03 | «робот зачитай рэп про енотика» | WAKEWORD_DROP (без «робот») | dialog_node §6.3 #3 (STT-wake coordination) | — | **L2 R3** — manual wake-fail-fast тест |
| 04 | «робот спой песенку про котика» | OK_TTS_only, НЕ backing | A2 (e2e v36 не падает — частично), parent-summary «50с backing» — **ОПРОВЕРГНУТО** (RMS=-24dB это TTS-хвост) | A11 (speak_text без confirm) | **L2 R4** — критично: добавить «RMS > -10dB на music-канале > 10с» |
| 05 | «робот будь диджеем таракан» | WAKEWORD_DROP | dialog_node §6.3 #3 | — | **L2 R5** |
| 06 | «робот говори по китайски» | OK_CHAT, barge-in 4/5 | A5 (barge-in storm — ассерт ≤ 1/5) | A11 | **L2 R6** |
| 07 | «робот будь лениным» | WAKEWORD_DROP | dialog_node §6.3 #3 | — | **L2 R7** |
| 08 | «робот хватит» | OK | A21 (reflex-cancel < 500мс) — НЕ ПРОВЕРЕНО (только 1 wake) | A11 | **L2 R8** — добавить «music_cleanup < 500мс» |

**Главные выводы матрицы:**

1. **8/8 прогонов 05.08 — это регресс-набор для scheduler'а.** Они НЕ покрывают acceptance-фазы 1.5/2/3 (нет «кухня?», «направо», «батарея», MERGE), но покрывают фазу 1 (TRACK-канал, FIFO, watchdog).
2. **Wake-word drop — это НЕ scheduler-bug.** Это dialog_node §6.3 #3 (STT-wake coordination), отдельный компонент. Scheduler на это не влияет; но дизайн e2e должен включать wake-word тесты (L0: «wake word ловит фразу без префикса» — это в dialog_node.md §8.2 P1, не в scheduler).
3. **LLM latency — НЕ scheduler-bug.** Это LLM-провайдер (MiniMax M2 thinking, 17с). Scheduler может маскировать (prefetch, MERGE без блокировки), но не устраняет. Дизайн e2e L2 это фиксирует через метрику `llm_response_total` (A3, A24).
4. **TRACK_silent — на стыке scheduler и Renardo.** Scheduler говорит «execute OK», Renardo создаёт паттерны в scsynth (recovery §5.1 NODE TREE 1089/1100/1106), но звук не идёт. Acceptance-пункт A1 («stop_music не доходит до железа раньше TTS») пройдёт, но acceptance-пункт «execute_music_code приводит к звуку на выходе» — **отсутствует в SCHEDULER_DESIGN** и должен быть добавлен (см. §7 «Открытые вопросы»).

## 6. Маппинг на dialog_node.md §6 «Test Coverage»

dialog_node.md §6.3 перечисляет 8 непокрытых критических путей. Эта таблица — какие из них покрывает наш дизайн:

| # | Untested Critical Path (dialog_node.md §6.3) | Покрытие в e2e-дизайне | L | Test ID |
|---|----------------------------------------------|-------------------------|---|---------|
| 1 | Dialogue state machine transitions | A6, A9, A10, A11, A12, A13, A14, A15, A16, A22, A23 (status transitions) | L0+L1 | test_segment_* + stend_N1-N11 |
| 2 | LLM provider fail-over (timeout → fallback) | A24 + новый **A39** (см. §7) | L1 | stend_test_llm_failover |
| 3 | STT wake word coordination | **A40** (новый — см. §7) | L0+L2 | test_wake_word + live_R3/R5/R7 |
| 4 | TTS chunk streaming with barge-in | A5, A22 (barge-in ≤ 1/5 chunks) | L1+L2 | stend_test_barge_in + live_R6 |
| 5 | Concurrent user inputs (race) | A17 (race AWAITING), A27 (два MERGE) | L0 | test_race_* |
| 6 | Redis session persistence | **A41** (новый — см. §7) | L1 | stend_test_session_persistence |
| 7 | Multi-modal context aggregation | **N/A** — вне scope scheduler'а | — | (вне scope) |
| 8 | Tool-call dispatch (mcp_server.py) | A9–A11, A19, A20, A31, A32 | L0 | test_tool_* |

**Итого:** 7 из 8 непокрытых путей dialog_node.md имеют покрытие в нашем e2e-дизайне. Один (multi-modal context aggregation) — вне scope scheduler'а и будет покрыт отдельным дизайном для context_aggregator_node.


## 7. Открытые вопросы и недостающие acceptance-пункты

При сверке матрицы с реальными прогонами 05.08 и SCHEDULER_DESIGN обнаружено 4 пробела, которые **нужно обсудить с PM/Author**.

### 7.1 Нет acceptance для «execute_music_code → звук на выходе»

SCHEDULER_DESIGN §11.1 A1 проверяет «stop_music не доходит до железа раньше TTS». Но **симметричного** пункта «execute_music_code ДОХОДИТ до железа и приводит к RMS > -10dB на music-канале в течение 5с» — нет.

**Проблема:** recovery t_4f546ead показал, что execute_music_code возвращает success за 87–125мс, паттерны создаются в scsynth (NODE TREE 1089/1100/1106), но RMS на .wav — 0. Это **главный симптом 5/8 прогонов 05.08** (R1, R2, R3, R5, R7 частично).

**Предложение:** добавить в §11.1 новый acceptance-пункт:

> **A39 (новый)**: `execute_music_code` приводит к RMS > -10dB на music-канале в течение 5с после submit.
> L1+L2: stend_test_music_output_present + live_R1/R2.
> Метрика: `volumedetect` на записанном .wav, RMS в окне 0–5с после submit > -10dB.
> Статус: **OPEN**, требует решения PM.

### 7.2 Нет acceptance для STT-wake coordination

dialog_node.md §6.3 #3 указывает «STT wake word coordination» как непокрытый. Scheduler не виноват, но e2e-дизайн должен включать.

**Проблема:** 3/8 прогонов 05.08 (R3, R5, R7) упали из-за wake word drop. Recovery t_4f546ead §3 подтвердил, что STT yandex ОК распознаёт фразу, но wake-word detector не пропускает в dialogue (без префикса «робот»).

**Предложение:** добавить новый acceptance-пункт:

> **A40 (новый, dialog_node-уровень)**: wake-word detector пропускает команду в dialogue_node, если STT confidence > 0.7 И текст содержит ключевые слова (зачитай, спой, едь, и т.д.), даже без префикса «робот».
> L0+L2: test_wake_word_no_prefix + live_R3/R5/R7.
> Метрика: ratio `wake_accepted / stt_ok` ≥ 0.95.
> Статус: **OPEN**, требует решения QA + owner dialog_node.

### 7.3 Нет acceptance для LLM fail-over

dialog_node.md §6.3 #2 указывает «LLM provider fail-over (timeout → fallback)». Если MiniMax M2 17с зависает (recovery показал: 17с для 01_baha, 70с для 02_kuznechik), что происходит? Graceful degradation или kill?

**Предложение:** добавить в §11.3 новый acceptance-пункт:

> **A41 (новый)**: принудительный timeout MiniMax API → fallback на локальную модель / кэшированный ответ / «ой, подвисло, повтори».
> L1: stend_test_llm_failover с `--llm-mock-timeout=30s`.
> Метрика: user-facing message в течение 35с от wake, без процесса-висяка.
> Статус: **OPEN**, требует решения architect + backend.

### 7.4 L2-стенд не имеет git-rev

В recovery t_4f546ead §7.2 уже отмечено: «непонятно, какой git rev был на роботе во время прогонов 05.08». Без этого результаты несравнимы.

**Предложение:** в L2 entry point добавить:

```bash
git_rev=$(sshpass -p open ssh ros2@10.1.1.21 \
  'docker exec voice-assistant bash -c "cd /ws && git rev-parse HEAD 2>/dev/null || echo unknown"')
echo "git_rev=$git_rev" >> $ARTIFACTS/state.json
```

Статус: **TODO**, мелкая правка скрипта.

## 8. Метрики и пороги (полный реестр)

> Каждая метрика имеет: имя, единицы, порог, способ измерения, источник данных.

| Метрика | Единицы | Порог | Способ | Источник |
|---------|---------|-------|--------|----------|
| `stt_to_wake` | мс | ≤ 50 (L2), ≤ 20 (L1) | delta от STT-ok до wake-detected | va.log timestamps |
| `wake_to_llm` | мс | ≤ 50 | delta от wake до LLM INPUT | va.log |
| `llm_response_total` | с | ≤ 30 (p95), без жёсткого max | delta от LLM INPUT до tool_call | va.log |
| `exec_round_trip` | мс | ≤ 500 | delta от tool_call submit до result | va.log |
| `tts_chunk_to_play` | мс | ≤ 200 | delta от speak_text publish до TTS play start | va.log + sc.log |
| `barge_in_ratio` | доля | ≤ 0.2 (1/5 chunks) | `barge_in_count / tts_chunks` | va.log |
| `music_rms_first_5s` | dB | > -10 (execute_music_code) | volumedetect на .wav, окно 0–5с после submit | wav |
| `music_rms_long_band` | dB | > -10 на music-канале > 10с | volumedetect, band > 10с | wav |
| `wake_acceptance_ratio` | доля | ≥ 0.95 | `wake_accepted / stt_ok` | va.log |
| `stop_music_latency` | мс | < 500 (reflex), < 100 (stop_navigation) | delta от cancel-event до all-CANCELLED | state.json |
| `awaiting_timeout` | с | 20 ± 1 | elapsed от AWAITING до REJECTED | state.json |
| `merge_pause` | мс | < 300 (фаза 3) | silence region в wav между сегментами | wav+spectrogram |
| `quick_decide_l1_latency` | мс | < 50 (p99) | wall-clock на 1000 фраз | pytest-benchmark |
| `quick_decide_l2_latency` | мс | < 800 (p95) | wall-clock на 100 фраз с mock-LLM | pytest-benchmark |
| `channel_fifo_order` | булеан | True (для всех операций) | enqueue ts ≤ dequeue ts в порядке submit | scheduler state |
| `coverage_scheduler` | % | ≥ 90 для `scheduler/` пакета | pytest --cov | CI |
| `coverage_dialogue` | % | ≥ 80 для `dialogue_node.py` (только стабильные части) | pytest --cov | CI |
| `e2e_total_runtime` | с | ≤ 30 мин на 13 сценариев L1 | wall-clock | CI |
| `e2e_live_total_runtime` | мин | ≤ 60 на 8 регресс + 5 новых | wall-clock | operator |

## 9. State snapshot — обязательный «срез состояния» перед каждым e2e

> Без snapshot результат несравним (принцип P4). Восстановлено из recovery t_4f546ead §1 + дополнено.

```bash
# tests/e2e/live/snapshot.sh
ARTIFACTS=$1  # например tests/e2e/_artifacts/live_20260806_120000/
mkdir -p $ARTIFACTS
ROBOT=10.1.1.21
SSHPASS=open

# 1. Робот: docker ps, uptime, git rev
sshpass -p $SSHPASS ssh ros2@$ROBOT 'docker ps --format "{{.Names}}\t{{.Status}}\t{{.Image}}"' \
  > $ARTIFACTS/docker_ps.tsv
sshpass -p $SSHPASS ssh ros2@$ROBOT 'uptime' > $ARTIFACTS/uptime.txt
sshpass -p $SSHPASS ssh ros2@$ROBOT \
  'docker exec voice-assistant bash -c "cd /ws && git rev-parse HEAD 2>/dev/null || echo unknown"' \
  > $ARTIFACTS/git_rev.txt

# 2. Билдовая машина: ls /tmp/dialog_e2e_*.wav, clock
sshpass -p $SSHPASS ssh builder@10.1.1.249 'ls -la /tmp/dialog_e2e_*.wav' \
  > $ARTIFACTS/build_wav_list.txt
sshpass -p $SSHPASS ssh builder@10.1.1.249 'date -Iseconds' > $ARTIFACTS/build_clock.txt

# 3. Локально: наш git rev, pytest version, python version
git rev-parse HEAD > $ARTIFACTS/local_git_rev.txt
pytest --version > $ARTIFACTS/pytest_version.txt 2>&1
python3 --version > $ARTIFACTS/python_version.txt

# 4. jack_lsp внутри supercollider (текущие подключения)
sshpass -p $SSHPASS ssh ros2@$ROBOT \
  'docker exec supercollider jack_lsp -c' > $ARTIFACTS/jack_lsp.txt 2>&1

# 5. Сводный json
python3 -c "
import json, datetime
state = {
  'snapshot_at': datetime.datetime.now().isoformat(),
  'robot': '$ROBOT',
  'docker_ps': open('$ARTIFACTS/docker_ps.tsv').read(),
  'uptime': open('$ARTIFACTS/uptime.txt').read().strip(),
  'git_rev_robot': open('$ARTIFACTS/git_rev.txt').read().strip(),
  'git_rev_local': open('$ARTIFACTS/local_git_rev.txt').read().strip(),
}
open('$ARTIFACTS/state_snapshot.json', 'w').write(json.dumps(state, indent=2, ensure_ascii=False))
"
```

**Результат:** 9 файлов в `$ARTIFACTS`, плюс `state_snapshot.json`. Это базовая линия для сравнения «было/стало».

## 10. Поэтапный план внедрения (для kanban-декомпозиции)

> Документ-результат, не код. Но дизайн подразумевает **последовательность задач** в kanban.

### Этап 0 — Подготовка (1–2 дня, owner: devops / backend)

- [ ] **T0.1** Восстановить `src/rob_box_voice/test/test_task_scheduler.py` (исходник утерян, есть .pyc). Из git history / переписать.
- [ ] **T0.2** Поднять docker-compose стенд `compose/stend/e2e.yaml` (voice-assistant + supercollider + mocks).
- [ ] **T0.3** Создать `tests/e2e/` скелет: `conftest.py`, `snapshot.sh`, `run_live_e2e.sh`, `ACCEPTANCE_MATRIX.csv` (генерируется из §4).
- [ ] **T0.4** Зафиксировать recovery-скрипт t_4f546ead в `scripts/recovery_collect_logs.sh` (для повторного использования).

### Этап 1 — L0-тесты (2–3 дня, owner: backend)

- [ ] **T1.1** Реализовать A6 (segment invariants), A3 (quick_decide latency), A9–A11 (status transitions), A19 (property-based stop_nav), A31/A32 (EventBus naming).
- [ ] **T1.2** Покрытие scheduler/ ≥ 90% в CI.
- [ ] **T1.3** Матрица `ACCEPTANCE_MATRIX.csv` обновляется автоматически из pytest results.

### Этап 2 — L1-стенд (3–4 дня, owner: backend + devops)

- [ ] **T2.1** Реализовать A12–A18, A21–A23, A24–A30, A33, A34–A38.
- [ ] **T2.2** Race-тесты: A17 (1000 итераций AWAITING+«да»), A27 (два MERGE за 2с).
- [ ] **T2.3** L1-стенд в CI: `pytest tests/e2e/stend/ -v` за ≤ 20 мин.

### Этап 3 — L2 live robot (2 дня, owner: backend + QA)

- [ ] **T3.1** Реализовать R1–R8 (регресс 05.08) + N1–N13 (новые сценарии).
- [ ] **T3.2** Скрипт `run_live_e2e.sh` + snapshot.sh.
- [ ] **T3.3** Cron раз в неделю (воскресенье 03:00 МСК), результат в Telegram "e2e-alerts".
- [ ] **T3.4** Ручной прогон перед каждым релизом scheduler'а в main.

### Этап 4 — Матрица как single source of truth (ongoing)

- [ ] **T4.1** `ACCEPTANCE_MATRIX.csv` живёт в git, обновляется при изменении SCHEDULER_DESIGN.
- [ ] **T4.2** Каждый PR с правкой scheduler'а ОБЯЗАН обновить соответствующие acceptance-строки (CI check: `python scripts/check_matrix_sync.py`).
- [ ] **T4.3** Раз в месяц — аудит «3 acceptance-пункта, которые больше не актуальны» (отдельная kanban-карточка).

## 11. Связь с текущими задачами и где этот дизайн помогает

| Задача | Где помогает наш дизайн |
|--------|--------------------------|
| `t_4f546ead` recovery (выполнен) | Acceptance-матрица §4 фиксирует, что recovery сделал, а что нет. 5 OPEN ISSUES из recovery → 4 в матрицу (A7, A39, A40, A41). |
| `t_5fb8a092` parent (выполнен) | §5 — маппинг 8 verdict'ов на acceptance. Parent-summary частично опровергнут (A1 «stop_music FIFO» — не про 50с backing, см. recovery §4 R4). |
| `t_30b17a23` SCHEDULER_REVIEW.md (выполнен) | §4 матрицы — все 41 acceptance-пункт SCHEDULER_DESIGN теперь имеют уровень тестирования. SCHEDULER_REVIEW.md фиксировал «8/8 acceptance не покрыто» — наш дизайн закрывает. |
| Issue #968 | §10 поэтапный план = roadmap для карточек #1-#4 из CHILD_TASKS_PROPOSAL.md + новые T0.x, T3.x. |
| Issue #935 (watchdog) | A1 + R1: явно тестируем watchdog 131.8с, как было 05.08. |
| Issue #993 (barge-in) | A5 + R6: barge-in ≤ 1/5 chunks. |
| voice-action-server aiohttp bug | **ВНЕ SCOPE** этого дизайна (отдельный компонент), но рекомендую в §7.1 — добавить проверку `docker ps` для всех контейнеров в snapshot. |

## 12. Что НЕ покрыто этим дизайном (явно)

1. **Multi-modal context aggregation** (dialog_node.md §6.3 #7) — это про context_aggregator_node, не про scheduler. Отдельный дизайн.
2. **TTS provider failover** (Silero → Edge → MiniMax) — есть в ADR-0003, но без acceptance-пунктов. **OPEN**: добавить в §7 как A42.
3. **Long-running stability (24ч uptime)** — это SRE-уровень, отдельный chaos-monkey дизайн.
4. **CI без робота (только docker-compose)** — L1-стенд покрывает, L2 — нет (нужен физический доступ).
5. **Безопасность (сетевые атаки, prompt injection)** — вне scope e2e-тестирования поведения.

## 13. Приложение A. Acceptance-матрица в CSV-формате (заготовка)

> Полная версия: `tests/e2e/ACCEPTANCE_MATRIX.csv`. Заголовок и 10 строк-примеров:

```csv
id,source_doc,section,text,level,test_id,entry_point,metric,threshold,evidence,status_2026-08-06,owner
A1,SCHEDULER_DESIGN.md,11.1,stop_music FIFO после TTS,L0+L2,test_channel_fifo+live_R1,unit+ssh,delta_enqueue_dequeue_ms,>tts_chunk_duration,logs+scsynth,FAIL,backend
A2,SCHEDULER_DESIGN.md,11.1,e2e v36 v38 pass,L2,live_R4,live,rms_music_band_dB,>-10 на >10с,wav,PARTIAL,backend
A3,SCHEDULER_DESIGN.md,11.1,L1 quick-decide latency,L0,test_quick_decide_l1_latency,unit,wall_clock_ms_p99,<50,pytest-benchmark,N/A,backend
A4,SCHEDULER_DESIGN.md,11.1,[CHANNELS] в feedback,L0+L1,test_feedback_has_channels+stend_N1,unit+stend,grep_in_feedback,True,logs,N/A,backend
A5,SCHEDULER_DESIGN.md,11.1,нет пост-амбла при voice,L1+L2,stend_N4+live_R6,stend+live,regex_tts_preamble,no_match,wav+stt,N/A,backend
A6,SCHEDULER_DESIGN.md,11.1,update() не трогает ACTIVE,L0,test_segment_invariant,unit,assert_unchanged,True,pytest,N/A,backend
A7,SCHEDULER_DESIGN.md,11.1,нет await tts/finished,L0,test_speak_text_nonblocking,unit,grep_count,0,shell,OPEN,backend
A8,SCHEDULER_DESIGN.md,11.1,двойной speak_text < 50мс,L1,stend_test_double_burst,stend,wall_clock_per_call_ms,<50,pytest,N/A,backend
A9,SCHEDULER_DESIGN.md,11.2,navigate_to_waypoint=AWAITING,L0,test_navigate_creates_awaiting,unit,status_eq,AWAITING_CONFIRMATION,pytest,N/A,backend
A10,SCHEDULER_DESIGN.md,11.2,stop_navigation=ACTIVE сразу,L0+L2,test_stop_nav_no_confirm+live_N3,unit+live,status_eq+latency_ms,ACTIVE; <100,pytest+logs,N/A,backend
```

(Полная матрица — 41 строка + 3 новых A39/A40/A41 = 44 строки. Генерация: `python scripts/render_matrix.py --out tests/e2e/ACCEPTANCE_MATRIX.csv`.)

## 14. Приложение B. Что нового по сравнению с существующими тестами

| Существующий тест | Что покрывает | Что НЕ покрывает | Где наш дизайн дополняет |
|-------------------|---------------|------------------|---------------------------|
| `tests/integration/test_e2e_harness_minimax.py` | Harness + MiniMax provider (request/response), TTS payload, auth probe | Race-условия scheduler'а, AWAITING, MERGE | L1-стенд в §3.2 |
| `src/rob_box_voice/test/test_dialogue_node.py` | Mock-based unit-тесты DialogueNode (state machine) | Реальные STT/wake/LLM/TTS, race-условия | L0 (scheduler invariants) + L2 (live) |
| `src/rob_box_voice/test/test_dialogue_shell.py` | ROS2 shell smoke | End-to-end pipeline | L1-стенд |
| `src/rob_box_voice/test/test_task_scheduler.py` (.pyc) | **Исходник потерян** — есть скомпилированный .pyc | — | T0.1 — восстановить |
| `tests/unit/core/test_tool_provider.py` | ToolProvider контракт | ToolConfirmationPolicy (scheduler-specific) | L0 A9–A11, A19 |
| `src/rob_box_voice/test/unit/tts/test_speech_id_arg_chain.py` | Один finished на speech_id | A7 acceptance (speak_text non-blocking) | L0 A7 |

**Главный пробел**: нет **связки** unit-тестов scheduler'а с реальным pipeline. Наш дизайн закрывает L1-стендом (3.2).

## 15. Приложение C. Список открытых архитектурных решений (для PM/Author)

> Эти вопросы нужно решить ДО начала Этапа 2 (L1-стенд).

1. **A39** — добавить acceptance «execute_music_code → RMS на выходе» в SCHEDULER_DESIGN §11.1? (см. §7.1)
2. **A40** — добавить acceptance «wake word без префикса» в dialog_node? (см. §7.2)
3. **A41** — добавить acceptance «LLM fail-over» в SCHEDULER_DESIGN §11.3? (см. §7.3)
4. **Q5** (из SCHEDULER_DESIGN §8.10.7) — «направо во время песни» параллельно или IGNORE? Без ответа нельзя L1-тест N5.
5. **Q7** (из §8.10.8) — reflex как ответ на confirm или отдельная операция? Без ответа нельзя L1-тест N11.
6. **Owner of `dialog_node.md` acceptance** — кому писать L0-тесты A40, dialog_node §6.3 #3? dialog_node owner или scheduler owner?
7. **Cron-расписание L2** — раз в неделю (воскресенье 03:00) — ок? Или каждый push в feature/scheduler-*?

## 16. Глоссарий

| Термин | Значение |
|--------|----------|
| **L0 / L1 / L2** | Уровни тестирования: unit hermetic / stend e2e / live robot |
| **Acceptance-пункт** | Проверяемое свойство из SCHEDULER_DESIGN §11 |
| **Вердикт сценария** | PASS / FAIL / PARTIAL / N/A — итог прогона одного сценария |
| **State snapshot** | Срез состояния робота + стенда перед e2e (P4) |
| **Scheduler-канал** | FIFO-очередь voice/music/anim/nav в `TaskScheduler` |
| **AWAITING_CONFIRMATION** | Статус сегмента, ожидающий «да/нет» от пользователя (§8 SCHEDULER_DESIGN) |
| **Reflex-слой** | Прямые команды без LLM (regex → command_node) — §8.10 |
| **Recovery** | Ручной сбор логов с живого робота (SSH + docker logs + volumedetect) |

## 17. Ссылки

- `docs/design/SCHEDULER_DESIGN.md` v5 (1797 строк, commit 5cd5445a, 04.08.2026) — основной дизайн
- `docs/design/SCHILD_TASKS_PROPOSAL.md` — 8-card split issue #968 (зафиксированный план разработки)
- `docs/design/SCHEDULER_REVIEW.md` (68 строк) — ревью этого дизайна
- `analysis/dialog_node.md` (601 строка, 17.07.2026) — реверс-инжиниринг dialogue_node
- `analysis/nodes-current-state.md` (49766 байт) — общий обзор нод
- `tests/integration/test_e2e_harness_minimax.py` — шаблон e2e-теста
- `src/rob_box_voice/test/test_dialogue_node.py` — unit-тесты диалоговой ноды
- Parent-task `t_5fb8a092` / `SUMMARY_8runs.json` — 8 прогонов 05.08
- Recovery `t_4f546ead` / `RECOVERY_REPORT.md` — реальные логи 06.08 МСК с робота
- Issue #968 — основная проблематика scheduler'а
- Issue #935 — watchdog timeout (auto-stop 300с)
- Issue #993 — barge-in агрессивный

---

**Конец документа.** Версия v1, готова к ревью PM/QA. Следующая ревизия — после ответа на §15 Q1–Q7.
