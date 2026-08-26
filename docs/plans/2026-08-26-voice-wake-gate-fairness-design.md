# Voice Wake-Gate Fairness + Backlog Watchdog (issue #1668)

## Контекст

В комнате робота 10.1.1.21 непрерывно играет чужой голос — обзоры
мини-кухонь, лестниц, резисторов RX24, ремонт квартир. STT корректно
распознаёт всё это в `no_wake_word` backlog (≈16 фраз/мин) и кладёт в
`:class:`SpeechAccumulator` для возможного восстановления запроса. Когда
e2e проигрывает синтез-команду «Робот, ...», wake-gate на равных
обрабатывает и фоновый голос, и синтез → синтез теряется (16 e2e раундов
FAIL подряд, см. `docs/adr/0027-systemic-wake-gate-no-wake-word-blocker.md`).

Нам нужна **fairness**: синтез-команды должны проходить через wake-gate
даже при непрерывном фоновом шуме. Без ухудшения UX (без DJ-mode, без
ручного mute).

## Что делаем

### 1. Backlog pressure metric (`/diagnostics/backlog_overflow`)

Новый pure-Python модуль
`src/rob_box_voice/rob_box_voice/core/backlog_pressure.py`:

* `:class:`BacklogPressure` — sliding-window tracker `no_wake_word` событий.
* Считает `events_per_minute` = `events / (window_sec/60)`.
* Классифицирует уровень:
  * `LOW` — `< backlog_pressure_low_above` (дефолт 5/мин).
  * `ELEVATED` — `< backlog_pressure_high_above` (дефолт 10/мин).
  * `HIGH` — `>= backlog_pressure_high_above`.
* Snapshot публикуется в ROS2 topic `/diagnostics/backlog_overflow`
  (JSON-строка в `std_msgs/String`) не чаще 1 Hz.

### 2. Wake-gate fairness gate

В `DialogueNode._on_stt`, ветка `no_wake_word`:

* **До** добавления в `SpeechAccumulator` — спрашиваем у `BacklogPressure`,
  можно ли публиковать.
* При `HIGH` — **подавляем** `accumulator.add`, но:
  * инкрементируем `_llm_skipped_counter["no_wake_word"]` (телеметрия);
  * логируем причину явно: `🔇 [diagnostics] ignored: no_wake_word
    (backlog_pressure HIGH, events_per_minute=16.3)`;
  * **НЕ отключаем** STT / wake-gate / микрофон — фраза всё равно
    распознаётся и видна в логе.
* При `LOW` / `ELEVATED` — поведение прежнее (копим в backlog).
* Если `backlog_pressure_enabled=false` (или init упал с `ValueError`) —
  поведение прежнее (всегда публикуем в backlog). Это backward-compat.

### 3. Self-test / healthcheck (`/voice/diagnostics/quiet`)

Новый ROS2 topic `std_msgs/Bool`:

* `True` — робот в тишине (за последнюю минуту ≤1 события, и rate < low_above).
* `False` — робот зашумлён.

Публикуется **только при смене** состояния (нет spam'а каждую секунду).
Использование из e2e pre-flight: подписаться на `/voice/diagnostics/quiet`
и подождать `True` перед проигрыванием синтез-команды. Если `False` —
fail-fast (это уже отдельная карточка `t_67394082` для devops).

## Конфиг (`dialogue_node.yaml`)

| Параметр | Дефолт | Смысл |
|---|---|---|
| `backlog_pressure_enabled` | `true` | вкл/выкл watchdog |
| `backlog_pressure_window_sec` | `60.0` | скользящее окно, сек |
| `backlog_pressure_low_above` | `5.0` | events/min для ELEVATED |
| `backlog_pressure_high_above` | `10.0` | events/min для HIGH |
| `backlog_pressure_publish_interval_sec` | `1.0` | min интервал публикации |

`window_sec=60` даёт events/min напрямую (без нормализации). При
`window_sec=30` и 16 фраз/мин наблюдаемая метрика = `16 * 60/30 = 32/мин`.

## Acceptance criteria

| Критерий | Как проверяем |
|---|---|
| Unit-тесты на `BacklogPressure` показывают fairness при 100% заполнении backlog | `test/unit/core/test_backlog_pressure.py::test_fairness_high_pressure` |
| Integration-тест: 1 мин непрерывного фонового шума (mock) → синтезированная wake-word команда после этого проходит ≤5с | `test/unit/node/test_wake_gate_fairness.py::test_wake_word_passes_under_saturation` |
| `/diagnostics/backlog_overflow` публикуется не чаще 1 Hz | `test/unit/core/test_backlog_pressure.py::test_publish_interval` |

## Граничные случаи

* `accumulate_no_wake_enabled=false` — watchdog работает только когда
  бэклог-аккумулятор включён; иначе и watchdog бессмысленен (нечего
  подавлять).
* Telegram-вход (`[TG:...]`) — wake-gate пропускается, watchdog не
  срабатывает (ветка не доходит до `record()`).
* SILENCED state — watchdog не срабатывает (ветка возвращается до
  wake-gate).
* `ValueError` при init (невалидный конфиг) — watchdog отключается,
  пишется warning, поведение прежнее.

## Что НЕ делаем (по требованию issue #1668)

* НЕ отключаем STT / wake-gate / микрофон.
* НЕ удаляем `no_wake_word` backlog полностью — только подавляем
  публикацию при HIGH давлении.
* НЕ меняем AEC / audio pipeline (другая задача, отдельная карточка).
* НЕ закрываем DJ-mode issue #1629 (другая корневая).

## Связанное

* Issue #1668 (root) — STT-регрессия, 16 e2e раундов FAIL.
* Issue #1629 — DJ-mode (другая корневая).
* Issue #1605 — Avatar mixed-mode.
* ADR-0027 — systemic wake-gate no_wake_word blocker (observed behavior).
* ADR-0029 — cold-start known-state (временный fail-streak после merge).
* `t_67394082` — e2e pre-flight fail-fast (devops, отдельная карточка).
* `t_6e587508` — ретро этого фикса (parent, done).
