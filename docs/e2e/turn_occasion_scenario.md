# E2E-сценарий «Денис заходит в мастерскую» — спецификация

> **Статус:** draft / не активный. Спецификация привязана к ADR-0102 §9.6
> («e2e-маркер, после PR-A + PR-F, не блокирует PR-A»). До merge PR-A
> (каркас `core/occasion.py`, ADR-0102 §8) и PR-F (тестовый адаптер
> `_on_meeting_signal`) — **запускать не на чем**: `OccasionGate` ещё
> не существует в коде (подтверждено `git grep "OccasionGate\|may_speak"
> src/` → 0 совпадений, 2026-09-15 13:55 CEST).
>
> **Автор:** tester (kanban t_7f1a4919, parent t_38c55959).
> **Родительский ADR:** ADR-0101 (`e7c9abc6`), ADR-0102 (PR #2581, draft).
> **Issue:** #2536.

---

## 0. Назначение документа

Спецификация e2e-теста, который подтверждает, что сценарий
«Денис входит в мастерскую → робот говорит первым» работает **целиком,
на живом роботе**, через тот же путь, что пользовательский wake-word.

Цель — не просто «текст сказан», а наблюдаемая цепочка
«perception-событие → конструкция `Occasion(MEETING, ...)` →
`may_speak` → `Verdict.ALLOW` → `_dispatch_turn(is_synthetic=True,
occasion=...)` → `_run_turn` → TTS-реплика «о, привет, Денчик…»
→ NO wake-word, NO новый STT-текст, NO инкремент
`_llm_skipped_counter["no_wake_word"]`».

---

## 1. Предусловия (pre-conditions)

### 1.1. Код

|| Артефакт | Где проверять | Статус на 2026-09-15 |
|---|---|---|---|
| ADR-0101 смержен | `git merge-base --is-ancestor e7c9abc6 HEAD` → exit 0 | ✅ merged 2026-09-15 13:37 |
| ADR-0102 принят | `docs/adr/0102-turn-occasion-api-contract.md` в `develop` | ⏳ PR #2581 OPEN |
| `core/occasion.py` существует | `git ls-files src/rob_box_voice/rob_box_voice/core/occasion.py` | ❌ нет (после PR-A) |
| `OccasionGate.may_speak` импортируется | `python -c "from rob_box_voice.core.occasion import OccasionGate"` | ❌ нет (после PR-A) |
| `EventDetector` имеет прода-импортёра | `git grep "from rob_box_perception.core.event_detector import EventDetector" -- src/ \| wc -l` → ≥2 (тест + `core/occasion.py`) | ⏳ (после PR-A, ADR-0101 §6.9) |
| `_on_meeting_signal` callback существует | `git grep "_on_meeting_signal" src/rob_box_voice/rob_box_voice/dialogue_node.py` → ≥1 | ❌ нет (PR-F) |
| `_dispatch_turn(..., occasion=...)` kwarg | `git grep "def _dispatch_turn" src/rob_box_voice/rob_box_voice/dialogue_node.py` → сигнатура с `occasion` | ❌ нет (после PR-A, ADR-0101 §3.3.2) |

**Критерий PASS для запуска теста:** все строки таблицы = ✅.
Сейчас (2026-09-15) — 1 ✅, 6 ⏳ / ❌. Тест **не активен**.

### 1.2. Конфигурация

|| Параметр | Где | Значение |
|---|---|---|
| `wake_words.yaml` в `personality` содержит «роб бокс» / «робот» | `docker/vision/config/wake_words.yaml` | ✅ (подтверждено, файл:1-50) |
| `dialogue_node.yaml.startup_greeting_text` пустая или дефолтная | `docker/vision/config/voice_assistant/dialogue_node.yaml:132-134` | "Я на связи, все системы в норме!" (не пустая) — это **неважно** для сценария, но нужно знать: после деплоя прозвучит startup-реплика ДО теста, ждать её окончания |
| `vision` реальный inference (не StubHEFLoader) | #2398 / #2531 (Взгляд) | ⏳ (PR-F требует реального кадра, иначе стаб-фильтр REFUSE) |
| `stt_node` не в degraded mode (cold-start VAD) | ADR-0029 | пройти ≥5 минут от старта контейнера до теста |
| `ROS_DOMAIN_ID` совпадает у робота и harness | `docker compose ps` | проверить |

### 1.3. Состояние ROS2-графа (обязательно для запуска)

```bash
# Проверить, что dialogue_node жив и подписан:
ros2 node info /dialogue_node 2>&1 | grep -E "Publishers|Subscriptions"
# Ожидаемо в подписках (после PR-F):
#   /vision/hailo/events          (sub)  ← новое в PR-F
#   /perception/context_update    (sub)  ← новое в PR-A
#   /voice/stt/result             (sub)  (existing)
#   /voice/dj_mode                (sub)  (existing)
#   ... (остальные 16 — без изменений)

# Проверить, что vision_hailo_node публикует:
ros2 topic info /vision/hailo/events -v 2>&1 | grep -E "Publisher|Message type"
# Ожидаемо: Publisher count ≥ 1, тип rob_box_perception_msgs/VisionEvent

# Проверить, что perception кэш живой (после PR-A):
ros2 service call /voice/occasion_stats ... # будет введён ADR-0102 §5
# Или через лог: grep "occasion.*stats" /var/log/rob_box/dialogue_node.log
```

### 1.4. Железо

- Робот включён, камера (ReSpeaker AI HAT+ / OAK-D) реально видит сцену.
- В кадре **только один** человек (тестовый актор = Денис, см. §6).
- Динамик и микрофон ReSpeaker не заглушены, `VAD max ≥ 12.0` (иначе реплика робота обрежется, см. `.github/e2e/VOICE_COMMANDS_RESEARCH.md`).

---

## 2. Шаги сценария

### 2.1. Подача perception-события «вижу Дениса»

**Способ 1 (PR-F, тестовый адаптер, основной):**

После PR-F в `dialogue_node` появится режим `test_meeting_inject`, который
позволяет **синтетически** опубликовать `VisionEvent` через CLI или
ROS2-сервис (а не ждать живого человека в кадре). Это важно для CI:
живой человек в кадре в production-тесте недопустим (детерминизм,
этика, скорость).

```bash
# Команда для harness (после PR-F):
ros2 topic pub --once /vision/hailo/events rob_box_perception_msgs/VisionEvent \
  "{header: {stamp: {sec: $(date +%s), nanosec: 0}, frame_id: 'realsense_front'},
   source_camera: 'realsense_front', event_type: 'person',
   class_name: 'person', confidence: 0.94,
   bbox_center_x: 320, bbox_center_y: 240, bbox_width: 200, bbox_height: 400,
   embedding_id: 'denis_8cb84bc9', display_name: 'Денис',
   attributes_json: '{}'}" \
  --qos-durability transient_local --qos-reliability reliable
```

**Способ 2 (живой человек в кадре, ручной тест на железе):**

Денис входит в кадр реально → `vision_hailo_node` с периодом ~250 ms
публикует `VisionEvent` с `event_type='person'`,
`source_camera='realsense_front'`, `embedding_id='denis_8cb84bc9'`,
`display_name='Денис'`, `confidence ≥ 0.85`. Это проверяет, что
**реальный** inference, а не stub. Для CI Способ 2 запрещён.

### 2.2. Ожидаемые потоки данных (по ADR-0102 §4)

```mermaid
sequenceDiagram
    participant V as vision_hailo_node
    participant CA as context_aggregator_node
    participant DN as dialogue_node
    participant OG as OccasionGate
    participant DT as _dispatch_turn
    participant TTS as tts_node

    V->>CA: /vision/hailo/events (VisionEvent)
    V->>DN: /vision/hailo/events → _on_meeting_signal
    Note over DN: шов РЕШЕНИЕ (ADR-0101 §3.1 п.6)
    DN->>DN: occasion = Occasion(MEETING, payload={...})
    DN->>OG: may_speak(occasion)
    OG->>OG: stub_filter: event_type=person, source_camera=realsense_front → PASS
    OG->>OG: per_source_cooldown: meeting ≥ N минут → PASS
    OG->>OG: global_debounce ≥ 2.0 с с прошлого ALLOW → PASS
    OG-->>DN: Verdict(ALLOW, reason="person_recognized denis")
    DN->>DT: _dispatch_turn(text="", is_synthetic=True, occasion=occasion)
    DT->>DN: _run_turn (промпт с perception_cache)
    DN->>TTS: /voice/tts/say "о, привет, Денчик..."
    TTS-->>DN: /voice/tts/finished
```

### 2.3. Что НЕ должно произойти (regression checks)

- **NO wake-word**: STT не должен слушать фразу робота «о, привет…»
  как вход — проверяется тем, что у робота **нет** свежего
  `/voice/stt/result` между шагами 2.1 и 2.4.
- **NO инкремент `_llm_skipped_counter["no_wake_word"]`**: ключевая
  post-condition брифа. Проверяется в логе:
  `grep "_llm_skipped_counter\[" /var/log/rob_box/dialogue_node.log`
  → 0 совпадений с момента подачи события.
- **NO новый ROS2-топик** `/voice/turn_occasion` — ADR-0102 §9.2.
  Проверяется: `ros2 topic list \| grep turn_occasion` → пусто.
- **NO `/voice/occasion/*` топиков** — ADR-0102 §9.2.
  Проверяется: `ros2 topic list \| grep occasion` → пусто.

### 2.4. Ожидаемая реплика робота

TTS должен произнести фразу вида:

```
о, привет, Денчик, как дела
```

(точная формулировка зависит от LLM-промпта после PR-F и от того,
что LLM видела в `perception_cache` — `current_time_human`,
`time_period`, `memory_summary`.)

**Не должны звучать:** wake-word «робот» в начале фразы робота
(это реплика робота, не пользовательская), никакая
`startup_greeting_text` («Я на связи, все системы в норме!» — этот
флаг уже отстрелян до теста, иначе отрабатывает один раз за uptime
и сбивает ожидания, см. ADR-0101 §1.2 п.4).

---

## 3. Постусловия (post-conditions)

| ID | Условие | Как проверить |
|---|---|---|
| P1 | `_llm_skipped_counter` НЕ инкрементировался (любой ключ) | `grep "skip summary\|_llm_skipped_counter" /var/log/rob_box/dialogue_node.log \| tail -5` → не менялось |
| P2 | `Verdict.reason == "person_recognized"` (или эквивалентное строковое значение, которое разработчик PR-F фиксирует) | `grep "occasion.*ALLOW\|occasion.*reason" /var/log/rob_box/dialogue_node.log` |
| P3 | `Occasion.kind == "meeting"` (или `SourceKind.MEETING`) | лог-уровенька ROS2 |
| P4 | `_run_turn` вызван с `is_synthetic=True` И `occasion=<Occasion(MEETING)>` | `grep "_run_turn\|_dispatch_turn" /var/log/rob_box/dialogue_node.log` (после PR-A добавляется kw `occasion`) |
| P5 | Никаких новых топиков `/voice/turn_occasion*`, `/voice/occasion*` | `ros2 topic list` (см. §2.3) |
| P6 | TTS реплика произнесена и не была обрезана | recording.wav из e2e-recorder, длительность ≥ 1.5с, RMS ≥ −30dB |
| P7 | `perception_cache` обновлён (поле `display_name='Денис'`) | ros2 service /voice/perception_cache_dump (если будет введён) или лог-уровенька |
| P8 | `EventDetector.mark_event_reacted` вызван хотя бы один раз | `git grep` в `core/occasion.py` подтверждает вызов; runtime — через `stats()` |

### 3.1. Замечание про `TurnOccasion.reason == "person_recognized"`

**Бриф карточки** формулирует: «`TurnOccasion.reason` равен
`person_recognized`». Это **устаревшая формулировка**, дочерняя от
ранней версии брифа (когда `TurnOccasion` ещё планировался как
ROS-сообщение). После принятия ADR-0101 и ADR-0102 §3.3 истинный
контракт:

- `Occasion` — Python-объект, не `.msg`.
- Решение выдаёт `Verdict(kind=ALLOW, reason=<string>)`, а не
  `TurnOccasion.reason`.
- Значение `reason="person_recognized"` — **соглашение implementation
  detail** для PR-F; разработчик backend-карточки PR-F должен
  зафиксировать это строковое значение в коде
  (`src/rob_box_voice/rob_box_voice/core/occasion.py`).
- Этот сценарий требует, чтобы **строка была именно
  `"person_recognized"`** (для grep-теста), а не `"meeting"`,
  `"person"`, `"person_seen"`. **Уточнение для PR-F:** строковое
  значение `reason` должно быть стабильным и предсказуемым
  (semver-уровень), чтобы внешние тесты (этот сценарий,
  существующие мониторинги по `/rosout`) могли на него полагаться.

---

## 4. Привязка к фиче-флагам ADR

| § ADR | Что фиксирует | Где в этом сценарии |
|---|---|---|
| ADR-0101 §3.1 | API `OccasionGate.may_speak` | §2.2 шаг 3-7 |
| ADR-0101 §3.2 | `EventDetector` per-source cooldown | §2.2 шаг 5 |
| ADR-0101 §3.3.1 | wake-word байт-в-байт эквивалентен | §2.3 (NO regression) |
| ADR-0101 §3.3.2 | `occasion=...` kwarg в `_dispatch_turn` | §3 P4 |
| ADR-0101 §3.3.3 | `_startup_greeting_fired` остаётся (PR-B) | §2.4 (не путать с тестом) |
| ADR-0101 §3.3.5 | DJ-тик не меняется в PR-A | §2.3 (другой источник повода) |
| ADR-0101 §3.5 п.4 | миграция `_startup_greeting_fired` в PR-B | §2.4 замечание |
| ADR-0101 §3.5 | стаб-фильтр для vision | §2.2 шаг 4, P7 |
| ADR-0102 §3.2 | почему `/voice/turn_occasion` НЕ используется | §2.3, §3 P5 |
| ADR-0102 §3.3 | контракт `Occasion/Verdict/SourceKind` | §2.2, §3 |
| ADR-0102 §3.4 | разделение шва данных vs решения | §2.2 (две подписки на `/vision/hailo/events` не нужны: одна для данных, одна для решения — `_on_meeting_signal` обрабатывает события, `_on_perception_event` обновляет кэш) |
| ADR-0102 §3.5 | `is_synthetic=True + occasion=Occasion(MEETING)` валидно | §3 P4 |
| ADR-0102 §9.1 | никаких `TurnOccasion.msg` | §3 P5 |
| ADR-0102 §9.2 | никаких топиков `/voice/turn_occasion` | §2.3, §3 P5 |
| ADR-0102 §9.3 | in-process API | §2.2 (callback → `may_speak` synchronous) |
| ADR-0102 §9.4 | регрессия на wake-word | §2.3 P1 |
| ADR-0102 §9.5 | `EventDetector` имеет прода-импортёра | §1.1, §3 P8 |
| ADR-0102 §9.6 | этот сценарий = e2e-маркер | весь документ |

---

## 5. JSON-сценарий для e2e-харнесса

После того как PR-F откроет `test_meeting_inject`-режим, этот сценарий
может быть положен в `.github/e2e/scenarios/turn_occasion_meeting_v1.json`:

```json
{
  "name": "turn_occasion_meeting_v1",
  "stability": "expected-red-until-pr-f",
  "_comment": [
    "Сценарий зависит от PR-A (каркас core/occasion.py) + PR-F (_on_meeting_signal, test_meeting_inject).",
    "До merge обоих — статус expected-red. После — этот сценарий становится acceptance-критерием для #2536.",
    "НЕ ставить в блокирующий gate CI, пока PR-A не в develop.",
    "Автор: tester (kanban t_7f1a4919, ADR-0102 §9.6)."
  ],
  "description": "Воспроизводит сценарий «Денис входит в мастерскую → робот говорит первым» через тестовый адаптер.",
  "preconditions": {
    "code": [
      "git ls-files src/rob_box_voice/rob_box_voice/core/occasion.py",
      "git grep -c 'from rob_box_perception.core.event_detector import EventDetector' -- src/ | sort  # ≥ 2"
    ],
    "config": [
      "docker/vision/config/wake_words.yaml содержит personality с 'роб бокс'",
      "ROS_DOMAIN_ID совпадает у робота и harness"
    ],
    "runtime": [
      "ros2 node info /dialogue_node  # подписан на /vision/hailo/events",
      "ros2 topic info /vision/hailo/events -v  # Publisher count ≥ 1, тип rob_box_perception_msgs/VisionEvent",
      "≥5 минут от старта dialogue_node (cold-start VAD, ADR-0029)"
    ]
  },
  "steps": [
    {
      "label": "to01_inject_meeting_event",
      "action": "ros2_topic_pub",
      "topic": "/vision/hailo/events",
      "msg_type": "rob_box_perception_msgs/VisionEvent",
      "payload": {
        "source_camera": "realsense_front",
        "event_type": "person",
        "class_name": "person",
        "confidence": 0.94,
        "embedding_id": "denis_test_inject",
        "display_name": "Денис"
      },
      "qos": {"durability": "transient_local", "reliability": "reliable"},
      "wait_after_s": 3.0,
      "_comment": "Тестовый адаптер PR-F. Без него — живой человек в кадре (запрещено в CI)."
    },
    {
      "label": "to02_expect_occasion_log",
      "wait_for_log_patterns": [
        "occasion.*ALLOW.*reason=person_recognized",
        "Occasion.*MEETING.*display_name=Денис"
      ],
      "wait_for_log_timeout_s": 5.0,
      "_comment": "Проверка, что _on_meeting_signal построил Occasion и may_speak вернул ALLOW."
    },
    {
      "label": "to03_expect_dispatch_turn_synthetic",
      "wait_for_log_patterns": [
        "_dispatch_turn.*is_synthetic=True.*occasion=Occasion.*MEETING"
      ],
      "wait_for_log_timeout_s": 3.0
    },
    {
      "label": "to04_expect_tts_phrase",
      "wait_for_log_patterns": [
        "/voice/tts/say.*привет.*Денчик"
      ],
      "wait_for_log_timeout_s": 15.0,
      "_comment": "Точная фраза зависит от LLM-промпта после PR-F. Паттерн ‘привет.*Денчик’ допускает вариации."
    },
    {
      "label": "to05_expect_no_llm_skipped_increment",
      "wait_for_log_negative_patterns": [
        "_llm_skipped_counter.*no_wake_word.*\\+1",
        "skip summary.*no_wake_word.*≥1"
      ],
      "wait_for_log_timeout_s": 3.0,
      "_comment": "Ключевая post-condition брифа."
    }
  ],
  "postconditions": {
    "ros2_topics_check": [
      "ros2 topic list | grep turn_occasion  # должен быть пустым (ADR-0102 §9.2)"
    ],
    "llm_skipped_counter_delta": 0,
    "verdict_reason_expected": "person_recognized",
    "tts_recording_min_duration_s": 1.5,
    "tts_recording_min_rms_db": -30
  }
}
```

См. также (для сравнения формата, но НЕ копировать слепо):
`.github/e2e/scenarios/dialogue_gap_probe_v1.json` + acceptance-вариант
`.github/e2e/scenarios/dialogue_gap_probe_acceptance_v1.json` —
формат полей и соглашение о `_comment` стабильны.

---

## 6. Тестовый актор «Денис»

Используется `embedding_id='denis_test_inject'` (синтетический, НЕ
`denis_8cb84bc9` — настоящий id зарегистрированного человека, чтобы
не путать acceptance с реальной идентификацией).

В живой e2e-сессии на железе (вне CI) можно использовать настоящий
`denis_8cb84bc9`, но **товарищ Шифу должен дать согласие на
эксперимент со своим профилем**, иначе — синтетический
`denis_test_inject`.

---

## 7. Что делать после merge PR-F

1. Довести `expected-red-until-pr-f` до `expected-green-after-pr-f` в JSON.
2. Прогнать сценарий на железе (run в `e2e-voice-test` workflow).
3. Прикрепить артефакт `e2e-voice-recording-<run_id>` к issue #2536.
4. Снять лейбл `needs-e2e` с issue #2536 (через бота agent-flow-merge-gate
   или вручную товарищем Шифу).

---

## 8. Что НЕ делает этот сценарий

- НЕ тестирует **другие** источники повода: wake-word (уже покрыто
  ADR-0101 §6.1), DJ-tick (уже покрыто отдельной карточкой, ADR-0101
  §3.3.5), startup (PR-B, отдельная карточка). Каждый источник =
  отдельный сценарий.
- НЕ тестирует LLM-качество реплики (LLM может сказать «привет,
  незнакомец» вместо «Денчик» — это не failure этого сценария,
  это другой gate).
- НЕ тестирует vision inference (стаб vs real) — это §1.1
  предусловие, не сам тест. Стаб-фильтр проверяется unit-тестом
  ADR-0101 §6.5.
- НЕ тестирует scenario «двое в кадре» (только один человек) —
  ADR-0101 §3.5 п.1 допускает несколько людей, но это **отдельная**
  проблема (multiface → несколько `embedding_id` → какой выбрать?),
  выходит за scope #2536.

---

## 9. История

| Дата | Событие |
|---|---|
| 2026-09-15 | draft создан (tester, kanban t_7f1a4919, parent t_38c55959) |
| ⏳ | активация после PR-A + PR-F |