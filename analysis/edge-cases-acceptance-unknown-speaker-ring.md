# Edge-кейсы и acceptance criteria: session-scoped ring embeddings (t_8909b73f)

**Источник задачи:** Kanban t_8909b73f (задача 2 в decompositon цепочке t_f824ec16).
**Входы:** разведка (t_fd94dc4f) + дизайн ring (t_7097619c) + ADR-черновик (t_00ce84f2).
**Назначение:** QA-ориентированная спецификация для последующей реализации и тестирования
in-memory ring-буфера эмбеддингов **неизвестных** голосов и контракта метки `/voice/speaker/result`.

> **Важно:** это **спецификация**, а не реализация. Кода в PR не добавляется — только
> этот документ + sanity-тесты для самого документа. Реализация ring будет
> отдельной задачей в ветке реализации (после того, как t_f824ec16 будет
> зафиксирован в ADR).

---

## 1. Контекст: что уже есть и что меняется

**Сейчас (origin/develop, HEAD 19679673):**

- Файл: `src/rob_box_voice/rob_box_voice/speaker_id_node.py`, метод
  `_publish_result` (~L729–751).
- Топик `/voice/speaker/result` (String, JSON):
  - known-спикер: `{"is_known":true, "speaker_id":"<uuid>", "name":..., "confidence":..., "epithet":...}`
  - unknown-спикер: `{"is_known":false}` — и **никаких других полей**.
- Внутри ноды уже есть `_recent_embeddings: Deque[Tuple[float, np.ndarray]]`
  (maxlen=20, **L113**) — это **НЕ ring unknown-голосов**, а пул «последних
  эмбеддингов для pending register». Новый ring — отдельная сущность.
- В файле `core/epithets.py` есть словарь эпитетов и константы причин
  (`REASON_FIRST_SEEN`, `REASON_NEW_TOPIC`, `REASON_USER_OVERRIDE`,
  `REASON_LLM`) — эпитет всегда привязан к **зарегистрированному**
  speaker_id, поэтому для **unknown** ring придётся использовать
  `transient_label` (новое поле), а не epithet-словарь.

**Меняется (после реализации):**

- В `/voice/speaker/result` для **unknown** появятся новые поля
  `transient_label`, `ring_size`, `confidence` (cosine sim к ближайшему соседу).
- `epithet` для **unknown** остаётся `null` или отсутствует — эпитет требует
  реплик и истории, которых у transient-голоса нет.
- В памяти ноды появляется новый объект `core/unknown_speaker_ring.py`
  (in-memory, **не** `speakers.db`), привязанный к сессии.
- Cosine-порог — общий с identify-порогом (по умолчанию `0.75`, см. ADR-0008),
  но **архитектурно отделён** от `IDENTIFY_THRESHOLD` (задача 1 зафиксирует
  финальное значение; пока в спецификации используем placeholder `X.YY`).

---

## 2. Сценарии: Given / When / Then

В каждом сценарии под «свежим unknown» понимается utterance,
для которого `SpeakerDatabase.identify(embedding)` вернул `None` (cosine
ниже `IDENTIFY_THRESHOLD=0.75`). Все сценарии относятся **только** к
обработке **неизвестных** голосов; путь known-спикера не затрагивается.

### 2.1 EC-1: Пустой ring, первый unknown

| | |
|---|---|
| **Given** | `UnknownSpeakerRing` инициализирован, `len(ring) == 0`, `counter == 0`. |
| **When**  | Поступает utterance, `db.identify(embedding)` вернул `None`. |
| **Then**  | `ring.append({embedding, timestamp, transient_label="Голос-1", epithet=None, source_utterance_id})`. |
| **Then**  | В `/voice/speaker/result` публикуется: `{"is_known":false, "transient_label":"Голос-1", "ring_size":1, "confidence":null}`. |
| **Then**  | Внутренний счётчик `counter` инкрементируется до 1. |
| **Then**  | **speakers.db не пишется** — `wc -c /data/speakers.db` не меняется. |

### 2.2 EC-2: В ring ровно один вектор — ниже порога

| | |
|---|---|
| **Given** | В ring 1 вектор `v0` с `transient_label="Голос-1"`. |
| **When**  | Поступает utterance с эмбеддингом `v1`, `cos(v0, v1) < X.YY`. |
| **Then**  | Создаётся новый вектор `v1` с `transient_label="Голос-2"`. |
| **Then**  | `ring_size == 2`. |
| **Then**  | В результате: `{"is_known":false, "transient_label":"Голос-2", "ring_size":2, "confidence":cos(v0,v1)}`. |
| **Then**  | `v0` остаётся в ring без изменений (`timestamp` и `transient_label` те же). |

### 2.3 EC-3: В ring ровно один вектор — выше/равно порога

| | |
|---|---|
| **Given** | В ring 1 вектор `v0` с `transient_label="Голос-1"`. |
| **When**  | Поступает utterance с эмбеддингом `v1`, `cos(v0, v1) >= X.YY`. |
| **Then**  | **Не создаётся** новый вектор — возвращается `transient_label="Голос-1"`. |
| **Then**  | `v0.timestamp` обновляется на текущее время (LRU-подобное поведение). |
| **Then**  | `ring_size == 1` (не 2). |
| **Then**  | В результате: `{"is_known":false, "transient_label":"Голос-1", "ring_size":1, "confidence":cos(v0,v1)}`. |

### 2.4 EC-4: Несколько кандидатов выше порога — argmax

| | |
|---|---|
| **Given** | В ring 3 вектора: `v0` ("Голос-1"), `v1` ("Голос-2"), `v2` ("Голос-3"). |
| **When**  | Поступает `v_new`, для которого `cos(v_new, v1)=0.81`, `cos(v_new, v2)=0.88`, `cos(v_new, v0)=0.76`. Все ≥ `X.YY`. |
| **Then**  | Победитель — `v2` ("Голос-3"): `transient_label="Голос-3"`. |
| **Then**  | `v2.timestamp` обновляется (LRU-семантика). |
| **Then**  | `v0`, `v1` остаются без изменений. |
| **Then**  | `ring_size == 3` (без эвикции — capacity не превышен). |
| **Then**  | В результате: `{"is_known":false, "transient_label":"Голос-3", "ring_size":3, "confidence":0.88}`. |

### 2.5 EC-5: Эвикция при переполнении — FIFO по timestamp

| | |
|---|---|
| **Given** | Ring заполнен до `capacity` (например, 64). Все 64 вектора имеют уникальные `transient_label` ("Голос-1".."Голос-64"). У `v_old` самый ранний `timestamp`. |
| **When**  | Поступает новый unknown — ни один из 64 не пересекает порог. |
| **Then**  | `v_old` удаляется из ring. |
| **Then**  | Новый вектор добавляется с новым `transient_label="Голос-65"`. |
| **Then**  | `ring_size == capacity` (64), **не** 65. |
| **Then**  | Уникальность `transient_label` сохраняется: даже если "Голос-1" эвикнут, новый вектор получает **новый** номер (счётчик глобальный в пределах сессии и **никогда не переиспользует** номера). |

### 2.6 EC-6: Перезапуск сессии — ring сбрасывается

| | |
|---|---|
| **Given** | В ring 3 вектора ("Голос-1".."Голос-3"). `counter == 3`. |
| **When**  | Нода рестартует (cold start ИЛИ явный `SpeakerIdNode.reset_session()`). |
| **Then**  | `len(ring) == 0`, `counter == 0`. |
| **Then**  | Следующий unknown получает `transient_label="Голос-1"` (нумерация с 1). |
| **Then**  | **speakers.db не затрагивается** — рестарт ring не удаляет/не добавляет зарегистрированных спикеров. |
| **Then**  | In-flight utterance из предыдущей сессии, обрабатываемый thread-pool'ом, **не пишет** в новый ring (защита от гонки при перезапуске). |

### 2.7 EC-7: Одинаковые голоса после рестарта — НЕ persistent match

| | |
|---|---|
| **Given** | До рестарта в ring были "Голос-1" (Аня), "Голос-2" (Боря). Затем сессия перезапущена. |
| **When**  | Аня говорит снова, Боря говорит снова. Оба — unknown (не зарегистрированы). |
| **Then**  | Аня получает `transient_label="Голос-1"`, Боря — `transient_label="Голос-2"`. |
| **Then**  | Если же Аня говорит первой, потом Боря: Аня → "Голос-1", Боря → "Голос-2" (тот же порядок, но **НЕ благодаря памяти**, а благодаря argmax в новом пустом ring). |
| **Then**  | **Нет** cross-session continuity: запись «Аня = старая Голос-1» невозможна (нет ни persistent id, ни БД-lookup). |

### 2.8 EC-8: Concurrent доступ — race-condition smoke

| | |
|---|---|
| **Given** | Ring инициализирован, `lock` (threading.Lock) — внутреннее поле ring. |
| **When**  | Два потока одновременно вызывают `ring.match_and_maybe_append(embedding, ts)` с разными эмбеддингами. |
| **Then**  | Оба вызова возвращают результат без `RuntimeError`, без потери данных. |
| **Then**  | `ring_size` после обоих вызовов равен `min(2, capacity)`. |
| **Then**  | `transient_label` для каждого результата уникальны в пределах текущего состояния ring. |
| **Then**  | `wc -c /data/speakers.db` не меняется. |

**Замечание про модель concurrency:** нода `speaker_id_node.py` работает
на `threading` + `ThreadPoolExecutor(max_workers=1)` (L126). Inference
**сериализован** executor'ом, поэтому в реальной ноде race на уровне
ring возникает только если `match_and_maybe_append` вызывают **другие**
потоки (например, инструмент ручной склейки, диагностический код).
Спецификация требует наличия `threading.Lock` для защиты от этого
класса сценариев — даже если в проде executor их сериализует.

### 2.9 EC-9: Отсутствие записей в speakers.db — проверка изоляции

| | |
|---|---|
| **Given** | `speakers.db` пуста (или отсутствует). Ring создан. |
| **When**  | Обработано N=10 unknown-utterances. |
| **Then**  | `wc -l /data/speakers.db \| awk '{print ...}'` (или `SELECT COUNT(*) FROM speakers`) == 0. |
| **Then**  | `wc -l /data/speakers.db` для таблицы `embeddings` == 0. |
| **Then**  | Все 10 utterance опубликованы в `/voice/speaker/result` с `transient_label="Голос-1".."Голос-10"`. |

### 2.10 EC-10: Один и тот же вектор дважды (деградированная запись)

| | |
|---|---|
| **Given** | Ring пуст. `embedding_v1` поступает → ring.append("Голос-1"). |
| **When**  | Через 200ms приходит `embedding_v2 = v1 + α·noise` (тот же голос, зашумлённый), `cos(v1, v2) >= X.YY`. |
| **Then**  | Возвращается `transient_label="Голос-1"`. |
| **Then**  | `v1.timestamp` обновляется. |
| **Then**  | `ring_size == 1` (НЕ 2 — старая запись переиспользована, новая не создана). |
| **Then**  | `confidence = cos(v1, v2)`. |

**Открытый вопрос (см. ADR t_00ce84f2, Open Questions):**
перезаписывать ли `embedding` на новый (адаптивный кластер) или хранить
«первый» (стабильный центроид)? Спецификация требует, чтобы **оба
варианта** проходили EC-3 и EC-4 — разница только в том, что именно
используется как `centroid` при следующем матче. Решение — за ADR
t_00ce84f2.

---

## 3. Контракт /voice/speaker/result для unknown

Спецификация требует **совместимости снизу вверх**: существующие поля
(`is_known`) остаются, новые — опциональные (consumers, которые их не
ждут, должны игнорировать).

### 3.1 Known-спикер (без изменений)

```json
{
  "is_known": true,
  "speaker_id": "<uuid>",
  "name": "Аня",
  "confidence": 0.91,
  "epithet": "Гроссмейстер"
}
```

### 3.2 Unknown-спикер (после реализации ring)

```json
{
  "is_known": false,
  "transient_label": "Голос-3",
  "ring_size": 5,
  "confidence": 0.83
}
```

**Поля:**
- `is_known: false` — обязательно, как и сейчас.
- `transient_label: str` — **новое**, `"Голос-<N>"`, `N` начинается с 1,
  монотонно растёт в пределах сессии, не переиспользуется после
  эвикции. **Никогда не пустая строка**, никогда не `null`.
- `ring_size: int` — **новое**, текущий `len(ring)` сразу после решения
  match/append. Диапазон `[0, capacity]`.
- `confidence: float | null` — **новое**, `cosine_similarity` к ближайшему
  соседу в ring (или `null`, если ring был пуст — EC-1).

**Явно НЕ добавляется:**
- `speaker_id` — нет persistent id для unknown (см. ADR-0008).
- `name` — нет публичного имени (см. ADR-0018: persistent identity).
- `epithet` — эпитет требует истории реплик (см. `core/epithets.py`,
  `MIN_WORDS_FOR_TAGS=8`). Для unknown без истории он невозможен.
- Любая запись в `speakers.db` (EC-9, явный инвариант).

### 3.3 Обратная совместимость

Существующие потребители (см. `dialog_node.py`, `context_aggregator_node.py`),
которые проверяют `payload["is_known"] is True` или
`payload["is_known"] is False`, продолжают работать как раньше.
Новые поля просто игнорируются.

---

## 4. Инварианты ring (проверяемые свойства)

Эти инварианты формулируются как **отдельные** тесты, чтобы любое
отклонение давало чёткий diff в pytest.

| # | Инвариант | Проверка |
|---|-----------|----------|
| I-1 | `len(ring) ∈ [0, capacity]` | После каждой операции. |
| I-2 | `transient_label` уникальны в ring в любой момент времени | Перебор при каждом `match_and_maybe_append`. |
| I-3 | `transient_label` **никогда** не переиспользуется в сессии, даже после эвикции | Счётчик монотонный. |
| I-4 | `transient_label` принимает форму `^Голос-[1-9][0-9]*$` | Regex. |
| I-5 | Эвикция удаляет самый старый по `timestamp` | FIFO. |
| I-6 | При эвикции `ring_size` не растёт выше `capacity` | Инвариант. |
| I-7 | Если `cos >= X.YY` для существующего вектора, новый не создаётся | Изменение `len(ring)` после EC-3. |
| I-8 | Если ни один существующий не пересекает порог, новый создаётся с инкрементом `counter` | EC-1, EC-2. |
| I-9 | `timestamp` матчимого вектора обновляется на текущее время | EC-3, EC-4, EC-10. |
| I-10 | После `reset_session()` ring и counter обнуляются | EC-6. |
| I-11 | Ring не пишет в `speakers.db` (таблицы `speakers`, `embeddings`) | EC-9. |
| I-12 | При race-condition smoke нет `RuntimeError`, нет потери данных | EC-8. |

---

## 5. Метрики наблюдения в проде (требование задачи)

Эти метрики добавляются **в задаче реализации** (после t_f824ec16).
Здесь — **только требования**: какие именно графики/алерты нужны,
что они означают, какие пороги алертинга.

| Метрика | Тип | Назначение | Аномалия → что проверять |
|---|---|---|---|
| `voice_unknown_ring_size` | gauge | Текущий размер ring | Если `> capacity/2` стабильно — рестарт сессии мог не сработать, либо ёмкость слишком мала. |
| `voice_unknown_ring_evictions_total` | counter | Сколько раз сработала FIFO-эвикция | Резкий рост → счётчик `transient_label` «съедает» номера, что ожидаемо, но UX-команда должна видеть, как часто. |
| `voice_unknown_confidence_histogram` | histogram | Распределение `confidence` для всех unknown | Если >50% unknown имеют `confidence < X.YY`, значит ring забит «далёкими» соседями — вероятно, слишком много разных людей за сессию. |
| `voice_unknown_share_low_confidence_total` / `voice_unknown_total` | counter pair | Доля unknown-результатов с `confidence < X.YY` | > 70% → кольцо не помогает, либо порог завышен. |
| `voice_speaker_recognize_total{known="false"}` | counter (уже есть, issue #1160) | Количество unknown-результатов | Рост при стабильном known-трафике → деградация микрофона/акустики. |
| `voice_speaker_session_resets_total` | counter | Сколько раз `reset_session()` был вызван | Используется для отладки EC-6. |

**Алерты (Prometheus, post-merge):**

- `voice_unknown_ring_size > capacity` — **не должно случаться никогда**
  (инвариант I-1). Если сработал — баг, p2.
- `rate(voice_unknown_ring_evictions_total[5m]) > 10` — высокая
  активность эвикции, p3 (UX-наблюдение, не блокер).
- `voice_unknown_share_low_confidence > 0.7` за 1 час — p3.

---

## 6. Тест-план

### 6.1 Unit-тесты (моки, без rclpy/resemblyzer)

Файл: `src/rob_box_voice/test/unit/test_unknown_speaker_ring.py` (после
появления модуля). Импорт: `importlib.util.spec_from_file_location`
(как `test_speaker_embeddings.py`) — чтобы не тянуть rclpy через
`utils/__init__.py`.

| Тест | Покрывает | Ожидаемый результат |
|---|---|---|
| `test_empty_ring_first_unknown` | EC-1, I-1, I-2, I-3, I-4, I-8 | `transient_label="Голос-1"`, `ring_size=1`. |
| `test_single_vector_below_threshold` | EC-2 | Новый вектор "Голос-2". |
| `test_single_vector_at_threshold` | EC-3 (boundary) | "Голос-1" возвращён, `timestamp` обновлён. |
| `test_single_vector_above_threshold` | EC-3 | "Голос-1" возвращён. |
| `test_argmax_among_multiple_candidates` | EC-4 | Лучший по cosine. |
| `test_argmax_updates_loser_timestamp` | EC-4 + I-9 | Только у победителя. |
| `test_eviction_removes_oldest` | EC-5, I-5, I-6 | Самый ранний по `timestamp`. |
| `test_evicted_label_not_reused` | EC-5, I-3 | Счётчик глобальный. |
| `test_session_reset_clears_ring` | EC-6, I-10 | `len == 0`, `counter == 0`. |
| `test_session_reset_does_not_touch_speakers_db` | EC-6, EC-9, I-11 | Файл БД не тронут. |
| `test_same_voice_after_reset_gets_new_labels` | EC-7 | "Голос-1" снова. |
| `test_concurrent_match_and_maybe_append_smoke` | EC-8, I-12 | Нет исключений, `ring_size ∈ {1, 2}`. |
| `test_degraded_recording_matches_existing` | EC-10 | "Голос-1" возвращён. |
| `test_publish_payload_shape_unknown` | §3 | Ключи и типы полей. |
| `test_publish_payload_shape_known` | §3 | Регрессия: known-путь не сломан. |
| `test_invariants_hold_under_random_sequence` | I-1..I-12 | 100 случайных операций; в конце все инварианты выполняются. |

**Mock-стратегия:**
- `embedding`: `np.random.default_rng(seed).standard_normal(256)` →
  нормализация. Размерность совпадает с resemblyzer d-vector (256).
- `SpeakerDatabase.identify` — мокается; для unit-тестов ring не нужен
  реальный `embed_audio`.
- `threading` — реальный (нужен для EC-8).

### 6.2 Интеграционные тесты (фикстуры с реальными utterance)

Файл: `src/rob_box_voice/test/test_speaker_id_unknown_ring.py`.

| Тест | Что используется |
|---|---|
| `test_real_audio_two_distinct_unknowns_create_two_labels` | 2 WAV-файла с разными голосами (fixtures), `embed_audio` через resemblyzer. |
| `test_real_audio_repeated_same_unknown_reuses_label` | 1 WAV × 3 повтора (деградация). |
| `test_published_unknown_payload_matches_spec` | Подписка на `/voice/speaker/result` через rclpy, парсинг JSON, проверка полей по §3. |
| `test_eviction_under_sustained_traffic` | 70 utterance → capacity=64 → после 6-го `transient_label="Голос-71"`, `ring_size==64`. |

**Маркер:** `@pytest.mark.integration` (см. `pytest.ini`, раздел markers).

### 6.3 Что НЕ покрывается тестами (явно)

- Семантику выбора `centroid` (см. EC-10, Open Question) — пока не
  выбрана стратегия, тесты пишутся в обе стороны и помечаются
  `@pytest.mark.skip(reason="ADR t_00ce84f2 open question")`.
- Реальная семантика `confidence` в проде (порог X.YY пока не выбран,
  задача 1 зафиксирует). Unit-тесты используют параметризуемый
  `threshold` через конструктор ring.

---

## 7. Связь с другими артефактами

- **t_fd94dc4f (разведка)**: оттуда берём подтверждённые строки кода
  (`_publish_result` ~L729, `_recent_embeddings` ~L113, `_process_utterance`
  ~L291). При реализации кольца строки 305/320/348 будут затронуты.
- **t_7097619c (дизайн ring)**: оттуда — финальное `capacity`, выбор
  алгоритма поиска (линейный скан vs annoy/faiss), concurrency-стратегия.
  Эта спецификация **не предрекает** выбор алгоритма, но требует, чтобы
  он был детерминированным и unit-тестируемым.
- **t_00ce84f2 (ADR-черновик)**: финальная нумерация ADR-NNNN, раздел
  «Open Questions» (включая политику centroid при EC-10), ссылки на
  ADR-0008 (offline-идентификация) и ADR-0018 (контракт публикации).
- **t_d80b25c5 (сводный план реализации)**: собирает результаты всех
  четырёх веток и формирует раздел «## Implementation plan» в ADR или
  отдельным файлом.

---

## 8. Acceptance для самой этой спецификации (мета-AC)

Задача считается выполненной, когда:

- [ ] Этот документ лежит в `analysis/edge-cases-acceptance-unknown-speaker-ring.md`.
- [ ] Документ содержит все 10 сценариев EC-1..EC-10 в формате
      Given/When/Then.
- [ ] Все 12 инвариантов I-1..I-12 перечислены и пронумерованы.
- [ ] Контракт §3.1 / §3.2 / §3.3 задокументирован с явным
      «что НЕ добавляется».
- [ ] Тест-план §6 перечисляет unit-тесты и интеграционные тесты
      с явным указанием, что мокается.
- [ ] Таблица метрик §5 содержит ≥5 метрик с типом, назначением,
      алертами.
- [ ] Документ закоммичен в `wt/t_8909b73f` и PR открыт (база —
      `develop`).

**Не нужно для этой задачи:**

- Реализация `core/unknown_speaker_ring.py`.
- Реализация интеграции в `speaker_id_node.py`.
- Реализация метрик Prometheus.
- Тесты самого кода (появятся в ветке реализации).

---

**Подготовил:** tester (Kanban t_8909b73f, run 6864)
**Дата:** 2026-09-14
**Базируется на:** origin/develop HEAD ce60bb48 (актуальный на момент коммита).