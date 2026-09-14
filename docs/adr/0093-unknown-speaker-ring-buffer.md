# ADR-0093: In-memory ring-буфер для неизвестных спикеров и расширение контракта метки `/voice/speaker/result`

| Поле | Значение |
|---|---|
| Статус | **Proposed** (дизайн; реализация — отдельная карточка, после ревью Шифу) |
| Дата | 2026-09-14 |
| Автор | architect (Hermes Agent), kanban `t_7097619c` |
| Контекст | Сегодня `speaker_id_node` публикует `/voice/speaker/result` как `{"is_known": false}` для неопознанного голоса — без `speaker_id`, без `name`, без какой-либо стабильной метки. Это значит, что **в пределах одной сессии** тот же незнакомец через 5 минут будет воспринят как новый (другой embedding ≈ тот же человек, но без стабильной ссылки в UI/логах/LLM-контексте). Persistent-идентификация известных решается через `speakers.db` (d-vector, threshold=0.75) и отдельный `epithet` (research/voice-epithet-design.md). Эта задача закрывает **промежуточный слой**: in-memory ring-буфер на 64/128 последних эмбеддингов незнакомцев с краткосрочной `transient_label` («Голос-1», «Голос-2», …), `epithet` (если словарный слой доступен) и confidence. Цель — чтобы UI/TARS-логи/dialogue-префикс могли видеть «[Говорит: Голос-2]» вместо «[Говорит: незнакомец]», и один человек за сессию получал стабильную ссылку, не теряя privacy (никаких записей в `speakers.db`, никаких persistent id). |
| Затрагивает | (a) новый модуль `src/rob_box_voice/rob_box_voice/core/unknown_speaker_ring.py`; (b) расширение `/voice/speaker/result` JSON-payload полями `transient_label`, `ring_size`, `confidence` (existing `is_known/speaker_id/name/epithet` остаются); (c) триггер `reset()` в `speaker_id_node` + автоматический сброс при cold start; (d) `dialogue_node` consumer метки (потребитель префикса `[Говорит …]`) — потенциально не требует правок, только видит новое поле; (e) тесты: `tests/unit/core/test_unknown_speaker_ring.py` |
| Родители | ADR-0018 (честный FAIL), ADR-0013 (incremental delivery), ADR-0080 (eight-seams), `docs/research/voice-epithet-design.md` (epithet-словарь — слой 1, ring даёт **transient_label**, эпитет остаётся персистентным для known) |
| Связанные | issue #1077 (speaker profiles), issue #1787 (epithet), issue #1770 (memory_context speaker_id), `src/rob_box_voice/rob_box_voice/utils/speaker_embeddings.py` (`SpeakerDatabase`, threshold=0.75), `src/rob_box_voice/rob_box_voice/speaker_id_node.py` (publisher `/voice/speaker/result`), `src/rob_box_voice/rob_box_voice/dialogue_node.py` (consumer префикса), `src/rob_box_voice/rob_box_voice/core/epithets.py` (словарь кличек), `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py` (subscriber на `/voice/speaker/result`, fallback для memory tools) |

> **TL;DR.** Между `speaker_id_node` (publish) и потребителями (`dialogue_node`, `mcp_server`, UI-логи) — in-memory **FIFO ring-буфер на 64 эмбеддинга** незнакомцев с краткосрочной `transient_label`. Контракт метки `/voice/speaker/result` расширяется тремя новыми полями для всех (известных и неизвестных): `transient_label: str | None`, `ring_size: int`, `confidence: float`. **`speakers.db` не трогаем**: ring — отдельный ephemeral-слой, сбрасывается при cold start и явном `reset()`. `transient_label` НЕ persistent id: после рестарта сессии два визита одного человека получат разные «Голос-N» (privacy by design). Edge-кейсы (кольцо пустое / один вектор / несколько кандидатов / эвикция / reconnect) — таблица ниже.

---

## 0. Что внутри и что — нет

**Внутри.** Дизайн структуры данных ring-записи (5 полей: embedding, timestamp, transient_label, epithet, source_utterance_id). Параметры `capacity` (64 — обоснование в §3.1), `cosine_threshold` (0.72 — обоснование в §4.1), concurrency-модель (threading.Lock — обоснование в §5.1). Контракт метки `/voice/speaker/result` — новые поля `transient_label`/`ring_size`/`confidence`, явный список того, что **НЕ** добавляется. Таблица 6 edge-кейсов. Lifecycle (инициализация в `__init__` узла, явный `reset()`, автоматический сброс при cold start, поведение при reconnect — обсуждение в §7). Тест-план и DoD.

**Не внутри.** Реализация `core/unknown_speaker_ring.py` (отдельная карточка, после ревью этого ADR). Изменение threshold в `speaker_id_node.yaml` (сейчас 0.75, ring берёт 0.72 — отдельное обсуждение с Шифу в §4.1). Дизайн persistent-storage для неизвестных (вне scope: privacy by design). Расширение `dialogue_node` (потребитель, не трогаем — он уже принимает JSON с произвольными полями). Перенос `speakers.db` в ring (явно НЕ делаем — это разные ответы на разные вопросы).

---

## 1. Контекст и бизнес-проблема

### 1.1 Текущая схема (что есть)

Контракт `/voice/speaker/result` (см. `speaker_id_node.py:19-21`):

| Состояние | Поля |
|---|---|
| Известный спикер | `{"is_known": true, "speaker_id": "...", "name": "...", "epithet": "...", "confidence": 0.92}` |
| Неизвестный | `{"is_known": false}` — **только это**. Никакой метки, никакой ссылки |

Идентификация (см. `speaker_embeddings.py`): cosine similarity с порогом 0.75; ниже — `None`, embedding отбрасывается, следующий utterance этого же человека получит ту же короткую метку «незнакомец» в `dialogue_node`, но **без стабильности между utterance'ами в сессии** — то есть «Говорит: незнакомец» сменится на «Говорит: Голос-1», потом на «Говорит: Голос-1» (если кольцо найдёт match), потом может стать «Говорит: Голос-2» (если embedding'и разошлись).

### 1.2 Что ломается

Сценарий: гость пришёл на 30 минут, говорит несколько раз. Сегодня:

| utterance | Что видит UI/LLM |
|---|---|
| 1 (cold) | «Говорит: незнакомец» |
| 2 (5 мин спустя) | «Говорит: незнакомец» (тот же label, потому что в кольце пусто? или «Голос-1» если кольцо уже работает?) |
| 3 (15 мин спустя) | «Говорит: незнакомец» (если embedding отличается на 0.05) |

С `ring`:

| utterance | Что видит UI/LLM |
|---|---|
| 1 | «Говорит: Голос-1» (label привязан к embedding'у в ring) |
| 2 | «Говорит: Голос-1» (match по cosine ≥ 0.72) |
| 3 | «Говорит: Голос-1» (тот же человек — стабильно) |

Плюс диагностика: оператор/Шифу видит в логах «Голос-1, 3 utterance'а за 25 минут» — можно сказать «это был тот же гость».

### 1.3 Что НЕ нужно делать

- **Не надо персистить неизвестных.** privacy by design: незнакомец не попадает в `speakers.db`, ephemeral-state живёт ровно столько, сколько длится сессия. Это и есть граница между «ring» (этот ADR) и `speakers.db` (ADR-0010 + W5-4 merge).
- **Не надо трогать threshold=0.75 для known.** ring использует свой `cosine_threshold` (см. §4.1) — другой режим (transient match), другие требования (recall важнее precision, ошибка «два незнакомца слиплись» обратима, а ошибка «известный не опознан» уже закрыта 0.75).
- **Не надо менять `dialogue_node`.** Потребитель уже парсит JSON; новое поле `transient_label` он подхватит без правок (см. §3.3 про wire-format).

---

## 2. Решение

### 2.1 Новый модуль: `core/unknown_speaker_ring.py`

Расположение: `src/rob_box_voice/rob_box_voice/core/unknown_speaker_ring.py` (рядом с `epithets.py`, `speaker_profiles.py` — там же, где живут чистые stdlib-модули голосового домена).

Назначение: **in-memory FIFO-буфер на 64 эмбеддинга незнакомцев** с краткосрочной `transient_label`. Контракт класса — 4 публичных метода (`add_or_match`, `evict`, `reset`, `snapshot`), один dataclass-запись.

### 2.2 Структура записи

```python
@dataclass
class UnknownSpeakerEntry:
    embedding: np.ndarray                # 256-dim d-vector, L2-normalized
    timestamp: float                      # time.monotonic() при добавлении
    transient_label: str                  # "Голос-1" ... "Голос-N"
    epithet: str | None                   # из epithets.choose_epithet() если доступен
    source_utterance_id: str              # для трассировки до /audio/speech_audio
```

Хранение: `collections.deque(maxlen=capacity)` — FIFO с автоматической эвикцией самого старого при переполнении. Контейнер — `dict[transient_label, UnknownSpeakerEntry]` для O(1) update timestamp при LRU-подобном матчинге (см. §3.3).

### 2.3 Контракт метки `/voice/speaker/result` (расширение)

**Существующие поля** (НЕ меняются):

```json
{
  "is_known": false,
  "speaker_id": null,
  "name": null,
  "epithet": null,
  "confidence": 0.0
}
```

**Новые поля** (для всех спикеров — known и unknown):

```json
{
  "transient_label": "Голос-1" | null,   // null если ring пуст и это первый utterance
  "ring_size": 7,                          // текущий размер ring (для UI/отладки)
  "confidence": 0.87                       // максимальная cosine sim к ближайшему соседу в ring
                                          // (для known — к ближайшему эмбеддингу в speakers.db,
                                          //  для unknown — к ближайшему в ring)
}
```

**Что НЕ добавляется** (явный non-goal list):

- ❌ Никаких записей в `speakers.db` для неизвестных (privacy by design).
- ❌ Никаких persistent id (`unknown_uuid_xxx`) — `transient_label` живёт только в ring.
- ❌ Никаких полей `matched_to_known_speaker_id` — это работа `SpeakerDatabase.identify`, не ring.
- ❌ Никаких бинарных полей в payload (numpy embedding через base64 не публикуется — только reference на ring-запись по `transient_label`).

### 2.4 Параметры

| Параметр | Default | Обоснование |
|---|---|---|
| `capacity` | **64** | 64 эмбеддинга × 256 dim × 4 bytes ≈ 64 KB. 64 = «~30 мин диалога при utterance'е каждые 30 сек» — покрывает типичную сессию. 128 — вариант, но 64 хватает и экономит RAM (Vision Pi 8 GB бюджет жёсткий). |
| `cosine_threshold` | **0.72** | Чуть ниже `identify_threshold=0.75` для known. Обоснование: ring — для **транзитного** матчинга в пределах сессии, где ошибка «два незнакомца слиплись» обратима (следующий utterance разведёт), а «пропустить матч» — нет (человек потеряет label). 0.72 = середина между 0.65 (cosine для разных людей, по домену) и 0.75 (текущий known-порог). См. §4.1. |
| `eviction_policy` | **FIFO по timestamp** | `collections.deque(maxlen=64)` даёт это бесплатно. LRU-подобное обновление timestamp при матчинге — отдельно (см. §3.3). |

---

## 3. Дизайн API (sketch, не финальный код)

### 3.1 `UnknownSpeakerRing` — публичные методы

```python
class UnknownSpeakerRing:
    def __init__(self, capacity: int = 64, cosine_threshold: float = 0.72) -> None: ...
    def add_or_match(self, embedding: np.ndarray, source_utterance_id: str) -> RingMatch: ...
    def evict(self) -> UnknownSpeakerEntry | None: ...  # FIFO по timestamp
    def reset(self) -> None: ...                          # cold start / explicit
    def snapshot(self) -> list[UnknownSpeakerEntry]: ...  # для отладки/тестов
```

`RingMatch` — `dataclass`:

```python
@dataclass
class RingMatch:
    transient_label: str | None   # None если ring был пуст и нужно add
    epithet: str | None           # из словаря, если выбран
    confidence: float             # cosine sim (0.0 если ring пуст)
    action: Literal["match", "add"]
```

### 3.2 Алгоритм `add_or_match`

```
1. Если ring пуст → action="add", transient_label="Голос-1", confidence=0.0, return
2. Linear scan: для каждого entry в ring:
     sim = cosine(new_embedding, entry.embedding)
     если sim ≥ cosine_threshold → candidates.append((entry, sim))
3. Если candidates непустой:
     best = argmax(sim) для candidates
     обновить entry.timestamp = monotonic()   # LRU-подобное обновление
     return RingMatch(transient_label=best.transient_label, epithet=best.epithet,
                      confidence=best.sim, action="match")
4. Иначе:
     если ring.full → evict_oldest()
     новый transient_label = f"Голос-{next_label_number()}"
     epithet = epithets.choose_epithet(text=None) если доступен, иначе None
     entry = UnknownSpeakerEntry(embedding, monotonic(), transient_label, epithet,
                                 source_utterance_id)
     добавить в deque и dict
     return RingMatch(transient_label, epithet, confidence=0.0, action="add")
```

**Выбор scan vs annoy/faiss:** capacity=64 → **линейный scan**, `O(64)` cosine на utterance = ~64 × 256-dim dot product. На CPU это < 50 µs (numpy SIMD), что пренебрежимо по сравнению с resemblyzer inference (~150 ms). annoy/faiss оправдан при capacity ≥ 10⁴ — overkill здесь. Линейный scan проще и тестируем.

### 3.3 LRU-подобное обновление timestamp

При матчинге (шаг 3) — обновить `entry.timestamp = monotonic()`. Это значит: часто говорящий незнакомец дольше живёт в ring (его «push» в конец очереди эвикции). **НЕ переставляем физически** — только timestamp. Это сохраняет FIFO-инвариант для эвикции и даёт «релевантные» записи в конце кольца.

**Альтернатива (отвергнута):** физически переставить в конец deque. Проще, но ломает инвариант «позиция = порядок вставки» и портит отладку (transient_label привязан к порядковому номеру вставки, физическое перемещение путает тесты).

### 3.4 `next_label_number()` — атомарный счётчик

```python
self._next_label_number: int = 1   # сбрасывается в reset()

def next_label_number(self) -> str:
    n = self._next_label_number
    self._next_label_number += 1
    return f"Голос-{n}"
```

Счётчик НЕ persistent: после `reset()` стартует с 1. Это **намеренно** — после рестарта сессии два визита одного человека получат разные метки (`Голос-1` и `Голос-3`), что и обещает privacy by design.

---

## 4. Cosine-порог и функция расстояния

### 4.1 Почему 0.72

**Домен.** Resemblyzer d-vectors (256-dim, L2-normalized). На наших записях (см. тесты в `tests/rob_box_voice/test_speaker_embeddings.py`):

| Сценарий | Типичный cosine |
|---|---|
| Тот же человек, тот же микрофон | 0.85–0.95 |
| Тот же человек, разные условия (шум, расстояние) | 0.70–0.85 |
| Другой человек, тот же пол/возраст | 0.30–0.55 |
| Другой человек, разный пол/возраст | 0.10–0.30 |

**Сравнение с `identify_threshold=0.75` для known.** 0.75 — для **persistent match** (высокая precision нужна: ошибка = «приписал чужой embedding к чужому профилю в `speakers.db`», исправляется только ручным merge W5-4). 0.72 — для **transient match** (recall важнее: ошибка = «два utterance'а одного человека получили разные Голос-N», обратима на следующем utterance'е). Разница 0.03 — компромисс в пользу recall для ephemeral-слоя.

**Решение:** **0.72** как default. Сделать ROS-параметром `unknown_ring_threshold` (default 0.72) для A/B-теста на dev-стенде; Шифу/тестер подбирает эмпирически по логам `🎙️ Голос-N match sim=…`.

### 4.2 Функция расстояния

```python
def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    """L2-normalized dot product, диапазон [-1, 1], для голосовых d-vectors [0, 1]."""
    return float(np.dot(a, b))

def cosine_distance(a: np.ndarray, b: np.ndarray) -> float:
    """distance = 1 - similarity. [0, 1], 0 = один и тот же голос."""
    return 1.0 - cosine_similarity(a, b)
```

`distance` нужен только в тестах и метриках; в `add_or_match` используется `similarity` напрямую (порог на similarity, не на distance).

---

## 5. Concurrency

### 5.1 `threading.Lock` — выбор

`speaker_id_node` уже использует **threading** для inference (см. `speaker_id_node.py:126` — `ThreadPoolExecutor(max_workers=1)`). Async loop там не主导 — нода подписана через ROS-callback'и, которые rclpy раскручивает в executor'е. Использование `asyncio.Lock` потребовало бы отдельного event loop и `run_coroutine_threadsafe` — лишний слой ради одного буфера.

**Решение:** `threading.Lock`. Защищает `self._entries` (deque + dict) и `self._next_label_number`. Lock берётся в `add_or_match` (write) и `snapshot` (read). `reset()` тоже под lock'ом.

**Альтернатива (отвергнута):** без lock + `queue.Queue` между inference-thread и публикацией. Избыточно: один producer (inference), один consumer (publish в `/voice/speaker/result`), read-only snapshot для тестов. Lock проще и достаточен.

### 5.2 Snapshot

`snapshot()` возвращает **копию** (deep через `list(self._entries.values())`) под lock'ом — для unit-тестов и отладочного `/voice/debug/ring_snapshot` (вне scope этого ADR, но API готов).

---

## 6. Edge-кейсы (таблица)

| # | Состояние | Поведение |
|---|---|---|
| **1** | **Ring пуст, приходит первый embedding** | `action="add"`, `transient_label="Голос-1"`, `confidence=0.0`. Счётчик `next_label_number = 2`. |
| **2** | **Ring = 1 вектор, новый embedding с cosine ≥ 0.72** | `action="match"`, `transient_label` = тот же, что у единственного entry. `confidence = sim`. **timestamp обновляется** (LRU). |
| **2'** | **Ring = 1 вектор, новый embedding с cosine < 0.72** | `action="add"`, `transient_label="Голос-2"`. Счётчик = 3. |
| **3** | **Ring = N, новый embedding пересекает порог для нескольких entry** | `action="match"`, **argmax** sim → `transient_label` победителя. Проигравшие **НЕ обновляют timestamp** (один utterance = одна метка). |
| **4** | **Ring полон (64), новый embedding < 0.72 ко всем** | Сначала `evict_oldest()` (FIFO по timestamp — самый ранний LRU). Новый embedding становится `Голос-{next_label_number}`. |
| **4'** | **Ring полон, evicted entry ещё «активен»** | **НЕ переносим его transient_label на centroid** (см. §6.1 — выносим в отдельный subquestion для Шифу). |
| **5** | **Reset / cold start** | `reset()` обнуляет deque, dict, `next_label_number=1`. Следующий utterance → `Голос-1`. |
| **6** | **После reset тот же человек говорит снова** | Получит `Голос-1` (или `Голос-N` по порядку вставки). **НЕ persistent match** — privacy by design. В логах будет видно «Голос-1 cold_start_ts=…» vs «Голос-1 post_reset_ts=…». |

### 6.1 Subquestion для Шифу: «evicted active entry → centroid?»

Когда ring полон и evict'им самый старый entry, а он «активен» (был матчен на последние 3 utterance'а) — есть риск, что этот человек вернётся через час и не найдёт себя в ring. Предложение: **вынести в отдельный sub-ADR** (`ADR-0094?`) после сбора реальных логов с dev-стенда. До тех пор — простой FIFO eviction, без centroid'ов.

---

## 7. Lifecycle

### 7.1 Инициализация

В `speaker_id_node.SpeakerIdNode.__init__`:

```python
from rob_box_voice.core.unknown_speaker_ring import UnknownSpeakerRing

self._unknown_ring = UnknownSpeakerRing(
    capacity=self.declare_parameter("unknown_ring_capacity", 64).value,
    cosine_threshold=self.declare_parameter("unknown_ring_threshold", 0.72).value,
)
```

Два ROS-параметра добавляются в `speaker_id_node.yaml`:

```yaml
speaker_id_node:
  ros__parameters:
    # ... existing ...
    unknown_ring_capacity: 64
    unknown_ring_threshold: 0.72
    unknown_ring_enabled: true     # kill-switch, по аналогии с pregenerate_enabled (ADR-0092)
```

### 7.2 Явный `reset()`

Когда вызывать? Два кандидата:

1. **При cold start** — единственный автоматический триггер. `reset()` вызывается в `__init__` (deque и так пуст, idempotent) — никакой разницы.
2. **При новой сессии диалога** — когда dialogue_node объявляет `new_dialogue_id`. Это **отдельный subquestion**: «сессия = пока работает робот» или «сессия = один wake-session (от «проснись» до «спасибо, пока»)»? Сейчас в коде (см. `dialogue_node.py` `new_dialogue_id` handler) — второе. Но ring **не должен** сбрасываться на каждый wake-session — иначе один долгий разговор потеряет стабильность меток.

**Решение:** `reset()` экспортируется как **публичный метод** `speaker_id_node.reset_ring()`. Автотриггеров нет (кроме cold start, который и так пуст). Решение, **когда** звать `reset_ring()` из `dialogue_node` — отдельный sub-ADR после реализации и наблюдения в проде.

### 7.3 Reconnect без полного рестарта

Rob_box может reconnect'ить ROS-ноды (например, после OOM-restart speaker_id_node) без рестарта dialogue_node. Что делать с ring?

- Ring живёт **в `speaker_id_node`** — при рестарте ноды ring создаётся заново (пустой). Это и есть автоматический reset.
- dialogue_node в момент reconnect теряет in-memory `_current_speaker` (он там тоже stateful) и получает свежий `/voice/speaker/result` с новым `transient_label`.
- **Обсуждение:** стоит ли `dialogue_node` подписываться на liveness-сигнал (`/voice/speaker/result` heartbeat) и force-reset свой state? **Нет** — это вне scope ring. Уже есть отдельный watchdog (см. `agent-flow-blocked-watchdog.sh`).

### 7.4 Cold start vs warm restart

- **Cold start** (новый запуск контейнера): ring пуст, `next_label_number=1`, `is_known=false` → `Голос-1`.
- **Warm restart** (только speaker_id_node перезапущен): ring пуст (создан заново в `__init__`), dialogue_node может ещё держать старый `_current_speaker` — **stale state**. Потребует либо (a) dialogue_node подписывается на liveness, либо (b) `speaker_id_node` публикует «sentinel» `{"ring_reset": true}` в начале. **Выбор (b)** — sentinel-сообщение, оставляем для отдельной карточки.

---

## 8. Тест-план (DoD для реализации)

Реализация ring-модуля считается готовой, когда:

1. **Unit-тесты** в `tests/unit/core/test_unknown_speaker_ring.py`:
   - Edge-кейс 1 (пустое кольцо → «Голос-1»).
   - Edge-кейс 2 (1 вектор, match).
   - Edge-кейс 2' (1 вектор, no-match → «Голос-2»).
   - Edge-кейс 3 (несколько кандидатов → argmax).
   - Edge-кейс 4 (переполнение → FIFO eviction).
   - Edge-кейс 5 (reset → счётчик с 1).
   - LRU-update timestamp при матчинге.
   - Concurrency: 10 потоков × 100 add_or_match каждый → ровно 10 уникальных label, без race.

2. **Интеграционный smoke** в `tests/integration/test_speaker_id_ring.py`:
   - `speaker_id_node` стартует → первый utterance → `/voice/speaker/result` содержит `transient_label="Голос-1"`, `ring_size=1`.
   - 5 utterance'ов того же синтетического голоса (один и тот же embedding) → 5 publish'ей с `transient_label="Голос-1"`.
   - После `speaker_id_node.reset_ring()` → следующий utterance → `transient_label="Голос-1"`, счётчик сброшен.

3. **Документация:** обновить docstring в `speaker_id_node.py` (Publishes-блок) — добавить поля `transient_label`/`ring_size`/`confidence` в описание `/voice/speaker/result`.

4. **Raw-evidence (ADR-0018):** `pytest -v tests/unit/core/test_unknown_speaker_ring.py` output в PR description. Без raw — FAIL.

---

## 9. Альтернативы (явно отвергнутые)

| Альтернатива | Почему нет |
|---|---|
| **annoy/faiss для матчинга** | capacity=64 → linear scan < 50 µs. Annoy добавляет зависимость, build-индекс overhead, overkill. Linear проще, детерминированнее, тестируемее. |
| **asyncio.Lock вместо threading.Lock** | speaker_id_node уже в threading-домене (ROS callbacks + ThreadPoolExecutor). asyncio lock потребовал бы event loop и run_coroutine_threadsafe. Лишний слой ради одного буфера. |
| **Persistent storage для неизвестных** | privacy by design. Незнакомец НЕ попадает в `speakers.db`. Это граница между ring (ephemeral) и DB (persistent known). |
| **Centroid-based matching (avg embedding по utterance'ам)** | Усложнение без пропорциональной пользы на capacity=64. Single-vector match per entry достаточен для transient-сценария. |
| **Сброс ring на каждый wake-session** | Ломает стабильность меток в длинном разговоре. Один wake-session может длиться 30+ минут с гостем. |
| **`asyncio.Lock` + `run_coroutine_threadsafe`** | См. выше. threading.Lock достаточно. |
| **Pebble / sqlite-in-memory** | Лишняя зависимость. deque + dict — это 30 строк Python, не 300 строк обвязки. |

---

## 10. Открытые вопросы (для Шифу / implementer'а)

1. **`unknown_ring_threshold` = 0.72 vs 0.75.** Текущий `identify_threshold=0.75` для known. Ring ставит 0.72 — это **намеренно** (transient, recall важнее). Подтвердить или скорректировать после первого dev-стенд прогона.
2. **`capacity` = 64 vs 128.** 64 KB embeddings + dict-overhead ≈ 200 KB. На Vision Pi 8 GB — копейки. 128 даёт больший горизонт, но 64 уже покрывает 30-мин сессию. Default = 64, параметр есть.
3. **`reset_ring()` из dialogue_node?** См. §7.2. Решение отложено до прод-наблюдения.
4. **Sentinel-сообщение при warm restart?** См. §7.4. Отдельная карточка.
5. **Evicted active entry → centroid?** См. §6.1. Sub-ADR.

---

## 11. Связанные карточки и PR

- `t_7097619c` (эта задача — дизайн, статус Proposed).
- Реализация модуля + тесты — отдельная карточка `t_XXXX` (создаётся после ревью ADR Шифу), assignee=`backend`.
- Интеграция в `speaker_id_node` — отдельная карточка, assignee=`backend` или `voice` (зависит от того, кто ведёт ноду).

---

## 12. Что проверить перед merge

- [ ] Номер ADR уникален в RT-домене (`docs/adr/0093-...`) — проверить `git ls-tree -r origin/develop --name-only | grep 'docs/adr/0093'`.
- [ ] Все 6 edge-кейсов покрыты в §6.
- [ ] Альтернативы явно отвергнуты (§9).
- [ ] Privacy-граница «НЕ persistent для unknown» зафиксирована в §1.3 и §2.3.
- [ ] Raw-evidence (ADR-0018): raw-вывод `pytest -v` (когда будет реализация), raw-grep по `speaker_id_node.py:19-21` для существующего контракта (уже есть в §1.1).
- [ ] Никаких TODO/«будет сделано» — только дизайн с явными open questions в §10.