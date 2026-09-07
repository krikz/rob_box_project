# ADR-0056: Контракт `pregenerate` в `tts_node` — Шаг 13 миграции оператор-агента

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-07 |
| Автор | architect по карточке kanban `t_201e2c64` (issue #2003) |
| Контекст | Issue #2003 «[operator-agent 13] Спекулятивная генерация (pregenerate в tts_node) — ЗАБЛОКИРОВАН». Между завершением чанка *k* и стартом чанка *k+1* одного assistant turn'а сейчас окно «синтезатор простаивает, динамик тоже»; в TTS-канале это десятки-сотни мс латентности, которые на каждом батче складываются в секунды. Спекулятивная генерация запускает синтез следующего чанка **до** `batch_complete` текущего. Это **не** про LLM — это про TTS-канал, **не** пре-эмпция планировщика. |
| Затрагивает | (a) новые методы `TTSNode.pregenerate`, `TTSNode.claim_pregen`, `TTSNode.cancel_pregen` в `src/rob_box_voice/rob_box_voice/tts_node.py`; (b) расширение payload `/voice/dialogue/response` и `/voice/tts/request` (опциональное вложенное поле `pregenerate`); (c) новые dataclasses `PreGenTask`, `PreGenResult`, `QualityVerdict`, `Decision`, `SegmentEstimate` в `src/rob_box_voice/rob_box_voice/scheduler/pregen/` (НЕ `scheduler/` — см. §6.3); (d) **НЕ** затрагивает: `dialogue_node`, `TaskScheduler`, существующий `scheduler/speculative_executor.py`, `scheduler/pre_gen.py` (см. §4 и §6.2 — они на неверном уровне абстракции) |
| Родители | ADR-0051 §2 (оператор-агент), целевая `docs/architecture/target-operator-agent-and-dialogue.md` §8а.4 п.3 и §13 Шаг 13, `docs/plans/2026-09-05-operator-agent-architecture-handoff.md` §4.8 «`pregenerate` не существует», `docs/design/SCHEDULER_DESIGN.md` §6.5, ADR-0018 (культура честности), ADR-0013 (incremental delivery) |
| Связанные | issue #2003 (эта карточка), issue #1996 «[operator-agent 07a] Приоритет в `tts_node`» (dep 7a, **ОТКРЫТА** — блокер реализации, контракт пишется ahead-of-merge), карточка `t_9798710b` (developer) — implementer берёт ADR и §3.1–3.7 как спеку; `docs/architecture/adr/pregenerate-in-tts-node-contract.md` в коммите `a1b823ce2` (предыдущая попытка на ветке `z-{agent}/2003-operator-agent-13-pregenerate-tts-node` — не слита; §4 здесь — явный отказ от её §4 «тонкий адаптер к `SpeculativePreGenerator`») |
| Предшественник в коде | WIP `a1b823ce2` (2026-09-07) «pregenerate-in-tts-node contract — frozen design until 7a lands» — единственный предыдущий драфт, учтён в §4 |

> **TL;DR.** Спекулятивная генерация делается **локально в `tts_node`**
> тонким слоем из пяти маленьких чистых модулей (`pre_gen`, `quality`,
> `estimator`, `decision`, `speculative_executor`) **под** директорией
> `scheduler/pregen/`. Существующий `scheduler/{pre_gen,speculative_executor,
> estimator,quality,decision}.py` НЕ переиспользуется — он работает на
> уровне scheduler-иерархии (сегменты / `MERGE` / `REPLACE` / сегмент-план),
> а чанковый pre-gen работает на уровне аудио-чанка (миллисекунды, один
> speech_id, один синтез-вызов). Это разные единицы прерывания; адаптер
> «через границу» — лишний IPC и лишняя поверхность для багов. Контракт
> ниже описывает **ровно** то, что нужно реализатору, и **ровно** то,
> что DoD issue'а требует проверить.

---

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдается

`tts_node.dialogue_callback` (4198 LOC, `src/rob_box_voice/rob_box_voice/tts_node.py:1725`) получает чанки assistant turn'а последовательно: `k`, `k+1`, …, `K-1`, `K` (последний батча, поле `batch_index == batch_total`). Между завершением воспроизведения чанка *k* и стартом синтеза чанка *k+1* сейчас есть два окна:

1. **Окно «свободного канала»** — после `_on_synthesis_done` (строка 2064) и до `_submit_synthesis` следующего чанка. Десятки мс (FIFO-gate ждёт `play_seq`).
2. **Окно синтеза** — сам TTS-вызов (Yandex gRPC ~300–800 мс, MiniMax HTTP стрим ~150–400 мс, Silero ~30–80 мс). Это **большое** окно.

Для assistant turn'а из 3–5 чанков суммарный простой ~0.5–2 с на повороте. Ощущается как «робот замолкает между предложениями».

### 1.2 Что не работает как решение

| Идея | Почему отвергнута |
|---|---|
| Передать решающее слово `TaskScheduler` (планировщик из `dialogue_node`) | Планировщик работает в **отдельном процессе** и не знает про аудио-чанки `tts_node`; передача `speech_id`/аудио через границу = IPC-удар и новый топик. Главное: единицы прерывания разные — у scheduler'а сегменты (`MERGE`/`REPLACE`/сегмент-план), у `tts_node` — отдельные чанки одного сегмента. См. ADR-0011 §6.3 (action server не пересекается с `SideEffectBus` по scope) — тот же принцип. |
| Использовать существующий `scheduler/pre_gen.py` (`SpeculativePreGenerator`, 475 LOC) как «тонкий адаптер» | Уровень абстракции неверный. `SpeculativePreGenerator` оперирует `PreGenCandidate(task_id, estimate, metadata)` и фабрикой `PreGenFactory = Callable[[PreGenCandidate], Awaitable[Tuple[str, Any]]]` (`pre_gen.py:128`). Это «обогати `task` плана и запусти фабрику». Нам нужно «возьми готовый текст следующего чанка и синтезируй». Попытка превратить `PreGenCandidate.metadata` в TTS-payload — это шим, который превратит один интерфейс в другой, и любая ошибка превратится в загадочный краш в чужом модуле. См. предыдущая попытка `a1b823ce2` §4 — отвергнуто **по тем же** причинам; согласен. |
| Пре-эмпция `dialogue_node` через LLM-streaming | Это LLM-канал, не TTS. LLM-streaming даёт **текст** следующего чанка быстрее, но `tts_node` всё равно синтезирует его с нуля после `batch_complete`. Не решает. |
| Ничего не делать | Текущее поведение комфорт-критично для Шифу (live наблюдение «робот молчит между предложениями»). DoD issue #2003 явно требует измеримого уменьшения `latency_chunk_to_chunk`. |

### 1.3 Целевая картина (после реализации)

```
[chunk k синтез]→[chunk k в динамик]→[тишина: ~300-800мс]→[chunk k+1 синтез]→[chunk k+1 в динамик]
                              ▲                                    ▲
                              │                                    │
              batch_complete(k) опубликован         pregenerate(k+1) уже в полёте,
                                                       аудио кладётся в _prefetch.results[id]
                              └──────────── результат ────────────┘
                              ▼
              audio playback берёт готовое аудио из _prefetch.results[id]
              и шлёт в динамик СРАЗУ, без окна синтеза
```

Окно синтеза схлопывается до латентности **одного только** «принять готовое аудио → ALSA». Окно свободного канала — тоже (внутри `tts_node`, локально).

---

## 2. Решение

Пять маленьких модулей в **новой** поддиректории `src/rob_box_voice/rob_box_voice/scheduler/pregen/`, тонкий слой оркестрации `_PrefetchEngine` внутри `tts_node`, расширение payload на 2 топиках. Никакого нового транспорта, никаких изменений в `dialogue_node` или `TaskScheduler`.

### 2.1 Пять модулей (`scheduler/pregen/`)

| Модуль | Что делает | Когда вызывается |
|---|---|---|
| `pre_gen.py` (новый, ≤120 LOC) | Превращает `(current_chunk, ctx, voice_config) → Optional[PreGenTask]`. Возвращает `None`, если спекуляция невозможна (нет следующего чанка в `ctx`, нечего пре-генерировать, etc.) | Sync, в ROS callback thread, **до** `_submit_synthesis` текущего чанка |
| `estimator.py` (новый, ≤80 LOC) | Возвращает оценку `(confidence, basis)` для одного спекулятивного чанка. Порог: `confidence ≥ CONFIDENCE_FLOOR` (default 0.6) | Sync, сразу после `pre_gen`, перед запуском asyncio.Task |
| `quality.py` (новый, ≤100 LOC) | Возвращает `QualityVerdict.{PASS,FAIL}` по **готовому** аудио. 3 эвристики: `duration_ratio ≤ 1.25`, `silence_ratio ≤ 0.15`, `rms_low == False` (см. §3.6) | Async, по завершении `_run_pregen_one` |
| `decision.py` (новый, ≤60 LOC) | Правило «принять/отбросить» спекулятивный чанк. На входе: `QualityVerdict` + `confidence`. На выходе: `Decision.{ACCEPT, REJECT}` | Sync, после `quality` |
| `speculative_executor.py` (новый, ≤150 LOC) | Оркестратор: `kickoff(current_chunk, ctx, voice_config) → None` (fire-and-forget), `cancel(reason: str) → int`, `claim(speech_id) → Optional[np.ndarray]`. Хранит `_active: dict[str, asyncio.Task]` + `_results: dict[str, np.ndarray]` | Async; создаётся **ровно один** на `TTSNode` |

**Почему НЕ `scheduler/pre_gen.py` и подобные (старые).** Существующие модули в `scheduler/` оперируют scheduler-иерархией: `PreGenCandidate.task_id` — это id scheduler-задачи (один сегмент, возможно много чанков); `SpeculativePreGenerator.build_plan` — это про «пройдись по pending segments и поставь freeze-boundary». Нам нужно про **один конкретный чанк** с уже готовым SSML и его `speech_id`. Подробнее — §6.2.

### 2.2 Что добавляется в `tts_node`

```python
class TTSNode(Node):
    # ... существующие поля ...

    # NEW (§3.1)
    _prefetch: Optional[_PrefetchEngine] = None   # лениво, при первом pregenerate

    # NEW (§3.4)
    def pregenerate(self, current_chunk: Chunk, ctx: DialogueContext) -> None:
        """Запустить спекулятивный синтез next-чанка. Fire-and-forget."""

    # NEW (§3.4)
    def claim_pregen(self, speech_id: str) -> Optional[np.ndarray]:
        """Забрать готовое аудио для speech_id. None если не готово."""

    # NEW (§3.5)
    def cancel_pregen(self, reason: str) -> int:
        """Отменить все in-flight пре-гены. Возвращает количество отменённых."""
```

`_PrefetchEngine` — приватный dataclass внутри `TTSNode.__init__`, не отдельный класс. Это оркестратор из `speculative_executor.py` плюс ссылка на `self` для вызова `_synthesize_yandex`/`_synthesize_minimax_streaming_publish`/etc.

### 2.3 Расширение payload (опциональное вложенное поле)

Поскольку dep 7a (issue #1996) ещё не слита, описываю **итоговый** payload (после merge 7a). Реализация ждёт.

| Топик | Поле | Тип | Когда обязательно |
|---|---|---|---|
| `/voice/dialogue/response` | `pregenerate.next_speech_id` | `str (uuid)` | Всегда, если паблишер знает следующий чанк |
| `/voice/dialogue/response` | `pregenerate.next_ssml` | `str` | Всегда (или `next_text`, см. §3.1) |
| `/voice/tts/request` | `pregenerate` | объект `{next_speech_id, next_ssml}` | То же |
| `/voice/dialogue/response` | `pregenerate.disabled_reason` | `str?` | Если паблишер знает следующий чанк, но решил не спекулировать (`"no_next"`, `"low_confidence"`, `"operator_interrupt"`); для метрик |

**Семантика «нет поля `pregenerate`»** = поведение прежнее (никаких изменений в существующих пайплайнах).

---

## 3. Контракт (по функциям и потоку данных)

### 3.1 Типы данных (`scheduler/pregen/__init__.py`)

```python
@dataclass(frozen=True)
class PreGenTask:
    """Спекулятивная задача: синтезировать аудио для next_speech_id."""
    next_speech_id: str                 # uuid, заранее зарезервированный паблишером
    next_ssml: str                      # SSML чанка k+1
    voice: str                          # 'anton' / 'lera' / ...
    language: Optional[str]             # 'ru' / None
    ssml_attributes: dict               # pitch/rate/volume, как в _parse_ssml_attributes
    dialogue_id: str                    # для stale-check
    priority: str                       # 'normal' | 'operator' | 'personality' (поле 7a)

@dataclass(frozen=True)
class SegmentEstimate:
    """Оценка confidence из estimator."""
    confidence: float                   # [0.0, 1.0]
    basis: str                          # 'heuristic_v1' | 'baseline_chars_per_sec' | ...

class QualityVerdict(str, Enum):
    PASS = "pass"                       # аудио годно
    REJECT_SILENT = "reject_silent"     # rms_low
    REJECT_CLIPPED = "reject_clipped"   # duration_ratio > 1.25
    REJECT_TRIMMED = "reject_trimmed"   # silence_ratio > 0.15

class Decision(str, Enum):
    ACCEPT = "accept"
    REJECT = "reject"

@dataclass(frozen=True)
class PreGenResult:
    """То, что кладётся в _PrefetchEngine._results."""
    speech_id: str
    audio: np.ndarray                   # float32 mono @ 22050 Hz (после ресемплинга)
    sample_rate: int
    decision: Decision
    confidence: float
    basis: str
    elapsed_ms: float                   # сколько синтез реально занял
```

### 3.2 Сигнатуры модулей

```python
# pre_gen.py
def build_pregen_task(
    current_chunk: Chunk,           # текущий (последний принятый)
    ctx: DialogueContext,            # snapshot диалога (recent_texts, dialogue_id, voice_cfg)
    voice_config: VoiceConfig,       # голос + язык из tts_node.__init__
) -> Optional[PreGenTask]:
    """Вернуть next-чанк для спекуляции или None.

    None если:
      - ctx.recent_texts пуст (LLM ещё ничего не прислала);
      - текущий чанк последний в turn'е (batch_total уже);
      - next_speech_id не зарезервирован паблишером.
    """

# estimator.py
CONFIDENCE_FLOOR: float = 0.6

def estimate_confidence(
    pregen: PreGenTask,
    recent_actual_durations_ms: list[float],  # последние 10 синтезов того же voice
) -> SegmentEstimate:
    """Pure function. confidence ≥ CONFIDENCE_FLOOR → спекуляция имеет смысл."""

# quality.py
def check_audio_quality(
    audio: np.ndarray,
    sample_rate: int,
    estimated_duration_ms: float,
) -> QualityVerdict:
    """Эвристики: rms_low, duration_ratio, silence_ratio. Sync, pure."""

# decision.py
def decide(verdict: QualityVerdict, confidence: float) -> Decision:
    """ACCEPT iff verdict==PASS AND confidence ≥ CONFIDENCE_FLOOR."""

# speculative_executor.py
class SpeculativeExecutor:
    def __init__(self, *, synth_callable: Callable[..., np.ndarray],
                 event_bus: Optional[EventBus] = None) -> None: ...

    async def kickoff(self, current_chunk: Chunk, ctx: DialogueContext,
                      voice_config: VoiceConfig) -> None:
        """pre_gen → estimator → если confidence ≥ FLOOR → asyncio.Task(synth+quality+decision)."""

    async def cancel(self, reason: str) -> int:
        """Отменить in-flight, очистить results. Возвращает count."""

    def claim(self, speech_id: str) -> Optional[PreGenResult]:
        """Ownership-transfer: забрать и удалить из кэша. None если нет."""

    def snapshot(self) -> dict[str, int]:
        """Диагностика: {active, completed, rejected, cached}."""

    async def shutdown(self) -> None: ...
```

### 3.3 Поток данных (sequence)

```
dialogue_callback(msg)
  │
  ├── chunk k → обычный путь синтеза (как сейчас)
  │
  └── _PrefetchEngine.speculative_executor.kickoff(current_chunk=chunk_k, ctx=…, voice_config=…)
        │
        ├── pre_gen.build_pregen_task() → Optional[PreGenTask]
        │     None → return (no spec)
        │
        ├── estimator.estimate_confidence() → SegmentEstimate
        │     confidence < 0.6 → return (low confidence, log INFO)
        │
        └── asyncio.create_task(_run_pregen_one(pregen, est))
              │
              ├── tts_node._synthesize_yandex/silero/etc(pregen.next_ssml, …)  [existing TTS code, not modified]
              │     raise → log WARNING, return (pregen failed, treated as if no pregen)
              │
              ├── quality.check_audio_quality(audio, sr, est.duration_ms) → QualityVerdict
              │
              └── decision.decide(verdict, confidence) → Decision
                    ACCEPT → _results[next_speech_id] = PreGenResult(...)
                    REJECT → log WARNING, не кладём

[позже, при приходе chunk k+1 в dialogue_callback]
  │
  ├── chunk k+1 → обычный путь синтеза
  │
  └── проверить _PrefetchEngine.speculative_executor.claim(k+1.speech_id)
        │
        ├── PreGenResult есть И dialogue_id не сменился → вставить в _synthesize_and_play
        │   как «pre-baked audio» (минуя обычный синтез; см. §3.4 ниже)
        │
        └── None или stale → обычный синтез k+1 (путь не меняется)
```

### 3.4 Интеграция в `tts_node._synthesize_and_play`

Минимально-инвазивно: новое kw-arg `prebaked_audio: Optional[np.ndarray] = None`. Если не None — пропускаем блок синтеза, идём прямо в `_publish_audio(prebaked_audio, …)`. Существующая сигнатура метода остаётся обратно-совместимой.

```python
def _synthesize_and_play(
    self, ssml, text, dialogue_id, ssml_attributes,
    speech_id, batch_id, batch_index, batch_total,
    play_seq=None, voice=None, language=None,
    prebaked_audio: Optional[np.ndarray] = None,   # NEW
):
    ...
    if prebaked_audio is not None:
        audio = prebaked_audio
        sample_rate = 22050
        # пропускаем synth, идём в publish
    else:
        audio = self._run_provider_chain(ssml, …)   # existing path
        sample_rate = ...
    ...
```

### 3.5 Cancellation (REPLACE / barge-in / dialogue switch)

При **любом** из этих событий `TTSNode.cancel_pregen(reason=…)`:

1. Смена `self.current_dialogue_id` (уже ловится в `dialogue_callback`, строки ~1758).
2. `self._interrupt_playback()` (уже есть, строка 1602).
3. STOP через `/voice/tts/control`.
4. Любая REPLACE-семантика (`voice_input_mode` или новый priority-флаг 7a).

`cancel_pregen` вызывает `speculative_executor.cancel(reason)`, который:
- `cancel_event.set()` (как в существующем `SpeculativePreGenerator.cancel`)
- `asyncio.Task.cancel()` на каждой in-flight задаче
- `_results.clear()` (да, **clear**, а не keep — потому что у нас смена `dialogue_id` = весь диалог становится stale, и cache-hits опасны)
- счётчик `cancelled_count` ++ для метрик

### 3.6 Quality gate (3 эвристики — DoD #3 issue #2003)

| Эвристика | Порог | Что ловит |
|---|---|---|
| `duration_ratio` = `actual_ms / estimated_ms` | ≤ 1.25 | Обрезанный чанк (Yandex обрезал длинный SSML) |
| `silence_ratio` = доля тишины в начале + конце | ≤ 0.15 | Чанк с пустым началом или концом (LLM сгенерил «…», но TTS его проглотил) |
| `rms_low` = `audio_energy_db < -40 dBFS` | False | Совсем тихий чанк (Silero на не-кириллице) |

Эти эвристики **не** претендуют на полноценный audio quality gate (это была бы задача для отдельного ADR). Но они — достаточный первый барьер: чанк, который явно звучит плохо, **не** попадёт в `_results` и не заменит свежий синтез.

**Ограничение:** для случаев, когда все три эвристики прошли, но чанк всё равно звучит плохо (например, неправильное ударение, артефакт на стыке) — этого ADR-уровневый gate не ловит. Решение: метрика `pregen_acceptance_rate` (см. §3.7), при которой аномалия > 5% = re-tune.

### 3.7 Метрики (DoD #2 — raw-цифры)

```python
class PreGenMetrics:
    kickoffs_total: int = 0          # сколько раз вызвали pregenerate()
    pregens_scheduled: int = 0       # сколько asyncio.Task запустили
    pregens_completed: int = 0       # сколько audio реально синтезировано
    pregens_rejected_quality: int = 0  # отбраковано quality gate
    pregens_rejected_confidence: int = 0  # не запущено из-за low confidence
    pregens_stale: int = 0           # claim когда dialogue_id сменился
    pregens_claimed: int = 0         # cache hit при синтезе k+1
    pregens_bypassed: int = 0        # cache hit + использован для воспроизведения
    latency_chunk_to_chunk_ms_total: float = 0.0  # для среднего
    latency_chunk_to_chunk_ms_count: int = 0
```

Публикация: `/voice/tts/metrics` (новый топик, ~10 LOC в `tts_node`). DoD #2 — сырое число `latency_chunk_to_chunk` (mean) в issue-комменте после реализации.

---

## 4. Почему НЕ «тонкий адаптер к существующему `SpeculativePreGenerator`»

Это решение повторяет отказ из предыдущей попытки (`a1b823ce2` §4) и фиксирует его как часть ADR. Три причины — все по факту кода:

1. **Уровень абстракции.** `SpeculativePreGenerator.build_plan(candidates, llm_eta_ms, safety_margin_ms)` (`pre_gen.py:201`) вычисляет **freeze-boundary** по cumulative duration'ам. Это уровень scheduler'а — у него `candidates: Sequence[PreGenCandidate]` где `candidate.task_id` — это scheduler-task-id. Наш уровень — **один чанк** с уже-известным SSML. Между этими уровнями — пропасть: у scheduler'а нет «следующего чанка», у нас нет «boundary». Адаптер = одна модель, имитирующая другую. Это шим.

2. **Cancellation contract.** Существующий `SpeculativePreGenerator.cancel` (`pre_gen.py:360`) сохраняет `_results` (claimed-кэш) — `«Cached payloads from already-completed pre-gens are kept — the caller can still :meth:\`claim\` them if useful»`. Это **неправильно** для нашего случая: при смене `dialogue_id` cache-hit опасен (см. §3.5). Делать адаптер, который **отменяет** этот контракт — хуже, чем новый модуль.

3. **Quality gate.** Существующий `quality.py` (402 LOC) — это `EstimatorQualityTracker` (EMA, MAPE, calibration bins для **предсказаний длительности**). `scheduler.quality.EstimatorSample` — это `(estimated_ms, actual_ms, confidence, outcome)`. Это **не** audio quality gate. Использовать его как audio gate — категориальная ошибка. Нужен новый модуль.

**Прямо сейчас:** существующий `scheduler/{pre_gen,speculative_executor,quality,estimator}.py` остаётся в покое (он для scheduler-уровня; когда `dialogue_node` дойдёт до интеграции — будет работать). Мы **не** трогаем эти 5 файлов, **не** ломаем их API, **не** пытаемся переиспользовать. Делаем рядом новый слой для другого уровня абстракции.

---

## 5. Definition of Done (синхронизировано с issue #2003)

- [ ] **Реализация в `tts_node`:** `grep -n "def pregenerate\|def claim_pregen\|def cancel_pregen" src/rob_box_voice/rob_box_voice/tts_node.py` даёт ≥ 3 совпадения.
- [ ] **Все 5 модулей в `scheduler/pregen/` созданы и импортируются:** `__init__.py` реэкспортирует `PreGenTask`, `QualityVerdict`, `Decision`, `SegmentEstimate`, `SpeculativeExecutor`; `tts_node` импортирует их и создаёт `_PrefetchEngine` (содержащий `SpeculativeExecutor`) в `__init__`.
- [ ] **Baseline не сломан:** без `pregenerate` (поле отсутствует в payload) — поведение строго прежнее; юнит-тесты на `_submit_synthesis`/`_synthesize_and_play` зелёные; `prebaked_audio=None` путь не отличается ни в одной строчке логики от текущего.
- [ ] **Метрики:** `latency_chunk_to_chunk` (mean) публикуется в `/voice/tts/metrics`, baseline vs with-pregenerate на одном и том же assistant turn'е — измеримое уменьшение ≥ 200 мс для коротких чанков, ≥ 400 мс для длинных. Raw-цифры в issue-комменте после e2e.
- [ ] **Quality gate:** unit-тест `test_prefetch_quality_rejects_below_threshold` зелёный; e2e-сценарий «5 чанков подряд с pregenerate» воспроизводится без явных артефактов (визуальная проверка waveform в `plots/`).

---

## 6. Скоуп и не-в-этой-карточке

### 6.1 В этой карточке (контракт)

- §3.1 типы данных
- §3.2 сигнатуры модулей
- §3.3 поток данных (sequence)
- §3.4 точка интеграции в `_synthesize_and_play` (kwarg `prebaked_audio`)
- §3.5 cancellation contract
- §3.6 quality gate (3 эвристики, достаточно для DoD #3)
- §3.7 метрики

### 6.2 НЕ в этой карточке

- Реализация в `tts_node` — это карточка `t_fe8facbe` (developer).
- Расширение payload в `dialogue_node` — это карточка `t_9798710b` (developer, blocked).
- Зависимость 7a (issue #1996, priority в tts_node) — отдельная карточка, ещё ОТКРЫТА.
- Оптимизация латентности для не-TTS каналов (анимация, музыка).
- «Полноценный» audio quality gate с обучением на фидбеке — это будущая ADR.

### 6.3 Почему новая поддиректория `scheduler/pregen/`, а не `tts_node/pregen/`

`scheduler/` — нейтральная территория: модули там не зависят от rclpy и могут импортироваться из `tts_node` (и, в будущем, из других нод). `tts_node/` — конкретная ROS-нода. Если pre-gen-логика понадобится, например, `wake_stream.py` для синтеза wake-ответа — лучше иметь её в `scheduler/pregen/`, чем тащить из `tts_node/` через приватный API. `pregen/` (а не `_pregen/`) — публичная поддиректория с явной границей: «всё здесь — спекулятивный синтез, который может вызывать кто угодно».

### 6.4 Минимальный тест-план для implementer'а (карточка `t_fe8facbe`)

1. `test_pregen.py::test_build_pregen_task_none_when_no_next` — нет `ctx.recent_texts` → None.
2. `test_pregen.py::test_build_pregen_task_returns_when_next_available` — есть next → PreGenTask с правильным `next_speech_id`.
3. `test_estimator.py::test_confidence_floor_filter` — confidence < 0.6 → не запускаем asyncio.Task.
4. `test_quality.py::test_rms_low_rejects` — синтетический тихий аудио → REJECT_SILENT.
5. `test_quality.py::test_duration_ratio_rejects_clipped` — actual_ms > 1.25 × estimated_ms → REJECT_CLIPPED.
6. `test_quality.py::test_silence_ratio_rejects_trimmed` — silence_ratio > 0.15 → REJECT_TRIMMED.
7. `test_decision.py::test_decide_accept_only_on_pass_and_floor` — матрица 4×2.
8. `test_speculative_executor.py::test_kickoff_claim_roundtrip` — kickoff → wait → claim → аудио совпадает.
9. `test_speculative_executor.py::test_cancel_clears_results` — cancel → claim → None.
10. `test_tts_prefetch_integration.py::test_synthesize_and_play_with_prebaked_skips_synth` — kwarg `prebaked_audio` путь не вызывает provider chain (mock проверить).

---

## 7. Открытые вопросы

1. **Кто публикует `pregenerate` в payload.** `dialogue_node` (глядя на следующий чанк в `task_delta`) или новый driver в `scheduler/pre_gen.py`? Решается в карточке `t_9798710b` после merge 7a. **Не блокирует контракт** — поле опциональное.
2. **Один активный pre-gen или очередь.** Предлагаю один (FIFO по `next_speech_id`). Очередь пре-генов создаст «скрытый канал» для REPLACE-отмены — лишняя поверхность для багов. Подтверждается при первом e2e. (Совпадает с решением предыдущей попытки `a1b823ce2` §7 Q2.)
3. **Что делать, если `confidence < FLOOR`, но чанк всё равно можно синтезировать.** Сейчас — пропуск (return). Альтернатива — `pregens_rejected_confidence_total += 1` и запуск всё равно. Решается метриками после e2e.

---

## 8. Связанное

- [issue #2003](https://github.com/krikz/rob_box_project/issues/2003) — task
- [issue #1996](https://github.com/krikz/rob_box_project/issues/1996) — dep 7a, открыт
- [issue #1004](https://github.com/krikz/rob_box_project/issues/1004) — оригинальный комментарий-заглушка `pregenerate: false`
- `docs/architecture/target-operator-agent-and-dialogue.md` §8а.4 п.3, §13 Шаг 13
- `docs/plans/2026-09-05-operator-agent-architecture-handoff.md` §4.8
- `docs/design/SCHEDULER_DESIGN.md` §0.0 (TL;DR — статус планировщика), §6.5 (runtime-budget и спекуляция на уровне scheduler'а — **другая** абстракция, см. §4)
- `src/rob_box_voice/rob_box_voice/tts_node.py:1725` `dialogue_callback` — точка расширения
- `src/rob_box_voice/rob_box_voice/scheduler/{pre_gen,quality,estimator,decision,speculative_executor}.py` — НЕ переиспользуем (см. §4), остаются для scheduler-уровня
- коммит `a1b823ce2` — предыдущая попытка (на ветке `z-{agent}/2003-operator-agent-13-pregenerate-tts-node`, не слита), `docs/architecture/adr/pregenerate-in-tts-node-contract.md` — учтена в §4