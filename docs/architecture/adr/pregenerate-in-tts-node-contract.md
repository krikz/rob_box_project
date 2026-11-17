# Контракт `pregenerate` в `tts_node` — оператор-агент, Шаг 13

> **Статус:** proposal v0.1 (kanban t_9798710b, 2026-09-07). Ждёт разблокировки
> зависимости 7a (issue #1996, открыта). После её merge — переходит в
> implementation-ready.

## 0. Контекст и почему сейчас

- Задача: [issue #2003](https://github.com/krikz/rob_box_project/issues/2003)
  «[operator-agent 13] Спекулятивная генерация (pregenerate в tts_node)».
- Целевая архитектура:
  [`docs/architecture/target-operator-agent-and-dialogue.md`](../target-operator-agent-and-dialogue.md)
  §8а.4 (пункт 3 «Спекулятивная генерация — в конце, отдельной карточкой») и
  §13 Шаг 13 («Требует реализации `pregenerate` в `tts_node`... Последним,
  после того как базовый путь оператора заработает»).
- Хендофф: [`docs/plans/2026-09-05-operator-agent-architecture-handoff.md`](../../plans/2026-09-05-operator-agent-architecture-handoff.md)
  §4.8 «`pregenerate` не существует».
- Зависимость: [issue #1996](https://github.com/krikz/rob_box_project/issues/1996)
  «[operator-agent 07a] Приоритет в `tts_node`: поле `priority` в
  `/voice/tts/request`» — **открыт**.

## 1. Зачем (бизнес-сценарий)

Между завершением чанка *k* и началом чанка *k+1* одного assistant turn'а
сейчас существует окно «синтезатор простаивает, динамик тоже» —

```
[chan k синтез]→[chan k в динамик]→[тишина, синтез k+1]→[chan k+1 в динамик]
                                      ^^^^^^^^^^^^^^^^^^^
                                      это окно и есть target pregenerate
```

Спекулятивная генерация запускает синтез следующего чанка **до** того, как
`batch_complete` текущего батча зафиксирован, и кладёт готовое аудио в
короткую очередь на динамике. Окно простоя схлопывается до латентности
одного только «принять готовое аудио и отправить в ALSA».

Это **не** про ускорение LLM. Это про TTS-канал.

## 2. Что НЕ делаем (анти-goal, прямо из §8а.4 п.3)

- Не оптимизируем латентность до того, как заработает базовый путь оператора
  (шаги 04a–07a). Это архитектурный запрет, не технический.
- Не превращаем `tts_node` в планировщик. Очередь — **приоритетная**, но
  локальная (один процесс, один динамик). `TaskScheduler` живёт в
  `dialogue_node`; `tts_node` его не знает и знать не должен.
- Не пишем собственный эстиматор/quality — переиспользуем существующие
  `scheduler/estimator.py` / `scheduler/quality.py`. Они уже на develop, но
  нигде не конструируются (`scheduler/__init__.py` экспортирует, вызовов нет).
  Это меняется: `tts_node` будет **первым** реальным вызывающим.

## 3. Контракт API (минимально-инвазивный)

### 3.1 Поле в `/voice/tts/request`

Текущий payload (см. `tts_node.dialogue_callback`, строки ~1750–1810):
```json
{
  "speech_id": "uuid",
  "dialogue_id": "uuid",
  "ssml": "<speak>...</speak>",
  "batch_id": "...",
  "batch_index": 0,
  "batch_total": 3,
  "voice": "anton",
  "language": "ru"
}
```

**Добавление** (под 7a — поле `priority` уже появится):
```json
{
  ... всё, что было,
  "priority": "normal" | "operator" | "personality",
  "pregenerate": {                                  // ← НОВОЕ
    "next_speech_id": "uuid",                       // заранее зарезервированный id
    "next_ssml": "<speak>...</speak>",              // текст чанка k+1
    "voice": "anton",
    "language": "ru"
  }
}
```

`pregenerate` — **опциональное** вложенное поле. Его наличие означает
«параллельно с обработкой текущего чанка начни синтез следующего».
Отсутствие — поведение прежнее.

### 3.2 Где живёт в `tts_node`

Новый класс **в том же файле** (4198 LOC уже, +1 модуль = перебор; держим
инкапсуляцию через `_PrefetchEngine` dataclass внутри `TTSNode.__init__`):

```
TTSNode
  ├── ... существующие поля ...
  ├── _prefetch: _PrefetchEngine | None  # NEW, лениво создаётся при первом pregenerate
  └── ...
```

`_PrefetchEngine` хранит:
- `asyncio.Task` синтеза `next_speech_id`
- `dict[speech_id, audio_bytes]` — готовые аудио-чанки
- `quality_verdict` от `scheduler.quality.QualityGate` (отбраковка)
- `cancel_event` — общий с `SpeculativePreGenerator.cancel` (если бы тот
  был сконструирован, но это **не** входит в скоуп; см. §6)

### 3.3 Событийный контракт

`pregenerate` — это **запрос** от LLM/оператора. Кто именно его публикует —
выходит за рамки этой карточки; предполагается, что это сделает либо
`dialogue_node` (глядя на следующий чанк в `task_delta`), либо отдельный
`scheduler/pre_gen.py` driver внутри `dialogue_node`. Главное:

- `tts_node` **не решает**, что пре-генерировать. Ему дают готовый текст.
- `tts_node` **не решает**, отбраковать ли результат. Это делает
  `scheduler.quality.QualityGate` через `speculative_executor`/`pre_gen`.

### 3.4 Lifecycle

```
   dialogue_node видит batch_complete(k, total=K)
                  │
                  ▼ публикует /voice/tts/request с
                  │   - chunk K (обычное поле ssml)
                  │   - pregenerate{next_speech_id, next_ssml}
                  │
   tts_node.dialogue_callback
                  │
                  ├── chunk K → обычный путь синтеза
                  │
                  └── _prefetch.start(next_speech_id, next_ssml)
                          │
                          ├── asyncio.Task: synth(next)
                          │     └── по завершении: quality.gate(audio) →
                          │         • PASS  → _prefetch.results[id] = audio
                          │         • FAIL  → отбрасываем, лог WARNING
                          │
                  ▼
   tts_node доигрывает chunk K
                  │
                  ├── если _prefetch.results[K+1] готов и
                  │   dialogue_id не сменился → отдать в AudioPlaybackManager
                  │   СРАЗУ (минуя обычный синтез)
                  │
                  └── иначе → обычный синтез K+1
```

### 3.5 Cancellation (REPLACE / barge-in / dialogue switch)

При **любом** из этих событий `_prefetch.cancel()`:
1. Смена `self.current_dialogue_id` (уже ловится в `dialogue_callback`,
   строки ~1758).
2. `self._interrupt_playback()` — уже есть.
3. `voice_input_mode` сменился на REPLACE.
4. Получен `/voice/tts/control` STOP.

`cancel()` — `asyncio.Task.cancel()` + чистка `results` dict.

### 3.6 Качество (не деградирует — DoD #3)

Перед тем как положить результат в `results`, прогоняем через
`scheduler.quality.QualityGate` (существующий, 656 LOC):

- `duration_ratio` — отклонение длительности от эстиматора больше 25%? → reject
- `silence_ratio` — тишина в начале/конце > 15%? → reject
- `rms_low` — энергия ниже порога? → reject

Это **обязательный** шаг — DoD issue'а прямо говорит «качество не
деградировало». Без `quality` шаг pregenerate опасен: тихий или
обрезанный чанк хуже, чем задержка.

### 3.7 Метрики (DoD #2 — raw-цифры)

При включённом pregenerate измеряем:

```
latency_chunk_to_chunk = T_play_start(K+1) - T_play_end(K)
```

Ожидание: уменьшение на **≥ 200 мс** для коротких чанков (≤ 5 сек), на
**≥ 400 мс** для длинных (≥ 10 сек). Метрика публикуется в `/voice/tts/metrics`
(топик нужно создать, ~10 LOC).

Сравнивается baseline (pregenerate=off) → with pregenerate=ON на том же
ассистентском turn'е.

## 4. Почему не «адаптер к существующему scheduler/pre_gen.py»

Предыдущая попытка воркера (kanban t_9798710b, attempt #5) предлагала
«тонкий адаптер к SpeculativePreGenerator». Это **неправильно** по трём
причинам:

1. `SpeculativePreGenerator` работает на **уровне scheduler** (иерархия
   задач, MERGE/REPLACE/IGNORE), а TTS-чанк живёт на уровне **аудио-чанка**.
   У них разные «единицы прерывания».
2. `SpeculativePreGenerator` — asyncio-задача в `dialogue_node`. Передавать
   ему `speech_id`/аудио через границу процесса — лишний IPC-удар и новый
   топик; проще держать `_PrefetchEngine` локально в `tts_node`.
3. Quality-gate — общий, и его переиспользуем как библиотеку, **не** как
   сервис.

## 5. Definition of Done (синхронизировано с issue #2003)

- [ ] `pregenerate` реализован в `tts_node` (grep по `_PrefetchEngine`,
      `async def _start_prefetch`, `pregenerate.*next_speech_id`).
- [ ] Спекулятивный чанк доходит до динамика **раньше**, чем
      `batch_complete` текущего батча. Замер: метрика
      `latency_chunk_to_chunk` уменьшилась, raw-цифры в issue-комменте.
- [ ] Качество не деградировало: unit-тест
      `test_prefetch_quality_rejects_below_threshold` зелёный;
      e2e-сценарий «5 чанков подряд с pregenerate» воспроизводится без
      артефактов в аудио.

## 6. Скоуп: что НЕ входит в эту карточку

- Конструирование `SpeculativePreGenerator` (он остаётся библиотечным
  модулем, как сейчас). Это отдельная задача.
- Изменения в `dialogue_node` для публикации `pregenerate` в payload —
  это **тоже** отдельная карточка, потому что зависит от 7a.
- Оптимизация латентности для не-TTS каналов (анимация, музыка).

## 7. Открытые вопросы

1. **Кто публикует `pregenerate`** — `dialogue_node` (глядя на `task_delta`)
   или новый driver в `scheduler/pre_gen.py`? Решается **после** 7a, при
   старте имплементации. Не блокирует контракт.
2. **Один активный pre-gen или очередь?** Предлагаю один (FIFO по
   `next_speech_id`). Очередь пре-генов создаст «скрытый канал» для
   REPLACE-отмены — лишняя поверхность для багов. Подтверждается при
   первом e2e.

## 8. Связанное

- [issue #2003](https://github.com/krikz/rob_box_project/issues/2003) — task
- [issue #1996](https://github.com/krikz/rob_box_project/issues/1996) — dependency 7a
- [issue #1004](https://github.com/krikz/rob_box_project/issues/1004) — оригинальный комментарий-заглушка `pregenerate: false`
- `docs/architecture/target-operator-agent-and-dialogue.md` §8а.4, §13
- `docs/plans/2026-09-05-operator-agent-architecture-handoff.md` §4.8
- `docs/design/SCHEDULER_DESIGN.md` §0.0 (TL;DR), §11.6 (статус реализации)
