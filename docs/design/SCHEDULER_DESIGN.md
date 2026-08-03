# Scheduler: intent-based execution вместо fire-and-forget

| Поле | Значение |
|------|----------|
| Документ | `docs/architecture/SCHEDULER_DESIGN.md` |
| Связанное | Issue [#968](https://github.com/krikz/rob_box_project/issues/968), PR #907, #935 |
| Статус | **Architecture proposal** — ready for review |
| Автор документа | Architect profile (kanban t_8f5bb012) |
| Дата | 2026-08-03 |
| Основание | Анализ issue #968 целиком + все 15 комментариев + аудит текущего кода |

---

## 0. TL;DR

Между LLM (которая выдаёт `tool_call`) и исполнителями (TTS, музыка, анимация) **нет слоя, который знает про порядок, границы и состояние каналов**. Результат — `stop_music` обгоняет `speak_text`, LLM не знает что TTS ещё играет, пользователь слышит обрубки.

**Решение:** ввести `TaskScheduler` — единая точка прохождения всех `tool_call`-ов и системных событий. Scheduler владеет каналами (voice, music, anim) и сегментами (атомарные единицы исполнения). LLM остаётся «дирижёром»: генерит намерения и видит состояние, scheduler — «оркестр»: исполняет по правилам.

**Ключевой принцип (из итоговой модели, согласовано в #13):** тулы от LLM идут в scheduler мгновенно (LLM не блокируется). Scheduler **не задерживает конструктивные** операции (`speak_text`, `play_animation` — встают в очередь канала), а **задерживает деструктивные** (`stop_music`, REPLACE, cancel) — до естественной границы voice-канала.

**Что УЖЕ есть в коде** (см. §11): вызовы `speak_text` уже блокируются через `_output_lock` + `await tts/finished` (dialogue_node.py:672–702). То есть половина исходной проблемы #968 уже решена. Осталось: (1) классификация ввода MERGE/REPLACE/QUEUE/IGNORE/CLARIFY до barge-in (блокер П1), (2) сегментная модель + PENDING-правки, (3) speculative pre-generation, (4) единый EventBus для системных событий.

---

## 1. Бизнес-проблема и сценарий

### 1.1 Проблема (verbatim из #968)

LLM вызывает инструменты как «пулемёт»: `speak()`, `play_music()`, `stop_music()` улетают мгновенно, без ожидания. TTS встаёт в очередь, LLM молотит дальше и дёргает `stop_music()` — хотя речь ещё не начала произноситься. Инструменты не синхронизированы.

**Симптомы в e2e (v28–v38):**
- v36: `stop_music()` через 3с после начала рэпа → музыка отвалилась на середине фразы
- v38: cleanup срабатывал на первом TTS-чанке (некорректный сигнал «TTS готов»)
- Костыли (3с-таймер, debounce, «запретить stop_music в промпте») лечат симптом, не корень

**Корень:** между LLM и исполнителями нет планировщика. LLM генерит `tool_call`-ы, они вываливаются на железо в произвольном порядке и темпе.

### 1.2 Целевой сценарий («комар + енот»)

```
Пользователь: "Спой песню про комара"
Робот:        [генерирует и начинает петь, куплет 1]
Пользователь: "И ещё про енота!"        ← во время куплета 1
Робот:        [куплет 1 допевает до конца]
              [куплет 2 уже про енота — без паузы, без тишины]
              [пользователь НЕ слышит разрыва]
```

### 1.3 Зачем это нужно

Робот-ассистент, который не умеет «договаривать» и «вплетать» — это набор костылей вокруг каждого нового сценария. Длинные творческие задачи (рэп, рассказ, обучающий монолог) требуют осознанной координации каналов и состояния. Без планировщика каждая правка — это race-condition hunt по логам.

---

## 2. Сегментная модель

### 2.1 Определения

**Задача (Task)** — намерение пользователя, которое LLM разложила в последовательность действий. Уникальный `task_id`, тип (`sing`, `rap`, `narrate`, `navigate`), параметры (`topics`, `duration_hint`).

**Сегмент (Segment)** — атомарная единица исполнения внутри задачи. Имеет:
- `idx` — порядковый номер
- `kind` — `voice` | `music` | `anim` | `silence`
- `payload` — что именно (текст, midi-паттерн, анимация)
- `status` — `PENDING` → `ACTIVE` → `COMPLETED` / `CANCELLED` / `SKIPPED`
- `eta_ms` — оценка длительности (см. §6)

### 2.2 Пример

```
Песня про комара:
  [seg_1: "куплет про комара"     — ACTIVE,  12с]
  [seg_2: "припев"               — PENDING,  8с]
  [seg_3: "куплет 2 про комара"  — PENDING, 12с]
```

### 2.3 Правила обновления (INVARIANT)

**`update(delta)` модифицирует ТОЛЬКО сегменты со статусом PENDING.** ACTIVE сегмент доигрывает до естественной границы.

Пример MERGE:
```
update("добавь енота"):
  [seg_1: "куплет про комара"     — ACTIVE,  НЕ ТРОГАЕМ]
  [seg_2: "припев про обоих"     — PENDING → переписан]
  [seg_3: "куплет про енота"     — PENDING → заменён]
  [seg_4: "общий финал"          — PENDING → добавлен]
```

**Этот инвариант — фундамент.** Без него любая правка «на лету» ломает воспроизведение. Покрывается unit-тестом (см. §10 acceptance).

### 2.4 Почему сегменты, а не «один большой task»

LLM-цикл занимает 1.5–3с. TTS-чанк — 2–5с. Между ними нужна точка синхронизации, которая:
- не блокирует LLM-цикл на синтез (иначе «кома» из дилеммы)
- не позволяет деструктивным операциям обогнать конструктивные
- позволяет правкам применяться к будущему, не к звучащему

Сегмент = минимальная единица, которая:
- имеет чёткую границу (TTS-чанк = предложение, music = такт, anim = ключевой кадр)
- может быть отменена/перезаписана, если ещё не активна
- может быть допита до конца, если уже активна

---

## 3. Каналы (Channels)

### 3.1 Модель

Scheduler владеет набором каналов. **Каждый канал = FIFO-очередь + state-машина**.

| Канал | Содержимое | Граница сегмента | Длительность |
|-------|------------|------------------|--------------|
| `voice` | SSML/текст → TTS | Конец предложения | 1–3с |
| `music` | MIDI-паттерн → audio_node | Конец такта/фразы | 2–4с |
| `anim` | Animation preset | Конец ключевого кадра | 0.5–2с |
| `led` / `expression` | Мимика, свет | Мгновенная | <100мс |

Каналы **независимы** — это критично:
- музыка может играть фоном, пока voice говорит
- анимация может идти параллельно с любым каналом
- `led` всегда исполняется мгновенно (не блокирует)

### 3.2 Что встаёт в очередь vs что исполняется сразу

| Операция | Канал | Поведение |
|----------|-------|-----------|
| `speak_text(t)` | voice | В очередь (FIFO). `speak_text` блокируется до `tts/finished`. |
| `play_sound(name)` | voice (или отдельный `sfx`) | В очередь, блокируется до `sound/ready` |
| `play_animation(preset)` | anim | В очередь |
| `execute_music_code(...)` | music | В очередь, длинный (LONG в терминах async_executor) |
| `stop_music(...)` | music | **НЕ в очередь как обычная команда** — см. §4 |
| `set_vibe_preset(...)` | led/expression | INSTANT, fire-and-forget |
| `set_emotion(...)` | expression | INSTANT, fire-and-forget |

### 3.3 Уже реализовано

В текущем `dialogue_node.py:672–702`:

```python
async def speak_text(text: str, animation: str = "neutral") -> str:
    ...
    async with lock:  # ← _output_lock, сериализует speak/play_sound/animation
        result_str = await _call("speak_text", {...}, timeout=60.0)
        # ↓↓↓ ждём пока TTS реально договорит ↓↓↓
        speech_id = json.loads(result_str).get("data", {}).get("speech_id", "")
        if speech_id:
            event = asyncio.Event()
            self._tts_events[speech_id] = event
            await asyncio.wait_for(event.wait(), timeout=30.0)
```

Это значит: **основной race «stop_music vs speak» уже частично решён** — потому что пока `speak_text` ждёт `tts/finished`, LLM-цикл не двигается дальше. Проблема остаётся в `execute_music_code` + параллельных вызовах (см. §10 acceptance, регресс v36).

---

## 4. MERGE / REPLACE / QUEUE / IGNORE / CLARIFY

### 4.1 Решения по новому вводу

Планировщик принимает один из пяти вердиктов на любое событие (пользовательский ввод ИЛИ системное событие):

| Решение | Что делает | Когда |
|---------|-----------|-------|
| **MERGE** | Правка применяется к PENDING-сегментам. ACTIVE не трогается. | «и ещё про енота», `battery_critical` |
| **REPLACE** | Очередь PENDING сбрасывается, новая задача стартует после текущего сегмента. | «хватит, расскажи анекдот», `obstacle_ahead` (critical) |
| **QUEUE** | Текущая задача до конца, новая становится в общую очередь задач. | «а потом спой колыбельную» |
| **IGNORE** | Ничего не делаем. | «угу», кашель, шум, повторный «стоп» |
| **CLARIFY** | Генерим уточняющий вопрос (LLM, короткий). | «про него» (неоднозначная ссылка) |

### 4.2 Двухуровневое решение (< 800мс)

**Уровень 1 — правила (< 50мс):**
- Короткие реплики с союзами («и ещё», «а также», «а потом») → MERGE / QUEUE
- Императивы («хватит», «стоп», «замолчи») → REPLACE
- Шум, междометия, < 2 слов → IGNORE
- `confidence > 0.9` → решение принято, LLM не дёргаем

**Уровень 2 — лёгкая/быстрая LLM (< 800мс):**
- Вызывается только если уровень 1 не уверен
- Structured output: `{decision, task_delta, confidence, reasoning}`
- **Основная (тяжёлая) LLM для этого НЕ вызывается**

**Почему не основная LLM:**
- latency: лёгкая 200–400мс vs тяжёлая 1.5–3с (для MERGE/REPLACE критичны < 800мс)
- стоимость: дёргать дорогую модель на каждый «угу» — расточительно
- простота промпта: классификация из 5 вариантов не требует тяжёлого контекста

**Ограничение на параллельные LLM-вызовы** — rate limit MiniMax (concurrent requests). Планировщик учитывает его при постановке в очередь.

### 4.3 ПРИОРИТЕТЫ СОБЫТИЙ

| Приоритет | Источники | Эффект |
|-----------|-----------|--------|
| `critical` | `obstacle_ahead`, перегрев, потеря сети | REPLACE + сокращение ACTIVE до ближайшей границы |
| `high` | `battery_critical`, «хватит», пользовательский imperative | MERGE-срочный (ввернуть в следующий PENDING) или REPLACE |
| `normal` | голос пользователя, обычные `Hermes`-команды | Стандартный MERGE/REPLACE/QUEUE |
| `low` | шум, междометия, повторы | IGNORE |

`critical > high > normal > low`. Конфликт разрешается по приоритету, при равенстве — по времени поступления.

### 4.4 Разрешение блокера П1 (barge-in vs MERGE)

**Аудит #15 (комментарий 15) выявил блокер:** текущий код (`dialogue_node.py:1731`) при любом новом вводе вызывает `_cancel_run` → `asyncio.CancelledError` → LLM-цикл умирает. А сценарий «комар + енот» требует, чтобы LLM-цикл ЖИЛ и переписывал PENDING-сегменты.

**Решение (фиксируем в этом документе):**

```
новый ввод → уровень 1 (правила) → решение {MERGE|REPLACE|QUEUE|IGNORE|CLARIFY}
                                    ↓
                              если MERGE/QUEUE/CLARIFY:
                                НЕ отменять LLM-цикл
                                событие уходит в scheduler как MERGE/QUEUE/CLARIFY
                                LLM на следующем ходу видит [PENDING EVENTS]
                                и выдаёт task_delta → планировщик правит PENDING
                              если REPLACE:
                                отменить LLM-цикл (barge-in)
                                дождаться завершения ACTIVE-сегмента (≤3с для voice)
                                запустить новую задачу
                              если IGNORE:
                                ничего
```

**Ключевое:** классификация MERGE-vs-REPLACE происходит ДО barge-in. Сейчас этого нет — это и есть блокер сценария «енот».

**Реализация:** заменить прямой вызов `_cancel_run` в триггере нового ввода на:
```python
async def on_new_input(text: str, source: str):
    decision = await quick_decide(text, source)  # уровень 1 + уровень 2
    if decision.action == "REPLACE":
        self._cancel_run(reason="user_replace")
    elif decision.action in ("MERGE", "QUEUE", "CLARIFY"):
        self._scheduler.enqueue_event(Event(
            source=source, type="user_input",
            priority=decision.priority, payload={"text": text, "decision": decision}
        ))
```

---

## 5. EventBus — единая точка входа

### 5.1 Зачем

Из #968, раздел «Источники событий»:
> Триггером для MERGE/REPLACE/QUEUE может быть не только голос пользователя, но и любое системное событие. Робот живёт в среде: батарея, датчики, сообщения от Hermes, таймеры. Планировщик должен обрабатывать все источники одинаково.

### 5.2 Архитектура

```
┌─────────────────┐
│  ROS2 topics    │─────┐
│  /battery/*     │     │
│  /obstacle/*    │     │
│  /hermes/*      │     ▼
└─────────────────┘  ┌──────────┐    ┌─────────────────┐
                     │ EventBus │───▶│  TaskScheduler  │
┌─────────────────┐  │          │    │                 │
│  VoiceInput     │──┤ единая   │    │ - decisions     │
│  STT result     │  │ очередь  │    │ - segments      │
└─────────────────┘  │ событий  │    │ - channels      │
                     └──────────┘    └─────────────────┘
┌─────────────────┐       │                  │
│  Internal hooks │───────┘                  ▼
│  (LLM, Agent)   │              ┌──────────────────────┐
└─────────────────┘              │ ROS topic для монито-│
                                  │ ринга: /harness/    │
                                  │ task_events         │
                                  └──────────────────────┘
```

### 5.3 Формат события

```python
@dataclass
class Event:
    source: str           # "user_input" | "battery_monitor" | "hermes" | "navigation" | "internal"
    type: str             # "speech" | "battery_critical" | "obstacle_ahead" | "merge" | ...
    priority: Priority    # critical | high | normal | low
    payload: dict
    timestamp: float
    correlation_id: str | None  # для связи с активной задачей
```

### 5.4 Где живёт

EventBus — singleton в `dialogue_node` (или новый node `scheduler_node`). Не новый топик — внутренняя шина внутри `dialogue_node` для пользовательских событий, плюс ROS-topic подписчики для внешних (`/battery/*`, `/obstacle/*`, `/hermes/*`).

**Не плодим ROS-нод без нужды** — EventBus живёт в том же процессе, что и `dialogue_node`, общается с внешним миром через ROS-subscriber'ов.

---

## 6. Эстиматоры (SegmentEstimator + LLMEstimator)

### 6.1 Зачем

Из INSIGHT #11 (комментарий 14): **сегменты исполняются ПОКА LLM отвечает**. Время ответа LLM — не «пауза между ходами», а часть бюджета воспроизведения. Без учёта этого сценарий «енот без паузы» умирает на первой же правке:

```
куплет 1 закончился → 5–8 секунд тишины → куплет 2 про енота
```

«Пауза и рестарт», которые задача явно запрещает.

### 6.2 SegmentEstimator

Оценивает длительность конкретного сегмента:

| Канал | Метод | Точность |
|-------|-------|----------|
| `voice` | символы → секунды (есть тул `estimate_tts_duration`, см. v38 в логе) | ±15% |
| `music` | `duration_sec` из `execute_music_code` (уже есть в payload) | ±10% |
| `anim` | из preset-манифеста | ±5% |

**Реализация:** планировщик использует тот же механизм, что и существующий тул `estimate_tts_duration`. LLM НЕ обязана вызывать его руками — планировщик сам считает при создании сегмента (см. G5 из аудита #15).

### 6.3 LLMEstimator

EMA по таймингам завершённых ходов (request → response):

```python
class LLMEstimator:
    def __init__(self, default_ms=2500.0, alpha=0.3):
        self._ema_ms = default_ms
        self._alpha = alpha
    
    def record(self, latency_ms: float):
        self._ema_ms = self._alpha * latency_ms + (1 - self._alpha) * self._ema_ms
    
    @property
    def estimate_ms(self) -> float:
        return self._ema_ms
```

- Обновляется после КАЖДОГО хода — runtime-метрика, не промптовая
- Первые N ходов (например, 5) — дефолт 2.5с, дальше самообучение
- Видна в feedback events (§7)

### 6.4 Инвариант очереди

Голосовая очередь не должна опустеть раньше, чем придёт ожидаемый ответ LLM:

```
remaining_voice_time > estimated_llm_latency + tts_synthesis_time + safety_margin
```

Если инвариант нарушается — планировщик делает что-то ДО того, как динамики замолчат (см. §6.5).

### 6.5 Speculative pre-generation

**Пока звучит сегмент N, планировщик заранее синтезирует TTS сегмента N+1** (по текущему плану, без ожидания правки):

- **Момент старта:** осталось ≤ `(llm_latency + tts_synth)` до конца сегмента N
- **Пришла правка (MERGE):** незавершённый синтез отменяется через `InterruptibleTask.cancel()` (уже есть в `async_executor.py:23`)
- **Результат:** к моменту ответа LLM следующий сегмент уже готов — паузы нет

**Edge case (G3 из аудита #15):** MERGE во время пред-генерации → незавершённый синтез отменяется, перезапуск по новому плану. Покрыть unit-тестом.

**Если LLM всё-таки опаздывает:**
- Не тишина! Музыка/ambient продолжается (fill-сегмент)
- Голос молчит до готовности
- В feedback LLM уходит пометка «была пауза Nс из-за моей задержки» — самообучение для `LLMEstimator`

### 6.6 Что УЖЕ есть

- `async_executor.py` с классификацией INSTANT/FAST/MEDIUM/LONG и `InterruptibleTask.cancel()` (подтверждено в коде)
- Тул `estimate_tts_duration` (упоминается в v38 логе)

**Что нужно построить:**
- `LLMEstimator` (EMA — простой класс)
- Связка `speculative_pre_gen` с `InterruptibleTask`
- `SegmentEstimator` = обёртка вокруг существующего механизма

---

## 7. Feedback events (что scheduler отдаёт LLM)

Перед каждым ходом LLM видит блок состояния — иначе не знает, что TTS ещё играет (INSIGHT #8 → пост-амбл «Готово! Зачитал тебе...»).

### 7.1 Формат инъекции в LLM (system prompt перед следующим ходом)

```
[ACTIVE TASKS]
- id=t_001, type=sing, topics=["комар","енот"], progress=0.4,
  current="куплет 2/4", eta=45s,
  channels={music: playing, voice: speaking, anim: idle}

[CHANNELS]
- voice: queue=3 segs, current_seg=seg_2, current_eta=2.1s, last_finished=0.4s ago
- music: queue=1 segs, current_seg=mseg_1, playing_for=12.3s
- anim: queue=0 segs, last=play_animation:wave 0.8s ago

[PENDING EVENTS]
- battery_critical (source=hermes, priority=high): "батарея 12%, скоро еду на базу"

[ESTIMATORS]
- llm_latency_ema: 1850ms (n=23)
- last_turn_latency: 2103ms
```

### 7.2 Публикуемые события

```python
task.created(id, type, params)
task.started(id)
task.segment_started(id, seg_idx)
task.segment_completed(id, seg_idx)
task.updated(id, delta)
task.completed(id)
task.cancelled(id, reason)
```

**Куда:**
1. **В контекст LLM** — блок выше, инжектится перед каждым ходом
2. **В ROS2-топик `/harness/task_events`** — для мониторинга (ros2 topic echo, foxglove)
3. **В MemoryStore** — для post-mortem и истории диалогов

### 7.3 Связь с проблемой пост-амбла

INSIGHT #8: LLM добавляет «Готово! Зачитал тебе...» после рэпа. **Это не косметика — это прямое следствие отсутствия feedback.** Если LLM видит `channels={voice: speaking, eta=8s}` на финальном ходе, она не добавляет пост-амбл.

**Acceptance:** «пост-амбл исчезает при активном voice-канале» (unit-тест или e2e-прогон).

---

## 8. Точка интеграции с текущим кодом

### 8.1 Где встаёт scheduler

```
Сейчас:   LLM → tool_call → dialogue_node._execute_tool() → ROS topic
Станет:   LLM → tool_call → TaskScheduler.submit()
                       ↓
              ┌────────┴────────┐
              ▼                 ▼
         EventBus         Segments
              ↓                 ↓
         быстрый слой    каналы (voice/music/anim)
              ↓                 ↓
         decision          FIFO-очередь
              ↓                 ↓
         ROS topics      исполнители (tts_node, audio_node)
```

**Точка входа:** `dialogue_node.py:657` (`speak_text` function_tool) и аналогичные для `execute_music_code`, `stop_music`, `play_sound`, `play_animation`. Планировщик реализует тот же интерфейс, но внутри держит очередь и зависимости.

### 8.2 Что уже есть в коде (повторно, для ясности)

| Компонент | Файл | Статус |
|-----------|------|--------|
| `AsyncToolExecutor` с INSTANT/FAST/MEDIUM/LONG | `async_executor.py:136` | ✅ есть |
| `InterruptibleTask.cancel()` | `async_executor.py:23` | ✅ есть |
| Блокирующий `speak_text` через `_output_lock` + `tts/finished` await | `dialogue_node.py:672–702` | ✅ есть |
| `_tts_events` dict (speech_id → Event) | `dialogue_node.py:169` | ✅ есть |
| `_cancel_run` (отмена LLM-цикла при barge-in) | `dialogue_node.py:1786` | ✅ есть, но см. §4.4 |
| `_run_cancelled` flag (проверка в speak_text) | `dialogue_node.py:157–162` | ✅ есть |

### 8.3 Что нужно построить

| Компонент | Где | Размер |
|-----------|-----|--------|
| `TaskScheduler` (новый класс) | `dialogue_node.py` или отдельный модуль `scheduler.py` | M (~400 LOC) |
| `Segment` + `Task` dataclasses | `scheduler.py` | S (~80 LOC) |
| `Channel` (voice/music/anim FIFO + state) | `scheduler.py` | M (~200 LOC) |
| `EventBus` | `scheduler.py` | S (~100 LOC) |
| `SegmentEstimator` | `scheduler.py` | S (~80 LOC) |
| `LLMEstimator` (EMA) | `scheduler.py` | XS (~30 LOC) |
| Quick-decide уровень 1 (правила) | `scheduler.py` | S (~80 LOC) |
| Quick-decide уровень 2 (лёгкая LLM) | новый модуль или extension | M (~200 LOC) |
| Speculative pre-generation loop | `scheduler.py` | M (~150 LOC) |
| Feedback events в LLM-контекст | `dialogue_node.py` (модификация) | S (~50 LOC) |
| ROS-топик `/harness/task_events` | `scheduler.py` | XS (~30 LOC) |

**Итого:** ~1.4K LOC нового кода, ~50 LOC правки `dialogue_node.py`.

### 8.4 Совместимость с `async_executor`

Scheduler **не дублирует** `async_executor`. Использует его как низкоуровневый исполнитель:

- `speak_text` от LLM → scheduler ставит в voice-очередь → исполнитель через `AsyncToolExecutor` (FAST/MEDIUM) отправляет в tts_node
- `execute_music_code` → scheduler ставит в music-очередь → исполнитель через `LONG`-задачу с `InterruptibleTask`
- `stop_music` → scheduler не «исполняет» как обычную команду — см. §4 (REPLACE-flow)

---

## 9. Альтернативы, которые отвергли

| Альтернатива | Почему нет |
|--------------|-----------|
| Блокирующий tool (LLM ждёт TTS-чанк) | «Кома» робота на 30с, не слышит barge-in |
| Неблокирующий tool (текущий) | Гонки, `stop_music` обгоняет speak |
| Полный event sourcing (хранить каждое событие) | Overkill для робота-ассистента, нет требования к audit trail |
| ROS2 Action Server для TTS | INSIGHT #7 — рабочий вариант, но требует переписать tts_node. Принимаем как **фазу 3** (см. §10.3) |
| Отдельный `scheduler_node` (ROS-нода) | Дополнительный IPC overhead, EventBus проще держать в `dialogue_node` |
| Переписать всё на LangGraph / LangChain | Vendor lock-in, текущий стек — OpenAI Agents SDK + собственный `async_executor` |

---

## 10. План реализации (фазы) и acceptance criteria

### 10.1 Фаза 1 — MVP (1–2 спринта)

**Цель:** убрать race `stop_music` vs `speak` + классификация ввода.

1. Добавить `TaskScheduler` с каналами voice/music/anim (FIFO-очереди).
2. Интегрировать в `speak_text`, `execute_music_code`, `stop_music` (через `dialogue_node.py`).
3. Quick-decide уровень 1 (правила + IGNORE для шума).
4. Feedback events в LLM-контекст (только `[CHANNELS]` — без PENDING EVENTS).
5. **Регресс v36:** `stop_music` после рэпа не обрывает музыку на середине.

**Acceptance (MVP):**
- [ ] `stop_music()` не может ДОЙТИ ДО ЖЕЛЕЗА раньше конца TTS-чанка (порядок гарантирован scheduler'ом)
- [ ] e2e v36, v38 перестают падать
- [ ] Уровень 1 решает MERGE/REPLACE/IGNORE для типовых фраз за < 50мс
- [ ] LLM в каждом ходе видит блок `[CHANNELS]` (queue depth, current, eta)
- [ ] Пост-амбл «Готово! Зачитал тебе...» исчезает при активном voice-канале
- [ ] **Инвариант сегментов (unit test):** `update()` модифицирует ТОЛЬКО PENDING-сегменты, ACTIVE не затрагивается

### 10.2 Фаза 2 — двухуровневое решение + EventBus

1. Quick-decide уровень 2 (лёгкая LLM, < 800мс).
2. `EventBus` для системных событий (`/battery/*`, `/obstacle/*`, `/hermes/*`).
3. Приоритеты событий (critical > high > normal > low).
4. Сценарий «батарея»: `battery_critical` → вплетается в PENDING-сегмент.
5. **Решение блокера П1:** MERGE/QUEUE не отменяют LLM-цикл (см. §4.4).

**Acceptance:**
- [ ] Решение по новому вводу (любое из 5) принимается < 800мс
- [ ] «и ещё про енота» во время куплета 1 → куплет 2 переписывается без паузы и рестарта
- [ ] `battery_critical` вплетается в следующий PENDING-сегмент без прерывания ACTIVE
- [ ] **Два MERGE подряд за < 2с (unit test):** оба применяются к PENDING, без конфликта
- [ ] **Приоритеты событий (unit test):** critical (`obstacle`) прерывает песню на границе такта, high (`battery`) вплетается в PENDING без прерывания
- [ ] **e2e «батарея»:** робот поёт → battery_critical → предупреждение в песне → финал → сообщение об уходе на базу (без паузы и рестарта)

### 10.3 Фаза 3 — эстиматоры + speculative pre-generation

1. `SegmentEstimator` (обёртка над `estimate_tts_duration` + music/anim метриками).
2. `LLMEstimator` (EMA latency).
3. Speculative pre-gen: пред-синтез TTS сегмента N+1.
4. `InterruptibleTask.cancel()` при правке во время пред-генерации.
5. Fill-сегменты (ambient music) при опоздании LLM.

**Acceptance:**
- [ ] `SegmentEstimator`: точность ±15% (unit test на TTS chars→sec)
- [ ] `LLMEstimator`: EMA обновляется после каждого хода, видна в feedback events
- [ ] Пауза между сегментами при MERGE < 300мс (если LLM уложилась в эстимацию)
- [ ] При опоздании LLM — музыка/ambient продолжается, тишины нет; пометка о паузе в feedback
- [ ] Пред-генерация N+1 отменяется при правке без артефактов (unit test)

### 10.4 Фаза 4 (будущее) — action server + PASTE

1. Перевести `tts_node` на ROS2 Action Server (goal/feedback/result/cancel) — даёт «честный сигнал TTS готов» из коробки (INSIGHT #7).
2. Speculative execution с shadow queue (Microsoft PASTE, arxiv 2603.18897) — для prefetch результатов тулов.
3. Полная замена debounce/таймеров на state-машины каналов.

---

## 11. Что уже решено в текущем коде (без scheduler)

Чтобы не строить лишнего — фиксируем, что **уже работает**:

| Проблема из #968 | Где решена | Как |
|------------------|------------|-----|
| `stop_music` обгоняет `speak_text` (race) | `dialogue_node.py:672` (`_output_lock`) | `speak_text` блокируется до `tts/finished` |
| Несколько `speak_text` в одном батче | `dialogue_node.py:672` (lock) + line 190 (`_output_lock`) | Сериализуются через asyncio.Lock |
| Cleanup срабатывал на первом чанке (v38) | `tts_node.py:741–744` (один finished на speech_id) | Теперь один speech_id = один finished event |
| LLM вызывает `speak_text` + `stop_music` подряд | `dialogue_node.py:678–702` (wait_for finished) | LLM-цикл не двигается, пока TTS не договорит |

**Что осталось как OPEN issue:**
- LLM не видит состояние каналов → пост-амбл, не знает что TTS играет
- Классификация ввода (MERGE/REPLACE/QUEUE/IGNORE/CLARIFY) до barge-in — **блокер П1**
- Сегментная модель + правка PENDING без прерывания ACTIVE
- Speculative pre-gen для устранения пауз
- Единый EventBus для внешних событий (батарея, препятствия, Hermes)

---

## 12. Аудит противоречий (по #15) — как разрешены

| # | Тип | Суть | Разрешение |
|---|-----|------|------------|
| **П1** | 🔴 блокер | barge-in убивает LLM-цикл, MERGE требует его жизни | **§4.4** — классификация ДО barge-in. MERGE/QUEUE/CLARIFY не отменяют цикл. REPLACE — отменяет. |
| **П2** | 🔴 | «прервать ACTIVE на естественной границе» — оксюморон | **§4.3** — critical **сокращает** ACTIVE до ближайшей границы (1–4с от момента события), а не «прерывает». Одна формулировка. |
| G1 | 🟡 | Кто исполняет fade-out? | `audio_node` (или его преемник) — НЕ `dialogue_node`. Fade = спецкоманда в music-канал. **Зафиксировать в коде audio_node при реализации фазы 1.** |
| G2 | 🟡 | Два механизма «TTS готов» | **§3.3 + §8.2** — выбран счётчик по `speech_id` (один finished на один заказ), уже реализован в `tts_node.py:741–744`. |
| G3 | 🟡 | MERGE во время пред-генерации | **§6.5** — `InterruptibleTask.cancel()` при правке. Покрыть unit-тестом (фаза 3). |
| G4 | 🟡 | Prefetch жжёт токены | **§6.5** — prefetch только при активной задаче (sing/rap/рассказ). В idle — цикл спит. |
| G5 | 🟡 | `estimate_tts_duration` vs SegmentEstimator | **§6.2** — `SegmentEstimator` = обёртка над существующим механизмом, LLM не вызывает руками. |
| N1 | 🟢 | «мгновенно от LLM» vs «мгновенно на железо» | **§0 + §4.3** — мгновенно в scheduler — ок; мгновенно на железо — нет. Scheduler = буфер. |
| N2 | 🟢 | «stop_music не может исполниться» | **§10.1 acceptance** — «не может ДОЙТИ ДО ЖЕЛЕЗА». |
| N3 | 🟢 | «без паузы» vs «пауза ≤ 2с при 429» | **§4 + edge cases** — норма = без паузы; ошибки API = пауза допустима, фиксируется в feedback. |

---

## 13. Открытые вопросы (для обсуждения с PM/Author #968)

1. **Fade-out (G1)** — какой компонент отвечает? `audio_node` уже умеет? Если нет — добавить в фазу 1.
2. **Лёгкая LLM (уровень 2)** — какой провайдер/модель? Отдельная конфигурация MiniMax? Или использовать основную с `temperature=0` и коротким промптом? **Требует решения до старта фазы 2.**
3. **ROS-топик `/harness/task_events`** — нужен ли владелец (отдельный node) или достаточно publisher'а из `dialogue_node`?
4. **Совместимость с Phase 5 / ADR-0001 §5** (AgentSession + SideEffectBus) — уточнить у @krikz, не дублируем ли SideEffectBus (см. t_c8396602).
5. **Prefetch budget (G4)** — какой лимит «активной задачи» для prefetch? 30с? 60с? Пока задача длиннее N — prefetch крутится, иначе спит.

---

## 14. Ссылки

- Issue [#968](https://github.com/krikz/rob_box_project/issues/968) — основная задача (15 комментариев)
- PR #907 — tool loop в `dialog_core.py` (предыдущая итерация, устарело)
- PR #935 — watchdog для музыки (временные фиксы в `dialogue_node`)
- ADR-0001 §5 — `AgentSession[StateT]` + `SideEffectBus` (Phase 5, см. t_c8396602)
- `src/rob_box_mcp_tools/rob_box_mcp_tools/async_executor.py` — `AsyncToolExecutor`, `InterruptibleTask`
- `src/rob_box_voice/rob_box_voice/dialogue_node.py` — `speak_text`, `_output_lock`, `_tts_events`, `_cancel_run`
- `src/rob_box_voice/rob_box_voice/tts_node.py` — публикация `tts/finished` (по одному на speech_id)

### Внешние источники (INSIGHT #9)

- Zylos Research (apr 2026) — DAG + topological sort для tool scheduling
- Microsoft PASTE (arxiv 2603.18897) — speculative execution с shadow queue
- LiveKit voice-agent docs — barge-in с `interruption_min_words`
- Spring AI issue #5195 — порядок результатов tool_calls = invariant
- LangChain interrupt docs — control-flow vs audio-event масштабы
- Akka actor docs — mailbox = FIFO ordering

---

## Приложение A. Глоссарий

| Термин | Значение |
|--------|----------|
| **Сегмент** | Атомарная единица исполнения (TTS-чанк, музыкальный такт, anim-кадр) |
| **Задача** | Намерение пользователя, разложенное LLM в последовательность сегментов |
| **Канал** | FIFO-очередь сегментов одного типа (voice/music/anim) |
| **MERGE** | Правка PENDING-сегментов без прерывания ACTIVE |
| **REPLACE** | Сброс очереди, новая задача после текущего сегмента |
| **Естественная граница** | Конец предложения (voice) / такта (music) / ключевого кадра (anim) |
| **Speculative pre-generation** | Пред-синтез TTS следующего сегмента до ответа LLM |
| **EventBus** | Единая шина для пользовательских и системных событий |
| **LLMEstimator** | EMA по latency LLM, обновляется после каждого хода |

---

## Приложение B. Связь с текущими задачами

- **t_8f5bb012** (этот документ) — design proposal для #968
- **t_57d67232** — Architect review #933+#935 (предыдущее состояние voice-assistant)
- **t_f919de81** — Architect review фазы 06-harness-p0-finalization
- **t_f0ddd678** — ADR harness (есть ADR-0001, ADR-0009-harness-tts-contract, и др.)
- **t_c8396602** — P1.1: AgentSession + SideEffectBus (Phase 5, ADR-0001 §5) — **возможна коллизия/синергия, требует уточнения**

---

*Документ готов к review. После согласования — перевод в раздел docs/adr/ как ADR-0010-scheduler.*