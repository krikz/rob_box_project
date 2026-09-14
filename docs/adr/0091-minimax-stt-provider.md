# ADR-0091: MiniMax Speech-to-Text как третий провайдер в STT-chain

| Поле         | Значение                                                                |
|--------------|-------------------------------------------------------------------------|
| Статус       | Proposed                                                                |
| Дата         | 2026-09-14                                                              |
| Автор        | architect (Hermes Agent)                                                |
| Контекст     | Kanban task `t_646e797d`, issue [#2365](https://github.com/krikz/rob_box_project/issues/2365) |
| Заменяет     | — (расширяет ADR-0002 на STT-домен)                                     |
| Заменяется   | —                                                                       |
| Связанные    | ADR-0002 (MiniMax TTS capability-segregated design), ADR-0018, ADR-0070, [#2346](https://github.com/krikz/rob_box_project/issues/2346), [#2348](https://github.com/krikz/rob_box_project/issues/2348) |
| Спецификация | [`docs/architecture/stt-provider-contract.md`](../architecture/stt-provider-contract.md) |
| Дочерние таски | `t_223f93db` (Phase 1: `minimax_provider.py`), `t_99e504d2` (Phase 2: wiring), `t_7283c042` (docs) |

---

## 1. Контекст

`rob_box_voice.stt_node` сегодня работает с двумя STT-провайдерами:

- **Yandex Cloud STT gRPC v3** — primary (cloud streaming, даёт
  `speaker_tag` через `speaker_analysis`, issue #1077).
- **Vosk 0.42 small-ru-0.22** — offline-fallback (CPU, без диаризации).

Цепочка жёстко прописана в `stt_node._recognize_with_fallback`:
`yandex → vosk`, с одним retry на Yandex (`yandex_max_retries=1`,
soft-timeout 12с, см. issue #1477). Логика отбора и метрик —
`select_recognition(providers, audio_bytes, ...)` в
`rob_box_voice.stt_fallback.py`.

В сентябре 2026 у MiniMax (text/TTS-провайдер в нашем стеке,
ADR-0002) появился **новый API: Speech-to-Text** ([docs](https://platform.minimax.io/docs/api-reference/speech-to-text)).
Ключевые для нас свойства:

- **Speaker diarization** встроенная (per-utterance `speaker_id`,
  `segments[]` с `start/end` ms).
- **Streaming output** (SSE-style), но также sync `response_format=json`.
- Cloud HTTPS POST `https://api.minimax.io/v1/speech_to_text`,
  multipart/form-data, bearer auth.
- Простой SDK-less контракт (как у MiniMax TTS `t2a_v2`,
  ADR-0002 §2 п.3 — httpx + MockTransport).

### 1.1 Бизнес-проблема

В ночном backlog-марафоне `run #34522852773` (act3, e2e сценарий
`night_marathon_act3_backlog_diarization_v1.json`) все три
зарегистрированных голоса упали в «незнакомец»: speaker_id_node
не справился (#2346, #2348). Гипотезы из #2346:

1. дрейф d-vector у синтетических голосов возле `IDENTIFY_THRESHOLD=0.75`;
2. гонка атрибуции в backlog;
3. рассинхрон регистрации.

**MiniMax STT не решает первопричину #2346/#2348 напрямую** —
speaker_id_node работает с d-vector (resemblyzer), а MiniMax даёт
per-utterance cluster id, не d-vector. Но:

- Diarization как **side-channel signal** помогает backlog-сценарию
  отличать «сменился спикер» от «тот же спикер плохо опознан»;
- Снижает зависимость от Yandex (folder `b1gfmjogjodcgff82pjd`
  в архиве) — текущая цепочка имеет единственный cloud-провайдер;
- ADR-0091 фиксирует **контракт** и **chain order**, а интеграция
  MiniMax `speaker_id` с порогами identify (#2348) — отдельный
  future ADR.

### 1.2 Текущее состояние цепочки

```
audio → yandex (primary + 1 retry, 12s) → vosk (offline, 0 retry)
```

Минусы:

- **Latency budget для barge-in.** Yandex cloud streaming 1.5–4с на
  типичной фразе. Vosk offline ~0.3–0.8с. Для `barge_in_policy`
  (issue #1734) разница существенна.
- **Offline-fallback медленный.** Если Yandex лежит (timeout 12с,
  DEADLINE_EXCEEDED на каждый фразы) — пользователь ждёт 12с ×
  каждый retry × каждый фраза. Vosk сразу даёт ответ, даже
  если мусорный.
- **Нет cloud-альтернативы.** Yandex — единственный облачный
  провайдер. Падение Yandex = полная зависимость от локальной
  модели (Vosk).

### 1.3 Что ADR-0091 фиксирует

- **Контракт STT-провайдера** (новый `STTResult` value object +
  опциональный `recognize_result()` метод в Protocol).
- **Chain policy**: новый порядок `vosk → minimax → yandex`
  (offline-first, cloud-second, primary-last).
- **Маппинг** MiniMax API fields → STTResult.
- **Error mapping** MiniMax exceptions → `FallbackReason`.
- **ROS-топики**: существующие инвариантны, новый
  `/voice/stt/segments` (opt-in, только при диаризации).
- **Конфиг** `stt_chain.yaml` (новый файл).

## 2. Решение

### 2.1 Расширение `STTProvider` Protocol (additive)

`stt_fallback.STTSProvider` уже определён как
`Protocol` с `name: str` + `recognize(audio_bytes) -> Optional[str]`
(`stt_fallback.py:74-86`). Этот контракт **НЕ ломается**. Добавляется:

1. **Value object `STTResult`** в новом файле
   `src/rob_box_voice/rob_box_voice/stt_result.py` — frozen
   dataclass с полями `text, provider, confidence, speaker_id,
   segments, latency_ms, reason, raw`. Полная спецификация —
   `docs/architecture/stt-provider-contract.md` §2.
2. **Optional метод** `recognize_result(audio_bytes) -> STTResult`
   в Protocol. Default-impl оборачивает `recognize()` в `STTResult`.
   Vosk и Yandex (если не переопределяют) автоматически получают
   `segments=[]`, `speaker_id=None`, `confidence=None`.
3. **`STTSegment`** (frozen dataclass) с полями
   `start_ms, end_ms, text, speaker, confidence`. Маппится из
   MiniMax `response.segments[]` и (в будущем) из Yandex
   `final_refinement.words[]`.

### 2.2 Новый порядок цепочки: `vosk → minimax → yandex`

| Позиция | Провайдер | Retry | Soft-timeout | Speaker info |
|---|---|---|---|---|
| 0 (primary) | Vosk | 0 | n/a (offline) | нет |
| 1 (fallback) | MiniMax | 1 (только timeout/5xx) | 5с | да (diarization) |
| 2 (last resort) | Yandex | 1 | 12с | да (`speaker_tag`) |

**Почему Vosk первый** (подробнее §3.1 спеки):

- offline, latency ~0.3–0.8с, не зависит от интернета;
- при barge-in (issue #1734) критична скорость;
- Vosk покрывает 80–90% фраз на нашем оборудовании (backlog data).

**Почему MiniMax второй** (подробнее §3.2 спеки):

- cloud, sync POST, ~1.5–3с в среднем;
- даёт diarization (нет ни у кого другого в chain);
- 5с timeout → budget на всю цепочку остаётся вменяемым.

**Почему Yandex последний** (подробнее §3.3 спеки):

- cloud streaming с speech_analysis (`speaker_tag` + conversation_analysis);
- исторически primary, но latency 1.5–4с;
- остаётся как fallback для случая «MiniMax API лежит».

### 2.3 Конфигурация: `src/rob_box_voice/config/stt_chain.yaml`

Новый файл с YAML-описанием цепочки и per-provider параметров.
Hot reload нодой через ROS-параметр `stt_chain_file` (Phase 2,
`t_99e504d2`). Полная спецификация — `docs/architecture/...` §6.

Минимальная Phase 1 конфигурация:

```yaml
stt_chain:
  chain: [vosk, minimax, yandex]
  vosk:    {enabled: true, model_path: /models/vosk-model-small-ru-0.22}
  minimax: {enabled: true, model: asr-1.0, language: ru,
            diarization: true, timeout_s: 5.0}
  yandex:  {enabled: true, model: general, language: ru-RU,
            timeout_s: 12.0, max_retries: 1, speech_analysis: true}
```

### 2.4 ROS-топики — что меняется

| Топик | Изменение | Обоснование |
|---|---|---|
| `/voice/stt/result` | нет | инвариант P0 (ADR-0001) |
| `/voice/stt/speaker` | нет (payload `{speaker_tag, text, duration_s}`) | MiniMax `speaker_id` идёт сюда как `speaker_tag` |
| `/voice/stt/state` | нет | существующий |
| `/voice/stt/segments` | **NEW** (Phase 2, opt-in) | сегменты с `start_ms/end_ms` |

`/voice/stt/segments` публикуется **только** если
`STTResult.segments` непустой. Payload:
```json
{
  "provider": "minimax",
  "text": "робот расскажи анекдот",
  "segments": [
    {"start_ms": 120, "end_ms": 540, "text": "робот",
     "speaker": "speaker_0", "confidence": 0.97}
  ]
}
```

### 2.5 MiniMax и speaker_id_node

MiniMax `speaker_id` — **per-utterance cluster id**
(например, `speaker_0`, `speaker_1`), а НЕ d-vector. Различие:

- MiniMax: «этот сегмент произнёс спикер с id=N внутри фразы».
- speaker_id_node (resemblyzer): «d-vector этой фразы ближе всего
  к зарегистрированному профилю X с cosine 0.78».

**Решение (Phase 2, реализует `t_99e504d2`):**

1. MiniMax `speaker_id` публикуется в `/voice/stt/speaker` по
   тому же JSON-контракту (dialogue_node не знает про источник,
   см. `dialogue_node.py:1933` `_on_speaker`).
2. **НЕ заменяет** d-vector pipeline. speaker_id_node продолжает
   работать с d-vector; MiniMax diarization — side-channel.
3. Ре-калибровка `IDENTIFY_THRESHOLD` / `REGISTER_MATCH_THRESHOLD`
   с учётом MiniMax — **не часть этого ADR** (issue #2348,
   отдельный future ADR-0091).

## 3. Обоснование по decision framework

### 3.1 Какую бизнес-проблему решает

- **Diarization as side-channel signal.** Backlog-марафон act3
  (#2346, #2348) — робот не различает «сменился спикер» vs
  «тот же спикер плохо опознан». MiniMax diarization даёт
  явный сигнал смены спикера внутри фразы; даже если d-vector
  ошибается, dialogue_node видит `speaker_tag="speaker_1"`
  в сегменте 2 после `speaker_tag="speaker_0"` в сегменте 1 —
  и может атрибутировать backlog правильно.
- **Latency для barge-in.** Vosk primary → barge-in за ~0.8с
  вместо ~2с с Yandex primary. Это ~1.2с на фразу → заметно в UX.
- **Cloud-провайдер альтернатива.** Yandex folder `b1gfmjogjodcgff82pjd`
  в архиве (см. issue #2365 §1) — при недоступности Yandex робот
  сейчас падает на медленный Vosk retry через 12с timeout. С
  MiniMax как middle — cloud-провайдер всегда доступен (пока
  MiniMax не лежит тоже).
- **Dependency reduction.** Текущая цепочка Yandex-first создаёт
  hard-dependency на cloud. Vosk-first делает offline основным
  сценарием, cloud — best-effort.

### 3.2 Самая простая альтернатива

Добавить MiniMax в существующий chain **после** Yandex
(`yandex → vosk → minimax`). Не трогать порядок.

**Почему не берём:**

- Сохраняет Yandex-first = медленный offline-fallback.
- Не решает latency-проблему barge-in.
- MiniMax «за шеренгой» Yandex → используется только когда Yandex
  не справился (редко). Diarization-польза минимальна.

Вторая альтернатива: **полностью заменить Yandex на MiniMax**.
Отвергнута по ADR-0002 §2.1 (обратная совместимость голоса
«anton» от Yandex).

Третья альтернатива: **использовать только d-vector**
(speaker_id_node) и не вводить MiniMax. Отвергнута — backlog
показал, что d-vector pipeline один не справляется (#2346, #2348),
нужен второй источник сигнала.

### 3.3 Complexity vs benefit

Добавляется:

- один новый файл `stt_result.py` (~80 LOC);
- один новый YAML `stt_chain.yaml` (~30 LOC);
- опциональный метод `recognize_result()` в Protocol (default
  impl, ~10 LOC);
- новый топик `/voice/stt/segments` (Phase 2, opt-in).

Выгода:

- chain policy становится data-driven (YAML), не hardcoded
  в `stt_node.py`;
- future-провайдеры (ElevenLabs, Azure) подключаются через
  тот же `recognize_result()` — паттерн уже есть (ADR-0002 §2.1
  про TTSProvider ABC);
- явный diarization-канал для backlog-сценариев;
- без поломки существующих тестов и ROS-топиков.

Trade-off оправдан: ~120 LOC архитектурного кода за измеримое
снижение latency + новый capability (diarization).

### 3.4 Что будет, если НЕ делать сейчас

- Backlog-марафон продолжит падать в «незнакомец» (#2346, #2348).
- При недоступности Yandex (issue #1193 — 402/429 на MiniMax)
  нет cloud-fallback → цепочка медленно деградирует до Vosk-only.
- Новые STT-провайдеры (ElevenLabs, Azure) будут добавляться
  через copy-paste в `stt_node.py` (текущая архитектура это
  поощряет).

Альтернативная стоимость: продолжение затыкания дыр в chain
через ещё один ad-hoc `_recognize_*` метод в `stt_node.py`.

## 4. Рассмотренные альтернативы

### A. MiniMax в конец цепочки (`yandex → vosk → minimax`)

Отклонено. Сохраняет все текущие проблемы latency и
hard-dependency на Yandex. Diarization-польза минимальна.

### B. Полная замена Yandex на MiniMax

Отклонено. ADR-0002 §2.1 — голос «anton» от Yandex считается
«оригинальным ROBBOX голосом», менять default без явного запроса
нельзя. MiniMax остаётся middle-вариантом.

### C. Capability registry (`ProviderCapabilities`)

Отложено. ADR-0002 §6 уже откладывал capability registry до
появления второго use-case. STT не создаёт его (по-прежнему
один домен — speech recognition; diarization — feature внутри).

### D. Использовать только d-vector (`speaker_id_node`) и не вводить MiniMax STT

Отклонено. #2346/#2348 показывают, что d-vector pipeline один
не справляется в backlog-сценарии. Нужен второй источник
сигнала — MiniMax diarization.

### E. Event bus / CQRS для STT-pipeline

Отклонено как overengineering. ROS topics уже обеспечивают
loose coupling между stt_node и downstream (dialogue_node,
speaker_id_node). Event bus добавит лишний indirection без
измеримой пользы.

### F. Re-калибровка `IDENTIFY_THRESHOLD`/`REGISTER_MATCH_THRESHOLD` в этом же ADR

Отклонена по scope. Это #2348, отдельная задача. ADR-0091
фиксирует **контракт** MiniMax-провайдера, а не пороги
identify. Если MiniMax даёт стабильный `speaker_id` в
пределах сессии — это prior для identify, что должно стать
отдельным ADR (ADR-0091 future) после накопления robot-logs.

## 5. Последствия

### 5.1 Положительные

- Цепочка становится **data-driven** (YAML), не hardcoded.
- Vosk-first снижает p95 latency barge-in с ~2с до ~0.8с.
- Diarization как side-channel signal для backlog-сценариев
  (#2346, #2348).
- Cloud-fallback при недоступности Yandex (MiniMax).
- Future-провайдеры подключаются через единый
  `recognize_result()` interface.
- Никаких изменений в инвариантных топиках
  `/voice/stt/result`, `/voice/stt/speaker`, `/voice/stt/state`.

### 5.2 Отрицательные / риски

| Риск | Mitigation |
|---|---|
| MiniMax API не вернёт ожидаемый JSON | провайдер парсит **defensively**, при отсутствии `segments[]` деградирует до `text`-only (см. §4.2 спеки) |
| MiniMax провайдер «лежит» (402/429) | `enabled: false` в YAML + env `MINIMAX_API_KEY` отсутствие → автоматическое исключение из chain (логируем warning, не error) |
| Увеличение latency для длинных фраз | MiniMax timeout 5с → fallback на Yandex; total chain budget остаётся ~22с (5+12+overhead) — норма для barge-in |
| Ломается контракт `/voice/stt/result` | **не меняется** — `text` берётся из `STTResult.text` |
| Бэкенд (`t_223f93db`) не успеет сверить MiniMax API поля до merge | ADR-0091 не блокирует merge спеки; маппинг в §4 спеки — **best-effort на основе публичных примеров**. Backend обязан провалидировать против актуальных docs перед merge и обновить ADR если расхождения. |
| Новый топик `/voice/stt/segments` не имеет consumer | opt-in (публикуется только если есть segments); ROS empty topic — безопасно |
| Concurrent MiniMax + Yandex дают разные `speaker_tag` | chain — последовательный: если MiniMax дал результат, Yandex не вызывается. Гонки нет. |

### 5.3 Нейтральные

- `rob_box_voice.__version__` не бампается (additive).
- Тесты `test_stt_fallback.py` остаются зелёными без изменений
  (Protocol-level contract, не value object).
- Yandex `speaker_tag` формат остаётся прежним
  (`{speaker_tag, text, duration_s}`) — MiniMax использует то же.

## 6. Совместимость с ADR-0002

ADR-0002 (MiniMax TTS) установил:

- Capability-segregated design (TTS не в LLM interface, STT
  не в TTS interface) — этот ADR следует: STT — отдельный
  domain с собственным Protocol и value objects.
- `rob_box_llm` как optional dep — здесь: `rob_box_voice`
  остаётся собираемым без `MINIMAX_API_KEY` (провайдер disabled).
- httpx как runtime dep (для тестов с `MockTransport`) — здесь:
  тот же подход, MiniMaxProvider использует `httpx.AsyncClient`.

Совместимо.

## 7. Совместимость с другими ADR

| ADR | Связь |
|---|---|
| ADR-0001 (Harness) | не затрагивается (harness — TTS contract, не STT) |
| ADR-0018 (честность) | raw-evidence обязателен в PR-описании (особенно для маппинга MiniMax API полей) |
| ADR-0021 (dialogue_node decomposition) | `stt_node.py` остаётся в CC-budget (Phase 2 wiring, `t_99e504d2`, должен вписаться) |
| ADR-0070 (wake-gate) | не затрагивается; новый `/voice/stt/segments` не ломает wake-flow |
| ADR-AF-0030 (ADR-нумерация) | ADR-0091 — runtime-домен, монотонный счётчик (после ADR-0089) |

## 8. План внедрения и gates

Этапы реализуются в дочерних карточках:

1. **Phase 1 (PoC, `t_223f93db`, backend)**:
   - `minimax_provider.py` — async httpx клиент.
   - Маппинг API → `STTResult` (см. §4 спеки).
   - `stt_chain.yaml` + loader в `stt_node.py`.
   - Unit-тесты с `MockTransport`.
2. **Phase 2 (integration, `t_99e504d2`, backend)**:
   - `_recognize_with_fallback` читает YAML и формирует
     список провайдеров в порядке `vosk → minimax → yandex`.
   - `/voice/stt/segments` публикация.
   - `recognize_result()` путь — `STTResult` пробрасывается
     до публикации segments и speaker.
   - Hot reload YAML (timer 5с) — опционально.
3. **Phase 3 (operator docs, `t_7283c042`, techwriter)**:
   - README.md / docs/voice.md — упоминание MiniMax.
   - CHANGELOG — note + link на #2365 + ADR-0091.
   - Docstring в `MiniMaxProvider.recognize_result()` — когда
     предпочитать.
4. **Phase 4 (e2e, отдельная задача)**:
   - e2e scenario с fallback на MiniMax STT.
   - `.github/e2e/voice_commands/` — новая команда.

### 8.1 Acceptance criteria для ADR-0091

- ADR merged/added (this file) — ✅ пишется.
- Спека `docs/architecture/stt-provider-contract.md` — ✅
  пишется.
- Output: list of changed/added files — заполняется при
  `kanban_complete`.

## 9. Границы решения

**Не входит в ADR-0091:**

- Реализация `minimax_provider.py` — `t_223f93db` (backend).
- Wiring в `stt_node.py` — `t_99e504d2` (backend).
- Operator docs — `t_7283c042` (techwriter).
- Ре-калибровка порогов identify — #2348, future ADR-0091.
- e2e scenario — отдельная задача после Phase 2.
- Capability registry — отложено (ADR-0002 §6).
- Голосовая биометрия (d-vector) — отдельный pipeline, не
  затрагивается.
- Изменение формата `/voice/stt/result` — ЗАПРЕЩЕНО (инвариант
  P0).

## 10. Когда пересмотреть решение

- MiniMax API сильно меняет формат response (нет
  `segments[]`, переименование полей) → пересмотреть маппинг
  §4 спеки.
- Появляется **второй use-case** для capability registry →
  унифицировать STT+TTS в одном реестре (ADR-0092 future).
- Backlog-марафон продолжает падать в «незнакомец» несмотря
  на diarization → нужно пересмотреть подход к identify
  (вероятно, ML-based clustering вместо threshold-based).
- Anthropic-compatible или другой API с лучшим качеством
  STT → ADR-0093 future (аналогично ADR-0002 alternative C).

## 11. Источники

- MiniMax STT API: <https://platform.minimax.io/docs/api-reference/speech-to-text>
- MiniMax TTS ADR (ADR-0002): `docs/adr/0002-minimax-provider.md`
- Issue #2365: MiniMax STT как новый провайдер
- Issue #2346: investigate n308 ermil (диагностика, закрыта)
- Issue #2348: калибровка порогов speaker-id (open)
- Issue #1077: speaker_analysis от Yandex (существующий
  pattern для `speaker_tag`)
- Issue #1734: barge-in policy (latency-мотивация для
  Vosk-first)
- Issue #1477: Yandex STT two-phase fallback (текущая логика)
- `src/rob_box_voice/rob_box_voice/stt_node.py` — runtime
- `src/rob_box_voice/rob_box_voice/stt_fallback.py` — fallback логика
- `src/rob_box_voice/rob_box_voice/dialogue_node.py:1933` —
  `_on_speaker`
- `src/rob_box_voice/rob_box_voice/speaker_id_node.py` —
  d-vector pipeline
- `src/rob_box_voice/rob_box_voice/speaker_embeddings.py:47,58,333`
  — пороги identify/register
