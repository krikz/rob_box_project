# STT-provider contract — reference spec

> **Назначение.** Это эталонная спецификация интерфейса STT-провайдера
> после ADR-0091 (MiniMax STT). Downstream-таски реализуют против неё:
> - `t_223f93db` — `minimax_provider.py` (backend, Phase 1)
> - `t_99e504d2` — wiring в `stt_node.py` chain (backend, Phase 2)
> - `t_7283c042` — пользовательская документация (techwriter)
>
> Это НЕ runtime-документ; правки контракта требуют нового ADR.

## 1. Контекст

`rob_box_voice.stt_node` (см. `src/rob_box_voice/rob_box_voice/stt_node.py`
и `stt_fallback.py`) исторически работает с двумя STT-провайдерами:

- **Yandex Cloud STT gRPC v3** — primary, cloud, streaming, поддерживает
  `speaker_analysis` → `speaker_tag` (issue #1077).
- **Vosk 0.42 small-ru** — offline-fallback, CPU, без диаризации.

Цепочка сейчас: Yandex (primary + 1 retry) → Vosk (fallback). Логика —
`select_recognition(providers, audio_bytes, ...)` в
`rob_box_voice.stt_fallback`.

С принятием ADR-0091 добавляется **MiniMax STT** (cloud, sync HTTP POST
`https://api.minimax.io/v1/speech_to_text`, поддержка diarization,
см. <https://platform.minimax.io/docs/api-reference/speech-to-text>).
Меняется **порядок** цепочки: `vosk → minimax → yandex` (offline-first,
cloud-second, primary-last). Обоснование — §3 ADR-0091.

Чтобы это было возможно без поломки существующих адаптеров и тестов
(`test_stt_fallback.py`, `test_stt_node_*`), контракт расширяется
**additive**:

1. Существующий `STTProvider.recognize(audio_bytes) -> Optional[str]`
   (Protocol, `stt_fallback.py:74-86`) **остаётся** required-методом
   для обратной совместимости с `select_recognition`.
2. Рядом появляется **новый value object `STTResult`** с полями
   `text`, `confidence`, `speaker_id`, `segments`, `provider`,
   `latency_ms`, `reason`.
3. Провайдеры, которые умеют больше, чем строку (MiniMax, Yandex),
   реализуют **новый** optional-метод
   `recognize_result(audio_bytes) -> STTResult`. Vosk продолжает
   отдавать только строку (старый `recognize()` достаточно).

## 2. Value object `STTResult`

Файл: `src/rob_box_voice/rob_box_voice/stt_result.py` (новый).

```python
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass(frozen=True)
class STTSegment:
    """Один диаризованный сегмент фразы.

    Маппинг на MiniMax API: response.segments[i] →
        start_ms = segments[i].start (int, ms)
        end_ms   = segments[i].end   (int, ms)
        text     = segments[i].text  (str)
        speaker  = segments[i].speaker  (str, e.g. "speaker_0")
        confidence = segments[i].confidence  (float, 0..1, optional)

    Маппинг на Yandex gRPC v3: response.final.alternatives[0].words[*] →
        start_ms = word.start_time_ms (нет в v3 по умолчанию,
            берём из final_refinement если есть, иначе None)
        end_ms   = word.end_time_ms   (то же)
        text     = word.word
        speaker  = None  (Yandex сейчас даёт speaker_tag одним полем,
            а не per-word — см. ADR-0091 §3.4 mapping table)
        confidence = None
    """
    start_ms: Optional[int]
    end_ms: Optional[int]
    text: str
    speaker: Optional[str] = None
    confidence: Optional[float] = None


@dataclass(frozen=True)
class STTResult:
    """Полный результат распознавания, возвращаемый провайдером.

    Минимально: ``text`` и ``provider``. Остальное опционально и
    заполняется по возможности.

    Поля:
        text:        финальный распознанный текст (НЕ пустой при
                     reason="ok"). Гарантированно strip().
        provider:    имя провайдера (str), напр. "yandex"/"vosk"/
                     "minimax". Стабильное, идёт в метрики и логи.
        confidence:  общая уверенность 0..1, либо None если провайдер
                     её не отдаёт. Per-segment confidence — в
                     STTSegment.confidence.
        speaker_id:  идентификатор доминирующего спикера фразы (str),
                     если провайдер даёт диаризацию (Yandex speaker_tag,
                     MiniMax segments[].speaker для одного спикера).
                     None если провайдер не поддерживает или
                     не смог (Vosk).
        segments:    диаризованные сегменты. Пустой list, если провайдер
                     не даёт (Vosk) или не запрашивалось.
        latency_ms:  длительность распознавания (int, ms).
        reason:      FallbackReason (см. stt_fallback.py).
        raw:         опциональный сырой ответ API для отладки
                     (НЕ публикуется в топики, только в логи).
    """
    text: str
    provider: str
    confidence: Optional[float] = None
    speaker_id: Optional[str] = None
    segments: List[STTSegment] = field(default_factory=list)
    latency_ms: int = 0
    reason: str = "ok"
    raw: Optional[dict] = None  # отладка; НЕ сериализуется в ROS
```

### 2.1 Почему `frozen=True`

`STTResult` иммутабелен: один вызов `recognize()` = один результат,
который шарится между метриками, логированием и публикацией.
Изменение после создания — симптом ошибки; frozen заставляет
использовать `dataclasses.replace()` если нужно «дополнить».

### 2.2 Обратная совместимость

`STTResult.text` — то, что раньше возвращал `recognize() -> str`.
Адаптеры, которые сейчас возвращают `str`, конвертируются
тривиально:

```python
def _wrap(self, text: str, latency_ms: int) -> STTResult:
    return STTResult(text=text or "", provider=self.name,
                     latency_ms=latency_ms,
                     reason="ok" if text else "empty")
```

Существующие тесты `test_stt_fallback.py` (на `recognize() -> str`)
**не меняются** — они проверяют protocol-level, а не value object.

## 3. Расширенный Protocol

Файл: `src/rob_box_voice/rob_box_voice/stt_fallback.py` (edit).

```python
class STTProvider(Protocol):
    """Минимальный интерфейс провайдера для select_recognition().

    Существующие контракты НЕ ломаются. Новые провайдеры с
    диаризацией дополнительно реализуют ``recognize_result``.
    """

    name: str

    def recognize(self, audio_bytes: bytes) -> Optional[str]:
        """Вернуть текст (или None). Старый контракт, ОБЯЗАТЕЛЕН."""
        ...

    def recognize_result(self, audio_bytes: bytes) -> "STTResult":  # NEW, optional
        """Вернуть полный STTResult. Опциональный; default-impl
        оборачивает ``recognize()`` в STTResult без сегментов.

        Если провайдер умеет больше (confidence/speaker_id/segments),
        он переопределяет этот метод.
        """
        ...
```

`recognize_result` имеет **default-реализацию** в `stt_fallback.py`,
которая вызывает `recognize()` и заворачивает результат. Vosk этим
default-путём и пользуется — никаких изменений в существующем коде.

## 4. Маппинг MiniMax API → STTResult

Из <https://platform.minimax.io/docs/api-reference/speech-to-text>:

```bash
curl --request POST \
  --url https://api.minimax.io/v1/speech_to_text \
  --header 'Authorization: Bearer ' \
  --header 'Content-Type: multipart/form-data' \
  --form model=asr-1.0 \
  --form file='@example-file' \
  --form response_format=json \
  --form timestamp_level=word \
  --form stream=false
```

Параметры, которые мы передаём:

| Поле формы | Значение | Обоснование |
|---|---|---|
| `model` | `asr-1.0` | текущая MiniMax STT-модель (см. docs; возможно `asr-1.5` после уточнения backend-ом в `t_223f93db`) |
| `file` | audio bytes (PCM int16 LE mono 16 kHz) | наш внутренний формат |
| `response_format` | `json` | минимально достаточен; `verbose_json` — для сегментов |
| `timestamp_level` | `word` | нужны start/end_ms для STTSegment |
| `language` | `ru` (или `auto`) | MiniMax-специфичный код; маппинг см. §4.1 |
| `diarization` / `speaker_diarization` | `true` | (имя поля проверит backend) |

### 4.1 Маппинг языков

| Yandex (`yandex_language`) | MiniMax (`language`) |
|---|---|
| `ru-RU` | `ru` |
| `en-US` | `en` |
| `auto` | `auto` |

Минимально Phase 1 поддерживает `ru`; остальные коды пробрасываются
как есть. Расширение — Phase 3.

### 4.2 JSON-ответ MiniMax → STTResult

Ожидаемая структура (на основе публичных примеров MiniMax docs;
backend в `t_223f93db` обязан сверить с актуальной документацией
перед мержем):

```json
{
  "text": "робот расскажи анекдот",
  "language": "ru",
  "duration_ms": 1834,
  "segments": [
    {"start": 120, "end": 540, "text": "робот",
     "speaker": "speaker_0", "confidence": 0.97},
    {"start": 540, "end": 980, "text": "расскажи",
     "speaker": "speaker_0", "confidence": 0.95},
    {"start": 980, "end": 1834, "text": "анекдот",
     "speaker": "speaker_0", "confidence": 0.99}
  ]
}
```

Маппинг:

| Поле STTResult | Источник MiniMax |
|---|---|
| `text` | `response.text` |
| `provider` | `"minimax"` (константа) |
| `confidence` | `None` для top-level (MiniMax не отдаёт), per-segment в `STTSegment.confidence` |
| `speaker_id` | доминирующий `speaker` в `segments[]` (mode), иначе `None` |
| `segments` | список `STTSegment` (см. §2) |
| `latency_ms` | реальное время вызова (int, ms) |
| `reason` | `"ok"` если `text` непустой иначе `"empty"` |
| `raw` | весь response dict (только для логов/debug) |

**NB.** Если MiniMax не отдаёт `diarization=true` (например,
endpoint его игнорирует или возвращает пустые `segments[]`),
провайдер ВСЁ РАВНО валиден — просто `speaker_id=None` и
`segments=[]`. Это не ошибка, а degraded mode.

## 5. Цепочка `vosk → minimax → yandex` — chain policy

> **⚠️ ЗАМЕНЕНО [ADR-0124](../adr/0124-stt-provider-chain-priority.md)
> (21.09.2026).** Действующий порядок — `minimax → yandex → vosk`, с
> per-provider бюджетом (`ProviderPolicy`) и кэшем «мёртвых»
> (`ProviderDeadCache`). Раздел ниже сохранён как историческая
> мотивировка Vosk-first; в таком виде он реализован не был.

`select_recognition()` в `stt_fallback.py` **не меняется**. Логика
остаётся: первый провайдер — primary + 1 retry, остальные — без
retry. Меняется **порядок провайдеров в списке**, который формирует
`stt_node._recognize_with_fallback()` (см. §6).

### 5.1 Почему Vosk первый (а не Yandex)

- **Latency budget.** Vosk offline отдаёт результат за ~0.3–0.8с
  на типичной фразе 1–3с. Yandex cloud streaming — 1.5–4с в среднем
  (см. ADR-0091 §3.2 latency-bench). Для barge-in (issue #1734)
  чем быстрее — тем лучше.
- **Offline-first.** Vosk работает без сети. Сейчас Yandex primary
  значит: нет интернета → робот НЕ слышит (Vosk fallback срабатывает
  только после Yandex timeout 12с, issue #1477). Vosk-first
  означает: нет интернета → распознавание работает на локальной
  модели, MiniMax/Yandex — best-effort.
- **Failure isolation.** Каждый последующий провайдер добавляет
  latency и потенциальный rate-limit. Vosk первым покрывает 80–90%
  фраз (наш опыт с backlog-марафоном).

### 5.2 Когда fallback на MiniMax

`select_recognition` падает на следующего провайдера, если
текущий вернул `reason in {"empty", "error", "timeout",
"low_confidence"}`. Это уже работает. Изменение: добавляется
новый класс ошибок для MiniMax (см. §7).

### 5.3 Когда fallback на Yandex (last resort)

Yandex остаётся **последним**: cloud streaming с speech_analysis.
Используется, если:

1. Vosk вернул пусто/мусор (типичный случай — VAD артефакт,
   шум).
2. **И** MiniMax либо вернул пусто/мусор, либо не настроен
   (`MINIMAX_API_KEY` отсутствует), либо вернул timeout/error.

Yandex НЕ пытается повторно распознать то, что Vosk уже
распознал — это уже сделано в `select_recognition`.

### 5.4 Retry и backoff

Минимальная retry-семантика (Phase 1):

- Vosk: 0 retry (offline; если мусор — retry не поможет).
- MiniMax: 1 retry на network/timeout (HTTP 5xx, ConnectError,
  TimeoutException). HTTP 4xx (401, 429) — НЕ retry,
  провайдер unavailable (см. §7).
- Yandex: 1 retry (существующее поведение,
  `yandex_max_retries=1`).

`retry_backoff_s` — общий; default 1.0с (см.
`DEFAULT_RETRY_BACKOFF_S` в `stt_fallback.py`).

### 5.5 MiniMax и speaker_id_node — interaction (issues #2346/#2348)

Тут — главный «зачем». MiniMax `speaker_id` — это **per-utterance
cluster id** (например, `speaker_0`, `speaker_1`), а не d-vector.
speaker_id_node работает с **d-vector** (resemblyzer) и сравнивает
его с зарегистрированными профилями. Они НЕ эквивалентны:

- d-vector `identify` сравнивает cosine-similarity с порогом
  `IDENTIFY_THRESHOLD=0.75` (см. `speaker_embeddings.py:47`).
- MiniMax `speaker_id` — кластер внутри одной фразы. Если фраза
  произнесена одним человеком, MiniMax даст `speaker_0`.
  Если двумя — `speaker_0` и `speaker_1` в разных сегментах.

**Решение (Phase 2, реализует `t_99e504d2`):**

1. MiniMax `speaker_id` **публикуется в существующий топик
   `/voice/stt/speaker`** по тому же JSON-контракту
   `{"speaker_tag": "...", "text": "...", "duration_s": ...}` —
   dialogue_node ничего не знает о том, какой провайдер дал
   speaker_tag, и работает с ним единообразно (line 1933
   `_on_speaker`).
2. **НЕ заменяет** d-vector pipeline: MiniMax diarization —
   подсказка, что «спикер сменился», но profile identity всё
   равно идёт через speaker_id_node (resemblyzer).
3. Конкретная интеграция MiniMax `speaker_id` с
   `REGISTER_MATCH_THRESHOLD`/`IDENTIFY_THRESHOLD` (#2348) — **не
   часть этого ADR**. Если MiniMax даёт стабильный `speaker_id`
   в пределах сессии (например, `speaker_0` всегда один и тот же
   человек), это можно использовать как **prior** для identify
   (см. future ADR-0091).

### 5.6 Граф вызовов

```
audio_bytes (PCM int16 LE mono 16 kHz)
  │
  ▼
┌──────────────────────────────────────────────────────────┐
│ select_recognition(providers=[vosk, minimax, yandex],    │
│                    audio_bytes,                          │
│                    timeout_s=12.0, max_retries=1)        │
│                                                          │
│   for provider in providers:                             │
│     if is_primary(provider):  # vosk                     │
│       retries = max_retries + 1                          │
│     else:                                                │
│       retries = 1                                        │
│     for attempt in retries:                              │
│       text = provider.recognize(audio_bytes)             │
│       if text and not is_short: → return text, attempts  │
│       else:                                              │
│         log attempt (provider, reason, latency_ms)       │
│         if not last retry → sleep(backoff_s) → continue  │
│     # retries exhausted → next provider                  │
│                                                          │
│   return last_nonempty_text, attempts                    │
└──────────────────────────────────────────────────────────┘
  │
  ▼
text → speech_audio_callback → publish /voice/stt/result
speaker_id (если есть) → publish /voice/stt/speaker
segments (если есть) → publish /voice/stt/segments (NEW, Phase 2)
```

## 6. Конфигурация цепочки

> **⚠️ ЗАМЕНЕНО [ADR-0124](../adr/0124-stt-provider-chain-priority.md) §2.6.**
> `config/stt_chain.yaml` удалён. Цепочка и параметры провайдеров —
> ROS-параметры (`stt_provider_chain`, `minimax_stt_*`,
> `provider_dead_ttl_*`, `provider_state_file`), объявленные через
> `declare_parameter` и зеркалируемые в `config/stt_node.yaml`
> (issue #1004). Hot-reload по таймеру отменён: ROS-параметры и так
> меняются на лету.

`src/rob_box_voice/config/stt_chain.yaml` — НОВЫЙ файл. До сих пор
порядок провайдеров был hardcoded в `_recognize_with_fallback`
(Yandex → Vosk). Теперь — YAML.

```yaml
# STT provider chain (issue #2365, ADR-0091).
# Order = priority. First entry = primary, rest = fallback.
# Hot reload: да, нода перечитывает файл по таймеру 5с (Phase 2).
stt_chain:
  chain:
    - vosk
    - minimax
    - yandex

  vosk:
    enabled: true
    model_path: /models/vosk-model-small-ru-0.22
    sample_rate: 16000
    # ROS_VOSK_DISABLE=1 → выключает Vosk принудительно (env override).

  minimax:
    enabled: true
    api_base: https://api.minimax.io/v1
    endpoint: /speech_to_text
    model: asr-1.0
    timeout_s: 5.0
    language: ru            # см. §4.1
    response_format: json
    timestamp_level: word
    diarization: true
    # MINIMAX_API_KEY — обязателен если enabled. Из ENV (приоритет)
    # или ROS-параметра minimax_api_key.

  yandex:
    enabled: true
    model: general
    language: ru-RU
    timeout_s: 12.0
    max_retries: 1
    retry_backoff_s: 1.0
    speech_analysis: true   # speaker_analysis + conversation_analysis
    # YANDEX_API_KEY — обязателен если enabled.
```

### 6.1 Per-provider enable flag

`enabled: false` исключает провайдера из цепочки. Это позволяет
оператору временно отключить, например, MiniMax при недоступности
сервиса (см. issue #1193 — MiniMax 402/429).

### 6.2 Минимально один провайдер

Если все три выключены — `stt_node` логирует ERROR и публикует
`/voice/stt/state=error`. Это уже существующее поведение в
`initialize_vosk` (`publish_state("error")`).

## 7. Ошибки MiniMax и маппинг на FallbackReason

Новые typed exceptions (в `stt_providers/__init__.py`):

```python
class MiniMaxProviderError(Exception):
    """Базовая для всех ошибок MiniMax STT."""


class MiniMaxAuthError(MiniMaxProviderError):
    """401/403 — invalid or missing API key.
    Провайдер unavailable до ротации ключа."""


class MiniMaxRateLimitError(MiniMaxProviderError):
    """429 — превышен rate limit.
    Fallback на следующего провайдера НЕМЕДЛЕННО (retry не поможет)."""


class MiniMaxBadRequestError(MiniMaxProviderError):
    """400 — некорректный формат аудио или параметров.
    Баг в нашей интеграции, не retry."""


class MiniMaxTimeoutError(MiniMaxProviderError, TimeoutError):
    """HTTP timeout / ConnectError / ReadError.
    Retry 1 раз, затем fallback."""


class MiniMaxServerError(MiniMaxProviderError):
    """5xx — серверная ошибка MiniMax.
    Retry 1 раз, затем fallback."""
```

Маппинг на `FallbackReason` (см. `stt_fallback.py:41-49`):

| MiniMax exception | FallbackReason | Retry? |
|---|---|---|
| `MiniMaxAuthError` | `error` | no |
| `MiniMaxRateLimitError` | `error` | no |
| `MiniMaxBadRequestError` | `error` | no |
| `MiniMaxTimeoutError` | `timeout` | yes (1) |
| `MiniMaxServerError` | `error` | yes (1) |
| HTTP 200 + `text=""` | `empty` | no |
| HTTP 200 + `len(text) < min_chars` | `low_confidence` | no |
| HTTP 200 + valid text | `ok` | n/a |

Этот маппинг — **часть контракта**, не деталь реализации.
Любой backend-тест `t_223f93db` сверяется с этой таблицей.

## 8. ROS-топики

Существующие контракты **не меняются**:

| Топик | Тип | Payload | Источник | Потребитель |
|---|---|---|---|---|
| `/voice/stt/result` | `std_msgs/String` | plain text (final) | stt_node | dialogue_node, telegram, perception |
| `/voice/stt/speaker` | `std_msgs/String` | JSON `{speaker_tag, text, duration_s}` | stt_node | dialogue_node (`_on_speaker`) |
| `/voice/stt/state` | `std_msgs/String` | `ready` / `recognizing` / `error` | stt_node | UI, observability |

**Новый топик (Phase 2):**

| Топик | Тип | Payload | Источник | Потребитель |
|---|---|---|---|---|
| `/voice/stt/segments` | `std_msgs/String` | JSON `{provider, text, segments: [{start_ms, end_ms, text, speaker, confidence}]}` | stt_node (только если есть segments) | dialogue_node / speaker_id_node (Phase 3) |

`/voice/stt/segments` публикуется **только** когда
`STTResult.segments` непустой (т.е. MiniMax или будущий
диаризующий провайдер). Vosk/Yandex без диаризации — не публикует.
Backpressure: если никто не подписан, топик просто пустой (ROS).

## 9. Backward compatibility matrix

| Что | Статус | Обоснование |
|---|---|---|
| `stt_fallback.STTSelectResult` (text) | unchanged | existing callers не меняются |
| `stt_fallback.STSTAttempt` (dataclass) | unchanged | metric labels стабильны |
| `stt_node._recognize_with_fallback` | edit, additive | новый optional параметр `provider_results: List[STTResult]` (default=None) |
| `stt_node.publish_result` (text на /voice/stt/result) | unchanged | инвариант P0 |
| `stt_node.publish_speaker` (/voice/stt/speaker) | unchanged payload | MiniMax использует тот же формат |
| `/voice/stt/segments` | NEW, opt-in | только при наличии диаризации |
| ROS-параметры `yandex_*` | unchanged | |
| ROS-параметры `model_path`, `sample_rate` | unchanged | Vosk конфиг |
| Тесты `test_stt_fallback.py` | unchanged | Protocol-level, без value object |
| Тесты `test_stt_node_router.py` | minor edit | новый chain order (3 провайдера вместо 2) |

## 10. Границы решения (out of scope)

- **Не делается** в этом ADR:
  - Голосовая биометрия (d-vector / resemblyzer) — отдельный pipeline,
    ADR-0091 только про STT.
  - Ре-калибровка порогов `IDENTIFY_THRESHOLD`/`REGISTER_MATCH_THRESHOLD`
    — это #2348, отдельная задача.
  - Переиспользование MiniMax `speaker_id` как prior для identify —
    future ADR-0091, после того как MiniMax provider даст реальные
    данные в robot logs.
  - Реализация `minimax_provider.py` — `t_223f93db` (backend).
  - Wiring в `stt_node.py` — `t_99e504d2` (backend).
  - Документация для оператора — `t_7283c042` (techwriter).
  - E2E сценарий — отдельная задача после `t_99e504d2`.

- **Решения, которые ADR-0091 НЕ принимает**:
  - Не менять `select_recognition` (можно, но не нужно —
    additive `recognize_result` достаточно).
  - Не вводить capability registry (`ProviderCapabilities`) —
    см. ADR-0002 §6, отложено.
  - Не менять `wake_words` namespace routing — это ADR-0070.

## 11. Cross-references

- Issue #2365 — основная задача.
- Issue #2346 — investigate n308 ermil (диагностика #2346, закрыта).
- Issue #2348 — калибровка порогов speaker-id (open).
- ADR-0002 — MiniMax provider (TTS + capability-segregated design).
- ADR-0018 — честность воркера (raw-evidence обязателен).
- ADR-0070 — systemic wake-gate.
- ADR-0091 — этот ADR.
- `src/rob_box_voice/rob_box_voice/stt_node.py` — runtime.
- `src/rob_box_voice/rob_box_voice/stt_fallback.py` — fallback логика.
- `src/rob_box_voice/rob_box_voice/dialogue_node.py:1933` — `_on_speaker`.
- `src/rob_box_voice/rob_box_voice/speaker_id_node.py` — d-vector pipeline.
- `src/rob_box_voice/rob_box_voice/speaker_embeddings.py:47,58,333` — пороги.
