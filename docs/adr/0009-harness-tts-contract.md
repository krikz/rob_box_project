# ADR-0009: Контракт `TTSProvider` и модель опций в harness

| Поле         | Значение                                                                          |
|--------------|-----------------------------------------------------------------------------------|
| Статус       | **Accepted** (retroactive agreement — implementation landed)                      |
| Дата         | 2026-07-26                                                                        |
| Автор        | architect (Hermes Agent)                                                          |
| Kanban       | `t_be58f765`                                                                      |
| Контекст     | Согласование harness-side `TTSProvider` после завершения имплементации в `t_aebd2d3b` |
| Родители     | [ADR-0001](0001-harness-architecture.md) §2.4, `t_35cfe938` (harness framework)    |
| Предшественники | [ADR-0003](0003-minimax-tts-architecture.md) (MiniMax T2A v2), [ADR-0008](0008-tts-provider-extension-points-landed.md) (landed `BaseTTSProvider`) |
| Связанные    | [`docs/architecture/tts-extension-points.md`](../architecture/tts-extension-points.md), [`docs/architecture/minimax-tts-architecture.md`](../architecture/minimax-tts-architecture.md) |
| Реализация   | `src/rob_box_harness/rob_box_harness/tts/{minimax_tts.py,registry.py}` (64 теста / зелёные) |
| Ветка        | `feature/harness-p0-foundation`                                                   |

---

## 1. Назначение

ADR-0001 §2.4 перечислил **7 портов** харнесса, но не детализировал TTS-порт — он был оставлен на уровне `SideEffectBus` (`CompositeBus(TTS+Sound+LED+…)`). После `t_35cfe938` (harness framework, M7 — env-only secrets) и `t_aebd2d3b` (harness-side `MiniMaxTTSProvider`) появился **отдельный TTS-порт** с собственной фабрикой и реестром. Этот ADR фиксирует контракт **post-factum**, чтобы будущие TTS-провайдеры (ElevenLabs, Google, local-Piper) могли подключаться по единой схеме и не дрейфовать от уже написанного.

> **Важно:** ADR-0001 immutable (SPEC_CURRENT §4). Этот ADR **не меняет** ADR-0001, а фиксирует уточнение одного из его портов на отдельной странице.

## 2. Контракт (что зафиксировано в коде)

### 2.1 Сигнатура

```python
# Наследуется от rob_box_llm.tts.TTSProvider (frozen в tts.py:121-177).
class TTSProvider(abc.ABC):
    name: str  # канонический ключ реестра ("minimax" по умолчанию)
    async def synthesize(text: str, *, settings: TTSSettings | None = None) -> TTSAudio: ...
    async def stream   (text: str, *, settings: TTSSettings | None = None) -> AsyncIterator[TTSChunk]: ...
    async def aclose(self) -> None: ...
```

**Стрелка контракта:**
- `text: str` — единственный позиционный аргумент (ровно как требует task body «синтезировать реплику»).
- `settings: TTSSettings | None` — keyword-only объект-значение со всеми опциями (`voice / model / language / speed / volume / pitch / emotion / sample_rate / format / text_normalization / extra`).
- Возврат: `TTSAudio(samples: bytes, sample_rate: int, format: TTSFormat, raw: Any | None)` — `samples` это **raw bytes в формате `format`** (PCM int16 LE mono, либо контейнер WAV/MP3/OGG).

> **Уточнение к task body:** ранняя формулировка `synthesize(text, voice, **opts) -> bytes` была strawman до заморозки контракта. После ADR-0008 принята keyword-only форма с `TTSSettings` value-object — она уже реализована, покрыта 64 тестами и используется во всех upstream-провайдерах. Менять сигнатуру «обратно на kwargs» = сломать восходящую совместимость и `FakeTTSProvider` (он тоже `TTSProvider`). См. §5 «Trade-off».

### 2.2 Поддерживаемые форматы и опции (MiniMax harness-side)

| Опция TTSSettings | Маппинг в MiniMax T2A v2 | Дефолт MiniMax |
|---|---|---|
| `model`         | `model` (`"speech-02-hd"`, `"speech-02-turbo"`, …) | `"speech-02-hd"` |
| `voice`         | `voice_setting.voice_id` (каталог из `list_voices()` — 6 голосов ru/en/zh) | `"English_expressive_narrator"` (provider default) |
| `language`      | BCP-47 → human-readable (`"ru"→"Russian"`, `"en"→"English"`) | `None` (API решает) |
| `speed`         | `voice_setting.speed` (0.5–2.0) | `1.0` |
| `volume`        | `voice_setting.vol` (0.0–10.0) | `1.0` |
| `pitch`         | `voice_setting.pitch` (semitone) | `0` |
| `emotion`       | `voice_setting.emotion` (`"happy"/"neutral"/"sad"/…`) | `None` |
| `sample_rate`   | `audio_setting.sample_rate` (8000/16000/22050/24000/**32000**) | `32000` (MiniMax дефолт) |
| `format`        | `audio_setting.format` (`"pcm"`/`"mp3"`/`"wav"`) — **OGG НЕ поддерживается MiniMax, fallback на `mp3`** | `TTSFormat.PCM` |
| `text_normalization` | `text_normalization` (bool) | `None` |
| `extra`         | whitelist 9 ключей; reserved (`model`/`text`/`stream`/`voice_setting`/`audio_setting`/`text_normalization`) → `TTSBadRequestError` | `{}` |

**Формат выхода** (что попадает в `TTSAudio.samples`):
- `TTSFormat.PCM` → raw int16 LE mono (PCM 22050/24000/32000 Hz в зависимости от `sample_rate`). Это **основной контракт топика** `/voice/audio/speech` (ADR-0003 §2.3).
- `TTSFormat.WAV` → RIFF/WAVE с правильным заголовком (MiniMax сам оборачивает; upsample к нужному `sample_rate` если требуется).
- `TTSFormat.MP3` → MPEG-1 Layer III (провайдер сам декодирует при желании).
- `TTSFormat.OGG` → **не поддерживается MiniMax**; если запрошен, провайдер тихо подменяет на MP3 с warning-логом (см. ADR-0003 §2.1).

### 2.3 Ошибки (иерархия — `src/rob_box_llm/rob_box_llm/errors.py`)

```
TTSError (база)
├── TTSAuthError           # 401/403 → НЕ retry (programming error)
├── TTSBadRequestError     # 400 + зарезервированные ключи в extra → НЕ retry
├── TTSRateLimitError      # 429 → retry (harness ≤1 попытка с backoff 1s)
└── TTSTimeoutError        # asyncio.TimeoutError, httpx timeout → retry (≤2, 0.5s/1.5s)
```

`base_resp.status_code != 0` (envelope MiniMax) → `TTSError` (НЕ retry, бизнес-ошибка).
Retry-петля реализована **в harness-обёртке** (`HarnessMiniMaxTTSProvider._call_with_retry`), не в upstream-провайдере (ADR-0003 §2.6 — обоснование «retry снаружи»).

### 2.4 Аутентификация и секреты

Только ENV (`ADR-0001 M7 + SPEC_CURRENT §4`):
- `MINIMAX_API_KEY` — обязателен; иначе `ConfigError(section="tts.api_key")` в `init()`.
- `MINIMAX_GROUP_ID` — обязателен (query-param MiniMax T2A v2); иначе `ConfigError(section="tts.group_id")`.
- YAML-литералы запрещены. Плейсхолдер `${MINIMAX_API_KEY}` подставляется в `load_config()`.

## 3. Регистрация в capability registry (harness-side)

Имя ключа реестра: **`"minimax"`** (не `"minimax_tts"`, как ошибочно сказано в `t_aebd2d3b` body — задача писалась до согласования). Реестры — два уровня:

```python
# Upstream (src/rob_box_llm/.../tts_provider_registry.py) — landed в t_8cbf9995, ADR-0008
from rob_box_llm.tts_provider_registry import (
    TTSProviderRegistry, TTSProviderFactory, register_builtin_tts_providers,
)
registry = register_builtin_tts_providers()          # {"minimax": builder}
provider = TTSProviderFactory.create("minimax", cfg, registry)

# Harness-side (src/rob_box_harness/.../tts/registry.py) — landed в t_aebd2d3b
from rob_box_harness.tts.registry import (
    TTSProviderRegistry, TTSProviderFactory, register_builtin_tts_providers,
)
registry = register_builtin_tts_providers()          # {"minimax": build_minimax_tts_provider}
provider = TTSProviderFactory.create("minimax", tts_config, registry)
```

**Метаданные регистрации** (что отдаёт builder):

| Поле               | Источник                                                |
|--------------------|---------------------------------------------------------|
| `name`             | строковый ключ реестра (`"minimax"`)                    |
| `TTSCapabilities`  | из upstream `BaseTTSProvider.capabilities()` (см. ADR-0008 §2.2.1) — флаги `streaming`, `voice_cloning`, `audio_format_pcm/mp3`, ssml=False, ogg=False |
| `TTSVoice[]`       | из `list_voices()` — статический каталог 6 голосов ru/en/zh |
| `TTSHealth`        | из `healthcheck()` — credential-presence check, без HTTP |
| `RetryPolicy`      | `max_attempts=3, base=0.5` (экспоненциальный backoff + jitter) |
| `cache`            | опциональный content-hash cache по `hash(text+voice+format+sample_rate+model)` (default OFF) |
| `rate_limit_per_min` | soft cap, rolling 60s window (default OFF)              |

**Пример минимальной регистрации нового провайдера** (для ElevenLabs / Google / Piper):

```python
# src/rob_box_harness/rob_box_harness/tts/elevenlabs_tts.py
from rob_box_harness.tts.registry import TTSProviderRegistry
from rob_box_llm.tts import TTSSettings, TTSProvider, TTSAudio, TTSFormat

class ElevenLabsTTSProvider(TTSProvider):
    name = "elevenlabs"

    def __init__(self, *, api_key: str, default_voice: str = "Rachel", timeout: float = 30.0):
        self._client = httpx.AsyncClient(timeout=timeout, headers={"xi-api-key": api_key})
        self._default_voice = default_voice

    async def synthesize(self, text: str, *, settings: TTSSettings | None = None) -> TTSAudio:
        s = settings or TTSSettings()
        # ... POST /v1/text-to-speech/{voice_id} → mp3 bytes → TTSAudio
        ...

    async def stream(self, text, *, settings=None):
        # ... SSE chunks → yield TTSChunk(samples, sr, fmt, finish_reason)
        ...

    async def aclose(self) -> None:
        await self._client.aclose()


def register(reg: TTSProviderRegistry) -> TTSProviderRegistry:
    def _builder(cfg) -> ElevenLabsTTSProvider:
        return ElevenLabsTTSProvider(api_key=os.environ["ELEVENLABS_API_KEY"], timeout=cfg.timeout_s)
    reg.register("elevenlabs", _builder)
    return reg
```

## 4. Что в этом контракте **НЕ** меняется

- **`/voice/audio/speech` топик**: payload остаётся `audio_common_msgs/AudioData` с int16 LE PCM (ADR-0003 §2.3 invariant — `tts_node._publish_audio` уже умеет).
- **`TTSProvider` ABC**: frozen в `tts.py:121-177` (PR #907). Менять форму `synthesize`/`stream`/`aclose` = breaking change для `FakeTTSProvider` и всех downstream-импортов.
- **Capability-driven fallback**: не вводим (ADR-0002 §2.1 явно отвергнут для TTS; §3.7 — MiniMax не default).
- **Авто-регистрация через `importlib.metadata.entry_points()`**: не вводим (ADR-0004 §2.3 + ADR-0008 §5 явно отвергли).

## 5. Trade-off (почему `TTSSettings`, а не `**opts`)

| Решение                                | Альтернатива                         | Почему выбрано |
|----------------------------------------|--------------------------------------|----------------|
| **Единый `TTSSettings` value-object** | `synthesize(text, voice, **opts)`    | (1) Frozen dataclass → безопасная сериализация в логи/replay; (2) mypy strict-clean по всем полям (видно в `test_tts_value_objects.py`); (3) `MappingProxyType` в `extra` (см. ADR-0008 §3.2) — нельзя подменить подтипов через kwargs. Альтернатива `**opts` потребовала бы дублировать валидацию в каждом провайдере. |
| **Retry в harness-обёртке, не в провайдере** | retry в `_UpstreamMiniMaxTTSProvider` | (1) Провайдер остаётся stateless; (2) harness имеет контекст (dialogue_id, общий timeout диалога); (3) cron/CLI вызывает провайдера напрямую без retry (дешевле). См. ADR-0003 §2.6. |
| **Двухуровневый реестр (upstream + harness)** | один общий registry | (1) Upstream-реестр переиспользуется тестами `rob_box_llm` (387 тестов), (2) harness-реестр добавляет TTSConfig-обёртку + env-auth — каждый слой тестируется отдельно. Объединение сейчас не даёт выгоды. |
| **OGG → MP3 fallback с warning**       | конвертировать OGG локально через encoder | (1) YAGNI: ни один текущий consumer не просит OGG; (2) `pydub`/ffmpeg в production-зависимостях — лишний 80 MB. ADR-0003 §4 явно отклонил. |

## 6. Связанные документы

- [ADR-0001](0001-harness-architecture.md) — общий harness contract (immutable, MADR, Accepted)
- [ADR-0003](0003-minimax-tts-architecture.md) — детальный маппинг MiniMax T2A v2 ↔ TTSSettings, retry policy, output chain
- [ADR-0008](0008-tts-provider-extension-points-landed.md) — landed `BaseTTSProvider` + 5 extension points в `rob_box_llm`
- [`docs/architecture/tts-extension-points.md`](../architecture/tts-extension-points.md) — design-only stub с class/sequence диаграммами
- [`docs/architecture/minimax-tts-architecture.md`](../architecture/minimax-tts-architecture.md) — полная таблица маппинга полей TTSSettings → MiniMax JSON body
- [`src/rob_box_llm/rob_box_llm/tts.py`](../../src/rob_box_llm/rob_box_llm/tts.py) — frozen `TTSProvider` ABC, `TTSSettings`, `TTSAudio`, `TTSChunk`, `TTSFormat`, `FakeTTSProvider`
- [`src/rob_box_llm/rob_box_llm/tts_provider_base.py`](../../src/rob_box_llm/rob_box_llm/tts_provider_base.py) — `BaseTTSProvider`, `TTSCapabilities`, `TTSVoice`, `TTSHealth` (ADR-0008 §2.2)
- [`src/rob_box_llm/rob_box_llm/tts_provider_registry.py`](../../src/rob_box_llm/rob_box_llm/tts_provider_registry.py) — upstream registry + factory
- [`src/rob_box_harness/rob_box_harness/tts/minimax_tts.py`](../../src/rob_box_harness/rob_box_harness/tts/minimax_tts.py) — `HarnessMiniMaxTTSProvider`, `MiniMaxTTSProvider`, `build_minimax_tts_provider`, `RetryPolicy`, env-auth
- [`src/rob_box_harness/rob_box_harness/tts/registry.py`](../../src/rob_box_harness/rob_box_harness/tts/registry.py) — harness-side `TTSProviderRegistry`, `TTSProviderFactory`, `register_builtin_tts_providers`

## 7. Verification

```sh
$ poetry run -C src/rob_box_harness pytest -q
239 passed, 1 skipped in 2.24s                       # (1 skip — test_sanity network marker)
$ poetry run -C src/rob_box_harness pytest test/test_minimax_tts.py -q
64 passed in 0.39s                                   # harness-side TTSProvider suite
```

Coverage на `rob_box_harness/tts/minimax_tts.py` — 95% (gate 85%). Все 64 теста harness-suite зелёные на момент написания ADR.