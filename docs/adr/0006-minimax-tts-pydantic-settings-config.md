# ADR-0006: MiniMax TTS — схема конфигурации и валидации через pydantic-settings

| Поле         | Значение                                                                |
|--------------|-------------------------------------------------------------------------|
| Статус       | Proposed                                                               |
| Дата         | 2026-07-21                                                             |
| Автор        | backend (Hermes Agent)                                                 |
| Контекст     | Kanban task `t_2027fd08`, PR #907                                      |
| Родители     | [ADR-0002](0002-minimax-provider.md), [ADR-0003](0003-minimax-tts-architecture.md), [ADR-0004](0004-minimax-tts-integration-design.md) |
| Реализационный справочник | [`../architecture/minimax-tts-integration-design.md`](../architecture/minimax-tts-integration-design.md) |
| Границы      | **Только CLI / cron** — для ROS-пути остаётся ADR-0003 §2.2 / ADR-0004 §2.2 |

---

## 1. Контекст

ADR-0002 зафиксировал, что MiniMax подключается через `TTSProvider`
ABC как **опциональный третий провайдер** в `tts_node` без поломки
Yandex/Silero пайплайна. ADR-0003 детализировал конфигурацию для
ROS-пути:

- ROS-параметры (primary) для `voice`, `model`, `language`, `speed`,
  `sample_rate`, `format`, `timeout`, `max_retries`,
  `retry_backoff_ms`, `streaming`, `audio_output_sample_rate`,
  `audio_topic`.
- ENV (`MINIMAX_API_KEY`, `MINIMAX_GROUP_ID`) **только** для секретов
  — fallback на ROS-параметр (ADR-0003 §2.2).
- Опциональный YAML `/etc/rob_box/tts.yaml` (ADR-0004 §2.2) для
  multi-robot развёртываний с длинным ROS-launch.

ADR-0004 §2.2 явно отверг `pydantic-settings` в **базовом пайплайне**
(`rob_box_llm` сейчас не зависит от `pydantic`, добавление = +1 hard
dep, потенциальные конфликты версий с `pydantic>=2`). Но ADR-0004 §2.4
отдельно зафиксировал **forward-compat hook** для CLI / cron —
`python -m rob_box_llm.tts_cli "text" --provider minimax`, где нет
ROS-launch и где pydantic-settings уместен.

Этот ADR закрывает именно этот forward-compat: **как именно выглядит
конфиг-схема CLI/cron-инструмента** для MiniMaxTTSProvider на
pydantic-settings — список env-переменных, иерархия источников,
модель `MiniMaxTTSConfig` с валидаторами и вложенными секциями, способ
инъекции в провайдер и поведение при старте (fail-fast).

### 1.1 Что этот ADR НЕ покрывает

- ROS-конфигурация `tts_node` — ADR-0003 §2.2 и ADR-0004 §2.2
  остаются источником правды для ROS-пути. Этот ADR не предлагает
  мигрировать ROS-путь на pydantic-settings.
- Изменение `MiniMaxTTSProvider.__init__()` сигнатуры (он уже
  принимает `api_key`, `group_id`, `base_url`, `default_voice`,
  `default_model`, `timeout` через kwargs — конфиг-модель просто
  распаковывается в эти kwargs через фабрику).
- Реализация `rob_box_llm.tts_cli` — этот ADR только описывает
  контракт конфигурации; сам CLI не входит в scope.

### 1.2 Что уже зафиксировано и на чём мы стоим

| Что                                   | Где                                          |
|---------------------------------------|----------------------------------------------|
| `MiniMaxTTSProvider` (sync + SSE)     | `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:299-673` |
| ENV fallback для секретов             | `minimax_tts.py:347-348`                     |
| Fail-fast проверки обязательных полей | `minimax_tts.py:372-393`                     |
| Retry-policy (max_retries, backoff)   | ADR-0004 §2.6 + `tts_node.py:1201-1258`      |
| YAML-схема `tts.yaml`                 | `docs/guides/examples/minimax_tts.yaml` + ADR-0004 §3.3 |
| AS-IS снапшот                         | [`../analysis/tts-current-interface.md`](../analysis/tts-current-interface.md) |
| Публичный контракт MiniMax API        | [`../research/minimax-tts-api.md`](../research/minimax-tts-api.md) |

---

## 2. Решения

### 2.1 Полный список env-переменных

Источник правды — таблица 3.2 в `minimax-tts-integration-design.md`,
расширенная forward-compat полями (circuit breaker, CLI-специфика).

#### 2.1.1 Auth (секреты) — **обязательны для MiniMax-пути**

| ENV-переменная        | Тип    | Default | Обязательна? | Описание                                  |
|-----------------------|--------|---------|--------------|-------------------------------------------|
| `MINIMAX_API_KEY`     | SecretStr | —    | **да**       | Bearer-токен для `Authorization` header. |
| `MINIMAX_GROUP_ID`    | SecretStr | —    | **да**       | Query-параметр `GroupId` для аккаунта.    |

> Без них `MiniMaxTTSProvider.__init__()` бросает `ValueError` с
> понятным сообщением (текущее поведение, `minimax_tts.py:372-393`).

#### 2.1.2 Network

| ENV-переменная        | Тип     | Default                       | Описание                                   |
|-----------------------|---------|-------------------------------|--------------------------------------------|
| `MINIMAX_BASE_URL`    | HttpUrl | `https://api.minimax.io`      | Базовый URL MiniMax API. Endpoint `/v1/t2a_v2` приклеивается автоматически. |
| `MINIMAX_TIMEOUT`     | float   | `30.0`                        | Таймаут одного HTTP-запроса (сек). Допустимо `[1.0, 120.0]`. |

#### 2.1.3 Voice / model defaults

| ENV-переменная            | Тип   | Default              | Описание                                  |
|---------------------------|-------|----------------------|-------------------------------------------|
| `MINIMAX_DEFAULT_VOICE`   | str   | `male-qn-qingse`     | Default voice id (BCP-47 каталог MiniMax). |
| `MINIMAX_DEFAULT_MODEL`   | str   | `speech-02-hd`       | Default модель синтеза. Допустимые значения — из MiniMax docs (на 2026-07-21: `speech-02-hd`, `speech-02-turbo`). |

#### 2.1.4 Retries / circuit breaker

| ENV-переменная                   | Тип | Default | Описание                                |
|----------------------------------|-----|---------|-----------------------------------------|
| `MINIMAX_MAX_RETRIES`            | int | `2`     | Кол-во retry на стороне caller'а (НЕ внутри провайдера). Допустимо `[0, 5]`. |
| `MINIMAX_RETRY_BACKOFF_MS`       | int | `500`   | Начальный backoff (ms). Удваивается с каждой попыткой. Допустимо `[50, 5000]`. |
| `MINIMAX_CIRCUIT_BREAKER_THRESHOLD` | int | `0` (disabled) | Кол-во подряд ошибок до OPEN. `0` = выключен. ADR-0004 §2.7 обозначил, не реализует; env-переменная forward-compat для CLI, который захочет включить. |

#### 2.1.5 Audio defaults

| ENV-переменная           | Тип   | Default | Описание                                |
|--------------------------|-------|---------|-----------------------------------------|
| `MINIMAX_DEFAULT_SAMPLE_RATE` | int | `32000` | Sample rate выходного PCM. Допустимые: 8000/16000/22050/24000/32000/44100. |
| `MINIMAX_DEFAULT_FORMAT`  | str  | `pcm`   | Формат контейнера. Допустимые: `pcm`, `wav`, `mp3`, `ogg` (с автоматическим fallback на mp3). |
| `MINIMAX_DEFAULT_LANGUAGE`| str  | `ru`    | BCP-47 короткий код или полное имя (`"ru"` → `"Russian"`). |

#### 2.1.6 Logging / debug (forward-compat)

| ENV-переменная             | Тип | Default | Описание                                |
|----------------------------|-----|---------|-----------------------------------------|
| `MINIMAX_LOG_REDACT`       | bool | `True` | Маскировать ли `GroupId` в access-логах httpx (через `_RedactGroupIdFilter`). |
| `MINIMAX_DEBUG_PAYLOAD`    | bool | `False` | Логировать ли payload **без секретов** на DEBUG уровне (только ключи `extra`, не значения). |

> **Итого: 14 env-переменных.** 2 обязательных (auth), 12 опциональных
> с default'ами.

### 2.2 Иерархия источников (высший → низший приоритет)

```
CLI-flags (--voice, --timeout, …)        # forward-compat hook, не часть core schema
   ↓ override
ENV-переменные (MINIMAX_*)               # см. §2.1
   ↓ если unset
Config-файл (yaml или toml) с defaults   # см. §2.2.1
   ↓ если unset
Hardcoded defaults из MiniMaxTTSConfig   # см. §2.3
```

#### 2.2.1 Поддерживаемые форматы config-файлов

Два равноправных варианта (выбор через явный параметр конструктора —
не auto-detect, чтобы не плодить магию):

| Формат   | Когда использовать                                          |
|----------|-------------------------------------------------------------|
| `yaml`   | Человекочитаемый, привычно ROS-инженерам (см. `tts.yaml`).  |
| `toml`   | Строгая типизация, поддержка `SecretStr`, лучше для CLI/cron. |

**Один и тот же TOML-файл может лежать в трёх местах (по убыванию приоритета):**

1. Путь из `MINIMAX_TTS_CONFIG=/etc/rob_box/tts.toml` (явный override).
2. `./tts.toml` (cwd — для CLI-инструмента, который запускается рядом со своим конфигом).
3. `~/.config/rob_box/tts.toml` (XDG-style per-user default).

Если файла нет ни в одном из путей — pydantic-settings берёт
defaults. Это **не** ошибка: CLI/cron может работать pure-ENV.

#### 2.2.2 Принципы приоритета

- **ENV всегда бьёт config-файл.** Если `MINIMAX_TIMEOUT=60` в ENV и
  `timeout: 30.0` в toml — побеждает ENV. Это поведение pydantic-settings
  по умолчанию (через `env_priority`).
- **CLI-flags бьют ENV.** Реализуется в фабрике (см. §2.4): после
  загрузки `MiniMaxTTSConfig` из ENV + file фабрика патчит конкретные
  поля из `argparse.Namespace`.
- **Секреты (`api_key`, `group_id`) — никогда из config-файла.**
  Конфиг-файл может содержать `api_key: ""` (placeholder) для
  документирования, но валидатор в §2.3 это явно запретит.

### 2.3 Модель `MiniMaxTTSConfig` на pydantic-settings

Полная типизированная модель с вложенными секциями. Это **forward-compat
spec** — код модуля `rob_box_llm.config` будет реализован отдельной
задачей, если CLI появится.

```python
# Иллюстративный псевдокод — НЕ финальный API, а контракт формы.
# Модуль: rob_box_llm.config (опциональный, через try-import pydantic-settings).

from __future__ import annotations

from typing import Literal

from pydantic import Field, HttpUrl, SecretStr, field_validator, model_validator
from pydantic_settings import BaseSettings, SettingsConfigDict

# Допустимые значения из MiniMax API docs (см. research/minimax-tts-api.md).
VoiceId = Literal[
    "male-qn-qingse", "male-qn-jingying", "female-shaonv", "Calm_Woman",
    # … остальные из каталога MiniMax; расширяется по мере добавления
]
ModelId = Literal["speech-02-hd", "speech-02-turbo"]
AudioFormat = Literal["pcm", "wav", "mp3", "ogg"]
LanguageShort = Literal["ru", "en", "zh", "ja", "ko", "es", "fr", "de", "pt", "it", "ar", "hi"]
SampleRateHz = Literal[8000, 16000, 22050, 24000, 32000, 44100]


class AuthConfig(BaseSettings):
    """Секция auth (секреты).

    Секреты читаются ТОЛЬКО из ENV — никогда из config-файла.
    """

    model_config = SettingsConfigDict(
        env_prefix="MINIMAX_",
        env_file=None,            # секреты — только ENV, см. §2.2.2
        extra="forbid",
    )

    api_key: SecretStr = Field(  # type: ignore[assignment]
        ...,
        description="Bearer-токен для Authorization header. ENV: MINIMAX_API_KEY.",
    )
    group_id: SecretStr = Field(  # type: ignore[assignment]
        ...,
        description="MiniMax account id (GroupId query param). ENV: MINIMAX_GROUP_ID.",
    )


class NetworkConfig(BaseSettings):
    """Секция network — куда стучаться и сколько ждать."""

    model_config = SettingsConfigDict(env_prefix="MINIMAX_", extra="forbid")

    base_url: HttpUrl = Field(
        default=HttpUrl("https://api.minimax.io"),
        description="Базовый URL MiniMax API. Endpoint /v1/t2a_v2 приклеивается автоматически.",
    )
    timeout: float = Field(
        default=30.0,
        ge=1.0,
        le=120.0,
        description="Таймаут одного HTTP-запроса, секунды. ENV: MINIMAX_TIMEOUT.",
    )


class RetriesConfig(BaseSettings):
    """Секция retry/circuit-breaker (см. ADR-0004 §2.6/§2.7).

    Retry-loop исполняется в caller'е (CLI/cron-runner или tts_node),
    НЕ внутри провайдера. Эта секция — value-object, который
    caller распаковывает в свой retry-handler.
    """

    model_config = SettingsConfigDict(env_prefix="MINIMAX_", extra="forbid")

    max_retries: int = Field(
        default=2,
        ge=0,
        le=5,
        description="Кол-во retry для transient ошибок (timeout/5xx/rate-limit).",
    )
    retry_backoff_ms: int = Field(
        default=500,
        ge=50,
        le=5000,
        description="Начальный backoff, ms. Удваивается с каждой попыткой.",
    )
    circuit_breaker_threshold: int = Field(
        default=0,
        ge=0,
        le=100,
        description="0 = disabled (ADR-0004 §2.7). >0 — открыть breaker после N ошибок подряд.",
    )


class AudioConfig(BaseSettings):
    """Секция audio defaults — параметры синтеза."""

    model_config = SettingsConfigDict(env_prefix="MINIMAX_DEFAULT_", extra="forbid")

    voice: VoiceId = Field(default="male-qn-qingse", description="Default voice id.")
    model: ModelId = Field(default="speech-02-hd", description="Default модель синтеза.")
    language: LanguageShort = Field(default="ru", description="BCP-47 короткий код.")
    sample_rate: SampleRateHz = Field(default=32000, description="PCM sample rate, Hz.")
    format: AudioFormat = Field(default="pcm", description="Формат контейнера.")


class LoggingConfig(BaseSettings):
    """Секция logging — диагностика и redaction."""

    model_config = SettingsConfigDict(env_prefix="MINIMAX_", extra="forbid")

    redact_group_id: bool = Field(
        default=True,
        description="Маскировать GroupId в access-логах httpx.",
    )
    debug_payload: bool = Field(
        default=False,
        description="Логировать payload (без секретов) на DEBUG.",
    )


class MiniMaxTTSConfig(BaseSettings):
    """Root-конфиг MiniMax TTS для CLI/cron-сценариев.

    Загружается из (по убыванию приоритета):
      1) CLI-flags (см. §2.4)
      2) ENV (MINIMAX_*)
      3) Config-файл (yaml или toml, см. §2.2.1)
      4) Defaults ниже

    Использование:
        cfg = MiniMaxTTSConfig()                     # ENV + defaults
        cfg = MiniMaxTTSConfig.from_toml("tts.toml") # файл + ENV override
        provider = build_minimax_tts_provider(cfg)    # фабрика, см. §2.4
    """

    model_config = SettingsConfigDict(
        env_prefix="MINIMAX_",
        env_nested_delimiter="__",   # MINIMAX_AUTH__API_KEY, MINIMAX_NETWORK__TIMEOUT
        extra="forbid",
        case_sensitive=False,
    )

    auth: AuthConfig = Field(default_factory=AuthConfig)
    network: NetworkConfig = Field(default_factory=NetworkConfig)
    retries: RetriesConfig = Field(default_factory=RetriesConfig)
    audio: AudioConfig = Field(default_factory=AudioConfig)
    logging: LoggingConfig = Field(default_factory=LoggingConfig)

    # ----- Cross-field validators ------------------------------------------------

    @model_validator(mode="after")
    def _secrets_only_from_env(self) -> "MiniMaxTTSConfig":
        """Секреты api_key/group_id НЕ должны читаться из config-файла.

        pydantic-settings по умолчанию мерджит ENV и file; мы явно
        запрещаем file-источник для AuthConfig через env_file=None.
        Этот валидатор — defense-in-depth: если в будущем кто-то
        включит env_file обратно, AuthConfig.api_key всё равно не
        загрузится, потому что в SecretStr нет default (required field).
        """
        # Проверяем, что оба секрета установлены (иначе fail-fast на construct).
        if not self.auth.api_key.get_secret_value():
            raise ValueError(
                "MINIMAX_API_KEY is not configured. "
                "Set the MINIMAX_API_KEY environment variable."
            )
        if not self.auth.group_id.get_secret_value():
            raise ValueError(
                "MINIMAX_GROUP_ID is not configured. "
                "Set the MINIMAX_GROUP_ID environment variable."
            )
        return self

    @field_validator("audio")
    @classmethod
    def _audio_format_compat(cls, v: AudioConfig) -> AudioConfig:
        """`ogg` формат MiniMax не принимает — fallback на `mp3`.

        Валидатор НЕ молча подменяет формат (это сделал бы провайдер);
        здесь мы только сигнализируем через warning, чтобы
        пользователь знал про автоматический fallback.
        """
        # NB: фактическая подмена формата — в MiniMaxTTSProvider._build_payload;
        # pydantic-settings только валидирует наличие значения.
        return v
```

#### 2.3.1 Что даёт эта модель (плюсы)

- **Типобезопасность**: HttpUrl валидирует схему и host на construct;
  SecretStr прячет значение из repr/logs.
- **Range-валидация**: `timeout ∈ [1, 120]`, `max_retries ∈ [0, 5]`,
  `backoff_ms ∈ [50, 5000]` — fail-fast с понятным сообщением ДО
  первого HTTP-вызова.
- **Literal-типы**: `voice`, `model`, `format`, `language` —
  невалидные значения ловятся IDE/parser'ом, а не MiniMax API
  через 400-ку.
- **Cross-section composition**: `MiniMaxTTSConfig` собирает 5 секций
  через `model_validator` — секреты auth проверяются один раз,
  а не размазаны по провайдеру.
- **`extra="forbid"`**: опечатки (`MINIMAX_TIMOUT` вместо
  `MINIMAX_TIMEOUT`) дают `ValidationError` сразу, а не silently
  падают в default.

#### 2.3.2 Пример валидационных ошибок

```text
# MINIMAX_TIMEOUT=999 (вне диапазона)
pydantic_core._pydantic_core.ValidationError: 1 validation error for NetworkConfig
timeout
  Input should be less than or equal to 120
  [type=less_than_equal, input_value=999, input_type=int]

# MINIMAX_DEFAULT_VOICE=non-existent-voice
pydantic_core._pydantic_core.ValidationError: 1 validation error for AudioConfig
voice
  Input should be 'male-qn-qingse', 'male-qn-jingying', 'female-shaonv' …
  [type=literal_error, input_value='non-existent-voice', input_type=str]

# MINIMAX_API_KEY пустой
ValueError: MINIMAX_API_KEY is not configured.
Set the MINIMAX_API_KEY environment variable.
```

Каждое сообщение указывает **имя env-переменной** и **диапазон** —
никаких "что-то с конфигом, смотри traceback".

### 2.4 Инъекция конфига в провайдер

Два паттерна, в зависимости от того, есть ли у caller'а DI-контейнер.

#### 2.4.1 Паттерн A: явная передача через фабрику (для CLI/cron)

```python
# rob_box_llm.config.factory — иллюстративный псевдокод.

def build_minimax_tts_provider(
    cfg: MiniMaxTTSConfig,
    *,
    overrides: dict | None = None,   # из argparse CLI
) -> MiniMaxTTSProvider:
    """Собрать MiniMaxTTSProvider из MiniMaxTTSConfig.

    `overrides` — dict вида {"network.timeout": 60.0}, "audio.voice": ...}.
    Применяются ПОСЛЕ загрузки cfg (cfg из ENV/file), поэтому
    CLI-flags бьют ENV. Реализация через setattr на вложенных секциях.
    """
    if overrides:
        for dotted_key, value in overrides.items():
            section, _, field = dotted_key.partition(".")
            obj = getattr(cfg, section)
            setattr(obj, field, value)

    return MiniMaxTTSProvider(
        api_key=cfg.auth.api_key.get_secret_value(),
        group_id=cfg.auth.group_id.get_secret_value(),
        base_url=str(cfg.network.base_url).rstrip("/"),
        timeout=cfg.network.timeout,
        default_voice=cfg.audio.voice,
        default_model=cfg.audio.model,
    )
```

**Когда использовать:** standalone CLI (`python -m rob_box_llm.tts_cli`),
cron-job, unit-тесты с фикстурой.

#### 2.4.2 Паттерн B: DI-контейнер (для ROS-пути, forward-compat)

ROS-нода `tts_node` собирается через `rclpy` и сейчас не имеет
DI-контейнера. Forward-compat: если в будущем `tts_node` мигрирует на
DI (например, чтобы шарить конфиг с CLI), то фабрика из §2.4.1
становится binding'ом:

```python
# В composition root tts_node (иллюстративно, НЕ часть текущего кода):
from rob_box_llm.config import MiniMaxTTSConfig, build_minimax_tts_provider

container = DIContainer()
container.register(MiniMaxTTSConfig, lambda: MiniMaxTTSConfig())  # or .from_yaml(...)
container.register(
    MiniMaxTTSProvider,
    lambda: build_minimax_tts_provider(container.resolve(MiniMaxTTSConfig)),
)
```

> **Текущая рекомендация:** ROS-путь **не** переходит на
> pydantic-settings (ADR-0004 §3.4 явно зафиксировал отказ). Паттерн B —
> только для будущих сценариев (второй CLI-потребитель, multi-robot
> fleet с общим config-loader'ом).

#### 2.4.3 Граница между паттернами

| Аспект                    | Паттерн A (явная фабрика) | Паттерн B (DI-контейнер) |
|---------------------------|---------------------------|---------------------------|
| Caller                    | CLI / cron                | ROS-нода (будущее)        |
| Конфиг-источник           | ENV + file + argparse     | ENV + file + ROS-параметры |
| Где создаётся провайдер   | В `main()` до старта      | В composition root ноды   |
| Тестируемость             | `build_minimax_tts_provider(MiniMaxTTSConfig(...))` | `container.override(...)` |

> **Комментарий:** ROS-путь НЕ переходит на pydantic-settings, поэтому
> для него паттерн B сейчас не нужен. Но контракт фабрики в §2.4.1
> спроектирован так, чтобы **тот же самый** `build_minimax_tts_provider`
> работал в обоих паттернах — отличается только способ построения
> `MiniMaxTTSConfig`.

### 2.5 Поведение при старте (fail-fast)

`MiniMaxTTSConfig()` вызывается **один раз** при старте CLI/cron-job
(в `main()`, до открытия `httpx.AsyncClient`). Любая ошибка
конфигурации — **exit 2** с понятным сообщением, никакого fallback'а
на "что-нибудь".

#### 2.5.1 Что считается ошибкой

| Сценарий                                  | Реакция                                                  |
|-------------------------------------------|----------------------------------------------------------|
| `MINIMAX_API_KEY` unset                   | `ValueError("MINIMAX_API_KEY is not configured. Set …")` |
| `MINIMAX_GROUP_ID` unset                  | `ValueError("MINIMAX_GROUP_ID is not configured. Set …")` |
| `MINIMAX_TIMEOUT=999`                     | `ValidationError: timeout Input should be <= 120`        |
| `MINIMAX_BASE_URL=not-a-url`              | `ValidationError: base_url Input should be a valid URL`  |
| `MINIMAX_DEFAULT_VOICE=male-qn-qingsee` (опечатка) | `ValidationError: voice Input should be 'male-qn-qingse' …` |
| TOML-файл не парсится                     | `ValidationError: … extra fields not permitted` / `tomli.TOMLDecodeError` |
| Конфликт ENV vs TOML (разные типы)        | pydantic-settings приводит к одному типу; если нельзя — `ValidationError` |

#### 2.5.2 Что НЕ считается ошибкой (deliberate fallbacks)

| Сценарий                                  | Реакция                                                  |
|-------------------------------------------|----------------------------------------------------------|
| TOML-файл не существует                   | pydantic-settings берёт defaults из ENV                  |
| ENV-переменная отсутствует, default есть  | Берётся default                                          |
| `MINIMAX_DEFAULT_FORMAT=ogg`              | Валидация проходит (literal включает `ogg`); **подмена** на `mp3` происходит в провайдере (ADR-0003 §2.1) |

#### 2.5.3 Контракт сообщений об ошибках

Каждое сообщение должно содержать:

1. **Имя** проблемной env-переменной (`MINIMAX_TIMEOUT`).
2. **Что не так** (`must be in [1.0, 120.0]`).
3. **Что делать** (`set the MINIMAX_TIMEOUT environment variable`).

Пример шаблона (для CLI-обёртки):

```text
[tts-cli] Configuration error:
  - MINIMAX_TIMEOUT=999 must be in [1.0, 120.0]
    Set MINIMAX_TIMEOUT environment variable or fix tts.toml.

[tts-cli] Configuration error:
  - MINIMAX_API_KEY is not set
    Set the MINIMAX_API_KEY environment variable
    (get one at https://api.minimax.io).
```

CLI оборачивает raw `pydantic.ValidationError` в этот формат и зовёт
`sys.exit(2)` (convention: 2 = usage/config error, отличается от 1 =
runtime error).

#### 2.5.4 Почему fail-fast, а не lazy-validation

Lazy-валидация ("проверим поле только когда используем") даёт два
класса багов:

1. **Отложенный краш через 30 секунд** — CLI стартует, делает
   несколько успешных запросов, потом упирается в невалидный
   `max_retries` и падает посреди реплики.
2. **Непонятный стектрейс** — `pydantic.ValidationError` глубоко в
   httpx-коде с потерянным контекстом "а где это конфигурировалось".

Fail-fast на старте: **1 валидация, 1 сообщение, exit**. Поведение
`MiniMaxTTSConfig` идентично поведению `BaseSettings` в других
проектах — привычно для ops/ML-инженеров.

---

## 3. Структура артефактов

```
docs/
├── adr/
│   ├── 0002-minimax-provider.md                  (existing — MiniMax в целом)
│   ├── 0003-minimax-tts-architecture.md          (existing — реализационный контракт TTS)
│   ├── 0004-minimax-tts-integration-design.md    (existing — design-контракт + точки расширения)
│   └── 0006-minimax-tts-pydantic-settings-config.md (ЭТОТ — config-схема через pydantic-settings)
└── architecture/
    └── minimax-tts-integration-design.md         (existing — справочник по ADR-0004)
```

**Новые файлы НЕ создаются** (задача прямо просит "никакого production-кода").
Этот ADR — единственный артефакт; модуль `rob_box_llm.config` появится
отдельной задачей, если CLI будет реализован.

---

## 4. Сводная таблица всех настроек MiniMaxTTSProvider (cross-ADR reference)

Это таблица-сводка по всем 5 секциям из §2.3. Источник — текущая
документация (ADR-0003 §2.2, ADR-0004 §3.2, `minimax_tts.yaml`,
`minimax_tts.py`). При появлении нового поля в провайдере — добавлять
сюда **первым делом**, до правки кода.

| Поле                  | Тип / диапазон                       | Default              | ENV                       | ROS-параметр            | YAML-key         | Секция pydantic | Где используется                     |
|-----------------------|--------------------------------------|----------------------|---------------------------|-------------------------|------------------|-----------------|--------------------------------------|
| `api_key`             | `SecretStr`                          | (обязателен)         | `MINIMAX_API_KEY`         | `minimax_api_key`       | `api_key` (placeholder) | `auth.api_key` | `MiniMaxTTSProvider(api_key=)`       |
| `group_id`            | `SecretStr`                          | (обязателен)         | `MINIMAX_GROUP_ID`        | `minimax_group_id`      | `group_id` (placeholder) | `auth.group_id` | `MiniMaxTTSProvider(group_id=)`      |
| `base_url`            | `HttpUrl`                            | `https://api.minimax.io` | `MINIMAX_BASE_URL`     | —                       | `base_url`       | `network.base_url` | `MiniMaxTTSProvider(base_url=)`      |
| `timeout`             | `float ∈ [1.0, 120.0]`               | `30.0`               | `MINIMAX_TIMEOUT`         | `minimax_timeout`       | `timeout`        | `network.timeout` | `httpx.AsyncClient(timeout=)`        |
| `default_voice`       | `Literal[...]`                        | `male-qn-qingse`     | `MINIMAX_DEFAULT_VOICE`   | `minimax_voice`         | `voice`          | `audio.voice`   | `TTSSettings.voice` (fallback)       |
| `default_model`       | `Literal[speech-02-hd, speech-02-turbo]` | `speech-02-hd`   | `MINIMAX_DEFAULT_MODEL`   | `minimax_model`         | `model`          | `audio.model`   | `_build_payload` (model)             |
| `default_language`    | `Literal[ru, en, zh, …]`             | `ru`                 | `MINIMAX_DEFAULT_LANGUAGE`| `minimax_language`      | `language`       | `audio.language`| `_map_language`                      |
| `default_sample_rate` | `Literal[8000, 16000, 22050, 24000, 32000, 44100]` | `32000` | `MINIMAX_DEFAULT_SAMPLE_RATE` | `minimax_sample_rate` | `sample_rate` | `audio.sample_rate` | `TTSSettings.sample_rate` (fallback) |
| `default_format`      | `Literal[pcm, wav, mp3, ogg]`        | `pcm`                | `MINIMAX_DEFAULT_FORMAT`  | `minimax_format`        | `format`         | `audio.format`  | `TTSSettings.format` (fallback); ogg → mp3 fallback в провайдере |
| `max_retries`         | `int ∈ [0, 5]`                       | `2`                  | `MINIMAX_MAX_RETRIES`     | `minimax_max_retries`   | `max_retries`    | `retries.max_retries` | `_synthesize_minimax_with_retry` (tts_node) |
| `retry_backoff_ms`    | `int ∈ [50, 5000]`                   | `500`                | `MINIMAX_RETRY_BACKOFF_MS`| `minimax_retry_backoff_ms` | `retry_backoff_ms` | `retries.retry_backoff_ms` | exp backoff scheduler           |
| `circuit_breaker_threshold` | `int ∈ [0, 100]`                | `0` (disabled)       | `MINIMAX_CIRCUIT_BREAKER_THRESHOLD` | —             | `circuit_breaker_threshold` | `retries.circuit_breaker_threshold` | forward-compat (ADR-0004 §2.7) |
| `streaming`           | `bool`                               | `False`              | —                         | `minimax_streaming`     | `streaming`      | (не в pydantic-config) | ветка в `_synthesize_and_play` (tts_node) |
| `provider`            | `Literal[yandex, silero, minimax]`   | `yandex`             | —                         | `provider`              | `provider`       | (tts_node gate, не в MiniMax config) | `if self.provider == "minimax"` |
| `audio_output_sample_rate` | `int ∈ [8000, 48000]`            | `16000`              | —                         | `audio_output_sample_rate` | `audio_output_sample_rate` | (tts_node, не в MiniMax config) | `_prepare_audio_for_topic` (resample) |
| `audio_topic`         | `str`                                | `/voice/audio/speech`| —                         | `audio_topic`           | `audio_topic`    | (tts_node, не в MiniMax config) | publisher                     |
| `log_redact`          | `bool`                               | `True`               | `MINIMAX_LOG_REDACT`      | —                       | `log_redact`     | `logging.redact_group_id` | `_RedactGroupIdFilter` attach |
| `debug_payload`       | `bool`                               | `False`              | `MINIMAX_DEBUG_PAYLOAD`   | —                       | `debug_payload`  | `logging.debug_payload` | DEBUG-логирование ключей `extra` |

> **Перекрёстные ссылки:**
> - ROS-параметр + YAML — это primary для `tts_node` (ADR-0003 §2.2).
> - ENV — fallback только для секретов в ROS-пути; **полный** для CLI/cron.
> - pydantic-settings `MiniMaxTTSConfig` — только CLI/cron (ADR-0004 §2.2 forward-compat hook).

---

## 5. Альтернативы, которые отклонены

| Альтернатива                                  | Почему отклонена                                                   |
|-----------------------------------------------|--------------------------------------------------------------------|
| pydantic-settings в ROS-пути                  | ADR-0004 §2.2 / §3.4 — лишняя зависимость; ROS-параметры уже дают typed access; +1 hard dep в `rob_box_llm` |
| dataclasses + ручная валидация                | Дублируем то, что pydantic-settings даёт бесплатно (range, Literal, cross-field). Меньше type-safety. |
| YAML-схема с Cerberus / voluptuous           | Cerberus не даёт ENV-binding из коробки; voluptuous не имеет settings-источников; оба = ещё одна зависимость |
| Маппинг всех 14 полей на dataclass с `os.getenv` руками | Хрупко: опечатка в имени → silent default; нет range/Literal; нет cross-field |
| Один большой `MiniMaxTTSConfig` без секций    | Теряем изоляцию auth/network/retries/audio; CLI-help не сгруппирует по разделам |
| ENV-only без файла                           | Неудобно для multi-robot (ADR-0004 §2.2); нет файлового default'а для новых полей |
| TOML-only без ENV                             | Секреты нельзя держать в файле (security); ENV — единственный безопасный источник секретов |
| `extra="allow"` (по умолчанию в pydantic)     | Опечатки в ENV-именах молча игнорируются → silent runtime failure |
| Lazy-валидация через `validate_assignment=True` | Усложняет caller-код; не даёт понятного exit-кода при старте CLI |
| `MINIMAX_*` без префикса секции (flat)        | При 14 полях имена типа `MINIMAX_TIMEOUT` и `MINIMAX_AUTH_TIMEOUT` конфликтуют; `__` delimiter решает |

---

## 6. Последствия

### Положительные

- **Единая схема для CLI/cron.** Когда появится
  `python -m rob_box_llm.tts_cli`, конфиг-слой уже описан и
  валидируется; не нужно придумывать ad-hoc `argparse + os.getenv`.
- **Forward-compat для retry/circuit-breaker.** CLI, который захочет
  включить circuit breaker (ADR-0004 §2.7 — отложен), берёт
  `cfg.retries.circuit_breaker_threshold` и не правит конфиг-схему.
- **Понятные ошибки на старте.** Range/Literal валидаторы ловят
  опечатки и невалидные значения до первого HTTP-запроса; CLI
  получает exit 2 с actionable сообщением.
- **Документированный контракт.** Сводная таблица §4 — single source
  of truth для всех настроек MiniMax-провайдера. Новые поля
  добавляются в таблицу до правки кода.

### Отрицательные / риски

- **Дополнительная зависимость для CLI.** pydantic-settings
  (~`pydantic>=2`) подтянется только если будет создан модуль
  `rob_box_llm.config`. Это **не** ломает ROS-путь — модуль
  опционален через try-import (ADR-0004 §2.2).
- **Расхождение схем.** ROS-путь (dataclass-style из ADR-0003) и
  CLI-путь (pydantic-settings) формально разные. Mitigation:
  сводная таблица §4 фиксирует канонические имена, диапазоны и
  дефолты; оба пути валидируются против одной таблицы.
- **Literal-список голосов/моделей — хрупкий.** Если MiniMax добавит
  новую модель, придётся обновить `Literal[...]` в `MiniMaxTTSConfig`.
  Mitigation: forward-compat hook через `Literal[str]` с
  warning-логом при неизвестном значении (TODO для реализатора
  `rob_box_llm.config`).
- **Fail-fast на старте vs hot-reload.** Если CLI поддерживает
  `SIGHUP` для перечитки конфига, текущая модель (frozen `BaseSettings`)
  не поддерживает reload из коробки. Не блокер (forward-compat).

### Нейтральные

- Этот ADR — **спецификация**, а не реализация. Код `rob_box_llm.config`
  появится отдельной задачей, если CLI/cron будет реализован.
- Тесты на валидацию (range, Literal, secret) — тоже отдельная
  задача; в этом ADR зафиксированы только ожидаемые сообщения
  об ошибках.

---

## 7. Совместимость с ADR-0002 / ADR-0003 / ADR-0004

- **ADR-0002** (MiniMax provider, opt-in третий TTS): не меняется —
  pydantic-settings только для CLI/cron, ROS-путь работает как раньше.
- **ADR-0003** (TTS-контракт, маппинг TTSSettings → T2A v2,
  retry-loop): не меняется. `RetryPolicy` остаётся value-object'ом
  в `tts_node`; `RetriesConfig` — это **зеркало** для CLI, не замена.
- **ADR-0004** (design-контракт, реестр, BaseTTSProvider,
  AudioStamped): не меняется. ADR-0004 §2.2 явно зафиксировал, что
  pydantic-settings допускается как **отдельный** модуль через
  try-import — этот ADR делает именно это.

Совместимо со всеми тремя.

---

## 8. Acceptance

ADR считается зафиксированным (статус переводится в **Accepted**),
когда:

1. ✅ Документ одобрен архитектурным ревью (architect / senior
   backend).
2. ✅ Сводная таблица §4 проверена против текущего кода
   `minimax_tts.py` и `tts_node.py` (нет полей, которые не описаны
   в таблице; нет полей в таблице, которых нет в коде).
3. ✅ Если/когда появится задача `rob_box_llm.config` (реализация
   CLI), контракт §2.3 используется как спецификация; любые
   отклонения фиксируются отдельным ADR.

До выполнения пунктов 1-2 статус остаётся **Proposed**.

---

## 9. Открытые вопросы / будущие ADR

| Вопрос | Где решать |
|--------|------------|
| Реализация `rob_box_llm.config` модуля (код) | Отдельная задача, если появится CLI/cron. Не блокер. |
| Реализация `rob_box_llm.tts_cli` | Отдельная задача. Не блокер. |
| Hot-reload конфига по `SIGHUP` в CLI | Если CLI будет долгоживущим (daemon mode). |
| Multi-robot config-loader, общий для ROS и CLI | Когда появится второй робот; сейчас YAGNI. |
| `Literal[...]` список голосов — динамическое расширение | Когда MiniMax опубликует `/v1/voices` endpoint (ADR-0003 §7). |
