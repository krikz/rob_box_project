# ADR-0004: MiniMax TTS — design-контракт интеграции и точки расширения

| Поле         | Значение                                                                |
|--------------|-------------------------------------------------------------------------|
| Статус       | Proposed                                                               |
| Дата         | 2026-07-21                                                             |
| Автор        | architect (Hermes Agent)                                                |
| Контекст     | Kanban task `t_b4cb8948`, PR #907, потомки `t_25b8e221` (реализация)   |
| Родители     | [ADR-0002](0002-minimax-provider.md), [ADR-0003](0003-minimax-tts-architecture.md) |
| Реализационный справочник | [`../architecture/minimax-tts-integration-design.md`](../architecture/minimax-tts-integration-design.md) |
| Sequence-диаграмма | [`../diagrams/minimax-tts-integration-sequence.mmd`](../diagrams/minimax-tts-integration-sequence.mmd) |
| Class-диаграмма | [`../diagrams/minimax-tts-integration-class.mmd`](../diagrams/minimax-tts-integration-class.mmd) |
| Дополнительные исследования | [`../analysis/tts-current-interface.md`](../analysis/tts-current-interface.md), [`../research/minimax-tts-api.md`](../research/minimax-tts-api.md) |

---

## 1. Контекст

ADR-0002 зафиксировал, что MiniMax подключается через `TTSProvider`
ABC как **опциональный третий провайдер** в `tts_node` без поломки
Yandex/Silero пайплайна. ADR-0003 детализировал реализационный
контракт на стыке `tts_node ↔ MiniMaxTTSProvider ↔ api.minimax.io`
(маппинг `TTSSettings` → T2A v2 body, цепочка форматов
`hex PCM → TTSAudio → int16 AudioData`, retry-policy на стороне
`tts_node`).

К моменту принятия ADR-0003 остались открытыми (см. ADR-0003 §7):

- **Реестр провайдеров** — сегодня нет registry/factory,
  `tts_node` импортирует `MiniMaxTTSProvider` напрямую и
  переключает ветку через `if self.provider == "minimax": …`
  (документ `tts-current-interface.md`, smell D1/X-1,
  `tts_node.py:740-756`).
- **Hook для конфигурации** через файл (YAML / pydantic-settings) —
  сейчас только ROS-параметры и ENV для секретов.
- **Circuit breaker** — сейчас есть retry с exp backoff, но нет
  порога "API явно лежит, не дёргаем".
- **Streaming как forward-compat hook** — `MiniMaxTTSProvider.stream()`
  буферизует полный SSE-ответ; WebSocket chunk-per-frame остался в
  M5/M6 без явной точки входа.
- **AudioStamped-сообщение** как альтернатива `audio_common_msgs/AudioData`
  без метаданных — отложено.

Этот ADR закрывает **дизайн-контракт интеграции**: что именно
должна уметь любая будущая TTS-имплементация в нашей фабрике,
как она регистрируется, как конфигурируется, как ведёт себя под
нагрузкой, и как мы расширяем аудиостэк. Реализационная
детализация (конкретные маппинги полей) остаётся в
[`minimax-tts-architecture.md`](../architecture/minimax-tts-architecture.md)
и ADR-0003.

### 1.1 Что этот ADR НЕ покрывает

- Изменение `TTSProvider` ABC (`tts.py`) — frozen на P0.5.
- Замену Yandex/Silero на MiniMax по умолчанию — ADR-0002 §2.1
  зафиксировал opt-in.
- WebSocket-стриминг chunk-per-frame — отдельный ADR в M5/M6.
- Capability registry (`CapabilityUnavailableError`) — ADR-0002 §6
  отложил до второго потребителя; для TTS-стороны пока нет такого
  потребителя.

### 1.2 Что уже зафиксировано и на чём мы стоим

| Что                                  | Где                                          |
|--------------------------------------|----------------------------------------------|
| `TTSProvider` ABC + value-объекты    | `src/rob_box_llm/rob_box_llm/tts.py:121-177` |
| Иерархия `TTSError`                  | `src/rob_box_llm/rob_box_llm/errors.py:63-90` |
| `MiniMaxTTSProvider` (sync + SSE)    | `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:299-673` |
| Wire в `tts_node.py`                 | `src/rob_box_voice/rob_box_voice/tts_node.py:160-289, 740-756, 1148-1199, 1201-1258, 1260-1327` |
| Retry-loop в tts_node                | `tts_node.py:1201-1258`                       |
| AS-IS snapshot                       | [`../analysis/tts-current-interface.md`](../analysis/tts-current-interface.md) (502 строк) |
| Публичный контракт MiniMax API       | [`../research/minimax-tts-api.md`](../research/minimax-tts-api.md) (514 строк) |
| ADR верхнего уровня                  | [ADR-0002](0002-minimax-provider.md)         |
| Реализационный справочник TTS        | [ADR-0003](0003-minimax-tts-architecture.md) |

---

## 2. Решения

### 2.1 Единый порт `BaseTTSProvider`

**Бизнес-проблема:** `tts_node`, CLI и будущие потребители не должны знать
особенности MiniMax, ElevenLabs, Google Cloud TTS или локальной модели. Им
нужен один заменяемый контракт, который можно подменить тестовым адаптером.

**Решение:** сохранить существующий `TTSProvider` ABC как совместимый alias
на время миграции, а целевое имя порта — `BaseTTSProvider`. Это тонкий ABC,
а не место для retry, circuit breaker или vendor-specific конфигурации:

```python
# Иллюстративный контракт, production-кода в этом ADR нет.
class BaseTTSProvider(ABC):
    name: ClassVar[str]

    @abstractmethod
    async def synthesize(self, text: str, *, settings: TTSSettings) -> TTSAudio: ...

    @abstractmethod
    def stream(
        self, text: str, *, settings: TTSSettings
    ) -> AsyncIterator[TTSChunk]: ...

    async def list_voices(self) -> Sequence[TTSVoice]:
        """Опциональная capability; default — пустой список."""
        return ()

    async def healthcheck(self) -> TTSHealth:
        """Проверка готовности без синтеза пользовательского текста."""
        return TTSHealth(healthy=True)

    async def aclose(self) -> None:
        """Idempotent lifecycle hook."""
```

Обязательными являются `synthesize()`, `stream()` и idempotent `aclose()`.
`list_voices()` и `healthcheck()` имеют безопасные default-реализации, потому
что локальная модель может не иметь удалённого voice catalog/health endpoint.
`healthcheck()` не должен отправлять пользовательский текст или создавать
платный synthesis-вызов, если провайдер допускает дешёвую локальную проверку.
Все провайдеры возвращают общие value-objects и поднимают иерархию
`TTSError`; vendor DTO и HTTP-клиенты наружу не выходят.

**Самая простая альтернатива:** оставить прямой импорт MiniMax и очередной
`if provider == ...` в каждом потребителе. Она дешевле для одного провайдера,
но при втором размножает ветвления, конфиг и тесты.

**Trade-off:** ещё одна абстракция и conformance-suite против заменяемости и
единой поверхности ошибок. Если не вводить порт сейчас, первая интеграция
останется рабочей, но стоимость второго провайдера будет включать рефакторинг
всех потребителей.

### 2.2 `MiniMaxTTSProvider` как HTTP/WS adapter

`MiniMaxTTSProvider` реализует `BaseTTSProvider` и инкапсулирует transport:

- `synthesize()` строит T2A HTTP payload, вызывает sync HTTP endpoint,
  декодирует ответ в `TTSAudio`;
- `stream()` скрывает SSE сегодня и допускает WS transport позднее, но наружу
  всегда возвращает `AsyncIterator[TTSChunk]`;
- `list_voices()` обращается к MiniMax voice catalog, если endpoint доступен;
  при отсутствии capability возвращает пустой список либо документированную
  статическую выборку, не маскируя сетевую ошибку как успех;
- `healthcheck()` проверяет локальную валидность конфигурации и доступность API
  без логирования `api_key`, `group_id` или текста;
- `_build_payload`, `_map_exception`, HTTP/WS client и vendor response DTO
  остаются private деталями адаптера;
- `aclose()` закрывает только клиент, которым владеет экземпляр, и остаётся
  idempotent.

Retry остаётся политикой orchestration-слоя: автоматический retry внутри
провайдера сделал бы число платных запросов неявным и затруднил бы отмену
streaming-вызовов. DI используется для передачи HTTP/WS client, clock и
настроек — это позволяет тестировать адаптер без сети.

### 2.3 Registry + Factory: одна composition root

```python
ProviderBuilder = Callable[[Mapping[str, Any]], BaseTTSProvider]

class TTSProviderRegistry:
    def register(self, name: str, builder: ProviderBuilder) -> None: ...
    def get(self, name: str) -> ProviderBuilder: ...
    def names(self) -> tuple[str, ...]: ...


def register_tts_provider(name: str):
    """Decorator, регистрирующий builder/class в default registry."""
    ...


class TTSProviderFactory:
    @classmethod
    def create(
        cls, name: str, config: Mapping[str, Any], *,
        registry: TTSProviderRegistry = default_registry,
    ) -> BaseTTSProvider:
        builder = registry.get(name.casefold())
        return builder(config)
```

Registry хранит **builders**, а не только классы: у облачного адаптера и
локальной модели разные constructor dependencies. Factory отвечает за
нормализацию имени, lookup и создание экземпляра; provider-specific builder —
за валидацию своего config и DI. Дубликат canonical-name и неизвестное имя —
fail-fast конфигурационные ошибки со списком доступных имён.

Built-ins регистрируются явно в composition root (`register_builtin_tts()`),
а не через сканирование модулей/entry points: это детерминировано и не требует
plugin framework. Decorator допустим как синтаксический sugar, но модуль всё
равно должен быть импортирован composition root — "магической" регистрации
без импорта нет.

Потребитель вызывает только
`TTSProviderFactory.create(config.provider, config.provider_options)` и затем
работает через `BaseTTSProvider`. В тестах ему можно инъецировать отдельный
registry с `FakeTTSProvider`; глобальный registry не мутируется.

**Паттерны:**

- **Strategy** — конкретный provider выбирается runtime-конфигурацией;
- **Factory** — централизует создание и единые ошибки конфигурации;
- **Registry** — добавляет provider без изменения factory и потребителей;
- **DI** — передаёт registry/builders/transport clients и устраняет hardcoded
  зависимости.

**Почему не service locator в доменной логике:** registry доступен только в
composition root. Передача registry глубоко в `tts_node` скрыла бы зависимости.
**Почему не entry points сейчас:** deployment монорепозитория не требует
динамической установки сторонних plugins. Их можно добавить позже, не меняя
порт или factory API.

### 2.4 Контракт добавления нового провайдера

Чтобы ElevenLabs, Google Cloud TTS или локальная модель появились в factory
без изменений потребителей, реализатор обязан:

1. Реализовать `BaseTTSProvider`: `synthesize`, `stream`, idempotent `aclose`;
   реализовать `list_voices`/`healthcheck`, если capability осмысленна.
2. Преобразовать provider response в `TTSAudio`/`TTSChunk` и ошибки — в
   `TTSError`; не выдавать наружу vendor SDK types.
3. Определить builder, который валидирует только свою config schema и получает
   внешние зависимости через DI.
4. Зарегистрировать уникальное canonical-name через decorator или явный
   `registry.register(name, builder)` в composition root.
5. Пройти общий provider conformance-suite и adapter-specific tests.

Изменения в `tts_node`, CLI и других потребителях после этого не требуются.
Локальная модель отличается только builder'ом (model path/device вместо API
key); контракт и lifecycle те же.

**Что будет, если отложить registry:** при единственном MiniMax это допустимо
и проще. Обязательный триггер миграции — добавление второго runtime-selectable
TTS-провайдера или второго потребителя вне ROS; до этого дизайн остаётся
целевым контрактом, а не оправданием преждевременного plugin framework.

### 2.5 Конфигурация: ROS-параметры + ENV + опциональный YAML

**Решение:** текущая схема (ADR-0003 §2.2) — **ROS-параметры +
ENV-fallback только для секретов** — остаётся primary-каналом.
Дополнительно:

- **Опциональный YAML-файл** `/etc/rob_box/tts.yaml` для развёртываний,
  где ROS-launch длинный (мульти-робот). Схема YAML — зеркало
  ROS-параметров; парсер читает файл **только если он есть** и
  мерджит в `TTSSettings` как lower-priority override относительно
  runtime-параметров.
- **Без pydantic-settings на уровне базового пайплайна.** Добавление
  `pydantic` в `rob_box_llm` потребует пересмотра зависимостей и
  рискует раздуть ABI. Для CLI/cron-инструментов (см. §2.4) допускается
  **отдельный** `pydantic-settings`-модуль (`rob_box_llm.config`)
  **опционально** через `try-import` — но это **не часть базового
  контракта**.

**Приоритет источников (высший → низший):**

```
runtime ROS-параметры → ENV (только секреты) → YAML-файл (если есть) → hardcoded defaults
```

| Поле        | ENV-fallback? | YAML-файл? | ROS-параметр? |
|-------------|---------------|------------|---------------|
| `api_key`   | **да** (`MINIMAX_API_KEY`) | да | да |
| `group_id`  | **да** (`MINIMAX_GROUP_ID`) | да | да |
| `voice`     | нет           | да         | да |
| `model`     | нет           | да         | да |
| `language`  | нет           | да         | да |
| `speed`     | нет (SSML override) | да | да |
| `sample_rate` | нет         | да         | да |
| `format`    | нет           | да         | да |
| `timeout`   | нет           | да         | да |
| `max_retries` / `retry_backoff_ms` | нет | да | да |
| `streaming` | нет           | да         | да |

> **Обоснование "ENV-fallback только для секретов":** ADR-0003 §2.2
> уже отверг ENV-fallback для всех 7 параметров как плодящий два
> источника правды. Этот ADR **подтверждает** решение и расширяет:
> в YAML можно всё (кроме секретов — они остаются ENV-only).

### 2.6 Контракт выходных данных с аудиостэком

**Решение:** **зафиксировать версию контракта на `/voice/audio/speech`**
и **зафиксировать forward-compat hook** для будущего `AudioStamped`.

#### 2.6.1 Текущий контракт (frozen в P0.5)

```
audio_common_msgs/AudioData.data : uint8[]
container = int16 little-endian PCM
channels = 1 (mono)
sample_rate = audio_output_sample_rate (default 16000 Hz)
metadata = out-of-band (audio_output_sample_rate, audio_channels ROS-параметры)
```

Этот контракт остаётся **неизменным** в рамках ADR-0004. Существующие
подписчики (`sound_node`, `mcp_audio_tools`) работают без правок.

#### 2.6.2 Forward-compat: `AudioStamped`-вариант

Открываем **отдельный топик** `/voice/audio/speech_meta` (по умолчанию
выключен), где публикуется `audio_common_msgs/AudioDataStamped`-подобное
сообщение со встроенными `sample_rate`, `channels`, `frame_id`,
`timestamp`. Контракт:

| Поле        | Тип                | Семантика                                     |
|-------------|--------------------|-----------------------------------------------|
| `header`    | `std_msgs/Header`  | DDS-время публикации, `frame_id = "voice_out"` |
| `sample_rate` | `int32`          | Hz, ровно тот, что в samples                  |
| `channels`  | `int8`             | 1=mono, 2=stereo                               |
| `format`    | `string`           | `"pcm_s16le"` | `"pcm_s32le"` | `"mp3"` | `"wav"` |
| `samples`   | `uint8[]`          | PCM bytes (или compressed blob для mp3/wav)    |

> **Важно:** это **не ломает** текущий контракт. Существующие
> подписчики продолжают читать `/voice/audio/speech` (без метаданных),
> новые подписчики (multi-channel mic stack, calibration, VAD) могут
> подписаться на `AudioStamped`-вариант и получить полный контекст.

**Триггер для активации:** включается только когда `tts_node`
запущен с `audio_meta_topic != ""` (ROS-параметр, default пустая
строка → off).

### 2.7 Forward-compat для CLI и cron

**Решение:** допускается использование `TTSProvider` **вне ROS**:

- CLI: `python -m rob_box_llm.tts_cli "text" --provider minimax`
- cron: `python -m rob_box_llm.tts_cli "Reminder text"` через
  cron-job с собственным `RetryPolicy`.

**Обязанности CLI/cron-runner (НЕ провайдера):**

- `aclose()` в `finally`-блоке (для провайдеров с stateful клиентом,
  как `MiniMaxTTSProvider`).
- Retry-цикл (тот же `RetryPolicy`, что и в `tts_node`, см. §2.9).
- Логирование в stdout (без зависимостей от `rclpy`).

> **Граница:** CLI НЕ входит в этот ADR как реализация. Это
> **forward-compat hook** — сейчас `rob_box_llm.tts_cli` не
> существует. Если он появится, он обязан использовать `RetryPolicy`
> из §2.9 и НЕ зависеть от `rclpy`.

### 2.8 Точки расширения: registry и DI

**Решение:** ввести `TTSProviderRegistry` в `rob_box_llm.tts_registry`
**опционально**. Существующий код (`tts_node.py:740-756`) **не
обязан** мигрировать на registry сразу — это forward-compat hook.

```python
# Иллюстративный псевдокод
class TTSProviderRegistry:
    """Реестр TTS-провайдеров по canonical-имени."""

    def __init__(self) -> None:
        self._providers: dict[str, type[TTSProvider]] = {}

    def register(self, name: str, cls: type[TTSProvider]) -> None:
        if name in self._providers:
            raise ValueError(f"provider {name!r} already registered")
        self._providers[name] = cls

    def create(self, name: str, /, **kwargs) -> TTSProvider:
        try:
            cls = self._providers[name]
        except KeyError as e:
            raise KeyError(
                f"unknown TTS provider {name!r}; "
                f"available: {sorted(self._providers)}"
            ) from e
        return cls(**kwargs)

    def names(self) -> list[str]: return sorted(self._providers)


# Built-in registration (lazy import — не тащим всё на старте):
def _register_builtin(reg: TTSProviderRegistry) -> None:
    try:
        from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider
        reg.register("minimax", MiniMaxTTSProvider)
    except ImportError:
        pass
    reg.register("fake", FakeTTSProvider)  # always available
```

**`tts_node.py` миграция (НЕ часть этого ADR — отдельная подзадача):**

```python
# Было:
if self.provider == "minimax":
    self.minimax_provider = MiniMaxTTSProvider(...)

# Стало (после миграции):
from rob_box_llm.tts_registry import default_registry
self._provider_inst = default_registry().create(
    self.provider,
    api_key=self.minimax_api_key,
    group_id=self.minimax_group_id,
    timeout=self.minimax_timeout,
)
```

> **Trade-off:** registry добавляет indirection, но даёт
> * добавлять нового провайдера без правки `tts_node`;
> * uniform error surface (все через `TTSError`);
> * tooling: `default_registry().names()` для CLI help / health-check.

> **Когда мигрировать:** когда появится **второй** opt-in провайдер
> (ElevenLabs, Azure, Yandex v4). Сейчас миграция ради одного
> `MiniMaxTTSProvider` — YAGNI (ADR-0003 §6 уже отложил).

### 2.9 Retry-policy как first-class объект

**Решение:** retry-policy выносится в `RetryPolicy` dataclass и
передаётся в `tts_node` через ROS-параметры (или через YAML/CLI).
Сейчас retry-логика живёт в `_synthesize_minimax_with_retry`
(`tts_node.py:1201-1258`) — ADR-0003 §5.2 её уже зафиксировал.
Этот ADR **формализует** её как reusable value-object, чтобы
новые провайдеры использовали тот же контракт.

```python
# Иллюстративный псевдокод
@dataclass(frozen=True)
class RetryPolicy:
    """Retry policy for transient TTS failures.

    Per ADR-0004 §2.9 / ADR-0003 §5.2:
    - TTSAuthError / TTSBadRequestError → NO retry (raise immediately)
    - TTSRateLimitError (429)            → retry_budget = 1, exp backoff
    - TTSTimeoutError / TTSError (5xx)   → retry_budget = max_retries, exp backoff
    - backoff_ms doubles each attempt:  delay_n = backoff_ms / 1000 * (2 ** n)
    """

    max_retries: int = 2           # 0..3, validation in __post_init__
    backoff_ms: int = 500          # initial backoff, doubles each retry
    retry_on_rate_limit: bool = True

    @classmethod
    def default(cls) -> "RetryPolicy":
        return cls(max_retries=2, backoff_ms=500)

    def backoff_seconds(self, attempt: int) -> float:
        return (self.backoff_ms / 1000.0) * (2 ** attempt)

    def should_retry(self, exc: Exception, attempt: int) -> bool:
        if isinstance(exc, (TTSAuthError, TTSBadRequestError)):
            return False
        budget = 1 if (isinstance(exc, TTSRateLimitError) and self.retry_on_rate_limit) \
                 else self.max_retries
        return attempt < budget
```

**Контракт для реализаторов (forward-compat для CLI/cron):**

> При получении исключения:
> 1. Если `TTSAuthError` или `TTSBadRequestError` → поднять сразу.
> 2. Иначе → если `should_retry(exc, attempt)` True, ждать
>    `backoff_seconds(attempt)` и повторить; иначе — поднять.

**Маппинг на текущий код `tts_node.py:1226-1254`** — точное
соответствие. Никаких изменений в поведении MiniMax-пути.

### 2.10 Circuit breaker (отложенный, но обозначенный)

**Решение:** **не реализуем сейчас**. Причина: retry-policy уже
снижает каскадные сбои, а у нас один робот с десятками TTS-вызовов
в день — circuit breaker тут не даёт измеримой выгоды. Но **форма
хранения состояния** зафиксирована:

```python
@dataclass
class CircuitBreaker:
    """Placeholder for future circuit-breaker state.

    Per ADR-0004 §2.10 — НЕ реализуется в P0.5/P1. Обозначаем форму,
    чтобы будущие провайдеры сразу использовали её.

    States:
      CLOSED    — все вызовы проходят
      OPEN      — все вызовы возвращают TTSError без HTTP
      HALF_OPEN — следующий вызов пробный; success → CLOSED, fail → OPEN
    """

    failure_threshold: int = 5      # открыть после N ошибок подряд
    reset_timeout_s: float = 60.0   # через сколько перейти в HALF_OPEN

    @classmethod
    def disabled(cls) -> "CircuitBreaker":
        """Default — always closed, no-op."""
        # В current P0.5 возвращает self с disabled-флагами.
        ...
```

**Триггер для реализации:** когда TTS-вызовы начнут случаться чаще
100 раз в час на одном роботе (например, длительные dialogues) или
когда появится **несколько потребителей** TTS одновременно
(второй сценарий — multi-robot fleet).

### 2.11 Стриминг: sync (default) + SSE (optional) + WebSocket (reserved)

**Решение:** без изменений по сравнению с ADR-0003 §2.4 —
трехуровневая стратегия:

| Уровень        | Где                                              | Latency-выгода | Стоимость                                 |
|----------------|--------------------------------------------------|----------------|-------------------------------------------|
| Sync `synthesize()` | основной путь, блокирующий            | —              | Baseline                                  |
| SSE `stream()` | HTTP/SSE, `stream=true`; один terminal chunk     | 0% (буферизуется) | +1 retry-логика, +1 QoS-настройка     |
| WebSocket chunk-per-frame | M5/M6, требует persistent connection | ~50-200 ms TTFA | lifecycle пересмотр, persistent pool      |

**Реализационная точка входа** для будущего WebSocket-провайдера:

```python
# В tts_node — отдельный класс MiniMaxStreamingTTSProvider(MiniMaxTTSProvider)
# с переопределённым stream() и persistent `wss://api.minimax.io/v1/t2a_ws_v2`.
# НЕ входит в этот ADR — отдельная задача.
```

> **Совместимость:** текущая `tts_node._synthesize_minimax_streaming_publish`
> (`tts_node.py:1260-1327`) уже реализует SSE-обвязку; ADR-0004
> подтверждает её контракт (см. ADR-0003 §2.4).

---

## 3. Структура артефактов

```
docs/
├── adr/
│   ├── 0002-minimax-provider.md                  (existing — MiniMax в целом)
│   ├── 0003-minimax-tts-architecture.md          (existing — реализационный контракт TTS)
│   └── 0004-minimax-tts-integration-design.md    (ЭТОТ — design-контракт + точки расширения)
├── architecture/
│   ├── minimax-provider.md                       (existing — обзор MiniMax)
│   ├── minimax-tts-architecture.md               (existing — реализационная детализация)
│   └── minimax-tts-integration-design.md         (НОВЫЙ — инженерный справочник по этому ADR)
├── diagrams/
│   ├── minimax-tts-sequence.mmd                  (existing — синхронный happy-path)
│   ├── minimax-tts-integration-sequence.mmd      (НОВЫЙ — расширенный: retry, registry, AudioStamped)
│   └── minimax-tts-integration-class.mmd         (НОВЫЙ — структура классов, точки расширения)
└── analysis/
    └── tts-current-interface.md                  (existing — AS-IS снапшот)
```

**Ответственности:**
- ADR-0004 = **что** и **почему** (этот документ).
- `minimax-tts-integration-design.md` = **как** (инженерный
  справочник с примерами кода и конфигов).
- `minimax-tts-architecture.md` = **что внутри MiniMax-провайдера**
  (маппинг полей, цепочка форматов).
- `tts-current-interface.md` = **что в коде сейчас** (AS-IS,
  pre-изменение).

---

## 4. Альтернативы, которые отклонены

| Альтернатива                                  | Почему отклонена                                                   |
|-----------------------------------------------|--------------------------------------------------------------------|
| Сразу ввести registry как обязательный         | YAGNI для одного opt-in провайдера; ADR-0003 §6 уже отложил       |
| pydantic-settings в базовом пайплайне          | Доп. зависимость в `rob_box_llm`; CLI может использовать отдельно |
| Capability-флаги (`CapabilityUnavailableError`)| ADR-0002 §6 отложил; нет второго потребителя runtime-check         |
| Streaming через WebSocket сразу                 | Persistent connection ломает lifecycle; отложено в M5/M6          |
| Включить circuit breaker сейчас                 | Retry-policy достаточно для текущей нагрузки; обозначаем, не реализуем |
| Полная замена ROS-параметров на YAML            | Ломает текущий deployment; YAML — дополнительный override          |
| AudioStamped в основном топике                  | Ломает существующих подписчиков; вводим отдельный `audio_meta_topic` |
| Retry внутри провайдера (а не снаружи)          | ADR-0003 §4 уже отверг: stateful, мешает unit-тестам               |

---

## 5. Последствия

### Положительные

- **Дизайн-контракт формализован** — следующие TTS-провайдеры
  (ElevenLabs, Azure) получают готовый шаблон (BaseTTSProvider +
  RetryPolicy + Registry) вместо копирования структуры MiniMax.
- **Retry-policy переиспользуется** — CLI и cron-инструменты
  берут тот же `RetryPolicy.default()` вместо своей реализации.
- **AudioStamped как opt-in** — будущие подписчики с метаданными
  (multi-channel mic stack, calibration) получают forward-compat
  путь без поломки текущего контракта.
- **YAML-конфиг как дополнительный канал** — multi-robot
  deployment больше не зависит от длинного ROS-launch.

### Отрицательные / риски

- **Дополнительная документация.** Три ADR по TTS подряд могут
  смутить нового разработчика — поэтому в `minimax-tts-integration-design.md`
  явный "Reading order: ADR-0002 → ADR-0003 → ADR-0004 → architecture".
- **Registry миграция отложена.** Если кто-то начнёт добавлять
  второго opt-in провайдера без миграции — будет дублирование
  `if provider == "X"` в `tts_node`. Mitigation: ADR-0004 явно
  фиксирует "мигрировать при появлении второго".
- **AudioStamped-топик — обязательство.** Если мы его объявим,
  нужно поддерживать обратную совместимость метаданных (топик
  семантически меняться не должен). Для робота с одним
  потребителем это лишняя поверхность; рекомендуем не активировать
  без явного use-case.
- **Circuit breaker обозначен, но не реализован.** Если нагрузка
  вырастет быстрее, чем ожидалось — придётся срочно дореализовывать.
  Mitigation: мониторинг TTS-failure-rate в runbook.

### Нейтральные

- Расширение диаграмм в `docs/diagrams/`: +2 файла (sequence + class).
- `tts_registry.py` пока не существует; в ADR-0004 приведён
  иллюстративный код — реализация отложена до второго провайдера.

---

## 6. Совместимость с ADR-0001, ADR-0002, ADR-0003

| ADR       | Что зафиксировано                              | Совместимость с ADR-0004                      |
|-----------|------------------------------------------------|-----------------------------------------------|
| ADR-0001  | P0 harness: 3-слойная архитектура, ROS-топики frozen | ✅ Не меняет топики `/voice/dialogue/response`, `/voice/audio/speech`; `AudioStamped` — отдельный топик |
| ADR-0002  | MiniMax в целом, opt-in через `provider=minimax`, no fallback | ✅ Подтверждаем opt-in; расширяем на будущих провайдеров через registry |
| ADR-0003  | Реализационный контракт TTS: маппинг, форматы, retry | ✅ Не противоречит; `RetryPolicy` формализует таблицу retry из ADR-0003 §5.2 |

---

## 7. Открытые вопросы / будущие ADR

| Вопрос                                          | Где решать                                 |
|-------------------------------------------------|--------------------------------------------|
| Реальная реализация `BaseTTSProvider` и Registry | Когда появится второй opt-in TTS-провайдер |
| WebSocket chunk-per-frame streaming               | M5/M6, отдельный ADR                        |
| pydantic-settings в `rob_box_llm` CLI            | Если CLI станет production-инструментом     |
| Реальная реализация `CircuitBreaker`              | Если TPS превысит 100/час/робот            |
| AudioStamped-контракт для multi-channel mic stack | Когда появится ReSpeaker 4-mic / другое     |
| Capability-флаги для TTS-провайдеров              | Если появятся runtime-ветвления по capabilities (например, "voice clone only") |
| YAML-конфиг как first-class                      | Если multi-robot deployment станет стандартом |

---

## 8. Acceptance

ADR считается зафиксированным (статус переводится в **Accepted**),
когда:

1. ✅ Документ одобрен архитектурным ревью.
2. ⏳ `t_25b8e221` (потомок) реализует **минимум** §2.5 (Registry)
   и §2.9 (`RetryPolicy` value-object), даже если `BaseTTSProvider`
   ещё не используется.
3. ⏳ `/voice/audio/speech_meta` топик **не активирован** в production
   без явного use-case (сохраняем обратную совместимость).
4. ⏳ Существующий `MiniMaxTTSProvider` остаётся без изменений —
   подтверждается регрессионными тестами (≥ 95% pass rate).

До выполнения пунктов 2-4 статус остаётся **Proposed**.

---

## Приложение A. Ключевые решения и trade-off

| Решение                                | Стоимость                                   | Латентность                  | Расширяемость                | Надёжность                                |
|----------------------------------------|---------------------------------------------|------------------------------|------------------------------|--------------------------------------------|
| BaseTTSProvider как forward-compat (§2.1) | +1 абстрактный класс, +0 строк кода сейчас | Без изменений                | ✅ База для всех будущих     | ✅ Idempotent aclose + retry-hook           |
| YAML-конфиг опционально (§2.2)         | +1 парсер (если используется)                | Без изменений                | ✅ Multi-robot deployment    | ✅ Только секреты в ENV (не дублируется)   |
| AudioStamped как opt-in топик (§2.6)   | +1 ROS-publisher (если активирован)          | Без изменений                | ✅ Multi-channel mic stack   | ⚠️ Не активировать без use-case             |
| TTSProviderRegistry (§2.5)              | +1 indirection (после миграции)              | Без изменений                | ✅ Add provider без правки tts_node | ✅ Single source of error surface      |
| RetryPolicy value-object (§2.9)        | +1 dataclass, рефактор tts_node              | Без изменений (та же политика) | ✅ CLI/cron переиспользуют  | ✅ Зафиксированный контракт retry          |
| Circuit breaker обозначен, не реализован (§2.10) | +0 строк сейчас, +datalass позже      | Без изменений сейчас         | ✅ Shape зафиксирована       | ⚠️ До ~100/час retry-policy хватает       |
| Streaming: sync+SSE, WebSocket reserved (§2.11) | +0 сейчас                              | Sync: ~2-3 s TTFA, SSE: ~2 s TTFA, WS reserved | ⚠️ WS ждёт M5/M6      | ✅ Buffering SSE не врёт, контракт frozen   |