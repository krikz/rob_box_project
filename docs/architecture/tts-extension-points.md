# TTS Provider — Структура классов и точки расширения

| Поле          | Значение                                                                  |
|---------------|---------------------------------------------------------------------------|
| Каталог       | `docs/architecture/`                                                      |
| Статус        | **Accepted** — landed in t_8cbf9995 (see [ADR-0008](../adr/0008-tts-provider-extension-points-landed.md)) |
| Дата          | 2026-07-21                                                                |
| Автор         | architect (Hermes Agent)                                                  |
| Контекст      | Kanban task `t_8d714ff0` (спроектировать структуру класса + точки расширения TTS) |
| Родительские ADR | [ADR-0004 §2.1–2.4](../adr/0004-minimax-tts-integration-design.md) (port, registry, retry, CB), [ADR-0007](../adr/0007-minimax-tts-integration-final.md) §2.2 (итоговая архитектура), [ADR-0003](../adr/0003-minimax-tts-architecture.md) (текущая реализация MiniMax) |
| AS-IS         | [docs/analysis/tts-current-interface.md](../analysis/tts-current-interface.md) |
| Диаграммы     | [docs/diagrams/tts-extension-class.mmd](../diagrams/tts-extension-class.mmd), [docs/diagrams/tts-extension-sequence.mmd](../diagrams/tts-extension-sequence.mmd) |
| Код-стабы     | [`docs/architecture/stubs/tts_provider_base.py`](#stubs) (сигнатуры методов без реализации) |

---

## 0. Что это за документ и чем он НЕ является

**Это — архитектурный design-only документ.** Здесь:

- ✅ Описывается **форма** новой абстракции (`BaseTTSProvider`) и **сигнатуры** методов 3-х будущих провайдеров как **заготовки без реализации** (`...` / `pass`).
- ✅ Фиксируется **реестр / фабрика** как composition-root mechanism (только сигнатуры).
- ✅ Обосновываются **5 точек расширения**, через которые новый TTS-провайдер подключается к `tts_node`.
- ✅ Приводится Mermaid class-диаграмма и sequence-диаграмма фабрики.
- ❌ **НЕ пишется production-код**, который реализует HTTP/SSE/WS-обвязку (это дочерняя задача `t_25b8e221` и `t_b4cb8948`).
- ❌ **НЕ модифицируется** существующий `MiniMaxTTSProvider` (`src/rob_box_llm/rob_box_llm/providers/minimax_tts.py`) и текущая фабрика в `tts_node.py:_synthesize_and_play`.

> Совместимость с существующим кодом из PR #907 — `TTSProvider` ABC (frozen в `tts.py:121-177`) **не трогаем**. Все новые абстракции — **поверх** существующего контракта. Точки расширения выбираются так, чтобы `MiniMaxTTSProvider(TTSProvider)` мог быть **позже** переведён на `MiniMaxTTSProvider(BaseTTSProvider)` без изменения сигнатуры публичных методов.

---

## 1. Проблема и зачем нужны точки расширения

В [docs/analysis/tts-current-interface.md](../analysis/tts-current-interface.md) зафиксировано:

1. В `rob_box` уже есть Yandex/Silero/MiniMax, но **фабрики / реестра TTS-провайдеров нет** — `tts_node._synthesize_and_play` хардкодит `MiniMaxTTSProvider` (см. `tts_node.py:59, 1148-1155, 1377`).
2. Подключение 4-го провайдера (ElevenLabs/Google/Azure/local-Piper) сейчас требует правки в **трёх местах** (`__init__.py`, `_synthesize_and_play`, конфиг), что ADR-0001 уже назвал top-priority архитектурным риском.
3. `TTSSettings` уже включает поля `voice / model / language / sample_rate / format / extra`, и все 3 текущих провайдера умеют в них мапиться, **но единой схемы capabilities нет** — каждый провайдер тихо игнорирует то, что не поддерживает.

**Что хотим получить** после введения точек расширения:

- Добавление нового TTS-провайдера = **одна правка** (зарегистрировать builder в composition root) + сам файл провайдера.
- `tts_node` и CLI получают **одну и ту же фабрику** (никто не дублирует `if provider == "minimax": ...`).
- Capability-флаги (streaming / voice-cloning / SSML / pricing) **явно объявлены** в одном месте; `tts_node` может проверить `provider.supports("streaming")` до вызова.
- Voice-каталог **нормализован** через общий value-object `TTSVoice` (а не как сейчас — MiniMax отдаёт `"Calm_Woman"`, Yandex — `"alena"`, Silero — локальный путь к ONNX).

---

## 2. Решение: 5 точек расширения

Вводим **одну новую абстракцию** `BaseTTSProvider(TTSProvider)` с **5 точками расширения**, через которые адаптер заявляет о своих особенностях. Все 5 — **override-friendly** методы с default-реализацией, поэтому провайдер, который не умеет SSML или voice-cloning, может **не переопределять** соответствующий метод (получит честный `False` / `NotImplementedError`).

| #  | Точка расширения             | Что объявляет провайдер                                | Default                  | Какой провайдер заполняет полезно                            |
|----|------------------------------|--------------------------------------------------------|--------------------------|--------------------------------------------------------------|
| 1  | `capabilities()`             | Статическое объявление `TTSCapabilities`               | пустой флаг-сет          | все; MiniMax заявляет streaming=False, voice-clone=True       |
| 2  | `list_voices()`              | Получение каталога голосов через API провайдера        | raise `NotImplementedError` | все; local-Piper возвращает 1-3 голоса из локальных файлов |
| 3  | `healthcheck()`              | Удобный pre-flight перед `synthesize` (для retry-loop)  | возвращает `TTSHealth(ok=True)` | cloud-провайдеры (HTTP `/healthz` или аналог)             |
| 4  | `_build_request_payload(...)` | Чисто-функциональный маппинг `TTSSettings → dict` для HTTP body | **обязателен** к override | MiniMax, ElevenLabs, Google — каждый по своей схеме |
| 5  | `_http_client_factory(...)`  | Ленивая инициализация `httpx.AsyncClient` с нужными TLS / proxy / headers настройками | default `httpx.AsyncClient()` | Google (требует service-account JWT), ElevenLabs (proxy headers) |

Точки 4 и 5 — **обязательные** для override (это и есть полезная нагрузка адаптера); 1, 2, 3 — **опциональные** (default-реализация честно сообщает «не умею»). Это даёт KISS: «добавить нового провайдера» = реализовать минимум 2 метода + зарегистрировать builder.

### 2.1 Почему именно так, а не «общая реализация всех»

Альтернатива — `UniversalTTSProvider` с фиксированной картой параметров, которую каждый провайдер мерджит со своими defaults. **Отклонено**: маппинг слишком специфичен (MiniMax `voice_setting.voice_id`, ElevenLabs `voice_settings.voice_id`, Google `voice.name` + `audioConfig.audioEncoding`), попытки унифицировать порождают «leaky abstraction». ADR-0001 уже зафиксировал «tiny ABC over factory» — здесь тот же подход: ABC объявляет **что** адаптер должен уметь, но не **как** он это делает.

### 2.2 Почему default-реализации не «pass / NotImplementedError» везде

Для **опциональных** точек расширения (1–3) делаем **честный default**: `list_voices()` по умолчанию возвращает пустой список + логирует warning. Это позволяет `FakeTTSProvider` (уже есть в `tts.py:185-245`) **ничего не переопределять**, кроме обязательных `synthesize/stream/aclose` — backward-compat с тестами в PR #907. Production-провайдер, который не переопределил `healthcheck()`, **не падает** в retry-loop — получает `ok=True` и движется дальше; если ему это не подходит, он переопределяет.

---

## 3. Карта классов (где что лежит в коде)

| Слой                                  | Файл / модуль                                                       | Что в нём                                                                                   |
|---------------------------------------|---------------------------------------------------------------------|---------------------------------------------------------------------------------------------|
| Доменные value-объекты (frozen P0.5) | `src/rob_box_llm/rob_box_llm/tts.py`                                | `TTSProvider` ABC, `TTSSettings`, `TTSAudio`, `TTSChunk`, `TTSFormat`, `FakeTTSProvider`    |
| Иерархия ошибок (frozen P0.5)         | `src/rob_box_llm/rob_box_llm/errors.py`                             | `TTSError` + `TTSAuthError`, `TTSBadRequestError`, `TTSRateLimitError`, `TTSTimeoutError`     |
| **Новое** — порты и capability-мета  | `docs/architecture/stubs/tts_provider_base.py` (см. §stubs)    | `BaseTTSProvider(TTSProvider)`, `TTSCapabilities`, `TTSVoice`, `TTSHealth`, `ProviderBuilder` |
| **Новое** — реестр и фабрика         | `docs/architecture/stubs/tts_provider_registry.py` (см. §stubs) | `TTSProviderRegistry`, `TTSProviderFactory.create()`, `register_builtin_tts_providers()`    |
| MiniMax-адаптер (текущий, не трогаем)| `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py`              | `MiniMaxTTSProvider(TTSProvider)` — в P0.5 не мигрирует                                       |
| Composition root (точка подключения) | `src/rob_box_voice/rob_box_voice/tts_node.py:_synthesize_and_play`  | `tts_node` сейчас хардкодит `MiniMaxTTSProvider`; **в P0.6** заменяется на `TTSProviderFactory.create(provider_name, config, registry)` |
| CLI composition root (будущее)        | `src/rob_box_llm/rob_box_llm/tts_cli.py`                            | ещё не существует; ADR-0006 зафиксировал `MiniMaxTTSConfig` (pydantic-settings) для него     |

---

## 4. Реестр / фабрика — зачем и как

`TTSProviderRegistry` хранит `dict[str, ProviderBuilder]` — отображение **имя → фабричная функция**. `ProviderBuilder` принимает конфиг и возвращает готовый `BaseTTSProvider`. Это позволяет:

- Регистрировать **3-rd party провайдеры** из внешних пакетов без правки `rob_box_llm` (см. §5.3).
- Тестировать `tts_node` с **mock-провайдером**, зарегистрированным под именем `"mock"` в conftest.
- Переключать провайдера **через ROS-параметр** `tts_node.tts.provider_name = "minimax" | "elevenlabs" | ...` без правки кода.

`TTSProviderFactory.create(name, config, registry)` — единая точка входа для **обоих** consumer'ов (`tts_node` и будущего CLI). Идемпотентна: если провайдер с тем же именем уже создан в этом процессе — возвращает кэшированный instance. **Не** auto-discovery (`importlib.metadata.entry_points()`) — это запрещено ADR-0004 §2.3 (неявные side-effects, плохо тестируется). Только явный `register_builtin_tts_providers()` в composition root.

> **Совместимость с PR #907:** на P0.5 существующий код продолжает работать без изменений — `tts_node` всё ещё может импортировать `MiniMaxTTSProvider` напрямую (см. `tts_node.py:59-70`, опциональный импорт через `MINIMAX_AVAILABLE`). Миграция на фабрику — **отложена до момента, когда появится 2-й opt-in провайдер** (ADR-0004 §2.8). То есть **никаких breaking changes** в этом PR.

---

## 5. Точки расширения — детальный разбор

### 5.1 Capability-метаданные (`capabilities()`)

`TTSCapabilities` — `frozen @dataclass` с **8 boolean-полями**: streaming, voice_cloning, ssml, pronunciation_dict, audio_format_pcm, audio_format_mp3, audio_format_ogg, custom_endpoint. Default = все `False`. Провайдер переопределяет **только те, что реально поддерживает**.

Зачем: `tts_node` (и CLI) могут до вызова проверить `if not provider.capabilities().streaming: use sync path`. Это убирает неявный fallback `mini_max_provider.stream → synthesize` (см. ADR-0003 §2.4) и даёт **явный failure** вместо silent-degradation.

**Future-провайдеры, которые выиграют:**

- **ElevenLabs** — `streaming=True`, `voice_cloning=True`, `ssml=False` (только plain text), `audio_format_mp3=True`.
- **Google Cloud TTS** — `streaming=True`, `voice_cloning=False`, `ssml=True`, `audio_format_pcm=True`, `audio_format_ogg=True`, `audio_format_mp3=True`.
- **Local Piper** — всё `False`, зато `custom_endpoint=True` (локальный subprocess).

### 5.2 Voice-каталог (`list_voices()`)

Единая нормализация: `TTSVoice(id, name, language, gender, preview_url=None, supports_cloning=False)`. Default-реализация возвращает `[]` (без HTTP). Провайдер, умеющий `/v1/voices` (MiniMax это имеет, см. [docs/research/minimax-tts-api.md](../research/minimax-tts-api.md) §13) — переопределяет и кэширует на 1 час.

**Future-провайдеры:**

- MiniMax — 40+ системных + клоны (см. ADR-0003 §7).
- ElevenLabs — `voices` endpoint, ~30 community + custom.
- Google — `voices.list` (защищён OAuth, см. §5.5).
- Local Piper — голоса = локальные `.onnx`-файлы в `/usr/share/piper/voices/`, не сетевые.

### 5.3 Health-check (`healthcheck()`)

Ленивая проверка доступности upstream'а перед `synthesize`. Default-реализация — `TTSHealth(ok=True, latency_ms=0.0, provider=self.name)` (no-op). Production-провайдеры, которые знают про rate-limit (RPM=60 у MiniMax), могут переопределить и вернуть `TTSHealth(ok=False, reason="rate_limited")`, чтобы `tts_node` мог отдать `STOP` пользователю вместо бессмысленного retry.

Возвращается **frozen** `TTSHealth` value-object. **Логирование секретов запрещено** (только `provider / ok / latency_ms / reason`).

**Future-провайдеры:**

- MiniMax — отдельного `/healthz` нет; cheap-проверка = list-voices с TTL cache (см. §5.2).
- ElevenLabs — `GET /v1/user` (требует API key, но не делает TTS).
- Google — OAuth-токен + 1 cheap read.
- Local Piper — `subprocess.run([piper, "--version"])` (offline health).

### 5.4 Payload-маппинг (`_build_request_payload()`)

**Обязательный** к override. Чистая функция `(text, settings, voice_meta) -> dict[str, Any]`, возвращающая JSON-ready body для HTTP-запроса. **Никаких side-effects** (не делает HTTP, не логирует секреты). Логирование — на стороне `_post()` (HTTP-обвязка), не здесь.

Зачем **отдельный** метод, а не просто переопределение `synthesize`: чтобы **тестировать маппинг без HTTP** — `unittest.mock` против `_build_request_payload` вместо `httpx.MockTransport`. Это в 10× быстрее и не зависит от сетевого стека.

**Пример маппинга** (для понимания, не реализация):

```
MiniMax T2A v2:    {"model": ..., "text": ..., "voice_setting": {...}, "audio_setting": {...}, "stream": False}
ElevenLabs:        {"text": ..., "voice_settings": {...}, "model_id": ..., "output_format": "mp3_44100_128"}
Google TTS:        {"input": {"ssml": ...}, "voice": {"name": ..., "languageCode": ...}, "audioConfig": {...}}
Local Piper CLI:   {"text_file": ..., "voice_model": ..., "output_file": ..., "config": {...}}
```

### 5.5 HTTP-клиент фабрика (`_http_client_factory()`)

**Обязательный** к override, если провайдер требует нестандартный transport. Default = `httpx.AsyncClient(timeout=self._timeout)`. Переопределение нужно для:

- **Google** — OAuth-токен в `Authorization` header, refresh в background-задаче (см. `google-auth` lib).
- **ElevenLabs** — кастомные `User-Agent` / `xi-api-key` headers, опциональный proxy.
- **Local Piper** — НЕ HTTP, а `subprocess.Popen(...)` (override `_http_client_factory` для подмены типа возврата через `Any`, или ввести отдельный hook `_transport_factory()` в будущем — пока оставляем HTTP-only).

Зачем: TLS/timeout/proxy настройки **разные** для разных провайдеров, и они не должны «утекать» в общую конфигурацию.

---

## 6. Совместимость с PR #907 — что не сломается

| Что в PR #907                                                            | Что меняется                                                                |
|--------------------------------------------------------------------------|-----------------------------------------------------------------------------|
| `TTSProvider` ABC в `tts.py:121-177`                                     | **Ничего.** Это frozen.                                                      |
| `MiniMaxTTSProvider(TTSProvider)` в `providers/minimax_tts.py:299`        | На P0.5 — **ничего**. На P0.6 (после добавления 2-го провайдера) — `class MiniMaxTTSProvider(BaseTTSProvider)` через 1 строку. |
| `FakeTTSProvider(TTSProvider)` в `tts.py:185`                             | **Ничего.** Не обязан переопределять опциональные точки расширения.         |
| `tts_node._synthesize_and_play` (хардкод `MiniMaxTTSProvider`, `tts_node.py:59,1148`) | На P0.5 — **ничего**. На P0.6 — заменяется на `TTSProviderFactory.create(provider_name, config, registry)`. В этом PR **не трогаем**. |
| Тесты `test_tts_value_objects.py`, `test_tts_conformance.py`, `test_minimax_tts*` | **Ничего.** Все они работают с `TTSProvider` (родитель), а не с `BaseTTSProvider` (потомок). |
| ENV-переменные `MINIMAX_API_KEY`, `MINIMAX_GROUP_ID`                      | **Ничего.** Прочитаются в `MiniMaxTTSProvider.__init__` как и сейчас.       |
| ROS-параметры `tts_node.tts.minimax_*`                                    | **Ничего.** Они живут в `tts_node._synthesize_and_play`, а не в фабрике.    |

**Итог**: 0 строк существующего кода меняется в этом PR. **0 breaking changes.** Новые абстракции — чисто additive.

---

## 7. Trade-off

| Решение                                                              | Стоимость                                                            | Выгода                                                                                     |
|----------------------------------------------------------------------|----------------------------------------------------------------------|--------------------------------------------------------------------------------------------|
| `BaseTTSProvider(TTSProvider)` — новый слой                          | +1 файл, ~80 строк ABC-кода                                          | Единая точка для capability-мета и voice-каталога; совместим с PR #907                       |
| `capabilities()` как метод (не property / не class-attr)             | На 1 вызов больше в hot path; можно кэшировать на стороне caller'а  | Поддерживает runtime-queries (`provider.capabilities().streaming`) без singleton-ловушек     |
| Default-реализации для опциональных точек расширения                 | Чуть больше boilerplate в ABC; легко забыть override                | `FakeTTSProvider` остаётся 30 строками; PR #907 тесты не трогаются                          |
| `_build_request_payload()` отдельно от `synthesize()`                 | +1 метод; провайдер пишет 2 функции вместо 1                         | Unit-тесты маппинга без HTTP — в 10× быстрее; чёткое разделение pure/impure                  |
| `TTSProviderRegistry` без auto-discovery                             | 0 cost (явный `register_builtin_*` в composition root)               | Предсказуемые side-effects, нет скрытых зависимостей при import (ADR-0004 §2.3)             |
| `MiniMaxTTSProvider` мигрирует на `BaseTTSProvider` **только** при появлении 2-го провайдера | 0 сейчас                                                              | YAGNI — не платим за абстракцию, пока она не нужна                                          |

Альтернативы, которые **отклонены**:

| Альтернатива                                              | Почему отклонена                                                                              |
|-----------------------------------------------------------|-----------------------------------------------------------------------------------------------|
| `UniversalTTSProvider` с общей схемой параметров           | Leaky abstraction: маппинг слишком специфичен (MiniMax vs ElevenLabs vs Google — разные формы)|
| `entry_points()` auto-discovery (как в `pluggy`)            | Неявные side-effects при import; ADR-0004 §2.3 явно отвергает                                  |
| `enum Capability` с бит-флагами                           | Type-unsafe (`capabilities() & Capability.STREAMING` легко багуется); frozen dataclass безопаснее |
| `voice_settings` как глобальный dict                       | Скрытый контракт; unit-тесты без явной схемы — хрупко                                         |
| Один большой `__init__(**kwargs)`                          | Невозможно type-check; IDE не поможет; «спрятанный» контракт                                  |

---

## 8. Acceptance criteria задачи t_8d714ff0

| # | Критерий                                                                                       | Где закрыт                                                                              |
|---|------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------|
| 1 | Mermaid class-диаграмма рендерится без ошибок                                                  | [docs/diagrams/tts-extension-class.mmd](../diagrams/tts-extension-class.mmd) (см. §9.1)  |
| 2 | Mermaid sequence-диаграмма фабрики (при необходимости)                                         | [docs/diagrams/tts-extension-sequence.mmd](../diagrams/tts-extension-sequence.mmd)        |
| 3 | Для каждой точки расширения описано, какие провайдеры её используют                            | §5.1–5.5 (по каждой точке — примеры будущих провайдеров)                                  |
| 4 | Код-стабы сигнатур методов на Python (без реализации)                                          | [§Stubs ниже](#stubs) — `tts_provider_base.py` + `tts_provider_registry.py`              |
| 5 | Совместимость с PR #907 (минимум breaking changes)                                              | §6 — таблица «что в PR #907 / что меняется»                                               |

---

## 9. Артефакты (что-где лежит)

| Артефакт                                                                                          | Назначение                                                              |
|---------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------|
| [docs/diagrams/tts-extension-class.mmd](../diagrams/tts-extension-class.mmd)                       | Class-диаграмма: TTSProvider → BaseTTSProvider → MiniMax / ElevenLabs / Google / LocalPiper + capabilities + registry |
| [docs/diagrams/tts-extension-sequence.mmd](../diagrams/tts-extension-sequence.mmd)                 | Sequence: tts_node → factory.create() → registry.resolve() → provider    |
| [docs/architecture/tts-extension-points.md](../architecture/tts-extension-points.md)               | Этот документ                                                           |
| `docs/architecture/stubs/tts_provider_base.py`                                                | Сигнатуры `BaseTTSProvider`, `TTSCapabilities`, `TTSVoice`, `TTSHealth`, `ProviderBuilder` (design-only, no logic) |
| `docs/architecture/stubs/tts_provider_registry.py`                                             | Сигнатуры `TTSProviderRegistry`, `TTSProviderFactory.create()`, `register_builtin_tts_providers()` (design-only)  |
| `docs/architecture/stubs/elevenlabs_tts.py`                                         | Stub `ElevenLabsTTSProvider(BaseTTSProvider)` (только сигнатуры + docstring) |
| `docs/architecture/stubs/google_tts.py`                                              | Stub `GoogleTTSProvider(BaseTTSProvider)` (только сигнатуры)              |
| `docs/architecture/stubs/local_piper_tts.py`                                         | Stub `LocalPiperTTSProvider(BaseTTSProvider)` (только сигнатуры)          |

---

## Stubs

> Файлы лежат в `docs/architecture/stubs/` (design-only, **НЕ импортируются** production-кодом до завершения `t_25b8e221` и перевода ADR-0007 в `Accepted`). Содержат только docstring + сигнатуры + `pass` / `...`.

### `tts_provider_base.py`

```python
"""TTS Provider extension ports and capability metadata.

**DESIGN ONLY — этот файл не импортируется production-кодом до завершения
t_25b8e221 и перевода ADR-0007 в Accepted.**

Extension surface for future TTS providers (ElevenLabs, Google, Azure,
local Piper). Mirrors the shape of :mod:`rob_box_llm.provider` (LLM):
a small async-only ABC with frozen dataclasses for inputs and outputs,
plus capability metadata + voice catalogue + health-check as overridable
hooks.

Public contract (frozen):
    * ``BaseTTSProvider``           — abstract port
    * ``TTSCapabilities``           — frozen dataclass of 8 boolean flags
    * ``TTSVoice``                  — normalized voice catalogue entry
    * ``TTSHealth``                 — frozen pre-flight health snapshot
    * ``ProviderBuilder``           — factory callback type

Subclassing rules:
    * Override ``_build_request_payload`` (mandatory) — pure mapping
      ``(text, settings, voice_meta) -> dict``.
    * Override ``_http_client_factory`` (mandatory unless default httpx works)
      to customise transport / TLS / proxy / OAuth.
    * Override ``capabilities``, ``list_voices``, ``healthcheck`` only if the
      provider supports them; default impls are honest no-ops.

Backward-compat with PR #907:
    * ``BaseTTSProvider`` IS-A ``TTSProvider`` — every existing call site
      that type-annotates ``TTSProvider`` keeps working unchanged.
    * ``MiniMaxTTSProvider(TTSProvider)`` stays untouched until the second
      opt-in provider lands; migration is a single-line ``class … (BaseTTSProvider)``.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable, Mapping

import httpx

from rob_box_llm.tts import TTSProvider, TTSSettings


# ---------------------------------------------------------------------------
# Capability metadata — frozen dataclass, 8 boolean flags
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TTSCapabilities:
    """Static capability declaration for a TTS provider.

    Providers override :meth:`BaseTTSProvider.capabilities` to return an
    instance with only the flags they actually support; every flag is
    ``False`` by default, so an unconfigured provider is always
    capability-honest (no silent degradation).
    """

    streaming: bool = False
    voice_cloning: bool = False
    ssml: bool = False
    pronunciation_dict: bool = False
    audio_format_pcm: bool = False
    audio_format_mp3: bool = False
    audio_format_ogg: bool = False
    custom_endpoint: bool = False  # for local / on-prem providers


# ---------------------------------------------------------------------------
# Normalized voice catalogue entry
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TTSVoice:
    """Provider-neutral voice description.

    Different providers expose wildly different voice metadata (MiniMax
    uses ``"Calm_Woman"``, ElevenLabs uses UUIDs, Google uses
    ``"en-US-Wavenet-A"``, local Piper uses ``.onnx`` filenames).
    This dataclass is the smallest common subset the rest of rob_box
    can rely on. Provider-specific extras go in ``extra``.
    """

    id: str
    name: str
    language: str  # BCP-47, e.g. "ru", "en-US"
    gender: str = "unknown"  # "male" | "female" | "neutral" | "unknown"
    preview_url: str | None = None
    supports_cloning: bool = False
    extra: Mapping[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Pre-flight health snapshot
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TTSHealth:
    """Snapshot returned by :meth:`BaseTTSProvider.healthcheck`.

    Default impl returns ``(ok=True, latency_ms=0.0)`` — no upstream call.
    Production providers override this to do a cheap pre-flight (e.g.
    list-voices with TTL cache, OAuth-token check, subprocess ``--version``).
    """

    ok: bool
    provider: str
    latency_ms: float = 0.0
    reason: str | None = None  # short human-readable string when ok=False


# ---------------------------------------------------------------------------
# Factory callback type — registry input
# ---------------------------------------------------------------------------


# ProviderBuilder = Callable[[Mapping[str, Any]], "BaseTTSProvider"]
# (signature only; runtime construction left to t_25b8e221)
ProviderBuilder = Callable[[Mapping[str, Any]], "BaseTTSProvider"]
"""Factory callback used by :class:`TTSProviderRegistry`.

Takes a config dict (already validated by pydantic-settings in CLI path,
or ROS-params dict in ROS path) and returns a fully constructed
:class:`BaseTTSProvider`. Builders SHOULD be idempotent — calling twice
with the same config returns equivalent instances.
"""


# ---------------------------------------------------------------------------
# Extension port
# ---------------------------------------------------------------------------


class BaseTTSProvider(TTSProvider):
    """Extension port for future TTS providers.

    Inherits all behaviour from :class:`rob_box_llm.tts.TTSProvider`
    (synthesize/stream/aclose contract — frozen at P0.5, see tts.py:121-177)
    and adds 5 override hooks (capabilities, list_voices, healthcheck,
    _build_request_payload, _http_client_factory).

    Two of the five hooks are mandatory to override:

    * ``_build_request_payload`` — pure mapping, never touches HTTP.
    * ``_http_client_factory``   — unless default httpx works for you.

    The other three have honest default implementations:

    * ``capabilities()`` — empty flag-set.
    * ``list_voices()``  — empty list (no upstream call).
    * ``healthcheck()``  — ``TTSHealth(ok=True, provider=self.name)``.

    .. note::

        Existing ``MiniMaxTTSProvider(TTSProvider)`` does NOT migrate to
        this base in this PR — that change is deferred until the second
        opt-in provider lands (ADR-0004 §2.8). Migrating earlier is YAGNI.

    Subclassing example::

        class ElevenLabsTTSProvider(BaseTTSProvider):
            name = "elevenlabs"

            async def _http_client_factory(self) -> httpx.AsyncClient:
                return httpx.AsyncClient(
                    base_url="https://api.elevenlabs.io",
                    headers={"xi-api-key": self._api_key},
                    timeout=httpx.Timeout(self._timeout),
                )

            def _build_request_payload(
                self,
                text: str,
                settings: TTSSettings,
                voice_meta: TTSVoice | None,
            ) -> dict[str, Any]:
                return {
                    "text": text,
                    "voice_settings": {"stability": 0.5, "similarity_boost": 0.75},
                    "model_id": settings.model or "eleven_monolingual_v1",
                    "output_format": "mp3_44100_128",
                }
    """

    name: str = "abstract-base"

    # ---- optional extension points (default = honest no-op) ----

    def capabilities(self) -> TTSCapabilities:
        """Static capability declaration. Override if your provider supports
        streaming / voice-cloning / SSML / etc. Default: all flags False."""
        return TTSCapabilities()

    async def list_voices(self) -> list[TTSVoice]:
        """Return normalized voice catalogue. Default: empty list."""
        return []

    async def healthcheck(self) -> TTSHealth:
        """Pre-flight health check. Default: always ok."""
        return TTSHealth(ok=True, provider=self.name)

    # ---- mandatory extension points (override required) ----

    @abc.abstractmethod
    def _build_request_payload(
        self,
        text: str,
        settings: TTSSettings,
        voice_meta: TTSVoice | None,
    ) -> dict[str, Any]:
        """Pure mapping ``TTSSettings → provider-specific JSON body``.

        MUST be a pure function: no HTTP calls, no logging of secrets, no
        global state mutation. Tested in isolation via ``unittest.mock``,
        no transport mocking needed.
        """

    def _http_client_factory(self) -> httpx.AsyncClient:
        """Construct the HTTP client this provider uses.

        Default = ``httpx.AsyncClient(timeout=self._timeout)``. Override
        for OAuth (Google), custom headers (ElevenLabs), proxy / TLS, etc.
        The provider DOES NOT own the returned client — caller (or
        ``aclose()``) handles teardown.
        """
        timeout: float = getattr(self, "_timeout", 30.0)
        return httpx.AsyncClient(timeout=timeout)


__all__ = [
    "BaseTTSProvider",
    "TTSCapabilities",
    "TTSVoice",
    "TTSHealth",
    "ProviderBuilder",
]
```

### `tts_provider_registry.py`

```python
"""TTS Provider registry + factory — composition-root mechanism.

**DESIGN ONLY — этот файл не импортируется production-кодом до завершения
t_25b8e221 и перевода ADR-0007 в Accepted.**

Why a separate registry module:
    * Single source of truth for ``provider_name → builder`` mapping.
    * Lets 3rd-party packages register their own providers without
      touching ``rob_box_llm`` (callers add to a passed-in registry).
    * Gives tests a way to inject mock providers under any name.

Composition-root contract:
    * ``register_builtin_tts_providers()`` is the ONLY place built-in
      providers get registered. Called once at process start.
    * No auto-discovery via ``importlib.metadata.entry_points()`` —
      ADR-0004 §2.3 explicitly rejects this.
    * ``TTSProviderFactory.create(name, config, registry)`` is the only
      entry point for both ROS path (tts_node) and CLI path (future).
"""

from __future__ import annotations

from typing import Any, Mapping

from rob_box_llm.tts_provider_base import BaseTTSProvider, ProviderBuilder


class TTSProviderRegistry:
    """In-memory ``name → builder`` registry.

    NOT thread-safe for concurrent registration. Construct once at
    process start, then read-only for the rest of the lifetime.
    """

    def __init__(self) -> None:
        self._builders: dict[str, ProviderBuilder] = {}

    def register(self, name: str, builder: ProviderBuilder) -> None:
        """Register a builder under ``name``. Raises ``ValueError`` if
        the name is already taken (no silent override — refactor instead)."""

    def resolve(self, name: str) -> ProviderBuilder:
        """Return the builder for ``name``. Raises ``KeyError`` if missing."""

    def names(self) -> list[str]:
        """Return all registered provider names (sorted)."""


class TTSProviderFactory:
    """Single entry point for constructing a TTS provider.

    Both the ROS path (``tts_node._synthesize_and_play``) and the future
    CLI path (``rob_box_llm.tts_cli``) call this method. Caches built
    instances per ``(name, config_hash)`` so accidental double-construction
    in long-running processes doesn't open two HTTP pools.
    """

    _cache: dict[tuple[str, str], BaseTTSProvider]

    @classmethod
    def create(
        cls,
        name: str,
        config: Mapping[str, Any],
        registry: TTSProviderRegistry,
    ) -> BaseTTSProvider:
        """Resolve ``name`` in ``registry``, invoke builder with ``config``,
        cache and return the result.

        Raises ``KeyError`` if ``name`` is unknown. Raises ``ValueError``
        if ``config`` fails provider-specific validation (builders may
        defer validation to first call — that's allowed but discouraged).
        """


def register_builtin_tts_providers(
    registry: TTSProviderRegistry | None = None,
) -> TTSProviderRegistry:
    """Register MiniMax (and future built-ins) under their canonical names.

    Called once at process start from the composition root
    (``tts_node.on_init`` for ROS, ``tts_cli`` entry-point for CLI).
    Returns the (possibly fresh) registry for convenience.

    Currently registers only ``"minimax"``. Future built-ins
    (``"elevenlabs"``, ``"google"``, ``"local-piper"``) will be added
    here as their providers land in ``providers/``.
    """


__all__ = [
    "TTSProviderRegistry",
    "TTSProviderFactory",
    "register_builtin_tts_providers",
]
```

### `providers/elevenlabs_tts.py` (stub)

```python
"""ElevenLabs TTS provider — STUB ONLY (design), t_8d714ff0.

**Do not import in production.** The class below is a signature-only
placeholder showing how the 5 extension points are filled in for
ElevenLabs. Real implementation lands as a separate task once
``BaseTTSProvider`` and ``TTSProviderRegistry`` (ADR-0007) are Accepted.

Reference: https://docs.elevenlabs.io/api-reference/text-to-speech
"""

from __future__ import annotations

from typing import Any

import httpx

from rob_box_llm.tts import TTSSettings
from rob_box_llm.tts_provider_base import (
    BaseTTSProvider,
    TTSCapabilities,
    TTSHealth,
    TTSVoice,
)


class ElevenLabsTTSProvider(BaseTTSProvider):
    """ElevenLabs TTS adapter — stub.

    Extension points filled in:

    1. ``capabilities()``  → streaming=True, voice_cloning=True,
                             audio_format_mp3=True; rest False.
    2. ``list_voices()``    → HTTP GET ``/v1/voices`` → list[TTSVoice].
    3. ``healthcheck()``    → HTTP GET ``/v1/user`` (cheap, auth-only).
    4. ``_build_request_payload`` → ElevenLabs-specific body
       (text, voice_settings, model_id, output_format).
    5. ``_http_client_factory``  → httpx.AsyncClient with ``xi-api-key``
       header and ``https://api.elevenlabs.io`` base URL.
    """

    name = "elevenlabs"
    DEFAULT_BASE_URL = "https://api.elevenlabs.io"

    def __init__(
        self,
        *,
        api_key: str | None = None,
        base_url: str = DEFAULT_BASE_URL,
        default_voice: str = "21m00Tcm4TlvDq8ikWAM",  # "Rachel"
        default_model: str = "eleven_monolingual_v1",
        timeout: float = 30.0,
        client: httpx.AsyncClient | None = None,
    ) -> None:
        # TODO: implementation lands in t_25b8e221 (registry + builders)
        # plus dedicated t_elevenlabs_implementation task.
        ...

    def capabilities(self) -> TTSCapabilities:
        # TODO
        return TTSCapabilities(
            streaming=True,
            voice_cloning=True,
            audio_format_mp3=True,
        )

    async def list_voices(self) -> list[TTSVoice]:
        # TODO: HTTP GET {base_url}/v1/voices → list[TTSVoice(...)]
        ...

    async def healthcheck(self) -> TTSHealth:
        # TODO: HTTP GET {base_url}/v1/user → TTSHealth(ok=..., latency_ms=...)
        ...

    def _build_request_payload(
        self,
        text: str,
        settings: TTSSettings,
        voice_meta: TTSVoice | None,
    ) -> dict[str, Any]:
        # TODO: pure mapping → ElevenLabs body shape
        ...

    async def synthesize(  # type: ignore[override]
        self, text: str, *, settings: TTSSettings | None = None,
    ) -> ...:
        # TODO: 1. resolve voice_meta, 2. _build_request_payload,
        #       3. POST {base_url}/v1/text-to-speech/{voice_id},
        #       4. wrap audio bytes in TTSAudio(sample_rate=44100,
        #          format=TTSFormat.MP3).
        ...

    async def stream(  # type: ignore[override]
        self, text: str, *, settings: TTSSettings | None = None,
    ) -> ...:
        # TODO: SSE chunked response → TTSChunk(...) until
        #       finish_reason="stop".
        ...

    async def aclose(self) -> None:
        # TODO: close owned httpx.AsyncClient (idempotent).
        ...


__all__ = ["ElevenLabsTTSProvider"]
```

### `providers/google_tts.py` (stub)

```python
"""Google Cloud Text-to-Speech provider — STUB ONLY (design), t_8d714ff0.

**Do not import in production.** Signature-only placeholder showing
how the 5 extension points are filled in for Google. Real implementation
lands as a separate task once ``BaseTTSProvider`` and ``TTSProviderRegistry``
(ADR-0007) are Accepted.

Reference: https://cloud.google.com/text-to-speech/docs/reference/rest
"""

from __future__ import annotations

from typing import Any

import httpx

from rob_box_llm.tts import TTSSettings
from rob_box_llm.tts_provider_base import (
    BaseTTSProvider,
    TTSCapabilities,
    TTSHealth,
    TTSVoice,
)


class GoogleTTSProvider(BaseTTSProvider):
    """Google Cloud TTS adapter — stub.

    Extension points filled in:

    1. ``capabilities()``  → streaming=True, ssml=True,
                             audio_format_pcm/mp3/ogg=True; voice_cloning=False.
    2. ``list_voices()``    → voices.list with OAuth2 token.
    3. ``healthcheck()``    → cheap OAuth token introspection.
    4. ``_build_request_payload`` → ``{"input": {"ssml": ...},
       "voice": {"name": ..., "languageCode": ...}, "audioConfig": ...}``.
    5. ``_http_client_factory``  → httpx.AsyncClient with OAuth Bearer
       header (token refreshed in background via ``google.auth``).
    """

    name = "google"
    DEFAULT_BASE_URL = "https://texttospeech.googleapis.com"

    def __init__(
        self,
        *,
        credentials_path: str | None = None,  # service-account JSON
        base_url: str = DEFAULT_BASE_URL,
        default_voice: str = "en-US-Wavenet-A",
        default_language: str = "en-US",
        timeout: float = 30.0,
        client: httpx.AsyncClient | None = None,
    ) -> None:
        # TODO: implementation lands in t_25b8e221 + dedicated task.
        ...

    def capabilities(self) -> TTSCapabilities:
        # TODO
        return TTSCapabilities(
            streaming=True,
            ssml=True,
            audio_format_pcm=True,
            audio_format_mp3=True,
            audio_format_ogg=True,
        )

    async def list_voices(self) -> list[TTSVoice]:
        # TODO: voices.list with OAuth2 → list[TTSVoice(...)]
        ...

    async def healthcheck(self) -> TTSHealth:
        # TODO: OAuth token introspection → TTSHealth(ok=..., latency_ms=...)
        ...

    def _build_request_payload(
        self,
        text: str,
        settings: TTSSettings,
        voice_meta: TTSVoice | None,
    ) -> dict[str, Any]:
        # TODO: pure mapping → Google texttospeech body shape.
        ...

    async def synthesize(  # type: ignore[override]
        self, text: str, *, settings: TTSSettings | None = None,
    ) -> ...:
        # TODO: POST {base_url}/v1/text: synthesize → JSON with
        #       ``audioContent`` (base64). Decode → TTSAudio.
        ...

    async def stream(  # type: ignore[override]
        self, text: str, *, settings: TTSSettings | None = None,
    ) -> ...:
        # TODO: not natively streaming — document explicitly that
        #       stream() degrades to single-chunk synthesize() with
        #       finish_reason="stop" per ADR-0003 §2.4 contract.
        ...

    async def aclose(self) -> None:
        # TODO: close owned httpx.AsyncClient (idempotent).
        ...


__all__ = ["GoogleTTSProvider"]
```

### `providers/local_piper_tts.py` (stub)

```python
"""Local Piper TTS provider — STUB ONLY (design), t_8d714ff0.

**Do not import in production.** Signature-only placeholder showing
how the 5 extension points are filled in for a fully-local, offline
Piper (https://github.com/rhasspy/piper) adapter. Real implementation
lands as a separate task.

Use case: privacy-sensitive scenarios, no internet, ROS deployment
without external API quotas. ``custom_endpoint=True`` marks it as
non-cloud; capability flags are all False (no streaming, no cloning,
no SSML) — but availability is 100% local.
"""

from __future__ import annotations

from typing import Any

from rob_box_llm.tts import TTSSettings
from rob_box_llm.tts_provider_base import (
    BaseTTSProvider,
    TTSCapabilities,
    TTSHealth,
    TTSVoice,
)


class LocalPiperTTSProvider(BaseTTSProvider):
    """Local Piper TTS adapter — stub.

    Extension points filled in:

    1. ``capabilities()``  → custom_endpoint=True; all flags False
                             (Piper is local, single-voice per ONNX,
                              no streaming, no cloning, no SSML).
    2. ``list_voices()``    → list ``.onnx`` files in
                             ``/usr/share/piper/voices/`` → list[TTSVoice].
    3. ``healthcheck()``    → ``subprocess.run(["piper", "--version"])``.
    4. ``_build_request_payload`` → CLI arg-list shape (NOT JSON),
       since Piper is a subprocess not an HTTP endpoint.
    5. ``_http_client_factory``  → NOT applicable; default returns
       httpx.AsyncClient (unused) so the type contract holds; runtime
       uses subprocess instead. A future ADR may add a
       ``_transport_factory()`` hook for non-HTTP providers.
    """

    name = "local-piper"
    DEFAULT_VOICES_DIR = "/usr/share/piper/voices"

    def __init__(
        self,
        *,
        piper_binary: str = "piper",
        voices_dir: str = DEFAULT_VOICES_DIR,
        default_voice: str = "ru_RU-irina-medium",
        timeout: float = 30.0,
    ) -> None:
        # TODO: implementation lands as a separate task; this provider
        #       does NOT use httpx (subprocess transport).
        ...

    def capabilities(self) -> TTSCapabilities:
        # TODO
        return TTSCapabilities(custom_endpoint=True)

    async def list_voices(self) -> list[TTSVoice]:
        # TODO: glob voices_dir/*.onnx → list[TTSVoice(id=filename, ...)]
        ...

    async def healthcheck(self) -> TTSHealth:
        # TODO: subprocess.run([piper_binary, "--version"]) → TTSHealth.
        ...

    def _build_request_payload(
        self,
        text: str,
        settings: TTSSettings,
        voice_meta: TTSVoice | None,
    ) -> dict[str, Any]:
        # TODO: NOT a real HTTP payload — return Piper CLI shape
        #       {"text_file": ..., "voice_model": ..., "output_file": ...}.
        #       Subprocess wrapper reads this dict and shells out.
        ...

    async def synthesize(  # type: ignore[override]
        self, text: str, *, settings: TTSSettings | None = None,
    ) -> ...:
        # TODO: 1. write text to tmpfile, 2. shell out to ``piper``
        #       with ``--model {voice}.onnx --output_file out.wav``,
        #       3. read WAV, wrap in TTSAudio(sample_rate=22050,
        #          format=TTSFormat.WAV).
        ...

    async def stream(  # type: ignore[override]
        self, text: str, *, settings: TTSSettings | None = None,
    ) -> ...:
        # TODO: Piper is single-shot — stream() degrades to single-chunk
        #       synthesize() with finish_reason="stop" per ADR-0003 §2.4
        #       contract. Explicitly NOT a streaming provider.
        ...

    async def aclose(self) -> None:
        # TODO: no HTTP client to close; default no-op idempotent.
        ...


__all__ = ["LocalPiperTTSProvider"]
```

---

## 10. Что дальше (для следующих задач)

- **t_25b8e221** — реализовать `BaseTTSProvider` + `TTSProviderRegistry` + `TTSProviderFactory`; мигрировать `MiniMaxTTSProvider` на новую базу; обновить `tts_node._synthesize_and_play` на `factory.create(provider_name, ...)`.
- **t_eed9d0f3** — собрать ADR-review-пакет, объединяющий ADR-0004/0006/0007 + этот design-only документ.
- **Отдельные задачи** на реализацию каждого stub-провайдера (`elevenlabs`, `google`, `local-piper`) — каждая начинается ТОЛЬКО после `Accepted` у ADR-0007.
- **Будущее расширение** — если появится non-HTTP провайдер (типа local-Piper, но без subprocess-shell-out), добавить `_transport_factory()` hook поверх существующего `_http_client_factory()`. YAGNI пока.