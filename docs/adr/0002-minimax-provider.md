# ADR-0002: MiniMax TTS-провайдер через rob_box_llm.TTSProvider

| Поле         | Значение                                                                |
|--------------|-------------------------------------------------------------------------|
| Статус       | Accepted                                                                |
| Дата         | 2026-07-20                                                              |
| Автор        | developer (Hermes Agent)                                                |
| Контекст     | Kanban task `t_ac5f796b`, родительская review `t_7e32da0e`              |
| Заменяет     | —                                                                       |
| Заменяется   | —                                                                       |

---

## 1. Контекст

В РОББОКСе `tts_node` работает с Yandex Cloud TTS gRPC v3 (основной) и
Silero v5 (офлайн-fallback). В 2026 году появился интерес к MiniMax —
AI-провайдеру с TTS-сервисом (`T2A v2` HTTP endpoint), который:

* даёт качественный русский голос на уровне Yandex;
* поддерживает эмоциональную окраску (`emotion`), отсутствующую у Yandex
  и Silero;
* использует простой HTTPS POST без gRPC — проще дебажить и проксировать;
* унифицирует счёт с LLM-сервисом MiniMax (`MiniMax-M3`), который уже
  рассматривается как опциональный LLM-провайдер.

Цель — добавить MiniMax как **опциональный третий TTS-провайдер** в
`tts_node`, не ломая существующий Yandex/Silero пайплайн и не меняя
ROS-топики (инвариант P0 foundation).

## 2. Решение

Принято решение реализовать **P0.5 расширение** `rob_box_llm`:

1. **Новый ABC**: `TTSProvider` в `rob_box_llm.tts` — async-only контракт,
   симметричный `LLMProvider`. Методы: `synthesize(text, *, settings)` и
   `stream(text, *, settings)`. Value objects — `TTSAudio`, `TTSChunk`,
   `TTSSettings`, `TTSFormat`.

2. **Новая иерархия ошибок**: `TTSError` + `TTSAuthError`,
   `TTSRateLimitError`, `TTSTimeoutError`, `TTSBadRequestError`.
   **Отдельная** от LLM-иерархии (`ProviderError`) — чтобы
   `except ProviderError` в LLM-коде не ловил TTS-сбои и наоборот.

3. **Конкретный провайдер**: `MiniMaxTTSProvider` в
   `rob_box_llm.providers.minimax_tts`. HTTP-only (`httpx.AsyncClient`),
   без async-SDK, чтобы юнит-тесты мокались одной строкой через
   `httpx.MockTransport`.

4. **Wire в `tts_node.py`**: новый branch в `_synthesize_and_play` —
   `if self.provider == "minimax": ...`. Импорт `rob_box_llm` — через
   try/except с fallback на `MINIMAX_AVAILABLE=False` (если rob_box_llm
   не собран в Docker-образе, Yandex/Silero путь продолжает работать).

5. **Конфигурация**: ROS-параметры `minimax_*` ИЛИ ENV (`MINIMAX_API_KEY`,
   `MINIMAX_GROUP_ID`). Параметр имеет приоритет над ENV.

### 2.1 Альтернативы, которые мы НЕ взяли

* **Полная замена Yandex на MiniMax** — отвергнуто: Yandex "оригинальный
  ROBBOX голос" (anton), менять качество дефолта без явного запроса
  нельзя.
* **Добавление MiniMax SDK напрямую в rob_box_voice** — отвергнуто:
  дублирует абстракцию (LLM-пакет уже имеет `LLMProvider` ABC, идиома —
  side-effect провайдеры живут в `rob_box_llm`).
* **Стриминг chunk-per-frame через T2A WebSocket** — отложено: SSE
  через HTTP проще и для нашего use-case (не интерактивный voice) — fine.
* **Capability-флаг `CapabilityUnavailableError` (упомянут в review
  `t_7e32da0e`)** — отвергнут как class: нам достаточно явного
  `TTSError("rob_box_llm not available")` в `tts_node.py`. Создание
  нового exception-типа без конкретного потребителя — over-engineering
  (YAGNI).

## 3. Структура изменений

```
src/rob_box_llm/
├── rob_box_llm/
│   ├── __init__.py                       (bump: добавить TTSProvider, MiniMaxTTSProvider)
│   ├── errors.py                         (добавить: TTSError + 4 подкласса)
│   ├── tts.py                            (новый: TTSProvider ABC + value objects)
│   └── providers/
│       ├── __init__.py                   (добавить: MiniMaxTTSProvider)
│       └── minimax_tts.py                (новый: HTTP impl)
├── test/
│   ├── test_tts_value_objects.py         (новый: 11 тестов)
│   └── test_minimax_tts_provider.py      (новый: 39 тестов)
├── package.xml                           (добавить exec_depend: python3-httpx)
└── setup.py                              (добавить: httpx>=0.27)

src/rob_box_voice/
├── rob_box_voice/
│   └── tts_node.py                       (добавить: provider="minimax" + lazy init)
└── package.xml                           (добавить exec_depend: python3-httpx, optional: rob_box_llm)

docker/vision/
└── .env.secrets.template                 (добавить: MINIMAX_API_KEY, MINIMAX_GROUP_ID)

.env.example                              (добавить: секция TTS ПРОВАЙДЕРЫ)
docs/architecture/minimax-provider.md     (новый: архитектурный обзор)
docs/adr/0002-minimax-provider.md         (этот файл)
```

## 4. Последствия

### Положительные

* TTS-нода получает **третий опциональный провайдер** без поломки
  существующего поведения. Default `provider=yandex` работает как
  раньше.
* `TTSProvider` ABC открывает дорогу для будущих провайдеров
  (ElevenLabs, Azure Speech, etc.) — паттерн уже задан.
* Тесты мокаются одной строкой через `httpx.MockTransport` —
  50 unit-тестов за < 3 секунд, без сети.

### Отрицательные / риски

* Зависимость от `httpx` (раньше rob_box_llm требовал только `openai`).
  `httpx` уже есть в большинстве ROS-окружений; добавили в
  `package.xml`.
* `rob_box_llm` становится **опциональной** зависимостью для
  `rob_box_voice` (`<exec_depend optional="true">`). Yandex/Silero
  работают без неё; при сборке без rob_box_llm TTS-нода просто
  залогирует warning при `provider=minimax`.

### Нейтральные

* `rob_box_llm.__version__` бамп `0.1.0 → 0.2.0` (semver-minor: добавлены
  публичные классы без breaking changes).

## 5. Совместимость с ADR-0001

ADR-0001 ("Харнесы для dialog / persistent / telegram нод") установил
правило: **никаких изменений в dialogue_node.py и ROS-топиках до явного
ADR**. Этот ADR-0002:

* **Не трогает** `dialogue_node.py`, топики `/voice/dialogue/response`,
  `/voice/tts/request`, `/voice/audio/speech` — payload и контракт
  остаются идентичными.
* **Расширяет** `rob_box_llm` (P0 foundation) — additive, в духе
  ADR-0001.
* **Добавляет** опциональную зависимость `rob_box_llm` в
  `rob_box_voice` — пакет может собираться и без неё.

Совместимо.

## 6. Открытые вопросы / будущие ADR

* **Capability registry** (`CapabilityUnavailableError`) — отложено
  до появления второго use-case, где нужен runtime check capabilities
  по строковому имени (см. review `t_7e32da0e`, SUGGESTION #2).
* **T2A WebSocket для true streaming** — отдельная задача, не блокер.
* **Capability-флаг для image-generation через MiniMax** (SUGGESTION
  #4 из review) — относится к ADR будущей фазы M5/M6, не этого.
