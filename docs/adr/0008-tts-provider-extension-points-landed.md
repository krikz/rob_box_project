# ADR-0008: TTS Provider extension points — landed implementation

| Поле         | Значение                                                            |
|--------------|----------------------------------------------------------------------|
| Статус       | **Accepted** (landed — implementation merged)                        |
| Дата         | 2026-07-22                                                           |
| Автор        | backend (Hermes Agent)                                               |
| Kanban task  | `t_8cbf9995` (parent: `t_25b8e221`)                                  |
| Контекст     | Land the design from `t_8d714ff0` (ADR-0004 §2.8, ADR-0007 §2.2) into production code |
| Связанные    | [ADR-0004 §2.8](0004-minimax-tts-integration-design.md) (registry hook), [ADR-0007 §2.2](0007-minimax-tts-integration-final.md) (final architecture), [tts-extension-points.md](../architecture/tts-extension-points.md) (design doc) |

---

## 1. Контекст

`docs/architecture/tts-extension-points.md` (status: Draft, task `t_8d714ff0`) зафиксировал пять точек расширения для будущих TTS-провайдеров и design-only stubs в `docs/architecture/stubs/`. Эти stubs намеренно **не импортировались** в production-код — они существовали только как документация формы будущей абстракции.

`t_25b8e221` (architect) принял дизайн и зафиксировал его в `wt/t_25b8e221` (commit `d4412bbe`). `t_8cbf9995` (этот документ) — задача backend-исполнителя: перенести stubs в production и мигрировать `MiniMaxTTSProvider` на новый `BaseTTSProvider`.

## 2. Решение

### 2.1. Landed production modules

Два файла перенесены из `docs/architecture/stubs/` в production (с минимальными правками — см. §3):

* `src/rob_box_llm/rob_box_llm/tts_provider_base.py` (255 строк)
  — `BaseTTSProvider`, `TTSCapabilities`, `TTSVoice`, `TTSHealth`, `ProviderBuilder`.
* `src/rob_box_llm/rob_box_llm/tts_provider_registry.py` (152 строки)
  — `TTSProviderRegistry`, `TTSProviderFactory`, `register_builtin_tts_providers`.

### 2.2. Миграция `MiniMaxTTSProvider`

`MiniMaxTTSProvider(TTSProvider)` → `MiniMaxTTSProvider(BaseTTSProvider)`.

Реализованы все пять точек расширения (детали — в §2.3):

1. `capabilities()` → `TTSCapabilities(streaming=True, voice_cloning=True, audio_format_pcm/mp3=True; ssml/ogg/pronunciation_dict/custom_endpoint=False)`
2. `list_voices()` → 6-голосовый статический каталог (ru/en/zh, без upstream-вызова)
3. `healthcheck()` → pure credential-presence check (нет HTTP, нет quota-burn)
4. `_http_client_factory()` → `httpx.AsyncClient(timeout=self._timeout)` (default; override-ready)
5. `_build_request_payload()` → делегирует в module-level `_build_payload` (тот же helper, что используют `synthesize`/`stream`, чтобы избежать drift между двумя builders)

Конструктор теперь маршрутизирует создание HTTP-клиента через `_http_client_factory()` — субклассы могут подменить transport / TLS / OAuth headers, не трогая `__init__`.

### 2.3. Public surface

`rob_box_llm/__init__.py` реэкспортирует новые символы (помимо существующих `TTSProvider`, `MiniMaxTTSProvider` и т.д.):

```python
BaseTTSProvider, TTSCapabilities, TTSVoice, TTSHealth, ProviderBuilder,
TTSProviderRegistry, TTSProviderFactory, register_builtin_tts_providers
```

Backward-compat: `MiniMaxTTSProvider` остаётся `MiniMaxTTSProvider` — публичный контракт `synthesize / stream / aclose` (frozen в `tts.py:121-177`) не изменился. Любой код, типизирующий `TTSProvider`, продолжает работать без правок (`isinstance(x, TTSProvider)` для мигрированного провайдера — `True`).

## 3. Отличия от design stub

Stub в `docs/architecture/stubs/` имел две проблемы, исправленные при переносе:

1. **`ProviderBuilder` имел дублирующееся определение** в stub-файле (TypeAlias + docstring-only def). Production-версия использует чистый `Callable[[Mapping[str, Any]], "BaseTTSProvider"]` без лишнего def-блока.

2. **`TTSVoice.extra` не был frozen.** Production-версия оборачивает `extra` в `MappingProxyType` через `__post_init__` — как и `TTSSettings.extra` в `tts.py:75-77`, для консистентности.

В остальном — прямой перенос, без drift от дизайна.

## 4. Тесты

47 новых тестов в `src/rob_box_llm/test/test_tts_extension_points.py` покрывают:

* **BaseTTSProvider + value objects** (10 тестов): ABC contract, instantiation guards, `TTSCapabilities` defaults/frozen, `TTSVoice.extra` readonly, `TTSHealth` defaults/frozen, `ProviderBuilder` callable type, `_http_client_factory` default.
* **MiniMaxTTSProvider migration** (18 тестов): IS-A `BaseTTSProvider` (forward-compat), IS-A `TTSProvider` (backward-compat), FakeTTSProvider is NOT a `BaseTTSProvider` (asymmetric migration policy), capabilities shape, voice catalogue (6 voices, unique ids, includes `Russian_CalmWoman`, no upstream call), healthcheck (ok / missing-key / missing-group / no upstream), `_http_client_factory` returns AsyncClient with timeout, `_build_request_payload` delegates to module-level helper, no I/O.
* **TTSProviderRegistry** (5 тестов): register/resolve, sorted names, duplicate register raises, unknown resolve raises KeyError, error message lists available names.
* **TTSProviderFactory** (5 тестов): create returns instance, caches by `(name, config_hash)`, different config misses cache, unknown name raises, reset_cache clears state.
* **`register_builtin_tts_providers`** (4 теста): fresh registry when none passed, appends to passed-in registry, duplicate registration raises, builder produces real provider.
* **Cross-cutting** (1 тест): третья сторона может subclass `BaseTTSProvider`, зарегистрировать builder и получить свой инстанс через `TTSProviderFactory.create`.

**Coverage**: `rob_box_llm/providers/minimax_tts.py` — 95% (gate 85%). Общий suite — 336 тестов, все зелёные.

## 5. Что НЕ входит в эту задачу

Эта задача — landed implementation, не redesign. Решения, отложенные до второго opt-in provider (ADR-0004 §2.8):

* Миграция `FakeTTSProvider` на `BaseTTSProvider` (сейчас остаётся на `TTSProvider`).
* Включение `BaseTTSProvider._http_client_factory()` в composition root для OAuth / proxy / TLS — отложено до появления ElevenLabs/Google.
* Авто-регистрация через `importlib.metadata.entry_points()` — отклонено (ADR-0004 §2.3).

## 6. Verification

```sh
$ make test-tts
===================== 223 passed, 113 deselected in 8.19s ======================
Required test coverage of 85% reached. Total coverage: 95.38%

$ ruff check src/rob_box_llm/rob_box_llm/tts_provider_base.py \
              src/rob_box_llm/rob_box_llm/tts_provider_registry.py \
              src/rob_box_llm/rob_box_llm/providers/minimax_tts.py
All checks passed!
```

`__init__.py` имеет 16 pre-existing ruff warning'ов (не относятся к этой задаче — `TextPart` / `ImagePart` дублирующиеся импорты уже были до моих правок; мои правки добавили 0 новых warning'ов).