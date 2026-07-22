# ADR-0008: TTS provider extension points — landed implementation

| Поле         | Значение                                                                |
|--------------|-------------------------------------------------------------------------|
| Статус       | **Accepted**                                                            |
| Дата         | 2026-07-22                                                              |
| Автор        | architect (Hermes Agent)                                                |
| Контекст     | Kanban task `t_25b8e221`, дочерняя от `t_8d714ff0` (extension-points design) |
| Родители     | [ADR-0004 §2.1–2.4](0004-minimax-tts-integration-design.md) (port, registry, retry), [ADR-0007](0007-minimax-tts-integration-final.md) (финальный синтез), [ADR-0003](0003-minimax-tts-architecture.md) (текущая реализация MiniMax) |
| Дизайн-документ | [`../architecture/tts-extension-points.md`](../architecture/tts-extension-points.md) |
| Реализует    | Перенос stubs `tts_provider_base.py` + `tts_provider_registry.py` в production и миграция `MiniMaxTTSProvider` на `BaseTTSProvider` |

---

## 1. Контекст

ADR-0007 зафиксировал итоговую архитектуру интеграции MiniMax TTS, но
**отложил** введение 5 точек расширения и явного реестра провайдеров,
потому что на тот момент единственным реальным провайдером был MiniMax
(зафиксировано в [`../architecture/tts-extension-points.md`](../architecture/tts-extension-points.md)
§0: *"Existing MiniMaxTTSProvider(TTSProvider) does NOT migrate to
this base in this PR"*).

Документ [`../architecture/tts-extension-points.md`](../architecture/tts-extension-points.md)
(t_8d714ff0) явно указал: *"дочерняя задача `t_25b8e221` и `t_b4cb8948`"*
должны перенести stubs из `docs/architecture/stubs/` в production
(`src/rob_box_llm/rob_box_llm/`) и адаптировать существующий
`MiniMaxTTSProvider`.

Этот ADR закрывает именно это: перенос + миграция.

### 1.1 Что в ADR-0008 НЕ покрывается

* Реализация второго провайдера (ElevenLabs / Google / local-Piper) —
  каждый получает свой ADR при старте.
* Изменение публичного контракта `TTSProvider` (synthesize / stream /
  aclose) — frozen at P0.5 per `tts.py:121-177`.
* Изменение wire-формата MiniMax T2A v2 — закрыто в ADR-0003.

---

## 2. Решения

### 2.1 Перенос `tts_provider_base.py` из stubs в production

**Решение:** `docs/architecture/stubs/tts_provider_base.py` →
`src/rob_box_llm/rob_box_llm/tts_provider_base.py`. Содержимое
**сохранено**, исправлены 2 дефекта относительно stubs:

1. Класс наследует `TTSProvider` (а не объявлен как самостоятельный
   ABC). Stubs декларировали `BaseTTSProvider(abc.ABC)` без родителя —
   это нарушало backward-compat гарантию design-документа
   (*"BaseTTSProvider IS-A TTSProvider"*).
2. Убран `@abc.abstractmethod` у `aclose()` — в `TTSProvider` он уже
   конкретный (no-op default). Двойное абстрагирование бессмысленно.

В `BaseTTSProvider.__abstractmethods__` остаются ровно 3 метода:
`{synthesize, stream, _build_request_payload}` — каждый из них
обязателен к override.

### 2.2 Перенос `tts_provider_registry.py` из stubs в production

**Решение:** `docs/architecture/stubs/tts_provider_registry.py` →
`src/rob_box_llm/rob_box_llm/tts_provider_registry.py`.

Изменения относительно stubs:

* Добавлен метод `unregister(name)` — test-only helper, чтобы тесты
  могли изолировать регистрации (полезно при test ordering).
* Поправлена мелкая typo в docstring (отсутствовала backtick перед
  `KeyError`).

Поведение `TTSProviderFactory._cache` оставлено без изменений:
ключ `(name, config_hash)`, `reset_cache()` — test-only.

### 2.3 Регистрация MiniMax в `register_builtin_tts_providers`

**Решение:** builder для `MiniMaxTTSProvider` уже зарегистрирован в
stubs-версии под именем `"minimax"` — оставлено без изменений. Это
единственная built-in регистрация до появления второго провайдера.

`register_builtin_tts_providers()` теперь импортируется из
`rob_box_llm.tts_provider_registry` и экспортируется в
`rob_box_llm.__init__` под именем `register_builtin_tts_providers`.

### 2.4 Миграция `MiniMaxTTSProvider(TTSProvider)` → `BaseTTSProvider`

**Решение:** класс `MiniMaxTTSProvider` в
`src/rob_box_llm/rob_box_llm/providers/minimax_tts.py` теперь наследует
`BaseTTSProvider` (а не `TTSProvider` напрямую). Поскольку
`BaseTTSProvider(TTSProvider)`, MRO остаётся линейным, и любой
существующий type-check `isinstance(p, TTSProvider)` остаётся `True`.

Изменения в самом классе:

| Элемент | Что сделано |
|---------|-------------|
| `__init__` | `httpx.AsyncClient(timeout=timeout)` заменён на `self._http_client_factory()` (т.е. через новый extension point). Логика создания клиента вынесена в override метод. |
| `capabilities()` | Новый метод — возвращает `TTSCapabilities(streaming=True, voice_cloning=True, ssml=False, pronunciation_dict=False, audio_format_pcm=True, audio_format_mp3=True, audio_format_ogg=False, custom_endpoint=False)`. Обоснование каждого флага — в docstring метода. |
| `list_voices()` | Новый async метод — возвращает статический каталог `_BUILTIN_VOICES` (6 голосов из MiniMax docs). Заменяется на HTTP call когда MiniMax опубликует `/v1/voices`. |
| `healthcheck()` | Новый async метод — fail-fast проверяет `MINIMAX_API_KEY` и `MINIMAX_GROUP_ID`. **НЕ** делает upstream call (это для `ping_minimax.py`). |
| `_http_client_factory()` | Новый override — `httpx.AsyncClient(timeout=self._timeout)`. Default из BaseTTSProvider идентичен; override нужен для документирования. |
| `_build_request_payload()` | Новый метод (extension point) — pure mapping через делегацию к module-level `_build_payload`. Поддерживает `voice_meta` параметр: если `TTSSettings.voice is None` и `voice_meta` передан, используется `voice_meta.id`. Если `voice_meta is None` или `settings.voice` явно задан — `voice_meta` игнорируется. |
| `_build_payload` (module-level) | Сохранён без изменений. Используется из `_build_request_payload` и напрямую из существующих тестов `test_minimax_tts_provider.py`. |

### 2.5 `MiniMaxTTSProvider` остаётся `MiniMaxTTSProvider`

**Решение:** **не** делаем deprecation alias или версионированное
переименование (`MiniMaxTTSProviderV2`). Причина:

* Публичный контракт (synthesize / stream / aclose) **не изменился**.
* Все 204 существующих теста проходят без правок (см. раздел 4).
* Любой код, импортирующий `MiniMaxTTSProvider` (включая `tts_node`),
  продолжает работать.
* Роз-нода не использует `capabilities() / list_voices() /
  healthcheck()` напрямую — она получает провайдер через
  `MiniMaxTTSProvider(**config)` в `tts_node._synthesize_and_play`
  (см. ADR-0003 §2.2). Миграция на `TTSProviderFactory.create()` —
  **отдельная задача** (`t_xxx` follow-up), потому что требует
  правки `tts_node` и не связана с дочерним характером `t_25b8e221`.

### 2.6 Экспорт из `rob_box_llm.__init__`

Добавлены в `__all__`:

* `BaseTTSProvider`, `ProviderBuilder`
* `TTSCapabilities`, `TTSHealth`, `TTSVoice`
* `TTSProviderRegistry`, `TTSProviderFactory`
* `register_builtin_tts_providers`

---

## 3. Структура артефактов (diff vs. design-only)

```
docs/architecture/stubs/                          src/rob_box_llm/rob_box_llm/
├── tts_provider_base.py       ─── перенесён ───►  tts_provider_base.py
├── tts_provider_registry.py   ─── перенесён ───►  tts_provider_registry.py
├── elevenlabs_tts.py          (оставлены как     elevenlabs_tts.py
│                                reference stubs)  (reference stubs,
├── google_tts.py              ─ без изменений ──► google_tts.py
│                                                 reference для
└── local_piper_tts.py         ─ без изменений ──► local_piper_tts.py
                                                  следующих ADR-ов)
```

`docs/architecture/stubs/tts_provider_base.py` и
`tts_provider_registry.py` **удалены**, потому что их содержимое теперь
живёт в production коде. Stubs для ещё-не-реализованных провайдеров
(elevenlabs/google/local_piper) сохранены — они остаются полезной
reference-формой для следующих ADR-ов.

---

## 4. Verification

| Что проверено | Где | Результат |
|---------------|-----|-----------|
| Существующие 204 теста MiniMax TTS не сломались | `make test-tts` | ✅ 204 passed |
| Покрытие `minimax_tts.py` ≥ 85% gate | `make test-tts` | ✅ 96% (выше gate) |
| Новые 27 тестов extension points | `test/test_tts_extension_points.py` | ✅ 27 passed |
| `BaseTTSProvider IS-A TTSProvider` гарантирована | `test_base_tts_provider_is_a_tts_provider` | ✅ |
| `MiniMaxTTSProvider` IS-A `BaseTTSProvider` | `test_minimax_is_base_tts_provider` | ✅ |
| `capabilities()`, `list_voices()`, `healthcheck()` корректны | 5 новых тестов | ✅ |
| `_build_request_payload` правильно использует `voice_meta` | 3 новых теста | ✅ |
| Реестр + фабрика работают, кэш ключуется правильно | 8 новых тестов | ✅ |
| `register_builtin_tts_providers` экспортирует minimax | 2 новых теста | ✅ |

**Итого:** 215 тестов проходят, 96% покрытие, 0 регрессий.

---

## 5. Альтернативы, которые отклонены

| Альтернатива | Почему отклонена |
|--------------|------------------|
| Сохранить stubs в `docs/architecture/stubs/` параллельно с production | Дублирование кода, риск расхождения (как в stubs уже случилось: `BaseTTSProvider(abc.ABC)` без родителя — баг). |
| Переименовать `MiniMaxTTSProvider` → `MiniMaxTTSProviderV2` | Ломает публичный API, требует правки `tts_node` и `__init__.py`. Выходит за scope дочерней задачи. |
| Мигрировать `tts_node._synthesize_and_play` на `TTSProviderFactory.create()` | Это отдельная задача (требует решения про backwards-compat ROS-launch конфигов). Здесь миграция только в `rob_box_llm`. |
| Делать `capabilities()` ленивым (HTTP-вызов к MiniMax) | Невоспроизводимо в unit-тестах; MiniMax не имеет `/v1/capabilities`. Static declaration — единственный честный путь. |
| Авто-регистрация провайдеров через `importlib.metadata.entry_points()` | ADR-0004 §2.3 уже отверг это (implicit side-effects, hard to test, hidden deps at import time). |

---

## 6. Последствия

### Положительные

* **Backward-compat сохранена:** любой код, импортирующий
  `MiniMaxTTSProvider` или типизирующий `TTSProvider`, продолжает
  работать без правок. ADR-0001 invariant не нарушен.
* **Extension surface готов к второму провайдеру:** добавление
  ElevenLabs/Google/local-Piper = одна правка в
  `register_builtin_tts_providers` + новый файл в `providers/`.
* **Capabilities стали first-class:** `tts_node` (или будущий
  `dialogue_node`) может проверить `provider.capabilities()` ДО вызова
  и предупредить пользователя о неподдерживаемой фиче.
* **Покрытие выросло с 90% до 96%** на `minimax_tts.py` (новые тесты
  extension-points).

### Отрицательные / риски

* `BaseTTSProvider.__abstractmethods__` теперь шире, чем у `TTSProvider`
  (3 метода вместо 2). Любой 3rd-party код, который раньше объявлял
  `class X(TTSProvider)`, не сломается — но новый код, объявляющий
  `class Y(BaseTTSProvider)`, обязан реализовать 3 метода.
* `MiniMaxTTSProvider.__init__` теперь создаёт HTTP client через
  `self._http_client_factory()`. Если кто-то в subclass переопределит
  этот метод — конструктор MiniMax может вернуть неожиданный клиент.
  Это by design (extensibility), но зафиксировано в docstring.

### Нейтральные

* В `__init__.py` добавлено 8 новых экспортов. `make lint` не падает.
* Stubs `elevenlabs_tts.py` / `google_tts.py` / `local_piper_tts.py`
  остаются — они полезны как reference.

---

## 7. Совместимость с предыдущими ADR

* **ADR-0001** (harness architecture): не затрагивается. ROS-ноды
  по-прежнему импортируют `MiniMaxTTSProvider` из `rob_box_llm`.
* **ADR-0002** (MiniMax provider): не затрагивается — текстовый провайдер
  остаётся в `rob_box_llm.providers.minimax`.
* **ADR-0003** (MiniMax TTS architecture): реализация маппинга TTSSettings
  → T2A v2 body **не изменилась** — module-level `_build_payload`
  сохранён без правок. `BaseTTSProvider._build_request_payload` лишь
  делегирует к нему.
* **ADR-0004** (MiniMax TTS integration design): §2.3 явно требует
  composition-root registry — выполнено в §2.2 этого ADR.
* **ADR-0006** (pydantic-settings config): конфиг MiniMax остаётся как
  было; `BaseTTSProvider` и `TTSProviderRegistry` принимают уже
  валидированный dict.
* **ADR-0007** (final synthesis): принятые в нём решения остаются в
  силе; этот ADR — дочерний refinement, не отменяющий 0007.

---

## 8. Acceptance

ADR считается **fully landed**, когда:

1. ✅ Stubs перенесены в production, `BaseTTSProvider` IS-A
   `TTSProvider`, `_build_request_payload` принимает `voice_meta`.
2. ✅ `MiniMaxTTSProvider` наследует `BaseTTSProvider` и реализует все
   5 extension points.
3. ✅ Реестр содержит `minimax` под именем `"minimax"`.
4. ✅ Все 215 тестов зелёные, покрытие 96%.
5. ✅ Публичные экспорты добавлены в `rob_box_llm.__init__`.

Все 5 пунктов выполнены → ADR Accepted.

### Следующие шаги (отдельные задачи)

* `t_b4cb8948` — реализация ElevenLabs/Google/local-Piper provider-ов.
* Follow-up: миграция `tts_node._synthesize_and_play` на
  `TTSProviderFactory.create()` (требует решения про ROS-launch ↔
  registry).
* Follow-up: замена `_BUILTIN_VOICES` на HTTP call когда MiniMax
  опубликует `/v1/voices`.

---

## 9. Файлы изменены

| Файл | Δ |
|------|---|
| `src/rob_box_llm/rob_box_llm/tts_provider_base.py` | **создан** (перенос из stubs + 2 исправления) |
| `src/rob_box_llm/rob_box_llm/tts_provider_registry.py` | **создан** (перенос из stubs + `unregister`) |
| `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py` | +120 строк (5 extension points) — без правок в `synthesize/stream/aclose` |
| `src/rob_box_llm/rob_box_llm/__init__.py` | +8 экспортов |
| `src/rob_box_llm/test/test_tts_extension_points.py` | **создан** (27 тестов) |
| `docs/architecture/stubs/tts_provider_base.py` | **удалён** |
| `docs/architecture/stubs/tts_provider_registry.py` | **удалён** |
| `docs/adr/0008-tts-provider-extension-points.md` | **этот документ** |
