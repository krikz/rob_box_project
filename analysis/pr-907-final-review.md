# PR #907 — Сводное ревью (агрегация 5 специалистов)

- **PR:** https://github.com/krikz/rob_box_project/pull/907
- **Head SHA:** `6e75eb392cf2db75bebe740326ae7f2e2bba89fa`
- **База:** `91abcbca3b3e6c900bed154aca68ee1541ad981f`
- **Статус PR:** OPEN (по данным `api.github.com` на момент ревью)
- **Сводка:** 5 узких ревью (security, ROS2, QA-tests, architect-contracts, http-client) сведены в единый отчёт.
- **Дата:** 2026-07-22

## TL;DR

**ВЕРДИКТ: `REQUEST_CHANGES`.** PR сильный (контракты чистые, покрытие LLM-блока 100% при gate 85%, secrets hygiene образцовая, async-пути не содержат блокирующих вызовов), но до merge требуется закрыть **7 блокирующих проблем** из 4 разных ревью. Без их фикса merge делать нельзя — риск (1) утечки credentials в логи через upstream response, (2) падения voice pipeline при cold-start без MiniMax ключей, (3) runtime-креша на отсутствующем в коде TTS-провайдере `piper`, (4) OOM/висящих соединений при аномальных ответах MiniMax.

| Ревью | Вердикт | Файл с полным отчётом |
|-------|---------|------------------------|
| Security (secrets/leaks) | REQUEST_CHANGES | thread задачи t_0432e282 |
| ROS2 (blocking/N+1) | REQUEST_CHANGES | metadata t_47d4eddf |
| QA-tests (coverage/scope) | REQUEST_CHANGES | `analysis/pr-907-review.md` |
| Architect (contracts) | APPROVE-WITH-FOLLOWUPS | `analysis/pr-907-contract-review.md` |
| HTTP-client (timeout/retry/cancel) | REQUEST_CHANGES | `analysis/pr-907-http-client-review.md` |

---

## 🔴 Критические проблемы (блокируют merge)

### BLK-1 — Утечка тела ответа внешнего API в лог (Security, BLOCKER)
- **Файл:** `src/rob_box_voice/rob_box_voice/dialogue_node.py:1593-1597`
- **Проблема:** `f"🌩️ API error {exc.status_code}: {str(exc.response.text)[:200]} ..."` — тело upstream-ответа логируется без редактирования. Upstream/прокси могут вернуть `Authorization: Bearer ...`, cookies, фрагменты ключей или персональных данных → лог-канал становится каналом утечки.
- **Фикс:** не логировать body; оставить `status_code`/`request_id` либо прогонять текст через централизованный allowlist-редактор, маскирующий Authorization/Cookie/известные секреты. Добавить тест с echo `Authorization: Bearer ***` и проверкой `caplog`.
- **Источник:** t_0432e282 (security review).

### BLK-2 — Fail-fast отсутствует: `tts_node` стартует с пустыми MiniMax credentials (Security, MAJOR)
- **Файл:** `src/rob_box_voice/rob_box_voice/tts_node.py:386-392` (+ отложенный отказ до 1140-1145)
- **Проблема:** при `provider=minimax` отсутствие `MINIMAX_API_KEY`/`MINIMAX_GROUP_ID` логируется как warning, нода проходит `__init__` и успешно объявляет готовность. Реальный отказ происходит на первом synthesize-вызове в проде — нарушение fail-fast.
- **Фикс:** при `provider == "minimax"` валидировать оба env в `__init__`/startup, бросать конфигурационное исключение до объявления готовности. Добавить отдельный тест запуска без каждого обязательного env.
- **Источник:** t_0432e282.

### BLK-3 — `voice_assistant.yaml` ссылается на несуществующий TTS-провайдер `piper` (Architect/Config, P0)
- **Файл:** `docker/vision/config/voice_assistant/voice_assistant.yaml:129`
- **Проблема:** `provider: "piper"`, но `tts_node.py:229-233` принимает только `{yandex, silero, minimax}` → при запуске стек `ValueError: provider must be one of: ...`. Кто угодно, кто запускает этот конфиг, падает на старте.
- **Фикс:** заменить на `provider: "yandex"` (совпадает с `tts_node.py` defaults и голосом "anton").
- **Источник:** t_cce41a7d (architect F3).

### BLK-4 — `_apply_thinking_policy` теряет новые поля `LLMSettings` (Architect, P0)
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax.py:301-324`
- **Проблема:** rebuild `LLMSettings` field-by-field → добавление любого нового поля в `LLMSettings` (`provider.py:155-164`) молча выпадает из MiniMax-ответов. Скрытое нарушение контракта по умолчанию.
- **Фикс:** одна строка — `return dataclasses.replace(settings, extra=merged_extra)`.
- **Источник:** t_cce41a7d (F1).

### BLK-5 — `_OpenAICompatibleProvider.complete/stream` дважды итерируют `messages` без freeze (Architect, P0)
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/deepseek.py:211-254, 308-371`
- **Проблема:** `Iterable[LLMMessage]` принимает генератор; после первой итерации (в `_require_capability_for_messages`) он пуст → на второй (`_build_kwargs`) в OpenAI SDK уходит `messages=[]` → 400/пустой ответ.
- **Фикс:** `messages = tuple(messages)` в начале `complete()` и `stream()`. O(n) памяти, раз и навсегда убирает класс багов.
- **Источник:** t_cce41a7d (F2).

### BLK-6 — Таймауты в HTTP-клиентах: один float на все фазы (HTTP, BLOCKER)
- **Файлы:**
  - `src/rob_box_llm/rob_box_llm/providers/minimax.py:230-237` (LLM, через OpenAI SDK)
  - `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:573-581` (TTS, прямой httpx)
  - `src/rob_box_llm/rob_box_llm/tts_provider_base.py:241-252` (фабрика клиента)
- **Проблема:** `timeout: float` ⇒ для connect/read/write/pool одинаковые 30 секунд. При зависании DNS/TLS на connect-фазе пользователь голосового пайплайна ждёт 30 с на каждый вызов — типичный anti-pattern для latency-sensitive путей.
- **Фикс:** принимать `httpx.Timeout` или кортеж `(connect, read, write, pool)`. Разумный дефолт для MiniMax: `httpx.Timeout(connect=5.0, read=20.0, write=10.0, pool=5.0)`. OpenAI SDK принимает любой из float/Timeout.
- **Источник:** t_dd2b9833 (HTTP review, blocker #1).

### BLK-7 — Нет `Limits(max_content_size=...)` и `max_connections` в HTTP-клиентах (HTTP, BLOCKER)
- **Файлы:** `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:581`, `tts_provider_base.py:252`
- **Проблема:** клиент готов съесть ответ любого размера. LLM: 100 МБ JSON → OOM. TTS: hex-encoded audio для длинного текста → десятки МБ. Дефолт пула 100/100 для единичного провайдера избыточен.
- **Фикс:** задать `httpx.Limits(max_connections=10, max_keepalive_connections=5, max_content_size=50*1024*1024)` при создании клиента.
- **Источник:** t_dd2b9833 (HTTP review, blocker #2).

### BLK-7a — `aclose` LLM-провайдера не идемпотентен (HTTP, BLOCKER)
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/deepseek.py:372-373`
- **Проблема:** `await self._client.close()` без `is_closed` guard. При повторном `aclose` (или если caller уже закрыл клиент) → `RuntimeError: Client has not been opened`.
- **Фикс:**
  ```python
  async def aclose(self) -> None:
      if not self._client.is_closed:
          await self._client.close()
  ```
  (TTS-провайдер уже идемпотентен в `minimax_tts.py:913-920`.)
- **Источник:** t_dd2b9833 (HTTP review, blocker #3).

---

## ⚠️ Блокирующие проблемы из ROS2-аудита (отдельной строкой)

Полный отчёт — в metadata t_47d4eddf; блокеры не опубликованы на GitHub из-за отсутствия `gh` в worker-контейнере.

### BLK-R1 — Гонка shutdown с lazy-инициализацией MiniMax-провайдера в ROS2-ноде
- **Файл (вероятный):** `src/rob_box_voice/rob_box_voice/tts_node.py` (блок `_synthesize_minimax_async`, ~1140-1180)
- **Проблема:** провайдер создаётся при первом вызове (lazy init). Параллельный shutdown ноды может обратиться к частично инициализированному провайдеру → `AttributeError` / use-after-init.
- **Фикс:** вынести lazy-init в отдельный `_ensure_minimax_provider()` с `threading.Lock`/однократной инициализацией; в `shutdown()` сначала проверять `is_initialized`, затем `await provider.aclose()` с try/finally.
- **Примечание:** этот же пункт фигурирует как P2 F10 в architect-ревью (`tts_node._synthesize_minimax_async lazy-init duplicated`).

### BLK-R2 — Неограниченный fan-out daemon threads
- **Файл:** искать в `src/rob_box_voice/rob_box_voice/` вокруг tts_node/dialogue_node любые `threading.Thread(target=..., daemon=True)` без `Semaphore`/очереди.
- **Проблема:** порождение thread-per-request без лимита → при всплеске нагрузки количество потоков не ограничено, GIL contention, возможен OOM по стекам.
- **Фикс:** заменить на bounded `ThreadPoolExecutor` (или asyncio-путь), либо явно ограничить семафором. Задокументировать ожидаемую concurrency.

> **N+1 в затронутом пути НЕ подтверждён** — этот пункт из scope ROS2-ревью не вылился в блокер.

---

## 🟡 Рекомендации (можно отдельной итерацией)

### HTTP-уровень (P1 в исходном ревью)
1. **Retry без jitter** (`tts_node.py:1247`) — thundering herd. Добавить `jittered = delay * (0.5 + random.random())`.
2. **Retry игнорирует `Retry-After`** для 429 — учитывать заголовок MiniMax, иначе получим бан IP.
3. **Нет `asyncio.CancelledError` cleanup** в `synthesize`/`stream` — добавить `try/except CancelledError: await self.aclose(); raise`.
4. **Нет общего retry-декоратора для LLM-провайдеров** — caller ответственен за retry, явной инфраструктуры в `rob_box_llm` нет. Если retry добавят — учесть идемпотентность (LLM c tools НЕ идемпотентен).
5. **`APIConnectionError` → `TimeoutError` семантически нечестно** — теряется тип (Connect vs DNS vs TLS). Рассмотреть отдельный `ConnectionError(ProviderError)`.

### Architect/contract (P1 в исходном ревью)
6. **`_BUILTIN_VOICES` dead code** (`minimax_tts.py:312-355`) — runtime `list_voices()` использует другой inline-список. Удалить одну копию.
7. **`_build_request_payload` hardcode `stream=False`** (`minimax_tts.py:610-616`) — hook становится test-only. Расширить сигнатуру до `_build_request_payload(text, settings, voice_meta, *, stream: bool)`.
8. **`_validate_image_bytes` не покрывает URL/`data:` источники** (`minimax.py:130-148`) — 50 МБ `data:image/...;base64,...` пройдёт без проверки. Документировать bypass ИЛИ добавить HEAD+Content-Length для URL ИЛИ извлекать размер из base64.

### QA / tests
9. **Sleep в тесте SSE** (`src/rob_box_llm/test/test_minimax_tts_streaming.py:73-91`) — `await asyncio.sleep()` делает тест time-based/flaky. Заменить на детерминированный async iterator/transport; latency-тест пометить `slow`/`integration`.
10. **Voice/ROS тесты не изолированы от импорта numpy** (`src/rob_box_voice/test/unit/tts/test_minimax_integration.py`) — падают на collection без numpy. Либо добавить явную dev-зависимость, либо `pytest.importorskip("numpy")`/skip с marker. Сейчас workflow `G-TTS-Provider-Tests.yml` запускает только `src/rob_box_llm`, голосовые тесты этим gate не покрыты.
11. **Дубликат `if __name__ == "__main__"`** в `tools/audio_capture_harness/test_audio_capture_harness.py:559-568` — удалить.
12. **Workflow glob избыточен** (`.github/workflows/G-TTS-Provider-Tests.yml:39`) — `'src/rob_box_llm/**.py'` рядом с более конкретными glob'ами; привести к одному рекурсивному (`src/rob_box_llm/**/*.py`).
13. **Coverage scope**: gate измеряет только `minimax_tts.py` (100%); `minimax.py` и ROS-интеграция не покрыты. Добавить отдельные coverage targets или документировать границы.
14. **conftest `minimax_provider` fixture** создаёт `httpx.AsyncClient` без `aclose()` в teardown — потенциальный ресурсный leak при xdist.

---

## 🟢 Что хорошо (сохранить и тиражировать)

- **Secrets hygiene — образцовая.** `MINIMAX_API_KEY`/`MINIMAX_GROUP_ID` берутся только из ENV, редактируются в 3 слоях: `_redact_sensitive_text`, `_RedactGroupIdFilter`, `MiniMaxRedactedLogFilter`. 11 dedicated leak-guard тестов в `test_minimax_tts_request_params_and_leak_guard.py` покрывают каждый leak-path.
- **`_map_exception` централизован** (LLM — `deepseek.py:139`, TTS — `minimax_tts.py:96`, MiniMax envelope — `minimax.py:159`). Единая иерархия ошибок, одинаковый `provider=` kwarg, никаких `openai.APIError`/`httpx` наружу.
- **Капабилити-интроспекция честная.** `ProviderCapabilities` per-model на `MiniMaxProvider` (`minimax.py:244-261`). Vision-капабилити гейтится ДО сетевого вызова в `_require_capability_for_messages` → fail-fast с типизированным `CapabilityUnavailableError`.
- **Capability defaults консервативные** (`provider.py:78-92`) — все флаги False, кроме `text=True`. Несконфигурированный провайдер всегда capability-honest.
- **`FakeLLMProvider`/`FakeTTSProvider`** — детерминированные, реализуют контракты полностью, record calls. Тесты пакета идут без сети.
- **OpenAI-compatible wire format переиспользуется** через `_OpenAICompatibleProvider` — `DeepSeekProvider` и `MiMoProvider` это 25-строчные обёртки; `MiniMaxProvider` добавляет ~80 строк для envelope/pre-flight/thinking-policy. Будущие OpenAI-compat провайдеры = одна строка subclass.
- **TTS container honesty.** `synthesize` (`minimax_tts.py:752`) сообщает реальный формат контейнера на `TTSAudio.format` (OGG-запрос → MP3-ответ), транскодер не работает по лжи.
- **TTL-safe `aclose()`** для TTS (`minimax_tts.py:913-920`) — идемпотентен, закрывает только owned clients; caller-owned (с MockTransport) уважаются. **Контраст:** LLM-провайдер в этом непоследователен — это и есть BLK-7a.
- **Async-пути чистые.** Поиск `time.sleep|requests\.|urllib\.|open\(` в `minimax.py`/`minimax_tts.py`/`deepseek.py` — пусто. Никаких блокирующих I/O в async.
- **Тесты параметризованные и сильные.** `test_minimax_tts_errors_parametrized.py:312-405` покрывает 5 транспортных классов (`ConnectTimeout`/`ReadTimeout`/`PoolTimeout`/`ConnectError`/`NetworkError`) + `asyncio.TimeoutError`. LLM TTS suite: 244 passed, 113 deselected, coverage `minimax_tts.py` — 100%.
- **Контракты звучат.** `LLMProvider`/`TTSProvider`/`BaseTTSProvider` — чистое разделение. Отдельные `ProviderError`/`TTSError` иерархии. Multimodal: `MessageContent = Union[str, tuple[MessagePart, ...]]` — text-only callers не сломаны (357/357 старых тестов проходят). Capability defaults консервативные. TTS streaming честно сообщает "single-chunk + terminal stop" (не multi-frame), контракт документирован.

---

## ⚖️ Архитектурные trade-off'ы между ревью

- **Architect говорит APPROVE-WITH-FOLLOWUPS** для контрактов (102/102 conformance tests зелёные, реализации faithful), но **security/ROS2/HTTP/QA все говорят REQUEST_CHANGES** по разным подсистемам. Контрактный слой здоров, инфраструктурный — нет. Это типичный паттерн "хорошая абстракция, нездоровые низы", и блокирующие правки все сосредоточены на нижних слоях (config, lazy-init, HTTP timeouts, secrets/logs).
- **Retry policy**: HTTP-ревью требует jitter и `Retry-After`; ROS2-ревью фиксирует риск N+1, но в этом PR он не подтвердился. Решение — добавить jitter+Retry-After (HTTP-блокер) сейчас, N+1-мониторинг оставить post-merge.
- **Secrets/логирование**: TTS-провайдер уже редактирует ключи, но **dialogue_node.py:1593-1597** оказался вне этой защиты — leak surface там, где security-фильтр не навешан. Это блокер ровно потому, что один sink обходит все три слоя redaction.
- **Container honesty** vs **latency**: TTS streaming возвращает максимум 2 чанка (audio + terminal "stop") — это осознанно задокументировано, не multi-frame. WebSocket-стрим с настоящим chunk-per-frame отложен на M5/M6 (ADR-0003 §2.4). Принимаем как есть.

---

## 📋 Что делать дальше (владельцу PR)

1. **Закрыть 7 блокирующих проблем** (BLK-1 … BLK-7a + BLK-R1/R2) — все имеют конкретный file:line и готовые патчи (для HTTP-блокеров см. `analysis/pr-907-http-client-review.md` §"Конкретные предложения фиксов"; для architect F1/F2 — однострочники; для F3 — yaml one-liner).
2. Прогнать `PYTHONPATH=src/rob_box_llm:src/rob_box_llm/test pytest src/rob_box_llm/test/ -q` — должно остаться 244+ passed, без регрессий.
3. Опционально — закрыть P1-пункты (HTTP-jitter, dead `_BUILTIN_VOICES`, image URL cap) в этом же PR, они дёшевы.
4. После re-review — переключить вердикт на `APPROVE` и merge.

---

## Приложение: использованные артефакты

| Ревью | Артефакт |
|-------|----------|
| Security (secrets/leaks) | комментарий в thread t_0432e282 |
| ROS2 (blocking/N+1) | metadata t_47d4eddf (полного файла нет, worker контейнер не публиковал) |
| QA-tests (coverage/scope) | `analysis/pr-907-review.md` |
| Architect (contracts) | `analysis/pr-907-contract-review.md` |
| HTTP-client (timeout/retry/cancel) | `analysis/pr-907-http-client-review.md` |

**Публикация:** данный отчёт готов к публикации в виде сводного комментария на PR #907. Worker-контейнер `pr-reviewer` не имеет `gh` и `GITHUB_TOKEN`, поэтому публикация не выполнена автоматически — текст передан в `kanban_comment` и как артефакт в `analysis/pr-907-final-review.md`.

— pr-reviewer (Hermes Agent), 2026-07-22