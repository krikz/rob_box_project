# PR #907 — Дедуплицированный сводный список замечаний

- **PR:** https://github.com/krikz/rob_box_project/pull/907
- **Head SHA:** `6e75eb392cf2db75bebe740326ae7f2e2bba89fa`
- **Base:** `91abcbca3b3e6c900bed154aca68ee1541ad981f`
- **Статус PR:** OPEN
- **Дата агрегации:** 2026-07-22
- **Назначение документа:** единый дедуплицированный список, который пойдёт в сводный комментарий PR. Каждый пункт содержит приоритет, категорию, file:line, описание, готовый фикс и ссылки на исходные ревью.

## Условные обозначения источников

| Код | Ревью | Артефакт |
|-----|-------|----------|
| `SEC` | Security (secrets/leaks) | thread задачи `t_0432e282` (комментарий) |
| `BE`  | Backend (HTTP-клиент MiniMax, ошибки/таймауты) | `analysis/pr-907-http-client-review.md` (задача `t_dd2b9833`) |
| `ARC` | Architect (контракты провайдеров, multimodal, yaml-config) | `analysis/pr-907-contract-review.md` (задача `t_cce41a7d`) |
| `ROS2`| ROS2 (callback safety, N+1, QoS, shutdown) | metadata `t_47d4eddf` (комментарий `analysis/pr-907-review.md` §ROS2) |
| `QA`  | QA-tests (unit coverage, flakiness, scope workflow) | `analysis/pr-907-review.md` §QA (задача `t_6063713a`) |

## Приоритеты

- **blocker** — ломает функциональность/безопасность/релиз; требует фикса до merge.
- **major** — важная проблема; желательно до merge, но обсуждаемо.
- **minor** — стиль/мелкие улучшения/можно отложить в отдельную итерацию.

---

## 🔴 Critical (blockers) — требуют фикса до merge

### BLK-1 — Утечка тела upstream-ответа в лог (Security, BLOCKER)
- **Категория:** security
- **Файл:** `src/rob_box_voice/rob_box_voice/dialogue_node.py:1593-1597`
- **Описание:** `f"🌩️ API error {exc.status_code}: {str(exc.response.text)[:200]} ..."` — тело upstream-ответа логируется без редактирования. Upstream/прокси могут вернуть `Authorization: Bearer ***`, cookies, фрагменты ключей или пользовательские данные → лог-канал становится каналом утечки.
- **Фикс:** не логировать body; оставить `status_code`/`request_id`, либо прогонять текст через централизованный allowlist-редактор, маскирующий `Authorization`, `Cookie` и известные секреты. Добавить тест с echo `Authorization: Bearer ***` и проверкой `caplog`.
- **Источник:** `SEC` (задача `t_0432e282`). Других пересечений нет.

### BLK-2 — `voice_assistant.yaml` ссылается на несуществующий TTS-провайдер `piper` (Architect/Config, BLOCKER)
- **Категория:** architecture
- **Файл:** `docker/vision/config/voice_assistant/voice_assistant.yaml:129`
- **Описание:** `provider: "piper"`, но `tts_node.py:229-233` принимает только `{yandex, silero, minimax}` → при запуске стек падает на старте с `ValueError: provider must be one of: ...`. Кто угодно, кто разворачивает этот конфиг, ловит startup crash.
- **Фикс:** заменить на `provider: "yandex"` (совпадает с `tts_node.py` defaults и голосом "anton").
- **Источник:** `ARC` (F3). Других пересечений нет.

### BLK-3 — `_apply_thinking_policy` теряет новые поля `LLMSettings` (Architect, BLOCKER)
- **Категория:** architecture / backend
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax.py:301-324`
- **Описание:** rebuild `LLMSettings` field-by-field → добавление любого нового поля в `LLMSettings` (`provider.py:155-164`) молча выпадает из MiniMax-ответов. Скрытое нарушение контракта по умолчанию — следующий контрибьютор, добавивший поле, не заметит, что MiniMax его игнорирует.
- **Фикс:** одна строка — `return dataclasses.replace(settings, extra=merged_extra)`.
- **Источник:** `ARC` (F1). Других пересечений нет.

### BLK-4 — `_OpenAICompatibleProvider.complete/stream` итерируют `messages` без freeze (Architect, BLOCKER)
- **Категория:** architecture / backend
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/deepseek.py:211-254, 308-371`
- **Описание:** `Iterable[LLMMessage]` принимает генератор. После первой итерации (в `_require_capability_for_messages`) он пуст → на второй (`_build_kwargs`) в OpenAI SDK уходит `messages=[]` → 400/пустой ответ. Класс багов, который триггерится ровно когда caller передаёт generator (например, для стриминга истории).
- **Фикс:** `messages = tuple(messages)` в начале `complete()` и `stream()`. O(n) памяти, раз и навсегда убирает класс багов.
- **Источник:** `ARC` (F2). Других пересечений нет.

### BLK-5 — HTTP-таймауты: один float на все фазы (Backend, BLOCKER)
- **Категория:** backend
- **Файлы:**
  - `src/rob_box_llm/rob_box_llm/providers/minimax.py:230-237` (LLM, через OpenAI SDK)
  - `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:573-581` (TTS, прямой httpx)
  - `src/rob_box_llm/rob_box_llm/tts_provider_base.py:241-252` (фабрика клиента)
- **Описание:** `timeout: float` ⇒ connect/read/write/pool = одинаковые 30 секунд. При зависании DNS/TLS на connect-фазе пользователь голосового пайплайна ждёт 30 с на каждый вызов — типичный anti-pattern для latency-sensitive путей.
- **Фикс:** принимать `httpx.Timeout` или кортеж `(connect, read, write, pool)`. Разумный дефолт для MiniMax: `httpx.Timeout(connect=5.0, read=20.0, write=10.0, pool=5.0)`. OpenAI SDK принимает любой из `float`/`Timeout`.
- **Источник:** `BE` (блокер #1). Других пересечений нет. Готовые патчи — в `analysis/pr-907-http-client-review.md` §Fix 1.

### BLK-6 — Нет `Limits(max_content_size=...)` в HTTP-клиентах (Backend, BLOCKER)
- **Категория:** backend
- **Файлы:**
  - `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:581`
  - `src/rob_box_llm/rob_box_llm/tts_provider_base.py:252`
- **Описание:** клиент готов съесть ответ любого размера. LLM: 100 МБ JSON → OOM. TTS: hex-encoded audio для длинного текста → десятки МБ. Дефолт пула 100/100 для единичного провайдера избыточен.
- **Фикс:** задать `httpx.Limits(max_connections=10, max_keepalive_connections=5, max_content_size=50*1024*1024)` при создании клиента.
- **Источник:** `BE` (блокер #2). Других пересечений нет.

### BLK-7 — `aclose` LLM-провайдера не идемпотентен (Backend, BLOCKER)
- **Категория:** backend
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/deepseek.py:372-373`
- **Описание:** `await self._client.close()` без `is_closed` guard. При повторном `aclose` (или если caller уже закрыл клиент) → `RuntimeError: Client has not been opened`.
- **Фикс:**
  ```python
  async def aclose(self) -> None:
      if not self._client.is_closed:
          await self._client.close()
  ```
  TTS-провайдер уже идемпотентен (`minimax_tts.py:913-920`).
- **Источник:** `BE` (блокер #3). Других пересечений нет.

### BLK-8 — Гонка shutdown с lazy-инициализацией MiniMax-провайдера в ROS2-ноде (ROS2, BLOCKER; пересечение с ARC F10)
- **Категория:** ros2 / architecture
- **Файл:** `src/rob_box_voice/rob_box_voice/tts_node.py` (блок `_synthesize_minimax_async`, ~1140-1180)
- **Описание:** провайдер создаётся при первом вызове (lazy init). Параллельный shutdown ноды может обратиться к частично инициализированному провайдеру → `AttributeError` / use-after-init. Дубль lazy-init также отмечен архитектором.
- **Фикс:** вынести lazy-init в отдельный `_ensure_minimax_provider()` с `threading.Lock`/однократной инициализацией; в `shutdown()` сначала проверять `is_initialized`, затем `await provider.aclose()` с `try/finally`.
- **Источник:** `ROS2` (BLK-R1) **и** `ARC` (F10). Дедуплицировано — единая запись.

### BLK-9 — Неограниченный fan-out daemon threads (ROS2, BLOCKER)
- **Категория:** ros2
- **Файл:** поискать в `src/rob_box_voice/rob_box_voice/` вокруг `tts_node`/`dialogue_node` любые `threading.Thread(target=..., daemon=True)` без `Semaphore`/очереди.
- **Описание:** порождение thread-per-request без лимита → при всплеске нагрузки количество потоков не ограничено, GIL contention, возможен OOM по стекам.
- **Фикс:** заменить на bounded `ThreadPoolExecutor` (или asyncio-путь), либо явно ограничить семафором. Задокументировать ожидаемую concurrency.
- **Источник:** `ROS2` (BLK-R2). N+1 в затронутом пути **не подтверждён** — из scope ROS2-ревью не вылился в блокер. Других пересечений нет.

---

## 🟠 Major — желательно до merge, но обсуждаемо

### MAJ-1 — Fail-fast отсутствует: `tts_node` стартует с пустыми MiniMax credentials (Security, MAJOR)
- **Категория:** security / backend
- **Файл:** `src/rob_box_voice/rob_box_voice/tts_node.py:386-392` (отложенный отказ до 1140-1145)
- **Описание:** при `provider=minimax` отсутствие `MINIMAX_API_KEY`/`MINIMAX_GROUP_ID` логируется как warning, нода проходит `__init__` и успешно объявляет готовность. Реальный отказ происходит на первом synthesize-вызове в проде — нарушение fail-fast. Система функционально работает (для других провайдеров), но MiniMax-конфигурация деградирует до "первый запрос упадёт".
- **Фикс:** при `provider == "minimax"` валидировать оба env в `__init__`/startup, бросать конфигурационное исключение до объявления готовности. Добавить отдельный тест запуска без каждого обязательного env.
- **Источник:** `SEC`. **Почему major, а не blocker:** при отказе MiniMax-credentials система не падает целиком — другие TTS-провайдеры (yandex, silero) продолжают работать, нода озвучивает ошибку явно. Это operational hazard, не security-уязвимость.

### MAJ-2 — Retry в `tts_node` без jitter и без учёта `Retry-After` (Backend, MAJOR)
- **Категория:** backend
- **Файл:** `src/rob_box_voice/rob_box_voice/tts_node.py:1201-1258` (особенно ~1247)
- **Описание:** exponential backoff есть, но **нет jitter** → при одновременном отказе у нескольких клиентов они все ретраят в один момент (thundering herd). Также **не учитывается `Retry-After`** от MiniMax для 429 — игнорирование этого заголовка быстро ведёт к бану IP.
- **Фикс:**
  ```python
  import random
  delay = (backoff_ms / 1000.0) * (2 ** attempt)
  jittered = delay * (0.5 + random.random())  # ±50%
  await asyncio.sleep(jittered)
  ```
  Плюс отдельная ветка для 429: если в `exc` доступен `Retry-After`, использовать его вместо backoff.
- **Источник:** `BE` (warning #5-6). Других пересечений нет. Готовый патч — `analysis/pr-907-http-client-review.md` §Fix 3.

### MAJ-3 — Нет `asyncio.CancelledError` cleanup в провайдерах (Backend, MAJOR)
- **Категория:** backend
- **Файлы:** `src/rob_box_llm/rob_box_llm/providers/minimax.py` (synthesize/stream), `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:731-763, 765-911` (synthesize/stream)
- **Описание:** OpenAI SDK и httpx поддерживают cancellation "из коробки", но если ROS-нода завершается с активным `await self.minimax_provider.synthesize(...)`, `CancelledError` пробросится без cleanup → `aclose()` не вызовется, соединения повиснут.
- **Фикс:**
  ```python
  async def synthesize(self, text, *, settings=None):
      try:
          data = await self._post(payload)
      except asyncio.CancelledError:
          await self.aclose()  # best-effort cleanup
          raise
  ```
- **Источник:** `BE` (warning #7). Пересечение с `ROS2` (shutdown-race, BLK-8) — связаны, но разные: BLK-8 = race на init, MAJ-3 = cleanup при cancellation.

---

## 🟡 Minor — можно отложить в отдельную итерацию/follow-up PR

### MIN-1 — `APIConnectionError` → `TimeoutError` семантически нечестно (Backend, MINOR)
- **Категория:** backend
- **Файлы:** `src/rob_box_llm/rob_box_llm/providers/deepseek.py:139-161`, `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:96-135`
- **Описание:** OpenAI/httpx `APIConnectionError`/`ConnectError`/`NetworkError` маппятся в `TimeoutError`. Семантически это не таймаут, а ECONNREFUSED/DNS/TLS/reset. Теряется тип проблемы, что усложняет диагностику. Сейчас строка ошибки содержит `str(exc)`, но не имя класса.
- **Фикс:** создать `ConnectionError(ProviderError)` / `TTSConnectionError` параллельно `TimeoutError` (по ADR-0005 уже обсуждалось), либо сохранять `exc.__class__.__name__` в строке ошибки.
- **Источник:** `BE` (warning #1, #3). Других пересечений нет.

### MIN-2 — `_BUILTIN_VOICES` dead code (Architect, MINOR)
- **Категория:** architecture
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:312-355`
- **Описание:** модульная константа `_BUILTIN_VOICES` нигде не используется в runtime. Runtime-функция `list_voices()` (`minimax_tts.py:495-547`) определяет свой inline-список. Docstring в строке 374 утверждает, что `_BUILTIN_VOICES` — источник истины; это неверно.
- **Фикс:** удалить `_BUILTIN_VOICES` (44 строки дублирующего каталога) и его docstring-ссылки; переименовать inline-список в `_LOCAL_CATALOGUE` (или наоборот — точечно использовать константу, удалив inline).
- **Источник:** `ARC` (F4). Других пересечений нет.

### MIN-3 — `_build_request_payload` hardcode `stream=False` (Architect, MINOR)
- **Категория:** architecture
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:610-616`
- **Описание:** mandatory override hook не различает streaming/non-streaming. `synthesize`/`stream` обходят hook (`minimax_tts.py:743, 795`) и вызывают `_build_payload` напрямую с правильным флагом. Hook становится test-only, что противоречит docstring.
- **Фикс:** расширить hook до `_build_request_payload(text, settings, voice_meta, *, stream: bool)`; `synthesize`/`stream` пробрасывают `stream`.
- **Источник:** `ARC` (F5). Других пересечений нет.

### MIN-4 — `_validate_image_bytes` не покрывает URL/`data:` источники (Architect, MINOR)
- **Категория:** backend
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax.py:130-148`
- **Описание:** cap 10 MB применяется только к `bytes`. URL/`data:image/...;base64,...` (50 MB+) проходят без проверки. MiniMax отвергнет серверной стороной → caller увидит generic 400 вместо typed `CapabilityUnavailableError`.
- **Фикс:** документировать bypass ИЛИ добавить `HEAD + Content-Length` для URL ИЛИ извлекать размер из base64. Самый дешёвый — последний.
- **Источник:** `ARC` (F6). Других пересечений нет.

### MIN-5 — Нет общего retry-декоратора для LLM-провайдеров (Backend, MINOR)
- **Категория:** backend
- **Описание:** `_map_exception` в LLM-провайдерах (`deepseek.py:139`) сразу пробрасывает исключения. Retry-инфраструктуры в `rob_box_llm` нет; caller ответственен за retry. Если retry добавят — нужно учитывать идемпотентность (LLM c tools НЕ идемпотентен: повторный вызов может исполнить tool дважды, если сервер принял запрос, но ответ не дошёл).
- **Фикс:** добавить лёгкий `with_retry(exceptions=(RateLimitError, TimeoutError), max_attempts=3, base=0.5, jitter=True)` и применить в `MiniMaxProvider`/`DeepSeekProvider`/`MiMoProvider`. Задокументировать, что tools=True вызовы — не идемпотентны, retry-политика должна это учитывать.
- **Источник:** `BE` (warning #8). Других пересечений нет.

### MIN-6 — Sleep в тесте SSE (QA, MINOR)
- **Категория:** qa
- **Файл:** `src/rob_box_llm/test/test_minimax_tts_streaming.py:73-91`
- **Описание:** тестовый SSE-generator содержит реальный `await asyncio.sleep()` между чанками. Делает тест time-based/flaky, противоречит требованию "без sleep".
- **Фикс:** заменить на детерминированный async iterator/transport без задержки; latency-тест вынести в marker `slow`/`integration`.
- **Источник:** `QA` (MUST-1, понижено до MINOR — функционально работает, но flaky). Других пересечений нет.

### MIN-7 — Voice/ROS тесты не изолированы от numpy (QA, MINOR)
- **Категория:** qa
- **Файл:** `src/rob_box_voice/test/unit/tts/test_minimax_integration.py`
- **Описание:** на чистом окружении без `numpy` collection падает с `ModuleNotFoundError`. Workflow `G-TTS-Provider-Tests.yml` запускает только `src/rob_box_llm` → ROS/voice-тесты этим gate не покрыты.
- **Фикс:** добавить `pytest.importorskip("numpy")` или явную dev-зависимость в `requirements*.txt` для voice-пакета; либо skip с понятным marker.
- **Источник:** `QA` (MUST-2, понижено до MINOR — это проблема CI-конфигурации, а не runtime-дефект). Других пересечений нет.

### MIN-8 — Дубликат `if __name__ == "__main__"` (QA, MINOR)
- **Категория:** qa
- **Файл:** `tools/audio_capture_harness/test_audio_capture_harness.py:559-568`
- **Описание:** два одинаковых блока `if __name__ == "__main__": unittest.main(...)`. Функционально не ломает pytest, но снижает поддерживаемость.
- **Фикс:** удалить дубликат.
- **Источник:** `QA` (SHOULD-5, понижено до MINOR). Других пересечений нет.

### MIN-9 — Workflow glob избыточен (QA, MINOR)
- **Категория:** qa
- **Файл:** `.github/workflows/G-TTS-Provider-Tests.yml:39`
- **Описание:** `'src/rob_box_llm/**.py'` рядом с более конкретными glob'ами — вводит в заблуждение, не покрывает рекурсивно.
- **Фикс:** привести к одному явному рекурсивному шаблону `src/rob_box_llm/**/*.py`.
- **Источник:** `QA` (SHOULD-3). Других пересечений нет.

### MIN-10 — Coverage scope (QA, MINOR)
- **Категория:** qa
- **Описание:** gate измеряет только `minimax_tts.py` (100%, threshold 85%); `minimax.py` и ROS-интеграция не покрыты.
- **Фикс:** добавить отдельные coverage targets или документировать границы job в README workflow.
- **Источник:** `QA` (COULD-8). Других пересечений нет.

### MIN-11 — `conftest` fixture `minimax_provider` не закрывает `httpx.AsyncClient` (QA, MINOR)
- **Категория:** qa
- **Файл:** `src/rob_box_llm/test/conftest.py` (fixture `minimax_provider`)
- **Описание:** клиент создаётся без `aclose()` в teardown → потенциальный ресурсный leak при xdist/длинных прогонах. Сейчас при текущем размере suite предупреждение не проявилось.
- **Фикс:** добавить `yield` и `await client.aclose()` либо использовать MockTransport/клиент из HTTP-fixture.
- **Источник:** `QA` (COULD-7). Других пересечений нет.

### MIN-12 — Docs drift: MiMo-7B vs `mimo-v2.5-pro` (Architect, MINOR)
- **Категория:** documentation
- **Файлы:** `docs/guides/examples/minimax_llm.yaml:60`, `docs/guides/MINIMAX.md:46`, `src/rob_box_llm/README.md:25`
- **Описание:** документация говорит `MiMo-7B`, код (`providers/mimo.py:27`) и live config (`src/rob_box_voice/config/voice_assistant.yaml:118`) — `mimo-v2.5-pro`. Code is source of truth.
- **Фикс:** обновить три doc-файла до `mimo-v2.5-pro`.
- **Источник:** `ARC` (F9). Других пересечений нет.

### MIN-13 — Тест `_raise_for_base_resp` для LLM не найден (Backend, MINOR)
- **Категория:** backend / qa
- **Описание:** для LLM-`_raise_for_base_resp` (`minimax.py:159-183`) не нашлось прямого параметризованного теста. Покрытие косвенное через `_post_process_response`. Стоит добавить явный `test_minimax_raise_for_base_resp.py` с кейсами `status_code != 0 → raises`.
- **Фикс:** добавить тест.
- **Источник:** `BE` (info #9). Других пересечений нет.

### MIN-14 — `TTSChunk(finish_reason="error")` после `yielded_audio=True` (Backend, MINOR)
- **Категория:** backend
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:856-858`
- **Описание:** если SSE-цикл получил error envelope с `yielded_audio=True`, возвращается `TTSChunk(finish_reason="error")` без `raise`. Caller в `tts_node.py:1280` ловит это как `raise Exception("MiniMax stream reported error: finish_reason=error")` → двойная обёртка.
- **Фикс:** привести к единому формату — либо всегда `raise` после yield, либо всегда terminal-chunk.
- **Источник:** `BE` (info #10). Других пересечений нет.

---

## 🟢 Positives — что сделано хорошо

Собрано из одобрительных комментариев всех 5 ревью. Не дублируется (каждый пункт — отдельная техника, не повторяющаяся в нескольких ревью):

1. **Secrets hygiene — образцовая** (`SEC`, `ARC`, `BE`). `MINIMAX_API_KEY`/`MINIMAX_GROUP_ID` берутся только из ENV, редактируются в 3 слоях (`_redact_sensitive_text`, `_RedactGroupIdFilter`, `MiniMaxRedactedLogFilter`). 11 dedicated leak-guard тестов покрывают каждый путь утечки.

2. **Контракты звучат** (`ARC`). `LLMProvider`/`TTSProvider`/`BaseTTSProvider` — чистое разделение. Отдельные `ProviderError`/`TTSError` иерархии. Multimodal: `MessageContent = Union[str, tuple[MessagePart, ...]]` — text-only callers не сломаны (357/357 старых тестов проходят). Capability defaults консервативные.

3. **Capability-интроспекция честная** (`ARC`). `ProviderCapabilities` per-model на `MiniMaxProvider` (`minimax.py:244-261`). Vision-капабилити гейтится ДО сетевого вызова в `_require_capability_for_messages` → fail-fast с типизированным `CapabilityUnavailableError`.

4. **`_map_exception` централизован** (`ARC`, `BE`). LLM — `deepseek.py:139`, TTS — `minimax_tts.py:96`, MiniMax envelope — `minimax.py:159`. Единая иерархия ошибок, одинаковый `provider=` kwarg, никаких `openai.APIError`/`httpx` наружу.

5. **Параметризованные тесты сильные** (`BE`, `QA`). `test_minimax_tts_errors_parametrized.py:312-405` покрывает 5 транспортных классов (`ConnectTimeout`/`ReadTimeout`/`PoolTimeout`/`ConnectError`/`NetworkError`) + `asyncio.TimeoutError`. LLM TTS suite: 244 passed, 113 deselected, coverage `minimax_tts.py` — 100%.

6. **`FakeLLMProvider`/`FakeTTSProvider` детерминированные** (`ARC`). Реализуют контракты полностью, record calls. Тесты пакета идут без сети.

7. **OpenAI-compatible wire format переиспользуется** (`ARC`). `_OpenAICompatibleProvider` — общая база; `DeepSeekProvider`/`MiMoProvider` это 25-строчные обёртки; `MiniMaxProvider` добавляет ~80 строк для envelope/pre-flight/thinking-policy. Будущие OpenAI-compat провайдеры = одна строка subclass.

8. **TTS container honesty** (`ARC`). `synthesize` (`minimax_tts.py:752`) сообщает реальный формат контейнера на `TTSAudio.format` (OGG-запрос → MP3-ответ). Транскодер не работает по лжи.

9. **TTL-safe `aclose()` для TTS** (`ARC`, `BE`). `minimax_tts.py:913-920` идемпотентен, закрывает только owned clients; caller-owned (с MockTransport) уважаются. LLM-провайдер в этом **непоследователен** — это и есть BLK-7.

10. **Async-пути чистые** (`BE`). Поиск `time.sleep|requests\.|urllib\.|open\(` в `minimax.py`/`minimax_tts.py`/`deepseek.py` — пусто. Никаких блокирующих I/O в async.

11. **Сетевые ошибки обрабатываются честно** (`BE`). Никаких `except: pass`. Все сетевые ошибки маппятся в типизированные доменные.

12. **Контракт ↔ реализация ↔ ROS-consumer ↔ yaml-config matrix проверена** (`ARC`). 102/102 conformance tests зелёные. yaml-config согласован с реализацией (после фикса BLK-2).

---

## ⚖️ Open architectural questions / trade-offs

Эти пункты сознательно оставлены нерешёнными; требуют владельца PR или архитектурного решения.

### OPEN-1 — Retry-policy для LLM с `tools=True` (требует решения по идемпотентности)
- **Конфликт:** `BE` (MIN-5) предлагает добавить общий `with_retry` для LLM-провайдеров; `ARC` (implicit через иерархию ошибок) подразумевает, что retry — caller-side. LLM c tools НЕ идемпотентен.
- **Trade-off:** retry в провайдере = удобство для caller, но риск двойного исполнения tool. Retry в caller = явная ответственность, требует дисциплины.
- **Предложение:** оставить retry-инфраструктуру **caller-side**, документировать идемпотентность-нюанс в docstring каждого LLM-провайдера; **не** добавлять retry в `rob_box_llm` без явного решения. См. ADR-0005.
- **Эскалация:** нет (задокументировать достаточно).

### OPEN-2 — TTS streaming latency: SSE vs WebSocket (deferred per ADR)
- **Конфликт:** `ARC` (info §2.3) отмечает, что `MiniMaxTTSProvider.stream()` возвращает максимум 2 чанка (audio + terminal "stop"), не multi-frame. Это осознанно задокументировано (`tts.py:160-166`, `minimax_tts.py:771-781`).
- **Trade-off:** SSE = сейчас работает, latency TTFA ~159ms (из bench 8/8) приемлема. WebSocket = реальный chunk-per-frame, но требует MiniMax API change и нового transport. ADR-0003 §2.4 откладывает на M5/M6.
- **Предложение:** принять SSE-реализацию как есть; зафиксировать в ADR, что multi-frame WebSocket — explicit non-goal для этого PR.
- **Эскалация:** нет.

### OPEN-3 — `BaseTTSProvider` ↔ `TTSProvider`: два слоя, один consumer (Architect flagged F12)
- **Конфликт:** `ARC` (F12) отмечает, что `TTSProviderRegistry` / `TTSProviderFactory` экспортируются, но `tts_node` всё ещё создаёт `MiniMaxTTSProvider` напрямую (`tts_node.py:1149, 1377`). Это intentional для этого PR (миграция = P1+).
- **Trade-off:** extension architecture сейчас — "pure addition with no consumer", что снижает ценность PR. Но миграция `tts_node` на registry — отдельная задача, ломать composability не нужно.
- **Предложение:** принять как есть, добавить issue/todo на миграцию `tts_node` → `TTSProviderFactory.create()` в post-merge.
- **Эскалация:** нет (задокументировать достаточно).

### OPEN-4 — `APIConnectionError` vs `TimeoutError` (BE MIN-1)
- **Конфликт:** текущий маппинг "всё в `TimeoutError`" — осознанный trade-off "retry-or-not важнее диагностики" (см. комментарий в `test_minimax_tts_errors_parametrized.py`). Если ввести отдельный `ConnectionError`/`TTSConnectionError`, нужно обновить тесты и caller-политику.
- **Trade-off:** отдельный класс → точнее retry-политика (не ретраить, например, TLS-handshake failure) и точнее дашборды. Но это breaking change для всех caller'ов, которые ловят `TimeoutError`.
- **Предложение:** в этом PR не менять; запланировать мажорное обновление иерархии ошибок с явным deprecation period.
- **Эскалация:** нет.

---

## Резюме: что сводный комментарий в PR должен отразить

- **Вердикт:** `REQUEST_CHANGES`.
- **Critical (9 пунктов):** BLK-1…BLK-9 (security, yaml-config, dataclass/tuple-freeze, HTTP timeouts/Limits/aclose, lazy-init race, unbounded threads).
- **Major (3 пункта):** MAJ-1 (MiniMax fail-fast), MAJ-2 (retry jitter+Retry-After), MAJ-3 (cancellation cleanup).
- **Minor (14 пунктов):** MIN-1…MIN-14 (семантика ошибок, dead code, hardcoded params, doc drift, test flakiness, workflow scope, fixtures).
- **Positives (12 пунктов):** secrets hygiene, contracts, capability, exception mapping, parameterised tests, fakes, OpenAI-compat reuse, container honesty, TTL-safe aclose, async-clean, error handling honesty, config matrix.
- **Open (4 пункта):** retry-tools идемпотентность, SSE-vs-WebSocket, dual-TTS-слой, error hierarchy evolution.

Все блокеры и major имеют готовые фиксы (где возможно — в `analysis/pr-907-http-client-review.md` §Fix 1-4, иначе — однострочники).

— pr-reviewer (Hermes Agent), 2026-07-22