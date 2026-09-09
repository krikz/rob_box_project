# PR #907 — Сводный review-комментарий
- **PR:** https://github.com/krikz/rob_box_project/pull/907
- **Head SHA:** `6e75eb392cf2db75bebe740326ae7f2e2bba89fa` (`6e75eb39`)
- **Base SHA:** `91abcbca3b3e6c900bed154aca68ee1541ad981f`
- **Статус PR:** OPEN
- **Дата аудита:** 2026-07-23
- **Reviewer:** `architect` (Hermes Agent)
**Источники:** 5 независимых ревью — Security / Backend (HTTP-клиент MiniMax) / Architect (контракты провайдеров) / ROS2 (callback safety, N+1, QoS, shutdown) / QA (unit coverage, flakiness, scope workflow). Структурированный JSON-свод (этот комментарий) сгенерирован `architect` и собран на основе:
- `analysis/pr-907-deduplicated.md` (дедуплицированный список всех замечаний)
- `analysis/pr-907-architectural-conflicts.md` (межревьюерные trade-offs)
- `analysis/pr-907-final-review.md`, `pr-907-contract-review.md`, `pr-907-http-client-review.md`, `pr-907-review.md` (исходные ревью)

## TL;DR
**Вердикт: `REQUEST_CHANGES`.**

Обнаружено **9 blockers**, **3 major**, **14 minor**. 7 positives. Все blockers и major имеют готовые фиксы. Архитектурных конфликтов между ревью не выявлено — секция опущена.

> **Рекомендуемое состояние ревью:** `REQUEST_CHANGES`.

**Security-priority по OWASP:** A04 (Insecure Design), A05 (Misconfiguration), A08 (Integrity), A09 (Logging)
**Primary security blocker:** BLK-1 — прямое нарушение A09 (логирование upstream response body без редактирования)

## Critical (blockers) — требуют фикса до merge
### BLK-1 — Утечка тела upstream-ответа в лог (Authorization/Cookie/секреты)
- **Файл:** `src/rob_box_voice/rob_box_voice/dialogue_node.py:L1593-L1597`
- **OWASP:** A09:2021
- **Описание:** f-строка лога включает str(exc.response.text)[:200] без редактирования. Upstream/прокси могут вернуть Authorization: Bearer *** cookies или фрагменты ключей → лог-канал становится каналом утечки. Это прямое нарушение OWASP A09 (Security Logging & Monitoring Failures).
- **Предлагаемый фикс:** Не логировать body ответа. Оставить только status_code и request_id, либо прогонять текст через централизованный allowlist-редактор (_redact_sensitive_text или новый _RedactUpstreamBody), маскирующий Authorization, Cookie и известные секреты. Добавить тест с echo 'Authorization: Bearer ***' и проверкой caplog.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_voice/rob_box_voice/dialogue_node.py#L1593-L1597
### BLK-2 — voice_assistant.yaml ссылается на несуществующий TTS-провайдер piper
- **Файл:** `docker/vision/config/voice_assistant/voice_assistant.yaml:L129`
- **OWASP:** A04:2021
- **Описание:** provider: 'piper', но tts_node.py:229-233 принимает только {yandex, silero, minimax} → при запуске стек падает на старте с ValueError: provider must be one of: ...
- **Предлагаемый фикс:** Заменить на provider: 'yandex' (совпадает с tts_node.py defaults и голосом 'anton').
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/docker/vision/config/voice_assistant/voice_assistant.yaml#L129
### BLK-3 — _apply_thinking_policy теряет новые поля LLMSettings
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax.py:L301-L324`
- **OWASP:** A04:2021
- **Описание:** rebuild LLMSettings field-by-field → добавление любого нового поля в LLMSettings (provider.py:155-164) молча выпадает из MiniMax-ответов. Скрытое нарушение контракта по умолчанию.
- **Предлагаемый фикс:** Одна строка — return dataclasses.replace(settings, extra=merged_extra).
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax.py#L301-L324
### BLK-4 — _OpenAICompatibleProvider.complete/stream итерируют messages без freeze
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/deepseek.py:L211-L254`
- **OWASP:** A04:2021
- **Описание:** Iterable[LLMMessage] принимает генератор. После первой итерации (в _require_capability_for_messages) он пуст → на второй (_build_kwargs) в OpenAI SDK уходит messages=[] → 400/пустой ответ.
- **Предлагаемый фикс:** messages = tuple(messages) в начале complete() и stream() (deepseek.py:308-371 — аналогично). O(n) памяти, раз и навсегда убирает класс багов.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/deepseek.py#L211-L254
### BLK-5 — HTTP-таймауты: один float на все фазы (connect/read/write/pool)
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax.py + src/rob_box_llm/rob_box_llm/providers/minimax_tts.py + src/rob_box_llm/rob_box_llm/tts_provider_base.py:L230`
- **OWASP:** A08:2021
- **Описание:** timeout: float ⇒ connect/read/write/pool = одинаковые 30 секунд. При зависании DNS/TLS на connect-фазе пользователь голосового пайплайна ждёт 30 с на каждый вызов. Anti-pattern для latency-sensitive путей.
- **Предлагаемый фикс:** Принимать httpx.Timeout или кортеж (connect, read, write, pool). Дефолт: httpx.Timeout(connect=5.0, read=20.0, write=10.0, pool=5.0).
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax.py#L230-L237
### BLK-6 — Нет Limits(max_content_size=...) и нет max_connections в HTTP-клиентах
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py + src/rob_box_llm/rob_box_llm/tts_provider_base.py:L581`
- **OWASP:** A08:2021
- **Описание:** Клиент готов съесть ответ любого размера. LLM: 100 МБ JSON → OOM. TTS: hex-encoded audio для длинного текста → десятки МБ. Дефолт пула 100/100 для единичного провайдера избыточен.
- **Предлагаемый фикс:** Задать httpx.Limits(max_connections=10, max_keepalive_connections=5, max_content_size=50*1024*1024) при создании клиента.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax_tts.py#L581
### BLK-7 — aclose LLM-провайдера не идемпотентен
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/deepseek.py:L372-L373`
- **OWASP:** A04:2021
- **Описание:** await self._client.close() без is_closed guard. При повторном aclose → RuntimeError: Client has not been opened. TTS-провайдер уже идемпотентен (minimax_tts.py:913-920).
- **Предлагаемый фикс:** async def aclose(self) -> None: if not self._client.is_closed: await self._client.close()
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/deepseek.py#L372-L373
### BLK-8 — Гонка shutdown с lazy-инициализацией MiniMax-провайдера в ROS2-ноде
- **Файл:** `src/rob_box_voice/rob_box_voice/tts_node.py:LNone`
- **OWASP:** A04:2021
- **Описание:** Провайдер создаётся при первом вызове (lazy init). Параллельный shutdown ноды может обратиться к частично инициализированному провайдеру → AttributeError / use-after-init. ROS2-ревью указало диапазон ~1140-1180 (блок _synthesize_minimax_async), но точные строки lazy-init / shutdown не верифицированы в этом прогоне.
- **Предлагаемый фикс:** Вынести lazy-init в отдельный _ensure_minimax_provider() с threading.Lock / однократной инициализацией; в shutdown() сначала проверять is_initialized, затем await provider.aclose() с try/finally.
- **⚠️ needs_more_info:** точные номера строк lazy-init и shutdown-блока в tts_node.py — требуют ручной верификации против PR diff (указано только ~1140-1180)
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_voice/rob_box_voice/tts_node.py
### BLK-9 — Неограниченный fan-out daemon threads в ROS2-нодах
- **OWASP:** A04:2021
- **Описание:** ROS2-ревью указало риск: порождение thread-per-request без лимита (threading.Thread(target=..., daemon=True) без Semaphore/очереди) в src/rob_box_voice/rob_box_voice/ вокруг tts_node/dialogue_node. Конкретные file:line не верифицированы — это требует grep по PR diff.
- **Предлагаемый фикс:** Заменить на bounded ThreadPoolExecutor (или asyncio-путь), либо явно ограничить семафором. Задокументировать ожидаемую concurrency.
- **⚠️ needs_more_info:** file:line не указаны в исходных ревью — требуется ручной поиск 'threading.Thread(target=..., daemon=True)' в PR diff
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_voice/rob_box_voice/

## Major — желательно закрыть в этом PR
### MAJ-1 — Fail-fast отсутствует: tts_node стартует с пустыми MiniMax credentials
- **Файл:** `src/rob_box_voice/rob_box_voice/tts_node.py:L386-L392`
- **OWASP:** A04:2021
- **Описание:** При provider=minimax отсутствие MINIMAX_API_KEY/MINIMAX_GROUP_ID логируется как warning, нода проходит __init__ и успешно объявляет готовность. Реальный отказ — на первом synthesize-вызове.
- **Предлагаемый фикс:** При provider == 'minimax' валидировать оба env в __init__/startup, бросать конфигурационное исключение до объявления готовности. Добавить тест запуска без каждого обязательного env.
- **Почему не блокер:** Система не падает целиком — другие TTS-провайдеры (yandex, silero) продолжают работать. Это operational hazard, не security-уязвимость. Однако при policy 'all major fix before merge' фактически блокирует.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_voice/rob_box_voice/tts_node.py#L386-L392
### MAJ-2 — Retry в tts_node без jitter и без учёта Retry-After (thundering herd + IP ban)
- **Файл:** `src/rob_box_voice/rob_box_voice/tts_node.py:L1201-L1258`
- **OWASP:** A08:2021
- **Описание:** Exponential backoff есть, но нет jitter → thundering herd. Retry не учитывает Retry-After от MiniMax для 429 → риск бана IP.
- **Предлагаемый фикс:** delay = (backoff_ms / 1000.0) * (2 ** attempt); jittered = delay * (0.5 + random.random())  # ±50%; await asyncio.sleep(jittered). Отдельная ветка для 429: если в exc доступен Retry-After — использовать его вместо backoff.
- **Почему не блокер:** Проявляется только под нагрузкой / при частичных отказах MiniMax; не ломает cold-start, но ухудшает поведение системы при первом же инциденте.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_voice/rob_box_voice/tts_node.py#L1201-L1258
### MAJ-3 — Нет asyncio.CancelledError cleanup в провайдерах (висящие соединения при shutdown)
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax.py + src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:L731`
- **OWASP:** A04:2021
- **Описание:** OpenAI SDK и httpx поддерживают cancellation, но если ROS-нода завершается с активным await self.minimax_provider.synthesize(...), CancelledError пробросится без cleanup → aclose() не вызовется, соединения повиснут.
- **Предлагаемый фикс:** async def synthesize(self, text, *, settings=None): try: data = await self._post(payload) except asyncio.CancelledError: await self.aclose(); raise
- **Почему не блокер:** Проявляется только при нештатном shutdown ROS-ноды с активным запросом. Не блокирует happy path.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax_tts.py#L731-L763

## Minor — можно отложить в отдельную итерацию
### MIN-1 — APIConnectionError → TimeoutError семантически нечестно
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/deepseek.py + src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:L139`
- **Описание:** OpenAI/httpx APIConnectionError/ConnectError/NetworkError маппятся в TimeoutError. Семантически это не таймаут, а ECONNREFUSED/DNS/TLS/reset. Теряется тип проблемы.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/deepseek.py#L139-L161
### MIN-2 — _BUILTIN_VOICES dead code в minimax_tts.py
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:L312-L355`
- **Описание:** Модульная константа _BUILTIN_VOICES нигде не используется в runtime. Runtime list_voices() (line 495-547) определяет свой inline-список. Docstring в строке 374 утверждает, что _BUILTIN_VOICES — источник истины; это неверно.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax_tts.py#L312-L355
### MIN-3 — _build_request_payload hardcode stream=False
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:L610-L616`
- **Описание:** Mandatory override hook не различает streaming/non-streaming. synthesize/stream обходят hook (minimax_tts.py:743, 795) и вызывают _build_payload напрямую с правильным флагом. Hook становится test-only.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax_tts.py#L610-L616
### MIN-4 — _validate_image_bytes не покрывает URL/data: источники (size cap bypass)
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax.py:L130-L148`
- **Описание:** Cap 10 MB применяется только к bytes. URL/data:image/...;base64,... (50 MB+) проходят без проверки. MiniMax отвергнет серверной стороной → caller увидит generic 400. Слабо связано с OWASP A10 (SSRF) — URL без Content-Length pre-check.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax.py#L130-L148
### MIN-5 — Нет общего retry-декоратора для LLM-провайдеров
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/deepseek.py:L139`
- **Описание:** Retry-инфраструктуры в rob_box_llm нет; caller ответственен. Если retry добавят — нужно учитывать идемпотентность (LLM c tools НЕ идемпотентен). См. ADR-0005 и OPEN-1 в architectural-conflicts.md.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/deepseek.py#L139
### MIN-6 — Sleep в тесте SSE (time-based flaky test)
- **Файл:** `src/rob_box_llm/test/test_minimax_tts_streaming.py:L73-L91`
- **Описание:** Тестовый SSE-generator содержит реальный await asyncio.sleep() между чанками. Делает тест time-based/flaky, противоречит требованию 'без sleep'.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/test/test_minimax_tts_streaming.py#L73-L91
### MIN-7 — Voice/ROS тесты не изолированы от numpy (collection failure)
- **Файл:** `src/rob_box_voice/test/unit/tts/test_minimax_integration.py`
- **Описание:** На чистом окружении без numpy collection падает с ModuleNotFoundError. Workflow G-TTS-Provider-Tests.yml запускает только src/rob_box_llm → ROS/voice-тесты этим gate не покрыты.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_voice/test/unit/tts/test_minimax_integration.py
### MIN-8 — Дубликат if __name__ == '__main__' в test_audio_capture_harness
- **Файл:** `tools/audio_capture_harness/test_audio_capture_harness.py:L559-L568`
- **Описание:** Два одинаковых блока if __name__ == '__main__': unittest.main(...). Функционально не ломает pytest, но снижает поддерживаемость.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/tools/audio_capture_harness/test_audio_capture_harness.py#L559-L568
### MIN-9 — Workflow glob избыточен в G-TTS-Provider-Tests.yml
- **Файл:** `.github/workflows/G-TTS-Provider-Tests.yml:L39`
- **Описание:** 'src/rob_box_llm/**.py' рядом с более конкретными glob'ами — вводит в заблуждение, не покрывает рекурсивно.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/.github/workflows/G-TTS-Provider-Tests.yml#L39
### MIN-10 — Coverage scope ограничен только minimax_tts.py
- **Файл:** `.github/workflows/G-TTS-Provider-Tests.yml`
- **Описание:** Gate измеряет только minimax_tts.py (100%, threshold 85%); minimax.py и ROS-интеграция не покрыты.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/.github/workflows/G-TTS-Provider-Tests.yml
### MIN-11 — conftest fixture minimax_provider не закрывает httpx.AsyncClient
- **Файл:** `src/rob_box_llm/test/conftest.py`
- **Описание:** Клиент создаётся без aclose() в teardown → потенциальный ресурсный leak при xdist/длинных прогонах.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/test/conftest.py
### MIN-12 — Docs drift: MiMo-7B vs mimo-v2.5-pro
- **Файл:** `docs/guides/examples/minimax_llm.yaml + docs/guides/MINIMAX.md + src/rob_box_llm/README.md`
- **Описание:** Документация говорит MiMo-7B, код (providers/mimo.py:27) и live config (src/rob_box_voice/config/voice_assistant.yaml:118) — mimo-v2.5-pro. Code is source of truth.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/docs/guides/examples/minimax_llm.yaml
### MIN-13 — Тест _raise_for_base_resp для LLM не найден
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax.py:L159-L183`
- **Описание:** Для LLM-_raise_for_base_resp не нашлось прямого параметризованного теста. Покрытие косвенное через _post_process_response.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax.py#L159-L183
### MIN-14 — TTSChunk(finish_reason='error') после yielded_audio=True (двойная обёртка)
- **Файл:** `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:L856-L858`
- **Описание:** Если SSE-цикл получил error envelope с yielded_audio=True, возвращается TTSChunk(finish_reason='error') без raise. Caller в tts_node.py:1280 ловит это как raise Exception → двойная обёртка.
- **Permalink:** https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax_tts.py#L856-L858

## Positives — что сделано хорошо
Secrets hygiene — образцовая. MINIMAX_API_KEY/MINIMAX_GROUP_ID берутся только из ENV, редактируются в 3 слоях (_redact_sensitive_text, _RedactGroupIdFilter, MiniMaxRedactedLogFilter). 11 dedicated leak-guard тестов покрывают каждый путь утечки. (SEC, ARC, BE)

Контракты звучат. LLMProvider/TTSProvider/BaseTTSProvider — чистое разделение. Отдельные ProviderError/TTSError иерархии. Multimodal: MessageContent = Union[str, tuple[MessagePart, ...]] — text-only callers не сломаны (357/357 старых тестов проходят). Capability defaults консервативные. (ARC)

Capability-интроспекция честная. ProviderCapabilities per-model на MiniMaxProvider (minimax.py:244-261). Vision-капабилити гейтится ДО сетевого вызова в _require_capability_for_messages → fail-fast с типизированным CapabilityUnavailableError. (ARC, BE)

_map_exception централизован. LLM — deepseek.py:139, TTS — minimax_tts.py:96, MiniMax envelope — minimax.py:159. Единая иерархия ошибок, одинаковый provider= kwarg, никаких openai.APIError/httpx наружу. (ARC, BE)

FakeLLMProvider/FakeTTSProvider детерминированные. Реализуют контракты полностью, record calls. Тесты пакета идут без сети. (ARC)

OpenAI-compatible wire format переиспользуется. _OpenAICompatibleProvider — общая база; DeepSeekProvider/MiMoProvider это 25-строчные обёртки; MiniMaxProvider добавляет ~80 строк для envelope/pre-flight/thinking-policy. Будущие OpenAI-compat провайдеры = одна строка subclass. (ARC)

Async-пути чистые. Поиск time.sleep|requests.|urllib.|open( в minimax.py/minimax_tts.py/deepseek.py — пусто. Никаких блокирующих I/O в async. (BE)


## Architectural conflicts & escalations

Архитектурных конфликтов между ревью не выявлено — секция опущена.

_В pr-907-architectural-conflicts.md все 4 пункта OPEN-1…OPEN-4 (retry-policy для tools=True, SSE-vs-WebSocket, dual-TTS-layer, error-hierarchy) явно помечены архитектором как deferred без эскалации. Реальных архитектурных конфликтов (где рекомендации ревьюеров взаимоисключают друг друга и требуют примирения) не выявлено. Согласно п.2 ТЗ, секция опущена._


## Требуется ручная верификация (needs_more_info)
- BLK-8: точные строки lazy-init и shutdown-блока в tts_node.py — требуют ручной верификации против PR diff (указано только ~1140-1180)
- BLK-9: file:line не указаны в исходных ревью — требуется ручной поиск 'threading.Thread(target=..., daemon=True)' в PR diff


## Что делать дальше (владельцу PR)

1. Закрыть **9 blockers** (BLK-1, BLK-2, BLK-3, BLK-4, BLK-5, BLK-6, BLK-7, BLK-8, BLK-9). Все имеют готовые фиксы (BLK-8/BLK-9 — сначала верифицировать точные строки по PR diff).
2. Желательно закрыть **3 major** (MAJ-1, MAJ-2, MAJ-3) в этом же PR.
3. Прогнать `PYTHONPATH=src/rob_box_llm:src/rob_box_llm/test pytest src/rob_box_llm/test/ -q` — должно остаться 244+ passed без регрессий.
4. После re-review — переключить вердикт на `APPROVE` и merge.

— architect (Hermes Agent), 2026-07-23
