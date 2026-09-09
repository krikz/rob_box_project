# PR #907 — Architectural & Security Audit (final synthesis)

- **PR:** https://github.com/krikz/rob_box_project/pull/907
- **Head SHA:** `6e75eb392cf2db75bebe740326ae7f2e2bba89fa` (`6e75eb39`)
- **Base:** `91abcbca3b3e6c900bed154aca68ee1541ad981f`
- **Статус PR:** OPEN
- **Дата аудита:** 2026-07-23
- **Reviewer:** `architect` (Hermes Agent)
- **Назначение:** финальный structured JSON-свод по результатам ранее агрегированных
  BE / ARC / SEC / ROS2 / QA ревью (`analysis/pr-907-deduplicated.md`,
  `analysis/pr-907-architectural-conflicts.md`, `analysis/pr-907-final-review.md`).
  Никакого самостоятельного ревью кода PR в этом прогоне не выполнялось —
  только структурирование и security-верификация уже выявленных замечаний.

## Метод

1. По каждому blocker проверено наличие `file:line` + `proposed_fix`. Если нет — помечено `needs_more_info` (без выдумывания).
2. Архитектурные эскалации: согласно `pr-907-architectural-conflicts.md`, все 4 пункта OPEN-1…OPEN-4 помечены архитектором как **deferred**, без отдельной эскалации. Реальных архитектурных конфликтов (где рекомендации ревьюеров взаимоисключают друг друга) **не выявлено** — секция `architectural[]` в JSON опущена согласно п.2 ТЗ.
3. Security-проверка по OWASP Top 10 проведена **только в рамках уже известных замечаний** (без самостоятельного ревью кода PR), согласно п.3 ТЗ. Маппинг ниже.
4. Positives — только из переданных положительных пунктов, без выдумывания.

## OWASP Top 10 — маппинг выявленных замечаний

| OWASP категория | Связанные пункты | Что покрыто | Что НЕ покрыто в рамках этого PR |
|---|---|---|---|
| **A01:2021 Broken Access Control** | — | (не в scope PR — это provider-инфраструктура без auth-логики) | — |
| **A02:2021 Cryptographic Failures** | — | Secrets берутся только из ENV, не из файлов | TLS-конфигурация out of scope (внешний провайдер) |
| **A03:2021 Injection** | — | Все запросы к MiniMax — структурированный JSON через httpx/OpenAI SDK; ни одного SQL/shell/eval | — |
| **A04:2021 Insecure Design** | BLK-3 (provider=piper yaml crash), BLK-8 (lazy-init race), MAJ-1 (отсутствие fail-fast credentials), MAJ-3 (no CancelledError cleanup) | Все 4 — design defects, зафиксированы в ревью | Контрактная матрица соответствия — ок |
| **A05:2021 Security Misconfiguration** | BLK-1 (upstream body в логе), MAJ-1 (env не валидируется) | Default-конфигурация валидна; misconfigurations локальны | — |
| **A06:2021 Vulnerable & Outdated Components** | — | Новый код; зависимости — pinned httpx/openai | Зависимости других пакетов out of scope |
| **A07:2021 Identification & Auth Failures** | — | Auth = API-ключ из ENV, редактируется в 3 слоях | — |
| **A08:2021 Software & Data Integrity** | MAJ-2 (retry без jitter / без Retry-After → риск rate-limit-ban), BLK-5 (timeouts), BLK-6 (нет max_content_size) | Все три — поведенческие риски, не injection | — |
| **A09:2021 Security Logging & Monitoring Failures** | **BLK-1** (критический: тело upstream-ответа логируется без редактирования), Positives (3 слоя redaction) | Это прямое нарушение A09.1 (logs не содержат sensitive data) | — |
| **A10:2021 SSRF** | — | MiniMax client использует только захардкоженные upstream URLs; URL-input отсутствует | `_validate_image_bytes` (MIN-4) допускает URL без Content-Length cap — отдельная A10-связанная заметка |

**Security-вывод:** в рамках уже собранных замечаний покрыты A04 (Insecure Design — BLK-3, BLK-8, MAJ-1, MAJ-3), A05 (Misconfiguration — BLK-1, MAJ-1), A08 (Integrity — MAJ-2, BLK-5, BLK-6), A09 (Logging — BLK-1). Главный security-blocker — **BLK-1**: тело upstream-ответа логируется без редактирования, это прямое нарушение A09.

---

## Структурированный JSON-свод (machine-readable)

```json
{
  "pr": "https://github.com/krikz/rob_box_project/pull/907",
  "head_sha": "6e75eb392cf2db75bebe740326ae7f2e2bba89fa",
  "base_sha": "91abcbca3b3e6c900bed154aca68ee1541ad981f",
  "audit_date": "2026-07-23",
  "reviewer": "architect",
  "head_short": "6e75eb39",
  "blockers": [
    {
      "id": "BLK-1",
      "title": "Утечка тела upstream-ответа в лог (Authorization/Cookie/секреты)",
      "file": "src/rob_box_voice/rob_box_voice/dialogue_node.py",
      "line": 1593,
      "end_line": 1597,
      "description": "f-строка лога включает str(exc.response.text)[:200] без редактирования. Upstream/прокси могут вернуть Authorization: Bearer *** cookies или фрагменты ключей → лог-канал становится каналом утечки. Это прямое нарушение OWASP A09 (Security Logging & Monitoring Failures).",
      "proposed_fix": "Не логировать body ответа. Оставить только status_code и request_id, либо прогонять текст через централизованный allowlist-редактор (_redact_sensitive_text или новый _RedactUpstreamBody), маскирующий Authorization, Cookie и известные секреты. Добавить тест с echo 'Authorization: Bearer ***' и проверкой caplog.",
      "owasp": "A09:2021",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_voice/rob_box_voice/dialogue_node.py#L1593-L1597"
    },
    {
      "id": "BLK-2",
      "title": "voice_assistant.yaml ссылается на несуществующий TTS-провайдер piper",
      "file": "docker/vision/config/voice_assistant/voice_assistant.yaml",
      "line": 129,
      "description": "provider: 'piper', но tts_node.py:229-233 принимает только {yandex, silero, minimax} → при запуске стек падает на старте с ValueError: provider must be one of: ...",
      "proposed_fix": "Заменить на provider: 'yandex' (совпадает с tts_node.py defaults и голосом 'anton').",
      "owasp": "A04:2021",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/docker/vision/config/voice_assistant/voice_assistant.yaml#L129"
    },
    {
      "id": "BLK-3",
      "title": "_apply_thinking_policy теряет новые поля LLMSettings",
      "file": "src/rob_box_llm/rob_box_llm/providers/minimax.py",
      "line": 301,
      "end_line": 324,
      "description": "rebuild LLMSettings field-by-field → добавление любого нового поля в LLMSettings (provider.py:155-164) молча выпадает из MiniMax-ответов. Скрытое нарушение контракта по умолчанию.",
      "proposed_fix": "Одна строка — return dataclasses.replace(settings, extra=merged_extra).",
      "owasp": "A04:2021",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax.py#L301-L324"
    },
    {
      "id": "BLK-4",
      "title": "_OpenAICompatibleProvider.complete/stream итерируют messages без freeze",
      "file": "src/rob_box_llm/rob_box_llm/providers/deepseek.py",
      "line": 211,
      "end_line": 254,
      "description": "Iterable[LLMMessage] принимает генератор. После первой итерации (в _require_capability_for_messages) он пуст → на второй (_build_kwargs) в OpenAI SDK уходит messages=[] → 400/пустой ответ.",
      "proposed_fix": "messages = tuple(messages) в начале complete() и stream() (deepseek.py:308-371 — аналогично). O(n) памяти, раз и навсегда убирает класс багов.",
      "owasp": "A04:2021",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/deepseek.py#L211-L254"
    },
    {
      "id": "BLK-5",
      "title": "HTTP-таймауты: один float на все фазы (connect/read/write/pool)",
      "file": "src/rob_box_llm/rob_box_llm/providers/minimax.py + src/rob_box_llm/rob_box_llm/providers/minimax_tts.py + src/rob_box_llm/rob_box_llm/tts_provider_base.py",
      "line": 230,
      "description": "timeout: float ⇒ connect/read/write/pool = одинаковые 30 секунд. При зависании DNS/TLS на connect-фазе пользователь голосового пайплайна ждёт 30 с на каждый вызов. Anti-pattern для latency-sensitive путей.",
      "proposed_fix": "Принимать httpx.Timeout или кортеж (connect, read, write, pool). Дефолт: httpx.Timeout(connect=5.0, read=20.0, write=10.0, pool=5.0).",
      "owasp": "A08:2021",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax.py#L230-L237"
    },
    {
      "id": "BLK-6",
      "title": "Нет Limits(max_content_size=...) и нет max_connections в HTTP-клиентах",
      "file": "src/rob_box_llm/rob_box_llm/providers/minimax_tts.py + src/rob_box_llm/rob_box_llm/tts_provider_base.py",
      "line": 581,
      "description": "Клиент готов съесть ответ любого размера. LLM: 100 МБ JSON → OOM. TTS: hex-encoded audio для длинного текста → десятки МБ. Дефолт пула 100/100 для единичного провайдера избыточен.",
      "proposed_fix": "Задать httpx.Limits(max_connections=10, max_keepalive_connections=5, max_content_size=50*1024*1024) при создании клиента.",
      "owasp": "A08:2021",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax_tts.py#L581"
    },
    {
      "id": "BLK-7",
      "title": "aclose LLM-провайдера не идемпотентен",
      "file": "src/rob_box_llm/rob_box_llm/providers/deepseek.py",
      "line": 372,
      "end_line": 373,
      "description": "await self._client.close() без is_closed guard. При повторном aclose → RuntimeError: Client has not been opened. TTS-провайдер уже идемпотентен (minimax_tts.py:913-920).",
      "proposed_fix": "async def aclose(self) -> None: if not self._client.is_closed: await self._client.close()",
      "owasp": "A04:2021",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/deepseek.py#L372-L373"
    },
    {
      "id": "BLK-8",
      "title": "Гонка shutdown с lazy-инициализацией MiniMax-провайдера в ROS2-ноде",
      "file": "src/rob_box_voice/rob_box_voice/tts_node.py",
      "line": null,
      "end_line": null,
      "description": "Провайдер создаётся при первом вызове (lazy init). Параллельный shutdown ноды может обратиться к частично инициализированному провайдеру → AttributeError / use-after-init. ROS2-ревью указало диапазон ~1140-1180 (блок _synthesize_minimax_async), но точные строки lazy-init / shutdown не верифицированы в этом прогоне.",
      "proposed_fix": "Вынести lazy-init в отдельный _ensure_minimax_provider() с threading.Lock / однократной инициализацией; в shutdown() сначала проверять is_initialized, затем await provider.aclose() с try/finally.",
      "owasp": "A04:2021",
      "needs_more_info": "точные номера строк lazy-init и shutdown-блока в tts_node.py — требуют ручной верификации против PR diff (указано только ~1140-1180)",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_voice/rob_box_voice/tts_node.py"
    },
    {
      "id": "BLK-9",
      "title": "Неограниченный fan-out daemon threads в ROS2-нодах",
      "file": null,
      "line": null,
      "description": "ROS2-ревью указало риск: порождение thread-per-request без лимита (threading.Thread(target=..., daemon=True) без Semaphore/очереди) в src/rob_box_voice/rob_box_voice/ вокруг tts_node/dialogue_node. Конкретные file:line не верифицированы — это требует grep по PR diff.",
      "proposed_fix": "Заменить на bounded ThreadPoolExecutor (или asyncio-путь), либо явно ограничить семафором. Задокументировать ожидаемую concurrency.",
      "owasp": "A04:2021",
      "needs_more_info": "file:line не указаны в исходных ревью — требуется ручной поиск 'threading.Thread(target=..., daemon=True)' в PR diff",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_voice/rob_box_voice/"
    }
  ],
  "major": [
    {
      "id": "MAJ-1",
      "title": "Fail-fast отсутствует: tts_node стартует с пустыми MiniMax credentials",
      "file": "src/rob_box_voice/rob_box_voice/tts_node.py",
      "line": 386,
      "end_line": 392,
      "description": "При provider=minimax отсутствие MINIMAX_API_KEY/MINIMAX_GROUP_ID логируется как warning, нода проходит __init__ и успешно объявляет готовность. Реальный отказ — на первом synthesize-вызове.",
      "proposed_fix": "При provider == 'minimax' валидировать оба env в __init__/startup, бросать конфигурационное исключение до объявления готовности. Добавить тест запуска без каждого обязательного env.",
      "blocks_merge": false,
      "owasp": "A04:2021",
      "rationale_blocks_merge_false": "Система не падает целиком — другие TTS-провайдеры (yandex, silero) продолжают работать. Это operational hazard, не security-уязвимость. Однако при policy 'all major fix before merge' фактически блокирует.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_voice/rob_box_voice/tts_node.py#L386-L392"
    },
    {
      "id": "MAJ-2",
      "title": "Retry в tts_node без jitter и без учёта Retry-After (thundering herd + IP ban)",
      "file": "src/rob_box_voice/rob_box_voice/tts_node.py",
      "line": 1201,
      "end_line": 1258,
      "description": "Exponential backoff есть, но нет jitter → thundering herd. Retry не учитывает Retry-After от MiniMax для 429 → риск бана IP.",
      "proposed_fix": "delay = (backoff_ms / 1000.0) * (2 ** attempt); jittered = delay * (0.5 + random.random())  # ±50%; await asyncio.sleep(jittered). Отдельная ветка для 429: если в exc доступен Retry-After — использовать его вместо backoff.",
      "blocks_merge": false,
      "owasp": "A08:2021",
      "rationale_blocks_merge_false": "Проявляется только под нагрузкой / при частичных отказах MiniMax; не ломает cold-start, но ухудшает поведение системы при первом же инциденте.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_voice/rob_box_voice/tts_node.py#L1201-L1258"
    },
    {
      "id": "MAJ-3",
      "title": "Нет asyncio.CancelledError cleanup в провайдерах (висящие соединения при shutdown)",
      "file": "src/rob_box_llm/rob_box_llm/providers/minimax.py + src/rob_box_llm/rob_box_llm/providers/minimax_tts.py",
      "line": 731,
      "description": "OpenAI SDK и httpx поддерживают cancellation, но если ROS-нода завершается с активным await self.minimax_provider.synthesize(...), CancelledError пробросится без cleanup → aclose() не вызовется, соединения повиснут.",
      "proposed_fix": "async def synthesize(self, text, *, settings=None): try: data = await self._post(payload) except asyncio.CancelledError: await self.aclose(); raise",
      "blocks_merge": false,
      "owasp": "A04:2021",
      "rationale_blocks_merge_false": "Проявляется только при нештатном shutdown ROS-ноды с активным запросом. Не блокирует happy path.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax_tts.py#L731-L763"
    }
  ],
  "minor": [
    {
      "id": "MIN-1",
      "title": "APIConnectionError → TimeoutError семантически нечестно",
      "file": "src/rob_box_llm/rob_box_llm/providers/deepseek.py + src/rob_box_llm/rob_box_llm/providers/minimax_tts.py",
      "line": 139,
      "description": "OpenAI/httpx APIConnectionError/ConnectError/NetworkError маппятся в TimeoutError. Семантически это не таймаут, а ECONNREFUSED/DNS/TLS/reset. Теряется тип проблемы.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/deepseek.py#L139-L161"
    },
    {
      "id": "MIN-2",
      "title": "_BUILTIN_VOICES dead code в minimax_tts.py",
      "file": "src/rob_box_llm/rob_box_llm/providers/minimax_tts.py",
      "line": 312,
      "end_line": 355,
      "description": "Модульная константа _BUILTIN_VOICES нигде не используется в runtime. Runtime list_voices() (line 495-547) определяет свой inline-список. Docstring в строке 374 утверждает, что _BUILTIN_VOICES — источник истины; это неверно.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax_tts.py#L312-L355"
    },
    {
      "id": "MIN-3",
      "title": "_build_request_payload hardcode stream=False",
      "file": "src/rob_box_llm/rob_box_llm/providers/minimax_tts.py",
      "line": 610,
      "end_line": 616,
      "description": "Mandatory override hook не различает streaming/non-streaming. synthesize/stream обходят hook (minimax_tts.py:743, 795) и вызывают _build_payload напрямую с правильным флагом. Hook становится test-only.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax_tts.py#L610-L616"
    },
    {
      "id": "MIN-4",
      "title": "_validate_image_bytes не покрывает URL/data: источники (size cap bypass)",
      "file": "src/rob_box_llm/rob_box_llm/providers/minimax.py",
      "line": 130,
      "end_line": 148,
      "description": "Cap 10 MB применяется только к bytes. URL/data:image/...;base64,... (50 MB+) проходят без проверки. MiniMax отвергнет серверной стороной → caller увидит generic 400. Слабо связано с OWASP A10 (SSRF) — URL без Content-Length pre-check.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax.py#L130-L148"
    },
    {
      "id": "MIN-5",
      "title": "Нет общего retry-декоратора для LLM-провайдеров",
      "file": "src/rob_box_llm/rob_box_llm/providers/deepseek.py",
      "line": 139,
      "description": "Retry-инфраструктуры в rob_box_llm нет; caller ответственен. Если retry добавят — нужно учитывать идемпотентность (LLM c tools НЕ идемпотентен). См. ADR-0005 и OPEN-1 в architectural-conflicts.md.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/deepseek.py#L139"
    },
    {
      "id": "MIN-6",
      "title": "Sleep в тесте SSE (time-based flaky test)",
      "file": "src/rob_box_llm/test/test_minimax_tts_streaming.py",
      "line": 73,
      "end_line": 91,
      "description": "Тестовый SSE-generator содержит реальный await asyncio.sleep() между чанками. Делает тест time-based/flaky, противоречит требованию 'без sleep'.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/test/test_minimax_tts_streaming.py#L73-L91"
    },
    {
      "id": "MIN-7",
      "title": "Voice/ROS тесты не изолированы от numpy (collection failure)",
      "file": "src/rob_box_voice/test/unit/tts/test_minimax_integration.py",
      "line": null,
      "description": "На чистом окружении без numpy collection падает с ModuleNotFoundError. Workflow G-TTS-Provider-Tests.yml запускает только src/rob_box_llm → ROS/voice-тесты этим gate не покрыты.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_voice/test/unit/tts/test_minimax_integration.py"
    },
    {
      "id": "MIN-8",
      "title": "Дубликат if __name__ == '__main__' в test_audio_capture_harness",
      "file": "tools/audio_capture_harness/test_audio_capture_harness.py",
      "line": 559,
      "end_line": 568,
      "description": "Два одинаковых блока if __name__ == '__main__': unittest.main(...). Функционально не ломает pytest, но снижает поддерживаемость.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/tools/audio_capture_harness/test_audio_capture_harness.py#L559-L568"
    },
    {
      "id": "MIN-9",
      "title": "Workflow glob избыточен в G-TTS-Provider-Tests.yml",
      "file": ".github/workflows/G-TTS-Provider-Tests.yml",
      "line": 39,
      "description": "'src/rob_box_llm/**.py' рядом с более конкретными glob'ами — вводит в заблуждение, не покрывает рекурсивно.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/.github/workflows/G-TTS-Provider-Tests.yml#L39"
    },
    {
      "id": "MIN-10",
      "title": "Coverage scope ограничен только minimax_tts.py",
      "file": ".github/workflows/G-TTS-Provider-Tests.yml",
      "line": null,
      "description": "Gate измеряет только minimax_tts.py (100%, threshold 85%); minimax.py и ROS-интеграция не покрыты.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/.github/workflows/G-TTS-Provider-Tests.yml"
    },
    {
      "id": "MIN-11",
      "title": "conftest fixture minimax_provider не закрывает httpx.AsyncClient",
      "file": "src/rob_box_llm/test/conftest.py",
      "line": null,
      "description": "Клиент создаётся без aclose() в teardown → потенциальный ресурсный leak при xdist/длинных прогонах.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/test/conftest.py"
    },
    {
      "id": "MIN-12",
      "title": "Docs drift: MiMo-7B vs mimo-v2.5-pro",
      "file": "docs/guides/examples/minimax_llm.yaml + docs/guides/MINIMAX.md + src/rob_box_llm/README.md",
      "line": null,
      "description": "Документация говорит MiMo-7B, код (providers/mimo.py:27) и live config (src/rob_box_voice/config/voice_assistant.yaml:118) — mimo-v2.5-pro. Code is source of truth.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/docs/guides/examples/minimax_llm.yaml"
    },
    {
      "id": "MIN-13",
      "title": "Тест _raise_for_base_resp для LLM не найден",
      "file": "src/rob_box_llm/rob_box_llm/providers/minimax.py",
      "line": 159,
      "end_line": 183,
      "description": "Для LLM-_raise_for_base_resp не нашлось прямого параметризованного теста. Покрытие косвенное через _post_process_response.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax.py#L159-L183"
    },
    {
      "id": "MIN-14",
      "title": "TTSChunk(finish_reason='error') после yielded_audio=True (двойная обёртка)",
      "file": "src/rob_box_llm/rob_box_llm/providers/minimax_tts.py",
      "line": 856,
      "end_line": 858,
      "description": "Если SSE-цикл получил error envelope с yielded_audio=True, возвращается TTSChunk(finish_reason='error') без raise. Caller в tts_node.py:1280 ловит это как raise Exception → двойная обёртка.",
      "source_permalink": "https://github.com/krikz/rob_box_project/blob/6e75eb39/src/rob_box_llm/rob_box_llm/providers/minimax_tts.py#L856-L858"
    }
  ],
  "architectural": [],
  "architectural_section_omitted_reason": "В pr-907-architectural-conflicts.md все 4 пункта OPEN-1…OPEN-4 (retry-policy для tools=True, SSE-vs-WebSocket, dual-TTS-layer, error-hierarchy) явно помечены архитектором как deferred без эскалации. Реальных архитектурных конфликтов (где рекомендации ревьюеров взаимоисключают друг друга и требуют примирения) не выявлено. Согласно п.2 ТЗ, секция опущена.",
  "positives": [
    "Secrets hygiene — образцовая. MINIMAX_API_KEY/MINIMAX_GROUP_ID берутся только из ENV, редактируются в 3 слоях (_redact_sensitive_text, _RedactGroupIdFilter, MiniMaxRedactedLogFilter). 11 dedicated leak-guard тестов покрывают каждый путь утечки. (SEC, ARC, BE)",
    "Контракты звучат. LLMProvider/TTSProvider/BaseTTSProvider — чистое разделение. Отдельные ProviderError/TTSError иерархии. Multimodal: MessageContent = Union[str, tuple[MessagePart, ...]] — text-only callers не сломаны (357/357 старых тестов проходят). Capability defaults консервативные. (ARC)",
    "Capability-интроспекция честная. ProviderCapabilities per-model на MiniMaxProvider (minimax.py:244-261). Vision-капабилити гейтится ДО сетевого вызова в _require_capability_for_messages → fail-fast с типизированным CapabilityUnavailableError. (ARC, BE)",
    "_map_exception централизован. LLM — deepseek.py:139, TTS — minimax_tts.py:96, MiniMax envelope — minimax.py:159. Единая иерархия ошибок, одинаковый provider= kwarg, никаких openai.APIError/httpx наружу. (ARC, BE)",
    "FakeLLMProvider/FakeTTSProvider детерминированные. Реализуют контракты полностью, record calls. Тесты пакета идут без сети. (ARC)",
    "OpenAI-compatible wire format переиспользуется. _OpenAICompatibleProvider — общая база; DeepSeekProvider/MiMoProvider это 25-строчные обёртки; MiniMaxProvider добавляет ~80 строк для envelope/pre-flight/thinking-policy. Будущие OpenAI-compat провайдеры = одна строка subclass. (ARC)",
    "Async-пути чистые. Поиск time.sleep|requests.|urllib.|open( в minimax.py/minimax_tts.py/deepseek.py — пусто. Никаких блокирующих I/O в async. (BE)"
  ],
  "positives_selection_note": "Из 12 переданных позитивов выбрано 7 наиболее независимых (покрывающих security, contracts, capability, errors, tests, extensibility, async-safety). Остальные 5 (parameterised tests details, TTS container honesty, TTL-safe aclose, network error honesty, contract↔impl↔ROS↔yaml matrix) являются частными случаями / детализацией выбранных 7. Сокращение до 7 сделано согласно п.4 ТЗ ('positives[]: 3-7 пунктов'). Ничего не выдумывалось.",
  "recommended_review_state": "REQUEST_CHANGES",
  "recommended_review_state_rationale": "9 blockers (BLK-1…BLK-9) явно подтверждены с file:line и proposed_fix (BLK-8 и BLK-9 имеют needs_more_info по точным строкам, но факт наличия проблемы подтверждён ROS2-ревью). Per repo convention 'blockers fix before merge' и согласно п.4 правил ТЗ, вердикт = REQUEST_CHANGES. Все blockers и majors имеют готовые фиксы; major (MAJ-1…MAJ-3) желательно закрыть в этом же PR, т.к. они дёшевы, но не блокируют merge при согласовании с владельцем PR.",
  "needs_more_info_summary": [
    "BLK-8: точные строки lazy-init и shutdown-блока в tts_node.py — требуют ручной верификации против PR diff (указано только ~1140-1180)",
    "BLK-9: file:line не указаны в исходных ревью — требуется ручной поиск 'threading.Thread(target=..., daemon=True)' в PR diff"
  ],
  "owasp_summary": {
    "covered": ["A04 (Insecure Design)", "A05 (Misconfiguration)", "A08 (Integrity)", "A09 (Logging)"],
    "primary_security_blocker": "BLK-1 — прямое нарушение A09 (логирование upstream response body без редактирования)",
    "out_of_scope": ["A01 (Access Control)", "A02 (Crypto)", "A03 (Injection)", "A06 (Components)", "A07 (Auth)", "A10 (SSRF) — только косвенно MIN-4"]
  }
}
```

## Что делать дальше (владельцу PR)

1. Закрыть 9 blockers (BLK-1…BLK-9). Все имеют готовые фиксы (для HTTP — см. `pr-907-http-client-review.md` §Fix 1-4; для architect F1/F2 — однострочники; для F3 — yaml one-liner; для F8/F9 — `needs_more_info` сначала верифицировать точные строки).
2. Желательно закрыть MAJ-1…MAJ-3 в этом же PR (дёшево).
3. Прогнать `PYTHONPATH=src/rob_box_llm:src/rob_box_llm/test pytest src/rob_box_llm/test/ -q` — должно остаться 244+ passed без регрессий.
4. После re-review — переключить вердикт на APPROVE и merge.

— architect (Hermes Agent), 2026-07-23
