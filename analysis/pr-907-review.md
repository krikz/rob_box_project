# PR #907 — review unit-тестов (task t_6063713a)

Дата: 2026-07-22
Base/head: `origin/main...pr-907`

## Результат проверки

### MUST

1. `src/rob_box_llm/test/test_minimax_tts_streaming.py:73-91` — тестовый SSE generator содержит реальный `await asyncio.sleep()` между чанками. В production-unit suite это не сетевой вызов, но это делает тесты time-based/flaky и противоречит требованию «без sleep». Если нужна проверка итерации по нескольким SSE-событиям, использовать deterministic async iterator/transport без задержки; отдельный latency-тест пометить `slow`/`integration` и вынести из default CI.

2. `src/rob_box_voice/test/unit/tts/test_minimax_integration.py` не изолирован от импортных окружений: запуск набора в чистом окружении невозможен без `numpy` уже на collection (`ModuleNotFoundError`). Для заявленного CI запуска требуется либо явная dev-зависимость/установка voice-пакета, либо тесты должны быть действительно ROS/voice-опциональными и skip с понятным marker. Сейчас workflow `G-TTS-Provider-Tests.yml` запускает только `src/rob_box_llm`, поэтому эти ROS/voice тесты не проверяются этим CI gate.

### SHOULD

3. `.github/workflows/G-TTS-Provider-Tests.yml:39` — glob `'src/rob_box_llm/**.py'` не покрывает рекурсивно все Python-файлы так, как это обычно ожидается от workflow path filter; рядом уже есть более конкретные `rob_box_llm/**.py` и `test/**.py`, но первый glob избыточен/вводит в заблуждение. Привести paths к одному явному рекурсивному шаблону (`src/rob_box_llm/**/*.py`) и добавить необходимые файлы конфигурации/requirements, если они меняются.

4. `.github/workflows/G-TTS-Provider-Tests.yml` запускает `-k 'minimax or tts_conformance'`, что фактически отбрасывает provider-independent tests, хотя job описан как conformance + provider tests. Это осознанно может быть быстрым фильтром, но тогда не следует создавать впечатление полного `rob_box_llm` unit gate. Лучше запускать `test_tts_conformance.py` полностью и добавить отдельный marker/selection для MiniMax.

5. `tools/audio_capture_harness/test_audio_capture_harness.py:559-568` содержит два одинаковых `if __name__ == "__main__": unittest.main(...)`. Функционально не ломает pytest, но это явный дефект тестового файла и снижает поддерживаемость; удалить дубликат.

### COULD

6. `src/rob_box_llm/test/test_conftest_fixtures.py` проверяет приватные поля (`_api_key`, `_group_id`, `_base_url`, `_owns_client`). Такие assertions полезны как fixture smoke tests, но привязывают тест к реализации конструктора. Оставить только поведенческие инварианты (параметры запроса/отсутствие реального сетевого вызова), а приватные проверки сократить.

7. В `conftest.py` fixture `minimax_provider` создаёт `httpx.AsyncClient`, который не закрывается в fixture teardown. При текущем размере suite предупреждение не проявилось, но это потенциальный ресурсный leak при xdist/длинных прогонах. Добавить `yield` и `await client.aclose()` либо использовать MockTransport/клиент из HTTP fixture.

8. Присутствует хороший coverage gate: `244 passed, 113 deselected`, `minimax_tts.py` — 100%, threshold 85%. Однако coverage измеряет только `minimax_tts.py`; provider `minimax.py` и ROS-интеграция этим gate не покрываются. Добавить отдельные coverage targets/тесты или явно документировать границы job.

## Что проверено

- PR head fetched локально как `pr-907`.
- `src/rob_box_llm`: `PYTHONPATH=. python3 -m pytest -k 'minimax or tts_conformance' -q`: **244 passed, 113 deselected**.
- С coverage: **100.00%** для `rob_box_llm/providers/minimax_tts.py`; gate 85% пройден.
- `src/rob_box_voice/test/unit/tts` и `utils`: collection остановилась на **2 errors** из-за отсутствия `numpy` в окружении. Это blocker среды/зависимостей, а не утверждение о runtime-дефекте.
- Сетевые вызовы MiniMax в проверенном provider suite заменены `respx`/`httpx` mock transport; реальный API key не используется.

## Итог

PR имеет сильный unit coverage для MiniMax TTS и адекватные mock-based HTTP проверки. До merge рекомендую закрыть MUST-1 (sleep/flakiness) и MUST-2 (явный CI/зависимости для voice/ROS tests), затем исправить дубликат `__main__` и уточнить scope workflow. Формальный GitHub review не отправлялся: задача требовала анализа и перечня пробелов, а не публикации review-комментария.

---

## Дополнение: ревью HTTP-клиента MiniMax (t_dd2b9833)

Детальный разбор сетевой части (timeout, retry, cancellation, секреты) — в файле
`analysis/pr-907-http-client-review.md`. Краткие выводы:

**Блокеры:**
1. Таймауты — единый `float`, нет разделения `connect/read/write/pool`. На TCP-handshake зависание → 30 секунд на каждый вызов. Файлы: `minimax.py:230-237`, `minimax_tts.py:573-581`, `tts_provider_base.py:241-252`.
2. Нет `Limits(max_content_size=...)` — теоретический риск OOM на гигантском JSON-ответе.
3. `aclose()` LLM-провайдера (`deepseek.py:372-373`) не идемпотентен.

**Warnings:**
4. `APIConnectionError` → `TimeoutError` семантически нечестно (теряем тип).
5. Retry без jitter (`tts_node.py:1247`) — thundering herd.
6. Retry игнорирует `Retry-After` для 429.
7. Нет cleanup при `asyncio.CancelledError`.
8. В LLM-провайдерах вообще нет retry-инфраструктуры.

**Что хорошо:**
- `except: pass` нигде нет — все сетевые ошибки честно маппятся в доменные.
- Параметризованные тесты (`test_minimax_tts_errors_parametrized.py:312-405`) реально покрывают 5 транспортных классов + `asyncio.TimeoutError`.
- Защита секретов через `_redact_sensitive_text` и `MiniMaxRedactedLogFilter` — defense in depth.
- Никаких блокирующих вызовов (`time.sleep`, `requests`, `open()`) в async-путях.

Готовые патчи для блокеров — в конце `pr-907-http-client-review.md`.
