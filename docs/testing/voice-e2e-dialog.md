# Voice E2E Dialog — регрессионный тест длинных LLM-ответов (issue #933)

> **Regression test для issue #933** — «voice/tts: Silero fallback тоже падает
> на 1005 chars — нужен per-provider max chunk + retry-halve».

Скрипт: [`scripts/diagnostics/voice_e2e_dialog.sh`](../../scripts/diagnostics/voice_e2e_dialog.sh)

## Что проверяет

Голосовой ассистент должен **полностью озвучивать длинные ответы LLM**
(>1000 chars). До фикса (#931 → #933) на длинных ответах:

1. Yandex gRPC v3 падал на ~291 chars — `INVALID_ARGUMENT - Too long text`;
2. Silero v5 fallback падал на ~1005 chars — `Synthesis error: Model couldn't
   generate your text, probably it's too long`;
3. Итог: `thinking.mp3` → тишина.

Фикс: per-provider max chunk (`CHUNK_LIMITS` в
`src/rob_box_voice/rob_box_voice/tts_chunking.py`, конфиг
`chunk_max_chars_yandex/silero/minimax` в `config/voice_assistant/voice_assistant.yaml`)
+ retry-halve (`synthesize_with_retry`, max 3 попытки).

Тест проверяет по логам `voice-assistant`:

| Маркер | Ожидание |
|---|---|
| `Воспроизведение завершено` | ✅ должно быть — робот доиграл весь ответ |
| `Too long text` (Yandex) | ❌ не должно быть |
| `Synthesis error` (Silero) | ❌ не должно быть |
| `LLM OUTPUT` / `Turn done. Response` | ✅ должно быть — диалог реально прошёл |

## Запуск

```bash
# С билд-машины (249) или любого хоста с sshpass:
./scripts/diagnostics/voice_e2e_dialog.sh 10.1.1.21
```

Env-переменные:

| Переменная | Default | Описание |
|---|---|---|
| `ROBOT_HOST` | `10.1.1.21` | Vision Pi |
| `ROBOT_USER` | `ros2` | SSH user |
| `SSHPASS` | `open` | SSH password |
| `PHASE_TEXT` | `робок расскажи длинный анекдот` | STT-фраза, инъекция в `/voice/stt/result` |
| `REACTION_TIMEOUT_S` | `150` | Сколько ждать полный цикл |
| `LLM_MIN_CHARS` | `500` | Ниже этой длины ответа → SKIP |

## Exit codes

| Код | Значение |
|---|---|
| 0 | PASS — полный цикл, ошибок нет |
| 2 | FAIL — `Too long text` / `Synthesis error` / нет «Воспроизведение завершено» |
| 3 | SKIP — LLM ответил коротко (<500 chars), регрессия не воспроизведена |
| 4 | ENV/SSH ошибка (робот недоступен, контейнера нет) |

## Как это связано с атомарным e2e-харнессом

- `.github/workflows/scripts/e2e_voice_test.sh` — атомарный голосовой цикл
  (динамик → микрофон → STT → LLM → TTS → playback). Он ждёт полный цикл
  `ПРИНЯТО → LLM INPUT → TTS finished → Воспроизведение завершено`, но
  **не проверяет негативные маркеры** (`Too long text` / `Synthesis error`).
- `voice_e2e_dialog.sh` — дополняет харнесс именно негативной проверкой:
  длинный ответ должен пройти БЕЗ провайдерских отказов. Прямая инъекция STT
  (без динамика/микрофона) делает тест детерминированным и не зависящим от
  акустики помещения.

## Пример прогона

```text
>>> voice_e2e_dialog.sh: регрессия issue #933 (per-provider chunking)
>>> ROBOT_HOST=10.1.1.21  PHASE_TEXT='робок расскажи длинный анекдот'
✓ voice-assistant запущен на 10.1.1.21
>>> BEFORE=2026-07-30T11:57:08Z
>>> Инъекция STT: робок расскажи длинный анекдот
>>> Ожидание полного цикла (150s)...
──────────────────────────────────────────────────────────────
  Сводка логов (с 2026-07-30T11:57:08Z):
  LLM:  📤 LLM OUTPUT: 'О, сейчас я прочитаю тебе знаменитый Ганстер-рэд про Федота' (1005 chars)
  TTS:  ✅ Yandex gRPC v3 (ROBBOX original!): 123456 samples, source 22050 Hz, speed=1.0, 4 chunk(s)
  TTS:  ✅ Воспроизведение завершено

✅ PASS: полный цикл для длинного ответа (1005 chars) — без Too long text / Synthesis error
```

До фикса тот же прогон давал:

```text
❌ FAIL: провайдерский error в логах
     [tts_node] Yandex gRPC chunk 1/1 failed (1005 chars): INVALID_ARGUMENT - Too long text
     [tts_node] Synthesis error: Model couldn't generate your text, probably it's too long
```
