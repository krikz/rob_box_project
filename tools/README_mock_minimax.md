# tools/mock_minimax_server.py — локальный HTTP/SSE mock MiniMax TTS

Standalone-сервер, имитирующий MiniMax T2A v2 HTTP endpoint
(`https://api.minimax.io/v1/t2a_v2`) на localhost. Принимает запросы от
`MiniMaxTTSProvider` / `tts_node` (или любого curl) и отвечает из локальной
директории фикстур, без реальных API-ключей и без интернета.

> **Зачем нужен:** tts_node использует MiniMax через HTTP. Когда
> разработчик хочет прогнать ROS2-bench или ручной curl без оплаченного
> аккаунта, без сетевого доступа, и без leak'а настоящего ключа в логи —
> этот mock подменяет эндпоинт. Все ответы **wire-compatible** с
> реальным API: один и тот же провайдер работает с mock'ом и с продакшном.

## Запуск одной командой

```bash
# из корня репозитория (worktree .worktrees/t_ac5f796b):
python3 tools/mock_minimax_server.py

# — это запускает сценарий pcm-chunked на 127.0.0.1:18080
# — фикстуры берутся из tts_audio_bench/fixtures/{pcm,wav,mp3,ogg}/sine_*.{pcm,wav,mp3,ogg}
```

Запуск печатает готовые `curl`-команды, которые можно скопировать в
терминал для проверки.

## Предустановленные сценарии

```bash
# сценарий 1 — PCM chunked (single-chunk non-streaming JSON, 16 kHz)
python3 tools/mock_minimax_server.py --scenario pcm-chunked

# сценарий 2 — WAV SSE streaming (4 чанка @ ~50 ms)
python3 tools/mock_minimax_server.py --scenario wav-streaming

# сценарий 3 — MP3 non-streaming (provider-уровневый MP3 fallback от OGG)
python3 tools/mock_minimax_server.py --scenario mp3

# сценарий 4 — OGG non-streaming (real OGG fixture, fallback на MP3 если нет)
python3 tools/mock_minimax_server.py --scenario ogg

# сценарий 5 — auth-error injection (MiniMax base_resp.status_code=1001)
python3 tools/mock_minimax_server.py --scenario auth-error

# явный порт/фикстуры/флаги
python3 tools/mock_minimax_server.py \
    --port 18080 \
    --fixtures tts_audio_bench/fixtures \
    --scenario wav-streaming \
    --chunk-count 4 \
    --chunk-delay-ms 50 \
    --verbose
```

## Документированные эндпоинты

### `GET /health` — health-check mock'а (НЕ часть MiniMax API)

Возвращает JSON с диагностикой:

```json
{
  "status": "ok",
  "mock": "MiniMax T2A v2",
  "fixtures_root": "/abs/path/to/tts_audio_bench/fixtures",
  "active_fail_status": null,
  "active_chunk_count": 4,
  "active_chunk_delay_ms": 50
}
```

Пример: `curl -s http://127.0.0.1:18080/health`

### `POST /v1/t2a_v2` — единственная ручка MiniMax T2A v2

Заголовки:

| Header | Обязательность | Поведение mock'а |
|---|---|---|
| `Authorization` | обязателен | Любое значение, начинающееся с `Bearer `, принимается. Отсутствие/неправильный префикс → `401 base_resp=1001 "missing Bearer"`. |
| `Content-Type` | обязателен | Должно быть `application/json`. Иначе JSON-decode вернёт 400. |

Query-параметры:

| Param | Обязательность | Поведение |
|---|---|---|
| `GroupId` | обязателен | Любое непустое значение. Отсутствие/пустота → `401 base_resp=1001 "missing GroupId"`. |
| `chunk_count` | опц. | Сколько SSE-чанков выдать в streaming-режиме. По умолчанию = scenario default. |
| `chunk_delay_ms` | опц. | Пауза между чанками в мс (первый чанк отправляется СРАЗУ, чтобы TTFA = transport latency). |
| `fail_status` | опц. | Включает error injection — ответом будет `{"base_resp": {"status_code": N, "status_msg": "..."}}`. |
| `fail_msg` | опц. | status_msg для error injection (default: `injected failure`). |

Тело запроса (JSON):

```json
{
  "model": "speech-02-hd",
  "text": "произвольный текст",
  "voice_setting": {
    "voice_id": "male-qn-qingse"
  },
  "audio_setting": {
    "format": "pcm",        // pcm | wav | mp3 | ogg
    "sample_rate": 16000,   // 16000 | 24000 | 32000
    "channel": 1
  },
  "stream": false           // true → SSE streaming
}
```

## Форматы ответов

### Non-streaming (`stream: false`)

```json
{
  "data": {
    "audio": "<hex-encoded audio bytes>",
    "audio_sample_rate": 16000,
    "audio_length": 32000
  },
  "extra_info": {
    "voice_id": "male-qn-qingse",
    "format": "pcm",
    "fixture": true
  },
  "base_resp": {
    "status_code": 0,
    "status_msg": "success"
  }
}
```

**Поддерживаемые комбинации `format × sample_rate`:**

| format | sample_rate | Файл фикстуры | Магия первых байт |
|---|---|---|---|
| pcm | 16 kHz | `fixtures/pcm/sine_440hz_1.00s_16000.pcm` | `raw int16 LE` |
| pcm | 32 kHz | `fixtures/pcm/sine_440hz_1.00s_32000.pcm` | `raw int16 LE` |
| wav | 16 kHz | `fixtures/wav/sine_440hz_1.00s_16000.wav` | `RIFF....WAVE` |
| wav | 32 kHz | `fixtures/wav/sine_440hz_1.00s_32000.wav` | `RIFF....WAVE` |
| mp3 | 16 kHz | `fixtures/mp3/sine_440hz_1.00s_16000.mp3` | `ID3\x04` или `\xff\xfb` |
| mp3 | 32 kHz | `fixtures/mp3/sine_440hz_1.00s_32000.mp3` | `ID3\x04` или `\xff\xfb` |
| ogg | 16 kHz | `fixtures/ogg/sine_440hz_1.00s_16000.ogg` (или mp3 fallback) | `OggS` |

Для sample_rate=24000 Hz тоже есть фикстуры (`sine_440hz_1.00s_24000.{pcm,wav,mp3,ogg}`).
Если запрошенная комбинация не найдена — ответ будет
`{"base_resp": {"status_code": 1002, "status_msg": "no fixture for fmt=... sr=..."}}`.

### Streaming (`stream: true`)

HTTP/1.1 chunked transfer-encoding с `Content-Type: text/event-stream`.
Каждое событие — `data:<json>\n\n`, терминатор — `data:[DONE]\n\n`:

```
data:{"data":{"audio":"<hex1>","audio_sample_rate":16000,"audio_length":8011},"extra_info":{"voice_id":"...","format":"wav","fixture":true,"chunk_index":0,"chunk_count":4},"base_resp":{"status_code":0,"status_msg":"success"}}

data:{"data":{"audio":"<hex2>","audio_sample_rate":16000,"audio_length":8011},"extra_info":{"chunk_index":1,"chunk_count":4},"base_resp":{"status_code":0,"status_msg":"success"}}

data:{"data":{"audio":"<hex3>",...},"extra_info":{"chunk_index":2,"chunk_count":4},"base_resp":{"status_code":0,"status_msg":"success"}}

data:{"data":{"audio":"<hex4>",...},"extra_info":{"chunk_index":3,"chunk_count":4},"base_resp":{"status_code":0,"status_msg":"success"}}

data:[DONE]
```

Точно такой же формат потребляет `MiniMaxTTSProvider.stream()` — доказан
тестом `test_minimax_tts_streaming.py`.

## Примеры curl

```bash
# Health check
curl -s http://127.0.0.1:18080/health

# Non-streaming PCM
curl -s -X POST "http://127.0.0.1:18080/v1/t2a_v2?GroupId=local-test" \
  -H "Authorization: Bearer anything" \
  -H "Content-Type: application/json" \
  -d '{"model":"speech-02-hd","text":"hello","voice_setting":{"voice_id":"male-qn-qingse"},"audio_setting":{"format":"pcm","sample_rate":16000,"channel":1}}'

# Streaming WAV с 4 чанками
curl -N -s -X POST "http://127.0.0.1:18080/v1/t2a_v2?GroupId=local-test&chunk_count=4&chunk_delay_ms=50" \
  -H "Authorization: Bearer anything" \
  -H "Content-Type: application/json" \
  -d '{"model":"speech-02-hd","text":"hello","voice_setting":{"voice_id":"male-qn-qingse"},"audio_setting":{"format":"wav","sample_rate":16000,"channel":1},"stream":true}'

# Error injection: MiniMax вернёт base_resp.status_code=1001
curl -s -X POST "http://127.0.0.1:18080/v1/t2a_v2?GroupId=local-test&fail_status=1001&fail_msg=invalid%20api%20key" \
  -H "Authorization: Bearer anything" \
  -H "Content-Type: application/json" \
  -d '{"model":"speech-02-hd","text":"hello","voice_setting":{"voice_id":"male-qn-qingse"},"audio_setting":{"format":"pcm","sample_rate":16000,"channel":1}}'
```

## Что НЕ нужно настраивать в продакшн-коде

Mock **принимает любые** значения `Authorization` и `GroupId`. Поэтому
для тестов не нужны настоящие креды MiniMax.

Но чтобы **переключить `tts_node` на этот mock** без правок продакшн-кода,
нужны изменения в `MiniMaxTTSProvider.__init__` (см. `BLOCKERS.md`).
Прямо сейчас tts_node всегда ходит на `https://api.minimax.io`,
поскольку `base_url` не читается из env. Это задокументировано как blocker.

## Связь с test-bench

`tts_audio_bench/scripts/mock_minimax_server.py` — функционально эквивалентный
скрипт, который импортируется `run_bench.py` напрямую (через `start_server()`).
Версия в `tools/` — это standalone для ручной работы, CI и ad-hoc curl-проверок.
Если правите mock — синхронизируйте обе версии, либо вынесите общую логику
в shared-модуль (пока out of scope для этой задачи).