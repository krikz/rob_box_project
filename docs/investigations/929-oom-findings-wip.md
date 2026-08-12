# Issue #929 — предварительные findings (WIP)

## Измерения на живом роботе (Vision Pi 10.1.1.21, voice-assistant, 12.08 20:00)

docker stats: 3.07-3.39GiB / 4GiB (77-85%), PIDS 203.

RSS по нодам (ps aux --sort=-rss):
- tts_node (pid 73):       1 042 000 kB (~1.04 GB) — torch + Silero v5 warm-load
- speaker_id_node (pid 81):  997 000 kB (~1.0 GB) — torch + resemblyzer
- sclang (pid 33):           549 000 kB (~550 MB) — SuperCollider/FoxDot
- stt_node (pid 75):         339 000 kB (Vosk)
- sound_node:                238 MB, audio_node: 234 MB, dialogue_node: 256 MB
- mcp_server: 151 MB, command/led/animation: ~120 MB each

Суммарный RSS ~4.1-4.2 GB при лимите 4 GB.

## Тест «10 запросов» (2 раунда по 10 STT-команд «робок скажи тест N»)

Раунд 1 (первые 10 запросов после старта):
- tts_node: 976 824 -> 1 038 800 kB (+62 MB)
- speaker_id: 951 684 -> 998 308 kB (+47 MB)
- dialogue: 219 276 -> 251 532 kB (+32 MB)

Раунд 2 (следующие 10 запросов):
- tts_node: 1 038 800 -> 1 042 012 kB (+3.2 MB)
- speaker_id: 998 308 -> 997 108 kB (-1.2 MB)
- dialogue: 251 532 -> 255 700 kB (+4.2 MB)

ВЫВОД: рост в раунде 1 = одноразовый warm-up (HTTP/gRPC клиенты, кэши, numpy).
Раунд 2 = плато (3 MB на 10 запросов — шум). ЛИНЕЙНОЙ УТЕЧКИ НЕТ.

## Структурная причина OOM (при лимите 2 GB)

- tts_node и speaker_id_node — ДВА отдельных процесса, каждый грузит PyTorch
  (tts_node: libtorch_cuda 300 MB + libcublasLt 488 MB + 1 GB anonymous allocator;
   speaker_id: то же самое).
- Silero v5 warm-load запускается при старте tts_node (коммит ac0bf908 07-31)
  даже когда provider=yandex/minimax — ~700 MB-1 GB держится постоянно.
- speaker_id_node (resemblyzer, ~1 GB) добавлен в headless launch 08-12 (193c6cf3).
- При лимите 2 GB суммарный RSS ~3.5-4 GB -> OOM killer убивает tts_node при
  попытке догрузить Silero v5 fallback (нужен ещё ~200 MB).

## Хронология

- 07-28: W5 рефакторинг dialogue_node (2a0aee26)
- 07-30: issue #929 создан; 77b75d79 поднял mem_limit 2g->4g + нашёл дубль speech_id
- 07-31: Silero warm-load добавлен (ac0bf908) — tts_node держит torch+Silero всегда
- 08-12: speaker_id_node в headless launch (193c6cf3) — второй torch-процесс ~1 GB

## Кандидаты на фикс

1. Параметр silero_warm_load (default true) — позволяет отключить warm-load
   на конфигурациях, где Silero fallback редок (экономия ~1 GB).
2. Ленивый импорт torch в tts_node (импорт внутри _load_silero_model) —
   экономия ~500-800 MB когда Silero не нужен.
3. speaker_id_node: lazy-warmup resemblyzer (грузить модель при первом
   utterance, не при старте) — экономия ~1 GB.
