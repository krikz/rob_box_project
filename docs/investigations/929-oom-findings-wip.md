# Issue #929 — расследование OOM kill tts_node (финальные findings)

## Диагноз: линейной утечки НЕТ, причина — структурный перевес памяти

### Измерения на живом роботе (Vision Pi 10.1.1.21, voice-assistant, 12.08 20:00-20:20)

docker stats: 3.07-3.39GiB / 4GiB (77-85%), PIDS 203. На Pi 7.8GB RAM, 5.8GB занято, swap=0.

RSS по нодам (ps aux --sort=-rss):
- tts_node (pid 73):       1 042 000 kB (~1.04 GB) — torch + Silero v5 warm-load
- speaker_id_node (pid 81):  997 000 kB (~1.0 GB) — torch + resemblyzer
- sclang (pid 33):           549 000 kB (~550 MB) — SuperCollider/FoxDot
- stt_node (pid 75):         339 000 kB (Vosk)
- sound_node: 238 MB, audio_node: 234 MB, dialogue_node: 256 MB
- mcp_server: 151 MB, command/led/animation: ~120 MB each

Суммарный RSS ~4.1-4.2 GB при лимите 4 GB. PSS (реальное потребление с учётом shared):
tts_node 834 MB, speaker_id 798 MB.

### Тест «10 запросов» (2 раунда по 10 STT-команд «робок скажи тест N»)

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
После 90-секундной паузы память НЕ вернулась — это не временные аллокации,
а постоянный baseline от загруженных моделей.

### Реальная причина OOM (при лимите 2 GB, до 30.07)

1. **Два процесса с PyTorch**: tts_node (torch + Silero v5, ~1 GB) и
   speaker_id_node (torch + resemblyzer, ~1 GB). У каждого 1 GB anonymous
   mapping (PyTorch allocator) + shared libs (libtorch_cuda 300 MB,
   libcublasLt 488 MB).
2. **Silero v5 warm-load при старте** (коммит ac0bf908 07-31): tts_node
   грузит torch + модель (~700 MB-1 GB) в фоне при старте, даже когда
   provider=yandex/minimax и Silero — только fallback.
3. **speaker_id_node добавлен в headless launch 08-12** (193c6cf3) — второй
   torch-процесс ~1 GB (resemblyzer warmup при старте, 98b98d94).
4. При лимите 2 GB суммарный RSS ~3.5-4 GB → OOM killer убивает tts_node
   при попытке догрузить Silero v5 fallback (нужно ещё ~200 MB).

### Хронология

- 07-28: W5 рефакторинг dialogue_node (2a0aee26)
- 07-30: issue #929 создан; 77b75d79 поднял mem_limit 2g->4g + нашёл дубль speech_id
- 07-31: Silero warm-load добавлен (ac0bf908) — tts_node держит torch+Silero всегда
- 08-12: speaker_id_node в headless launch (193c6cf3) — второй torch-процесс ~1 GB

## Что сделано (фикс)

1. **Параметр `silero_warm_load`** (default: true — сохраняет контракт G-933-B
   и тесты test_silero_warm_load.py). При `false` фоновый warm-load Silero при
   старте НЕ запускается; модель грузится лениво при первом реальном fallback
   (2-3 с cold-load, приемлемо для аварийного пути). Экономия ~700 MB-1 GB RSS
   в tts_node на постоянной основе.
2. **src/rob_box_voice/config/tts_node.yaml**: `silero_warm_load: false` — на
   роботе (headless launch читает per-node YAML из src, см. ADR-0004).
3. **Hot-path `_synthesize_and_play`**: при `silero_warm_load=false` не ждёт
   событие `_silero_loaded` (его никто не set — warm-load не запускался) и не
   скипает chunk: идёт в синхронный lazy-load. getattr-fallback для стабов.
4. **Тесты**: +2 регрессионных (test_warm_load_disabled_by_parameter,
   test_warm_load_disabled_hot_path_lazy_loads). Весь tts unit-набор: 132 passed.

## Остаточный риск

- speaker_id_node (resemblyzer, ~1 GB) — второй torch-процесс; при лимите 4 GB
  контейнер ~77-85%. Если Pi упрётся в память снова, следующий шаг:
  lazy-warmup resemblyzer (грузить модель при первом utterance, не при старте).
- sclang 550 MB + 77% CPU в момент теста — отдельная история (DJ/music), не
  связана с OOM tts_node напрямую, но вклад в суммарный RSS есть.

## Воспроизведение (для e2e)

1. `docker restart voice-assistant` (чистый старт)
2. Подождать 1 мин (init всех нод)
3. Повторить 10 раз:
   ```bash
   for i in $(seq 1 10); do
     ros2 topic pub --once /voice/stt/result std_msgs/msg/String \
       "{data: \"робок скажи тест $i\"}"
     sleep 12
   done
   ```
4. Критерий: tts_node не умирает (нет "process has died ... exit code -9"),
   docker stats voice-assistant < 80% MEM после часа работы.
