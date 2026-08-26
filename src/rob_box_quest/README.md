# rob_box_quest

WebXR / Meta Quest telepresence service for Rob Box (Phase 2+).

## Структура

- `rob_box_quest/quest_node.py` — основной WSS-сервер (Phase 1+).
- `rob_box_quest/protocol/` — фрейм-кодек и ROS2-бриджи.
- `rob_box_quest/server/` — aiohttp WSS-сессии.
- `rob_box_quest/streams/` — реестр топиков (camera/lidar/robot_status/voice).
- `rob_box_quest/core/` — teleop/safety FSM.
- `rob_box_quest/perf/` — **Phase 2.2**: telemetry reporter + logger.
- `webxr_client/` — TypeScript-клиент (Three.js + WebXR), собирается vite'ом.

## Phase 2.2: telemetry reporter

См. `docs/architecture/meta-quest-api.md` §6.1 + ADR-0032 §3.5.

### Компоненты

| Файл | Роль |
|---|---|
| `webxr_client/src/perf/reporter.ts` | клиентский репортер (FPS/GPU/VRAM/RTT) |
| `webxr_client/src/wire/connection.ts` | минимальный WSS-клиент + ping/pong EMA |
| `webxr_client/src/main.ts` | bootstrap + `?telemetry=off` opt-out |
| `rob_box_quest/perf/logger_node.py` | ROS2 subscriber /quest/perf → jsonl |

### Тесты

```bash
# TypeScript
cd webxr_client
npm install
npx vitest run --reporter=verbose
npx tsc --noEmit

# Python (без ROS2 зависимостей)
cd ..
python3 -m pytest src/rob_box_quest/test/test_perf_logger.py -v --no-cov
```

### Acceptance (см. карточку)

- [x] Reporter работает в desktop browser (FPS / frame time виден в console)
- [x] Reporter работает в WebXR-сессии (FPS / GPU time шлётся в WSS)
- [x] WSS event `0x40 TELEMETRY_PERF` добавлен в API doc
- [x] ROS2 subscriber пишет в `/var/log/quest_perf/$(date).jsonl`
- [x] `?telemetry=off` отключает reporter
- [x] Memory leak check — SlidingWindow capacity=1024 + auto-trim
- [x] WSS reconnect через 5 с down → reporter продолжает после reconnect