# Voice Passthrough (рация) — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Правый grip в Quest → голос оператора играет на роботе без обработки (raw PCM), гейтится `voice_floor` супервизора.

**Architecture:** Quest mic (PCM 16k) → WS `VOICE_AUDIO` → quest-сервер → (voice_floor) → `/avatar/voice_in` → нода `avatar_voice_player` → ReSpeaker. Супервизор только раздаёт floor.

**Tech Stack:** TypeScript/Three.js (клиент), Python/aiohttp (quest-сервер), ROS 2 Humble rclpy (supervisor + player), `AudioPlaybackManager` (sounddevice), `audio_common_msgs/AudioData`.

**Design doc:** `docs/plans/2026-08-27-quest-voice-passthrough-design.md`

---

## Фаза P1 — Supervisor voice_floor в active mode

### Task 1.1: Сервисы acquire/release floor в active mode

**Files:**
- Modify: `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py`
- Test: `src/rob_box_supervisor/test/unit/test_supervisor_node.py`

**Step 1: Прочитать текущий контракт сервисов** — сейчас `std_srvs/Trigger`, mode=`monitor` → `applied=false`. Определить: в `active` mode `acquire_floor`/`release_floor` должны реально вызывать `LockManager.acquire/release` с `client_id` (из body сервиса) и `floor` (из имени/тела).

**Step 2: Написать падающий тест** — в `test_supervisor_node.py` добавить кейс: нода с `mode=active` + мок `LockManager`; вызов `acquire_floor(voice_floor)` возвращает `success=true, applied=true`; повторный от другого клиента → `ConflictError` → `applied=false` с причиной.

**Step 3: Прогнать тест** — `python -m pytest src/rob_box_supervisor/test/unit/test_supervisor_node.py -v` → FAIL.

**Step 4: Реализовать** — wire `LockManager` (уже в `core/locks.py`) в сервисы: при `mode=active` — `acquire(client_id, floor)`; в monitor — текущее поведение. Не менять сигнатуры IDL (пока Trigger-совместимо), `client_id`/`floor` передавать в request-строке или добавить минимальный кастомный IDL по ADR-0028 AV-5.

**Step 5: Прогнать** — `python -m pytest src/rob_box_supervisor/test/unit -v` → PASS (все существующие + новый).

**Step 6: Commit**
```bash
git add src/rob_box_supervisor/
git commit -m "feat(supervisor): wire acquire/release floor in active mode"
```

### Task 1.2: Публикация voice_floor в /avatar/state

**Files:**
- Modify: `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py`
- Test: `src/rob_box_supervisor/test/unit/test_supervisor_node.py`

**Step 1:** Тест — после `acquire_floor(voice_floor)` published `/avatar/state` содержит `voice_floor=<client_id>`.

**Step 2:** Реализовать — `LockManager.holder(floor)` → pack в `AvatarState` (core/state.py уже есть) → publish.

**Step 3:** Покрыть, commit:
```bash
git commit -m "feat(supervisor): publish voice_floor in /avatar/state"
```

---

## Фаза P2 — sound_node: подписка /avatar/voice_in + стрим

### Task 2.1: Подписка на /avatar/voice_in (AudioData)

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/sound_node.py`
- Test: `src/rob_box_voice/test/unit/sound/test_sound_voice_passthrough.py` (NEW)

**Step 1: Тест** — в `SoundNode.__init__` есть `create_subscription(AudioData, "/avatar/voice_in", ...)`; QoS best-effort/volatile (чтобы не доигрывать stale-чанки). Импорт `AudioData` из `audio_common_msgs.msg`.

**Step 2: Реализовать** — подписка + заглушка callback (пока логирует длину чанка).

**Step 3: `python -m pytest src/rob_box_voice/test/unit/sound -v` → PASS. Commit:**
```bash
git commit -m "feat(voice): subscribe sound_node to /avatar/voice_in"
```

### Task 2.2: Стриминговое воспроизведение PCM

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/sound_node.py`
- Test: `src/rob_box_voice/test/unit/sound/test_sound_voice_passthrough.py`

**Step 1: Тест** — fake `sd.OutputStream`: первый чанк открывает stream (16k int16 mono), последующие пишутся в него; тишина > 300 мс закрывает stream. Пустой чанк не падает.

**Step 2: Реализовать** — в callback: `np.frombuffer(msg.data, np.int16)` → stream.write; открытие/закрытие по таймеру тишины. НЕ использовать `play_audio` на каждый чанк (клики/лаг).

**Step 3: Commit:**
```bash
git commit -m "feat(voice): stream operator voice PCM in sound_node"
```

### Task 2.3: Координация с эффектами

**Step 1:** Тест — во время голосового стрима эффект-триггер не рвёт поток; `stop_playback` останавливает стрим.

**Step 2:** Реализовать — при активном голосовом стриме триггеры эффектов не перебивают поток (dmix-микс ALSA default, как сейчас у TTS+effects).

**Step 3: Commit:**
```bash
git commit -m "fix(voice): coordinate passthrough stream with sound effects"
```

---

## Фаза P3 — Клиент: mic + PTT + VOICE_AUDIO

### Task 3.1: Фрейм VOICE_AUDIO в codec

**Files:**
- Modify: `src/rob_box_quest/webxr_client/src/wire/protocol.ts`
- Test: `src/rob_box_quest/webxr_client/tests/protocol.test.ts`

**Step 1:** Добавить `VOICE_AUDIO = 0x13` в enum `FrameType` (client→server).

**Step 2:** Тест — encode/decode VOICE_AUDIO с сырым payload (int16 PCM) round-trip.

**Step 3:** `npm --prefix src/rob_box_quest/webxr_client test` → PASS. Commit.

### Task 3.2: Захват микрофона в PCM

**Files:**
- Create: `src/rob_box_quest/webxr_client/src/input/voice_capture.ts`
- Test: `src/rob_box_quest/webxr_client/tests/voice_capture.test.ts`

**Step 1:** Тест — `createVoiceCapture` отдаёт чанки int16 PCM; стоп освобождает getUserMedia-трек.

**Step 2:** Реализовать — `navigator.mediaDevices.getUserMedia({audio:{echoCancellation:true, noiseSuppression:true}})` + AudioWorklet/ScriptProcessor → resample 48k→16k, int16, чанки ~20 мс → callback.

**Step 3:** Commit.

### Task 3.3: PTT на правом grip + отправка PCM

**Files:**
- Modify: `src/rob_box_quest/webxr_client/src/input/teleop_config.ts` (добавить поле `pttButton`/handedness или отдельный биндинг)
- Modify: `src/rob_box_quest/webxr_client/src/main.ts`
- Modify: `src/rob_box_quest/webxr_client/src/wire/connection.ts` (sendBinary(VOICE_AUDIO))
- Test: `src/rob_box_quest/webxr_client/tests/xr_teleop.test.ts`

**Step 1:** Тест — правый grip (handedness=`right` + squeeze) → edge PTT start/stop; левый grip НЕ триггерит PTT (остаётся deadman).

**Step 2:** Реализовать — в `pollXrControllers` по `handedness` различать: `right` squeeze → `voice_ptt_start/stop` (JSON_CMD) + старт/стоп `voice_capture`; PCM → `conn.sendBinary(VOICE_AUDIO)`.

**Step 3:** Commit.

---

## Фаза P4 — quest-сервер: голосовой путь

### Task 4.1: VOICE_AUDIO в серверном frame.py

**Files:**
- Modify: `src/rob_box_quest/rob_box_quest/protocol/frame.py`
- Test: `src/rob_box_quest/test/unit/` (frame codec)

**Step 1:** Добавить `VOICE_AUDIO = 0x13`, decode в `ws_server` loop.

**Step 2:** Тест round-trip. Commit.

### Task 4.2: voice_ptt_start/stop → supervisor + publish PCM

**Files:**
- Modify: `src/rob_box_quest/rob_box_quest/server/ws_server.py`
- Test: `src/rob_box_quest/test/unit/test_ws_server_voice.py` (NEW)

**Step 1:** Тест — `voice_ptt_start`: acquire voice_floor granted → start; denied → drop + `voice_state(denied)` event. `VOICE_AUDIO` → publish AudioData на `/avatar/voice_in`. `voice_ptt_stop` → release.

**Step 2:** Реализовать — handler в `_on_json_cmd` + frame loop для 0x13; supervisor client (reuse паттерн `rob_box_telegram/supervisor_client.py`).

**Step 3:** `python -m pytest src/rob_box_quest/test/unit -q` → PASS. Commit.

---

## Фаза P5 — Telegram voice message (позже)
- Скачать OGG → транскод в int16 PCM 16k → `acquire_floor(voice)` → publish `/avatar/voice_in`. Отдельная карточка.

## Фаза P6 — e2e на железе
- Deploy via GitHub Actions, ручной прогон: правый grip → голос слышен; raw-evidence (`docker logs quest`, `ros2 topic echo /avatar/voice_in`).

---

## Quality gates (каждая фаза)
- клиент: `npm --prefix src/rob_box_quest/webxr_client run typecheck` + `npm ... test` — GREEN
- python: `python -m pytest <изменённые test>` — GREEN; `black --check --line-length 120`; `flake8`
- raw-вывод в commit/PR (ADR-0018)
