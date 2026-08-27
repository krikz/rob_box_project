# Voice Passthrough (рация) — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Правый grip в Quest → голос оператора играет на роботе без обработки (raw PCM). Сейчас — только рация, без супервизора.

**Architecture:** Quest mic (PCM 16k) → WS `VOICE_AUDIO` → quest-сервер → `/avatar/voice_in` → `sound_node` (стрим) → ReSpeaker. PTT start шлёт `STOP` (barge-in) в TTS/музыку. Супервизор/панель режимов — follow-up (P5).

**Tech Stack:** TypeScript/Three.js (клиент), Python/aiohttp (quest-сервер), ROS 2 Humble rclpy (sound_node), `AudioPlaybackManager` (sounddevice), `audio_common_msgs/AudioData`.

**Design doc:** `docs/plans/2026-08-27-quest-voice-passthrough-design.md`

**Порядок:** P1 → P2 → P3 → P4 = рация. P5–P8 = follow-up (после рации).

---

## Фаза P1 — sound_node: подписка /avatar/voice_in + стрим

### Task 1.1: Подписка на /avatar/voice_in (AudioData)

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/sound_node.py`
- Test: `src/rob_box_voice/test/unit/sound/test_sound_voice_passthrough.py` (NEW)

**Step 1: Тест** — в `SoundNode.__init__` есть `create_subscription(AudioData, "/avatar/voice_in", ...)`; QoS best-effort/volatile (чтобы не доигрывать stale-чанки). Импорт `AudioData` из `audio_common_msgs.msg`.

**Step 2: Реализовать** — подписка + заглушка callback (пока логирует длину чанка).

**Step 3: `python -m pytest src/rob_box_voice/test/unit/sound -v` → PASS. Commit:**
```bash
git commit -m "feat(voice): subscribe sound_node to /avatar/voice_in"
```

### Task 1.2: Стриминговое воспроизведение PCM

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/sound_node.py`
- Test: `src/rob_box_voice/test/unit/sound/test_sound_voice_passthrough.py`

**Step 1: Тест** — fake `sd.OutputStream`: первый чанк открывает stream (16k int16 mono), последующие пишутся в него; тишина > 300 мс закрывает stream. Пустой чанк не падает.

**Step 2: Реализовать** — в callback: `np.frombuffer(msg.data, np.int16)` → дублировать mono→stereo (`np.column_stack`, ReSpeaker требует 2 канала, как в tts_node) → stream.write; открытие/закрытие по таймеру тишины. НЕ использовать `play_audio` на каждый чанк (клики/лаг).

**Step 3: Commit:**
```bash
git commit -m "feat(voice): stream operator voice PCM in sound_node"
```

### Task 1.3: Координация с эффектами

**Step 1:** Тест — во время голосового стрима эффект-триггер не рвёт поток; `stop_playback` останавливает стрим.

**Step 2:** Реализовать — при активном голосовом стриме триггеры эффектов не перебивают поток (dmix-микс ALSA default, как сейчас у TTS+effects).

**Step 3: Commit:**
```bash
git commit -m "fix(voice): coordinate passthrough stream with sound effects"
```

---

## Фаза P2 — Клиент: mic + PTT + VOICE_AUDIO

### Task 2.1: Фрейм VOICE_AUDIO в codec

**Files:**
- Modify: `src/rob_box_quest/webxr_client/src/wire/protocol.ts`
- Test: `src/rob_box_quest/webxr_client/tests/protocol.test.ts`

**Step 1:** Добавить `VOICE_AUDIO = 0x13` в enum `FrameType` (client→server).

**Step 2:** Тест — encode/decode VOICE_AUDIO с сырым payload (int16 PCM) round-trip.

**Step 3:** `npm --prefix src/rob_box_quest/webxr_client test` → PASS. Commit.

### Task 2.2: Захват микрофона в PCM

**Files:**
- Create: `src/rob_box_quest/webxr_client/src/input/voice_capture.ts`
- Test: `src/rob_box_quest/webxr_client/tests/voice_capture.test.ts`

**Step 1:** Тест — `createVoiceCapture` отдаёт чанки int16 PCM; стоп освобождает getUserMedia-трек.

**Step 2:** Реализовать — `navigator.mediaDevices.getUserMedia({audio:{echoCancellation:true, noiseSuppression:true}})` + AudioWorklet/ScriptProcessor → resample 48k→16k, int16, чанки ~20 мс → callback.

**Step 3:** Commit.

### Task 2.3: PTT на правом grip + отправка PCM

**Files:**
- Modify: `src/rob_box_quest/webxr_client/src/input/teleop_config.ts` (добавить поле `pttButton`/handedness или отдельный биндинг)
- Modify: `src/rob_box_quest/webxr_client/src/main.ts`
- Modify: `src/rob_box_quest/webxr_client/src/wire/connection.ts` (sendBinary(VOICE_AUDIO))
- Test: `src/rob_box_quest/webxr_client/tests/xr_teleop.test.ts`

**Step 1:** Тест — правый grip (handedness=`right` + squeeze) → edge PTT start/stop; левый grip НЕ триггерит PTT (остаётся deadman).

**Step 2:** Реализовать — в `pollXrControllers` по `handedness` различать: `right` squeeze → `voice_ptt_start/stop` (JSON_CMD) + старт/стоп `voice_capture`; PCM → `conn.sendBinary(VOICE_AUDIO)`.

**Step 3:** Commit.

---

## Фаза P3 — quest-сервер: голосовой путь

### Task 3.1: VOICE_AUDIO в серверном frame.py

**Files:**
- Modify: `src/rob_box_quest/rob_box_quest/protocol/frame.py`
- Test: `src/rob_box_quest/test/unit/` (frame codec)

**Step 1:** Добавить `VOICE_AUDIO = 0x13`, decode в `ws_server` loop.

**Step 2:** Тест round-trip. Commit.

### Task 3.2: voice_ptt_start/stop + VOICE_AUDIO → /avatar/voice_in + STOP

**Files:**
- Modify: `src/rob_box_quest/rob_box_quest/server/ws_server.py`
- Test: `src/rob_box_quest/test/unit/test_ws_server_voice.py` (NEW)

**Step 1:** Тест — `voice_ptt_start`: публикует `STOP` в `/voice/tts/control` + `/voice/sound/stop`; `VOICE_AUDIO` → publish AudioData на `/avatar/voice_in`; `voice_ptt_stop` — поток закрывается.

**Step 2:** Реализовать — handler в `_on_json_cmd` + frame loop для 0x13; publish AudioData (int16 PCM 16k) в `/avatar/voice_in` (QoS best-effort). Без супервизора — прямой barge-in.

**Step 3:** `python -m pytest src/rob_box_quest/test/unit -q` → PASS. Commit.

---

## Фаза P4 — e2e на железе
- Deploy via GitHub Actions, ручной прогон: правый grip → голос слышен; raw-evidence (`docker logs quest`, `ros2 topic echo /avatar/voice_in`).

---

## Фаза P5 — (follow-up) Supervisor active + панель режимов + супервизор-агент
- Wire `acquire_floor`/`release_floor` (voice_floor) в active mode; STOP dispatch переезжает в супервизор.
- Супервизор-агент: LLM + набор инструментов (TTS/музыка/анимации/навигация), инструменты добавляем инкрементально.
- UI-панель на мостике: «я оператор» / «только я рулю» / «выключить диалог» / «голос-рация».
- См. `docs/adr/0028-avatar-supervisor.md`.

## Фаза P6 — (follow-up) Команда супервизору (голос/Telegram)
- Голос Quest и текст Telegram → один вход → супервизор-агент → действия («мотивируй народ» ≈ одинаково из обоих).
- Перед командой оператор выключает диалог на панели (личность не отвечает параллельно).

## Фаза P7 — (follow-up) Робот-голос (STT → LLM → TTS)
- Левый grip; пресеты стиля (технический/по понятиям/пещерный/деловой/философ/Ленин) + язык вывода; `voice_input_mode=quest_llm_formalize` (ADR-0027 §3.4).

### P7-simple (реализовано, 2026-08-27) — «робот говорит своим голосом»
Простой режим без пресетов: левый grip (PTT) → STT → LLM (личность робота) → TTS.
Маршрутизация — **через супервизор** (ADR-0028 S5), голосовой пайплайн остаётся
в `dialogue_node` (ADR-0028 S7):

```
Quest левый grip
  ├─ voice_mode {mode: ttts_proxy}      → ws_server → bridge → /avatar/set_voice_mode → supervisor (SetVoiceMode)
  ├─ voice_ptt_start {mode: robot_voice} → barge-in + буферизация PCM
  ├─ VOICE_AUDIO (int16 PCM 16k)        → /audio/quest_in (буфер)
  └─ voice_ptt_stop  {mode: robot_voice} → AudioData → /audio/quest_in → stt_node
stt_node: /audio/quest_in → /voice/stt/quest (отдельный топик, без маркеров)
dialogue_node: _on_quest_stt → voice_input_mode ∈ {quest_ttts, quest_stt} → _on_stt(from_quest=True) → LLM → TTS
```

Ключевые файлы:
- `dialogue_node.py`: `_on_quest_stt` + подписка `/voice/stt/quest` + `QUEST_STT_MODES`;
  `_on_stt(from_quest=…)` — источник задаёт флаг, а не текстовый маркер.
- `stt_node.py`: `/audio/quest_in` → `quest_result_pub` (`/voice/stt/quest`).
- `supervisor_node.py`: `_apply_voice_mode` + подписка `/avatar/set_voice_mode`
  (monitor → `applied=false`; active → `SetParameters` на `/dialogue_node/set_parameters`).
- `quest_node.py`/`ws_server.py`: `voice_mode` cmd + `WIRE_TO_VOICE_INPUT_MODE`.
- `webxr_client`: левый grip = `voice_mode(ttts_proxy)` + `voice_ptt_start/stop(robot_voice)`.

Осталось на P7-full: пресеты стиля + `quest_llm_formalize`; voice_floor-гейтинг
(сейчас один источник голоса — floor всегда свободен, design D5).

## Фаза P8 — (follow-up) Telegram voice message
- Скачать OGG → транскод int16 PCM 16k → publish `/avatar/voice_in`.

---

## Quality gates (каждая фаза)
- клиент: `npm --prefix src/rob_box_quest/webxr_client run typecheck` + `npm ... test` — GREEN
- python: `python -m pytest <изменённые test>` — GREEN; `black --check --line-length 120`; `flake8`
- raw-вывод в commit/PR (ADR-0018)
