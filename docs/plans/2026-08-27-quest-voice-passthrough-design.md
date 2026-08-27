# Voice Passthrough (рация) — правый grip → голос оператора на роботе

| Поле | Значение |
|---|---|
| Статус | Design (одобрено, 2026-08-27) |
| Дата | 2026-08-27 |
| Родители | ADR-0027 (Meta Quest/WebXR), ADR-0028 (Avatar Supervisor), ADR-0013 (incremental delivery) |
| Затрагивает | `rob_box_quest` (клиент + сервер), `rob_box_supervisor` (voice_floor active), новый плейер, (позже) `rob_box_telegram` |
| Out of scope | левый grip = STT→TTS голосом робота — ОТДЕЛЬНАЯ фича |

## 1. Цель

Оператор в Quest зажимает **правый grip** и говорит — робот сразу, **без
обработки** (без STT/LLM/TTS), воспроизводит голос оператора через динамик
(ReSpeaker). Это «рация». Та же способность должна быть переиспользована
Telegram-голосовыми сообщениями, поэтому аудио-путь гейтится супервизором
(`voice_floor`), а не является частной логикой Quest-клиента.

## 2. Что уже есть (точки опоры)

- **ADR-0028 / `rob_box_supervisor`**: `LockManager` с двумя floor-ами
  `teleop_floor` и `voice_floor` + dead-man 500 мс; `ModeManager` FSM;
  нода `avatar_supervisor` (Phase 1 — **monitor**, сервисы
  `acquire_floor`/`release_floor` принимают, но `applied=false`).
- **Wire-протокол** (ADR-0027 §3.1): команды `voice_mode`,
  `voice_ptt_start`, `voice_ptt_stop` уже зарезервированы в JSON_CMD.
  Реального обработчика в `ws_server.py` нет.
- **Воспроизведение**: `AudioPlaybackManager` (синглтон + лок на
  `sounddevice`), используется `sound_node`/`tts_node`. Отдельной ноды
  «сыграть сырой PCM из топика» нет.
- **`audio_common_msgs/AudioData`** — контейнер int16 LE PCM; в
  `tts_node` речь идёт как int16 PCM (16 kHz).

## 3. Архитектура / поток данных

```
Quest Browser (webxr_client)
  ├─ getUserMedia + AudioWorklet → int16 PCM 16 kHz mono (чанки ~20 мс)
  ├─ правый grip ↓ → JSON_CMD voice_ptt_start
  ├─ PCM-чанки → VOICE_AUDIO фреймы (client→server)
  └─ правый grip ↑ → JSON_CMD voice_ptt_stop

quest-сервер (Vision Pi, aiohttp WSS, порт 8765 / Caddy 8443)
  ├─ voice_ptt_start → supervisor.acquire_floor(voice_floor)
  │     denied → НЕ публикуем, шлём клиенту voice_state(denied)
  ├─ VOICE_AUDIO → publish AudioData → /avatar/voice_in
  └─ voice_ptt_stop → supervisor.release_floor(voice_floor)

avatar_supervisor (Vision Pi) — active mode (P1)
  └─ раздаёт voice_floor / teleop_floor, публикует /avatar/state

avatar_voice_player (Vision Pi, NEW, P2)
  ├─ подписка /avatar/voice_in (AudioData)
  └─ AudioPlaybackManager.play_audio(...) → ReSpeaker
```

Ключевой принцип: **супервизор не трогает звук** (ADR-0028 S7), он только
раздаёт `voice_floor`. Гейтинг — на стороне издателя: публиковать PCM в
`/avatar/voice_in` может только клиент, удерживающий `voice_floor`. Плейер
тупой — играет всё, что приходит.

Telegram-голосовое (P5, позже): скачал OGG → транскод в int16 PCM 16k →
`acquire_floor(voice)` → публикация в тот же `/avatar/voice_in`. Та же
способность, тот же контракт.

## 4. Ключевые решения

| # | Решение | Почему |
|---|---|---|
| D1 | Формат: **int16 PCM, 16 kHz, mono** | совпадает с `AudioData`; «без обработки»; на LAN 32 KB/s — дёшево; нет транскода на сервере |
| D2 | Новый фрейм **`VOICE_AUDIO = 0x13`** (client→server), payload = сырой int16 PCM | существующий BINARY_FRAME (0x10) — только server→client; нужен обратный канал |
| D3 | PTT: JSON_CMD `voice_ptt_start`/`voice_ptt_stop` (уже в ADR-0027) | не вводим новый cmd-тип |
| D4 | Воспроизведение: **новая нода `avatar_voice_player`** (Option A) | не смешиваем «эффекты» sound_node и «голос оператора»; проще гейтить и тестировать |
| D5 | Супервизор — **active mode для voice_floor** (P1) | FSM/locks уже написаны и покрыты тестами; осталось связать с сервисами |
| D6 | Левый grip = STT→TTS — **не в этой фиче** | отдельная фича (голос робота), свой цикл |

## 5. Безопасность / edge-cases

- Отпустил grip → `voice_ptt_stop`; если клиент замолчал без release —
  dead-man супервизора (500 мс) снимает `voice_floor`.
- `voice_floor` занят (говорит Telegram) → quest-сервер дропает PCM и шлёт
  клиенту `voice_state(denied)` (event, для индикации в UI).
- Два голоса одновременно невозможны: публикует только держатель floor.
- Latency target: < 300 мс в обе стороны (LAN, PCM без сжатия).

## 6. Фазы (ADR-0013 — инкрементально)

| Фаза | Что | Ценность |
|---|---|---|
| **P1** | Supervisor: wire `acquire_floor`/`release_floor` (voice_floor) в active mode | фундамент гейтинга |
| **P2** | Нода `avatar_voice_player`: `/avatar/voice_in` → AudioPlaybackManager | слышно PCM |
| **P3** | Клиент: getUserMedia+AudioWorklet PCM, правый grip PTT, фрейм VOICE_AUDIO | захват голоса |
| **P4** | quest-сервер: voice_ptt_start/stop + VOICE_AUDIO → `/avatar/voice_in` | сквозной путь Quest |
| **P5** | Telegram voice message → тот же `/avatar/voice_in` | переиспользование |
| **P6** | e2e на железе (grip → слышно голос) | proof |

## 7. Тестирование

- P1: unit — LockManager/FSM уже покрыты; добавить тест сервисов active-mode.
- P2: unit — fake `AudioPlaybackManager` (sd=None уже поддерживается в CI).
- P3: unit — codec VOICE_AUDIO + fake mic-буфер; правый-grip edge в FSM.
- P4: unit — voice_ptt handler: granted → publish, denied → drop + event.
- P6: ручной e2e, raw-evidence (`docker logs quest`, `ros2 topic echo /avatar/voice_in`).

## 8. Файлы (ориентир)

- `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py` (P1)
- `src/rob_box_voice/rob_box_voice/avatar_voice_player.py` (P2, NEW) + package export
- `src/rob_box_quest/webxr_client/src/input/voice_ptt.ts` (P3, NEW)
- `src/rob_box_quest/webxr_client/src/wire/protocol.ts` (P3, VOICE_AUDIO)
- `src/rob_box_quest/rob_box_quest/protocol/frame.py` (P4, VOICE_AUDIO)
- `src/rob_box_quest/rob_box_quest/server/ws_server.py` (P4)
- `src/rob_box_telegram/rob_box_telegram/...` (P5, позже)
