# Voice Passthrough (рация) — правый grip → голос оператора на роботе

| Поле | Значение |
|---|---|
| Статус | Design (одобрено, 2026-08-27) |
| Дата | 2026-08-27 |
| Родители | ADR-0027 (Meta Quest/WebXR), ADR-0028 (Avatar Supervisor), ADR-0013 (incremental delivery) |
| Затрагивает | `rob_box_quest` (клиент + сервер), `rob_box_voice` (sound_node), (позже) `rob_box_supervisor`, `rob_box_telegram` |
| Out of scope (сейчас) | нашёптывание, робот-голос (STT→LLM→TTS), панель режимов — отдельные фичи, но архитектура их учитывает |

## 1. Цель и скоуп

Оператор в Quest зажимает **правый grip** и говорит — робот сразу, **без
обработки** (без STT/LLM/TTS), воспроизводит голос оператора через динамик
(ReSpeaker). Это «рация».

**Строим сейчас: только рацию.** Остальное — требования к архитектуре,
чтобы не переделывать рацию, когда добавим другие голосовые режимы.

### 1.1. Все голосовые режимы (архитектурная картина)

| Режим | Кнопка | Что делает | Статус |
|---|---|---|---|
| **рация** | правый grip | мой голос → динамик робота, без обработки | **сейчас** |
| **нашёптывание** | (позже) | мой голос → STT → `/voice/stt/result` как «обычный юзер»; dialogue НЕ знает, что вмешивается оператор | позже |
| **робот-голос** | левый grip | мой голос → STT → LLM(пресет+язык) → TTS | позже |
| **панель режимов** (супервизор) | UI | присутствие / только-я-рулю / мьют диалога | позже |

Ключевой принцип: весь голос Quest идёт в один PCM-хаб **`/avatar/voice_in`**,
а маршрутизация (куда дальше) — переключаемый `voice_mode`. Рация =
`voice_mode=passthrough` → `sound_node`. Нашёптывание = тот же PCM → STT →
`/voice/stt/result` (путь уже есть: `scenario_runner` инжектит туда же,
`dialogue_node._on_stt` не знает источник). Робот-голос = PCM → STT → LLM →
TTS. Ни один из них не ломает рацию.

## 2. Что уже есть (точки опоры)

- **ADR-0028 / `rob_box_supervisor`**: `LockManager` с двумя floor-ами
  `teleop_floor` и `voice_floor` + dead-man 500 мс; `ModeManager` FSM;
  нода `avatar_supervisor` (Phase 1 — **monitor**, сервисы
  `acquire_floor`/`release_floor` принимают, но `applied=false`).
- **Wire-протокол** (ADR-0027 §3.1): команды `voice_mode`,
  `voice_ptt_start`, `voice_ptt_stop` уже зарезервированы в JSON_CMD.
  Реального обработчика в `ws_server.py` нет.
- **Воспроизведение**: `AudioPlaybackManager` (синглтон + лок на
  `sounddevice`), используется `sound_node`/`tts_node`. Для голоса
  оператора расширяем `sound_node` подпиской на `/avatar/voice_in`.
- **`audio_common_msgs/AudioData`** — контейнер int16 LE PCM; в
  `tts_node` речь идёт как int16 PCM (16 kHz).
- **Прерывание (barge-in) уже есть**: wake-word → `stt_node` → `STOP` на
  `/voice/tts/control` → `tts_node` останавливает (`_interrupt_playback`,
  с immune-window / post-synth буфером, issue #1563). Музыка/эффекты —
  `STOP` на `/voice/sound/stop`. `audio_node` держит grace-период после
  TTS (анти-эхо, issue #989).

## 3. Архитектура / поток данных

```
Quest Browser (webxr_client)
  ├─ getUserMedia + AudioWorklet → int16 PCM 16 kHz mono (чанки ~20 мс)
  ├─ правый grip ↓ → JSON_CMD voice_ptt_start
  ├─ PCM-чанки → VOICE_AUDIO фреймы (client→server)
  └─ правый grip ↑ → JSON_CMD voice_ptt_stop

quest-сервер (Vision Pi, aiohttp WSS, порт 8765 / Caddy 8443)
  ├─ voice_ptt_start → STOP в /voice/tts/control + /voice/sound/stop (barge-in)
  ├─ VOICE_AUDIO → publish AudioData → /avatar/voice_in
  └─ voice_ptt_stop → конец потока

avatar_supervisor (Vision Pi) — позже (follow-up P5)
  └─ раздаёт voice_floor / teleop_floor + панель режимов

sound_node (Vision Pi, РАСШИРЕНИЕ, P2)
  ├─ подписка /avatar/voice_in (AudioData, int16 PCM 16k)
  └─ стриминговый вывод PCM → ReSpeaker (через AudioPlaybackManager/dmix)
```

Ключевой принцип: **супервизор не трогает звук** (ADR-0028 S7), он только
раздаёт `voice_floor` (follow-up). Пока второго источника голоса нет —
quest-сервер публикует PCM в `/avatar/voice_in` напрямую. Плейер (sound_node)
тупой — играет всё, что приходит.

Telegram-голосовое (P8, позже): скачал OGG → транскод в int16 PCM 16k →
публикация в тот же `/avatar/voice_in`. Та же способность, тот же контракт.

## 4. Ключевые решения

| # | Решение | Почему |
|---|---|---|
| D1 | Формат: **int16 PCM, 16 kHz, mono** | совпадает с `AudioData`; «без обработки»; на LAN 32 KB/s — дёшево; нет транскода на сервере |
| D2 | Новый фрейм **`VOICE_AUDIO = 0x13`** (client→server), payload = сырой int16 PCM | существующий BINARY_FRAME (0x10) — только server→client; нужен обратный канал |
| D3 | PTT: JSON_CMD `voice_ptt_start`/`voice_ptt_stop` (уже в ADR-0027) | не вводим новый cmd-тип |
| D4 | Воспроизведение: **расширяем `sound_node`** (Option B) подпиской на `/avatar/voice_in` | без новой ноды; голос оператора и эффекты в одном месте (осознанно смешиваем) |
| D5 | Супервизор + панель режимов — **follow-up (P5)**, не блокирует рацию | пока один источник голоса — floor всегда свободен; вводим когда появится Telegram-голос |
| D6 | Левый grip = STT→TTS — **не в этой фиче** | отдельная фича (голос робота), свой цикл |
| D7 | Голос — **поток**, не one-shot: держим `sd.OutputStream` открытым пока идёт PTT, чанки пишем в него | `play_audio` на каждый 20мс чанк = клики + лаг; поток даёт низкую задержку |

## 5. Безопасность / edge-cases

- Отпустил grip → `voice_ptt_stop`; если клиент замолчал без release —
  dead-man супервизора (500 мс) снимает `voice_floor`.
- `voice_floor` занят (говорит Telegram) → quest-сервер дропает PCM и шлёт
  клиенту `voice_state(denied)` (event, для индикации в UI).
- Два голоса одновременно невозможны: публикует только держатель floor.
- Latency target: < 300 мс в обе стороны (LAN, PCM без сжатия).

### 5.1 Конфликт с диалогом (робот говорит, оператор вклинивается)

Что уже воспроизводит: `tts_node` (речь) и `sound_node` (эффекты/музыка),
оба через `AudioPlaybackManager`. Готовый barge-in: `STOP` на
`/voice/tts/control` прерывает TTS. На выдаче `voice_floor` супервизор
(dispatcher) публикует:

- `STOP` → `/voice/tts/control` — прервать речь робота;
- `STOP` → `/voice/sound/stop` — прервать музыку/эффекты.

После release `voice_floor` `dialogue_node` возвращается к обычной работе.
ReSpeaker может услышать голос оператора как wake-word/речь — эхо гасится
grace-периодом `audio_node` (`tts_grace_s`) + подавлением VAD на время
passthrough (уточняется в P1/P4).

## 6. Фазы (ADR-0013 — инкрементально)

**Сначала рация:** P1 → P2 → P3 → P4. Остальное — follow-up после рации.

| Фаза | Что | Ценность |
|---|---|---|
| **P1** | `sound_node`: подписка `/avatar/voice_in` + стриминговый вывод PCM | слышно PCM |
| **P2** | Клиент: getUserMedia+AudioWorklet PCM, правый grip PTT, фрейм VOICE_AUDIO | захват голоса |
| **P3** | quest-сервер: voice_ptt_start/stop + VOICE_AUDIO → `/avatar/voice_in` + STOP barge-in | сквозной путь Quest |
| **P4** | e2e на железе (grip → слышно голос) | proof |
| **P5** | (follow-up) Supervisor active + панель режимов (присутствие/рулю/мьют) | гейтинг + UI |
| **P6** | (follow-up) Нашёптывание: PCM → STT → `/voice/stt/result` | команды «как юзер» |
| **P7** | (follow-up) Робот-голос: STT → LLM(пресет+язык) → TTS | стилизация |
| **P8** | (follow-up) Telegram voice message → `/avatar/voice_in` | переиспользование |

## 7. Тестирование

- P1: unit — fake `AudioPlaybackManager` (sd=None уже в CI) + тест стрима/буфера тишины.
- P2: unit — codec VOICE_AUDIO + fake mic-буфер; правый-grip edge.
- P3: unit — voice_ptt handler: STOP barge-in + publish PCM.
- P4: ручной e2e, raw-evidence (`docker logs quest`, `ros2 topic echo /avatar/voice_in`).

## 8. Файлы (ориентир)

- `src/rob_box_voice/rob_box_voice/sound_node.py` (P1, подписка /avatar/voice_in + стрим)
- `src/rob_box_quest/webxr_client/src/input/voice_ptt.ts` (P2, NEW)
- `src/rob_box_quest/webxr_client/src/wire/protocol.ts` (P2, VOICE_AUDIO)
- `src/rob_box_quest/rob_box_quest/protocol/frame.py` (P3, VOICE_AUDIO)
- `src/rob_box_quest/rob_box_quest/server/ws_server.py` (P3)
- `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py` (P5, follow-up)
- `src/rob_box_telegram/rob_box_telegram/...` (P8, позже)
