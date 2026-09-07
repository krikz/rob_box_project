# План реализации: ADR-0054 — operator-agent Шаг 5а (wake stream)

> Документ для backend-воркера. ADR — `docs/adr/0054-operator-agent-step-5a-wake-stream.md`.
> Это **план**, не код. Каждый шаг — отдельный коммит внутри одного PR.

## 0. Контекст, который нужно прочесть до правок

1. `docs/adr/0054-operator-agent-step-5a-wake-stream.md` — этот шаг полностью.
2. `docs/architecture/target-operator-agent-and-dialogue.md` §7.2, §13 Шаг 5а,
   §11 инвариант 6a/6b (микрофоны не пересекаются; два выхода звука не
   смешиваются).
3. `docs/plans/2026-09-05-operator-agent-architecture-handoff.md` §4.9
   (что из старых документов действует) — особенно про `voice_input_mode`
   (удаляется) и ADR-0028 (действует в части floor и режимов).
4. Существующий код — прочесть в этом порядке:
   - `src/rob_box_quest/webxr_client/src/input/voice_capture.ts` (основа)
   - `src/rob_box_quest/webxr_client/src/main.ts:130-170` (PTT-логика)
   - `src/rob_box_quest/webxr_client/src/wire/connection.ts:218-235` (sendVoiceAudio)
   - `src/rob_box_quest/webxr_client/src/wire/protocol.ts` (FrameType)
   - `src/rob_box_quest/rob_box_quest/server/ws_server.py:444-470` (JSON_CMD обработка)
   - `src/rob_box_quest/rob_box_quest/server/ws_server.py:610-625` (VOICE_AUDIO приём)
   - `src/rob_box_quest/rob_box_quest/quest_node.py:240-300` (VOICE_AUDIO публикация)

## 1. Разбивка на коммиты

### Коммит 1 — `feat(quest-voice): voice_capture.ts — onChunk channel + setWakeGate + RMS VAD`

Файлы:
- `src/rob_box_quest/webxr_client/src/input/voice_capture.ts`
- `src/rob_box_quest/webxr_client/tests/voice_capture.test.ts`

Изменения:
- Экспорт `rmsInt16(pcm: Int16Array): number` — pure function.
- В `VoiceCaptureOptions` — `onChunk: (pcm, channel: "ptt"|"wake") => void`
  и опциональный `vad: { rmsThreshold?, hangoverMs? }`.
- В `createVoiceCapture` — внутреннее состояние `wakeGate`, `hangoverSamplesLeft`.
- В `onaudioprocess` — расчёт `rmsInt16`, обновление hangover,
  вызов `opts.onChunk(pcm, "ptt")` всегда (если голос включён в `applyVoicePtt`),
  и `opts.onChunk(pcm, "wake")` только при `wakeGate.enabled && !wakeGate.suppressed
  && speechActive`.
- Публичный метод `setWakeGate({enabled?, suppressed?})`.
- Тесты: см. §5.1 ADR.

Регрессия: `tests/voice_capture.test.ts` уже есть (см. `git log`), должен
проходить без изменений сигнатур.

### Коммит 2 — `feat(quest-wire): sendVoiceAudio streamId parameter (1|2)`

Файлы:
- `src/rob_box_quest/webxr_client/src/wire/connection.ts`
- `src/rob_box_quest/webxr_client/src/wire/protocol.ts` (без изменений, только проверка)
- `src/rob_box_quest/webxr_client/tests/protocol.test.ts`

Изменения:
- Сигнатура `sendVoiceAudio(payload: Uint8Array, streamId: 1 | 2 = 1)`.
- Тест: `encodeFrame(VOICE_AUDIO, 2, payload)` → `decodeFrame` → `streamId === 2`.

### Коммит 3 — `feat(quest-main): main.ts — wake по умолчанию + grip suppression`

Файлы:
- `src/rob_box_quest/webxr_client/src/main.ts`

Изменения:
- В колбэке `onChunk` — `conn.sendVoiceAudio(bytes, channel === "wake" ? 2 : 1)`.
- В WELCOME-обработчике (или сразу после создания `voiceCapture`) —
  `voiceCapture.setWakeGate({ enabled: true, suppressed: false })`.
- В `applyVoicePtt(radio, robot)` — добавить
  `voiceCapture.setWakeGate({ suppressed: radio || robot })`.
- В shutdown (`dispose`/`exitVr` ветки) — `voiceCapture.setWakeGate({enabled:false})`.

**Не делать в этом коммите:** UI-тумблер панели — отдельная маленькая
карточка. DoD шага 5а проверяется grep'ом `voice_listen` в `main.ts` —
команды не нужны для grep, нужно наличие логики подавления (что и
добавляется).

### Коммит 4 — `feat(quest-server): ws_server — voice_listen_start/stop + stream_id routing`

Файлы:
- `src/rob_box_quest/rob_box_quest/server/ws_server.py`
- `src/rob_box_quest/rob_box_quest/server/bridge.py` (или где лежит BridgeState)
- `src/rob_box_quest/test/unit/server/test_ws_server_voice.py`

Изменения:
- В `_on_json_cmd` после `voice_ptt_stop` (около `ws_server.py:459`):
  - `voice_listen_start` → `bridge.set_wake_stream_state(active=True)` + ack.
  - `voice_listen_stop` → `bridge.set_wake_stream_state(active=False)` + ack.
- В `_on_binary_frame` ветка `FrameType.VOICE_AUDIO` (`ws_server.py:618`):
  - `frame.stream_id == 2` → `bridge.publish_quest_wake_audio(payload)`.
  - Иначе (1 или 0) → `bridge.publish_audio_data(payload)` (текущее поведение).
- В `BridgeState` (или эквивалент) — поле `_wake_active: bool = False` и
  сеттер `set_wake_stream_state(active)`; опционально — публикация в
  `/avatar/wake_stream{state}` для observability.

**Что НЕ делать:** финальная маршрутизация wake-аудио в `stt_node` — это
**шаг 5**, не 5а. Метод `publish_quest_wake_audio` оставить как no-op с
`# TODO(step-5): подписчик появится в шаге 5` и **passing-test через mock**.

### Коммит 5 — `docs(adr-0054-impl): короткий пост-merge changelog`

Файлы:
- `docs/adr/0054-operator-agent-step-5a-wake-stream.md` — статус
  `Accepted` → `Implemented (see PR #N)`.
- `docs/architecture/target-operator-agent-and-dialogue.md` §13 — отметка
  «шаг 5а: implementation merged».

## 2. Что в коммитах НЕ делается

- ✗ Переход на `AudioWorklet` — отдельная карточка (шаг 5а-0). До её merge
  поток `wake` живёт **за флагом `?wake=1` в URL** для замера на устройстве;
  в прод-дефолте НЕ включается.
- ✗ Маршрутизация wake → stt_node → ТАРС — шаг 5.
- ✗ Обратный канал звука в шлем (`/avatar/tts/request`) — шаг 5б.
- ✗ Wake detector на клиенте — отклонён архитектурой §7.2.
- ✗ Список STT-искажений «ТАРС» — собирается по e2e-логам, не выдумывается.
- ✗ UI-тумблер панели — отдельная карточка; grep по `voice_listen` в `main.ts`
  DoD выполняется.

## 3. Регрессия и CI

- `pytest src/rob_box_quest/test/` — должен пройти. **Сырой -v output
  в PR-описании** (ADR-0018).
- `cd src/rob_box_quest/webxr_client && npm test` — должен пройти.
  **Сырой -v output в PR-описании.**
- Grep-проверки в PR-описании:
  - `git grep -n "voice_listen_start\|voice_listen_stop" src/`
  - `git grep -n "stream_id == 2\|streamId: 2" src/`
  - `git grep -n "setWakeGate" src/`

## 4. Замер на устройстве (DoD пункт 3)

**Без этого карточка не закрывается по DoD.** Если hardware-доступа нет —
вызвать `kanban_block kind=hardware-required` с конкретикой:

```
"Требуется SSH на vision (10.x.x.x) с правами docker exec в quest-build
контейнер; окно 2.5 ч непрерывно для замера:
 - battery_level через WebXR Device API или getBattery() — каждые 60 с в JSON
 - mediaStreamTrackStats — каждые 60 с в JSON
 - результат: git diff файл + adb logcat фрагмент
 Готовое место для логов: <path>"
```

**Не блокироваться** на абстрактное «нет доступа» — только с конкретной
формулировкой.

## 5. E2E-контракт (заполнить issue body)

В карточке реализации добавить блок `## e2e`:

```
## e2e
voice_text: "Робот, вруби музыку"
voice_file: .github/e2e/voice_commands/rabot_wake_phrase.ogg
volume: 150
record_seconds: 60
llm: minimax-m3
tts: minimax-male-qn-qingse
stt: yandex
```

Файл `rabot_wake_phrase.ogg` коммитится в карточке (`.github/e2e/voice_commands/`).
Если файл не приложен — e2e-process генерирует его сам в `ensure_voice_file`
через Yandex TTS, потом забираем с 249 (`/tmp/<name>.ogg`) и коммитим
в репо для последующих прогонов.

## 6. Чеклист «готово» (ADR-0018)

Перед `kanban complete`:

- [ ] pytest -v output в PR-описании (не «тесты прошли»)
- [ ] npm test output в PR-описании
- [ ] grep-выводы (см. §3) в PR-описании
- [ ] **Замер на устройстве** (батарея + getUserMedia 2 ч) — raw-цифры
      в PR-описании, или `kanban_block kind=hardware-required` с конкретикой
- [ ] E2E блок в issue body заполнен
- [ ] Closes #1992 — да, шаг 5а закрыт когда есть и код, и замер
