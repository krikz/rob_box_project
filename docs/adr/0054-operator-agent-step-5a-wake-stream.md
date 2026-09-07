# ADR-0054: operator-agent Шаг 5а — всегда-включённый поток `stream_id=wake` (клиентский VAD-гейт + `voice_listen_*`)

| Поле | Значение |
|---|---|
| Статус | Accepted (architectural design) — implementation карточки отдельная |
| Дата | 2026-09-07 |
| Автор | architect profile (Claude, сессия kanban t_97a82c5a) |
| Реализует | issue #1992, шаг **5а** плана миграции `target-operator-agent-and-dialogue.md §13` |
| Зависит от | шаг **5а-0** (`AudioWorklet` — отдельная карточка), шаг **5** (wake router в `stt_node`) |
| Разблокирует | шаги **5б** (обратный канал звука в шлем), **6** (`/dialogue/control`) и весь дальнейший план — без always-on шлема ТАРС доступен только по PTT |
| Родители | ADR-0051 (supervisor/operator-agent/arbiter split), `target-operator-agent-and-dialogue.md` §7.2, §13 Шаг 5а; хендофф 2026-09-05 §4.9 (что из старых документов действует) |
| Связанные | ADR-0018 (честный FAIL лучше красивого PASS — протокольные DoD ниже включают raw-evidence), ADR-0013 (incremental delivery, revert-ветка) |

## 0. Что внутри и что — нет

**Внутри этого ADR.** Контракт `stream_id=wake` (что в кадре, какие команды, приоритет над `ptt`), клиентский VAD-гейт, реализация в `voice_capture.ts` (без второго `getUserMedia`), серверная обработка, подавление при грипе, тумблер в панели, тест-план, e2e-контракт.

**Не внутри.** Переход на `AudioWorklet` (шаг 5а-0 — отдельная карточка, ADR/Issue создаётся воркером backend). Маршрутизация вейка на стороне `stt_node` (шаг 5 — отдельная карточка). Wake router по `config/wake_words.yaml namespace=operator` — ADR-0052 уже зафиксировал SSoT. Пайплайн грипа (шаг 4б) — уже существует и не меняется.

## 1. Контекст и бизнес-проблема

### 1.1 Что сейчас

Сегодня микрофон шлема стримится **только пока зажат грип**:
`voiceCapture.start()` вызывается в `applyVoicePtt` (`src/rob_box_quest/webxr_client/src/main.ts:155-170`)
и `voiceCapture.stop()` при отпускании. То есть робот слышит оператора
ровно столько, сколько оператор держит грип. Вейк-слово «ТАРС» на таком
транспорте физически невозможно.

Сервер (`src/rob_box_quest/rob_box_quest/server/ws_server.py:444-459`) принимает
только `voice_ptt_start/stop{mode}` и `voice_mode` — никакого «всегда-включён»
контура нет.

`voice_capture.ts:58-134` использует `createScriptProcessor(4096,1,1)` —
устаревший API, работающий в **главном потоке**. Для PTT это терпимо: захват
живёт секунды. Для всегда-включённого потока он будет конкурировать с
рендер-циклом three.js в VR.

### 1.2 Что требуется

Целевая архитектура (`target-operator-agent-and-dialogue.md §7.2`) требует
**второй поток с того же захвата** (`stream_id=2`), отличаемый по `stream_id`,
с локальным VAD-гейтом (тишина не шлётся), командами `voice_listen_start/stop`
(по умолчанию включён при HELLO, выключается тумблером панели), и подавлением
потока `wake` при зажатом грипе (одна фраза — один маршрут).

| поток | `stream_id` | когда идёт | назначение |
|---|---|---|---|
| `ptt` | `1` | пока зажат левый грип | фраза целиком, вейк не нужен |
| `wake` | `2` | пока сессия жива, гейт по локальному VAD | поиск «ТАРС» в `stt_node` |

### 1.3 Почему VAD-гейт на клиенте, а не на сервере

| вариант | почему отклонён |
|---|---|
| Шлём всё, гейтим на сервере | Удвоение трафика WS → Quest по WiFi 5 GHz, лишние ~32 КБ/с/поток int16 PCM 16 kHz; серверный VAD уже есть (`audio_node.py` `_vad_gated`), но он отбрасывает для `/audio/vad`, а не для шлема |
| Wake detector на клиенте (Porcupine/openWakeWord/vosk-browser) | См. §7.2 архитектуры: Porcupine — нет русского, openWakeWord — нет web-сборки, vosk-browser — это STT, не keyword-spotter. **Главное**: на роботе уже крутится STT с русским и списком вейк-слов, там «ТАРС» добавляется строкой в YAML |
| **Клиентский RMS-VAD** (этот ADR) | Дешёво (O(N) на чанк 320 семплов), нативный `AudioWorklet`-ready, не зависит от языка, не требует модели |

Клиент режет тишину **до** WS-отправки — `stt_node` получает только озвученные
сегменты, и внутри них уже ищет вейк-слова по своему YAML.

### 1.4 Почему `stream_id`, а не два фрейм-типа

Wire-протокол (`frame.py:33`, `protocol.ts:91`) уже кодирует 1 байт type + 4 байта
`stream_id`. Один `FrameType.VOICE_AUDIO = 0x13` с разными `stream_id` —
минимальное расширение, не требует нового кода на сервере и не ломает
существующих подписчиков (`ws_server.py:618` уже принимает `VOICE_AUDIO`
по любому `stream_id`).

## 2. Решение

### 2.1 Поток `stream_id=2` (`wake`)

```
int16 PCM 16 kHz mono, чанки 320 семплов (20 мс),
gate перед отправкой:
  rms(чанк) ≥ VAD_RMS_THRESHOLD   → отправить как VOICE_AUDIO(0x13, stream_id=2, payload=чанк)
  rms(чанк) <  VAD_RMS_THRESHOLD   → drop
HDR (выкл/вкл) + паника при грипе — см. 2.4
```

Дефолтные параметры VAD (raw, не выдуманы — взяты из существующего
`audio_node.py` strict-VAD §564-573):

```python
# rob_box_voice/rob_box_voice/audio_node.py:564-573 — strict-VAD под музыку
# (используем как референс; конкретные числа под шлем подбираются на устройстве)
VAD_RMS_THRESHOLD = 0.01   # int16 единиц, отнормировано к [-32768..32767]
VAD_HANGOVER_MS   = 200    # после детектирования речи — ещё 200 мс шлём,
                           # чтобы не рвать слоги внутри одной фонетической группы
```

**Эти числа — стартовая точка, а не догмат.** Под шлемом нужен замер на
устройстве (см. §6 DoD — «замер батареи и стабильности getUserMedia ≥ 2 ч»).

### 2.2 Команды `voice_listen_start` / `voice_listen_stop`

Семантика — **включение/выключение always-on потока целиком**, не per-чанк:

| команда | эффект на клиенте | эффект на сервере |
|---|---|---|
| `voice_listen_start` (default при HELLO) | если mic ещё не запущен — `getUserMedia`; `wakeEnabled = true`; чанки из VAD идут в WS с `stream_id=2` | публикация `/avatar/wake_stream{state:"active"}` для observability (по аналогии с `voice_ptt_start`) |
| `voice_listen_stop` | `wakeEnabled = false`; чанки дропаются **до** VAD; mic **не закрывается**, если PTT ещё активен | `/avatar/wake_stream{state:"paused"}` |

**Граничный случай.** Если `voice_listen_stop` пришёл во время активного PTT,
PTT продолжает работать (у него свой `stream_id=1`). Если `voice_listen_stop`
пришёл, а PTT не активен — клиент НЕ закрывает `getUserMedia` (на горячую это
дороже, чем держать поток; см. §4 trade-off). Когда оба стопа — тогда закрываем.

**Дефолт при HELLO.** Поток `wake` включается автоматически — иначе оператору
пришлось бы каждый раз лезть в панель. Это согласовано с §7.2: «по умолчанию
включён при HELLO, выключается тумблером панели».

### 2.3 Подавление `wake` при зажатом грипе

Одна фраза не должна попасть и в пайплайн грипа (`stream_id=1`), и в агента
(`stream_id=2` — wake). Решение — **client-side gate на уровне стрима**:

```typescript
// в main.ts, рядом с applyVoicePtt:
function applyWakeSuppression(suppress: boolean): void {
  if (suppress === wakeSuppressed) return;
  wakeSuppressed = suppress;
  // suppress=true  → wakeEnabled остаётся true, но чанки дропаются
  //                  ДО VAD-гейта (если подавлять ПОСЛЕ VAD, на сервер всё
  //                  равно уйдут PCM-кадры — а нам ровно это и нельзя).
  // suppress=false → VAD снова гейтит и шлёт.
}
```

Триггер — `applyVoicePtt(radio, robot)` в `main.ts:155-170`. Если `radio ||
robot` (любой грип зажат) → `applyWakeSuppression(true)`. Если оба отпущены →
`applyWakeSuppression(false)`.

**Почему НЕ server-side.** Серверная сторона не знает, что grip зажат — она
получает только `voice_ptt_start/stop`. Если давить на сервере, между кликом
грипа и `voice_ptt_start` есть окно ~50 мс (`applyVoicePtt` шлёт команду в
том же frame'е, что и mic.start). Это окно — лишние чанки `wake` за тот же
фрагмент фразы. Подавление **на клиенте, до отправки** — единственный способ
гарантировать «одна фраза — один маршрут».

### 2.4 Тумблер в панели (`voice_pipeline_panel.ts`)

По §7.2 «выключается тумблером панели». Минимальное расширение:

```
+----------------------------------------+
|  Голос: 🎤 шлем                      |
|  [✓] Всегда слушать wake              |  ← новый чекбокс
|  [✓] Левый грип → робот-голос         |  (существующее)
|  [✓] Правый грип → рация              |  (существующее)
+----------------------------------------+
```

При изменении тумблера панель шлёт:

```json
JSON_CMD{cmd: "voice_listen_start"}   // чекбокс включён
JSON_CMD{cmd: "voice_listen_stop"}    // чекбокс выключен
```

(Состояние сохраняется в localStorage — для следующих сессий; адресовано в
маленькой follow-up карточке backend, не блокирует DoD шага 5а.)

### 2.5 Один `getUserMedia`, два потока

`voice_capture.ts:58-134` сегодня возвращает **один** `VoiceCapture` с
единственным `onChunk`. Расширение без поломки API:

```typescript
export interface VoiceCaptureOptions {
  onChunk: (pcm: Int16Array, channel: "ptt" | "wake") => void;  // +channel
  onError?: (err: Error) => void;
  deps?: Partial<VoiceCaptureDeps>;
  vad?: { rmsThreshold?: number; hangoverMs?: number };          // +VAD config
}

export interface VoiceCapture {
  start(): Promise<void>;
  stop(): void;
  isCapturing(): boolean;
  // расширение для шага 5а:
  setWakeGate(opts: { enabled: boolean; suppressed: boolean }): void;
}
```

`setWakeGate` вызывается из `main.ts` при (a) HELLO-дефолте → `{enabled:true, suppressed:false}`,
(b) тумблере панели → `{enabled:false, suppressed:false}`,
(c) грипе → `{enabled:true, suppressed:true}` (enabled остаётся true,
gate прокинут внутрь).

Тест в `tests/voice_capture.test.ts` уже есть — расширяется без ломки (см. §5).

### 2.6 Серверная сторона: `ws_server.py` принимает `stream_id`

Текущий код (`ws_server.py:618`):

```python
elif ftype == FrameType.VOICE_AUDIO:
    """VOICE_AUDIO: publish AudioData в /avatar/voice_in (int16 PCM 16 kHz)."""
```

Уже шлёт всё в `/avatar/voice_in`. Расширение для шага 5а:

```python
elif ftype == FrameType.VOICE_AUDIO:
    # stream_id ∈ {1: ptt, 2: wake}. Маршрут — по stream_id, не по mode.
    if frame.stream_id == 2:                     # wake
        self.bridge.publish_quest_wake_audio(payload)
    else:                                         # ptt (default = 1)
        self.bridge.publish_audio_data(payload)
```

`bridge.publish_quest_wake_audio` — новый метод (добавляется в `bridge.py`),
публикует в отдельный топик (например `/avatar/quest_wake` или сразу в
`stt_node` через `/audio/quest_in` — точное имя топика решает шаг 5,
здесь только контракт «stream_id=2 публикуется в отдельный топик»).

**Маршрутизация вейка (`stream_id=2` → «ТАРС» / discard) — это шаг 5, не 5а.**
Шаг 5а заканчивается на публикации в отдельный топик с маркировкой источника.

## 3. Альтернативы, которые отклонены

| альтернатива | почему отклонена |
|---|---|
| Wake detector на клиенте (Porcupine/openWakeWord/vosk-browser) | См. §1.3 — нет русского / нет web-сборки / это STT, не keyword-spotter |
| Два независимых `getUserMedia` | Удвоение OS-ресурсов, риск конфликта с audio-routing Quest, лишний расход батареи |
| Гейтить только на сервере | Лишний трафик WS (~32 КБ/с × 2 ч = 230 МБ на сессию), батарея, латентность |
| Отдельный `FrameType.VOICE_WAKE_AUDIO` | Ломает совместимость с подписчиками `VOICE_AUDIO`; тот же `stream_id` дешевле |
| Шаг 5а-0 (`AudioWorklet`) внутри этой карточки | ADR-0013 — incremental delivery. `AudioWorklet` — отдельная карточка backend с ревью циклла рендера; шаг 5а явно **зависит** от него, но это не повод мержить |
| VAD с обучением (energy + ZCR + спектральные признаки) | Избыточно для шлема; RMS+hangover достаточно для «режь тишину» (детект «ТАРС» — на сервере) |

## 4. Trade-off

**Выигрыш:**
- Вейк-слово «ТАРС» физически достижимо без зажатого грипа → разблокирует шаги 5б/6/7+.
- Один mic-захват на две задачи → нет конфликта OS audio-routing.
- VAD-гейт до WS → трафик только когда есть речь → экономия батареи и канала.

**Цена:**
- Постоянный `AudioContext` живёт всю сессию → расход батареи Quest. **Не измерен** в этой сессии — DoD требует замера на устройстве (см. §6).
- `ScriptProcessorNode` в главном потоке конкурирует с VR-рендером. Это и есть причина, почему шаг 5а-0 — отдельная карточка **до** включения `wake` по умолчанию. До merge шага 5а-0 поток `wake` заводится **за флагом** `?wake=1` (URL-параметр) для измерения на устройстве, в проде не дефолтится.
- RMS-VAD режет тишину — это значит **первый звук фразы теряется**, если порог слишком высокий. Hangover 200 мс — компромисс. На устройстве это калибруется.

**Что НЕ делаем (anti-scope):**
- Wake detector на клиенте — отклонён, см. §1.3.
- Список STT-искажений «ТАРС» — собирается по e2e-логам (открытый вопрос §14 архитектуры), не придумывается.
- Модернизация `voice_capture.ts` за пределы минимального VAD и `setWakeGate` — не лезем в архитектуру модуля, пока шаг 5а-0 не заменит `ScriptProcessorNode`.

## 5. Файлы и код (план реализации для backend-воркера)

### 5.1 `src/rob_box_quest/webxr_client/src/input/voice_capture.ts`

Точечные изменения (≈60 строк нового кода):

```typescript
// (a) +channel в onChunk
onChunk: (pcm: Int16Array, channel: "ptt" | "wake") => void

// (b) +VAD-конфиг в опциях + +state в модуле
const VAD_RMS_THRESHOLD_DEFAULT = 200;        // int16 ~ 0.0061
const VAD_HANGOVER_SAMPLES_DEFAULT = 3200;    // 200 ms @ 16 kHz

interface WakeGate { enabled: boolean; suppressed: boolean; }
let wakeGate: WakeGate = { enabled: false, suppressed: false };
let hangoverSamplesLeft = 0;

// (c) +rmsInt16 helper (для тестов — pure function)
export function rmsInt16(pcm: Int16Array): number {
  // sum of squares / N, sqrt; возвращает 0..32767
}

// (d) в onaudioprocess, перед push():
const r = rmsInt16(input);
const isSpeech = r >= VAD_RMS_THRESHOLD_DEFAULT;
if (isSpeech) hangoverSamplesLeft = VAD_HANGOVER_SAMPLES_DEFAULT;
else if (hangoverSamplesLeft > 0) hangoverSamplesLeft -= input.length;
const speechActive = isSpeech || hangoverSamplesLeft > 0;

// Один и тот же PCM-чанк уходит в оба канала с разным gate:
if (voicePttEnabled)  opts.onChunk(resampled, "ptt");
if (wakeGate.enabled && !wakeGate.suppressed && speechActive) {
  opts.onChunk(resampled, "wake");
}

// (e) +setWakeGate в публичном API
function setWakeGate(opts: { enabled?: boolean; suppressed?: boolean }): void {
  if (opts.enabled   !== undefined) wakeGate.enabled   = opts.enabled;
  if (opts.suppressed !== undefined) wakeGate.suppressed = opts.suppressed;
  if (wakeGate.enabled && !wakeGate.suppressed) hangoverSamplesLeft = 0;
}
return { start, stop, isCapturing, setWakeGate };
```

**Чистые функции** (`rmsInt16`, `resampleToInt16`, `floatToInt16`) выносятся в
экспорт для unit-тестов — pure, без `AudioContext`.

### 5.2 `src/rob_box_quest/webxr_client/src/wire/connection.ts`

Расширить `sendVoiceAudio` параметром `streamId`:

```typescript
sendVoiceAudio(payload: Uint8Array, streamId: 1 | 2 = 1): void {
  // 1 = ptt (default, существующее поведение)
  // 2 = wake (новое)
  const bytes = encodeFrame(FrameType.VOICE_AUDIO, streamId, payload);
  // ...
}
```

Сигнатура обратно совместима (default = 1 = текущее поведение).

### 5.3 `src/rob_box_quest/webxr_client/src/main.ts`

Вокруг `voiceCapture` (~30 строк):

```typescript
const voiceCapture = createVoiceCapture({
  onChunk: (pcm, channel) => {
    if (!conn || disconnected) return;
    const bytes = new Uint8Array(pcm.buffer, pcm.byteOffset, pcm.byteLength);
    if (channel === "ptt")  conn.sendVoiceAudio(bytes, 1);  // существующее
    if (channel === "wake") conn.sendVoiceAudio(bytes, 2);  // новое
  }
});

// (a) при WELCOME — включаем wake по умолчанию
voiceCapture.setWakeGate({ enabled: true, suppressed: false });

// (b) внутри applyVoicePtt — подавление wake при грипе
function applyVoicePtt(radio: boolean, robot: boolean): void {
  // ... существующий код ...
  voiceCapture.setWakeGate({ suppressed: radio || robot });
  // ... остальное без изменений ...
}

// (c) обработка voice_listen_start/stop от сервера (опционально) или панели
//     (state-машина ModeManager / voiceState)
```

### 5.4 `src/rob_box_quest/rob_box_quest/server/ws_server.py`

Расширить `VOICE_AUDIO` ветку (~10 строк):

```python
elif ftype == FrameType.VOICE_AUDIO:
    # stream_id: 1=ptt (default), 2=wake
    if frame.stream_id == 2:
        self.bridge.publish_quest_wake_audio(payload)
    else:
        self.bridge.publish_audio_data(payload)
```

И добавить команды `voice_listen_start/stop` рядом с `voice_ptt_start/stop`
(после `ws_server.py:459`, ещё ~20 строк):

```python
if cmd == "voice_listen_start":
    self.bridge.set_wake_stream_state(active=True)
    await self._send(ws, FrameType.JSON_EVENT, 0,
                     {"type": "voice_listen_ack", "state": "active", "ts_ms": int(time.time()*1000)})
    return
if cmd == "voice_listen_stop":
    self.bridge.set_wake_stream_state(active=False)
    await self._send(ws, FrameType.JSON_EVENT, 0,
                     {"type": "voice_listen_ack", "state": "paused", "ts_ms": int(time.time()*1000)})
    return
```

`bridge.set_wake_stream_state` — поле в `BridgeState` (тонкая обвязка; не
архитектурное изменение).

### 5.5 Тумблер в панели (`voice_pipeline_panel.ts` — файл существует в
worktree backend'а; для шага 5а — это **выход за scope webxr_client**)

Помечается как **out of step 5а**. Панельный тумблер — отдельная маленькая
карточка (5 строк UI + JSON_CMD). В DoD шага 5а проверяется **наличие**
команды `voice_listen` в `main.ts` (grep), не полный UI.

## 6. Definition of Done — проверяемые факты

Карточка issue #1992 уже формулирует DoD. Этот ADR добавляет **как именно**
каждый пункт проверяется (raw evidence обязателен по ADR-0018):

- [ ] **`voice_listen_start`/`stop` меняют состояние потока `wake` (raw-лог сервера).**
  Acceptance: `docker logs voice-assistant --since 5m` содержит строки
  `voice_listen_ack{state:active}` сразу после клика панели / HELLO, и
  `{state:paused}` сразу после второго клика. Лог копируется в PR-описание.

- [ ] **При зажатом грипе поток `wake` подавлен (одна фраза не попадает в два маршрута).**
  Acceptance: тест в `tests/voice_capture.test.ts` — `setWakeGate({suppressed:true})`
  → `onChunk` не получает `channel="wake"` за время подавления, но
  `channel="ptt"` приходит. Локальный запуск `pytest -v tests/voice_capture.test.ts`,
  output в PR.

- [ ] **Замер батареи и стабильности `getUserMedia` ≥ 2 ч непрерывно — с цифрами.**
  Acceptance: на Quest запускается сессия с `?wake=1`, в фоне пишется
  `battery_level.json` (через `WebXR Device API` или `getBattery()` где
  доступно) и `mediaStreamTrackStats` каждые 60 с. Через 2 ч — `git diff`
  цифр + `adb logcat` фрагмент. **Без raw-цифр DoD не считается закрытым**
  (ADR-0018). Если замер на этой неделе невозможен — задача блокируется
  с `kind=hardware-required` (см. §7).

- [ ] **Тумблер в панели управляет `voice_listen` (grep команды в `main.ts`).**
  Acceptance: `grep -n "voice_listen" src/rob_box_quest/webxr_client/src/main.ts`
  возвращает ≥2 строки (отправка команды + приём state).

## 7. Что архитектор НЕ делает в этой сессии

Эта сессия — design + ADR, **не** реализация. Карточка должна быть
`kanban_complete` после merge этого ADR в develop. Реализацию делает
backend-профиль отдельной карточкой (по образцу `t_e1d6d151`,
`t_f1d3c319`, `t_25eef7c1` из recent work architect'а).

Если воркеру backend нужен hardware-доступ (Quest) для замера батареи и
`getUserMedia` 2 ч — он вызывает `kanban_block kind=hardware-required`
**с конкретной формулировкой**: «нужен SSH на vision с правами `docker exec`
в quest-build, окно 2.5 ч непрерывно». Не блокируется на абстрактное
«нет доступа» — только с конкретикой.

## 8. Риски и митигация

| риск | вероятность | митигация |
|---|---|---|
| `getUserMedia` отваливается через ~30 мин на Quest | средняя | Шаг 5а-0 (`AudioWorklet`) — не мерджим `wake` в прод до замера. За флагом `?wake=1` |
| RMS-VAD режет начало фразы | средняя | Hangover 200 мс; на замере — подобрать; альтернатива: энергия + ZCR — не в скоупе |
| Расход батареи > 30 % за сессию | низкая–средняя | VAD-гейт режет трафик; серверная сторона не декодирует PCM; точная цифра — после замера |
| Конфликт с существующим PTT при пересечении grip+wake | низкая | Подавление клиентское, до VAD; raw-лог сервера не получит `wake`-кадров во время PTT |
| `bridge.publish_quest_wake_audio` пишет в топик, которого ещё нет в шаге 5 | средняя | Контракт ADR описывает только «отдельный топик»; имя фиксирует шаг 5. Шаг 5а закоммичен `publish_quest_wake_audio` как no-op с TODO, пока шаг 5 не подключит подписчика |

## 9. Открытые вопросы (для следующих сессий)

1. **Имя топика для wake-аудио на стороне робота.** Шаг 5а публикует в
   `bridge.publish_quest_wake_audio`, шаг 5 определяет имя топика. Кандидаты:
   `/avatar/quest_wake_audio`, `/audio/quest_in`, отдельный `/avatar/wake_audio`.
2. **RMS-порог и hangover для шлема.** Стартовые 0.01 / 200 мс — компромисс;
   калибруется на замере.
3. **Persist тумблера панели.** localStorage vs server-side config —
   решается в карточке UI, не блокирует шаг 5а.

## 10. Тест-план

### Unit (`tests/voice_capture.test.ts` — файл существует, расширяется)

```typescript
test("rmsInt16 returns 0 for silence", () => {
  expect(rmsInt16(new Int16Array(320))).toBe(0);
});
test("resampleToInt16 is pure and stable", () => { /* существующий */ });
test("setWakeGate({suppressed:true}) blocks wake channel but keeps ptt", () => {
  // mock onChunk; запустить onaudioprocess с шумным PCM;
  // вызвать setWakeGate({suppressed:true}); второй запуск — wake не зовётся,
  // ptt зовётся.
});
test("VAD hangover: 200 ms after last voiced chunk", () => {
  // последовательность: 100 ms тишина → 50 ms голос → 250 ms тишина
  // ожидание: wake приходит ещё 200 ms после последнего голоса (итого 250 ms тишины включая hangover)
});
```

### Server (`test/unit/server/test_ws_server_voice.py`)

```python
async def test_voice_listen_start_publishes_ack():
    # cmd voice_listen_start → JSON_EVENT{type:voice_listen_ack, state:"active"}
async def test_voice_audio_stream_id_2_routes_to_quest_wake():
    # VOICE_AUDIO с stream_id=2 → bridge.publish_quest_wake_audio вызван
async def test_voice_audio_stream_id_1_still_routes_to_voice_in():
    # регрессия: существующее поведение не сломано
```

### Wire-contract (`tests/protocol.test.ts`)

```typescript
test("encodeFrame VOICE_AUDIO stream_id=2 round-trip", () => {
  // симметрично encode/decode с streamId=2
});
```

### E2E (контракт для `e2e-process`)

Карточка заполняет блок `## e2e` в issue body:

```
voice_text: "Робот, вруби музыку"
voice_file: .github/e2e/voice_commands/rabot_wake_phrase.ogg   # новая команда
volume: 150
record_seconds: 60
llm: minimax-m3
tts: minimax-male-qn-qingse
stt: yandex
```

**Сценарий.** Робот в immersive-сессии, mic захвачен (`?wake=1`), панель
`voice_listen` = on. Команда произносится **без нажатия грипа**. Ожидание:
`voice_listen_ack{state:active}` в логе сервера до команды; в логе stt_node —
фраза «робот вруби музыку» пришла в `/avatar/quest_wake` (или эквивалентный
топик шага 5); ответ ТАРС — **отдельная карточка шага 5б**, не шага 5а.

Файл `rabot_wake_phrase.ogg` коммитится в
`.github/e2e/voice_commands/rabot_wake_phrase.ogg` в карточке реализации
backend'ом — файл ещё не существует, его сгенерирует MiniMax TTS или Yandex
TTS в `ensure_voice_file` шаге e2e-process.

### Rough ручная проверка (на устройстве, вне этой сессии)

```bash
# 1. Запустить клиента с ?wake=1
# 2. В логах voice-assistant:
ssh vision "docker logs -f voice-assistant 2>&1 | grep -E 'voice_listen_ack|quest_wake'"
# 3. Замерить батарею: adb shell dumpsys battery (раз в 60 с, 2 ч)
# 4. Проверить, что mediaStream не отвалился: getStats() в devtools
```

## 11. Сводка для PR

**Один PR в develop:** этот ADR. **Никакого кода в этом PR.** Реализация —
отдельная карточка backend-профиля со ссылкой на этот ADR.

Заголовок PR: `docs(adr-0054): operator-agent step 5a — wake stream contract + VAD-gate + voice_listen_*`

Лейблы: `ai-generated`, `source:gsd`, `docs`, `architecture`.

Closes #1992 — **не ставим**. Шаг 5а закроется только после реализации и
замера батареи. Этот PR закрывает **design gap**, не issue.
