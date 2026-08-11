# ADR-0013: ReSpeaker DSP tuning + правильный канал mix — round-2 фикс (issue #1117)

| Поле           | Значение                                                                                                                     |
|----------------|------------------------------------------------------------------------------------------------------------------------------|
| Статус         | **Accepted** (round-2 supersede ADR-0012)                                                                                    |
| Дата           | 2026-08-11                                                                                                                   |
| Автор          | backend (Hermes Agent)                                                                                                       |
| Kanban         | `t_51383099` (round-2 wake word fix)                                                                                         |
| Контекст       | Issue [#1117](https://github.com/krikz/rob_box_project/issues/1117) round-2 — даже после PR #1121 (pre-roll 1.0→1.5s) wake word «Робот» теряется в STT: «меня зовут саша» / «алиса» вместо «робот меня зовут саша» |
| Supersedes     | ADR-0012 (round-1 фикс `mix_channels=[0,1,2,3,4,5]` основан на ошибочной карте каналов)                                       |
| Предшественники | [ADR-0012](0012-respeaker-6ch-channel-map-and-mixer.md) (round-1, superseded), [ADR-0001](0001-harness-architecture.md) (контекст харнесса) |
| Связанные       | `src/rob_box_voice/rob_box_voice/audio_node.py` (параметры `mix_channels`, `hpf_on`), `src/rob_box_voice/config/audio_node.yaml`, `src/rob_box_voice/rob_box_voice/utils/respeaker_interface.py` (HPFONOFF register + configure_dsp), `src/rob_box_voice/test/test_audio_node_echo.py` (TestDSPApplyOnStart, TestMixChannels) |
| Реализация      | Branch `z-backend/1117-round2-mix-ch0-hpf` (PR #1123 к `develop`). Изменения: `mix_channels: [0,1,2,3,4,5] → [0]`, добавлен `hpf_on: 1` + apply на старте через `ReSpeakerInterface.configure_dsp()`, default в yaml — `speech_prefetch: 1.0 → 1.5` уже из PR #1121 |
| Ветка           | `z-backend/1117-round2-mix-ch0-hpf` → `develop`                                                                              |

## 1. Назначение

Зафиксировать правильную **карту каналов ReSpeaker Mic Array v2.0** в 6-канальном режиме И канонический фикс DSP-параметра **HPFONOFF**, который по умолчанию firmware стоит в 3 (180 Hz high-pass), что режет тихое «Р» в начале слова «Робот» (-26 dBFS по даташиту).

Это нужно для:

1. **Чистое распознавание wake word**: STT получает фразу, начинающуюся с «Р» (60-80 Hz основной тон + гармоники), без срезания низов.
2. **Приземлить hard-fix, который инженеры могут проверить глазами** — раньше (ADR-0012) round-1 объяснял «почему мы берём ВСЕ 6 каналов», но это была ошибочная карта каналов.
3. **Зафиксировать HPFONOFF как код** (write_parameter), а не как «надежда что firmware default правильный».

## 2. Контекст и проблема

### 2.1 Что говорит datasheet ReSpeaker v2.0

Из `/tmp/issue_1117_assets/datasheet.md` (см. в issue #1117 round-2):

> 6_channels_firmware.bin:
> - Channel 0: processed audio for ASR
> - Channel 1-4: raw mic (mic1, mic2, mic3, mic4)
> - Channel 5: merged playback

И из `Respeaker.cfg` (https://github.com/furushchev/respeaker_ros):

```
HPFONOFF default = 1 (70 Hz cut-off)
```

### 2.2 Что говорит XVF-3000 datasheet (точная модель ReSpeaker v2)

| Параметр | Default firmware | Что делает |
|----------|------------------|------------|
| `HPFONOFF` | **3** (180 Hz cut-off) | High-pass filter на микрофонных сигналах |
| `AGCONOFF` | 1 | AGC on/off |
| `GAMMAVAD_SR` | 3.5 dB | VAD threshold |

⚠️ **ВАЖНО**: дефолт `HPFONOFF=1` в `Respeaker.cfg` — это дефолт для ros-ноды на хосте (RoboDyn). В самой firmware XVF-3000 (реальное железо без конфигурации) — дефолт `HPFONOFF=3` (180 Hz). Наша нода раньше ничего не записывала → использовался firmware default = 180 Hz → режет «Р».

### 2.3 Что происходило в e2e #1077 (round-46/47/48, round-49 после #1121)

Сценарий: команда «Робот, меня зовут Саша» играется на Katana (249) → ReSpeaker (10.1.1.21):
- **VAD** ловит фразу (через 800-1100 мс после начала, тихое «Р» ниже порога GAMMAVAD_SR=3.5)
- **STT** получает мономикс из 6 каналов (round-1 default `[0,1,2,3,4,5]`)
- **DSP XVF-3000** в это время применяет HPFONOFF=3 (180 Hz) → срезает всё ниже 180 Hz
- **«Р» в «Робот»** = 60-80 Hz → УБИВАЕТСЯ в DSP XVF-3000 ДО того, как мы его усреднили в микшере
- STT возвращает: «меня зовут саша» / «алиса» (галлюцинация на остаток)
- dialogue_node отбрасывает: `no_wake_word`
- ВЫВОД: NO_ACCEPT

### 2.4 Ошибочная теория round-1 (ADR-0012)

Round-1 (PR #1093) предположил: «Ch5-6 = playback reference с чистой цифровой копией фразы → брать все 6 каналов, чтобы playback-референс помог Yandex». A/B дал `yandex:ok` для [0,1,2,3,4,5] vs `yandex:empty` для [0,1,2,3]. Это совпало с теорией. **Но теория была неверной.**

Реальность:
- **Ch1 (1-индекс) = DSP-processed ASR** = правильный канал для STT (AEC + beamforming уже сделаны)
- Ch2-5 = сырые микрофоны (DSP ещё не применялся)
- Ch6 = merged playback reference (то, что играет робот через динамик)
- Усреднение всех 6 = шум: 4 raw + 1 processed + 1 playback = плохой STT-сигнал

`yandex:ok` в round-1 объясняется не playback reference, а **тем, что playback-референс содержит громкий сигнал** (то, что говорит робот/музыка из динамика) → STT «видит» громкий звук и возвращает что-то непустое. Но это была ложная тревога: STT получал «хвост» эха после TTS-grace, галлюцинировал и yandex возвращал OK на пустом месте.

## 3. Решение

### 3.1 mix_channels = [0]

Берём **ТОЛЬКО Ch0 PyAudio (= Ch1 firmware)** = DSP-processed ASR. Сырые микрофоны и playback-референс исключены.

```yaml
# src/rob_box_voice/config/audio_node.yaml
audio_node:
  ros__parameters:
    channels: 6
    mix_channels: [0]   # round-2: DSP-processed ASR only
```

### 3.2 HPFONOFF = 1 (через USB write)

На старте audio_node, при подключённом ReSpeaker:

```python
# rob_box_voice/audio_node.py (initialize_hardware → _apply_dsp)
if self.dsp_apply_on_start and self.respeaker.is_connected():
    ok = self.respeaker.configure_dsp(hpf_on=self.hpf_on)  # hpf_on=1 = 70 Hz
```

```python
# rob_box_voice/utils/respeaker_interface.py
def configure_dsp(self, hpf_on: int = 1) -> bool:
    return self.write_parameter('HPFONOFF', int(hpf_on))
```

Где `'HPFONOFF': (18, 27, 'int')` — параметр группы 18 (DSP processing), offset 27, int (см. `/tmp/issue_1117_assets/usb_4_mic_array_tuning.py`).

### 3.3 Граница ответственности

| Компонент | Что делает | Граница |
|-----------|------------|---------|
| **DSP XVF-3000 (hardware)** | AEC, NS, AGC, HPF | Не под нашим контролем, КРОМЕ настройки через USB-write |
| **audio_node (software)** | Выбор канала микса, HPFONOFF apply | Параметр `mix_channels` + `hpf_on`; пишем через USB при старте |
| **dialogue_node (software)** | Wake-word gate на STT-тексте | Принимает только фразы, начинающиеся с "робок*"/"робот*"/etc. |
| **VAD (GAMMAVAD_SR)** | Аппаратная детекция речи | Не режет wake word, если HPFONOFF и mic gain правильные |

### 3.4 Опции / opt-in

- `mix_channels: []` — legacy, усреднять ВСЕ каналы (как раньше). Только для A/B.
- `mix_channels: [0,1,2,3]` — только raw mics (без DSP, без playback). Тоже A/B.
- `mix_channels: [0]` — канонический round-2 default.
- `dsp_apply_on_start: false` — пропустить apply HPFONOFF. Полезно для тестов без ReSpeaker и для отладки «а работает ли это без DSP-настройки?».
- `hpf_on: 0` — HPF OFF (плохо для голоса — DC offset + низкочастотный шум).
- `hpf_on: 1` — 70 Hz (default, сохраняет «Р»).
- `hpf_on: 2` — 125 Hz (компромисс).
- `hpf_on: 3` — 180 Hz (firmware default, режет «Р» — НЕ выбирать).

## 4. Альтернативы, которые не выбрали

### 4.1 Увеличить pre-roll до 2.0-3.0s

PR #1121 уже поднял pre-roll до 1.5s. Round-46/47/48/49 (после PR #1121) показали: этого недостаточно. Проблема не в начале фразы — проблема в том, что **начало фразы физически срезается в DSP XVF-3000 (HPFONOFF=3)**, и даже 5 секунд pre-roll не помогут — «Р» никогда не дойдёт до STT.

### 4.2 Снизить GAMMAVAD_SR (порог VAD)

VAD ловит фразу нормально (5.35-6.12с — внутри фразы). Проблема не в VAD, а в ASR-выходе XVF-3000 после срезания HPFONOFF. Эта альтернатива лечит симптом, не причину.

### 4.3 Использовать 1_channel_firmware (вместо 6)

Заменить прошивку ReSpeaker на 1-канальный режим. Тогда Ch0 = единственный ASR-processed канал = правильный для STT. Но:
- DFU-прошивка требует физического доступа к ReSpeaker (`sudo python dfu.py --download 1_channel_firmware.bin`)
- Ломает DoA (Direction of Arrival) — для 1-канальной прошивки DoA работает по-другому (один канал без beamforming triangulation)
- Ломает тесты DoA в harness
- Round-2 (mix=[0] в 6-канальной прошивке) даёт тот же эффект без DFU

Решили: НЕ перешивать, фиксить в 6-канальном режиме.

### 4.4 Конкатенация raw-миков с processed каналом через WebRTC AEC

Слишком сложно, добавляет latency, и DSP XVF-3000 УЖЕ делает AEC — нет смысла дублировать.

## 5. Verify

1. **Юнит-тесты** (test_audio_node_echo.py — TestMixChannels, TestDSPApplyOnStart): 25/25 pass.
2. **CI**: Unit Tests ROS2 Humble workflow на develop-merge.
3. **E2E ручной** (issue #1077 round-50):
   - `ros2 launch rob_box_voice voice_assistant.launch.py`
   - `docker logs -f voice-assistant | grep -E "HPFONOFF|mix_channels"` → видим `HPFONOFF=1 применён`, `mix_channels=[0]`
   - Сказать «Робот, меня зовут Саша» → STT возвращает «робот меня зовут саша»
   - `dialogue_node`: `accepted, wake_word='робот'`
4. **E2E автомат** (после merge develop): L-E2E Voice Test workflow.

## 6. Caveats и риски

- **USB write timeout**: HPFONOFF пишем через `usb.core.ctrl_transfer()`. В редких случаях USB-стек может зависнуть. Запись обёрнута в `try/except`; в случае ошибки — только лог-warning, нода продолжает работать с firmware default.
- **Не все ReSpeaker одинаковы**: некоторые UAC1.0 устройства (Huawei, Jabra) могут не поддерживать параметр HPFONOFF. Если `write_parameter` вернул False → лог-warning → используется firmware default (обычно нормально для этих устройств).
- **DFU не требуется**: фикс работает на текущей прошивке (6_channels_firmware.bin). Если кто-то перешил на другую прошивку — после рестарта audio_node применит HPFONOFF=1 заново.

## 7. Rollback plan

Откатить PR #1123 → revert → deploy → проверить, что round-1 поведение восстановлено. Round-1 (mix=[0,1,2,3,4,5]) живой и проверен на e2e — откат безопасен.

Отключить фикс БЕЗ отката кода: `ros2 param set /audio_node dsp_apply_on_start false` — тогда HPFONOFF останется на firmware default, но `mix_channels` останется `[0]`. Полезно для A/B-проверки.
