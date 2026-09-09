# E2E Test: Voice Assistant + Microphone Recording

Проведение **полноценного e2e теста** голосового ассистента с записью реального
аудио через микрофон. Используется когда нужно **убедиться** что робот реально
произносит реплики и что звук физически воспроизводится в комнате.

## Когда использовать

- TTS робота молчит (проверить что звук **реально** воспроизводится, а не только
  пишется в логах)
- Тест новых аудио-фич (эффекты, pitch shift, форматы)
- Диагностика e2e перед PR
- Валидация после деплоя (робот реально говорит, не только синтезирует)

## Предусловия

1. **Vision Pi** (`10.1.1.21`) — на роботе, запущен voice-assistant контейнер
2. **Билдовая машина** (`10.1.1.249`) — ноут рядом с роботом в той же комнате
3. На билдовой есть **PulseAudio** и **микрофон** (HDA Intel PCH, ALC3266)
4. На билдовой работает SSH (`ssh ros2@10.1.1.249`, пароль `open`)
5. На Vision Pi работает SSH (`ssh ros2@10.1.1.21`, пароль `open`)
6. Docker-контейнер `voice-assistant` запущен на Vision Pi

## Подготовка тестового аудио

```bash
# Генерируем espeak-аудио (русский голос)
espeak-ng -v ru -s 140 -w /tmp/test_robot.wav \
  "Шёл робот по улице, увидел лужу и говорит: \
   Опять эту ошибку дебажить. \
   Заходит в бар, а бармен ему: Тебе повторить? \
   Робот: Нет, я уже с первого раза завис."

# Копируем на Vision Pi (на хост)
sshpass -p 'open' scp /tmp/test_robot.wav ros2@10.1.1.21:/tmp/

# Копируем в контейнер voice-assistant
sshpass -p 'open' ssh ros2@10.1.1.21 \
  'docker cp /tmp/test_robot.wav voice-assistant:/tmp/test_robot.wav'

# Устанавливаем ffmpeg в контейнер (если ещё нет)
sshpass -p 'open' ssh ros2@10.1.1.21 \
  'docker exec voice-assistant bash -c "apt-get install -y ffmpeg"'

# Конвертируем в формат, который принимает ReSpeaker (24-bit stereo 22050 Hz)
sshpass -p 'open' ssh ros2@10.1.1.21 \
  'docker exec voice-assistant bash -c "
    ffmpeg -i /tmp/test_robot.wav -f wav -acodec pcm_s24le \
      -ar 22050 -ac 2 /tmp/test_stereo.wav -y"'
```

## Процедура e2e теста

### Шаг 1: Запустить запись с микрофона (в фоне на билдовой)

```bash
sshpass -p 'open' ssh ros2@10.1.1.249 \
  "nohup parec --format=s16le --channels=1 --rate=16000 \
   --device=alsa_input.pci-0000_00_1f.3.analog-stereo \
   /tmp/e2e_test.wav > /tmp/rec.log 2>&1 &"
```

### Шаг 2: Воспроизвести тестовое аудио на Vision Pi (через dmix)

```bash
sshpass -p 'open' ssh ros2@10.1.1.21 \
  "docker exec voice-assistant bash -c '
    aplay -D dmix_respeaker -t wav -f S24_3LE -r 22050 -c 2 \
      /tmp/test_stereo.wav'"
```

### Шаг 3: Подождать 25-30 секунд (пока проиграет)

```bash
sleep 25
```

### Шаг 4: Остановить запись и скачать

```bash
# Остановить запись (pkill, файл будет неполный — header нет)
sshpass -p 'open' ssh ros2@10.1.1.249 "pkill -f 'parec.*e2e_test'"

# Подождать что pkill завершится
sleep 1

# Скачать RAW файл
sshpass -p 'open' scp ros2@10.1.1.249:/tmp/e2e_test.wav \
  /home/builder/hermes-share/e2e_test_raw.wav
```

### Шаг 5: Добавить WAV header (raw файл из pkill)

```python
import struct
import os

raw = open('/home/builder/hermes-share/e2e_test_raw.wav', 'rb').read()
# parec: 16-bit, mono, 16000 Hz
data_size = len(raw)
file_size = 36 + data_size

header = b'RIFF' + struct.pack('<I', file_size) + b'WAVE'
header += b'fmt ' + struct.pack('<I', 16)        # fmt chunk size
header += struct.pack('<H', 1)                    # PCM
header += struct.pack('<H', 1)                    # mono
header += struct.pack('<I', 16000)                # sample rate
header += struct.pack('<I', 32000)                # byte rate
header += struct.pack('<H', 2)                    # block align
header += struct.pack('<H', 16)                   # bits per sample
header += b'data' + struct.pack('<I', data_size)

out = '/home/builder/hermes-share/e2e_test.wav'
with open(out, 'wb') as f:
    f.write(header)
    f.write(raw)
print(f"Saved: {out} ({os.path.getsize(out)} bytes)")
```

## Проверка результата

```python
import wave, struct

with wave.open('/home/builder/hermes-share/e2e_test.wav', 'rb') as w:
    samples = struct.unpack(f'<{w.getnframes()}h', w.readframes(w.getnframes()))
    max_amp = max(abs(s) for s in samples)
    duration = w.getnframes() / w.getframerate()

print(f"Duration: {duration:.1f} sec")
print(f"Max amplitude: {max_amp} ({100*max_amp/32768:.1f}%)")
# Успех: amplitude > 50%
```

## Метрики успеха

| Метрика | Успех | Провал |
|---------|-------|--------|
| Max amplitude | > 50% | < 20% |
| Длительность записи | > 5 сек | < 1 сек |
| Громких сэмплов (>1000) | > 5% | < 1% |

## E2E через STT (полный диалог)

Если хотите протестировать **через STT** а не raw-файл:

```bash
# Шаг 1: Запись на билдовой (2 минуты)
sshpass -p 'open' ssh ros2@10.1.1.249 \
  "nohup parec --format=s16le --channels=1 --rate=16000 \
   --device=alsa_input.pci-0000_00_1f.3.analog-stereo \
   /tmp/e2e_dialogue.wav > /tmp/rec.log 2>&1 &"

sleep 1

# Шаг 2: STT команда (робот должен ответить через TTS)
sshpass -p 'open' ssh ros2@10.1.1.21 \
  "docker exec voice-assistant bash -c '
    source /opt/ros/humble/setup.bash
    timeout 5 ros2 topic pub --once /voice/stt/result std_msgs/msg/String \
      \"{data: \\\"робок расскажи длинный смешной анекдот\\\"}\"'"

# Шаг 3: Ждём 2 минуты
sleep 120

# Шаг 4: Стоп записи, скачать
sshpass -p 'open' ssh ros2@10.1.1.249 "pkill -f 'parec.*e2e_dialogue'"
sleep 1
sshpass -p 'open' scp ros2@10.1.1.249:/tmp/e2e_dialogue.wav \
  /home/builder/hermes-share/e2e_dialogue_raw.wav

# Добавить WAV header (см. выше)
```

## Подводные камни

### `Device or resource busy`

`aplay -D plughw:CARD=ArrayUAC10,DEV=0` падает с busy — ReSpeaker уже занят
voice-assistant. Используйте **dmix_respeaker**:

```bash
aplay -D dmix_respeaker -t wav -f S24_3LE -r 22050 -c 2 /tmp/test.wav
```

### `Sample format non available`

ReSpeaker ожидает **24-bit**, не 16. Конвертируйте через ffmpeg:

```bash
ffmpeg -i source.wav -f wav -acodec pcm_s24le -ar 22050 -ac 2 out.wav
```

### `Channels count non available`

ReSpeaker ожидает **stereo**, не mono. Конвертируйте с `-ac 2`.

### `pkill` оставляет файл без WAV header

`parec` пишет заголовок в начале и обновляет при закрытии. `pkill -f parec`
убивает процесс до graceful shutdown — файл остаётся **без RIFF header**.

Решение: добавить header вручную (см. Шаг 5).

### Микрофон билдовой не слышит Vision Pi

Если они в **разных комнатах** — запись будет тихой. Нужно чтобы устройства
были **в одной акустической среде** (одна комната).

## Результаты тестов

### 2026-07-29: e2e с espeak WAV

- Воспроизведён espeak WAV через dmix_respeaker на Vision Pi
- Записан с микрофона билдовой: **41.8 сек, max amplitude 83.1%** ✅
- Подтверждено: канал Vision Pi → билдовая работает

### 2026-07-29: e2e через STT

- STT "робок расскажи длинный анекдот"
- TTS **НЕ воспроизвёл** аудио (обнаружена утечка памяти, см. #929)
- Записан тихий звук (< 10% amplitude) — тишина

## Связанные документы

- [Главная документация по голосовому ассистенту](../packages/rob_box_voice.md)
- [ADR-0001: Архитектура harness](../adr/0001-harness-architecture.md)
- [Issue #929: Утечка памяти в voice-assistant](https://github.com/krikz/rob_box_project/issues/929)
- [Issue #918: DialogCore state machine busy loop](https://github.com/krikz/rob_box_project/issues/918)

## Авторы

Тест создан 2026-07-29 при диагностике OOM killer в voice-assistant.
