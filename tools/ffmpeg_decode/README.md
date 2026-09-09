# tools/ffmpeg_decode — ffmpeg decode scenarios for PCM / WAV / MP3 / OGG

Интеграционные сценарии для проверки того, что `ffmpeg` декодирует
исходные аудио-файлы разных форматов в формат, ожидаемый
`tts_node` (`int16 LE mono @ 16 kHz`), и что production-путь
`to_pcm_int16` из `rob_box_voice.utils.audio_transcode` принимает
декодированный поток и возвращает корректный `DecodedAudio`.

## Что каждый сценарий делает

```
[source.<fmt>]  →  ffmpeg -i ... -f s16le -ac 1 -ar 16000 decoded.raw
                       ↓
                   wav_wrap → decoded.wav
                       ↓
                   ffprobe (duration)
                       ↓
                   assert_duration (PASS/FAIL)
                       ↓
                   to_pcm_int16(round-trip) (PASS/FAIL)
```

1. Берёт фикстуру (по умолчанию — sine 440 Гц 1.00 с при 16 кГц).
2. Прогоняет через `ffmpeg` в сырой PCM s16le mono 16 кГц — это
   ровно тот формат, который `tts_node` ожидает на
   `/voice/audio/speech` после `_decode_compressed` /
   `_decode_wav` / прямого PCM-пути.
3. Оборачивает в WAV и зовёт `ffprobe` чтобы получить длительность.
4. Автоматически проверяет длительность (ожидание 1.0 с ± 0.05 с).
5. Гоняет декодированный поток через production-функцию
   `rob_box_voice.utils.audio_transcode.to_pcm_int16` — для PCM/WAV
   это прямой round-trip, для MP3/OGG сначала переупаковываем
   декодированный PCM обратно в сжатый контейнер через `ffmpeg`,
   чтобы на вход `to_pcm_int16` пришёл «настоящий» MP3/OGG-блоб.

## Acceptance gate

Каждый сценарий **запускается одной командой**, **завершается с
известным кодом возврата**, **длительность decoded-аудио
проверяется автоматически**.

## Сценарии

| Сценарий | Скрипт | Источник по умолчанию |
|---|---|---|
| `pcm` | `scenario_pcm.sh` | `tts_audio_bench/fixtures/pcm/sine_440hz_1.00s_16000.pcm` |
| `wav` | `scenario_wav.sh` | `tts_audio_bench/fixtures/wav/sine_440hz_1.00s_16000.wav` |
| `mp3` | `scenario_mp3.sh` | `tts_audio_bench/fixtures/mp3/sine_440hz_1.00s_16000.mp3` |
| `ogg` | `scenario_ogg.sh` | `tts_audio_bench/fixtures/ogg/sine_440hz_1.00s_16000.ogg` |

## Запуск

```bash
# один сценарий
bash tools/ffmpeg_decode/scenario_pcm.sh
bash tools/ffmpeg_decode/scenario_mp3.sh tts_audio_bench/fixtures/mp3/sine_100hz_1.00s_32000.mp3

# все четыре
bash tools/ffmpeg_decode/run_all.sh
```

## Коды возврата (стабильные)

| Код | Что значит |
|---|---|
| 0 | PASS — ffmpeg декодировал, длительность совпала, `to_pcm_int16` round-trip прошёл |
| 1 | Требуемый инструмент отсутствует на PATH (`ffmpeg` / `ffprobe` / `python3`) |
| 2 | Исходная фикстура не найдена |
| 3 | `ffmpeg decode` упал с ошибкой |
| 4 | Длительность декодированного аудио вне допуска (±0.05 с) |
| 5 | `to_pcm_int16` round-trip провалился (например, вернул неверный `sample_rate` или пустой `pcm`) |

## Артефакты

Каждый сценарий пишет под
`tts_audio_bench/artifacts/ffmpeg_decode/<scenario>/`:

- `decoded.raw` — сырой PCM s16le mono 16 кГц, вышедший из `ffmpeg`
- `decoded.wav` — тот же поток обёрнутый в WAV (для `ffprobe`)

## Ограничения / нюансы

- **MP3-фикстуры**: контейнер сообщает `1.08 с` (LAME encoder
  padding), но декодированный PCM — ровно 1.0 с. Acceptance gate
  проверяет длительность **декодированного** аудио, не контейнера,
  потому что именно декодированные байты попадают в `AudioData`.
- **PCM raw**: фикстура не имеет заголовка, поэтому ffmpeg получает
  явные подсказки формата (`-f s16le -ar 16000 -ac 1`). Это та же
  схема, что в `tts_audio_bench/scripts/make_fixture.py`.
- **`to_pcm_int16` для MP3/OGG**: на хостах без `pydub` /
  `ffmpeg`-биндингов сценарий полагается на системный `ffmpeg` (тот
  же путь, что использует production `_decode_compressed` — fallback
  на `ffmpeg` subprocess).

## Связь с задачей

Задача kanban `t_fb948646` декомпозирует верхнеуровневую цель
«интеграционные сценарии ffmpeg decode для PCM/WAV/MP3/OGG». Эти
скрипты — закрытый deliverable: каждый сценарий самодостаточен,
выходит с известным кодом, длительность проверяется автоматически.
