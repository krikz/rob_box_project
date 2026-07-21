# TEST_BENCH — MiniMax TTS → ROS 2 AudioData

**Статус:** reproducible test bench для `rob_box_llm` и `rob_box_voice`  
**Последнее обновление:** 2026-07-21  
**Запускать из:** корня этого worktree  
**Цель прогона:** проверить путь `MiniMax HTTP/SSE → decode/transcode → tts_node → /voice/audio/speech → WAV` без реального API-ключа и без ReSpeaker.

> Документ описывает фактическое состояние текущей ревизии. Важное ограничение: `tts_node` сейчас не читает `MINIMAX_BASE_URL` и всегда создаёт провайдер с `https://api.minimax.io`. Поэтому локальный mock можно подключить к production-ноду только после минимального изменения, описанного в [BLOCKERS.md](BLOCKERS.md). Полный локальный bench уже передаёт `base_url` напрямую и выполняется без изменения production-кода.

## Содержание

- [Что проверяется](#что-проверяется)
- [Структура репозитория](#структура-репозитория)
- [Зафиксированное окружение](#зафиксированное-окружение)
- [Подготовка окружения](#подготовка-окружения)
- [Быстрый прогон за час](#быстрый-прогон-за-час)
- [1. Запуск локального MiniMax mock](#1-запуск-локального-minimax-mock)
- [2. Полный `tts_audio_bench`](#2-полный-tts_audio_bench)
- [3. Запуск `tts_node` с переопределённым endpoint](#3-запуск-tts_node-с-переопределённым-endpoint)
- [4. Capture harness для AudioData](#4-capture-harness-для-audiodata)
- [5. ffmpeg-сценарии](#5-ffmpeg-сценарии)
- [Acceptance и артефакты](#acceptance-и-артефакты)
- [Диагностика](#диагностика)
- [Blocker и минимальный change](#blocker-и-минимальный-change)

## Что проверяется

Тестовый контракт для этого bench:

| Gate | Проверка | Критерий успеха |
|---|---|---|
| Сборка | `rob_box_llm` и `rob_box_voice` | `colcon build` завершился с кодом 0; на host без ROS используется Python fallback |
| Endpoint | MiniMax T2A v2 | mock отвечает на `/v1/t2a_v2`; production provider получает тот же JSON/SSE envelope |
| Аудиоформат | `AudioData.data` | raw `int16` little-endian, mono, 16 kHz; длина bytes кратна 2 |
| QoS | publisher `tts_node` | `KEEP_LAST`, depth 10, `VOLATILE`, `best_effort` по умолчанию |
| TTFA | time-to-first-AudioData | поле `ttfa_s` или `ttfa_ms` записано в отчёт; для streaming получено не менее 2 чанков |
| WAV | сборка чанков | заголовок `sr=16000`, `channels=1`, `sample_width=2`; duration около 1.0 s |
| Joints | границы чанков | максимальный скачок не превышает threshold 4000 int16 units по умолчанию |
| Decode | PCM/WAV/MP3/OGG | `ffmpeg` и production `to_pcm_int16` дают валидный PCM; duration `1.0 ± 0.05 s` |
| Диагностика | stdout и JSON | subprocess-команды, exit code и причины ошибок доступны в логе/отчёте |

Есть два уровня проверки:

1. **Полный локальный bench** — рекомендуемый путь. Он запускает production-класс `TTSNode` с узким `rclpy` shim, локальным HTTP/SSE mock и отдельным subprocess subscriber. DDS на минимальном host заменён pipe, но payload `AudioData` и production audio pipeline проверяются теми же функциями.
2. **Реальный ROS 2 путь** — `tts_node` запускается как ROS-нода, а harness подписывается настоящим `rclpy` на `/voice/audio/speech`. Он требует ROS 2 Humble, `audio_common_msgs` и разрешённого endpoint blocker.

Физический динамик и ReSpeaker для проверки публикации `AudioData` не нужны. Реальное устройство влияет только на последующее воспроизведение через ALSA.

## Структура репозитория

```text
tts_audio_bench/
├── fixtures/                         # sine-файлы PCM/WAV/MP3/OGG
├── artifacts/                        # WAV, JSON summary и decode artifacts
├── logs/                             # bench.log
└── scripts/
    ├── build_bench.sh                # colcon build или Python fallback
    ├── make_fixture.py               # регенерация fixtures
    ├── run_bench.py                  # orchestrator, 8 сценариев по умолчанию
    ├── mock_minimax_server.py        # mock, используемый run_bench
    ├── real_subscriber.py             # отдельный JSON-lines → WAV subscriber
    ├── ros_stub.py                   # узкий rclpy/audio shim для minimal host
    ├── audio_subscriber.py            # in-process WAV recorder
    ├── audio_validator.py             # duration/header/joints checks
    └── test_*.py                      # unit tests bench helpers

tools/
├── mock_minimax_server.py             # standalone mock для ручного запуска/curl
├── audio_capture_harness/             # ROS2/stdin capture и JSON report
└── ffmpeg_decode/                     # четыре самостоятельных ffmpeg-сценария

src/rob_box_llm/                       # MiniMaxTTSProvider и TTS contract
src/rob_box_voice/                     # production tts_node и audio_transcode
BLOCKERS.md                            # endpoint blocker; production-код не менялся
```

## Зафиксированное окружение

Ниже — reference snapshot, зафиксированный в корневом `README.md` и использованный для зелёного прогона. Команды в следующем разделе печатают фактические значения вместо доверия к таблице.

| Компонент | Зафиксированное значение | Где проверять |
|---|---|---|
| OS | Debian GNU/Linux 13 (trixie) | `/etc/os-release` |
| Kernel | `6.8.0-124-generic` | `uname -r` |
| ROS 2 | Humble, `/opt/ros/humble` | `ROS_DISTRO`, setup path |
| Python для ROS | `3.10.12`, `/usr/bin/python3.10` | shebang ROS/entry points |
| System Python на bench host | `3.11.15`, `/usr/local/bin/python3` | `python3 --version` |
| `colcon-common-extensions` | `0.26.0` | `colcon version-check`/apt |
| `rosdep` | `0.26.0` | apt |
| `ffmpeg` | `7.1.5-0+deb13u1` | `ffmpeg -version` |
| `pytest` | `9.1.1` | `python -m pip show pytest` |
| `pytest-asyncio` | `1.4.0` | `python -m pip show pytest-asyncio` |
| `pytest-cov` | `7.1.0` | `python -m pip show pytest-cov` |
| `httpx` | `0.28.1` | `python -m pip show httpx` |
| `openai` | `2.46.0` | `python -m pip show openai` |
| `rob_box_llm` Python package | `0.2.1` | `src/rob_box_llm/setup.py` |
| `rob_box_llm` ROS package.xml | `0.1.0` | `src/rob_box_llm/package.xml` |
| `rob_box_voice` Python package | `0.1.0` | `src/rob_box_voice/setup.py` |
| `rob_box_voice` ROS package.xml | `0.1.0` | `src/rob_box_voice/package.xml` |
| Audio contract | mono, signed 16-bit LE, 16 kHz | `tts_node` + harness |
| ALSA loopback | unavailable on reference kernel | `modinfo snd_aloop` |

`rob_box_llm` имеет расхождение версий: `setup.py` содержит `0.2.1`, а `package.xml` — `0.1.0`. Это не меняйте в рамках test-bench; при публикации пакета версию нужно синхронизировать отдельной задачей.

### Снять fingerprint перед прогоном

```bash
set -u
printf '%s\n' '--- OS ---'
. /etc/os-release
printf 'PRETTY_NAME=%s\nVERSION_ID=%s\n' "${PRETTY_NAME:-unknown}" "${VERSION_ID:-unknown}"
printf 'kernel=%s\n' "$(uname -r)"

printf '%s\n' '--- Python ---'
python3 --version
command -v python3.10 >/dev/null && python3.10 --version || true

printf '%s\n' '--- ROS ---'
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-unset}"
test -f /opt/ros/humble/setup.bash && echo 'ROS2 Humble setup: present' || echo 'ROS2 Humble setup: absent'
command -v ros2 >/dev/null && ros2 pkg prefix rclpy || true

printf '%s\n' '--- media/tools ---'
command -v ffmpeg >/dev/null && ffmpeg -version | sed -n '1p' || echo 'ffmpeg: absent'
command -v ffprobe >/dev/null && ffprobe -version | sed -n '1p' || echo 'ffprobe: absent'
command -v colcon >/dev/null && colcon version-check || echo 'colcon: absent'

printf '%s\n' '--- Python packages ---'
python3 -m pip show pytest pytest-asyncio pytest-cov httpx openai 2>/dev/null \
  | grep -E '^(Name|Version):' || true

printf '%s\n' '--- rob_box package versions ---'
grep -nE "^(version|package_name)" src/rob_box_llm/setup.py src/rob_box_voice/setup.py
grep -nE '<(name|version)>' src/rob_box_llm/package.xml src/rob_box_voice/package.xml
```

> На минимальном Debian shell часть команд может показать `absent`. Это не означает, что код сломан: такой host использует fallback bench после установки Python-зависимостей либо запускает ROS внутри проектного Docker-образа.

## Подготовка окружения

### Вариант A: ROS 2 Humble / проектный контейнер

В контейнере или host, где `/opt/ros/humble/setup.bash` существует:

```bash
cd <repo-root>
source /opt/ros/humble/setup.bash

# Установить ROS-зависимости, если они ещё не установлены.
rosdep install --from-paths src --ignore-src -r -y

# Установить Python package и dev-зависимости rob_box_llm.
python3.10 -m pip install -e 'src/rob_box_llm[dev]'

# Сборка только пакетов, которые участвуют в bench.
colcon build --symlink-install \
  --packages-select rob_box_llm rob_box_voice
source install/setup.bash
```

Или используйте общий helper:

```bash
bash tts_audio_bench/scripts/build_bench.sh
```

При наличии одновременно `colcon` и `/opt/ros/humble/setup.bash` helper выполняет настоящий build. Иначе он выполняет Python import/compile fallback и завершается кодом 0 только если импорт `rob_box_llm`, `minimax_tts`, `audio_transcode` и compile `tts_node.py` прошли.

### Вариант B: minimal host без ROS 2

Нужны Python-зависимости bench и системные `ffmpeg`/`ffprobe`:

```bash
cd <repo-root>
python3 -m pip install -e 'src/rob_box_llm[dev]'
python3 -m pip install numpy

# Debian/Ubuntu, если tools отсутствуют и у пользователя есть права apt:
# sudo apt-get update && sudo apt-get install -y ffmpeg

python3 tts_audio_bench/scripts/make_fixture.py
```

`tts_audio_bench` использует локальные fixture и mock. Для обычного прогона не задавайте `MINIMAX_API_KEY`: полный bench не должен обращаться к интернету.

## Быстрый прогон за час

Команды ниже образуют минимальный путь от нуля до проверенного отчёта:

```bash
cd <repo-root>

# 1. Зафиксировать окружение в терминале/CI-логе.
#    Скопируйте fingerprint-команды из предыдущего раздела.

# 2. Проверить/создать детерминированные audio fixtures.
python3 tts_audio_bench/scripts/make_fixture.py

# 3. Проверить Python unit tests harness.
python3 -m pytest tools/audio_capture_harness/test_audio_capture_harness.py -q

# 4. Прогнать полный bench: PCM, WAV, MP3, OGG, streaming TTFA и 3 error paths.
python3 -m tts_audio_bench.scripts.run_bench --scenarios all

# 5. Прогнать четыре независимых ffmpeg-сценария.
bash tools/ffmpeg_decode/run_all.sh

# 6. Проверить итоговые файлы.
python3 - <<'PY'
import json
from pathlib import Path
p = Path('tts_audio_bench/artifacts/bench-summary.json')
data = json.loads(p.read_text())
print(f"bench: {data['passed']}/{data['total']}")
for result in data['results']:
    print(result['name'], 'OK' if result['ok'] else 'FAIL',
          f"ttfa_ms={result.get('ttfa_ms')}",
          f"bytes={result.get('bytes_total')}")
PY
```

Ожидаемый reference result полного bench: `8/8 scenarios passed`. При изменении CPU, ffmpeg или cold-start время TTFA может отличаться; фиксированными являются наличие измерения, формат, duration и exit code, а не конкретные миллисекунды.

## 1. Запуск локального MiniMax mock

### Standalone mock

Из корня репозитория:

```bash
python3 tools/mock_minimax_server.py \
  --port 18080 \
  --fixtures tts_audio_bench/fixtures \
  --scenario pcm-chunked
```

Сервер остаётся foreground-процессом. В другом терминале:

```bash
curl --fail-with-body -sS http://127.0.0.1:18080/health
```

`/health` должен вернуть JSON со `status=ok`, `fixtures_root` и активными параметрами сценария. Остановить mock: `Ctrl-C`.

Для streaming-сценария:

```bash
python3 tools/mock_minimax_server.py \
  --port 18080 \
  --fixtures tts_audio_bench/fixtures \
  --scenario wav-streaming \
  --chunk-count 4 \
  --chunk-delay-ms 50
```

Сценарий задаёт defaults для streaming, но сам запрос должен содержать `"stream": true`:

```bash
curl --fail-with-body -N -sS -X POST \
  'http://127.0.0.1:18080/v1/t2a_v2?GroupId=local-test' \
  -H 'Authorization: Bearer bench-key' \
  -H 'Content-Type: application/json' \
  -d '{
    "model": "speech-02-hd",
    "text": "hello",
    "voice_setting": {"voice_id": "male-qn-qingse"},
    "audio_setting": {"format": "wav", "sample_rate": 16000, "channel": 1},
    "stream": true
  }'
```

Для non-streaming PCM:

```bash
curl --fail-with-body -sS -X POST \
  'http://127.0.0.1:18080/v1/t2a_v2?GroupId=local-test' \
  -H 'Authorization: Bearer bench-key' \
  -H 'Content-Type: application/json' \
  -d '{
    "model": "speech-02-hd",
    "text": "hello",
    "voice_setting": {"voice_id": "male-qn-qingse"},
    "audio_setting": {"format": "pcm", "sample_rate": 16000, "channel": 1},
    "stream": false
  }'
```

Mock принимает любое непустое значение `Bearer` и `GroupId`; реальные секреты не нужны и не должны попадать в этот прогон.

### Сценарии mock

| CLI scenario | Ответ | Для чего |
|---|---|---|
| `pcm-chunked` | JSON, PCM | базовый single-shot |
| `wav-streaming` | SSE, обычно 4 чанка | TTFA и streaming |
| `mp3` | JSON, MP3 | compressed decode |
| `ogg` | JSON, OGG или MP3 fallback | OGG path |
| `auth-error` | `base_resp.status_code=1001` | auth error/no retry |

Query overrides: `chunk_count`, `chunk_delay_ms`, `fail_status`, `fail_msg`.

## 2. Полный `tts_audio_bench`

### Запуск

```bash
# Регистрация package/fixtures не требуется: запуск из worktree root.
python3 tts_audio_bench/scripts/make_fixture.py
python3 -m tts_audio_bench.scripts.run_bench
```

По умолчанию запускаются восемь сценариев:

```text
format_pcm
format_wav
format_mp3
format_ogg
streaming_ttfa_pcm
rate_limit_exhausted
auth_forbidden
bad_request
```

Выбрать subset можно через запятую:

```bash
python3 -m tts_audio_bench.scripts.run_bench --scenarios pcm,wav
python3 -m tts_audio_bench.scripts.run_bench --scenarios ttfa
python3 -m tts_audio_bench.scripts.run_bench --scenarios rate_limit,auth,bad_request
```

Если порт `18080` занят:

```bash
python3 -m tts_audio_bench.scripts.run_bench --port 19080 --scenarios all
```

`run_bench.py` поднимает mock на `PORT` и streaming mock на `PORT+10`. Error scenarios используют `PORT+1`, `PORT+2` и `PORT+3`. Убедитесь, что все эти порты свободны на `127.0.0.1`.

### Что происходит внутри

1. `run_bench.py` импортирует production `TTSNode` после установки `ros_stub` и lightweight stubs для тяжёлых hardware libraries.
2. `_make_test_node()` явно создаёт `MiniMaxTTSProvider(base_url=http://127.0.0.1:<port>)`; это единственный локальный endpoint override, доступный текущей ревизии без production change.
3. Mock возвращает fixture. Для MP3/OGG вызывается production `audio_transcode.to_pcm_int16`, использующий ffmpeg fallback.
4. `TTSNode._prepare_audio_for_topic()` и `_publish_audio()` создают настоящие `AudioData`-подобные сообщения с `int16 LE` payload.
5. In-process `WavRecorder` и отдельный `real_subscriber.py` записывают независимые WAV.
6. `audio_validator` проверяет header, duration и joints. Streaming scenario дополнительно требует несколько сообщений и вычисляет TTFA.
7. Error scenarios проверяют typed errors: `TTSRateLimitError`, `TTSAuthError`, `TTSBadRequestError`.

### Сборка/тесты bench

```bash
# Build helper: настоящий colcon в ROS-контейнере, fallback на minimal host.
bash tts_audio_bench/scripts/build_bench.sh

# Unit tests bench helpers.
python3 -m pytest tts_audio_bench/scripts/ -q

# Unit tests capture harness.
python3 -m pytest tools/audio_capture_harness/test_audio_capture_harness.py -q
```

### Exit codes и артефакты

`run_bench.py`:

- `0` — все выбранные сценарии прошли;
- `1` — хотя бы один сценарий не прошёл;
- `2` — окружение не готово, например отсутствует required import или `ffmpeg`.

После запуска проверьте:

```text
tts_audio_bench/artifacts/bench-summary.json
  machine-readable result: passed, total, results[], warnings[]
tts_audio_bench/artifacts/<scenario>.wav
  in-process recorder output
tts_audio_bench/artifacts/<scenario>_real_subscriber.wav
  separate-process subscriber output
tts_audio_bench/artifacts/ffmpeg_decode/<scenario>/decoded.raw
tts_audio_bench/artifacts/ffmpeg_decode/<scenario>/decoded.wav
tts_audio_bench/logs/bench.log
  stdout-equivalent log, HTTP status, TTFA, errors and summary
```

Не коммитьте сгенерированные `artifacts/`, `logs/`, `.colcon-bench/` или временные WAV, если они не нужны как evidence конкретного прогона.

## 3. Запуск `tts_node` с переопределённым endpoint

### Текущее состояние: команда заблокирована

`tts_node` имеет ROS-параметры `minimax_api_key` и `minimax_group_id`, но **не имеет** `minimax_base_url`. В lazy initializer `src/rob_box_voice/rob_box_voice/tts_node.py` создаётся:

```text
MiniMaxTTSProvider(
    api_key=self.minimax_api_key,
    group_id=self.minimax_group_id,
    default_voice=self.minimax_voice,
    default_model=self.minimax_model,
    timeout=self.minimax_timeout,
)
```

`MiniMaxTTSProvider` по умолчанию использует `https://api.minimax.io`, а его constructor текущей ревизии не читает `MINIMAX_BASE_URL`. Поэтому следующая команда **не переключает endpoint на mock на текущей ревизии** и не должна запускаться с реальным ключом:

```bash
# Только как диагностическая команда после исправления blocker.
export MINIMAX_BASE_URL=http://127.0.0.1:18080
export MINIMAX_API_KEY=bench-key
export MINIMAX_GROUP_ID=bench-group

ros2 run rob_box_voice tts_node --ros-args \
  -p provider:=minimax \
  -p minimax_api_key:=bench-key \
  -p minimax_group_id:=bench-group \
  -p minimax_voice:=male-qn-qingse \
  -p minimax_model:=speech-02-hd \
  -p minimax_format:=pcm \
  -p minimax_sample_rate:=32000 \
  -p audio_output_sample_rate:=16000 \
  -p audio_qos_reliability:=best_effort \
  -p audio_qos_depth:=10 \
  -p minimax_streaming:=false
```

В текущем состоянии эта нода либо обратится к реальному MiniMax, либо завершится typed auth/network error. Это ожидаемое проявление blocker, а не acceptance failure mock-сервера.

### После разрешения blocker

После применения минимального изменения из [BLOCKERS.md](BLOCKERS.md) порядок такой:

```bash
# Terminal A: mock
python3 tools/mock_minimax_server.py \
  --port 18080 \
  --fixtures tts_audio_bench/fixtures \
  --scenario pcm-chunked

# Terminal B: environment + ROS setup
source /opt/ros/humble/setup.bash
source install/setup.bash
export MINIMAX_BASE_URL=http://127.0.0.1:18080
export MINIMAX_API_KEY=bench-key
export MINIMAX_GROUP_ID=bench-group
```

Не передавайте настоящий ключ в командной строке, если shell history сохраняется. В mock достаточно `bench-key`.

## 4. Capture harness для AudioData

`tools/audio_capture_harness` умеет два транспорта:

- `ros2` — настоящий `rclpy` subscriber на `/voice/audio/speech`; нужен ROS 2 Humble и `audio_common_msgs`;
- `stdin` — JSON-lines wire format; используется minimal host и acceptance runner.

### Реальный ROS 2 capture

Выполняйте после build и после разрешения endpoint blocker:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 -m tools.audio_capture_harness.audio_capture_harness \
  --transport ros2 \
  --audio-topic /voice/audio/speech \
  --qos-reliability best_effort \
  --qos-depth 10 \
  --expected-sample-rate 16000 \
  --expected-channels 1 \
  --expected-sample-width 2 \
  --expected-duration-s 1.0 \
  --expected-duration-tol-s 0.5 \
  --min-msgs 1 \
  --capture-max-seconds 1.0 \
  --deadline-s 30 \
  --spawn-warmup-s 1 \
  --publish-after-s 1 \
  --tts-node-cmd 'ros2 run rob_box_voice tts_node --ros-args \
    -p provider:=minimax \
    -p minimax_api_key:=bench-key \
    -p minimax_group_id:=bench-group \
    -p minimax_format:=pcm \
    -p minimax_sample_rate:=32000 \
    -p audio_output_sample_rate:=16000' \
  --publish-topic /voice/tts/request \
  --publish-payload '{"ssml":"<speak>тест test bench</speak>"}' \
  --wav-out tts_audio_bench/artifacts/ros2_capture.wav \
  --report-out tts_audio_bench/artifacts/ros2_capture.json \
  --log-file tts_audio_bench/logs/ros2_capture.log
```

Harness запускает `tts_node` как subprocess, создаёт subscriber, публикует `std_msgs/msg/String` на `/voice/tts/request`, записывает AudioData и завершает capture после idle period/deadline. ROS-нода и subscriber должны быть в одном `ROS_DOMAIN_ID` и сетевом namespace.

`audio_topic`, QoS и sample rate должны совпадать с параметрами publisher. Production defaults:

```text
audio_topic=/voice/audio/speech
audio_output_sample_rate=16000
audio_qos_reliability=best_effort
audio_qos_depth=10
```

### Acceptance runner без ручного ROS CLI

```bash
python3 tools/audio_capture_harness/acceptance.py \
  --wav-out /tmp/acceptance_capture.wav \
  --report-out /tmp/acceptance_capture.json \
  --expected-sample-rate 16000 \
  --expected-duration-s 1.0 \
  --expected-duration-tol-s 0.05
```

Runner вызывает production `TTSNode._publish_audio`, подаёт JSON-lines в harness stdin и проверяет WAV round-trip. Reference output:

```text
[acceptance] harness rc=0
[acceptance] duration_s: 1.0000
[acceptance] ttfa_s:     0.1
[acceptance] joints_ok:  True
[acceptance] header_ok:  True
[acceptance] ok:         True
[acceptance] wav: sr=16000 ch=1 sw=2 frames=16000
```

Число `ttfa_s` в acceptance runner синтетическое (`publish_t_s` задаётся runner-ом); для сетевой latency используйте `--transport ros2` или streaming scenario полного bench.

### Ручной stdin smoke test

Строка stdin содержит base64 payload и out-of-band audio metadata:

```bash
python3 - <<'PY' | python3 -m tools.audio_capture_harness.audio_capture_harness \
  --transport stdin \
  --wav-out /tmp/stdin_capture.wav \
  --report-out /tmp/stdin_capture.json \
  --expected-sample-rate 16000 \
  --expected-duration-s 1.0 \
  --expected-duration-tol-s 0.05
import base64, json, math, struct
sr = 16000
pcm = b''.join(struct.pack('<h', int(12000 * math.sin(2 * math.pi * 440 * i / sr)))
               for i in range(sr))
print(json.dumps({
    'frame_index': 0,
    'publish_t_s': 0.1,
    'sample_rate': sr,
    'channels': 1,
    'sample_width': 2,
    'layout': 'little_endian',
    'data_b64': base64.b64encode(pcm).decode('ascii'),
}))
PY
```

Проверка JSON:

```bash
python3 - <<'PY'
import json
p = json.load(open('/tmp/stdin_capture.json'))
assert p['ok'] is True, p
assert p['header']['sample_rate'] == 16000
assert p['header']['channels'] == 1
assert p['header']['sample_width'] == 2
assert abs(p['duration_s'] - 1.0) <= 0.05
print('stdin capture: PASS')
PY
```

## 5. ffmpeg-сценарии

Сценарии находятся в `tools/ffmpeg_decode/`. Каждый сценарий:

1. берёт fixture из `tts_audio_bench/fixtures`;
2. запускает `ffmpeg` → raw `s16le`, mono, 16 kHz;
3. оборачивает raw в WAV;
4. запускает `ffprobe` и проверяет duration `1.0 ± 0.05 s`;
5. прогоняет realistic container через production `to_pcm_int16`.

### Все сценарии

```bash
bash tools/ffmpeg_decode/run_all.sh
```

Reference summary: `pcm`, `wav`, `mp3`, `ogg` — `4/4 OK`.

### Отдельные сценарии

```bash
bash tools/ffmpeg_decode/scenario_pcm.sh
bash tools/ffmpeg_decode/scenario_wav.sh
bash tools/ffmpeg_decode/scenario_mp3.sh
bash tools/ffmpeg_decode/scenario_ogg.sh
```

У каждого сценария первый аргумент — optional custom fixture:

```bash
bash tools/ffmpeg_decode/scenario_mp3.sh \
  tts_audio_bench/fixtures/mp3/sine_100hz_1.00s_32000.mp3
```

### Коды возврата

Для отдельного сценария:

| Код | Значение |
|---:|---|
| 0 | decode, duration и `to_pcm_int16` прошли |
| 1 | отсутствует `ffmpeg`, `ffprobe` или `python3` |
| 2 | fixture не найден |
| 3 | `ffmpeg` decode завершился ошибкой |
| 4 | duration вне допуска |
| 5 | production round-trip `to_pcm_int16` провалился |
| 6 | предварительная duration sanity check провалилась |

`run_all.sh` возвращает число неуспешных сценариев (`0` означает `4/4`). Все артефакты пишутся в `tts_audio_bench/artifacts/ffmpeg_decode/<scenario>/`.

## Acceptance и артефакты

### Машинная проверка полного summary

```bash
python3 - <<'PY'
import json
from pathlib import Path
p = Path('tts_audio_bench/artifacts/bench-summary.json')
data = json.loads(p.read_text())
assert data['passed'] == data['total'], data
assert data['total'] >= 1
for r in data['results']:
    assert r['ok'] is True, r
    if r['name'].startswith('format_') or r['name'] == 'streaming_ttfa_pcm':
        assert r['sample_rate'] == 16000, r
        assert r['joints_ok'] is True, r
        assert r['bytes_total'] > 0, r
print(f"bench acceptance: {data['passed']}/{data['total']} PASS")
PY
```

### WAV header/duration check

```bash
python3 - <<'PY'
from pathlib import Path
import wave
for p in sorted(Path('tts_audio_bench/artifacts').glob('*.wav')):
    with wave.open(str(p), 'rb') as w:
        sr, ch, sw, frames = w.getframerate(), w.getnchannels(), w.getsampwidth(), w.getnframes()
    duration = frames / sr
    print(f'{p}: sr={sr} ch={ch} sw={sw} frames={frames} duration={duration:.4f}s')
    assert (sr, ch, sw) == (16000, 1, 2), p
    assert 0.5 <= duration <= 3.0, p
PY
```

### Что сохранить в evidence

Для issue/PR достаточно приложить:

- `tts_audio_bench/artifacts/bench-summary.json`;
- `tts_audio_bench/logs/bench.log` или stdout полного прогона;
- один WAV из in-process и соответствующий `_real_subscriber.wav`;
- итог `bash tools/ffmpeg_decode/run_all.sh`;
- fingerprint окружения.

Не включайте API keys, Authorization headers с реальными значениями и полные секретные env dumps. Mock использует placeholder credentials.

## Диагностика

### `ModuleNotFoundError: httpx` или `openai`

Установите package extras в тот Python, которым запускаете bench:

```bash
python3 -m pip install -e 'src/rob_box_llm[dev]'
python3 -c 'import httpx, openai; print(httpx.__version__, openai.__version__)'
```

Для ROS entry points используйте Python 3.10, соответствующий ROS Humble; не запускайте ROS-бинарии под Python 3.11 на reference host.

### `ffmpeg: command not found` / `ffprobe: command not found`

Установите системный пакет `ffmpeg` или запустите сценарии внутри проектного образа, где он есть:

```bash
sudo apt-get update
sudo apt-get install -y ffmpeg
```

Проверьте:

```bash
ffmpeg -version | sed -n '1p'
ffprobe -version | sed -n '1p'
```

### `rclpy` не импортируется на Debian 13

Это ожидаемо для minimal host: ROS Humble packages собраны под Ubuntu Jammy/Python 3.10 и не являются нативным Debian 13 stack. Используйте `tts_audio_bench` с `ros_stub` либо Docker-образ с ROS Humble. Не force-install jammy `.deb` в production Python: reference attempt упирался в `libpython3.10`, `spdlog/fmt` ABI и `librcl_logging_spdlog.so`.

### `librcl_logging_spdlog.so` / NumPy ABI error

Для test-bench применяйте documented workaround из корневого `README.md`, не изменяя production source:

```bash
# Только в disposable test environment и только после подтверждения пути.
apt-get install -y ros-humble-rcl-logging-noop
mv /opt/ros/humble/lib/librcl_logging_spdlog.so{,.bak}
ln -s librcl_logging_noop.so /opt/ros/humble/lib/librcl_logging_spdlog.so
python3.10 -m pip install --user --no-cache-dir 'numpy<2'
```

### Mock отвечает, но `tts_node` идёт в интернет

Это endpoint blocker. Проверьте, что вы не пытаетесь использовать `MINIMAX_BASE_URL` на текущей ревизии. Полный bench должен использовать direct constructor override внутри `_make_test_node`; production `tts_node` требует change из [BLOCKERS.md](BLOCKERS.md). Не подставляйте настоящий API key для проверки mock.

### `no AudioData frames received`

Проверьте по порядку:

```bash
ros2 node list
ros2 topic list | grep '/voice/audio/speech\|/voice/tts/request'
ros2 topic info /voice/audio/speech -v
```

Затем сверяйте `audio_topic`, QoS, `audio_output_sample_rate`, `ROS_DOMAIN_ID` и наличие subscriber. Для stdin transport проверьте, что каждая JSON-строка содержит непустой валидный `data_b64`.

### ALSA/ReSpeaker warnings

Capture gate проверяет публикацию ROS AudioData, а не физическое воспроизведение. `sounddevice` может сообщить, что ReSpeaker/ALSA output отсутствует. Для production playback отдельно настройте `module-null-sink`, `module-loopback` или реальное USB-устройство; не подменяйте этим результат WAV capture.

## Blocker и минимальный change

Существующий [BLOCKERS.md](BLOCKERS.md) — authoritative record для текущего blocker. Production-код в рамках этой документационной задачи не менялся.

Причина:

- `MiniMaxTTSProvider.__init__` принимает явный `base_url`, но не читает `MINIMAX_BASE_URL`, если аргумент не передан;
- `tts_node._synthesize_minimax_async` создаёт provider без `base_url=`;
- ROS-параметр `minimax_base_url` не объявлен;
- поэтому локальный `tools/mock_minimax_server.py` невозможно подключить к реальному `ros2 run rob_box_voice tts_node` только env-переменной.

Минимальный change, который нужно реализовать отдельной backend-задачей и покрыть тестом:

```python
# src/rob_box_llm/rob_box_llm/providers/minimax_tts.py
self._base_url = (
    base_url
    if base_url != self.DEFAULT_BASE_URL
    else os.getenv('MINIMAX_BASE_URL', self.DEFAULT_BASE_URL)
).rstrip('/')
```

```python
# src/rob_box_voice/rob_box_voice/tts_node.py
self.minimax_provider = MiniMaxTTSProvider(
    api_key=self.minimax_api_key,
    group_id=self.minimax_group_id,
    base_url=os.getenv('MINIMAX_BASE_URL', MiniMaxTTSProvider.DEFAULT_BASE_URL),
    default_voice=self.minimax_voice,
    default_model=self.minimax_model,
    timeout=self.minimax_timeout,
)
```

Код выше — описание требуемого изменения, не применённый patch. После его review/merge повторите реальный ROS2 capture из раздела [Capture harness](#4-capture-harness-для-audiodata) с `MINIMAX_BASE_URL=http://127.0.0.1:18080`.

## Итоговый checklist

Перед закрытием прогона отметьте всё выполненное:

- [ ] fingerprint окружения сохранён;
- [ ] `rob_box_llm` и `rob_box_voice` собраны или Python fallback прошёл;
- [ ] fixtures существуют и не были взяты из внешнего API;
- [ ] mock health check вернул `status=ok`;
- [ ] полный bench завершился `8/8` (или выбранный subset — `N/N`);
- [ ] `bench-summary.json` содержит `ok=true` для каждого выбранного сценария;
- [ ] WAV header: `16000 Hz`, mono, `sample_width=2`;
- [ ] duration и joints прошли;
- [ ] streaming TTFA записан и получено несколько AudioData chunks;
- [ ] `run_all.sh` завершился `0` (`4/4`);
- [ ] blocker endpoint явно указан, если реальный ROS2 path не запускался;
- [ ] в evidence нет реальных ключей и приватных payloads.
