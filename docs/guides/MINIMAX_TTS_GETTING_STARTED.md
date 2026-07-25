# MiniMax TTS — Getting Started

> **Цель документа:** за 5–10 минут провести читателя от чистого робота до работающего синтеза MiniMax TTS с публикацией аудио в ROS2-топик `/voice/audio/speech`.
>
> Это **минимальный** путь. Полный пользовательский гайд (все ROS-параметры, troubleshooting, секреты в docker-compose) — в [MINIMAX_TTS.md](./MINIMAX_TTS.md). API-контракт `MiniMaxTTSProvider` — в [docs/api/MINIMAX_TTS.md](../api/MINIMAX_TTS.md).

---

## Содержание

1. [Что понадобится](#1-что-понадобится)
2. [Шаг 1 — API ключ](#2-шаг-1--api-ключ)
3. [Шаг 2 — ENV-переменные](#3-шаг-2--env-переменные)
4. [Шаг 3 — Минимальный синтез из Python](#4-шаг-3--минимальный-синтез-из-python)
5. [Шаг 4 — Публикация в ROS2](#5-шаг-4--публикация-в-ros2)
6. [Шаг 5 — Полный цикл через `tts_node`](#6-шаг-5--полный-цикл-через-tts_node)
7. [Что дальше](#7-что-дальше)

---

## 1. Что понадобится

- Рабочий ROS 2 Humble workspace с собранными пакетами `rob_box_llm` + `rob_box_voice`.
- Учётная запись MiniMax с выпущенным API-ключом.
- Сетевой доступ из робота (или dev-машины) до `api.minimax.io:443`.
- Python 3.10 (используется ROS Humble).

Проверить, что workspace собран:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg list | grep -E 'rob_box_(llm|voice)'
```

Ожидаемо: оба пакета в списке.

---

## 2. Шаг 1 — API ключ

1. Зарегистрируйтесь: <https://platform.minimaxi.com/user-center/basic-information/interface-key>.
2. Создайте **Interface Key** — это JWT вида `eyJhbGciOi...` (без префикса `Bearer`).
3. Скопируйте **Group ID** с той же страницы (Account / Group).

---

## 3. Шаг 2 — ENV-переменные

Секреты считываются провайдером из ENV при инициализации:

```bash
export MINIMAX_API_KEY="eyJhbGciOi..."        # без Bearer
export MINIMAX_GROUP_ID="123456789012345678"
# Опционально:
# export MINIMAX_TTS_BASE_URL="https://api.minimax.io"
# export MINIMAX_TTS_VOICE="Russian_CalmWoman"
# export MINIMAX_TTS_MAX_CONCURRENCY=1
```

Быстрая проверка — `curl` напрямую к MiniMax:

```bash
curl -sS -X POST "https://api.minimax.io/v1/t2a_v2?GroupId=$MINIMAX_GROUP_ID" \
  -H "Authorization: Bearer $MINIMAX_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{"model":"speech-02-hd","text":"hello","stream":false,
       "voice_setting":{"voice_id":"male-qn-qingse","language":"English"},
       "audio_setting":{"sample_rate":32000,"bitrate":128000,"format":"pcm","channel":1}}' \
  | jq '.base_resp, .extra_info.usage_characters'
```

Ожидаемо: `"status_code": 0`, `"status_msg": "success"`.

---

## 4. Шаг 3 — Минимальный синтез из Python

Готовый пример [`examples/tts_minimax_example.py`](../../examples/tts_minimax_example.py)
читает настройки из ENV, получает провайдер `minimax` через registry/factory и
сохраняет raw PCM как корректный WAV-файл стандартным модулем `wave`.

Из корня репозитория выполните:

```bash
PYTHONPATH=src/rob_box_llm \
python3 examples/tts_minimax_example.py \
  --text "Привет, я MiniMax TTS!" \
  --output /tmp/minimax-hello.wav

python3 - <<'PY'
import wave

with wave.open("/tmp/minimax-hello.wav", "rb") as audio:
    print(audio.getparams())
PY
```

В результате появится mono WAV: signed 16-bit PCM, 24 000 Hz. Прослушать его
можно командой `aplay /tmp/minimax-hello.wav` или
`ffplay /tmp/minimax-hello.wav`.

Пример использует `TTSProviderFactory.create("minimax", ...)`, а не создаёт
`MiniMaxTTSProvider` напрямую. Это тот же composition-root путь, который
используют интеграции проекта.

---

## 5. Шаг 4 — Публикация в ROS2

`rob_box_voice` экспортирует стандартное сообщение `AudioData` (из `audio_common_msgs`). Минимальная нода, которая синтезирует через MiniMax и публикует в `/voice/audio/speech`:

```python
#!/usr/bin/env python3
"""minimax_say.py — пример ROS2-ноды, публикующей MiniMax TTS в /voice/audio/speech."""
import asyncio, os, rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData

from rob_box_llm import MiniMaxTTSProvider, TTSSettings, TTSFormat
from rob_box_llm.errors import TTSError


class MiniMaxSay(Node):
    def __init__(self):
        super().__init__("minimax_say")
        self.declare_parameter("text", "Привет, я MiniMax TTS!")
        self.declare_parameter("voice", "male-qn-qingse")
        self.declare_parameter("language", "ru")

        # Аудио-топик — тот же, что у tts_node (по умолчанию).
        self._pub = self.create_publisher(AudioData, "/voice/audio/speech", 10)

        self._provider = MiniMaxTTSProvider(
            api_key=os.environ.get("MINIMAX_API_KEY", ""),
            group_id=os.environ.get("MINIMAX_GROUP_ID", ""),
        )
        self.get_logger().info("minimax_say ready (provider=minimax)")

    async def say(self, text: str) -> None:
        try:
            audio = await self._provider.synthesize(
                text,
                settings=TTSSettings(
                    voice=self.get_parameter("voice").value,
                    language=self.get_parameter("language").value,
                    sample_rate=16000,    # ReSpeaker native
                    format=TTSFormat.PCM,
                ),
            )
        except TTSError as exc:
            self.get_logger().error(f"TTS failed: {exc}")
            return

        msg = AudioData()
        # AudioData.data — uint8[] (portable across ROS 2 distros — assigning
        # raw bytes works in some bindings but not all). MiniMax TTS returns
        # int16 little-endian PCM, so byte-by-byte is correct here.
        msg.data = list(audio.samples)
        msg.info.sample_rate = audio.sample_rate
        msg.info.channels = 1
        self._pub.publish(msg)
        self.get_logger().info(f"published {len(msg.data)} bytes @ {audio.sample_rate} Hz")

    def destroy_node(self):
        asyncio.run(self._provider.aclose())
        super().destroy_node()


def main():
    rclpy.init()
    node = MiniMaxSay()
    text = node.get_parameter("text").value
    try:
        asyncio.run(node.say(text))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Запуск:

```bash
source /opt/ros/humble/setup.bash
export MINIMAX_API_KEY="eyJhbGciOi..."
export MINIMAX_GROUP_ID="1234567890..."
ros2 run rob_box_voice minimax_say.py  # или python3 ./minimax_say.py
```

Подписчик на `/voice/audio/speech` (например, `audio_play_node` из `audio_common`) воспроизведёт фразу. Убедитесь, что **другой `tts_node` не запущен** одновременно — иначе будет конфликт за топик.

---

## 6. Шаг 5 — Полный цикл через `tts_node`

В продакшне ROS2-код **не должен** вызывать `MiniMaxTTSProvider` напрямую — синтез делегируется в `tts_node`. Через `tts_node` вы просто публикуете текст в топик, а вся остальная магия (miniMax-вызов, ретраи, кэширование, transcoding → 16 kHz для ReSpeaker) уже сделана.

### 6.1 Конфигурация

Добавьте в свой `voice_assistant.yaml` (или используйте шаблон [`examples/minimax_tts.yaml`](./examples/minimax_tts.yaml)):

```yaml
tts_node:
  ros__parameters:
    provider: "minimax"               # включить MiniMax (вместо yandex/silero)
    minimax_voice: "male-qn-qingse"   # MiniMax voice id
    minimax_model: "speech-02-hd"     # speech-02-hd | speech-02-turbo
    minimax_language: "ru"
    minimax_sample_rate: 32000
    minimax_format: "pcm"
    minimax_timeout: 30.0
    minimax_max_retries: 2
    minimax_streaming: false          # true → SSE
    audio_topic: "/voice/audio/speech"
```

**Секреты** (`minimax_api_key`, `minimax_group_id`) оставьте пустыми — `tts_node` сам подхватит их из ENV.

### 6.2 Запуск

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export MINIMAX_API_KEY="eyJhbGciOi..."
export MINIMAX_GROUP_ID="1234567890..."
ros2 launch rob_box_voice voice_assistant.launch.py provider:=minimax
```

### 6.3 Произнести фразу

В любой другой ROS2-ноде:

```python
from std_msgs.msg import String
pub = node.create_publisher(String, "/voice/tts/request", 10)
pub.publish(String(data="Привет, мир!"))
```

`tts_node` подхватит, синтезирует через MiniMax, опубликует PCM в `/voice/audio/speech`. На стандартном dev-стенде `audio_play_node` (audio_common_msgs) автоматически воспроизведёт.

### 6.4 Наблюдение за состоянием

`tts_node` публикует два диагностических топика:

```bash
ros2 topic echo /voice/tts/state     # "idle" | "synthesizing" | "playing"
ros2 topic echo /voice/tts/finished  # содержит duration_s, sample_rate, format
```

---

## 7. Что дальше

- **Troubleshooting** (TTSAuthError, TTSRateLimitError, секреты в логах) — [MINIMAX_TTS.md §7](./MINIMAX_TTS.md#7-troubleshooting).
- **Полный список параметров** `tts_node` для MiniMax — [MINIMAX_TTS.md §4](./MINIMAX_TTS.md#4-конфигурация-ros2-ноды-tts_node).
- **API reference** (все методы, исключения, TTSSettings-поля) — [docs/api/MINIMAX_TTS.md](../api/MINIMAX_TTS.md).
- **Регистрация кастомного провайдера** — [docs/architecture/tts-extension-points.md](../architecture/tts-extension-points.md).
- **Альтернативные голоса и языки** — [MINIMAX_TTS.md §5](./MINIMAX_TTS.md#5-поддерживаемые-голоса-и-языки).

---

**Версия документа:** 1.0 (2026-07-22) · **Покрывает:** `rob_box_llm>=0.2.1`, `rob_box_voice` (ветка `feature/harness-p0-foundation` / `wt/t_ac5f796b`).
