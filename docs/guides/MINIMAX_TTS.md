# MiniMax TTS — руководство пользователя

> Гайд по подключению `MiniMaxTTSProvider` к роботу и его использованию
> в коде и ROS2-нодах. Архитектурный контракт и решения описаны в
> [`docs/architecture/minimax-tts-architecture.md`](../architecture/minimax-tts-architecture.md)
> и [`docs/adr/0003-minimax-tts-architecture.md`](../adr/0003-minimax-tts-architecture.md);
> этот документ — **практическая инструкция**, которую можно дать
> оператору или интегратору.

---

## Содержание

1. [Что такое MiniMax TTS в контексте Rob Box](#1-что-такое-minimax-tts-в-контексте-rob-box)
2. [Получение API ключа](#2-получение-api-ключа)
3. [Настройка переменных окружения](#3-настройка-переменных-окружения)
4. [Конфигурация ROS2-ноды `tts_node`](#4-конфигурация-ros2-ноды-tts_node)
5. [Поддерживаемые голоса и языки](#5-поддерживаемые-голоса-и-языки)
6. [Примеры вызова](#6-примеры-вызова)
   - 6.1 [Из Python-кода](#61-из-python-кода)
   - 6.2 [Из ROS2-ноды](#62-из-ros2-ноды)
7. [Troubleshooting](#7-troubleshooting)
8. [Связанные документы](#8-связанные-документы)

---

## 1. Что такое MiniMax TTS в контексте Rob Box

`MiniMaxTTSProvider` — реализация абстрактного `TTSProvider` (см.
[`src/rob_box_llm/rob_box_llm/tts.py`](../../src/rob_box_llm/rob_box_llm/tts.py))
поверх HTTP-эндпоинта `POST https://api.minimax.io/v1/t2a_v2`.
Провайдер добавляет **третий опциональный канал синтеза речи** в ROS2-ноду
`tts_node` (пакет `rob_box_voice`) рядом с уже существующими Yandex Cloud
TTS gRPC v3 и офлайн-Silero v5.

| Сценарий | Что использовать |
|----------|------------------|
| Полностью офлайн, без сети | Silero v5 (текущий fallback) |
| Облачный TTS, ROBBOX-голос `anton`, минимальная настройка | Yandex Cloud TTS |
| Многоязычный TTS (RU/EN/ZH/…), эмоциональная окраска, выбор голоса из широкого каталога, без gRPC | **MiniMax TTS (opt-in)** |

Особенности интеграции в `tts_node`:

- Активируется **только** через ROS-параметр `tts_node.provider = "minimax"`.
- При ошибке MiniMax **нет автоматического fallback** в Silero/Yandex —
  пользователь явно выбрал MiniMax, и ошибка пробрасывается наверх,
  чтобы её можно было увидеть и обработать.
- Секреты (`MINIMAX_API_KEY`, `MINIMAX_GROUP_ID`) живут только в ENV,
  никогда не в launch-yaml и не в логах.

---

## 2. Получение API ключа

1. Зарегистрируйтесь на платформе MiniMax:
   <https://platform.minimaxi.com/user-center/basic-information/interface-key>.
2. В разделе **«Interface Key»** создайте новый ключ. MiniMax выдаёт JWT
   вида `eyJhbGciOi...` (без префикса `Bearer ` — префикс добавляет
   провайдер автоматически).
3. На той же странице, в разделе **«Group ID»**, скопируйте свой
   `MINIMAX_GROUP_ID` — цифро-буквенный идентификатор аккаунта. Он
   используется как query-параметр `GroupId=` при каждом запросе к T2A v2.
4. Пополните баланс (тарификация поминутная + по объёму синтеза;
   цены и пакеты — в личном кабинете).

> **Тестовая среда:** MiniMax даёт trial-кредиты новым аккаунтам; для
> smoke-теста хватает 1–2 минут синтеза.

---

## 3. Настройка переменных окружения

Секреты провайдера считываются из ENV при инициализации
`MiniMaxTTSProvider` (и при старте `tts_node`). Шаблон лежит в
[`docker/vision/.env.secrets.template`](../../docker/vision/.env.secrets.template):

```bash
# ============================================================
# MiniMax (для TTSNode — опциональный синтез речи через MiniMax T2A v2)
# ============================================================
# Использование: установить ROS-параметр tts_node.provider=minimax и
# заполнить оба ключа ниже. Без них MiniMax-провайдер не активируется,
# нода продолжит использовать Yandex gRPC + Silero.

# MiniMax API Key (Bearer-токен)
# Где взять: https://platform.minimaxi.com/user-center/basic-information/interface-key
# Формат: строка без префикса, например "eyJhbGciOi..."
MINIMAX_API_KEY=

# MiniMax Group ID
# Где взять: там же, в разделе "Account / Group"
# Формат: цифро-буквенный идентификатор
MINIMAX_GROUP_ID=
```

Минимальная локальная установка для отладки:

```bash
# 1. Экспортируйте секреты в текущую shell-сессию:
export MINIMAX_API_KEY="eyJhbGciOi...   # ваш ключ"
export MINIMAX_GROUP_ID="1234567890..."  # ваш group id"

# 2. (опционально) проверьте доступ к API одной командой curl:
curl -sS -X POST "https://api.minimax.io/v1/t2a_v2?GroupId=$MINIMAX_GROUP_ID" \
  -H "Authorization: Bearer $MINIMAX_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{"model":"speech-02-hd","text":"hello","stream":false,"voice_setting":{"voice_id":"male-qn-qingse","language":"English"},"audio_setting":{"sample_rate":32000,"bitrate":128000,"format":"pcm","channel":1}}' \
  | jq '.data.audio_sample_rate,.base_resp'
```

В docker-compose (production / dev-стенд) секреты прокидываются через
`docker/vision/.env.secrets` (подмонтирован read-only в контейнер
`voice_assistant`). Сам файл `.env.secrets` в Git не коммитится —
добавьте его в `.gitignore` локально, если собираетесь копировать
шаблон вручную.

> **Приоритет источников:** ROS-параметр `minimax_api_key` /
> `minimax_group_id` → ENV `MINIMAX_API_KEY` / `MINIMAX_GROUP_ID`.
> Для остальных полей (`voice`, `model`, `language`, …) ENV-fallback
> **не** реализован — это намеренное решение, чтобы не плодить
> источников правды (см. ADR-0003 §2.3).

---

## 4. Конфигурация ROS2-ноды `tts_node`

`tts_node` (`src/rob_box_voice/rob_box_voice/tts_node.py`) объявляет
следующие параметры (см. `voice_assistant.yaml`):

```yaml
tts_node:
  ros__parameters:
    # === Общий выбор провайдера ===
    provider: "yandex"          # "yandex" | "silero" | "minimax"

    # ===== MiniMax TTS (opt-in, см. ADR-0003) =====
    # Секреты берутся из ENV; в yaml их писать НЕ нужно.
    minimax_api_key: ""           # → fallback на os.getenv("MINIMAX_API_KEY")
    minimax_group_id: ""          # → fallback на os.getenv("MINIMAX_GROUP_ID")
    minimax_voice: "male-qn-qingse"   # см. §5
    minimax_model: "speech-02-hd"     # "speech-02-hd" | "speech-02-turbo"
    minimax_language: "ru"            # "ru" | "en" | "zh" | полное имя "Russian"
    minimax_speed: 1.0                 # 0.5 – 2.0
    minimax_sample_rate: 32000         # 8000/16000/22050/24000/32000/44100
    minimax_format: "pcm"             # "pcm" | "wav" | "mp3" | "ogg"
    minimax_timeout: 30.0              # httpx timeout, seconds
    minimax_max_retries: 2             # 0..3 (retry на стороне tts_node)
    minimax_retry_backoff_ms: 500      # начальный backoff, ms (exp x2)
    minimax_streaming: false           # SSE-streaming через stream()
```

**Запуск через launch с переопределением провайдера:**

```bash
# Включить MiniMax, остальные параметры — из voice_assistant.yaml
ros2 launch rob_box_voice voice_assistant.launch.py provider:=minimax
```

Полный валидный пример — [`examples/minimax_tts.yaml`](examples/minimax_tts.yaml).

> **Важно:** yaml содержит ТОЛЬКО non-secret параметры. Секреты читаются
> из ENV (`MINIMAX_API_KEY`, `MINIMAX_GROUP_ID`) — это
> намеренное решение архитектурного комитета (ADR-0003 §3).

---

## 5. Поддерживаемые голоса и языки

### Языки

Поле `minimax_language` (или `TTSSettings.language`) принимает как
короткие BCP-47 коды, так и полные английские имена, которые MiniMax
документирует для `voice_setting.language`:

| Short code | Полное имя (`language_boost`) | Примечание |
|------------|-------------------------------|------------|
| `ru` | `Russian` | Маппинг в `Russian` |
| `en` | `English` | Маппинг в `English` |
| `zh` | `Chinese` | Mandarin |
| `ja` | `Japanese` | |
| `ko` | `Korean` | |
| `es` | `Spanish` | |
| `fr` | `French` | |
| `de` | `German` | |
| `pt` | `Portuguese` | |
| `it` | `Italian` | |
| `ar` | `Arabic` | |
| `hi` | `Hindi` | |

Полный список поддерживаемых языков и лучшие практики выбора — в
[официальной документации MiniMax T2A v2 → «Voice Setting / language_boost»](https://platform.minimaxi.com/docs/api-reference/voice-cloning/voice-cloning-1).

> **Примечание:** Любое другое значение (включая `ru-RU`, `en-US`) будет
> передано в API «как есть». Если MiniMax его не распознает — получите
> `TTSBadRequestError`. См. `_LANGUAGE_ALIASES` в
> [`src/rob_box_llm/rob_box_llm/providers/minimax_tts.py`](../../src/rob_box_llm/rob_box_llm/providers/minimax_tts.py).

### Голоса

Каталог голосов MiniMax обширен и периодически расширяется; **полный
актуальный список — в документации MiniMax**:

* [MiniMax T2A v2 — список голосов (`voice_id`)](https://platform.minimaxi.com/docs/api-reference/voice-cloning/voice-cloning-1)
* [Demo-песочница (можно послушать голоса перед выбором)](https://platform.minimaxi.com/docs)

Часто используемые русскоязычные голоса (значения по умолчанию
зафиксированы в коде):

| `voice_id` | Описание | Когда использовать |
|------------|----------|---------------------|
| `male-qn-qingse` | Мужской, спокойный (default `MiniMaxTTSProvider.DEFAULT_VOICE`) | ROBBOX-голос по умолчанию |
| `female-shaonv` | Женский, молодёжный | Альтернатива для эмоциональных ответов |
| `Calm_Woman` | Женский, ровный | Спокойный, дикторский |
| `male-qn-jingying` | Мужской, энергичный | Активные реплики |

> **Динамический список голосов:** эндпоинт `/v1/voices` на дату
> 2026-07-20 в публичной документации T2A v2 отсутствует, поэтому
> конфигурация голосов — статическая, через ROS-параметр
> `minimax_voice`. См. ADR-0003 §2.3 «Альтернативы, которые НЕ взяли».

---

## 6. Примеры вызова

### 6.1 Из Python-кода

Базовый пример — синхронный (не-streaming) вызов:

```python
import asyncio
import os

from rob_box_llm import MiniMaxTTSProvider, TTSSettings, TTSFormat


async def main() -> None:
    provider = MiniMaxTTSProvider(
        api_key=os.environ["MINIMAX_API_KEY"],
        group_id=os.environ["MINIMAX_GROUP_ID"],
        default_voice="male-qn-qingse",
        default_model="speech-02-hd",
    )
    try:
        audio = await provider.synthesize(
            "Привет, я MiniMax TTS!",
            settings=TTSSettings(
                voice="male-qn-qingse",
                language="ru",
                speed=1.0,
                sample_rate=32000,
                format=TTSFormat.PCM,
            ),
        )
        # audio.samples — int16 little-endian bytes, audio.sample_rate — Hz
        print(f"OK: {len(audio.samples)} bytes @ {audio.sample_rate} Hz "
              f"({audio.duration_s:.2f}s, format={audio.format.value})")
    finally:
        await provider.aclose()  # идемпотентно; безопасно из finally


asyncio.run(main())
```

Streaming-вызов (буферизованный SSE, выдаёт 0 или 1 чанк + терминальный):

```python
import asyncio
from rob_box_llm import MiniMaxTTSProvider, TTSSettings, TTSFormat


async def stream_example() -> None:
    provider = MiniMaxTTSProvider()  # ключи берутся из ENV
    try:
        async for chunk in provider.stream(
            "Длинный текст для потокового синтеза...",
            settings=TTSSettings(
                model="speech-02-turbo",   # быстрее для стриминга
                language="ru",
                format=TTSFormat.MP3,
            ),
        ):
            if chunk.finish_reason == "stop":
                print("stream finished")
            elif chunk.finish_reason == "error":
                print("stream error mid-flight")
            else:
                # chunk.samples — bytes (int16 LE для PCM)
                print(f"chunk: {len(chunk.samples)} samples")
    finally:
        await provider.aclose()
```

Обработка ошибок:

```python
from rob_box_llm import MiniMaxTTSProvider, TTSSettings
from rob_box_llm.errors import (
    TTSAuthError,
    TTSRateLimitError,
    TTSTimeoutError,
    TTSBadRequestError,
    TTSError,
)

try:
    audio = await provider.synthesize("...", settings=TTSSettings())
except TTSAuthError:
    # 401/403 или base_resp со словами auth/key/token — НЕ retry
    ...
except TTSRateLimitError:
    # HTTP 429 или quota/rate/limit — retry с backoff (≤ 2 попыток)
    ...
except TTSTimeoutError:
    # ConnectError/TimeoutException — retry допустим
    ...
except TTSBadRequestError as e:
    # 4xx, битый payload или volume вне [0,10] — НЕ retry
    ...
except TTSError:
    # 5xx, JSONDecodeError, "no data" — retry зависит от контекста
    ...
```

### 6.2 Из ROS2-ноды

В большинстве случаев ROS2-код **не должен** напрямую вызывать
`MiniMaxTTSProvider` — синтез делегируется в `tts_node` через топики.
Ниже — два примера: (a) прямой вызов из своего ROS2-узла (если нужна
тонкая настройка вне `tts_node`), и (b) типовое использование
`tts_node` через ROS-топики.

#### a) Прямой вызов из своей ROS2-ноды

```python
import asyncio
import rclpy
from rclpy.node import Node

from rob_box_llm import MiniMaxTTSProvider, TTSSettings
from rob_box_llm.errors import TTSError


class MiniMaxSynthNode(Node):
    """Пример ROS2-ноды, которая синтезирует речь через MiniMax напрямую.

    Используйте только если вам нужен синтез ВНЕ основного tts_node
    (например, для отдельного voice-канала). Для голосового ассистента
    предпочтительно ходить в tts_node через /voice/tts/say.
    """

    def __init__(self) -> None:
        super().__init__("minimax_synth_node")

        self.declare_parameter("minimax_api_key", "")
        self.declare_parameter("minimax_group_id", "")
        self.declare_parameter("minimax_voice", "male-qn-qingse")

        api_key = (
            self.get_parameter("minimax_api_key").value
            or self._env("MINIMAX_API_KEY")
        )
        group_id = (
            self.get_parameter("minimax_group_id").value
            or self._env("MINIMAX_GROUP_ID")
        )
        if not api_key or not group_id:
            self.get_logger().warn("MINIMAX_API_KEY/MINIMAX_GROUP_ID не заданы — синтез невозможен")

        self._provider = MiniMaxTTSProvider(
            api_key=api_key,
            group_id=group_id,
            default_voice=self.get_parameter("minimax_voice").value,
        )

    @staticmethod
    def _env(name: str) -> str:
        import os
        return os.getenv(name, "")

    async def synthesize(self, text: str) -> bytes:
        try:
            audio = await self._provider.synthesize(
                text,
                settings=TTSSettings(voice="male-qn-qingse", language="ru"),
            )
        except TTSError as exc:
            self.get_logger().error(f"MiniMax TTS failed: {exc}")
            raise
        return audio.samples

    def destroy_node(self) -> None:
        asyncio.run(self._provider.aclose())
        super().destroy_node()
```

#### b) Через стандартный `tts_node` (рекомендуемый путь)

В подавляющем большинстве случаев ROS2-код просто публикует текст
в топик `/voice/tts/say` (или `/voice/dialogue/response`), а
`tts_node` сам заботится о провайдере:

```python
# В вашей ROS2-ноде (например, dialogue_node):
from std_msgs.msg import String

publisher = node.create_publisher(String, "/voice/tts/say", 10)
publisher.publish(String(data="Привет, мир!"))
# tts_node с provider="minimax" сам вызовет MiniMaxTTSProvider,
# транскодирует PCM → 16 kHz/stereo для ReSpeaker и опубликует
# в /voice/audio/speech.
```

Так заданный `minimax_streaming: true` (ROS-параметр) активирует
стриминговый путь `stream()` вместо `synthesize()`. Параметры
`minimax_max_retries` / `minimax_retry_backoff_ms` управляют retry
**на стороне `tts_node`**, а не внутри провайдера (см. ADR-0003 §2.6).

---

## 7. Troubleshooting

Типовые ошибки, которые вы можете увидеть в логах `tts_node` или в
своём коде, и что с ними делать.

### 7.1 `TTSAuthError`: 401 / 403 / "invalid api key"

**Симптомы:**

```
TTSAuthError: 401: {"base_resp":{"status_code":1004,"status_msg":"invalid api key"}}
```

**Причины и решения:**

* Не задан `MINIMAX_API_KEY` или `MINIMAX_GROUP_ID` — `tts_node` падает
  с понятным сообщением «MINIMAX_API_KEY is not configured». Проверьте:
  ```bash
  # В shell, где запускается tts_node:
  env | grep MINIMAX
  # Должны быть MINIMAX_API_KEY и MINIMAX_GROUP_ID.
  ```
* Ключ отозван или удалён в личном кабинете MiniMax — перевыпустите ключ
  и обновите `.env.secrets`.
* Ключ и `GroupId` принадлежат разным аккаунтам — на дату 2026-07-20
  MiniMax требует, чтобы `GroupId` соответствовал аккаунту-владельцу
  ключа. Проверьте пару в UI.

### 7.2 `TTSRateLimitError`: 429 / "rate limit exceeded"

**Симптомы:**

```
TTSRateLimitError: 429: {"base_resp":{"status_code":...,"status_msg":"rate limit exceeded"}}
```

**Решения:**

* `tts_node` уже делает **1 retry с exponential backoff** на эту ошибку
  (настраивается через `minimax_max_retries`, `minimax_retry_backoff_ms`).
  Если ошибка повторяется — уменьшите частоту синтеза или поднимите
  тарифный план в личном кабинете MiniMax.
* **Биллинг:** ретрай списывает токены за оба запроса. Это намеренно и
  приемлемо для нашего масштаба, но для высокочастотных сценариев
  имеет смысл кэшировать результат на стороне `tts_node` (есть
  `cache_dir` / `cache_enabled` в `voice_assistant.yaml`).

### 7.3 `TTSBadRequestError`: 4xx / "voice not found" / volume вне диапазона

**Симптомы:**

```
TTSBadRequestError: 400: {"base_resp":...,"status_msg":"voice not found: ru_male_001"}
# или
TTSBadRequestError: volume=12.5 out of range [0.0, 10.0] (MiniMax T2A v2 spec)
```

**Решения:**

* Проверьте `minimax_voice` по [актуальному каталогу MiniMax](https://platform.minimaxi.com/docs/api-reference/voice-cloning/voice-cloning-1).
  Параметр чувствителен к регистру и подчёркиваниям.
* `volume` должен быть в `[0.0, 10.0]`; `speed` в `[0.5, 2.0]`.
  Выход за диапазон → провайдер сам бросает `TTSBadRequestError`
  (fail-fast, без сетевого запроса).
* `settings.extra` содержит **reserved keys** (`model`, `text`,
  `stream`, `voice_setting`, `audio_setting`, `text_normalization`) —
  провайдер выбрасывает `TTSBadRequestError` со списком конфликтующих
  ключей.

### 7.4 `TTSTimeoutError`: ConnectError / read timeout

**Симптомы:**

```
TTSTimeoutError: All connection attempts failed
# или
TTSTimeoutError: ReadTimeout
```

**Решения:**

* Проверьте сетевую связность до `api.minimax.io:443`:
  ```bash
  curl -I https://api.minimax.io
  ```
* Увеличьте `minimax_timeout` (по умолчанию 30.0 секунд) — для длинных
  текстов этого может не хватать.
* `tts_node` ретраит timeout **до 2 раз** (см. ADR-0003 §2.6). Если
  ошибка сохраняется, проверьте NAT/firewall и DNS-резолв на роботе.

### 7.5 `TTSError`: 5xx / non-JSON / "minimax returned no audio chunks"

**Симптомы:**

```
TTSError: 502: Bad Gateway
# или
TTSError: Non-JSON response: <html>...
# или
TTSError: minimax returned non-hex audio payload: ...
```

**Решения:**

* 5xx и non-JSON — обычно временные сбои на стороне MiniMax.
  `tts_node` ретраит до 2 раз. Если сохраняется > 5 минут — это
  инцидент на стороне провайдера, проверьте <https://status.minimaxi.com>
  (если доступен) или напишите в поддержку MiniMax.
* "non-hex audio payload" — MiniMax в исключительной ситуации вернул
  base64 вместо hex. На 2026-07-20 публичная документация обещает
  hex; если воспроизводится — откройте issue и приложите `audio.raw`
  (полный JSON ответа) и `request_id`.

### 7.6 Секреты видны в логах / process list

**Симптомы:** В ROS-логе или `journalctl` появляется `Authorization:
Bearer eyJhbG...`.

**Решения:**

* httpx.INFO access-log прикладывает URL целиком (включая `GroupId` query
  param). В коде провайдера установлен `_RedactGroupIdFilter`
  на `logging.getLogger("httpx")` — но если вы **сами** включаете
  глобальный `httpx` логгер на DEBUG, фильтр всё равно сработает (см.
  комментарий в `_HTTPX_GROUP_ID_FILTER`).
* **Никогда** не запускайте `tts_node` через
  `env MINIMAX_API_KEY=... ros2 launch ...` в screen/logged shell —
  ENV может попасть в `/proc/<pid>/environ` для пользователей с
  тем же UID. Используйте `docker-compose` + `.env.secrets`.

### 7.7 Голос "не такой, как в каталоге"

**Симптомы:** Голос заявлен как «энергичный мужской», а звучит
«старый и усталый».

**Решения:**

* MiniMax обновляет движок и каталог голосов регулярно; конкретный
  voice_id может менять характер. Используйте [demo-песочницу](https://platform.minimaxi.com/docs)
  для предварительного прослушивания.
* Проверьте, что `minimax_model` соответствует ожиданию. `speech-02-turbo`
  звучит заметно иначе, чем `speech-02-hd`. Переключение модели —
  самый быстрый способ проверить гипотезу.
* Эмоциональная окраска (`emotion: "happy" | "sad" | "neutral"`)
  может перевешивать голос. Попробуйте `emotion: "neutral"`.

---

## 8. Связанные документы

* [`docs/architecture/minimax-tts-architecture.md`](../architecture/minimax-tts-architecture.md) — реализационный контракт, маппинг параметров, sequence-диаграмма, retry-политика.
* [`docs/adr/0003-minimax-tts-architecture.md`](../adr/0003-minimax-tts-architecture.md) — ADR с обоснованием решений.
* [`docs/adr/0002-minimax-provider.md`](../adr/0002-minimax-provider.md) — родительский ADR по провайдеру MiniMax (LLM + TTS).
* [`docs/architecture/minimax-provider.md`](../architecture/minimax-provider.md) — обзорный архитектурный документ.
* [`docs/diagrams/minimax-tts-sequence.mmd`](../diagrams/minimax-tts-sequence.mmd) — sequence-диаграмма вызова.
* [`src/rob_box_llm/rob_box_llm/providers/minimax_tts.py`](../../src/rob_box_llm/rob_box_llm/providers/minimax_tts.py) — реализация провайдера.
* [`src/rob_box_llm/rob_box_llm/tts.py`](../../src/rob_box_llm/rob_box_llm/tts.py) — ABC и value-objects.
* [`src/rob_box_llm/rob_box_llm/errors.py`](../../src/rob_box_llm/rob_box_llm/errors.py) — иерархия `TTSError`.
* [`src/rob_box_voice/rob_box_voice/tts_node.py`](../../src/rob_box_voice/rob_box_voice/tts_node.py) — ROS2 wire-in (`tts_node.py:34-47, 158-202, 293-305, 608-622, 924-977`).
* [`src/rob_box_voice/config/voice_assistant.yaml`](../../src/rob_box_voice/config/voice_assistant.yaml) — основной конфиг voice-ассистента (секция `tts_node`).
* [`src/rob_box_voice/launch/voice_assistant.launch.py`](../../src/rob_box_voice/launch/voice_assistant.launch.py) — launch с `provider:=minimax`.
* [`docker/vision/.env.secrets.template`](../../docker/vision/.env.secrets.template) — шаблон секретов.

### Внешние ссылки

* [MiniMax T2A v2 — документация по API](https://platform.minimaxi.com/docs/api-reference/voice-cloning/voice-cloning-1)
* [MiniMax — личный кабинет и ключи](https://platform.minimaxi.com/user-center/basic-information/interface-key)

---

**Версия документа:** 1.0 (2026-07-21)
**Покрывает:** `rob_box_llm>=0.2.1`, `MiniMaxTTSProvider` (HTTP T2A v2)
