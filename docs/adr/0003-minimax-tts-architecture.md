# ADR-0003: MiniMax TTS-провайдер — архитектура интеграции в tts_node

| Поле | Значение |
| --- | --- |
| Статус | Proposed |
| Дата | 2026-07-20 |
| Автор | architect (Hermes Agent) |
| Контекст | Kanban task `t_034b1260`, PR #907 |
| Заменяет | — |
| Заменяется | — |
| Родитель | [ADR-0002 — MiniMax provider](0002-minimax-provider.md) |
| Реализуется | `t_98a417b9` (HTTP-провайдер), `t_257dbfb9` (ROS-интеграция) |
| Связанный артефакт | [`../architecture/minimax-tts-architecture.md`](../architecture/minimax-tts-architecture.md) |
| Sequence-диаграмма | [`../diagrams/minimax-tts-sequence.mmd`](../diagrams/minimax-tts-sequence.mmd) |

---

## 1. Контекст

ADR-0002 зафиксировал верхнеуровневое решение: MiniMax подключается
как **опциональный третий TTS-провайдер** в `tts_node` через
`rob_box_llm.TTSProvider` ABC, не ломая существующий Yandex/Silero
пайплайн и не меняя ROS-топики. Имплементация (`t_98a417b9`) и
wire-in (`t_257dbfb9`) сейчас в работе.

Для подзадач-реализаторов остаются открытыми вопросы уровня
**контракта на стыке** — то, что ADR-0002 сознательно не
детализирует, чтобы не размывать scope:

1. **Маппинг параметров** — какой конкретно ROS-параметр идёт в какое
   поле MiniMax T2A v2 body, и как разрешаются конфликты (ROS vs ENV
   vs SSML).
2. **Конфигурация** — где живут секреты, какой приоритет у ENV и ROS,
   что попадает в launch-yaml.
3. **Контракт выходных данных** — что MiniMax отдаёт, что
   `TTSProvider.synthesize()` возвращает, и что публикуется в
   `/voice/audio/speech` (`audio_common_msgs/AudioData`).
4. **Стратегия обработки ошибок и retry** — какие ошибки ретраятся,
   какие нет, где именно реализуется retry-loop, как ведёт себя
   tts_node при исчерпании retry.
5. **Документация для следующих фаз** — что отложено, какие решения
   требуют отдельного ADR.

Этот ADR закрывает вопросы 1–4 и явно фиксирует, что **отложено**
(вопрос 5).

---

## 2. Решения

### 2.1 Маппинг параметров (TTSSettings ↔ MiniMax T2A v2)

**Решение:** ввести `TTSSettings` (уже есть в `rob_box_llm/tts.py`)
как единый провайдер-агностичный value-object, который
`MiniMaxTTSProvider._build_payload` маппит в JSON T2A v2.

Полная таблица маппинга — в
[`../architecture/minimax-tts-architecture.md#21-ttssettings--t2a-v2-body`](../architecture/minimax-tts-architecture.md).
Ключевые правила:

- Провайдер **никогда не дропает неустановленные поля** из body —
  иначе MiniMax применит свои дефолты и результат может
  отличаться от ожиданий (например, `"hd"` vs `"turbo"` модель).
- Язык: BCP-47 короткие коды (`"ru"`, `"en"`) маппятся в
  human-readable (`"Russian"`, `"English"`) через
  `_LANGUAGE_ALIASES`. Неизвестный код пробрасывается as-is
  (API отвергнет с явной ошибкой).
- `format=OGG` → fallback на `mp3` (MiniMax OGG не принимает),
  с логированием.
- `settings.extra` — whitelist 9 ключей; reserved keys
  (`model`/`text`/`stream`/`voice_setting`/`audio_setting`/
  `text_normalization`) → `TTSBadRequestError` (защита от
  перетирания наших typed-полей через extra — security boundary).

### 2.2 Конфигурация: секреты, ENV, ROS

**Решение:** двухслойная конфигурация.

| Слой | Что | Где |
| --- | --- | --- |
| Запуск | `provider`, `voice`, `model`, `lang`, `speed`, `sample_rate`, `timeout` | launch-yaml → ROS-параметры tts_node |
| Секреты | `api_key`, `group_id` | docker-compose secrets → ENV `MINIMAX_API_KEY` / `MINIMAX_GROUP_ID`, fallback из ROS-параметра |

**Приоритет:** ROS-параметр > ENV (только для секретов). Для
остальных полей ENV-fallback не реализуется — намеренно: ROS
override-механизм + SSML уже покрывают runtime-переопределение
(`<prosody rate>` бьёт `minimax_speed`).

**Безопасность:** секреты никогда не попадают в launch-yaml и в
логи. `_log.warning("…")` в `minimax_tts.py` маскирует содержимое
`extra` (только ключи).

### 2.3 Контракт выходных данных: что попадает в `/voice/audio/speech`

**Решение:** MiniMax-путь публикует в `/voice/audio/speech` тот же
формат, что и Yandex-путь — `int16 little-endian PCM` через
`audio_common_msgs/AudioData.data`.

Цепочка преобразований:

```
MiniMax hex PCM (int16 LE, mono, 32 kHz)
  → MiniMaxTTSProvider: bytes.fromhex → TTSAudio(samples: bytes, sample_rate: 32000)
  → tts_node: np.frombuffer(int16).astype(float32)/32768 → audio_np float32 [-1,1]
  → resample_audio(audio_np, 32000, 16000) → float32 mono 16 kHz
  → tts_node._publish_audio: (audio_np*32767).astype(int16).tobytes() → AudioData
  → publish("/voice/audio/speech")
```

**Контракт топика НЕ меняется** (invariant из ADR-0001):
`/voice/audio/speech` остаётся `AudioData` msg, и payload остаётся
PCM int16. Существующие подписчики (sound_node, mcp_audio_tools)
не должны меняться.

**ReSpeaker-специфика** (16 kHz stereo) обрабатывается в общем
пути `tts_node._synthesize_and_play`, не в провайдере. Подзадача
`t_257dbfb9` обязана проверить на стенде, что mono→stereo
конверсия для MiniMax-ветки действительно происходит (исторически
Yandex отдавал стерео).

### 2.4 Sync vs streaming: выбран sync `synthesize()`

**Решение:** основной путь — sync `TTSProvider.synthesize()`.
Опциональный путь `minimax_streaming=true` использует
`TTSProvider.stream()` и публикует каждый SSE-аудиочанк в ROS-топик по
мере поступления; итоговое воспроизведение остаётся последовательным
после завершения синтеза. True fixed-size frame streaming через
WebSocket требует отдельного lifecycle и остаётся отложенным.

Обоснование:

- ROS-нода синхронная (`_synthesize_and_play` — блокирующий callback).
  Async-стриминг потребовал бы переделки на `asyncio.run_coroutine_threadsafe`
  с переходом на async subscriber — высокая стоимость, низкая выгода
  для нашего use-case (реплики ≤ 30 сек).
- True chunk-per-frame стриминг MiniMax даёт через **WebSocket**
  endpoint, не через HTTP SSE. WebSocket требует persistent connection
  и пересмотра lifecycle провайдера — отложено в фазу M5/M6 (см. ниже).
- Один `AudioData` msg на весь синтез — это **уже поведение Yandex**
  в текущем коде, так что контракт топика не нарушается.

`stream()` эмитит аудиочанки SSE по одному и завершается пустым
терминальным `TTSChunk(finish_reason="stop")`; mid-stream ошибки после
первого чанка представлены `finish_reason="error"`, а ошибки до первого
чанка выбрасываются как `TTSError`.

### 2.5 Raw bytes vs base64: выбран raw (hex-decode)

**Решение:** MiniMax T2A v2 возвращает аудио как **hex-encoded string**
в JSON-поле `data.audio`. Провайдер делает `bytes.fromhex(audio_hex)`
и кладёт результат в `TTSAudio.samples` как raw bytes.

Альтернативы, которые **не** взяли:

| Альтернатива | Почему отвергнуто |
| --- | --- |
| Base64 | MiniMax отдаёт hex (см. их документацию); перекодирование бессмысленно |
| Multipart/form-data | T2A v2 принимает только JSON; смена формата = изменение API контракта |
| Streaming bytes (chunked transfer) | HTTP/1.1 chunked уже работает, но MiniMax всё равно отдаёт один большой JSON |

Hex-decoding стоит ~1ms на 30 секунд аудио @ 32 kHz (1.92 MB raw →
960 KB bytes) — не bottleneck.

### 2.6 Retry: на стороне tts_node, не в провайдере

**Решение:** retry-loop реализуется в `tts_node`, вокруг вызова
`MiniMaxTTSProvider.synthesize()`. Провайдер остаётся чистой
raise-on-fail функцией.

Алгоритм:

| Тип ошибки | Retry? | Кол-во | Backoff |
| --- | --- | --- | --- |
| `TTSTimeoutError` | Да | ≤ 2 | 0.5s, 1.5s |
| `TTSRateLimitError` | Да | ≤ 1 | 1s |
| `TTSError` (5xx обёртка) | Да | ≤ 2 | 0.5s, 1.5s |
| `TTSAuthError` | **Нет** | — | — |
| `TTSBadRequestError` | **Нет** | — | — |
| `base_resp.status_code != 0` (бизнес) | **Нет** | — | — |

Обоснование "retry снаружи":

1. Провайдер остаётся stateless, легко тестируется `httpx.MockTransport`.
2. ROS-нода имеет контекст (timeout всего диалога, текущий `dialogue_id`)
   — может принять решение "ретрай не нужен" точнее.
3. Провайдер переиспользуется в cron/CLI, где retry не нужен.

**Идемпотентность:** MiniMax детерминирован для `(text, voice,
settings)`; retry безопасен по семантике (тот же аудио-блоб), но
**списывает токены дважды** при срабатывании. Приемлемо для нашего
масштаба (десятки запросов/день).

### 2.7 Что tts_node делает при ошибке MiniMax

**Решение:** при ошибке MiniMax tts_node:

1. Публикует `/voice/tts/status=error` через `publish_state()`.
2. Логирует через `get_logger().error()` с типом ошибки.
3. **Пробрасывает исключение наверх** (raise).
4. **НЕ** делает автоматический fallback в Silero — пользователь
   явно выбрал MiniMax через `provider=minimax`, молча откатиться
   было бы неуважением к выбору. ADR-0002 §2.1 уже зафиксировал
   это; здесь просто переподтверждаем.

Caller (dialogue_node или MCP tool) решает, что делать: повторить
диалог, переключить `provider` обратно на `yandex` через
`rclpy.set_parameters`, или сообщить пользователю.

---

## 3. Структура артефактов

```
docs/
├── adr/
│   ├── 0001-harness-architecture.md           (existing — dialog/persistent/telegram nodes)
│   ├── 0002-minimax-provider.md               (existing — MiniMax в целом, LLM+TTS+image)
│   └── 0003-minimax-tts-architecture.md       (ЭТОТ — TTS-конкретный контракт)
├── architecture/
│   ├── minimax-provider.md                    (existing — обзор MiniMax)
│   └── minimax-tts-architecture.md            (НОВЫЙ — TTS-детализация, реализационный справочник)
└── diagrams/
    └── minimax-tts-sequence.mmd               (НОВЫЙ — sequence diagram)
```

`minimax-tts-architecture.md` — это **детализация для инженеров**:
полные таблицы маппинга, конкретные значения по умолчанию, цепочки
преобразований. Этот ADR — **запись решения** и **trade-offs**.

---

## 4. Альтернативы, которые отклонены

| Альтернатива | Почему отклонена |
| --- | --- |
| **Retry внутри провайдера** | Делает провайдер stateful, мешает unit-тестам, лишает ROS-ноду контекста для решения "ретрай не нужен" |
| **Streaming через WebSocket сразу** | Persistent connection ломает lifecycle провайдера; для реплик ≤30 сек overkill |
| **Автоматический fallback MiniMax→Yandex→Silero** | Скрывает реальную проблему от пользователя; "Магический провайдер" = debug hell |
| **OGG-конверсия в MiniMax через свой encoder** | MiniMax не принимает OGG; добавлять локальный OGG-encoder ради одного формата — YAGNI |
| **Capabilities registry (`CapabilityUnavailableError`)** | Отклонён ещё в ADR-0002 §2.1; нет конкретного потребителя runtime-check |
| **Динамическое определение списка голосов через API** | `/v1/voices` endpoint не задокументирован публично для T2A v2 на 2026-07-20 |
| **Capability-флаг для image-generation через MiniMax** | Отдельная фаза M5/M6 (ADR-0002 §6) |

---

## 5. Последствия

### Положительные

- **Контракт на стыке** `tts_node ↔ TTSProvider ↔ MiniMax` формализован
  в одном месте — у `backend` и `ros2-engineer` одна правда.
- **Retry-политика** описана явно: где, сколько, для каких ошибок.
  Раньше была неявная (`httpx` сам ничего не ретраит) — теперь это
  документированное поведение.
- **Output contract** на `/voice/audio/speech` явно не меняется:
  существующие подписчики не ломаются. ADR-0001 invariant сохранён.
- **Безопасность:** reserved-keys защита в `_build_payload` теперь
  зафиксирована и в архитектурном документе (раньше жила только
  в комментарии в коде).

### Отрицательные / риски

- **Retry списывает токены дважды** при срабатывании. Масштаб
  сейчас низкий, но если MiniMax станет primary-провайдером для
  всех диалогов — может потребоваться token-aware retry budget.
- **Один терминальный chunk для streaming** — если в будущем
  появится use-case с интерактивным голосом (низкая latency важна),
  придётся переделывать на WebSocket. Это отложено в M5/M6 и
  зафиксировано в ADR-0002.
- **Mono→stereo конверсия** проверена для Yandex, но **не верифицирована**
  на стенде для MiniMax-пути. Это явный action-item для
  `t_257dbfb9`; до стенда этот ADR остаётся Proposed.

### Нейтральные

- Добавлен `docs/diagrams/` (новая директория). Документация
  Mermaid-стиля уже используется в проекте (`docs/MERMAID_DIAGRAMS.md`),
  так что формат не новый.
- Документ `minimax-tts-architecture.md` формально пересекается
  с `minimax-provider.md` (раздел TTS). Целенаправленно: обзорный
  `minimax-provider.md` остаётся точкой входа, а `minimax-tts-architecture.md`
  — справочник реализатора.

---

## 6. Совместимость с ADR-0001 и ADR-0002

- **ADR-0001** (harness architecture, dialog/persistent/telegram nodes):
  этот ADR **не меняет** топики `/voice/dialogue/response`,
  `/voice/tts/request`, `/voice/audio/speech`; payload и контракт
  остаются идентичными. **Совместим.**
- **ADR-0002** (MiniMax provider): этот ADR **расширяет** ADR-0002
  детализацией TTS-части. Решения ADR-0002 (P0.5 extension в
  `rob_box_llm`, opt-in через `provider=minimax`, fallback в Silero
  НЕ делается) — **сохранены и подтверждены**. Не противоречит.

---

## 7. Открытые вопросы / будущие ADR

| Вопрос | Где решать |
| --- | --- |
| WebSocket chunk-per-frame streaming | M5/M6 фаза, отдельный ADR |
| Token-aware retry budget | Если MiniMax станет primary — отдельный ADR |
| Динамический список голосов через `/v1/voices` | Когда MiniMax опубликует endpoint |
| Image generation через MiniMax | M5/M6 фаза (ADR-0002 §6) |
| Stereo-конверсия для MiniMax на уровне провайдера | Если будет use-case отличный от ReSpeaker |
| Capabilities registry (`CapabilityUnavailableError`) | Когда появится второй потребитель |

---

## 8. Acceptance

ADR считается зафиксированным (статус переводится в **Accepted**),
когда:

1. ✅ Документ одобрен архитектурным ревью.
2. ✅ Подзадача `t_98a417b9` (backend: HTTP-клиент + провайдер)
   завершена и реализует зафиксированный маппинг.
3. ✅ Подзадача `t_257dbfb9` (ros2-engineer: ROS-интеграция)
   завершена и верифицирует mono→stereo конверсию на стенде.
4. ✅ End-to-end сценарий "текст → MiniMax → ReSpeaker" работает
   на тестовом стенде с приемлемой latency.

До выполнения пунктов 2-4 статус остаётся **Proposed**.
