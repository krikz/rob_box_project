# ADR-0067: Обратный канал звука ТАРС в шлем (operator-agent шаг 5б, issue #1993)

- Статус: **Принят** (2026-09-07, architect, карточка t_dce67f3f)
- Контекст: target-operator-agent-and-dialogue.md §7.4, §9.1, §9.5, §10.1; ADR-0051 §2.9
- Шаг в разбивке: **5б** («Обратный канал звука в шлем»), блок-зависимость: 04a (avatar_supervisor публикует `/avatar/tts/request`).

## Решение

Голос ТАРС отвечает оператору через **отдельный ROS-канал** `/avatar/tts/audio` (AudioData, int16 LE PCM, тот же SR/формат, что `/voice/audio/speech`) и существующий **обобщённый `deliver_audio(stream, ...)`** в `ws_server`. Preview голосов и операторская речь — два пользователя одного приватного аудиоканала сессии (стримы `"preview"` и `"operator_tts"`).

```text
avatar_supervisor ─► /avatar/tts/request ─► tts_node ─► /avatar/tts/audio ─► quest_node ─► ws_server ─► наушники шлема
                     (sink:"headset")             (AudioData, PCM)            ws_server.deliver_audio(stream="operator_tts")
```

Инструмент `say` остаётся на `/voice/tts/request → /voice/audio/speech → динамики робота`. Два канала не пересекаются (инвариант §7.4).

## Контракты

### ROS

| топик | тип | pub | sub | смысл |
|---|---|---|---|---|
| `/avatar/tts/request` (новый) | `String` JSON | `avatar_supervisor` | `tts_node` | `{request_id, ssml, sink:"headset", voice?, language?}` — собственный ответ ТАРС |
| `/avatar/tts/audio` (новый) | `AudioData` int16 LE PCM | `tts_node` | `quest_node` | синтезированные чанки ТАРС (только `sink=="headset"`-запросы) |

Контракт `/voice/tts/request` **не** трогаем — он остаётся за `say` и dialogue_node. Шаг 5б этого не меняет (приоритет — 07a).

### ws_server: `deliver_audio(stream, request_id, bytes, format, seq, total, session=None)`

```python
def deliver_audio(self, *, stream: str, request_id: str, ws: WebSocket,
                  audio_bytes: bytes, audio_format: str, content_type: str,
                  seq: int, total: int) -> bool
```

Внутренняя таблица `_audio_pending: dict[str, dict[request_id, (ws, ts)]]` — по одному словарю на stream. `stream in {"preview", "operator_tts"}` — статичный whitelist (другие стримы молча отбрасываются, чтобы поток не стал «широким швом»). Старый `deliver_preview_audio(...)` остаётся тонкой обёрткой над `deliver_audio(stream="preview", ...)`. `deliver_preview_done/_error` тоже — без изменений сигнатуры (preview-канал уже работает).

### tts_node

- Подписка на `/avatar/tts/request` через `_on_avatar_tts_request(msg)`. Контракт сообщения **тот же**, что у `_on_tts_request` для `/voice/tts/request` (поле `ssml` обязательно), плюс `sink == "headset"` — единственный валидный stream на этом канале. Любой другой sink → `_avatar_tts_error_pub` с `reason="invalid_sink"` и DROP.
- Синтез идёт через существующую `_run_synthesis_worker` (BLK-9 slot pool), но **без** ALSA-воспроизведения и **без** паблиша в `/voice/audio/speech`: после успешного `_prepare_audio_for_topic` шлём `AudioData` в `/avatar/tts/audio` и публикуем `/voice/tts/finished` (для корреляции и метрик — те же `speech_id`/`dialogue_id`/`batch_*`). Ошибки → `/avatar/tts/error` (String JSON `{request_id, error}`) и тот же `/voice/tts/finished{success=False}`.
- `_avatar_tts_request_id` хранит **текущий** активный request_id. `/avatar/tts/control {cmd:"STOP"}` (тот же control_callback) сбрасывает его — синтезирующийся воркер увидит устаревший request_id и прервётся перед паблишем аудио.
- Параметр ноды `headset_audio_topic` (default `/avatar/tts/audio`) — для тестов и чтобы шов с `audio_topic`/`audio_output_sample_rate` остался единственным параметризуемым.

### quest_node

- Подписка `String` на `/avatar/tts/audio` не нужна — это `AudioData`. Создаём `create_subscription(AudioData, "/avatar/tts/audio", self._on_avatar_tts_audio, audio_qos)`. На стороне ws_server уже есть `deliver_preview_audio`, который ходит через `_schedule_ws_send_binary` по `BINARY_FRAME`. Чтобы не дублировать логику сериализации, делаем `quest_node._on_avatar_tts_audio(msg)` → `ws_server.deliver_audio(stream="operator_tts", request_id=current_avatar_tts_request_id, audio_bytes=msg.data, audio_format="pcm_s16le", content_type="audio/pcm", seq/total)`.
- Проблема сессионной привязки: `/avatar/tts/audio` приходит на ROS-уровне без `session_id`. Решение — **оператор один, его активная Quest-сессия — единственная на ноде**, и она же была источником `/avatar/tts/request` (avatar_supervisor → quest_node → ws_server). Берем `ws_server.get_active_sessions() == 1` → этот единственный ws, иначе выбираем самый свежий `session_id` из `ws_server._sessions`. Тот же подход уже работает в `deliver_preview_audio`, где request_id → ws (но там request_id рождается внутри ws-server, а тут — нет). Чтобы избежать race «avatar_supervisor шлёт request, а у оператора нет активной сессии», в `deliver_audio` ws-server сам регистрирует активный ws для stream `operator_tts` через `register_audio_session(stream, request_id, ws)` ИЛИ через side-channel «последний стартовавший request_id»: при старте `/avatar/tts/request` quest_node получает копию через подписку `/avatar/tts/request` (String JSON; tts_node И quest_node подписаны на одно сообщение) и сам регистрирует request_id → ws в ws_server. Это **даёт детерминированную сессионную привязку без race**.

### webxr_client (barge-in)

Новый `case "operator_tts_audio"` в `handleSupervisorEvent` (или `onServerEvent`) — отдельный AudioSink `operatorAudioSink`, симметричный `previewSink` (тот же `onMeta/onChunk/play`, но без `dispatchTts` и без preview-ui: это речь в шлем, не превью). На клиенте уже есть `voice_ptt_start/stop` от грипа (через ModeManager). Локальный barge-in: при `voice_ptt_start` зовём `operatorAudioSink.stop()` И шлём `voice_ptt_start` на сервер, чтобы сервер тоже остановил `/avatar/tts/*`. Динамиков робота это не касается (они живут на `/voice/tts/*`, а не в этом sink).

Новый тип в `wire/messages.ts`:
```ts
| { type: "operator_tts_audio"; request_id: string; format: "pcm_s16le"|"mp3"|"opus"|"wav"; content_type: string; seq: number; total: number; ts_ms: number }
| { type: "operator_tts_done"; request_id: string; ts_ms: number }
| { type: "operator_tts_error"; request_id: string; reason: string; ts_ms: number }
```

`operator_tts_done/_error` пока не шлются сервером (tts_node просто перестаёт слать аудио и публикует `/voice/tts/finished`), но типы заведены для forward-compat.

## Архитектурные trade-offs

| альтернатива | почему отклонена |
|---|---|
| Переиспользовать `/voice/audio/speech` с фильтром по stream | Смешивает два канала на одном топике — именно то, против чего §7.4; downstream-подписчики (recordings/head-tracking) не должны решать по payload, кому аудио |
| Шить аудио в WS JSON_EVENT base64 (как сейчас делает `preview_audio`) | Уже работает для preview, но base64+JSON × 16 kHz × 16 bit × 1 ch = ~50% оверхед; для речи оператора это лишнее; BINARY_FRAME-канал уже есть, используем |
| Передавать session_id в заголовке ROS AudioData | Нет поля; добавлять — это новый IDL surface ради одного edge-case'а. Текущий side-channel через подписку `/avatar/tts/request` дешевле |
| WebRTC DataChannel на шлем вместо WS audio | Меняет транспортный шов целиком — за рамки шага 5б. Текущий BINARY_FRAME работает в preview, переиспользуем |
| Делать отдельный ROS-сервис `/avatar/tts/synthesize` | Сервис = синхронный блок, реплики ТАРС должны ложиться в очередь и проигрываться подряд. Топик — естественный pub/sub, и в дальнейшем (07a приоритет) легко встаёт в общую очередь |

## Что НЕ делаем

- Не трогаем `/voice/tts/request` и приоритет — это шаг 07a (PR #2041 уже открыт, не пересекаемся).
- Не вводим новый IDL / msg-файл — `AudioData` уже есть, контракт `String` JSON на request остаётся каноническим из `/voice/tts/request`.
- Не выкидываем preview-канал — он остаётся стримом `"preview"` общего `deliver_audio`.
- Не делаем WebRTC / MediaStream — это за рамки шага 5б.

## Файлы

| файл | что меняется |
|---|---|
| `src/rob_box_quest/rob_box_quest/server/ws_server.py` | `deliver_audio(stream, ws, request_id, bytes, format, seq, total)`; `_audio_pending: dict[str, dict[…]]`; whitelist streams. Старый `deliver_preview_audio` → тонкая обёртка |
| `src/rob_box_quest/rob_box_quest/quest_node.py` | подписка `AudioData /avatar/tts/audio` → `ws_server.deliver_audio(stream="operator_tts", ws=…)`; side-channel: подписка `String /avatar/tts/request` → `register_audio_session("operator_tts", request_id, ws)` |
| `src/rob_box_voice/rob_box_voice/tts_node.py` | подписка `String /avatar/tts/request` (handler `_on_avatar_tts_request`); `_publish_headset_audio` параллельно `_publish_audio` при `sink="headset"`; `headset_audio_topic` параметр; `control_callback` общий для `/voice/tts/control` и `/avatar/tts/control` (тот же формат, без нового кода) |
| `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py` | новый `_avatar_tts_request_pub` (топик `/avatar/tts/request`); `_publish_avatar_tts(text, language?)` используется из всех точек, где сейчас supervisor отвечает «принял», «не умею», «камера повёрнута» |
| `src/rob_box_quest/webxr_client/src/wire/messages.ts` | новые типы `operator_tts_audio/_done/_error` |
| `src/rob_box_quest/webxr_client/src/ui/operator_audio_sink.ts` | новый файл, симметричный `preview_audio_sink.ts` |
| `src/rob_box_quest/webxr_client/src/main.ts` | `case "operator_tts_audio"` + подключение `operatorAudioSink`; `voice_ptt_start` → `operatorAudioSink.stop()` |

## Definition of Done (проверяемые факты, ADR-0018)

- [ ] `deliver_audio(stream="operator_tts", ...)` существует и проходит unit-тест: ros-msg AudioData → registered ws → JSON_EVENT+тихий binary frame (без base64 в JSON).
- [ ] Preview-канал не сломан: regression-тест на `deliver_preview_audio` через обёртку.
- [ ] Barge-in: unit-тест на `voice_ptt_start` → `operatorAudioSink.stop()` (клиент) + `control_callback("STOP")` → `_avatar_tts_request_id` сбрасывается (сервер).
- [ ] `avatar_supervisor` публикует `/avatar/tts/request` для каждой реплики «принял / не умею / камера повёрнута» (заменяет прямые ответы в `/voice/tts/request` для собственных реплик).
- [ ] Регресс e2e-теста инварианта 6b: `say` → динамики робота, реплика ТАРС → шлем, без пересечения. (на железе — это уже работа PR #2009 + 05б; вне рамок архитектурного PR — добавляется отдельной карточкой 09/10)