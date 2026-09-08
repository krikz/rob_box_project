# ADR-0078: preview-voice synthesis — канал picker'а голосов от supervisor'а до шлема

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-08 |
| Автор | backend worker по issue #2138.A.3 (просьба товарища Шифу: «жму превью на голосе, там крашится ошибка not implemented») |
| Контекст | Операторский picker голосов (Captain Bridge в Quest) шлёт `/avatar/preview_voice` чтобы услышать «как звучит голос X». supervisor_node.py:1623 валидирует запрос и публикует `preview_synthesis_not_implemented_in_mvp` — заглушку, потому что в tts_node не было отдельного pure-synth пути (только `_synthesize_and_play`, который идёт в FIFO/ALSA/metrics и публикует в `/avatar/tts/audio` для headset-канала ТАРС). ws_server и клиентский `preview_audio_sink.ts` уже заведены и ждут байты в mp3/wav контейнере. |
| Затрагивает | (a) `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py` — `_on_preview_voice` (заглушка → делегация) + `_publish_avatar_tts(sink, request_id)` (новые параметры); (b) `src/rob_box_voice/rob_box_voice/tts_node.py` — `_on_avatar_tts_request` switch по `sink` (новая ветка `preview`), `TTSNode.synthesize_preview` (новый pure-synth метод), preview publishers (`/avatar/preview_voice/{audio,result,error}`), иерархия ошибок `PreviewSynthesisError*`; (c) `src/rob_box_voice/test/unit/tts/test_tts_node_preview.py` + `test_tts_node_preview_ros.py` — unit-тесты; (d) `src/rob_box_supervisor/test/unit/test_supervisor_preview_voice.py` — supervisor unit-тесты. **НЕ затрагивает** `src/rob_box_quest/*` (ws_server и preview_audio_sink.ts уже на месте, см. ADR-0073) и `src/rob_box_voice/rob_box_voice/tts_node._publish_headset_audio` (sink="headset" остаётся). |
| Родители | ADR-0055 (`/avatar/tts/request` с `sink="headset"`), ADR-0018 (честный FAIL → preview_error с реальной причиной, не молчание), ADR-0021 (CC-budget — preview helpers добавляют < 50 строк, в пределах baseline). |
| Связанные | issue #2138.A.3, ADR-0073 (TARS voice picker end-to-end — определяет UI/UX), ADR-0077-multi-skill (verification-before-completion обязателен для этой карточки). |

## 1. Контекст и бизнес-проброблема

### 1.1 Что наблюдает Шифу

> «жму превью на голосе, там крашится ошибка not implemented»

Picker голосов в Captain Bridge кнопкой «превью» отправляет JSON в `/avatar/preview_voice` (request_id, voice_id, text, provider). supervisor_node.py:1623 валидирует поля и публикует `preview_voice_error{reason: "preview_synthesis_not_implemented_in_mvp"}` — честный отказ (не молчание, не fake-done), но и не синтез. ws_server мапит этот reason в `preview_voice_error` событие на клиент, UI рендерит «preview пока недоступен, попробуйте позже».

Транспорт supervisor ↔ ws_server ↔ клиент **готов целиком** (см. ADR-0073 §3.4): publishers `/avatar/preview_voice/{audio,result,error}` заведены в supervisor, ws_server форвардит `preview_voice_audio`/`_done`/`_error` через `deliver_audio(stream="preview", ...)`, `preview_audio_sink.ts` ждёт base64-decoded audio + content_type и играет через WebAudio `decodeAudioData`.

**Не хватает одного звена**: никто не синтезирует аудио для preview. Синтезировать умеет только tts_node, и его единственный путь `_synthesize_and_play` непригоден для preview:

- Он идёт в FIFO-очередь воспроизведения (`_play_order_cond`).
- Он публикует `/avatar/tts/audio` (PCM для headset ТАРС) — picker не тот канал.
- Он играет ALSA на динамиках робота — picker только в шлеме.
- Он трогает `minimax_voice` (активный голос личности) — preview должен быть **read-only** по голосам.

### 1.2 Почему ADR-0055 не закрыл проблему

ADR-0055 ввёл `/avatar/tts/request` с `sink="headset"` для шлема ТАРС. Этот канал идёт через `_run_synthesis_worker` (BoundedThreadPoolExecutor), который уважает FIFO, ставит `play_seq`, публикует в `/avatar/tts/audio` через `_publish_headset_audio`. Симметрично использовать его для preview **нельзя** — preview обязан быть коротким синхронным (sub-10s), не прерывать текущую реплику личности, не падать в очередь.

Но контракт канала (`{request_id, ssml, sink, voice?, language?}`) — **общий**, и это шанс: добавить `sink="preview"` как второе валидное значение в существующий switch, без нового топика и без нового контракта. ws_server прозрачно пропускает любой payload с `sink` через (`deliver_audio` уже параметризован `stream="preview"`), а ws_server-side слушатели `/avatar/preview_voice/{audio,result,error}` уже заведены.

## 2. Решение

### 2.1 Контракт топика `/avatar/tts/request` — расширение

Текущий контракт (ADR-0055 §tts_node):

```
{request_id: string, ssml: string, sink: "headset", voice?: string, language?: string}
```

Расширенный контракт (ADR-0078):

```
{request_id: string, ssml: string, sink: "headset" | "preview", voice?: string, language?: string}
```

`sink="preview"` — **допустимое** значение. Любой другой sink → `_avatar_tts_error_pub{error: "invalid_sink"}` + DROP (как раньше).

**Никаких новых полей не вводится.** `provider` в preview-режиме не нужен — supervisor выбирает hint самостоятельно по реестру голосов (см. §2.3).

### 2.2 Контракт `/avatar/preview_voice/audio` (изменение)

Был задекларирован в ADR-0073 §3.4 как «BINARY audio bytes», без указания формата. Реальный контракт (по `preview_audio_sink.ts`):

```
{
  request_id: string,
  format: "mp3" | "wav" | "ogg",
  content_type: "audio/mpeg" | "audio/wav" | "audio/ogg",
  sample_rate: int,
  duration_s: float,
  audio_b64: string,    // base64-encoded bytes в контейнере format
}
```

**base64, не raw bytes, не BINARY_FRAME.** ws_server форвардит это как `preview_voice_audio` JSON_EVENT клиенту; `preview_audio_sink.ts` декодирует `audio_b64` → ArrayBuffer и передаёт в `AudioContext.decodeAudioData()` с явным `content_type`. Это правильный путь: WebAudio `decodeAudioData` декодирует mp3/wav/opus по MIME, но **не raw PCM без контейнера** — отсюда жёсткое требование «контейнер, не PCM» (см. §грабли).

### 2.3 Контракт `/avatar/preview_voice/result`

```
{
  request_id: string,
  format: "mp3" | "wav" | "ogg",
  content_type: string,
  sample_rate: int,
  duration_s: float,
}
```

Done-маркер. ws_server форвардит как `preview_voice_done` JSON_EVENT. **Отдельный топик от audio** — UI может рендерить «прослушал: X секунд» пока аудио ещё играет (не блокируем на нём).

### 2.4 Контракт `/avatar/preview_voice/error`

```
{
  request_id: string,
  reason: "preview_timeout" | "minimax_unavailable" | "preview_synthesis_failed" | "empty_text" | "audio_publish_failed",
  ts_ms: int,
}
```

`reason` — **стабильная строка**, публичный контракт с UI (ADR-0018 capability-honest). Текущие reason'ы:

- `preview_timeout` — сетевой синтез не уложился в `timeout_s` (default 10s).
- `minimax_unavailable` — `MINIMAX_AVAILABLE=False` (нет opt-in к MiniMax, preview требует именно его).
- `preview_synthesis_failed` — провайдер бросил (auth/bad-request/rate-limit/5xx).
- `empty_text` — `ssml` или `text` пустой (mirror headset guard #2096).
- `audio_publish_failed` — publish в `/avatar/preview_voice/audio` упал (не должно случаться на проде, но честно сообщаем).

При любой ошибке preview — **НЕ молчание, НЕ fake-done**. ws_server получит `preview_voice_error`, клиент увидит reason.

### 2.5 supervisor_node.py — `_on_preview_voice`

Старая заглушка (lines 1623-1676):

```python
# Валидация → self._publish_preview_error(
#     request_id, "preview_synthesis_not_implemented_in_mvp"
# )
```

Новая реализация:

```python
# 1. Валидация (без изменений, кроме провайдера).
# 2. Если provider не указан и voice_id известен >1 провайдеру — выбираем
#    minimax (приоритет для preview). Иначе берём того, кто знает голос.
# 3. self._publish_avatar_tts(text, voice=voice_id, sink="preview",
#                              request_id=request_id)
#    — расширенный _publish_avatar_tts добавляет параметры sink и
#      request_id. request_id протаскивается от picker'а до tts_node
#      для корреляции preview_voice_audio/done/error с ws_server.
# 4. Если _publish_avatar_tts вернул пустую строку (text drop) — шлём
#    preview_error "empty_text_dropped" для ws_server, чтобы picker не висел.
```

Валидация по реестру (`_voices_for(provider)`) делается в supervisor **ДО** публикации, чтобы picker получил честный preview_error мгновенно, не дожидаясь сети до tts_node.

### 2.6 supervisor_node.py — расширение `_publish_avatar_tts`

Был:

```python
def _publish_avatar_tts(self, text, language=None, voice=None) -> str:
    request_id = _uuid.uuid4().hex[:8]
    payload = {"request_id": ..., "ssml": ..., "sink": "headset", ...}
```

Стал:

```python
def _publish_avatar_tts(
    self, text, language=None, voice=None,
    sink: str = "headset", request_id: Optional[str] = None,
) -> str:
    # Для sink="preview" caller ЗНАЕТ request_id (от picker'а), протаскиваем.
    # Для sink="headset" — генерируем свой uuid4().hex[:8] (старое поведение).
    if request_id is not None:
        rid = request_id
    else:
        rid = _uuid.uuid4().hex[:8]
    payload = {"request_id": rid, "ssml": ..., "sink": sink, ...}
```

Backward-compat: default `sink="headset"`, `request_id=None` → старое поведение byte-for-byte.

### 2.7 tts_node.py — `_on_avatar_tts_request` switch

Текущая структура (lines 2407+):

```python
sink = chunk_data.get("sink", "")
if sink != "headset":
    # invalid_sink → _avatar_tts_error_pub
    return
# headset-путь: ThreadPoolExecutor + _publish_headset_audio
```

Новая структура:

```python
sink = chunk_data.get("sink", "")
if sink == "preview":
    # Preview-путь: прямой вызов synthesize_preview + preview publishers.
    self._on_avatar_tts_request_preview(chunk_data)
    return
if sink != "headset":
    # invalid_sink → _avatar_tts_error_pub (sink in {preview, headset} only)
    return
# headset-путь (НЕ ТРОНУТ)
```

### 2.8 tts_node.py — `_on_avatar_tts_request_preview`

```python
def _on_avatar_tts_request_preview(self, chunk_data: dict) -> None:
    request_id = chunk_data.get("request_id", "")
    voice = chunk_data.get("voice")
    ssml = chunk_data.get("ssml", "")
    text = self._extract_text_from_ssml(ssml) if ssml else chunk_data.get("text", "")
    if not text or not text.strip():
        # mirror headset guard #2096
        self._publish_preview_error(request_id, "empty_text")
        return
    try:
        result = self.synthesize_preview(text=text, voice=voice, timeout_s=10.0)
    except PreviewSynthesisTimeoutError as exc:
        self._publish_preview_error(request_id, exc.reason)  # preview_timeout
        return
    except PreviewSynthesisUnavailableError as exc:
        self._publish_preview_error(request_id, exc.reason)  # minimax_unavailable
        return
    except PreviewSynthesisError as exc:
        self._publish_preview_error(request_id, exc.reason)  # preview_synthesis_failed
        return
    self._publish_preview_audio(request_id=..., audio_bytes=result.audio_bytes, ...)
    self._publish_preview_result(request_id=..., duration_s=result.duration_s, ...)
```

### 2.9 tts_node.py — `TTSNode.synthesize_preview` (новый pure-synth метод)

```python
def synthesize_preview(
    self, text: str, voice: Optional[str] = None, *,
    provider: Optional[Any] = None, timeout_s: float = 10.0,
) -> PreviewAudioResult:
    # 1. MINIMAX_AVAILABLE guard.
    # 2. empty_text guard.
    # 3. timeout_s > 0 guard.
    # 4. Резолвим provider (НЕ сохраняем в self.minimax_voice — preview
    #    не меняет активный голос личности).
    # 5. settings = TTSSettings(voice, model, language, format=preview_format)
    # 6. async _call() = asyncio.wait_for(provider.synthesize(...), timeout=timeout_s)
    # 7. _run_in_tts_loop(_call()) → tts_audio.
    # 8. Конвертируем TTSAudio → PreviewAudioResult(audio_bytes, content_type,
    #    sample_rate, format_str, duration_s).
```

**Важно**: метод **синхронный** (async внутри через `_run_in_tts_loop`), без `_submit_synthesis`, без `play_audio`, без метрик `record_tts_synthesize`. Preview — diagnostic-tool, и метрики «synthesis ok/fail» для отслеживания качества основного голоса **не должны** загрязняться preview'ом (иначе picker-preview исказит baseline).

## 3. Альтернативы, которые мы рассмотрели и отвергли

### 3.1 Новый топик `/avatar/tts/preview_request` вместо switch по sink

**Отвергнуто**: добавляет новый контракт и ws_server-side mapping. Текущий `/avatar/tts/request` уже параметризован `sink` для разделения путей — расширение перечня значений sink **дешевле**, чем новый канал. Минусы: контракт `/avatar/tts/request` становится «union of all sinks» (легко забыть новый sink при добавлении). Плюсы перевешивают — 1 топик, 1 диспатчер, общий control_callback.

### 3.2 preview через `_run_synthesis_worker` с sink="preview"

**Отвергнуто**: `_run_synthesis_worker` навешен на `_sap_wait_fifo_and_dialogue` (FIFO-очередь воспроизведения) и `_sap_finish_headset` (публикация в `/avatar/tts/audio`). Чтобы использовать его для preview, надо:
- Обойти `_sap_wait_fifo_and_dialogue` (preview не должен ждать диалоговой очереди).
- Обойти `_sap_finish_headset` (preview публикует в `/avatar/preview_voice/audio`, не headset).
- Обойти `_play_seq` (preview не занимает play_seq).

Это ~20-30 строк branch в worker'е + отдельный preview-finner. Прямой вызов `synthesize_preview` из `_on_avatar_tts_request_preview` — ~15 строк, изолировано от headset-пути, легче тестировать.

### 3.3 preview через ROS Service вместо pub/sub

**Отвергнуто**: Service — request/response, синхронный, блокирует caller (quest_node → ws_server). Сейчас архитектура pub/sub (request → audio+result+error), supervisor не блокируется и может логировать. Service добавит 1-1 round-trip зависимость; pub/sub с request_id корреляцией — проще и устойчивее к сетевым таймаутам.

## 4. Грабли (известные проблемы)

### 4.1 Сырой PCM без контейнера — грабли канала ТАРС

`AudioContext.decodeAudioData` декодирует mp3/wav/opus/ogg по MIME-типу. Сырой int16 PCM **без sample_rate в параметре** молча не декодируется (бросает на синхронном decode, либо возвращает пустой буфер на асинхронном). **PreviewAudioResult использует ЗАКОДИРОВАННЫЙ контейнер**, не PCM — иначе picker услышит «молчание без ошибки» (худший вариант для Шифу).

### 4.2 Preview НЕ должен менять активный голос

`synthesize_preview` **резолвит provider** для чтения, но **не сохраняет** voice в `self.minimax_voice`. Это инвариант: тест `test_synthesize_preview_does_not_change_active_voice` следит, чтобы preview не сбивал активный голос личности ТАРС (важно для chain minimax — если picker-preview сменит voice, потомки main-dialogue будут синтезироваться другим голосом).

### 4.3 Таймаут сетевого синтеза

MiniMax/HTTP-провайдер может «висеть» при недоступном upstream. Без таймаута сценарий «MiniMax не отвечает» делает picker нерабочим до рестарта supervisor'а. `asyncio.wait_for(timeout=10.0)` обрывает корутину и бросает `PreviewSynthesisTimeoutError(reason="preview_timeout")`, который supervisor пересылает в `/avatar/preview_voice/error`.

### 4.4 Тест-pollution между test_tts_node_preview и test_yandex_ssml_pitch_volume

Известная существующая проблема (воспроизводится и на HEAD без моих изменений): оба файла используют `object.__new__(TTSNode)` с разными моками; pytest подгружает оба модуля в одном процессе, что-то глобальное ломается. Изолированно оба файла зелёные. **Не блокирует** данную карточку, выходит за рамки — отложить в отдельную задачу.

## 5. Миграция (нулевые breaking changes)

**Никаких миграций не требуется.** Контракт `/avatar/tts/request` **расширен**, не изменён:
- Старые sender'ы (`_publish_avatar_tts(text)` без `sink`) → `sink="headset"` по умолчанию → старое поведение byte-for-byte.
- Старые receiver'ы (тесты headset, ws_server headset-путь) → `sink="headset"` обрабатывается как раньше.
- Новые sender'ы (preview-picker → `_publish_avatar_tts(sink="preview")`) → preview-путь.
- Новые receiver'ы (tts_node preview switch, ws_server preview listeners) → preview-канал.

ws_server `preview_audio_sink.ts` уже ждал mp3/wav контейнер (ADR-0073) — мы просто теперь реально публикуем.

## 6. Acceptance criteria (verify-before-completion)

1. **Unit-тесты** (RED → GREEN → сохраняется GREEN):
   - `src/rob_box_voice/test/unit/tts/test_tts_node_preview.py` (9 тестов на `synthesize_preview`) — все зелёные.
   - `src/rob_box_voice/test/unit/tts/test_tts_node_preview_ros.py` (5 тестов на preview ROS-обвязку в tts_node) — все зелёные.
   - `src/rob_box_supervisor/test/unit/test_supervisor_preview_voice.py` (10 тестов на preview-канал в supervisor) — все зелёные.
   - **Никакие существующие тесты** headset-канала не должны падать (315+ passed supervisor, 2190+ passed voice изолированно).

2. **CC-budget**: новый код (preview publishers + switch + helpers) добавляет < 100 строк в каждый из tts_node/supervisor_node, в пределах `scripts/lint/cc_budget_baseline.json` (ADR-0021).

3. **ADR-нумерация**: ADR-0078 (0077 уже занят `multi-skill-per-profile` и `stt-distortion-collection-tars-helmet`).

4. **e2e** (отдельный процесс после merge): ws_server принимает preview_voice_audio и форвардит в Quest-клиент, picker играет образец через `preview_audio_sink.ts`. **Только после merge в develop** — не блокер для этой карточки.

5. **Test pollution** (test_tts_node_preview vs test_yandex_ssml_pitch_volume): **known issue, не блокирует**. Описывается в PR отдельным пунктом, в отдельную задачу.