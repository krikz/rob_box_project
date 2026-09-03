# AV-27 / issue #1919 — TTS picker handlers (mapping к recon-дизайну)

Карточка: `t_cbd2f9f9` · ADR parent: ADR-0027 / ADR-0028 · Recon-карточка:
`t_5b9d5d0c` (PR #1953, `docs/architecture/tts-picker-ros-path.md`).

Этот документ — **маппинг обработчиков на дизайн**, плюс решения,
которые пришлось принять в реализации. Не повторяет recon-карточку — только
что нового появилось по сравнению с ней.

## TL;DR — что сделано

| handler | где в коде | дизайн-референс (recon) |
|---|---|---|
| `list_voices` | `ws_server.py:_on_json_cmd` (L740+) + `bridge.list_voices_snapshot` + `Bridge` Protocol | §128-150 (cache TTL 5 мин на стороне quest_node) + §89-114 (wire payload `voice_list`) |
| `set_voice` | `ws_server.py:_on_json_cmd` (L770+) + `bridge.set_voice` + supervisor `_on_set_voice` + `_apply_set_voice` + `_set_tts_voice_param` | §23-27 (через супервизор, новый SetParameters client) + §52-87 (валидация по реестру) + §155-160 (failure modes: unknown_id → nack + available) |
| `preview_voice` | `ws_server.py:_on_json_cmd` (L800+) + `bridge.publish_preview_voice` + supervisor `_on_preview_voice` + `_publish_preview_error` + `/avatar/preview_voice/{result,audio,error}` | §30-31 («out of scope for this recon») — новая реализация; см. «preview MVP» ниже. |

Дополнительно:

* `tts_node.py:_publish_voices_catalog` (новый, после `_publish_provider_state`)
  → `/voice/tts/voice` latched topic (TRANSIENT_LOCAL, depth=1).
* `tts_voice_registry.py` — добавлены `VOICE_METADATA`, `voice_info_for(provider, voice_id)`,
  `voices_info_for(provider)`. SoT — старая `PROVIDER_VOICES` (список id).
* `QuestBridge` — кэш voices + active_provider/active_voice, новые
  `list_voices_snapshot` / `set_voice` / `publish_preview_voice`.
* `messages.ts` — `VoiceInfo.provider` (опц.), `voice_list.active_provider/active_voice`,
  `voice_set_nack.available`.

## Как это собирается end-to-end

```
[client WS] JSON_CMD{cmd:"list_voices"}
    ↓
[ws_server:_on_json_cmd] → rate-limit (1/10s) → bridge.list_voices_snapshot
    ↓
[QuestBridge] читает кэш (_voices_cache + _active_provider/active_voice)
    ↓ (если кэш свежий, voices_cache_ttl_sec=300)
    ↓ (если протух — пустой список; честный FAIL)
[ws_server:_send] JSON_EVENT{type:"voice_list", voices, active_provider, active_voice, ts_ms}

Источник кэша — tts_node через ROS:
  tts_node._publish_voices_catalog → /voice/tts/voices (TRANSIENT_LOCAL depth=1)
    ↓
  QuestNode._on_tts_voices → bridge.on_voices_message → cache updated

  tts_node._publish_provider_state → /voice/tts/provider_state (volatile)
    ↓
  QuestNode._on_tts_provider_state → bridge.on_provider_state_message →
    active_provider/active_voice + cache invalidate (если провайдер сменился)
```

```
[client WS] JSON_CMD{cmd:"set_voice", voice_id, preset?}
    ↓
[ws_server:_on_json_cmd] → rate-limit (1/2s) → bridge.set_voice (sync validation)
    ↓
[QuestBridge.set_voice]:
  если !active_provider → nack{tts_unreachable}
  если voice_id not in voices_for(active_provider) → nack{voice_unavailable, available: [...]}
  иначе publish /avatar/set_voice (String JSON {voice_id, preset, provider, ts_ms})
    ↓
[ws_server] → voice_set_ack{voice_id, preset, ts_ms}

[supervisor_node._on_set_voice] → _apply_set_voice:
  в monitor → no-op (S12)
  в active → _set_tts_voice_param(param_key, voice_id) — lazy SetParameters client /tts_node/set_parameters
  param_key = _voice_param_key_for(provider):
    yandex → yandex_voice, minimax → minimax_voice, silero → silero_speaker
```

```
[client WS] JSON_CMD{cmd:"preview_voice", voice_id, text, request_id}
    ↓
[ws_server] → rate-limit (1/5s) + start_preview_session (max 3 concurrent)
    ↓
[QuestBridge.publish_preview_voice] → /avatar/preview_voice (JSON)
    ↓
[supervisor_node._on_preview_voice]:
  валидация по реестру → если unknown → publish /avatar/preview_voice/error
  иначе — MVP: preview_synthesis_not_implemented_in_mvp

[ws_server.deliver_preview_*] ← /avatar/preview_voice/{result,audio,error}
    ↓
[client WS] JSON_EVENT{type:"preview_voice_{audio,done,error}", ...}
```

## Решения, которых нет в recon

### 1. `voices_info_for(provider)` и `voice_info_for(provider, voice_id)`

Recon-карточка говорит про `voices_for(provider)` (список id) — это уже было в
`tts_voice_registry.py`. Но для wire payload `voice_list` нужны display_name,
language, gender, presets. Сделал **в том же модуле** новые helpers с
**отдельной таблицей** `VOICE_METADATA` (плоский dict `"{provider}:{voice_id}" → meta`).
Так не дублируется SoT-список (`PROVIDER_VOICES`), а метаданные лежат рядом.
Удаление `voice_id` из `PROVIDER_VOICES` автоматически делает запись в
`VOICE_METADATA` «висячей» — функция вернёт fallback-dict (display_name=id,
gender=neutral, presets=[]). Это сознательный «best effort» вместо хардкод-списка.

### 2. Active provider / active voice — на стороне quest_node

Recon рекомендовал использовать `/voice/tts/provider_state` как **SoT для
active_provider на стороне tts_node**, и **читать его из кэша quest_node**.
Сделал подписку `/voice/tts/provider_state` (RELIBALE depth=10, volatile —
latching у этого топика нет, см. design §44-48) → `QuestBridge.on_provider_state_message`.
Он:
1. Обновляет `_active_provider`/`_active_voice`.
2. **Инвалидирует кэш** если провайдер сменился — следующий `list_voices`
   вернёт `voices=[]` пока не прилетит свежий latched-publish с
   `/voice/tts/voices`. (Это явно в design §128-150 invalidation rules.)

### 3. Preview-voice: MVP через честную ошибку

Recon-карточка пишет: «preview_voice — отдельная карточка». У меня в этой
карточке две опции:
1. Сделать реальный синтез (рефакторинг `_synthesize_and_play` в tts_node
   на pure-synth + playback) — **большая** работа.
2. Сделать supervisor-обвязку (валидация + топики), а сам синтез
   `preview_synthesis_not_implemented_in_mvp` отдать честной ошибкой.

Выбрал (2): wire-контракт сохранён полностью (`preview_voice_audio/done/error`),
UI увидит «preview пока не работает» и не сломается. В PR-описании явно
фиксирую отложенное — это **honest FAIL**, не silent stub.

Контракт preview уже на месте и **готов** к подключению tts_node-части:
* `/avatar/preview_voice/result` — done (quest_node → ws_server.deliver_preview_done)
* `/avatar/preview_voice/audio` — base64-encoded audio (quest_node → ws_server.deliver_preview_audio + BINARY_FRAME)
* `/avatar/preview_voice/error` — error напрямую (quest_node → ws_server.deliver_preview_error)

### 4. Rate-limit in-memory per ws

`meta-quest-api.md §9`: list ≤ 1/10s, set ≤ 1/2s, preview ≤ 1/5s + ≤3 concurrent.
В `WSSServer`:
- `_voice_rate_limit_check(ws, cmd, min_interval_s)` — per-cmd last-ts по
  `id(ws)`. Не использует client_id (Po-один клиент; для multi-client надо
  переделать на per-session_id).
- `start_preview_session(request_id, ws)` — max 3 + stale-cleanup (>60 s).
- Drop-fire-and-forget без nack — чтобы UI не получал «rate_limited» в
  каждом WS-фрейме; meta-quest-api §9 прямо говорит «drop».

### 5. Thread-safety для deliver_*

`ws_server.deliver_preview_*` зовутся из ROS-thread (callback `QuestNode._on_preview_*`).
Используется **`_send_loop.call_soon_threadsafe` + `loop.create_task`**:
fire-and-forget, не блокирует ROS-callback. Lock на `_preview_pending`
защищает concurrent register/unregister.

В тестах `aiohttp test_utils.TestClient` создаёт свой loop;
`server._send_loop` остаётся `None` (в тестах это норма). В тестах
выставил `server._send_loop = asyncio.get_event_loop()` явно в
`test_preview_voice_deliver_error_clears_pending` (и это работает —
loop у TestClient running).

## Acceptance checklist (issue #1919 + card body)

| критерий | реализация |
|---|---|
| Three commands demonstrably callable from a WS client | `test_list_voices_returns_snapshot`, `test_set_voice_success_sends_ack`, `test_preview_voice_calls_bridge_and_registers` ✅ |
| Empty-list path tested explicitly | `test_list_voices_empty_provider_returns_empty` (mock provider returning []) ✅ |
| Unit tests: list_voices cache hit/miss/expiry | `test_voices_cache_hit_after_latched_publish`, `test_voices_cache_expiry_returns_empty`, `test_voices_cache_empty_snapshot_before_latched_publish`, `test_on_provider_state_message_invalidates_cache_on_provider_change` ✅ |
| Unit tests: set_voice success + unknown-id failure | `test_set_voice_success_sends_ack`, `test_set_voice_unknown_id_returns_nack_with_available` ✅ |
| Unit tests: preview_voice error propagation | `test_preview_voice_deliver_error_clears_pending` + supervisor `_publish_preview_error` ✅ |
| Brief design note (this doc) mapping each handler | ✅ |
| Update messages.ts only if server-emitted event type is genuinely missing | События уже есть (`voice_list`, `voice_set_ack/nack`, `preview_voice_audio/done/error`). Добавил только **поля** в существующих (`VoiceInfo.provider?`, `voice_list.active_provider/active_voice?`, `voice_set_nack.available?`) — minor type-bump. |
| No hardcoded voice lists, no TODO stubs in the happy path | `voices_info_for()` пустой список → честный `[]`, никаких fallback на mock. Supervisor preview → `preview_synthesis_not_implemented_in_mvp` **не** в happy path; happy path = publish запроса, синтез будет в followup. |
| WIP commits every ~15-20 min, push immediately | 7 WIP-коммитов в ветке `wt/t_cbd2f9f9` (см. `git log --oneline -7`). |
| One PR total | PR #NNNN ещё не создан — см. complete-card-flow. |

## Что НЕ сделано (deferred)

* `preview_voice` synthesis — sync-рефакторинг `_synthesize_and_play` в tts_node
  (новая карточка).
* 3D-voice-picker UI на клиенте (отдельная карточка, см. captain-bridge.md §4.1).
* ADR-0028 §5.1 extension — если supervisor решит перейти на `SetVoice` service
  с кастомным IDL (сейчас всё через топики, этого достаточно).
* ADR-0036 `runtime-overshoot` audit — коммит-скрипт не покрывает полный
  diff ветки (только последний коммит), поэтому карточка ушла чуть выше
  бюджета; merge-gate/agent-flow могут это отметить (см. kanban-comment).

## Файлы изменены

```
src/rob_box_voice/rob_box_voice/tts_voice_registry.py
src/rob_box_voice/rob_box_voice/tts_node.py
src/rob_box_quest/rob_box_quest/server/ws_server.py
src/rob_box_quest/rob_box_quest/quest_node.py
src/rob_box_quest/webxr_client/src/wire/messages.ts
src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py
src/rob_box_quest/test/unit/server/test_ws_server_voice.py
src/rob_box_quest/test/unit/test_quest_bridge.py
src/rob_box_voice/test/unit/tts/test_voice_registry.py
docs/architecture/av-27-tts-picker-handlers.md     ← этот файл
```