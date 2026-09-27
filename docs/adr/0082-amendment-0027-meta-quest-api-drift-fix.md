# ADR-0082: Amendment к ADR-0027 / `meta-quest-api.md` — drift fix (документ разъехался с кодом)

| Поле | Значение |
|---|---|
| Статус | **Accepted** (drift fix; владелец: Шифу, 2026-09-08) |
| Дата | 2026-09-08 |
| Автор | architect (Hermes Agent), kanban `t_43ce5ff4` (issue #2196) |
| Контекст | issue #2196 (medium, source:gsd) — документ `docs/architecture/meta-quest-api.md`, объявленный «замороженным, любые изменения через ADR-0027 amendment», **разъехался с кодом в обе стороны**: часть команд описана, но не реализована; часть реализована, но не описана; часть описана иначе, чем работает |
| Затрагивает | `docs/architecture/meta-quest-api.md` (правка), `docs/adr/0027-meta-quest-ar-control.md` (история), `src/rob_box_quest/rob_box_quest/server/ws_server.py` (НЕ правится этим ADR — только контракт) |
| Родители | ADR-0027 (Meta Quest AR), ADR-0018 (честный FAIL), ADR-AF-0013 (incremental delivery) |
| Связанные | ADR-0071 (voice_listen_start/stop — Implemented), ADR-0028 (avatar supervisor), ADR-0051 (arbiter split), `docs/architecture/meta-quest-api.md`, `src/rob_box_quest/rob_box_quest/server/ws_server.py`, `src/rob_box_quest/webxr_client/src/wire/messages.ts` |

> **TL;DR.** Companion-документ `meta-quest-api.md` к ADR-0027 объявлен **замороженным до Phase 1**, но с момента принятия (24.08) прошло 2+ недели, реализация Phase 1 + 2 приземлилась семью PR (Quest supervisor, voice_listen_*, voice_pipeline, stream_select и т.д.) и документ потерял синхронность. Этот ADR — **плановая синхронизация**: что снимаем из дока, что добавляем, что переформулируем. Не «дизайн-фаза → реализация», а **правка чертежа по результатам строительства** (с честным списком, что не доехало, что появилось сверх плана, и где документ врал).

---

## 0. Что внутри и что — нет

**Внутри этого ADR.** Каталог drift-точек (что снято, что добавлено, что переформулировано), решения по `deadman`/`seq`/`topic_id-prefix`, обновление статуса ADR-0027, снятие «frozen» с companion-документа, новые §13 «Amendment history» в `meta-quest-api.md`.

**Не внутри.** Реализация недостающих команд (`ui_button`, `admin_logs`, `admin_logs_stop` — отдельные worker-карточки Phase 2, см. §3.2). Правка серверного кода `ws_server.py` — этот ADR **не** правит код, только контракт. Каталог фаз `[voice-vr 07]` — это перекрёстная ссылка GSD-карточки issue #2196 на саму себя; ADR ничего не инвалидирует.

---

## 1. Контекст и бизнес-проблема

### 1.1 Что было написано в `meta-quest-api.md` шапке

Документ `docs/architecture/meta-quest-api.md` (companion к ADR-0027) заявляет в преамбуле:

> «Это reference для реализации Phase 1; **до Phase 1 — заморожен**, любые изменения через ADR-0027 amendment.»

Реальность на 2026-09-08: Phase 1 и часть Phase 2 уже реализованы и влиты (PR #1933 VoiceFloor, #2052/#2106/#2110 voice_listen_*, supervisor API, `voice_pipeline`, `stream_select`). Документ не пересматривался.

### 1.2 Что разъехалось (raw-факты, проверено `git grep` + чтение `ws_server.py`)

**Описано, но не реализовано** (фантомные команды):

| Команда | Док-ссылка | Код | Статус |
|---|---|---|---|
| `ui_button` | `meta-quest-api.md:209`, rate-limit `:531` | `ws_server.py:1624` — **только** в комментарии как «Phase 2»; **нет** `if cmd == "ui_button":` в dispatch `_on_json_cmd`. Упоминание `ui_button` в `sound_node.py:595` — это **отдельный** tool-catalog для MCP, не JSON_CMD от клиента. | Не реализовано. |
| `admin_logs` | `:364` | В коде отсутствует. | Не реализовано. |
| `admin_logs_stop` | `:379` | В коде отсутствует. | Не реализовано. |
| `set_panel_topic` | `:329`, `messages.ts:156` | В коде **отсутствует** в `ws_server._on_json_cmd`; вместо него — `stream_select` (см. ниже). В `messages.ts:156` есть type-only определение, но фактически сервер не знает команды. | Не реализовано (заменено на `stream_select` см. §2.2). |

**Реализовано, но не описано** (дыры в спеке):

| Команда | Код | Где описана | Статус |
|---|---|---|---|
| `voice_pipeline` | `ws_server.py:2277` | Нигде в `meta-quest-api.md`. ADR-0027 §3.4.1 (AV-28 §P7) описывает «шаг 4б, issue #1989», но **отдельный контракт JSON_CMD не задокументирован**. | Реализовано, не описано. |
| `voice_listen_start` / `voice_listen_stop` | `ws_server.py:1864` | ADR-0071 §2.2 (Implemented, 2026-09-07). В `meta-quest-api.md` §11.3 есть только колонка «Evolution полей в Phase 2.1+» — `voice_listen_*` там нет. | Реализовано, не описано. |
| `stream_select` | `ws_server.py:1886` | Только косвенно — в §11.3 как «Phase 2 §6.2: `set_panel_topic {panel_id, topic}`» (на самом деле это про `set_panel_topic`, **другая** команда). `stream_select` сама по себе — orphan. | Реализовано, описание потеряно заменителем. |
| `voice_listen_ack` | `ws_server.py:1880` | Нет в §6 (`JSON_EVENT` список) — а это уже отправляется клиенту. | Реализовано, не описано. |
| `voice_pipeline_ack` / `voice_pipeline_nack` | `ws_server.py:2366` / `:2312, 2349` | Нет в §6. | Реализовано, не описано. |

**Описано иначе, чем работает** (врачащая спека):

| Утверждение в доке | Где | Что в коде |
|---|---|---|
| §5: «`deadman=true` ОБЯЗАН быть на каждом teleop-фрейме; если `false` — сервер игнорирует фрейм» | `meta-quest-api.md:171-174` | `ws_server.py:1655` — поле **читается** в `deadman = bool(payload_obj.get("deadman", False))`, но **не используется** для гейта: `:1690` и `:1703` буквально `_ = deadman` (явный «no-op, не используется пока»). Dead-man живёт в **арбитре** (avatar_arbiter, ADR-0051, см. §3 §2 строки 1677-1689 — гейт по `floor_holder != session_id`), а `deadman` поле — исторический маркер, сейчас не работает. |
| §5: «Throttle: не чаще 30 Гц (`seq` монотонный, сервер отбрасывает фреймы с повторным `seq`)» | `meta-quest-api.md:174` | Anti-replay по `seq` **не реализован** (проверка в коде отсутствует, `seq` передаётся в `relay_teleop_heartbeat` для диагностики/метрик, см. `ws_server.py:1700` и `relay_teleop_heartbeat` в `quest_node.py:370-383` — `ts_ms`/`seq` «для диагностики и метрик `dead_man_trips_total`, но НЕ для синхронизации часов»). Throttle 30 Гц — клиентская ответственность, не серверная. |
| §2: «4-байтовый topic-tag в начало каждого BINARY_FRAME» (топик маршрутизируется по `topic_id` внутри payload) | `meta-quest-api.md:117-125` | `ws_server.py` шлёт `BINARY_FRAME` с **payload как есть** (`protocol.ts:8-14` маршрутизирует по `stream_id`, не `topic_id`). Префикса `topic_id` в payload нет. См. ADR-0027 §3.1: «`BINARY_FRAME.data` — Annex-B NAL-units as-is» — `topic_id` в доках архитектуры, но в wire — `stream_id`. |

**Скрытые правки** (которые дока не зафиксировала):
- ADR-0027 §3.1-bis (AV-25, PR #1933) добавил `voice_state` поля `holder_id` и `detail` — в `meta-quest-api.md` они **есть** (§6 комментарии про `voice_state` состояний), но это скорее случайность: правка была внесена в док **одновременно** с PR #1933.
- `floor_lost` event (AV-19) упомянут в §6, но `floor_lost` от супервизора — отдельный flow; добавление сделано параллельно с реализацией.

### 1.3 Последствия

1. **Фантомные команды** — клиент (или новый воркер) читает док, пытается использовать `ui_button` → получает `BAD_PAYLOAD` или молчаливый ignore → оператор не понимает, что сломалось.
2. **Новые команды без доки** — клиент (webxr_client) реализует кнопку voice_pipeline, шлёт команду; сервер отвечает `voice_pipeline_ack`, но UI не знает имени события, потому что §6 неполон.
3. **Врачащая спека по `deadman`/`seq`** — реальная безопасность teleop живёт в `avatar_arbiter`, **не** в `deadman` флаге. Если кто-то прочитает док и решит «клиент сам отвечает за deadman=true» — он неправильно поймёт, где живёт защита.
4. **`topic_id`-prefix** — если кто-то починит клиент по доке (впишет парсер `topic_id` в BINARY_FRAME), он сломает существующий поток (там только `stream_id`).

---

## 2. Решение

### 2.1 Снимаем «frozen» с companion-документа

Документ больше **не frozen**: это **reference** контракта, который эволюционирует вместе с реализацией. Преамбула переписывается:

> «Это reference для реализации и для клиента; обновляется при каждом изменении wire-протокола. Каждое значимое изменение фиксируется amendment-ом (см. §13 «Amendment history»).»

Шапка также явно указывает статус Phase 1 / Phase 2:
- §1-§4, §10, §11.1-11.2 — Phase 1 + 2 (реализовано);
- §5, §6, §9 — помечают per-cmd «phase N / implemented / planned» (см. §2.3).

### 2.2 Что снимаем / добавляем / переформулируем

| Действие | Где | Решение |
|---|---|---|
| **Снять** `ui_button`, `admin_logs`, `admin_logs_stop` | `meta-quest-api.md` §5 + §9 (rate-limit) | **Удаляем** из §5 как «Phase 2 R14, planned» — на самом деле не реализовано. Помещаем в §12 «Что не в Phase 1» с явной отметкой «not implemented, см. ADR-0027 §6 Q11». Rate-limit `ui_button` (`:531`) — снимаем. |
| **Снять** `set_panel_topic` | `meta-quest-api.md` §5 (строки 327-339) | **Заменяем** на `stream_select` (есть в коде). Раздел `set_panel_topic` помещаем в §12 как «planned, см. ADR-0027 §7 п.9 — стрим-селектор: registry + SUBSCRIBE на несколько камер; `set_panel_topic` отложен до R10 Phase 2». |
| **Добавить** `voice_pipeline` | новый §5.X «Грип-пайплайн (AV-28 §P7 + шаг 4б issue #1989)» | Payload: `{cmd: "voice_pipeline", ts_ms, llm_enabled: bool, preset?: "technical"\|"street"\|"caveman"\|"business"\|"philosopher"\|"lenin"\|"translate"\|"", language?: "ru"\|"en"\|"fr"\|"de"\|"zh"\|"hi"}`. ACK: `voice_pipeline_ack{llm_enabled, preset, language, ts_ms}`. NACK: `voice_pipeline_nack{preset?, language?, reason, ts_ms}` (`reason ∈ "rate_limited"\|"invalid_voice_pipeline_preset"\|"invalid_voice_pipeline_language"`). Rate-limit шарится с `set_voice_style` (1 per 0.5 s). |
| **Добавить** `voice_listen_start` / `voice_listen_stop` | новый §5.Y «Always-on wake-канал (ADR-0071)» | Payload: `{cmd: "voice_listen_start"\|"voice_listen_stop", ts_ms}`. ACK: `voice_listen_ack{active: bool, ts_ms}`. Default при HELLO — `active=true`. Подробности контракта — ADR-0071 §2.2. |
| **Добавить** `stream_select` | новый §5.Z «Stream selector (Phase 2 R10)» | Payload: `{cmd: "stream_select", ts_ms, topic: <ui-name>}`. ACK: `stream_select_ack{topic, stream_id: number\|null, kind: string}`. Если `topic` неизвестен — `ERROR{TOPIC_UNKNOWN}`. |
| **Добавить** `voice_pipeline_ack` / `voice_pipeline_nack` / `voice_listen_ack` | §6 «JSON_EVENT» (расширить) | См. §2.3. |
| **Переформулировать** §5 про `deadman` | `meta-quest-api.md:171-174` | «`deadman: bool` — **диагностический** маркер. Семантика гейта teleop — **avatar_arbiter** (ADR-0051): если `floor_holder != session_id` и `require_teleop_floor=true`, сервер не публикует `cmd_vel_quest` и шлёт `ERROR{FLOOR_HELD}` (rate-limited, ≤ 1 Hz). См. ADR-0027 §3.3 «dead-man switch» + §3.1-bis. Исторически поле `deadman` в `teleop_twist` планировалось как per-frame gate; в реализации gate переехал в арбитр, поле осталось для совместимости клиентских сборок.» |
| **Переформулировать** §5 про `seq` и throttle | `meta-quest-api.md:174` | «`seq: int` — **монотонный** sequence от клиента. Используется **только** в `teleop_heartbeat` relay для метрик `dead_man_trips_total` и диагностики. **НЕ** anti-replay: сервер не отбрасывает фреймы с повторным `seq`. Throttle 30 Гц — **клиентская** ответственность (документировано в `docs/design/teleop-throttle.md`, см. ADR-0027 §2 «latency-бюджет»).» |
| **Переформулировать** §2 про `topic_id`-prefix | `meta-quest-api.md:117-125` | Удалить «4-байтовый topic-tag в начало каждого BINARY_FRAME». Заменить на: «`BINARY_FRAME` payload = сырой байтовый массив (Annex-B NAL для camera, JPEG для ceiling, msgpack для lidar/status/voice_state). **Топик определяется по `stream_id`**, не по префиксу в payload. Маппинг `stream_id → topic` фиксируется в `SUBSCRIBE-ack` (см. §4). Это сознательное упрощение wire-формата против ранней версии дока, в которой планировался `topic_id`-prefix.» |
| **Обновить** §5 §3 handshake-пример | `:90-100` | Добавить `stream_select_ack` (после `subscribe_ack`) и `voice_listen_ack` (после HELLO) — в реалии они идут. |
| **Обновить** ADR-0027 §7 «Изменения в этом ADR» | `0027-meta-quest-ar-control.md:567-619` | Добавить запись: «2026-09-08 — Amendment: drift fix по `meta-quest-api.md` (ADR-0082). Снят 'frozen', companion-док обновлён под фактический код.» |

### 2.3 Каталог команд после правки

После правки §5 «`JSON_CMD` — client → server» содержит ровно те команды, что в dispatch `ws_server._on_json_cmd` (raw-факт на 2026-09-08):

| `cmd` | Фаза | Реализован | Где в доке |
|---|---|---|---|
| `ping` | Phase 1 | ✓ | §5 (есть, с note про webxr_client deviation) |
| `stream_list` | Phase 2 R10 | ✓ | §5 (есть) |
| `teleop_twist` | Phase 1 | ✓ | §5 (есть, переформулирован §2.2) |
| `teleop_heartbeat` | Phase 1 (AV-19) | ✓ | §5 (есть) |
| `stop_emergency` | Phase 1 | ✓ | §5 (есть) |
| `voice_ptt_start` / `voice_ptt_stop` | Phase 2.1+ | ✓ | §5 (есть) |
| `voice_mode` | Phase 2.1+ | ✓ | §5 (есть) |
| `set_voice` | Phase 2.1+ (AV-27/28) | ✓ | §5 §P7 (есть) |
| `list_voices` | Phase 2 §4.1 | ✓ | §5 (есть) |
| `preview_voice` | Phase 2 §4.2 | ✓ | §5 (есть) |
| `voice_pipeline` | Phase 2.1+ (шаг 4б) | ✓ | §5.X (добавлен §2.2) |
| `voice_listen_start` / `voice_listen_stop` | Phase 2.1+ (ADR-0071) | ✓ | §5.Y (добавлен §2.2) |
| `stream_select` | Phase 2 R10 | ✓ | §5.Z (добавлен §2.2) |
| `supervisor_set_mode` / `supervisor_acquire_floor` / `supervisor_release_floor` / `supervisor_get_state` | Phase 2 (AV-16) | ✓ | §5.1 (есть) |
| `ui_button` | Phase 2 R14 | **✗** | **Снят**, в §12 (planned, Q11) |
| `admin_logs` / `admin_logs_stop` | Phase 2 R14 | **✗** | **Снят**, в §12 (planned, Q11) |
| `set_panel_topic` | Phase 2 §6.2 (отложен) | **✗** | **Снят**, в §12 (planned, R10 sub-step) |

`JSON_EVENT` (server → client) после правки §6 содержит все события, реально отправляемые кодом (raw-факт `ws_server.py` + `quest_node.py`):

| `type` | Фаза | Реализован |
|---|---|---|
| `voice_mode_ack` | Phase 2.1+ | ✓ |
| `safety_stop` | Phase 1 | ✓ |
| `robot_alert` | Phase 1 (AV-26) | ✓ |
| `subscribe_ack` / `subscribe_nack` | Phase 1 | ✓ |
| `heartbeat` | Phase 1 | ✓ |
| `voice_state` | Phase 1 (AV-25) | ✓ |
| `stream_list` | Phase 2 R10 | ✓ |
| `stream_select_ack` | Phase 2 R10 | ✓ |
| `voice_listen_ack` | Phase 2.1+ (ADR-0071) | ✓ |
| `voice_pipeline_ack` / `voice_pipeline_nack` | Phase 2.1+ (шаг 4б) | ✓ |
| `voice_list` | Phase 2 §4.1 | ✓ |
| `voice_set_ack` / `voice_set_nack` | Phase 2 §4.3 (AV-27/28) | ✓ |
| `preview_voice_audio` / `preview_voice_done` / `preview_voice_error` | Phase 2 §4.2 | ✓ |
| `ping` / `pong` | Phase 1 | ✓ |
| `floor_lost` | Phase 1 (AV-19) | ✓ |
| `admin_logs_chunk` / `admin_logs_end` | Phase 2 R14 | **✗** — снят |
| `supervisor_state` (через `STATE_UPDATE`/`supervisor_get_state`) | Phase 2 (AV-16) | ✓ |

### 2.4 Тест для каталога (DoD-пункт 1)

В тесте `src/rob_box_quest/test/unit/server/test_meta_quest_api_catalog.py` (новый,
добавляется этим PR) собирается:

1. `implemented_cmds = {cmd for cmd in ws_server._on_json_cmd if ...}` — из AST-разбора `ws_server.py`, множество `if cmd == "...":` веток в `_on_json_cmd`.
2. `documented_cmds = parse_table_meta_quest_api_md()` — из парсинга §5 markdown-таблицы.
3. `assert implemented_cmds == documented_cmds` — после правки оба множества должны совпасть **минус** снятые `ui_button`/`admin_logs*`/`set_panel_topic`, перечисленные в §12 «не Phase 1».

Этот тест — авто-инвариант каталога из [voice-vr 02]-style проверки (issue #2196 DoD: «Каждая команда в `meta-quest-api.md` есть в каталоге и наоборот»).

---

## 3. Что НЕ делаем этим ADR

### 3.1 Не реализуем снятые команды

`ui_button`, `admin_logs`, `admin_logs_stop` — **отдельные** worker-карточки Phase 2 R14 (ADR-0027 §6 Q11 «границы админ-панели»). Этот ADR **не** пишет серверный код для них — только убирает ложные обещания из дока. Реализация — после того, как Шифу примет решение о границах админ-панели (Q11 открыт).

`set_panel_topic` отложен до явного запроса R10 sub-step: drag-from-gui → drop-on-panel требует UI-side работы (TypeScript), отдельная карточка.

### 3.2 Не правим серверный код `ws_server.py`

Этот ADR — **контрактный**, не имплементационный. Правка кода ради «соответствия доке» (если бы код врал в обратную сторону) — отдельные воркеры. Raw-факт: код `ws_server.py` на 2026-09-08 консистентен сам с собой; рассогласование **только** с документом.

### 3.3 Не делаем subprotocol bump (v1 → v2)

`voice_listen_*`, `voice_pipeline`, `stream_select` — добавлены **в Phase 2.1+** на subprotocol `robbox-quest-v1` (см. ADR-0027 §11.3 «Эволюция полей в Phase 2.1+» — naming evolution, без bump-а). v1-клиенты, не знающие команд, получают `ERROR{BAD_PAYLOAD}` — backward-compat, не требует v2-rollout. Эту строку в §11.3 дополняем (см. §2.2 последняя строка).

### 3.4 Не закрываем вопрос `deadman` в коде

Док меняется: «`deadman` — диагностический, gate живёт в арбитре». **Серверный** код (`ws_server.py:1655`, `:1690`, `:1703`) сейчас читает поле и явно no-op'ит — это **не** баг (комментарии в коде это признают: «Phase 1.5: telemetry через deadman-события»). Удаление поля из dispatch — отдельный refactor (touch risk: клиенты шлют, сервер не должен внезапно падать на `KeyError`). В этом ADR — только документ.

---

## 4. Trade-offs

| Получаем | Чем платим |
|---|---|
| Док = фактический код (каждая команда в dispatch имеет описание, каждое описание имеет реализацию) | ADR-0082 — **правка**, не новая фича: Шифу мержит как drift-fix (требование: дата+владелец в шапке, §13 Amendment history). |
| Убраны ложные обещания (`ui_button` 5 Hz rate-limit, `set_panel_topic` panel switching) | Внешний наблюдатель, читающий старую версию дока, не найдёт этих команд — но это и есть цель (док больше не врёт). |
| Тест `test_meta_quest_api_catalog.py` ловит future drift автоматически | +1 тест в pipeline; зависит от AST-парсинга `ws_server.py` (хрупко — переименование метода `_on_json_cmd` сломает парсер). |
| §2 про `topic_id`-prefix переформулирован под `stream_id` (фактический wire) | Док ранних версий, ссылающийся на «topic_id», врёт; нужно явно отметить в §13. |
| `deadman`/`seq` переформулированы под фактическое поведение (arbiter, не per-frame gate) | Внешний контрибьютор может решить «починю deadman в клиенте» — а оно не там живёт. Нужна явная cross-ref на ADR-0051 (avatar_arbiter). |

---

## 5. Файлы и изменения (как реализовано)

- `docs/architecture/meta-quest-api.md` — преамбула, §2, §5, §6, §9, §10, §11.3, **новый §13** «Amendment history». Не правим §1, §3, §4, §7, §8 (там содержание корректно).
- `docs/adr/0027-meta-quest-ar-control.md` — добавляем запись в §7 «Изменения в этом ADR» про этот amendment.
- `src/rob_box_quest/test/unit/server/test_meta_quest_api_catalog.py` — новый тест, проверяющий равенство `implemented_cmds == documented_cmds - planned`.

**НЕ правим**:
- `src/rob_box_quest/rob_box_quest/server/ws_server.py` (контракт vs код — drift идёт **в док**, не в код; см. §3.2).
- `src/rob_box_quest/webxr_client/src/wire/messages.ts` (там type-only определения, док обновляется до их состава; правка messages.ts — отдельный refactor если найдутся несоответствия после правки дока).

---

## 6. Definition of Done

- [ ] Преамбула `meta-quest-api.md` снята «frozen», указывает на §13.
- [ ] §2: удалён «4-байтовый topic-tag», `BINARY_FRAME` описан через `stream_id`.
- [ ] §5: удалены `ui_button`, `admin_logs`, `admin_logs_stop`, `set_panel_topic` (последние 3 — в §12 «не Phase 1»). Добавлены `voice_pipeline` (§5.X), `voice_listen_start/stop` (§5.Y), `stream_select` (§5.Z).
- [ ] §5: `deadman` и `seq`/`throttle` переформулированы согласно §2.2.
- [ ] §6: добавлены `voice_pipeline_ack`/`nack`, `voice_listen_ack`, `stream_select_ack`. Удалены `admin_logs_chunk`/`admin_logs_end`.
- [ ] §9: удалены rate-limit строки для `ui_button` (5 Hz) — команда снята.
- [ ] §11.3: добавлена строка про `voice_listen_*` / `voice_pipeline` / `stream_select` в таблицу naming evolution.
- [ ] §13 «Amendment history» — первая запись: «ADR-0082, 2026-09-08, drift fix (см. ADR-0082 §2.2)».
- [ ] ADR-0027 §7 — добавлена запись про amendment.
- [ ] `src/rob_box_quest/test/unit/server/test_meta_quest_api_catalog.py` — green,
  `implemented_cmds == documented_cmds` для строк со статусом `implemented`;
  planned-команды проверяются отдельно.
- [ ] Все «заморожен» / «not implemented» формулировки в доке имеют **дату** (2026-09-08) и **владельца** (architect profile, issue #2196).
- [ ] PR в `develop`, base=develop.

---

## 7. Риски и откат

| Риск | Митигация |
|---|---|
| Кто-то снаружи опирался на `topic_id` в BINARY_FRAME (несовместимо с wire) | §13 + §2.2 явно отмечают эволюцию. Pre-1.0 клиент не должен был закладываться; если закладывался — это их баг (wire = `stream_id` уже несколько месяцев). |
| Тест `test_meta_quest_api_catalog.py` ломается на любом переименовании в `ws_server.py` | Тест — смоук-инвариант, не unit. При рефакторе `_on_json_cmd` обновляется parser (1 файл). Альтернатива — grep по `if cmd == "...", что более хрупко к стилю. |
| Правка дока без правки кода создаёт новый drift в обратную сторону (док ↔ код через 2 PR) | Тест catalog ловит; CI зелёный = синхронно. Если в код добавится новая команда без дока — PR красный, воркер фиксит док. |
| Шифу не согласится с «снять §5 описание `ui_button`» (хочет, чтобы док оставался планом) | §12 «Что не в Phase 1» — это **тоже** план, но с явной отметкой «not implemented, см. ADR-0027 §6 Q11». Если Шифу скажет «вернуть в §5 как planned» — это 2-строчный revert, ADR остаётся. |

Откат: revert одного PR (правка дока + ADR-0027 §7 + новый тест). Код не тронут, риск отката = 0.

---

## 8. Что ADR НЕ покрывает (явно out of scope)

- **Реализация `ui_button`/`admin_logs`/`admin_logs_stop`** — Phase 2 R14, отдельная worker-карточка после Q11 (ADR-0027 §6).
- **`set_panel_topic`** — Phase 2 §6.2 sub-step, отдельная UI-карточка.
- **Каталог фаз `[voice-vr 07]`** — перекрёстная ссылка issue #2196 на саму себя; ADR-0082 не инвалидирует и не заменяет.
- **Тест из [voice-vr 02]** — ADR-0082 §2.4 вводит именно такой тест (`test_meta_quest_api_catalog.py`), как требовал DoD issue #2196.
- **Deadman в коде** — §3.4 явно: только документ, не код.
- **Subprotocol bump v1→v2** — §3.3, не нужен (backward-compat через `BAD_PAYLOAD` для v1-клиентов, не знающих новых команд).
