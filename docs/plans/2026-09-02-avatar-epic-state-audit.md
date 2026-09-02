# Аватар: аудит состояния эпика на 02.09.2026 — супервизор, капитанский API, мостик, Telegram

> Ревизия «что обещано ↔ что реально живёт на проводе» по четырём поверхностям
> эпика Avatar: `rob_box_supervisor`, wire-API квеста (`rob_box_quest`),
> капитанский мостик (`webxr_client`), Telegram-бот (`rob_box_telegram`).
>
> Предыдущий аудит — `docs/plans/2026-08-30-captain-bridge-feature-audit.md`
> (только мостик). Этот документ шире: он проверяет **сквозные контракты**
> между нодами, а не фичи одной ноды. Именно там нашлось самое неприятное.
>
> Все утверждения ниже — по коду на `develop` @ `bd62c6d4`, с `file:line`.
> Ничего не «должно работать» — либо есть строка, либо её нет.

---

## 0. TL;DR — три вывода

1. **Супервизор зелёный в юнит-тестах и мёртвый на проводе.** FSM, LockManager,
   StateAggregator, msgpack-схема — всё написано и покрыто тестами. Но между
   нодами не работает ни один из четырёх стыков: сервисы объявлены на
   `std_srvs/Trigger`, у которого **пустой Request** (передать `client_id`/`floor`
   физически нечем); `/teleop_heartbeat` публикует один Telegram и **не читает
   никто**; `/avatar/state` супервизор пишет msgpack'ом, а Telegram-клиент
   читает `json.loads`; `SupervisorClient._acquire_via_service` — **заглушка**,
   которая всегда возвращает локальный grant. Это не «недоделанная фича», это
   четыре разрыва контракта, и все четыре невидимы для существующих тестов,
   потому что каждая сторона тестируется отдельно моками.

2. **Мостик с супервизором не разговаривает вообще.** Фреймы `0x30`–`0x33`
   и subprotocol `robbox-quest-v2` **задокументированы** в
   `docs/architecture/meta-quest-api.md` §3/§5.1/§11 (карточка AV-8 закрыта
   правильно — она была docs-only), но в коде нет ни одной строки: сервер
   принимает только `robbox-quest-v1` (`ws_server.py:554`). Значит нет ни
   панели режимов, ни ламп floor, ни гейтинга ARM.

3. **Супервизор-агента («мозга оператора») не существует.** По дизайну
   `docs/plans/2026-08-27-quest-voice-passthrough-design.md` §1.1 голос из
   очков и текст из Telegram должны сходиться в **один** агент с инструментами,
   отдельный от `dialogue_node` (личности робота). Сегодня оба входа идут в
   `dialogue_node`: Telegram — через `forward_to_stt` (`handlers/commands.py:38-45`),
   Quest — через `_on_quest_stt`. То есть «мотивируй народ» из шлема и из
   Telegram отрабатывает **личность робота**, а не мозг оператора.

Порядок работ из этого следует сам: сначала W1 (оживить floor-протокол),
потом W2 (мостик ↔ супервизор), потом W3 (агент), дальше Telegram и добивка.

---

## 1. Поверхность 1 — `rob_box_supervisor`

### 1.1. Что реально есть

| Что | Где | Статус |
|---|---|---|
| `ModeManager` FSM (`off`/`telegram_active`/`avatar_present`/`mixed`) | `core/fsm.py` (470 строк) | ✅ + тесты |
| `LockManager` (`teleop_floor`/`voice_floor`, conflict, transfer) | `core/locks.py` (211) | ✅ + тесты |
| `StateAggregator` + `AvatarState` msgpack pack/unpack | `core/aggregator.py`, `core/state.py` | ✅ + тесты |
| `DeadManCounter` (счётчик трипов) | `core/dead_man.py` | ✅ + тесты |
| Нода: `mode=monitor\|active`, `/avatar/state` 1 Гц (latched, transient_local) | `supervisor_node.py:112-204, 250-263` | ✅ |
| `/avatar/set_voice_mode` → `SetParameters` на `dialogue_node` | `supervisor_node.py:304-360` | ✅ |
| Docker-сервис `avatar-supervisor` на Vision Pi, `AVATAR_SUPERVISOR_MODE=active` | `docker/vision/docker-compose.yaml:344-378` | ✅ |

Итого: **вся чистая логика написана**. Проблема ровно в стыках.

### 1.2. Разрывы контракта (все четыре — блокеры)

#### G1. Сервисы объявлены на `std_srvs/Trigger` — передать `client_id`/`floor` нечем

```
supervisor_node.py:181-196   create_service(Trigger, "acquire_floor", ...)
supervisor_node.py:372-399   _extract_floor_request(request) -> (client_id, floor)
```

`_extract_floor_request` пробует три пути: `request.client_id`/`request.floor`,
затем JSON в `request.data`, затем `request.message`. **В реальном ROS 2
`std_srvs/Trigger.Request` не имеет НИ ОДНОГО поля** (пустой message перед `---`),
поэтому все три `getattr` вернут `None`, и метод всегда отдаст `(None, None)`.
Докстринг это честно называет «заведомо переходным кодом» и ссылается на AV-5,
но AV-5 сделал только Python-dataclasses; **IDL-пакета в репо нет**
(`find src/rob_box_supervisor -name '*.srv'` → пусто, а `ament_python`-пакет
IDL и не может собирать — нужен отдельный `ament_cmake`-пакет, как
`rob_box_perception_msgs`).

Следствие: в `active`-режиме `acquire_floor` от любого клиента вернёт
`BAD_REQUEST`/`granted=false`. Floor-протокола на проводе нет.

#### G2. `/teleop_heartbeat` — один издатель, ноль подписчиков

```
telegram/supervisor_client.py:115  TOPIC_HEARTBEAT = "/teleop_heartbeat"
telegram/supervisor_client.py:271  create_publisher(RosString, TOPIC_HEARTBEAT, 10)
```

`grep -rn teleop_heartbeat src --include=*.py | grep -v test` даёт только
Telegram. Супервизор подписан на `/odom`, `/device/snapshot`,
`/voice/dialogue/state`, `/avatar/set_voice_mode` — и всё
(`supervisor_node.py:166-171`).

Следствие: dead-man 500 мс из ADR-0028 §4.4 S10 **не питается ничем**.
`_check_dead_man_trips` (`supervisor_node.py:227-248`) считает только те
снятия, которые сделал `LockManager` сам по себе — а он их не делает,
потому что `heartbeat()` ему никто не зовёт.

#### G3. `/avatar/state` — msgpack на одном конце, JSON на другом

```
supervisor_node.py:250-263        payload = msgpack.packb(...); msg.data = payload.decode("latin-1")
telegram/supervisor_client.py:388 payload = json.loads(getattr(msg, "data", "") or "{}")
```

`json.loads` на latin-1-обёрнутом msgpack бросает `JSONDecodeError`, который
тут же **молча проглатывается** (`except (JSONDecodeError, TypeError): return`).
Telegram никогда не увидит состояния супервизора и никогда об этом не сообщит.

Это ровно тот класс дефекта, что ADR-0043 (provider-chain docs↔yaml↔test sync) и
ADR-0044 (bot contract drift): две стороны, два теста, ноль тестов на стык.

#### G4. `SupervisorClient._acquire_via_service` — заглушка

```
telegram/supervisor_client.py:409-419
    # NOTE: здесь будет вызов self._node.create_client(...) + client.call_async(req).
    # Сейчас supervisor-нода не существует, поэтому active-режим ведёт себя как monitor.
    return self._fallback_grant(floor)
```

Комментарий устарел — нода существует и на роботе поднята в `active`. Но код
никогда не дёргает сервис: `acquire_floor` всегда возвращает
`AcquireResult(granted=True, contacted_service=False)`. То есть весь
`with_floor` в `telegram_node.py:194-236` — декорация: он никогда никого не
остановит.

### 1.3. Чего нет вовсе

- **G5. Супервизор-агента нет.** `avatar_supervisor` — координатор (FSM, floors,
  `voice_input_mode`), а по дизайну нужен ещё и агент: LLM + инструменты
  (TTS / музыка / анимации / навигация), растущие инкрементально.
- **G6. Клиентского wire-API нет** — см. поверхность 2.

---

## 2. Поверхность 2 — капитанский API (`rob_box_quest`, сервер)

### 2.1. Что реально есть

Wire-протокол v1 (frame codec, LEB128, topic registry), WSS + PIN + heartbeat +
watchdog, SUBSCRIBE/UNSUBSCRIBE + ack, `teleop_twist` → `cmd_vel_quest` +
dead-man/emergency, `camera_rear`, `lidar_2d`, `robot_status` 1 Гц **с реальными**
Wi-Fi (`/proc/net/wireless`) и батареей, голос (`voice_ptt_start/stop`,
`voice_mode`, `/avatar/voice_in`, EOU → `/audio/quest_in`), `stream_list` /
`stream_select`, `pong` с эхом `ts_ms`, Caddy + self-signed TLS + docker-сервис.

### 2.2. Разрывы

| # | Что | Доказательство |
|---|---|---|
| **G7** | subprotocol только v1; фреймов `0x30`–`0x33` нет | `ws_server.py:554` → `WebSocketResponse(protocols=("robbox-quest-v1",))`; `FrameType` (`protocol/frame.py:16-29`) заканчивается на `0x20`/`0xFF`. В `meta-quest-api.md` §3 строки 63-66 и §5.1 строки 316-329 — контракт есть |
| **G8** | `voice_state` (0x1202) — в реестре есть, публикатора нет | `streams/registry.py:74-81` (source `/voice/dialogue/state`); в `quest_node.py` подписки на этот топик нет (`create_subscription` — только odom/scan/camera/battery/vesc, строки 470-492) |
| **G9** | `set_panel_topic` — только тип в клиенте | `webxr_client/src/wire/messages.ts:102-104`; в `_on_json_cmd` (`ws_server.py:390-501`) команды нет |
| **G10** | `list_voices` / `set_voice` / `preview_voice` — только типы | `messages.ts:77-95`, `:149-158`; ни сервер, ни клиент не реализуют |
| **G11** | `robot_alert` — только тип и пороги | `messages.ts:142`, `scene/status_hud.ts:31-33`; события не шлёт никто |
| **G12** | `admin_logs` / `admin_logs_stop` (R14) — нет нигде | — |
| **G13** | `cmd_vel_quest` публикуется **без разрешения супервизора**; teleop-heartbeat 10 Гц с квеста не идёт | `ws_server.py:428` и далее — floor не запрашивается. Сознательное упрощение (design D5), ломается при втором операторе |

---

## 3. Поверхность 3 — капитанский мостик (`webxr_client`)

### 3.1. Что реально есть

PIN-форма + авто-вход в `immersive-vr`, desktop fallback (WASD), XR teleop
(стик / ARM-DISARM / emergency), сцена мостика (GLB + HDR + пол + сетка),
экран-стена `camera_rear`, боковые панели `camera_oak_depth` / `camera_ceiling`
(±75°), status HUD (BAT / WIFI / SPD / RTT / MODE), исправленный LiDAR
(ROS→сцена маппинг, реальная высота луча, красный→жёлтый→зелёный, «занавес»),
voice PTT (левый/правый grip), overlays, reconnect с backoff, слой указателя
(`src/interaction/`: `pointer_math`, `pointer`, `desktop_pointer`, `xr_pointer`),
меню выбора стрима (`scene/stream_menu.ts`, R10). Тесты: 187+.

### 3.2. Разрывы

| # | Что | Доказательство |
|---|---|---|
| **G14** | Панели режимов нет (R14 / V1c) | В `src/ui/` только `mode_manager.ts` (хранит `currentPreset`, `STATE_UPDATE` упомянут комментарием на `:41`), 3D-панели нет |
| **G15** | Индикации floor нет | Нет декодера `STATE_UPDATE`; в `status_hud.ts` строк `FLOOR`/`MODE-holder` нет |
| **G16** | Layout persistence (B1) — нет | `grep -rn localStorage src/rob_box_quest/webxr_client/src` → **ноль попаданий** |
| **G17** | Panel resize — нет | `interaction/` умеет наведение/клик/драг; resize отдельной задачей |
| **G18** | FPS-счётчик (B4) — нет | незакрытый пункт acceptance Phase 2.3 |
| **G19** | Индикатора `voice_state` (A4) нет | следствие G8 |
| **G20** | Пресетов стиля нет | `VoicePreset` в `messages.ts`, `mode_manager.currentPreset` есть; UI и серверная часть отсутствуют |

---

## 4. Поверхность 4 — Telegram-бот (`rob_box_telegram`)

### 4.1. Что реально есть

Auth-мидлварь, ~25 команд (`/photo*`, `/say`, `/goto`, `/waypoints`, `/stop`,
`/pose`, `/control`, `/menu`, `/volume`, `/animation`, `/sound`, `/map`,
`/repl`, `/stopmusic`, `/music`, `/status`, `/clear`, `/myid`), camera cache,
inline-клавиатуры, LLM-чат, голосовые сообщения → STT (Yandex/Whisper) → текст,
observability, `SupervisorClient` с `with_floor` на TTS и движение
(`telegram_node.py:194-236`, `handlers/callbacks.py:95,130`,
`handlers/commands.py:342`, `handlers/messages.py:178`).

### 4.2. Разрывы

| # | Что | Доказательство |
|---|---|---|
| **G21** | Floor-интеграция не работает по-настоящему | Следствие G1+G3+G4: `with_floor` всегда получает `granted=True` локально, состояние супервизора не парсится |
| **G22** | Рации из Telegram нет (P8 / V6) | `voice_processor.py` только транскрибирует OGG→текст; сырого PCM-пути в `/avatar/voice_in` нет |
| **G23** | UX режимов аватара нет (ADR-0028 §6 Q3) | В `handlers/` нет ни одного упоминания `avatar_present`/`telegram_active`/`mixed`; оператор не видит, что очки заняли floor |
| **G24** | Команды идут в личность робота, а не в мозг оператора | `handlers/commands.py:38-45` → `node.forward_to_stt(text)` → `dialogue_node`. Это и есть G5/V3 |

---

## 5. Сводная карта: что осталось и в каком порядке

### Волна W1 — «floor-протокол оживает» (блокирует W2, W3, W4)

| Карточка | Что | Закрывает |
|---|---|---|
| [AV-12 #1904](../../../issues/1904) | Пакет `rob_box_supervisor_msgs` (IDL) + переезд сервисов с `Trigger` | G1 |
| [AV-13 #1905](../../../issues/1905) | `/teleop_heartbeat` подписка + настоящий dead-man 500 мс | G2 |
| [AV-14 #1906](../../../issues/1906) | Единый кодек `/avatar/state` + тест стыка супервизор↔клиенты | G3 |
| [AV-15 #1907](../../../issues/1907) | Telegram: настоящий service-call вместо заглушки | G4, G21 |

### Волна W2 — «мостик разговаривает с супервизором»

| Карточка | Что | Закрывает |
|---|---|---|
| [AV-16 #1908](../../../issues/1908) | Сервер: subprotocol v2 + фреймы `0x30`–`0x33` + JSON-эквиваленты + `STATE_UPDATE` | G6, G7 |
| [AV-17 #1909](../../../issues/1909) | Клиент: v2 + декодер `STATE_UPDATE` + лампы floor в HUD | G15 |
| [AV-18 #1910](../../../issues/1910) | Панель режимов на мостике (3D, на слое указателя) | G14, R14 |
| [AV-19 #1911](../../../issues/1911) | Teleop-heartbeat 10 Гц + гейт `cmd_vel_quest` без floor + блокировка ARM | G13 |
| [AV-20 #1912](../../../issues/1912) | `voice_state` стрим + voice-floor гейтинг + `voice_state{denied}` + индикатор | G8, G19 |

### Волна W3 — «мозг оператора»

| Карточка | Что | Закрывает |
|---|---|---|
| [AV-21 #1913](../../../issues/1913) | Супервизор-агент: скелет (LLM + инструменты) + вход `/avatar/command` | G5 |
| [AV-22 #1914](../../../issues/1914) | Единый вход: голос Quest + текст Telegram → агент; «личность молчит» | G24, V3 |

### Волна W4 — Telegram

| Карточка | Что | Закрывает |
|---|---|---|
| [AV-23 #1915](../../../issues/1915) | Рация из Telegram: голосовое → PCM 16k → `/avatar/voice_in` | G22 |
| [AV-24 #1916](../../../issues/1916) | UX режимов аватара в боте: `/avatar`, live-состояние, уведомление о потере floor | G23 |

### Волна W5 — добивка мостика (документация обещала — кода нет)

| Карточка | Что | Закрывает |
|---|---|---|
| [AV-25 #1917](../../../issues/1917) | Layout persistence + resize панелей + FPS-счётчик | G16, G17, G18 |
| [AV-26 #1918](../../../issues/1918) | `robot_alert` (BATTERY_LOW / WIFI_WEAK / ROBOT_STUCK) + тост в сцене | G11 |
| [AV-27 #1919](../../../issues/1919) | TTS picker end-to-end (`list_voices` / `set_voice` / `preview_voice`) | G10 |
| [AV-28 #1920](../../../issues/1920) | Пресеты стиля + `quest_llm_formalize` (P7-full) | G20 |

### Не берём сейчас (нужно решение Шифу)

- **RViz-lite на мостике** (M1–M5 из аудита 30.08): карта/костмап/путь/поза
  поверх пола, голо-стол 1:10, клик по карте → `NavigateToPose`. Это **новая**
  фича, а не долг: её нет ни в ADR-0027, ни в ADR-0028.
- **C2** детекция людей (R11), **C4** ходимое 3D-пространство (R12),
  **C6** spatial audio (R6), **C7** multi-user/auth-эволюция (R7),
  **C8** H.264/AV1, **C3** `admin_logs` (G12).

---

## 6. Гигиена issue-трекера

Открыты, но по коду **закрыты** — нужно решение Шифу о закрытии:

| Issue | Почему закрыт по факту |
|---|---|
| #1597 [AV-3] FSM ModeManager | `core/fsm.py` + `test/unit/core/test_fsm.py` |
| #1598 [AV-4] LockManager + dead-man | `core/locks.py` + `test_locks.py` (сам dead-man на проводе — AV-13) |
| #1599 [AV-5] `/avatar/state` msgpack schema | `core/state.py` + `test_state.py` (IDL — отдельный долг AV-12) |
| #1601 [AV-7] `voice_input_mode` | `dialogue_node.py:1007,1035`; `config/dialogue_node.yaml:55` |
| #1602 [AV-8] frame-типы `0x30`–`0x33` в доке | `meta-quest-api.md:63-66, 316-329, 451-462` — карточка была docs-only |

Открыты как «зонтики», разложены на волны выше — предлагается закрыть со
ссылкой на этот документ:

| Issue | Что с ним |
|---|---|
| #1639 [quest] Phase 1.5 WebXR-клиент | Клиент существует, 187+ тестов; остатки → AV-25 |
| #1684 [quest] Captain Bridge Phase 2 | Раскладывается на AV-18 (режимы), AV-25 (layout), AV-27 (TTS picker) |
| #1706 [AV-11 deploy] rebuild supervisor образа | Проверить: на роботе уже `AVATAR_SUPERVISOR_MODE=active`, но active-FSM без AV-12 всё равно не рабочий |

---

## 7. Дисциплина для воркеров по этим карточкам

1. **OpenSpec обязателен** (ADR-0038, ADR-0039). Change-folder создаёт триаж;
   воркер **расширяет** `proposal.md` / `design.md` / `specs/<capability>/spec.md`
   / `tasks.md`. Capability для этого эпика — одна из:
   `avatar-floor-protocol`, `captain-bridge-supervisor-api`,
   `supervisor-agent`, `telegram-avatar-client`.
2. **Тест на стык, а не на сторону.** Главный урок этого аудита: четыре
   разрыва прожили месяц под зелёными тестами, потому что каждая сторона
   тестировалась отдельно с моком. В каждой карточке W1/W2 acceptance содержит
   **контрактный тест**, где сериализатор одной ноды кормит десериализатор другой.
3. **Raw-evidence** (ADR-0018): `pytest -v`, `npm test`, `ros2 topic echo`,
   `docker logs` — без этого «сделано» не принимается.
4. **Ветка от `origin/develop`** (ADR-0045). `feature/avatar` удалена — не
   ссылаться на неё (см. #1902).
