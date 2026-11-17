# Captain Bridge (Meta Quest / WebXR) — аудит фич на 30.08.2026

> Ревизия «что обещано ↔ что реально в коде» по капитанскому мостику.
> Источники ожиданий: ADR-0027 (R1–R14), ADR-0032, `docs/architecture/captain-bridge.md`,
> `docs/architecture/meta-quest-api.md`, `docs/plans/2026-08-24-meta-quest-telepresence.md`,
> `docs/plans/2026-08-25-webxr-captain-bridge-design.md`, `webxr_client/README.md`,
> `docs/plans/2026-08-27-quest-voice-passthrough-design.md` + `-plan.md` (голосовой
> пайплайн — см. §4-bis), ADR-0028 (avatar supervisor).
>
> Проверено по коду (`src/rob_box_quest/`), `git log -- src/rob_box_quest`,
> `npm test` → **142/142 PASS** (30.08.2026).
>
> **§2 и §3 описывают состояние ДО работ этой сессии.** Что изменилось —
> см. §5 (Wave 3.A: панели, status HUD, реальные battery/wifi); после
> изменений `npm test` → 182/182, `pytest` → 120 passed / 21 skipped.

---

## 1. Чего ты ждёшь от мостика (сводка ожиданий из документов)

Северная звезда — **телеприсутствие**: робот как аватар оператора в учебном
заведении. Мостик — кресло пилота: большой экран-стена с фронтальной камерой,
вокруг floating-панели с остальными камерами, LiDAR на полу, голос и teleop
с контроллеров.

| # | Требование (ADR-0027 §2) | Приоритет по ADR |
|---|---|---|
| R1 | HTTPS-страница из Quest Browser | must |
| R2 | 3D-сцена: passthrough + LiDAR + камера | must |
| R3 | Teleop через thumbstick / hand tracking | must |
| R4 | Микрофон очков → STT → dialogue_node | should |
| R5 | 3 голосовых режима: passthrough / TTS / STT-LLM | should |
| R6 | Spatial audio (голос из точки робота) | could |
| R7 | Auth / multi-user / failover / battery-warn | could |
| R8 | Live-индикация: Wi-Fi latency, battery, robot status | **must** |
| R9 | Работает и в обычном браузере | must |
| R10 | Стрим-селектор: список стримов, несколько одновременно | should |
| R11 | Детекция людей в сцене | should |
| R12 | Ходимое виртуальное пространство (grid-map + pointcloud) | should |
| R13 | Голосовой режим «LLM-формализация» | could |
| R14 | Панель управления: режимы + логи + админ-утилиты | should |

Плюс из дизайна мостика (2026-08-25): 4 floating-панели полукругом,
draggable, stream-selector UI, layout reset.

Плюс из голосового дизайна (2026-08-27): два агента (личность робота vs
мозг оператора), четыре голосовых режима оператора, панель режимов,
floor-гейтинг голоса. Подробно — **§4-bis**, потому что в R4/R5/R13/R14
это свёрнуто до пары строк и легко недооценить.

---

## 2. Что реально работает (проверено в коде)

### Сервер `rob_box_quest` (Vision Pi)

| Фича | Статус | Где |
|---|---|---|
| Wire-протокол (frame codec, LEB128, topic registry) | ✅ | `rob_box_quest/protocol/frame.py`, `topics.py` |
| WSS-сервер + PIN + heartbeat + watchdog | ✅ | `server/ws_server.py`, `server/session.py` |
| SUBSCRIBE/UNSUBSCRIBE + `subscribe_ack` | ✅ | `server/ws_server.py:371-390`, `:577-585` |
| `teleop_twist` → `cmd_vel_quest` + dead-man/emergency | ✅ | `ws_server.py:428`, `core/teleop.py`, `core/safety.py` |
| `twist_mux` input `quest` (priority 40, timeout 0.5) | ✅ | `docker/main/config/twist_mux/twist_mux.yaml:23` |
| `camera_rear` (OAK-D color, compressed JPEG форвардится как есть) | ✅ | `quest_node.py:446-533` |
| `lidar_2d` (/scan → float32 payload) | ✅ | `quest_node.py:520`, `protocol/topics.py` |
| `robot_status` 1 Hz | ⚠️ было: **battery/wifi заглушки** (`-1` / `0`) → ✅ Wave 3.A: Wi-Fi из `/proc/net/wireless`, battery из JSON/VESC | `streams/status.py`, `streams/wifi.py`, `streams/battery.py` |
| Мульти-камера (oak color/depth, ceiling /dev/video0) | ⚠️ есть код, не подтверждено на железе | `streams/provider.py`, `quest_node.py:480-487` |
| Голос: `voice_ptt_start/stop` (radio + robot_voice), `voice_mode` | ✅ | `ws_server.py:444-472`, `quest_node.py:286-315` |
| Маршрутизация `voice_input_mode` через супервизор | ✅ | `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py:307-330` |
| `stream_list` / `stream_select` (мета-команды) | ✅ сервер | `ws_server.py:417`, `:473` |
| Caddy + self-signed TLS + docker-сервис `quest` | ✅ | `docker/vision/quest/`, compose `:361` |

### Клиент `webxr_client` (Captain Bridge)

| Фича | Статус | Где |
|---|---|---|
| PIN-форма + авто-вход в `immersive-vr` из submit-хендлера | ✅ | `src/main.ts:337-395` |
| Desktop fallback (WASD / Space boost / E emergency) | ✅ | `src/input/desktop_teleop.ts` |
| XR teleop: левый стик — движение, клик правого стика — ARM/DISARM, B/Y — emergency | ✅ | `src/input/xr_teleop.ts`, `teleop_config.ts` |
| Сцена мостика: CC0 GLB + HDR, пол, сетка, ARM-HUD-спрайт | ✅ | `src/scene/captain_bridge.ts`, `bridge_assets.ts` |
| Большой экран-стена с `camera_rear` | ✅ | `captain_bridge.ts:116-131` |
| LiDAR-облако | ⚠️ было: скан **повёрнут на 90°** (перед робота показывался справа), плоско на полу, дальние точки съедал туман и синий цвет → ✅ исправлено (см. §5) | `src/scene/lidar_overlay.ts`, `lidar_payload.ts` |
| Voice PTT: левый grip = radio, правый = robot_voice (+ `voice_mode{ttts_proxy}` заранее) | ✅ | `src/main.ts:107-145`, `input/voice_capture.ts` |
| Overlays: loading / error+watchdog(5 s) / help (H) | ✅ | `src/ui/*` |
| Reconnect с backoff, статус-бейдж | ✅ | `src/wire/connection.ts` |
| Unit-тесты | ✅ 142/142 | `webxr_client/tests/` |

---

## 3. Расхождения: документация врёт

Эти пункты написаны в документах как «сделано», но в коде их нет.
Их надо либо реализовать, либо вычистить из доков.

| Что заявлено | Где заявлено | Факт |
|---|---|---|
| **4 floating-панели полукругом** (`camera_rear`, `oak_color`, `oak_depth`, `ceiling`) | `captain-bridge.md` §4, e2e-чеклист §2, `webxr_client/README.md` | **Панелей не было** (`PanelManager({ defaultTopics: [] })` после `d7aea8be`). ✅ Исправлено в Wave 3.A: 2 боковые панели (`camera_oak_depth`, `camera_ceiling`) на ±75°, доки приведены к факту. |
| «TTS picker (`list_voices` / `set_voice` / `preview_voice`)» | `README.md`, `captain-bridge.md` §8 | Только типы в `wire/messages.ts`. **Ни клиент, ни сервер не реализуют** ни одной из трёх команд. |
| «panel routing (`set_panel_topic`)» | `README.md` | Только тип в `messages.ts`; сервер команду не знает. |
| Панели draggable (raycaster + trigger) | дизайн 2026-08-25 §3 | `Raycaster` в клиенте отсутствует вообще. |
| `voice_state` стрим (0x1202) | `meta-quest-api.md` §4, registry | В registry есть, **публикатора нет** — сервер никогда не шлёт. (A4) |
| `admin_logs` / `admin_logs_stop` (R14) | `captain-bridge.md` §8 | Не реализовано ни на одной стороне. |
| `robot_alert` (BATTERY_LOW / WIFI_WEAK / ROBOT_STUCK) | `meta-quest-api.md` §6 | Не реализовано. |


---

## 4-bis. Голосовой пайплайн — отдельно (я его сначала недооценил)

Источники: `docs/plans/2026-08-27-quest-voice-passthrough-design.md` (+ `-plan.md`),
ADR-0028. В §4 ниже голос был свёрнут до «TTS picker + llm_formalize» — это
сильно уже, чем задумано. Реальная картина:

### Два агента (design §1.1)

| Агент | Что это | Вход |
|---|---|---|
| `dialogue_node` | **личность робота** — wake-word → STT → LLM → TTS | ReSpeaker, обычные люди рядом |
| **супервизор-агент** (в `avatar_supervisor`) | **мозг оператора** — команды → инструменты (TTS / музыка / анимации / навигация) | голос из Quest + текст из Telegram |

Смысл: голос Quest и текст Telegram идут в один вход супервизор-агента, чтобы
«мотивируй народ» из шлема и из Telegram давали одинаковое поведение. Перед
командой оператор гасит диалог на панели, чтобы личность не отвечала параллельно.
PCM-хаб: весь голос Quest → `/avatar/voice_in`; рация = passthrough в `sound_node`,
команда/робот-голос = STT → агент/LLM → TTS.

### Четыре голосовых режима оператора и их статус

| Режим | Кнопка | Что делает | Статус в коде |
|---|---|---|---|
| **рация** | правый grip | голос оператора → динамик робота без обработки | ✅ P1–P3 сделано: `sound_node` стримит `/avatar/voice_in` через `sd.OutputStream`, `ws_server` шлёт barge-in STOP в TTS и музыку |
| **робот-голос** | левый grip | голос → STT → робот повторяет своим голосом | ✅ P7-simple сделано: EOU-детекция в `quest_node`, `/audio/quest_in` → `stt_node` → `/voice/stt/quest` → `dialogue_node._on_quest_stt` (`quest_ttts` = дословный TTS) |
| **команда супервизору** | — | голос → STT → супервизор-агент → действия | ⚠️ **наполовину**: ROS-путь `quest_stt` → `_on_stt(from_quest=True)` → LLM в `dialogue_node` есть, но самого супервизор-агента нет, и клиент никогда не запрашивает этот режим (шлёт только `ttts_proxy`) |
| **панель режимов** | UI на мостике | «я оператор» / «только я рулю» / «выключить диалог» / «голос-рация» | ❌ нет ни UI, ни серверных команд |

### Что ещё лежит в этом пайплайне и не сделано

| # | Что | Где по плану | Факт |
|---|---|---|---|
| V1 | **Панель режимов на мостике** (P5) | design §6, ADR-0028 | Нет. Это и есть R14 «панель управления» — в §4 ниже я записал её как C3 «админ-панель/логи», что неточно: сначала нужны режимы, логи — потом |
| V2 | **Супервизор-агент** (LLM + инструменты, растут инкрементально) (P5) | design §1.1 | Нет. `avatar_supervisor` сегодня — координатор (floors, FSM, `voice_input_mode`), не агент |
| V3 | **Команда супервизору** (P6): голос Quest ≡ текст Telegram → один вход | design §1.1, P6 | Нет (см. «наполовину» выше) |
| V4 | **Пресеты стиля + язык** (P7-full): технический / по понятиям / пещерный / деловой / философ / Ленин; `quest_llm_formalize` | plan P7 | Нет. Но тип `VoicePreset` уже в `wire/messages.ts`, а `mode_manager` уже хранит `currentPreset` — UI и серверная часть отсутствуют |
| V5 | **voice_floor-гейтинг + `voice_state(denied)`** | design §5, D5 | Нет. `LockManager` в супервизоре есть, `acquire_floor`/`release_floor` есть у Telegram-клиента, но **quest-сервер floor не запрашивает вообще** — по D5 это сознательно отложено, «пока один источник голоса». Как только появится Telegram-голос — два голоса поедут одновременно |
| V6 | **Telegram voice → `/avatar/voice_in`** (P8) | plan P8 | Нет как рация: `voice_message_handler` транскрибирует в текст (`/voice/stt/result`) либо озвучивает через TTS; сырого PCM-пути нет |
| V7 | **Подавление эха ReSpeaker на время passthrough** | design §5.1 («уточняется в P1/P4») | Не проверено на железе: сейчас надежда на `tts_grace_s` в `audio_node` |

### Важное про развёртывание

`voice_mode{ttts_proxy}` от клиента применяется **только если супервизор поднят
в `mode:=active`** — в `monitor` `_apply_voice_mode` возвращает
`applied=false` и `voice_input_mode` на `dialogue_node` не меняется
(`supervisor_node.py:317-330`). В `docker/vision/docker-compose.yaml:331` стоит
`AVATAR_SUPERVISOR_MODE=active`, то есть на роботе робот-голос работает; но
Dockerfile-дефолт — `monitor`, и docstring ноды всё ещё утверждает, что active
= NOT_IMPLEMENTED (стало неправдой). Если робот-голос «молчит» — первым делом
проверять именно это.

### Как это меняет приоритеты

`A4` из §4 («`voice_state` стрим») я описал слишком узко: это не только
индикатор listening/thinking/speaking, а часть floor-протокола — по design §5
сервер обязан слать `voice_state(denied)`, когда floor занят. Делать его
осмысленно вместе с V5.

Наименьший шаг с наибольшей отдачей в голосовой ветке — **V1 (панель режимов)**:
она нужна и для команды супервизору (оператор гасит диалог), и для выбора
пресета (V4), и для индикации floor (V5), и закрывает R14.

---

## 4-ter. Взаимодействие с супервизором из очков — что должно быть

Спрашивалось: «как в очках должно быть реализовано взаимодействие с
супервизором». Контракт расписан в ADR-0028 §4.1–4.4 и
`meta-quest-api.md` §5.1/§3. **В коде мостика нет ничего из этого** — ни
одного фрейма, ни одной команды (проверено grep'ом по `rob_box_quest/` и
`webxr_client/src/`: единственное упоминание — комментарий про
`STATE_UPDATE` в `mode_manager.ts:41`).

### Транспорт (ADR-0028 §4.4 + meta-quest-api.md §3)

Тот же WSS-сокет, что и стримы, но **другой subprotocol** —
`robbox-quest-v2` (v1 = quest-сервер, v2 = супервизор; маршрутизация по
subprotocol, endpoint один). Четыре новых бинарных фрейма, msgpack:

| Фрейм | Направление | Payload |
|---|---|---|
| `0x30 SET_MODE` | клиент → супервизор | `{client_id, mode}` |
| `0x31 ACQUIRE_FLOOR` | клиент → супервизор | `{client_id, floor: teleop\|voice}` |
| `0x32 RELEASE_FLOOR` | клиент → супервизор | `{client_id, floor}` |
| `0x33 STATE_UPDATE` | супервизор → клиент | `{state}` — на каждое изменение FSM/floor + 1 Hz keep-alive |

JSON-эквиваленты для отладки (§5.1): `supervisor_set_mode`,
`supervisor_acquire_floor`, `supervisor_release_floor`,
`supervisor_get_state`. Ошибки: `FLOOR_HELD` (floor у другого клиента),
`MODE_CONFLICT` (FSM отклонила переход).

### Что оператор делает в очках

1. **Берёт права.** Прежде чем ехать — `ACQUIRE_FLOOR{teleop}`; прежде
   чем говорить — `ACQUIRE_FLOOR{voice}`. Права независимы: можно рулить,
   пока в Telegram кто-то говорит через робота (режим `mixed`).
2. **Держит права.** Клиент с `teleop_floor` шлёт `teleop_heartbeat`
   (`{client_id, ts_ms, seq}`) **не реже 10 Гц**; пропал больше чем на
   500 мс — супервизор снимает floor и присылает
   `STATE_UPDATE{floors.teleop: none}` (ADR-0028 §4.4 S10). Это отдельный
   контракт от нынешнего ping/watchdog quest-сервера.
3. **Видит, у кого права.** Клиент подписан на `STATE_UPDATE` и **сам**
   решает, что показать: не мой floor — гасим ARM и PTT, пишем «floor
   held by another operator».
4. **Меняет режим аватара** — `SET_MODE`: `off` / `telegram_active` /
   `avatar_present` / `mixed` / `teleop_only` / `voice_only`. Это и есть
   «панель режимов» из голосового дизайна: «я оператор» =
   `avatar_present`, «только я рулю» = `teleop_only`, «выключить диалог»
   — режим, в котором личность робота не отвечает параллельно.

### Как это ложится на мостик (предложение)

Панель режимов — 3D-панель слева от оператора (симметрично боковым
видео-панелям), выбор лучом контроллера + trigger (тот же raycaster, что
нужен для драга панелей, B2). На панели: текущий режим аватара, две лампы
floor-ов (`teleop` / `voice`) с именем держателя, кнопки режимов, тумблер
«диалог робота вкл/выкл». Индикация floor дублируется строкой `FLOOR` в
status-HUD.

Порядок работ: `V1a` фреймы 0x30–0x33 + v2-subprotocol на сервере →
`V1b` STATE_UPDATE в сцену (лампы floor) → `V1c` панель режимов (нужен
raycaster из B2) → `V1d` heartbeat 10 Гц и блокировка ARM без floor.

**Важно про сегодня:** пока floor никто не запрашивает, quest-сервер
публикует `cmd_vel_quest` и голос **без разрешения супервизора**. Это
сознательное упрощение (design D5: «пока один источник голоса — floor
всегда свободен»), но оно ломается ровно тогда, когда появится второй
оператор в Telegram.

---

## 4-quater. RViz-lite на мостике (карта, костмап, путь, поза)

Запрошено: «можно карту наложить, немного функционала rviz в капитанском
мостике». В доках этого нет — R12 говорит про «ходимое виртуальное
пространство» (research Q9), а карта как **слой поверх пола мостика**
нигде не описана. Предлагаю отдельной веткой.

### Что реально можно показать (источники на роботе есть)

| Слой | ROS-источник | Как рисуем | Стоимость |
|---|---|---|---|
| **Occupancy grid** (карта SLAM) | rtabmap (`Grid/CellSize: 0.05`, `Grid/RangeMax: 10`) → `/rtabmap/grid_map` или `/map` | текстура на плоскости пола вокруг центра робота: серый — свободно, чёрный — занято, прозрачный — unknown | новый topic_id + `nav_msgs/OccupancyGrid` → палитра в RGBA/PNG на сервере, чтобы не гонять сырой grid |
| **Local costmap** | nav2 `local_costmap` (`always_send_full_costmap: true`) | вторая текстура поверх карты, палитра инфляции, 1–2 Гц | тот же путь, что и grid |
| **Планируемый путь** | nav2 `/plan` (`nav_msgs/Path`) | `THREE.Line` по полу от центра робота | дёшево, msgpack-массив точек |
| **Поза робота** | `/rtabmap/localization_pose` или `/odom` | маркер + стрелка курса в центре, карта едет под ним | дёшево |
| **Цель / клик по карте** | nav2 `NavigateToPose` | луч контроллера в пол → отправить цель | нужен raycaster (B2) + команда на сервере |

LiDAR-скан (уже есть) — это «сейчас», карта — «память»; вместе они и дают
RViz-подобную картину.

### Решение, которое надо принять: масштаб

Карта помещения 20×20 м не влезает в комнату мостика ~7×8 м. Два честных
варианта:

1. **1:1 от центра** — как сейчас сделан лидар: карта расстелена по полу
   вокруг оператора в реальном масштабе. Видно ближние ~4 м, зато всё
   совпадает со сканом. Дальше стен — обрезается.
2. **Мини-карта-голограмма** — тактический стол перед оператором, карта
   целиком в масштабе 1:10, робот — фишка. Ближе к метафоре «капитанский
   мостик» и не конфликтует со сканом 1:1 на полу.

Предлагаю **оба, но по очереди**: сначала 1:1 (переиспользует центр и
масштаб лидара, минимум новой логики), потом голо-стол.

### Разбивка

| # | Шаг | Объём |
|---|---|---|
| M1 | `occupancy_grid` стрим: сервер (`nav_msgs/OccupancyGrid` → палитра, throttle 1 Гц, topic_id 0x1401) + слой-плоскость на полу 1:1 от центра | M |
| M2 | Путь и поза (`/plan`, `/odom`): линия по полу + маркер курса | S |
| M3 | Мини-карта-голограмма: та же текстура на «тактическом столе» 1:10 + фишка робота | M |
| M4 | Local costmap вторым слоем с переключателем видимости | S–M |
| M5 | Клик по карте → `NavigateToPose` (нужен raycaster B2) | M |

R12 («ходимое пространство» из pointcloud) остаётся отдельной
research-задачей — M1–M5 её не заменяют, но закрывают большую часть
практической пользы RViz на мостике гораздо дешевле.

---

## 4. Актуальный список фич — что осталось сделать

### Wave 3.A — «мостик выглядит и ведёт себя как мостик» (высокий приоритет)

| # | Фича | Почему сейчас | Объём |
|---|---|---|---|
| A1 | ✅ **СДЕЛАНО** — боковые панели (`camera_oak_depth` -75°, `camera_ceiling` +75°) + подписка и роутинг кадров по topic | Сервер отдавал 4 камеры, клиент брал одну | M |
| A2 | ✅ **СДЕЛАНО** — status HUD на стене: BAT / WIFI / SPD / RTT / MODE, подписка на `robot_status` + msgpack-декодер + RTT из ping/pong | Единственное `must`-требование ADR-0027, которое не было закрыто | S–M |
| A3 | ✅ **СДЕЛАНО** — Wi-Fi RSSI из `/proc/net/wireless`, батарея из JSON-снапшота / VESC-напряжения, режим из teleop-состояния | Заглушки `-1`/`0` заменены на реальные источники (батарея — там, где источник есть) | S |
| A4 | **`voice_state` стрим**: публикатор на сервере (`/voice/dialogue/state`) + индикатор listening/thinking/speaking в сцене | Оператор сейчас не видит, услышал ли его робот | S–M |
| A5 | **Стрим-селектор (R10)**: 3D-UI выбора топика для панели → `set_panel_topic` (или `stream_select` + resubscribe) | Сервер уже отдаёт `stream_list`; нужен только клиентский UI + серверный обработчик | M |

### Wave 3.B — качество и удержание состояния

| # | Фича | Объём |
|---|---|---|
| B1 | Layout persistence (`localStorage: rob_box_quest.panel_layout.v1`) | S |
| B2 | ✅ **СДЕЛАНО** — слой указателя (`src/interaction/`): луч, наведение, клик, драг панелей по сфере. Resize — отдельно | M |
| B3 | `robot_alert` события (BATTERY_LOW / WIFI_WEAK / ROBOT_STUCK) + тост в сцене | S–M |
| B4 | FPS-счётчик на Quest 2 (незакрытый пункт acceptance Phase 2.3). RTT — ✅ сделан в Wave 3.A | S |
| B5 | Вычистить враньё из доков (§3) — привести `captain-bridge.md`, README, e2e-чеклист к факту | S |

### Wave 3.C — крупные фичи из ADR (отдельные карточки)

| # | Фича | Требование | Объём |
|---|---|---|---|
| C1 | TTS picker: `list_voices` / `set_voice` / `preview_voice` end-to-end | — | L |
| C2 | Детекция людей + подсветка в сцене | R11, Q10 | L |
| C3 | Админ-панель: `admin_logs` стрим + просмотр в VR (R14 в части логов; режимы — V1) | R14, Q11 | L |
| C4 | Ходимое 3D-пространство (grid-map + pointcloud) | R12, Q9 | XL (research) |
| C5 | `llm_formalize` + пресеты стиля и язык (= V4, P7-full) | R13 | M |
| C6 | Spatial audio (HRTF, голос из точки робота) | R6 | L |
| C7 | Multi-user + эволюция доступа (TOTP/mTLS/туннель) | R7, Q2/Q3/Q12 | XL |
| C8 | H.264/AV1 вместо JPEG, если латентность > 200 мс | ADR §4.3 | L |

---

## 5. Что сделано в этой сессии (30.08.2026)

**A1 + A2 + A3** — «мостик показывает всё, что робот уже отдаёт».

Сервер (`src/rob_box_quest/rob_box_quest/`):

- `streams/wifi.py` (новый) — RSSI из `/proc/net/wireless` на Vision Pi;
  путь инжектится, парсер тестируется без Wi-Fi.
- `streams/battery.py` (новый) — разбор JSON-снапшота (проценты и/или
  вольты по нескольким именам полей) + линейный перевод вольт→проценты по
  ROS-параметрам `battery_voltage_empty` / `battery_voltage_full`. Без
  границ проценты НЕ выдумываются — HUD показывает вольты.
- `streams/status.py` — поле `battery_v`, методы `update_battery` /
  `update_wifi`; `protocol/topics.py` — аддитивное поле `battery_v`
  (`null`, когда источника нет — чтобы отличать от «0 вольт»).
- `quest_node.py` — подписка на `battery_json_topic` (default
  `/device/snapshot`) и опциональная на VESC `/sensors/motor_state/front_left`
  (через try-import, `vesc_msgs` есть не в каждом образе); обновление Wi-Fi
  и режима (`idle` / `teleop_active` / `emergency`) в 1 Hz status-таймере.
- `server/ws_server.py` — ответ `pong` с эхом клиентского `ts_ms`
  (meta-quest-api.md §6/§7): без него RTT измерить было нечем.

Клиент (`webxr_client/src/`):

- `wire/msgpack.ts` (новый) — минимальный msgpack-декодер (~200 строк,
  без runtime-зависимости): сервер шлёт всего два msgpack-стрима.
- `scene/status_hud.ts` (новый) — `parseRobotStatus` + `formatStatusLines`
  (чистая логика) + canvas-спрайт слева вверху на стене.
- `scene/panel_manager.ts` — опция `angles` (раскладка перестала быть
  захардкоженным полукругом).
- `scene/captain_bridge.ts` — боковые панели вернулись
  (`SIDE_PANEL_TOPICS`, ±75°), `videoTopics()`, `ingestPanelFrame()`,
  `setRobotStatus()`, status HUD в сцене.
- `wire/connection.ts` — RTT из ping/pong (`getRttMs()`, `onRtt`).
- `main.ts` — подписка на видео-топики сцены + `robot_status`, роутинг
  кадров по topic, сброс RTT на разрыве.

Проверки: `pytest src/rob_box_quest/test/unit` → 120 passed / 21 skipped;
`npm test` → 182/182; `npm run typecheck` и `npm run build` — чисто;
`black --line-length 120` и `flake8 --max-line-length 120` на изменённых
файлах — чисто.

Документация приведена к факту: `captain-bridge.md` (§1, §2, §3, §4, §5.4,
§7.4, §8, §9), `webxr_client/README.md`, `docs/e2e/captain-bridge-phase2-checklist.md`.

**Не проверено на железе:** боковые панели показывают картинку только если
на Vision Pi поднимается `CameraProvider` (depthai + `/dev/video0`) — в
dev-окружении их нет, поэтому панели остаются чёрными с подписью topic'а.
Батарея останется прочерком, пока не появится источник (`/device/snapshot`
не публикуется, ADR-0010 §4) или пока в образе не будет `vesc_msgs`.

### 5.1. Правка LiDAR (по отдельной просьбе)

Требование было: показания лидара должны быть **видны** и строиться в
пространстве **от точки, которую мы считаем центром**. Что нашлось и что
сделано:

| Что было | Почему это неправильно | Стало |
|---|---|---|
| `x = r·cos(a)`, `z = r·sin(a)` | Смешаны системы координат: в ROS (REP-103) x — вперёд, y — влево; в сцене вперёд — это −Z. Скан выходил **повёрнутым на 90°**: то, что перед роботом (и на экране-стене), рисовалось справа от оператора | `x = −r·sin(a)`, `z = −r·cos(a)` — «вперёд робота» совпадает с «вперёд оператора» |
| Точки на `y = 0.01` (пол) | Луч N10 идёт по плоскости **0.4765 м** над `base_link` (`rob_box.xacro:338`), сам лидар смещён на 0.17 м назад. Проекция на пол — это уже не «показания в пространстве» | Точки на реальной высоте луча + смещение лидара учтено |
| Цвет: близко красный → далеко **синий** | Синий (0x0000ff) на фоне мостика #0a0d11 практически не виден — дальние точки пропадали | Красный → жёлтый → **зелёный** (как в ADR-0027 §4.4), шкала до `max_range = 10 м` из конфига N10 |
| Туман сцены 6→16 м действовал на точки | Лидар видит до 10 м — половина скана растворялась в фоне | `fog: false` для точек и «занавеса» |
| `depthTest: true` | Виртуальная комната мостика ~7×8 м, скан — радиус 10 м: точки за стенами были не видны вообще | `depthTest: false` + `renderOrder` — данные сенсора рисуются поверх декорации (отключается опцией `alwaysVisible: false`) |
| Только точки 5 см | С высоты глаз (1,6 м) точка на полу в 8 м не читается | Точки крупнее + **вертикальный «занавес»**: линия от пола до плоскости луча. Стены комнаты читаются мгновенно |
| Центр нигде не задан явно | — | `LidarOverlay({ center })` + `setCenter()`; центр = начало координат сцены = `base_link` робота = пол под оператором |

Тесты: добавлены проверки маппинга (вперёд → −Z, влево → −X, вправо → +X)
и цветовой шкалы. `npm test` → 187/187.

## 6. Дальше

Четыре независимые ветки:

- **Видео/телеметрия:** A5 (стрим-селектор) → B1/B2 (persistence + drag) → B3 (alerts).
- **Супервизор в очках (§4-ter):** V1a/V1b (фреймы 0x30–0x33 + STATE_UPDATE
  → лампы floor) → V1c (панель режимов) → V1d (heartbeat + блокировка ARM
  без floor).
- **Голос (§4-bis):** V3/V2 (команда супервизору + супервизор-агент) →
  V4 (пресеты стиля и язык) → V5+A4 (floor-гейтинг и `voice_state`).
- **RViz-lite (§4-quater):** M1 (карта 1:1 по полу) → M2 (путь и поза) →
  M3 (голо-стол) → M4/M5.

Общий узел был **raycaster (B2)** — без него нет ни панели режимов, ни
выбора стрима, ни клика по карте. **Сделан:** `src/interaction/`
(`pointer_math.ts` — чистая математика, `pointer.ts` — наведение/клик/драг,
`desktop_pointer.ts` — мышь, `xr_pointer.ts` — контроллер + trigger).
Регистрация новой цели — одна строка:
`bridge.pointer.addTarget({ id, object, draggable })`.

Дальше по любой из веток: A5 (выбор стрима — цели уже кликаются),
V1c (панель режимов), M5 (клик по карте).
