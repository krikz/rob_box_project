# ADR-0027: AR-управление роботом через Meta Quest 2 — WebXR + HTTPS + Zenoh-мост

| Поле | Значение |
|---|---|
| Статус | Accepted (дизайн-фаза, реализация отложена) |
| Дата | 2026-08-24 |
| Автор | architect (Hermes Agent), kanban t_77f08bb8 |
| Контекст | Issue #1576 (LOW priority, design-only) — Meta Quest 2 / 3 / Pro как нативный WebXR-клиент к роботу: passthrough + camera stream + LiDAR overlay + teleop + микрофон очков |
| Затрагивает | (будущее) новый docker-сервис `rob_box_quest` в `docker/vision/`; companion-документ `docs/architecture/meta-quest-api.md` (HTTP/WS контракт) |
| Родители | ADR-0017 (Zenoh router SPOF), ADR-0018 (honesty), ADR-0026 (recovery-contract) |
| Связанные | issue #1576; ADR-0021 (dialogue_node discipline — точка входа для voice-mode bypass) |

> **TL;DR.** Поднимаем на Vision Pi лёгкий WebSocket-шлюз с самоподписанным TLS,
> который (а) отдаёт статический веб-клиент (Three.js + WebXR Device API),
> работающий и в обычном браузере (desktop/планшет), и в Meta Quest; (б)
> пробрасывает бинарные потоки camera + LiDAR как плоский `MessagePack`-стрим
> через единый WSS, (в) принимает команды teleop/voice и публикует их в ROS2
> через существующий Zenoh session. Никаких rosbridge / web_video_server — это
> лишняя движущаяся часть; у нас уже есть Zenoh между Vision Pi и Main Pi,
> хватит тонкого моста. Auth — короткоживущий PIN-код, привязанный к локальной
> сети робота. Никакого мульти-юзера на PoC-этапе. **Северная звезда** —
> удалённое присутствие (телеприсутствие), но PoC начинаем строго в локальной
> сети; эволюция auth/TLS/доступа — отдельная задача (§1.1).

---

## 1. Контекст и бизнес-проблема

Товарищ Шифу (issue #1576, 23.08.2026) предлагает Meta Quest 2 / 3 / Pro как
**нативный WebXR-клиент** для робота. Кейс:

1. Оператор в очках видит **passthrough** (то, что видят очки) + 3D-overlay
   (модель робота в уменьшенном масштабе, траектория, статус-иконки).
2. **Стрим с камеры робота** (rear/main) — для случаев, когда оператор
   смотрит «глазами робота», а не своими.
3. **LiDAR-точки** в AR-сцене — чтобы видеть, где препятствия вокруг робота.
4. **Teleop** через контроллеры Quest (thumbstick + buttons) или hand tracking.
5. **Микрофон очков** как альтернатива ReSpeaker: можно говорить с роботом
   через очки, в том числе в режиме **voice passthrough** (свой голос из
   динамика робота) или **voice → robot TTS** (робот говорит «от своего
   имени» голосом робота).

Текущее состояние репо (`docs/architecture/SYSTEM_OVERVIEW.md`):

- Два Raspberry Pi (Vision Pi 10.1.1.11, Main Pi 10.1.1.10), связаны через
  **Zenoh router** на каждом (ADR-0017), не DDS.
- Голос: `dialogue_node` (vision Pi) слушает `/audio/speech_audio`
  (AudioData), публикует `/voice/state`, вызывает LLM, синтезирует TTS.
- Teleop: `teleop_twist_joy` (main Pi) → `twist_mux` → `/cmd_vel`.
- Perception: OAK-D Lite, LSLIDAR N10 → ROS2 топики.
- **Внешних HTTP/WebSocket-эндпоинтов нет.** Мониторинг (Grafana / Prometheus)
  на отдельном docker-compose, не в основном стеке.

То есть текущая система полностью self-contained, без какого-либо внешнего
клиентского API. Это и нормально (робот автономен), но фича #1576 требует
**наружу торчащий HTTPS-сервис** с самоподписанным TLS.

### 1.1. Видение v2 (Шифу, 24.08.2026) — робот как аватар

Это **не** только «Quest-гейт в лаборатории», а инструмент **телеприсутствия**:
робот как аватар оператора.

**Северная звезда.** Удалённое присутствие в учебном заведении: оператор
через робота видит аудиторию, слышит, что говорят роботу, говорит через него,
перемещает его.

**Но начинаем с PoC в локальной сети** (DNS-имя на роутере
`quest.rob_box.local` → HTTPS → веб-консоль), без проброса наружу.
Удалённый доступ — позже и отдельно.

**Клиент перестаёт быть только Quest.** Та же консоль должна работать в
обычном браузере (desktop/планшет); Quest остаётся одним из клиентов.
Отсюда требования R9–R14 ниже.

Что расширяет исходное видение:

1. **Виртуальное пространство («комната»).** 3D-сцена окружения, по которой
   можно «ходить», а не только passthrough. Открытый вопрос: хватает ли
   текущих данных (grid-map nav2 + pointcloud rtabmap + OAK-D depth) для
   такого пространства, или нужен отдельный research (SLAM-меш/сплаттинг) — Q9.
2. **Отмечать людей.** Детекция людей в стриме/облаке точек, подсветка в сцене.
3. **Стримы — выбираемые.** Список доступных (rear/depth/OAK-D/…), выбор какие
   показывать одновременно; сейчас в §3.2 только один фиксированный `camera_rear`.
4. **Голос — 4-й режим «LLM-формализация»:** оператор наговорил речь → LLM
   переписал в формальную/структурированную → TTS голосом робота (§3.4).
5. **Панель управления:** переключение режимов (движение/голос/стримы),
   просмотр логов, задел на админ-утилиты (статус/restart/диагностика).

**Что это меняет в ADR-0027:** auth/TLS не должны навсегда замыкаться на
PIN + self-signed — северная звезда потребует mTLS/TOTP, настоящий DNS/TLS
и туннель (Tailscale/Cloudflare Tunnel). PoC не должен блокировать эти пути
(см. Q12).

---

## 2. Требования (acceptance из issue #1576)

| # | Требование | Приоритет |
|---|---|---|
| R1 | HTTPS-страница, доступная из Meta Quest WebXR browser | must (Phase 1) |
| R2 | 3D-сцена: passthrough + LiDAR overlay + стрим камеры | must (Phase 1) |
| R3 | Teleop через thumbstick / hand tracking | must (Phase 1) |
| R4 | Микрофон очков → STT → dialogue_node | should (Phase 2) |
| R5 | 3 голосовых режима: passthrough / TTS / STT-LLM | should (Phase 2) |
| R6 | Spatial audio (голос из точки робота) | could (Phase 3) |
| R7 | Auth / multi-user / failover / battery-warn | could (Phase 3) |
| R8 | Live-индикация: Wi-Fi latency, battery, robot status | must (Phase 1) |
| R9 | Веб-клиент работает и в обычном браузере (не только Quest) | must (Phase 1) |
| R10 | Стрим-селектор: список доступных стримов (rear/depth/OAK-D), выбор одновременно | should (Phase 2) |
| R11 | Детекция людей в сцене (отметка/подсветка) | should (Phase 2) |
| R12 | Виртуальное пространство: ходимая 3D-сцена окружения (grid-map + pointcloud) | should (Phase 2, research Q9) |
| R13 | Голосовой режим «LLM-формализация» (наговорённый текст → LLM → формальная речь → TTS) | could (Phase 3) |
| R14 | Панель управления: переключение режимов + просмотр логов + задел на админ-утилиты | should (Phase 2) |

Latency-бюджет для комфортного teleop (эмпирика для XR-телеопов):
- camera → display: ≤ 80 мс
- controller input → robot: ≤ 50 мс
- итого round-trip: ≤ 200 мс комфортно, ≤ 400 мс терпимо (только если
  без rapid maneuvers). Это будет формализованный SLO сервиса.

---

## 3. Решение (high-level)

Один новый docker-сервис **`rob_box_quest`** на Vision Pi (там уже сидит
voice_assistant и ближе к Wi-Fi роутеру пользователя — типично Quest в одной
комнате с Vision Pi). Сервис держит **один** TCP-сокет (WSS) на `8443`
(self-signed TLS, см. §4.1) и **три** внутренних стыка в ROS:

```
┌─────────────────────┐    wss://10.1.1.11:8443     ┌─────────────────┐
│  Meta Quest 2/3/Pro │◄────────────────────────────►│ rob_box_quest   │
│  (Three.js + WebXR) │  • binary frames (camera,    │ (Vision Pi)     │
│                     │    lidar, audio)             │                 │
│                     │  • JSON commands (teleop,    │ ws-server ◄──► ROS Zenoh session
│                     │    voice-mode, ui-buttons)   │                 │    │
└─────────────────────┘                              └─────────────────┘    │
                                                                              ▼
                                                                  Zenoh router (Vision)
                                                                              │
                                                                              ▼
                                                                  Zenoh router (Main)
                                                                              │
                                                                              ▼
                                                                  /cmd_vel, /audio/*, etc.
```

Ключевые принципы:

1. **Единый WSS, multiplexed.** Один сокет, фреймы тегируются `stream_id`.
   Проще балансировать, нет гонки за N портов, нет проблем CORS.
2. **MessagePack, не JSON.** Для бинарных потоков (camera JPEG/PointCloud)
   JSON в 5-10× тяжелее; для команд — JSON подмножество (только потому что
   человеко-читаемо и отлаживаемо из web-консоли).
3. **Не вводим rosbridge.** rosbridge — это generic ROS↔WS мост с discovery
   protocol'ом и bson-кодированием; для нашего use-case (3 конкретных потока
   + 3 конкретных команды) он over-engineered и лишняя движущаяся часть.
4. **Не вводим web_video_server.** MJPEG через HTTP — это latency-потолок в
   100-200 мс (один HTTP request/response per frame); нам нужен WebSocket
   binary stream.
5. **Безопасность = сетевая близость + PIN.** Никакого OAuth, никакого
   remote-доступа через интернет. Сервис слушает на LAN IP, аутентификация
   — 6-значный PIN, ротируемый при каждом включении. Этого хватает для
   research-сценария в лаборатории.

### 3.1. Транспорт и стримы

**WSS protocol schema (фрейм):**

```
[1 byte: type][4 bytes: stream_id (LE)][varint: payload_len][payload]
```

| type | name | payload | направление |
|---|---|---|---|
| 0x01 | `HELLO` | `{client_version, capabilities, session_pin}` | client → server |
| 0x02 | `WELCOME` | `{server_version, session_id, server_time_ms}` | server → client |
| 0x03 | `SUBSCRIBE` | `{topic: "camera_rear"\|"lidar_2d"\|"lidar_3d"\|"voice_state"\|"robot_status"}` | client → server |
| 0x10 | `BINARY_FRAME` | MessagePack `{topic, seq, ts_ms, data_bytes}` | server → client |
| 0x11 | `JSON_CMD` | MessagePack `{cmd: "teleop_twist"\|"ui_button"\|"voice_mode"\|"voice_ptt_start"\|"voice_ptt_stop"\|"stop_emergency", ...}` | client → server |
| 0x20 | `GOODBYE` | `{reason}` | обе стороны |
| 0xFF | `ERROR` | `{code, message}` | обе стороны |

Полная схема + примеры MessagePack-фреймов — в companion-документе
[`docs/architecture/meta-quest-api.md`](../architecture/meta-quest-api.md).

### 3.2. Внутренний ROS-стык

Сервис запускается внутри того же docker-compose, что и `zenoh-router-vision`,
и подключается к Zenoh session через `ZENOH_SESSION_CONFIG_URI` (тот же
механизм, что у `dialogue_node`, см. ADR-0017). Никакого DDS в
rob_box_quest — только Zenoh pub/sub.

Подписки (Zenoh keyexpr):

| Topic (UI-имя) | ROS2 keyexpr | Частота | Транспорт |
|---|---|---|---|
| `camera_rear` | `ros2_main/oak_d/stereo/image_rect_raw` или MJPEG topic с Vision Pi | 30 fps | H.264 в `BINARY_FRAME.data` (Annex-B, ~2 Mbps на 720p) |
| `lidar_2d` | `ros2_main/lslidar/scan` | 10 Hz | MessagePack: ranges + intensities |
| `lidar_3d` (опц.) | `ros2_main/rtabmap/cloud_map` | 2-5 Hz | подвыборка до 10k точек, zstd-compressed в `data_bytes` |
| `voice_state` | `voice/state` | event | MessagePack: `idle`/`listening`/`thinking`/`speaking`/`denied` (state + ts_ms + utterance_id? + holder_id? + detail?). Семантика `denied`/`holder_id`/`detail` — VoiceFloor (server-side mutex, см. §3.1-bis ниже и `rob_box_quest/server/voice_floor.py`); контракт зафиксирован в [meta-quest-api.md §6](../architecture/meta-quest-api.md). Аудит G8/G19 (issue #1912). |
| `robot_status` | агрегатор (mode, battery, Wi-Fi rssi) | 1 Hz | MessagePack |

Публикации (Zenoh keyexpr):

| Команда (UI) | ROS2 keyexpr | Throttle |
|---|---|---|
| `teleop_twist` | `cmd_vel_quest` (потом через `twist_mux` → `/cmd_vel`, см. ADR-sibling) | 30 Hz max |
| `ui_button` (sound/light/etc.) | сервис-вызовы через существующие ROS-сервисы | по нажатию |
| `voice_mode` (passthrough/ttts/stt) | параметр `dialogue_node` (`voice_input_mode`) | один раз при смене |
| `voice_ptt_start/stop` (push-to-talk) | публикация в `/audio/ptt` (новый topic) | edge-triggered |
| `stop_emergency` | `/safety/emergency_stop` (dead-man fail, см. §3.3) | edge-triggered |

**Teleop через `twist_mux`.** В docker-compose `docker/main/config/twist_mux/twist_mux.yaml`
уже определены input-ы для joystick и nav2. Добавляем input `quest` с
приоритетом **ниже** joystick (чтобы физический пульт всегда побеждал) и
таймаутом 0.5 с — если от Quest нет фреймов 0.5 с, twist_mux отключает этот
input и робот останавливается (dead-man switch, см. §3.3).

### 3.1-bis. VoiceFloor — серверный mutex на голосовой поток (дополнение, AV-25)

> **Дополнение от 03.09.2026** (post-ADR, реализовано в PR #1933,
> контракт — issue #1912 + meta-quest-api.md §6).

Когда AV-23 (Telegram-рация) приземлится, два WS-клиента (operator-quest
в Meta Quest + telegram-bridge) получат возможность одновременно слать
голос. Голосовой поток квеста (`voice_ptt_start/stop`, `VOICE_AUDIO`
фреймы → `/avatar/voice_in`) сейчас один на всех WS-клиентов. Без
серверного gate'а два клиента начнут микшировать голос в один PCM
и рвать звук.

**Решение**: добавляем in-memory `VoiceFloor` в `rob_box_quest.server`:

- ровно один держатель на все WS-сессии;
- `holder_id` = `"<client_id>:<session_id_short>"` (≤ ~32 символа, чтобы
  влезло в `voice_state.detail` без обрезки);
- `voice_ptt_start`:
  - floor свободен → `try_acquire` → `state=listening` + `holder_id`;
  - floor занят → `state=denied` + `holder_id` текущего + `detail="busy: <holder_id>"`,
    bridge **не** вызывается;
- `voice_ptt_stop`:
  - от держателя → `state=idle`;
  - от НЕ-держателя → no-op (защита от двойного stop от постороннего);
- watchdog/disconnect → `force_release_for(old_session_id)` → следующий
  клиент может захватить floor;
- `SUBSCRIBE voice_state` → snapshot текущего состояния сразу (UI не
  ждёт первого события).

Состояние `denied` — **локальное расширение** для UI (quest-сервера),
не входит в основной state-машину `dialogue_node`. Это позволяет
показать «у робота говорит другой» без введения нового типа события
или wire-frame.

Полный список тестов: `test_voice_floor.py` (13 unit, pure-logic) +
`test_ws_server_voice.py` (8 integration на aiohttp WS) +
`test_voice_floor_e2e_full_flow.py` (e2e-сценарий двух клиентов) +
`test_voice_floor_edge_cases.py` (watchdog-trip, retry-storm, recon).

### 3.3. Dead-man switch и safety

Teleop через Quest — это **контроллер, который пользователь может уронить,
или очки, которые могут потерять Wi-Fi**. Безопасность:

1. **Dead-man switch на grip-button.** Пока `grip` зажат — команды идут;
   как отпустил — `twist_mux` через 100 мс отрубает input. Безопаснее, чем
   палец на стике (стик можно случайно задеть).
2. **Wi-Fi watchdog.** Сервер отслеживает интервал между фреймами от
   клиента; если > 500 мс — шлёт emergency-stop и закрывает WSS.
3. **Emergency button (B) на контроллере Quest.** Один tap → публикация в
   `/safety/emergency_stop` (новый ROS-сервис или topic), который
   выставляет `twist_mux.lock = true` пока оператор не нажмёт A.
4. **Локальный safe-stop на стороне сервера.** Каждые 200 мс сервер шлёт
   heartbeat-фрейм; клиент не получил 3 подряд → UI показывает «CONNECTION
   LOST», стик в UI подсвечивается красным.

### 3.4. Voice modes (Phase 2)

Четыре режима — это **не** четыре параллельных пайплайна, а один параметр
`voice_input_mode` на стороне `dialogue_node` (новый), который меняет
поведение в одной точке:

| Режим | Поведение dialogue_node | Зачем |
|---|---|---|
| `passthrough` | AudioData из `/audio/quest_in` идёт **напрямую** в `/voice/audio/out` (динамик робота), без STT/LLM | Оператор хочет поговорить с человеком рядом с роботом через микрофон очков (hands-free) |
| `ttts_proxy` | AudioData → STT → LLM (без wake-word, потому что PTT) → TTS голосом робота | Оператор хочет, чтобы робот **от своего лица** ответил человеку рядом (например, ассистент на мероприятии) |
| `stt_llm` (default) | AudioData → STT → LLM, как обычная wake-word-активация, но без wake-word (gaze-click активирует режим) | Управление роботом голосом через очки (без ReSpeaker) |
| `llm_formalize` (Phase 3) | AudioData → STT → LLM-перефразирование в формальную/структурированную речь → TTS | Оператор наговорил черновик, робот озвучивает «причёсанную» версию (выступление/объявление) |

Реализация — `dialogue_node` декомпозирует voice_input (см. ADR-0021 R1),
новый параметр `voice_input_mode` ∈ `{respeaker, quest_passthrough,
quest_ttts, quest_stt, quest_llm_formalize, quest_command, off}`. Это
**мини-фича в dialogue_node**, не отдельный сервис — голосовой пайплайн
у нас уже есть, и дублировать его ради Quest было бы безумием.

| Режим | Поведение dialogue_node | Зачем |
|---|---|---|
| `passthrough` | AudioData из `/audio/quest_in` идёт **напрямую** в `/voice/audio/out` (динамик робота), без STT/LLM | Оператор хочет поговорить с человеком рядом с роботом через микрофон очков (hands-free) |
| `ttts_proxy` | AudioData → STT → LLM (без wake-word, потому что PTT) → TTS голосом робота | Оператор хочет, чтобы робот **от своего лица** ответил человеку рядом (например, ассистент на мероприятии) |
| `stt_llm` (default) | AudioData → STT → LLM, как обычная wake-word-активация, но без wake-word (gaze-click активирует режим) | Управление роботом голосом через очки (без ReSpeaker) |
| `llm_formalize` (Phase 3) | AudioData → STT → LLM-перефразирование в формальную/структурированную речь → TTS | Оператор наговорил черновик, робот озвучивает «причёсанную» версию (выступление/объявление) |
| **`quest_command`** (AV-22, Issue #1914) | AudioData → STT → **опубликовать** распознанную фразу в `/avatar/command` (`source="quest"`), **не запускать LLM** личности | Голосовая команда супервизор-агенту: «мотивируй народ» из очков → супервизор-агент (AV-21) обрабатывает. Личность молчит. |
| `off` (W3-1) | Глушит ТОЛЬКО обычных людей у ReSpeaker-микрофона | «Диалог выключен для окружающих, полное управление у оператора» (см. dialogue-mode-spec-2026-08-28.md §3.5) |

**Выбор `quest_command` как нового значения, а не переиспользование
`quest_stt`** (зафиксировано в `docs/plans/2026-09-02-avatar-worker-brief.md`
и OpenSpec `supervisor-agent/design.md`, AV-22):

* `quest_stt` → запускает LLM личности (`dialogue_node._on_stt(from_quest=True)`)
  — это «диалог с роботом через очки»;
* `quest_command` → публикует в `/avatar/command`, LLM личности НЕ
  вызывается — это «голосовая команда супервизор-агенту»;
* Разные потребители (диалоговая нода vs супервизор-агент) — разные
  режимы. Переиспользование `quest_stt` сломало бы гейт «личность молчит»
  и перепутало бы два независимых потребителя одной фразы.

#### 3.4.1. Стиль речи и язык вывода (AV-28 §P7, фаза P7-full)

В режиме `quest_llm_formalize` оператор говорит в грип своими словами, а
робот озвучивает их в выбранном **стиле речи** и на выбранном **языке
вывода**. Это **не диалог** — LLM не отвечает оператору, не задаёт
вопросов, не добавляет фактов. Она только переписывает реплику в стиле
выбранного пресета. Жёсткие ограничения (`max_tokens`/`temperature`/
no-tools/no-questions) — в спецификации PR #1952 §5.

Два дополнительных параметра на `dialogue_node`:

| Параметр | Допустимые значения | Где хранится |
|---|---|---|
| `voice_preset` | `technical` / `street` / `caveman` / `business` / `philosopher` / `lenin` | `src/rob_box_voice/config/voice_presets.yaml` (manifest) + `presets/<id>.txt` (system prompt, RU и EN в одном файле) |
| `voice_output_language` | `ru` / `en` | тот же файл, поле `languages: [ru, en]` |

**Тексты пресетов — данные, а не код**: добавление нового пресета =
правка YAML + новый `.txt`, без правок Python (требование origin-карточки
#1920). Контракт: «preserve meaning / no answering / no invented facts»
+ явная language-directive. Это no-dialog contract (см. PR #1952 §5).

**Маршрутизация — через супервизор (ADR-0028 S5)**, как `voice_input_mode`:

```
клиент (WebXR) → ws_server.cmd=='set_voice'{preset?, language?}
   → Bridge.set_voice_preset / set_voice_language
   → /avatar/set_voice_preset | /avatar/set_voice_language
   → avatar_supervisor → SetParameters(voice_preset=…) или
                                      SetParameters(voice_output_language=…)
   → dialogue_node (применяется к следующей фразе, без рестарта ноды)
```

Прямых `SetParameters` на `dialogue_node` из `rob_box_quest` нет — supervisor
единственная точка записи для всех voice-параметров (ADR-0028 S5, в т.ч.
`voice_input_mode` уже там). Whitelist пресетов и языков — единый на стороне
ws_server (`VOICE_PRESET_IDS` / `VOICE_LANGUAGES`) и supervisor; невалидное
значение → `voice_set_nack{reason:invalid_voice_preset|language}`, UI
откатывает optimistic update.

**HUD-индикатор** текущего пресета в WebXR-клиенте: короткая метка
`ST:LENIN` / `ST:LENIN@RU` рядом с chip-кнопками стиля речи (чтобы оператор
видел с расстояния, не всматриваясь в chip-надписи). Префикс `ST:`
именно для **стиля речи**, чтобы не путать с `voice_id` (TTS picker,
AV-27) — другой «слой», отдельный выбор голоса. Контракт рендера —
pure-функция `renderHud(preset, language)` в
`src/rob_box_quest/webxr_client/src/ui/voice_presets_panel.ts`; формат:

```
renderHud(null, null)             = "ST:--"
renderHud("lenin", null)          = "ST:LENIN"
renderHud("lenin", "ru")          = "ST:LENIN@RU"
```

HUD обновляется оптимистично при локальном клике (ещё до ack от
сервера) и подтверждается через `voice_set_ack` (mode_manager). На
невалидный preset/language → UI откатывает optimistic update по
`voice_set_nack.reason` (UI-state хранится в mode_manager, см.
`voice_presets_panel.ts:setCurrentPreset/setCurrentLanguage`).

**Контракт клиент↔сервер** (см. `docs/architecture/meta-quest-api.md` §P7
+ `src/rob_box_quest/webxr_client/src/wire/messages.ts`):

* CMD: `set_voice {voice_id?, preset?, language?}` — все поля опциональны,
  клиент шлёт то, что реально поменялось.
* ACK: `voice_set_ack {voice_id?, preset?, language?, ts_ms}` — UI синхронизирует
  mode_manager.
* NACK: `voice_set_nack {voice_id?, preset?, language?, reason, ts_ms}` — UI
  откатывает optimistic update (mode_manager к предыдущему значению).

`voice_id` (TTS picker, AV-27) — out of scope этой карточки: серверная
часть маршрутизирует только `preset` + `language`. Расширение на
`voice_id` — отдельная работа.

---

## 4. Решения по каждому подпункту acceptance

### 4.1. HTTPS-стек: выбираем Caddy, не nginx/Apache

| Вариант | Плюсы | Минусы | Решение |
|---|---|---|---|
| **Caddy 2.x** | auto-TLS из коробки (если есть домен), Hot-Reload конфига без перезапуска, единый бинарник ~40 MB, читаемый Caddyfile | менее распространён в ROS-комью-нити; мало готовых Dockerfile под ROS2 base | **выбираем** |
| nginx + certbot | самый популярный, много примеров | certbot требует публичный домен + 80 порт наружу; cert-renew таймер; конфиг verbose | отвергнут: не нужен публичный домен |
| Apache | legacy, ничего особенного | тяжёлый, конфиг сложнее | отвергнут |
| Только self-signed + nginx | минимум зависимостей | cert-renew вручную; нет HSTS pre-load | отвергнут: certbot даёт ту же сложность с дополнительной вознёй |

**Для lab-сценария — self-signed Caddy без ACME.** Caddyfile:

```
quest.rob_box.local, 10.1.1.11 {
    tls /certs/selfsigned.crt /certs/selfsigned.key {
        protocol tls1.3
    }
    reverse_proxy localhost:8765
}
```

`/certs/selfsigned.crt` генерируется при первом запуске контейнера
(`openssl req -x509 -newkey rsa:2048 -days 365 -nodes ...`). Импортируется
в Meta Quest через Settings → Privacy → Security → Trusted Sources (один
раз, persistent).

**Trade-off:** self-signed означает, что при смене IP/CN придётся
пере-импортировать сертификат. Это OK для лаборатории, где IP фиксирован
в `10.1.1.0/24` (см. ADR-0017).

### 4.2. Стек WebXR: Three.js + WebXR Device API, не Babylon/A-Frame

| Вариант | Плюсы | Минусы | Решение |
|---|---|---|---|
| **Three.js + нативный WebXR Device API** | ~120 KB gzip; полный контроль над пайплайном; огромная экосистема; AR/Passthrough через `XRSession.requestHitTestSource` | ниже уровень абстракции (нужно писать UI-фреймворк самому) | **выбираем** |
| Babylon.js | мощнее для 3D-сцен (физика, GUI), хорошая AR-поддержка | ~400 KB gzip; over-engineered для нашего overlay (5 объектов) | отвергнут |
| A-Frame | декларативный, прототип за час | плохо контролируется frame budget (60 fps для passthrough обязателен); не нативный WebXR, поверх three.js с магией | отвергнут как dev-стек, не для production |
| react-three-fiber | если бы у нас был React-стек | у нас нет React в репо (только server-side Python ROS) | отвергнут: лишняя зависимость |

**Three.js + hand-written TypeScript без UI-фреймворка.** Бандлер — esbuild
(быстро, ~5 сек build). Single-page client компилируется в
`docker/vision/quest_static/` и монтируется в Caddy как `root`.

### 4.3. Camera stream: H.264 через WebSocket, не MJPEG / WebRTC

| Вариант | Latency | CPU на Pi | Bandwidth | Решение |
|---|---|---|---|---|
| MJPEG over HTTP | 100-300 мс (HTTP overhead per frame) | низкий (ffmpeg -c copy) | ~5 Mbps на 720p | отвергнут: latency-потолок |
| **H.264 через WSS (binary frames)** | 50-100 мс | умеренный (ffmpeg h264_v4l2m2m на Pi) | 1-3 Mbps на 720p | **выбираем** |
| WebRTC (через janus/mediasoup) | 30-80 мс | высокий (SFU/MCU) | 1-3 Mbps | отвергнут: SFU = +100 MB RAM + сложный STUN |
| ROS `image_transport` + theora | 200+ мс | низкий | 1-3 Mbps | отвергнут: legacy, нет hard-real-time |

**H.264 через WSS-binary:** `ffmpeg -i /dev/video0 -c:v h264_v4l2m2m -pix_fmt
yuv420p -g 30 -bf 0 -tune zerolatency -f h264 pipe:1 | ros2_quest` —
ffmpeg-процесс пушит NAL-units прямо в WSS-фрейм. На стороне клиента —
`MediaSource` API для WebCodec, либо `<img src="blob:">` через
`URL.createObjectURL(new Blob([nal]))` каждые ~100 мс.

**Trade-off:** H.264 — проприетарный кодек (royalty-free начиная с 2018, но
патентная ситуация в РФ остаётся нюансом); для lab это OK, для production
продажи робота — пересмотр в сторону AV1 или H.266. Записать как
tech-debt для Phase 3.

### 4.4. LiDAR: 2D-scan (10 Hz) для MVP, 3D-pointcloud (2-5 Hz) для Phase 3

2D-scan (`/scan`, LaserScan, 10 Hz, ~360 точек) уже используется для
навигации; рендерим его как цветной point-cloud на полу AR-сцены с цветом
от дистанции (зелёный → красный). 3D-pointcloud (PointCloud2, до 1М точек
на скан) — подвыборка до 10k точек, zstd-сжатие в payload, рендер как
отдельный BufferGeometry в Three.js. Phase 3, потому что оптимизация
выборки и сжатия — это отдельная research-задача.

### 4.5. Auth: PIN-код, не OAuth

Для research-фичи в лаборатории:

1. При старте контейнера генерируется 6-значный PIN (логируется в
   `docker logs rob_box_quest`).
2. Клиент при `HELLO` шлёт PIN; сервер проверяет и либо даёт
   `WELCOME{session_id}`, либо `ERROR{code: AUTH_FAIL}`.
3. PIN — общий на сессию контейнера; для multi-user (Phase 3) добавим
   per-client nonce.

**Trade-off:** PIN — это security-by-obscurity, любой кто увидит `docker
logs` получит доступ. Для lab — приемлемо. Для production — нужен mTLS
или хотя бы TOTP. Записать как **открытый вопрос Q3** (см. §6).

### 4.6. Failover: «safe stop», не «continue last command»

Если HTTPS-сервер упал:

1. **Twist_mux** отрубает `quest` input через 500 мс (Wi-Fi watchdog, см. §3.3).
2. **Twist_mux** сам падает → робот останавливается (`/cmd_vel = 0`) по
   safety-цепочке VESC-контроллера.
3. **dialogue_node** получает `voice_input_mode != respeaker` → возврат
   в режим `respeaker` через 3 с (если есть ReSpeaker; если нет — просто
   ждёт wake-word через WebSocket-канал, который отвалился).

**Не выбрано «continue last command»**: для мобильного робота в AR-сцене с
человеком рядом «continue last command» означает «робот продолжает ехать
туда, куда оператор его послал 30 секунд назад, даже если оператор уже
не смотрит на робота». Это failure-mode, который в тестах VR-телеопов
приводит к инцидентам. Safe stop — единственный честный выбор.

---

## 5. Альтернативы, которые НЕ выбраны (целиком)

### 5.1. rosbridge_suite

rosbridge — generic WS↔ROS мост с discovery protocol и bson. Для нашего
use-case это over-engineering:

- Нам нужно ровно 3 read-топика и 5 write-топиков — статический schema.
- rosbridge тянет за собой JSON-marshalling всего ROS-graph'а
  (services, params, tf), что небезопасно (поверхность атаки).
- В репо нет rosbridge (ADR-0017 ушёл от DDS к Zenoh; rosbridge поверх
  Zenoh — ещё один слой абстракции, который мы не хотим поддерживать).

### 5.2. web_video_server

MJPEG-streaming для ROS2. Простой, но:

- Latency 100-300 мс (HTTP per-frame), не вписывается в 200 мс SLO.
- Не умеет passthrough/AR-overlay — это вообще не его зона.
- Конфликтует с §3.1 «единый WSS».

### 5.3. VNC / noVNC / RDP

«Просто покажем экран, на котором rviz с AR-overlay». Viable для
desktop-сценария, но:

- Meta Quest WebXR browser не имеет VNC-клиента; пришлось бы ставить
  Android-приложение и терять WebXR-overlay.
- Passthrough в rviz нет (это десктопное окно, а не AR).
- Latency на XR-телеопах по RDP/VNC: 150-400 мс, не вписывается.

### 5.4. Standalone Android-приложение Quest

Вместо WebXR — нативное Quest-приложение на Android. Мощнее, но:

- Цикл деплоя: Android Studio + gradle + signing — не наш стек.
- Удвоение работы (WebXR-вариант для всех браузеров + Android-вариант).
- Потеряем cross-platform (любой WebXR-браузер, не только Meta).

Оставлено как **Phase 4 опция**, если WebXR упрётся в производительность
(60 fps для passthrough — это порог).

---

## 6. Открытые вопросы (для follow-up карточек)

| # | Вопрос | Куда | Когда |
|---|---|---|---|
| Q1 | Где живёт сервис: Vision Pi (10.1.1.11) или Main Pi (10.1.1.10)? | Решено в §3 — Vision Pi | — |
| Q2 | Multi-user (2+ очков): кто главный? | ADR-followup при Phase 3 | не сейчас |
| Q3 | Auth: PIN → TOTP → mTLS? | ADR-followup при Phase 3 | не сейчас |
| Q4 | Codec: H.264 → AV1 / H.266 для production? | tech-debt Phase 3 | при выходе из lab |
| Q5 | Hand tracking: pinch-grab AR-объектов — нужна ли в MVP? | Phase 2 (опц.) | при физ. наличии Quest 3/Pro |
| Q6 | Spatial audio: где взять HRTF-pipeline для робота? | Phase 3 | — |
| Q7 | Battery-warn: client-side check через WebXR `XRDeviceUsage` API | Phase 1 (просто), Phase 3 (точный %) | — |
| Q8 | Что если упадёт Zenoh-роутер (ADR-0017)? | см. §4.6 — safe stop | связано с ADR-0017 |
| Q9 | Хватает ли данных для «ходимого» 3D-пространства (grid-map + pointcloud + OAK-D depth) или нужен SLAM-меш/сплаттинг? | research-карточка Phase 2 | после PoC-стримов |
| Q10 | Откуда брать детекцию людей: OAK-D depthai / отдельный YOLO-топик / rtabmap? | follow-up Phase 2 | с R11 |
| Q11 | Границы админ-панели: только чтение логов или restart/диагностика? Кто имеет права? | follow-up Phase 2 | с R14 |
| Q12 | Эволюция доступа к северной звезде: PIN → TOTP → mTLS, DNS/TLS, туннель (Tailscale/Cloudflare Tunnel) — не блокирует ли текущий контракт этот путь? | ADR-amendment при выходе из PoC | до Phase 3 |

---

## 7. Изменения в этом ADR

**Сейчас (дизайн-фаза):**

1. Этот файл — `docs/adr/0027-meta-quest-ar-control.md` (принят).
2. Companion-документ `docs/architecture/meta-quest-api.md` с детальным
   HTTP/WS-контрактом (frame schema, MessagePack-структуры, error codes).

**Когда дойдёт до реализации (Phase 1):**

3. Новый пакет `src/rob_box_quest/` — Python (ROS2 node + ws-server на
   `aiohttp` или `websockets`).
4. `docker/vision/docker-compose.yaml` — сервис `rob_box_quest` + опц.
   healthcheck.
5. `docker/main/config/twist_mux/twist_mux.yaml` — добавить `quest` input
   с приоритетом ниже joystick и timeout 0.5 с.
6. `src/rob_box_voice/rob_box_voice/dialogue_node.py` — параметр
   `voice_input_mode` и топик `/audio/quest_in` (мини-фича в рамках
   ADR-0021 R3 «per-bag workflow», отдельная worker-карточка).
7. `/safety/emergency_stop` topic + handler (или сервис, зависит от того,
   как уже сделано в rob_box_bringup).
8. Веб-клиент: статический билд Three.js + WebXR Device API, source
   в `src/rob_box_quest/webxr_client/` (или отдельный `webxr_quest/`
   монорепо), собирается esbuild'ом в `docker/vision/quest_static/`.
   Работает и в обычном браузере, и в Quest (R9).

**Phase 2/3 (Видение v2, §1.1):**

9. Стрим-селектор: registry доступных стримов + `SUBSCRIBE` на несколько
   `camera_*` одновременно (R10).
10. Детекция людей → топик `person_detections` → подсветка в 3D-сцене (R11, Q10).
11. Ходимое виртуальное пространство: grid-map + pointcloud как 3D-сцена (R12, Q9).
12. Голосовой режим `llm_formalize` в `dialogue_node` (R13, §3.4).
13. Админ-панель: логи, статус, restart/диагностика (R14, Q11).
14. Эволюция доступа к северной звезде: TOTP/mTLS + DNS/TLS + туннель (Q12).

**Не делаем:**

- Не вводим rosbridge / web_video_server (см. §5).
- Не делаем standalone Android-приложение (см. §5.4).
- Не выносим voice-pipeline в отдельный сервис — расширяем dialogue_node.

**Уточнение (03.09.2026, t_53a576a4, issue #1912):**

В §3.1 таблица стримов указывает Zenoh-keyexpr `voice/state` для
voice_state. В этом же разделе §7 п.6 выше упомянут топик `/audio/quest_in`
в контексте параметра `voice_input_mode` dialogue_node. Это **не**
расхождение имени одного и того же топика: `voice/state` — это
**событийный** VoiceState (FSM → UI), а `/audio/quest_in` — это
**командный** входной канал (PCM от Quest-микрофона). Они дополняют
друг друга, оба попадают в одну FSM внутри dialogue_node. Пометка
закрыта; AV-25 (PR #1933) добавил VoiceFloor как server-side mutex
между WS-клиентами (см. §3.1-bis).

---

## 8. Acceptance для дизайн-фазы (эта карточка)

| # | Критерий | Где |
|---|---|---|
| AC1 | Задокументировано архитектурное решение | этот ADR |
| AC2 | Определён WebSocket API контракт | `docs/architecture/meta-quest-api.md` |
| AC3 | Выбран HTTPS-стек (Caddy) с обоснованием | §4.1 |
| AC4 | Оценён latency budget для teleop | §2 + §4.3 |
| AC5 | Решён security model (PIN) | §4.5 |
| AC6 | Выбран стек WebXR (Three.js + native WebXR) | §4.2 |
| AC7 | Зафиксированы trade-offs всех альтернатив | §5 |
| AC8 | Открытые вопросы с owners для follow-up карточек | §6 |

После принятия этого ADR карточка завершается; реализация — отдельные
worker-карточки с конкретными acceptance (Phase 1/2/3).