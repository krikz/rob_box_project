# rob_box_quest — WebSocket / REST API контракт

> Companion-документ к [ADR-0027](../adr/0027-meta-quest-ar-control.md).
> Описывает **точный** wire-протокол между Meta Quest WebXR-клиентом и
> сервисом `rob_box_quest` (Vision Pi). Это reference для реализации
> Phase 1; до Phase 1 — заморожен, любые изменения через ADR-0027 amendment.

## 1. Транспорт

- **Сетевой уровень:** HTTPS + WSS (WebSocket over TLS 1.3).
- **Endpoint:** `wss://10.1.1.11:8443/quest` (single endpoint, multiplexed).
- **Альтернативный HTTP-endpoint** (для healthcheck и pre-auth UI):
  `https://10.1.1.11:8443/healthz`, `https://10.1.1.11:8443/`.
- **TLS:** self-signed сертификат на Caddy (см. ADR-0027 §4.1). CN =
  `quest.rob_box.local`, SAN = `10.1.1.11`. Импортируется в Meta Quest
  через Settings → Privacy → Security → Trusted Sources (один раз).
- **Auth:** 6-значный PIN, передаётся в `HELLO`-фрейме.
- **Subprotocol:** `robbox-quest-v1` (обязателен; иначе сервер отвергает
  рукопожатие).

## 2. Frame format (binary)

Единый формат для всех сообщений, оба направления:

```
[1 byte : type]
[4 bytes: stream_id (little-endian uint32)]
[varint : payload_len (LEB128)]
[payload_len bytes : payload]
```

- **`type`** — 1 байт, см. таблицу §3.
- **`stream_id`** — uint32, клиент и сервер используют **разные** диапазоны,
  чтобы не пересечься:
  - Client-initiated streams (commands): `0x0001..0x0FFF`
  - Server-initiated streams (data): `0x1000..0xFFFF`
  - `0x0000` зарезервирован для control-фреймов (`HELLO`, `WELCOME`,
    `GOODBYE`, `ERROR`).
- **`payload_len`** — LEB128 varint; позволяет payload до 16 MiB (для
  одного H.264 NAL-unit этого хватит с запасом; большие скан-блоки
  LiDAR режутся на ≤ 64 KiB чанки).
- **`payload`** — для `BINARY_FRAME` = сырой байтовый массив; для
  `JSON_CMD` / `HELLO` / `WELCOME` / `SUBSCRIBE` / `ERROR` = JSON
  (UTF-8, без BOM). **Не** MessagePack в payload — JSON человеко-читаем
  и проще дебажится из web-консоли; payload компактных потоков
  (LiDAR 2D-scan ~360 точек, voice_state event) и так влезает.

## 3. Frame types

| `type` | Имя | Направление | Payload schema |
|---|---|---|---|
| `0x01` | `HELLO` | client → server | `{client_version: "0.1.0", capabilities: ["webxr","hand_tracking"], session_pin: "123456"}` |
| `0x02` | `WELCOME` | server → client | `{server_version: "0.1.0", session_id: "<uuid4>", server_time_ms: 1234567890, robot_status: {...}}` |
| `0x03` | `SUBSCRIBE` | client → server | `{topic: "camera_rear"\|"lidar_2d"\|"lidar_3d"\|"voice_state"\|"robot_status", quality: "low"\|"med"\|"high"}` |
| `0x04` | `UNSUBSCRIBE` | client → server | `{topic: "..."}` |
| `0x10` | `BINARY_FRAME` | server → client | binary blob (raw bytes; topic указан в subscribe-confirm или заголовке см. §4) |
| `0x11` | `JSON_CMD` | client → server | см. §5 |
| `0x12` | `JSON_EVENT` | server → client | см. §6 |
| `0x20` | `GOODBYE` | обе стороны | `{reason: "user_logout"\|"shutdown"\|"timeout"\|"auth_fail"}` |
| `0xFF` | `ERROR` | обе стороны | `{code: "AUTH_FAIL"\|"BAD_PAYLOAD"\|"RATE_LIMIT"\|"TOPIC_UNKNOWN"\|"INTERNAL", message: "..."}` |

**Handshake:**

```
client → HELLO   (stream_id=0)
server → WELCOME (stream_id=0)
client → SUBSCRIBE(camera_rear)
client → SUBSCRIBE(lidar_2d)
client → SUBSCRIBE(robot_status)
server → BINARY_FRAME(camera_rear) ...    (loop)
server → JSON_EVENT(voice_state=idle)     (event)
client → JSON_CMD(teleop_twist)           (loop, 30 Hz max)
...
client → GOODBYE(reason="user_logout")
```

## 4. `BINARY_FRAME` — server → client data streams

Каждый server-initiated stream (после `SUBSCRIBE`) получает свой
`stream_id` из диапазона `0x1000..0xFFFF`. Чтобы клиент понимал, что
именно внутри `BINARY_FRAME`, в начало каждого фрейма добавлен
**4-байтовый topic-tag** (только для BINARY_FRAME, для JSON_CMD/JSON_EVENT
топик внутри JSON):

```
[4 bytes: topic_id (LE)]   ← внутри payload, перед данными
[N bytes: raw data]
```

`topic_id` — uint32, ассоциация задаётся в `SUBSCRIBE-confirm`:

| Topic UI-name | topic_id | Формат data |
|---|---|---|
| `camera_rear` | `0x1001` | H.264 Annex-B NAL-units (один или несколько подряд; не Annex-B целиком, потому что клиент сам собирает Annex-B для MediaSource) |
| `camera_front` | `0x1002` | (Phase 2) — панорамная передняя камера |
| `lidar_2d` | `0x1101` | little-endian float32: `[angle_min, angle_max, angle_inc, range_min, range_max, time_increment, scan_time, n_points]` + `n_points × float32 ranges` + `n_points × float32 intensities` (соответствует `sensor_msgs/LaserScan` ROS2 msg) |
| `lidar_3d` | `0x1102` | zstd-compressed MessagePack: подвыборка PointCloud2 до 10k точек, `{n_points, frame_id, fields: ["x","y","z","intensity"], points: [[x,y,z,i], ...]}` |
| `robot_status` | `0x1201` | MessagePack `{battery_pct, wifi_rssi, mode, vel_linear, vel_angular, ts_ms}` — 1 Hz |
| `voice_state` | `0x1202` | MessagePack `{state: "idle"\|"listening"\|"thinking"\|"speaking", ts_ms, utterance_id?}` — event-driven |

**Frequency policy:**

- `camera_rear`: до 30 fps (CBR), quality=high; 15 fps при quality=med;
  10 fps при quality=low.
- `lidar_2d`: 10 Hz всегда (LiDAR физически столько даёт).
- `lidar_3d`: 2 Hz (тяжёлый, клиент может unsubscribe если не нужен).
- `robot_status`: 1 Hz всегда (пока не unsubscribe).
- `voice_state`: event-driven (только при смене состояния).

**Backpressure:** если клиент не успевает читать, сервер применяет
`drop-oldest` политику для потоковых топиков (camera, lidar) — лучше
потерять кадр, чем копить очередь в RAM. Для `robot_status` и `voice_state`
— `drop-newest` (последний статус всегда интереснее).

## 5. `JSON_CMD` — client → server

```json
{
  "cmd": "teleop_twist",
  "ts_ms": 1234567890,
  "seq": 12345,
  "linear":  {"x": 0.5, "y": 0.0, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.3},
  "deadman": true
}
```

`deadman=true` ОБЯЗАН быть на каждом teleop-фрейме; если `false` —
сервер игнорирует фрейм (это страховка от «отпустил grip, но пакет
застрял в TCP буфере и пришёл позже»). Throttle: не чаще 30 Гц
(`seq` монотонный, сервер отбрасывает фреймы с повторным `seq`).

```json
{
  "cmd": "ui_button",
  "ts_ms": 1234567890,
  "button": "sound_play" | "sound_stop" | "light_toggle" | "led_preset:<name>",
  "press": true | false
}
```

Маппинг `button` → ROS-сервис: phase-1 — hardcoded dict в `rob_box_quest`,
phase-2 — registry из `rob_box_voice/command_node.py`.

```json
{
  "cmd": "voice_mode",
  "ts_ms": 1234567890,
  "mode": "off" | "passthrough" | "ttts_proxy" | "stt_llm"
}
```

Переключает параметр `voice_input_mode` в `dialogue_node` через
`set_parameters` (атомарно). Сервер отвечает `JSON_EVENT{type: "voice_mode_ack", mode: ...}`.

```json
{
  "cmd": "voice_ptt",
  "ts_ms": 1234567890,
  "state": "start" | "stop"
}
```

Edge-triggered. Сервер публикует в `/audio/quest_in` только пока
`state=start`. Phase-2.

```json
{
  "cmd": "stop_emergency",
  "ts_ms": 1234567890,
  "source": "controller_b" | "ui_button" | "client_lost"
}
```

Публикует в `/safety/emergency_stop`. `source=client_lost` отправляется
сервером автоматически через Wi-Fi watchdog (§3.3 ADR-0027).

## 6. `JSON_EVENT` — server → client (control/notification)

```json
{ "type": "voice_mode_ack", "mode": "stt_llm", "ts_ms": 1234567890 }
{ "type": "safety_stop",    "reason": "controller_b" | "client_lost", "ts_ms": 1234567890 }
{ "type": "robot_alert",    "level": "warn" | "error", "code": "BATTERY_LOW", "args": {"pct": 12}, "ts_ms": 1234567890 }
{ "type": "subscribe_ack",  "topic": "camera_rear", "stream_id": 0x1001, "quality": "med" }
{ "type": "subscribe_nack", "topic": "lidar_3d", "reason": "topic_not_available_yet" }
{ "type": "heartbeat",      "ts_ms": 1234567890 }   // каждые 200 мс, см. §7
{ "type": "voice_state",    "state": "speaking", "ts_ms": 1234567890, "utterance_id": "..." }
{ "type": "ping",           "ts_ms": 1234567890, "nonce": "..." }   // см. §7
{ "type": "pong",           "ts_ms": 1234567890, "nonce": "..." }
```

`robot_alert` codes (Phase 1): `BATTERY_LOW (<20%)`, `WIFI_WEAK (<-75 dBm)`,
`ROBOT_STUCK (3 с нет cmd_vel)`, `OVER_TEMP`. Phase 3 — `KIDNAPPED` (twist_mux
рапорт о внезапном отсутствии contact с роботом), `LIDAR_TIMEOUT`.

## 7. Heartbeat, latency, reconnect

- **Server → client heartbeat:** каждые 200 мс `JSON_EVENT{type: "heartbeat",
  ts_ms: <server_time>}`. Клиент, не получивший 3 подряд (600 мс) → UI
  показывает «CONNECTION LOST», стик подсвечивается красным, grip
  автоматически отпускается (dead-man fail-safe).
- **Client → server ping:** раз в 5 с клиент шлёт `JSON_EVENT{type: "ping",
  nonce}`; сервер отвечает `JSON_EVENT{type: "pong", nonce}`. RTT
  выводится в UI как «Wi-Fi RTT: 23 ms».
- **Reconnect strategy:** клиент при разрыве пытается переподключиться
  exponential backoff (1 с → 30 с). При успехе — `HELLO` с тем же PIN
  (PIN ротируется только при перезапуске контейнера, не при разрыве).

## 8. Error codes (`ERROR`-frame)

| Code | Когда | Что клиент делает |
|---|---|---|
| `AUTH_FAIL` | Неверный PIN | показать «Wrong PIN, ask operator for current one» |
| `BAD_PAYLOAD` | payload не парсится / не проходит schema | drop frame, лог в UI-console |
| `RATE_LIMIT` | teleop_twist чаще 30 Hz | drop frame, throttle на клиенте |
| `TOPIC_UNKNOWN` | subscribe на topic, которого нет в registry | показать «Topic not available» |
| `TOPIC_NOT_AVAILABLE_YET` | source-данные ещё не пришли (lidar не запущен) | автоматический retry через 1 с |
| `INTERNAL` | неожиданная ошибка сервера | UI показывает «Server error, reconnect» |
| `PROTOCOL_VERSION` | subprotocol не `robbox-quest-v1` | close socket, UI «Update client» |

## 9. Rate limits (Phase 1)

| Frame | Max rate | Action on overflow |
|---|---|---|
| `teleop_twist` | 30 Hz (per client) | drop + `ERROR{RATE_LIMIT}` |
| `ui_button` | 5 Hz | drop |
| `voice_ptt` | edge-triggered, max 2 start/s | drop |
| `voice_mode` | 1 per 5 s | drop |
| `stop_emergency` | 1 per 100 ms | drop |
| `ping` | 1 per 5 s | drop |

Server-side enforcement — token bucket per client; reset при reconnect.

## 10. Пример полного handshake (Phase 1, MVP)

```
[t=0]      client → server   HELLO  {client_version: "0.1.0", capabilities: ["webxr"], session_pin: "123456"}
[t=10ms]   server → client   WELCOME  {server_version: "0.1.0", session_id: "...", robot_status: {battery_pct: 87, mode: "idle"}}
[t=15ms]   client → server   SUBSCRIBE  {topic: "camera_rear", quality: "med"}
[t=16ms]   server → client   subscribe_ack  {topic: "camera_rear", stream_id: 0x1001, quality: "med"}
[t=16ms]   client → server   SUBSCRIBE  {topic: "lidar_2d"}
[t=17ms]   server → client   subscribe_ack  {topic: "lidar_2d", stream_id: 0x1101}
[t=17ms]   client → server   SUBSCRIBE  {topic: "robot_status"}
[t=18ms]   server → client   subscribe_ack  {topic: "robot_status", stream_id: 0x1201}
[t=33ms]   server → client   BINARY_FRAME  stream_id=0x1001  topic_id=0x1001  [H.264 NAL: SPS]
[t=33ms]   server → client   BINARY_FRAME  stream_id=0x1001  topic_id=0x1001  [H.264 NAL: PPS]
[t=66ms]   server → client   BINARY_FRAME  stream_id=0x1001  topic_id=0x1001  [H.264 NAL: IDR]
[t=99ms]   server → client   BINARY_FRAME  stream_id=0x1001  topic_id=0x1001  [H.264 NAL: non-IDR]
...
[t=100ms]  client → server   JSON_CMD  {cmd: "teleop_twist", linear: {x: 0.5}, angular: {z: 0.0}, deadman: true, seq: 1}
[t=200ms]  server → client   heartbeat  {ts_ms: ...}
[t=200ms]  client → server   JSON_CMD  {cmd: "teleop_twist", linear: {x: 0.5}, angular: {z: 0.0}, deadman: true, seq: 2}
...
[t=5000ms] client → server   JSON_EVENT  {type: "ping", nonce: "abc"}
[t=5003ms] server → client   JSON_EVENT  {type: "pong", nonce: "abc"}
```

## 11. Совместимость и версионирование

- `server_version` / `client_version` — semver.
- Negotiation: если `client_version > server_version` (major mismatch) →
  close с `ERROR{PROTOCOL_VERSION}`.
- Если `client_version < server_version` (minor mismatch) → `WELCOME`
  с предупреждением; несовместимые поля просто игнорируются клиентом.
- Любое изменение **формата payload** = major bump → новый subprotocol
  (`robbox-quest-v2`), Caddy маршрутизирует на другой endpoint.

## 12. Что НЕ в Phase 1 (out of scope для дизайна, отложено)

- mTLS / OAuth / TOTP (Phase 3, см. ADR-0027 §6 Q3).
- Multi-user arbitration (Phase 3, ADR-0027 §6 Q2).
- 3D-pointcloud streaming на 30 Hz (Phase 3, лень клиента + bandwidth).
- Hand-tracking pinch-grab для AR-объектов (Phase 2 опц., ADR-0027 §6 Q5).
- Spatial audio с HRTF (Phase 3, ADR-0027 §6 Q6).
- Replay/logging — записи сессий для отладки (Phase 3).