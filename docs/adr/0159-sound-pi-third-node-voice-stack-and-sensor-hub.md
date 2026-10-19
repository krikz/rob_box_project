# ADR-0159: Sound Pi — третий узел робота: голосовой стек и ReSpeaker уезжают с Vision Pi, нода сенсорного хаба ESP32 живёт там же

| Поле | Значение |
|---|---|
| Статус | **Proposed** — решения товарища Шифу по В1–В7 (§8) ещё не получены; §2 — предложение координатора |
| Дата | 2026-10-19 |
| Автор | Claude Code (координатор), по словам товарища Шифу 19.10 |
| Issue | эпик — завести после решений §8 |
| Код сверен | `origin/develop` @ `ca413e652` (19.10) |
| Слова Шифу (19.10) | «я добавил ещё одну распберри в робота — SoundPi, 10.1.1.22 … она пока работает через вайфай, надо настроить изернет так же, как на других … вижен пи уже был сильно загружен … переключу микрофон-аррей респикер на неё, и жирный войс-ассистент перейдёт туда … физический лед-матрикс останется на вижен пи … распределить нагрузку равномерно … как менять сиайсиди и все контейнеры … к саунд пи подкинуть сенсор-борт по TTL-конвертору через USB, так что нода, что отвечает за коммуникацию с ней, тоже туда … мы от micro-ROS отказались, нода будет просто нода, которая коммуницирует с железкой» |
| Родители | ADR-0111 (zram и лимиты контейнеров Vision Pi), ADR-0122 (voice-assistant 5g/6g — «долгосрочно вынести тяжёлые ноды», §3.1), ADR-0013 (инкрементальные PR), ADR-0018 (честность), ADR-0148 (код решает) |
| Что меняет | добавляет третий compose-проект `docker/sound` и роль `sound` в `scripts/setup/setup_node.sh`; переносит `voice-assistant`, `supercollider`, `voice-action-server` с Vision Pi на Sound Pi; добавляет третий Zenoh-роутер; расширяет манифест сборки, build/deploy workflow и E2E-скрипты на третий хост; заводит ноду `sensor_hub` (serial ↔ ESP32) на Sound Pi; хоронит micro-ROS в документации и меню setup |

---

## 0. Что проверено, что нет

- **Проверено 19.10 (raw — §1.1–§1.3):**
  - SoundPi по SSH (`plink`, пароль `open`, user `ros2`): модель, ОС, память, диск, сеть, netplan, `config.txt`, USB, `/dev/snd`, `/dev/ttyAMA*`, группы пользователя, отсутствие Docker;
  - Vision Pi: `docker stats`, `top`, `free`, `lsusb`, оба netplan-файла, маршруты, `IpcMode`/`Memory` контейнера `voice-assistant`;
  - Main Pi: `ip`, netplan, `docker stats`, `free`, `/dev/ttyACM*`;
  - код на `develop @ ca413e652`: compose обоих Pi, zenoh-конфиги, `start_voice_assistant.sh` и headless-launch (10 нод), `asound.conf`, `led_node.py` (USB-кольцо ReSpeaker), `build-manifest.yaml`, `L-Build Vision Pi Services.yml`, `L-Build Single Service.yml`, `L-Deploy and Verify.yml`, `check_container_status.sh`, `L-E2E Voice Test.yml`, agent-flow/E2E/hotpatch скрипты с адресами робота, `setup_node.sh`/`setup_main_pi.sh`/`setup_vision_pi.sh`, история micro-ROS в git.
- **Не проверено:**
  - пропускная способность и задержка Zenoh между тремя роутерами по гигабиту — не мерено (Sound Pi без кабеля, стека на нём нет);
  - реальный RSS голосового стека на Sound Pi после переезда — только лимиты compose и текущий `docker stats` на Vision Pi;
  - протокол ESP32 ↔ хост по serial (формат кадров, скорость) — прошивки сенсорного хаба в этом репо нет (`src/sensor_hub` из `setup_node.sh` не существует), есть только сообщения `robot_sensor_hub_msg` (`DeviceData`, `DeviceSnapshot`, `DeviceCommand`);
  - как ТАРС (`supervisor`) аутентифицируется в `/mcp/execute`, когда `mcp_server` на другом хосте (§3.6, В5);
  - `ollama` (profile `ai`, Vision Pi) — используется ли VoiceMemory сегодня вообще (в `docker ps` Vision Pi 19.10 контейнера `ollama` нет);
  - ничего из §5 на роботе не запускалось; `setup_sound_pi.sh` на SoundPi **не прогонялся** — только `bash -n`.

---

## 1. Проблема

### 1.1 Vision Pi перегружен (замер 19.10, 10:18–10:20 UTC+3, uptime 20 мин, 4 ядра, 8 ГБ)

```
load average: 7.59, 4.79, 4.48            ← 4 ядра
Mem: 7.8Gi total, 4.6Gi used, 3.1Gi available, swap 1.9Gi (0 used)
%Cpu(s): 33.3 us, 11.8 sy, 52.9 id
```

`docker stats --no-stream` (Vision Pi):

| Контейнер | CPU % | MEM | лимит |
|---|---|---|---|
| oak-d | 64.35 | 485 MiB | 6 GiB |
| vision-face | 40.63 | 314 MiB | 1 GiB |
| **voice-assistant** | **32.87** | **2.29 GiB** | **5 GiB** |
| zenoh-router-vision | 29.96 | 57 MiB | 128 MiB |
| vision-hailo | 21.79 | 288 MiB | 1 GiB |
| rob-box-quest | 20.78 | 237 MiB | 512 MiB |
| led-matrix | 9.30 | 241 MiB | 512 MiB |
| avatar-arbiter | 6.08 | 159 MiB | 512 MiB |
| supercollider | 1.22 | 241 MiB | 512 MiB |
| ceiling-camera | 1.06 | 183 MiB | 512 MiB |
| promtail / telegram-bot / supervisor / voice-action-server / cadvisor | < 1.2 | 31–186 MiB | — |

`top`: `apriltag` 38 %, `vision_*` 31 % + 15 %, `zenohd` 23 %, `camera_*` 23 %, `mcp_server` 15 %, `quest_node` 15 %.

Голосовой стек — один контейнер на 10 нод (`audio`, `led`, `animation_player`, `dialogue`, `tts`, `stt`, `sound`, `command`, `mcp_server`, `speaker_id`) с `mem_limit: 5g`, то есть **5 из 8 ГБ узла зарезервированы под один cgroup** (ADR-0122). ADR-0122 §3.1 прямо откладывал «вынести тяжёлые ноды» как backend-работу. Шифу решил иначе: не резать контейнер, а дать ему **свой узел**.

Для сравнения Main Pi (16 ГБ, 4 ядра): `load 3.99`, `2.8Gi used / 12Gi available`; `perception` 76 %, `nav2` 59 %, `rtabmap` 40 %, `zenoh-router` 26 %. Он тоже не пустой по CPU, но память свободна.

### 1.2 Что такое SoundPi сегодня (замер 19.10)

```
hostname SoundPi, user ros2 (sudo, dialout, audio, spi, i2c, gpio, plugdev)
Raspberry Pi 5 Model B Rev 1.0, 4 cores, 7.8Gi RAM, swap 0
Ubuntu 24.04.5 LTS, kernel 6.8.0-1064-raspi
mmcblk0 232.2G (SD), / 228G, 2.4G used
eth0 DOWN (кабеля нет); wlan0 UP 10.1.1.22/24 metric 600, MAC 2C:CF:67:18:5B:3D
default via 10.1.1.1 dev wlan0 metric 600
/etc/netplan/50-cloud-init.yaml — только wlan0 (hidden SSID "ROS2", PSK хэшем, regulatory-domain RU)
config.txt: enable_uart=1, dtoverlay=dwc2, camera_auto_detect=1 — usb_max_current_enable НЕТ
lsusb: только root-хабы; /dev/snd: controlC0/1, pcmC0D0p, pcmC1D0p (HDMI-выходы), /dev/ttyAMA0, /dev/ttyAMA10
docker: command not found; git, curl есть; ~/.ssh/authorized_keys есть (ключ этой рабочей станции в нём нет)
```

Эталон сети — Vision Pi: два netplan-файла, `01-netcfg.yaml` (eth0 metric 100 + wlan0 metric 600, пароль открытым текстом) и cloud-init'овский `50-cloud-init.yaml` (wlan0, hidden SSID); маршруты `default via 10.1.1.1 dev eth0 … src 10.1.1.11 metric 100` и `… dev wlan0 … src 10.1.1.21 metric 600`. Main Pi — то же (`10.1.1.10` / `10.1.1.20`). Схема адресов: Main `.10/.20`, Vision `.11/.21`, значит Sound — **`.12/.22`**; `.22` wlan0 уже получил от роутера сам.

### 1.3 Что привязано к ReSpeaker физически, а что — только топиками

ReSpeaker 4 Mic Array (`2886:0018`, ALSA card `ArrayUAC10`) — **и микрофон, и выход на колонки** (`asound.conf`: `dmix_respeaker` → `hw:CARD=ArrayUAC10`, 16 кГц, 2 канала). К нему привязаны **устройством**:

| Сервис | Почему привязан | Разделяет с кем |
|---|---|---|
| `voice-assistant` | `/dev/snd` (захват + TTS), `/dev/bus/usb` (`led_node` крутит 12 LED кольца через pyusb), `ipc: host` (dmix SHM) | `supercollider`: `ipc: host`, volume `sc_synthdefs`, `/opt/rob_box/samples`, `./data/voice/music_library` |
| `supercollider` | `/dev/snd` (scsynth → dmix), RT-приоритеты, `shm_size 256m` | `voice-assistant` |
| `voice-action-server` | тот же образ; `depends_on` voice-assistant | — |

Всё остальное общается с голосовым стеком **ROS-топиками через Zenoh** и переезжать не должно: `led-matrix` (SPI `/dev/spidev0.0`, слушает `/panel_image`, `/led_matrix/data`; издатель — `animation_player` внутри voice-assistant → топик, не устройство), `supervisor`/`avatar-arbiter` (`/voice/*`, `/dialogue/control`, `/avatar/state`), `quest` (рация и пайплайн грипа — звук приходит топиками с Vision Pi и уходит в динамики через `tts_node`/`sound_node`), `telegram-bot`.

Хостовые ресурсы, которые стек ждёт по фиксированным путям: `/opt/rob_box/{models,samples,scores}` (ресурсный пак, кладёт деплой), `docker/vision/cache/tts`, `docker/vision/data/voice` (`speakers.db`, `voice_memory.db`, `music_library/`, `.mcp_token`), `../../sound_pack`, `../../migrations`, `../../local_test/RENARDO_REFERENCE.md`.

### 1.4 Сенсорный хаб ESP32: где он сегодня

- `micro-ros-agent` был добавлен в compose Main Pi (`bf6744fe6`) и **удалён** (`fd8d95dfb`, «CR-06 part 1»); в манифесте сборки его нет, `test_service_manifest_sync.py:360` фиксирует «micro-ros-agent не собирается вовсе». Остатки: `robbox_aliases.sh:45` (`rblog-agent`), `setup_node.sh` (`setup_main_node` обещает micro-ROS agent; `setup_sensor_hub` шлёт в несуществующий `src/sensor_hub`), `docs/architecture/HARDWARE.md:104,111,171` («Micro-ROS Agent», `/dev/ttyUSB0`).
- Потребитель данных есть, но закомментирован: `context_aggregator_node.py:170–175` (`/device/snapshot`, `DeviceSnapshot`).
- Сообщения есть: `src/robot_sensor_hub_msg/msg/{DeviceData,DeviceSnapshot,DeviceCommand}.msg`.
- Решение Шифу 19.10: **micro-ROS не будет**; нода — обычная ROS-нода, читающая serial.

### 1.5 Где CI/CD и скрипты знают «ровно два Pi»

| Место | Как знает |
|---|---|
| `docker/build-manifest.yaml` | `pis: {vision, main}`; `voice-base`, `voice-assistant`, `supercollider` — под `vision` |
| `L-Build Vision Pi Services.yml` / `L-Build Main Pi Services.yml` | по workflow на Pi, матрица из манифеста (`gen_build_matrix.py`); `L-Build-All-Services.yml` вызывает оба |
| `L-Build Single Service.yml` | `pi_type: [main, vision]`, `docker/${PI_TYPE}/.image-versions.${TAG}`, ветки `ci/image-versions-{pi_type}` |
| `L-Deploy and Verify.yml` | `vision_pi_ip=10.1.1.11`, `main_pi_ip=10.1.1.10` в `case environment`; ~20 шагов `[Vision Pi] …` / `[Main Pi] …` копипастой; ресурсный пак (`apply_resource_pack.sh --only …models/samples/scores`) гоняется **на Vision Pi** |
| `.github/scripts/check_container_status.sh` | уже параметризован `PI_HOST`/`PI_COMPOSE_DIR`/`SCOPE` — третий вызов дёшев |
| `G-Validate Docker Compose.yml` | два пути compose захардкожены (`paths:` и два job'а) |
| `L-E2E Voice Test.yml` | `ros2@10.1.1.21` в 5 местах (190, 207, 209, 376, 478) — это **хост voice-assistant** |
| `agent-flow-e2e-process.sh:254` (`E2E_ROBOT_HOST=10.1.1.21`), `padavan-step4-voice-smoke.sh:77`, `watchdog-tts.sh:32`, `robot_dev/hotpatch.sh:17` (`ROBOT=ros2@10.1.1.21`), `agent-flow-deploy-sweep.sh:47–48` | дефолты «voice-assistant живёт на 10.1.1.21» |
| `docker/monitoring/NODE_MAPPING.md`, Grafana-дэшборды | «Main Pi + Vision Pi» |
| `scripts/maintenance/robbox_aliases.sh:52–53` | `rbstop-all`/`rbstart-all` по двум каталогам |

---

## 2. Решение

### 2.1 Три узла, роли по устройствам, а не по «тяжести»

| Узел | Адреса | Что держит | Почему |
|---|---|---|---|
| **Main Pi** (16 ГБ) | `.10/.20` | как сейчас: `zenoh-router` (корень, выход в облако), `nav2`, `rtabmap`, `perception`, `ros2-control`, `lslidar`, `twist-mux`, `teleop`, `robot-state-publisher` | движение и карта — не трогаем, CPU уже ~4 на 4 ядра |
| **Vision Pi** (8 ГБ) | `.11/.21` | `zenoh-router-vision`, `oak-d`, `vision-hailo`, `vision-face`, `ceiling-camera`, **`led-matrix`** (SPI), `quest`, `supervisor`, `avatar-arbiter`, `telegram-bot`, `ollama` (profile) | камеры и Hailo — PCIe/USB здесь; LED — SPI здесь |
| **Sound Pi** (8 ГБ) | `.12/.22` | `zenoh-router-sound`, **`voice-assistant`**, **`supercollider`**, **`voice-action-server`**, **`sensor-hub`** (новая нода), `cadvisor`, `promtail` | ReSpeaker (вход+выход звука, LED-кольцо) и TTL-USB конвертер — USB здесь |

Ожидаемый эффект на Vision Pi по цифрам §1.1: минус ~33 % CPU голосового стека, минус часть ~30 % `zenohd` (трафик аудио/TTS уйдёт в другой роутер), минус 2.3 ГБ RSS и 5 ГБ резерва. Sound Pi получает лимиты 5g + 512m + 128m + 128m + ~128m (`sensor-hub`) ≈ 6 ГБ из 8 — потому zram обязателен (как ADR-0111).

**Второй шаг (В2, после замера):** если Vision Pi всё ещё в красной зоне, следующие кандидаты на переезд на Sound Pi — `quest` (21 % CPU, топики) и `telegram-bot` (STT голосовых — тоже топики). Решение по ним — отдельно, по `docker stats` после первого переезда, не в этом ADR.

### 2.2 Сеть: третий Zenoh-роутер звездой через Main Pi

Как у Vision Pi сегодня (`zenoh_router_config.json5`: `listen tcp/10.1.1.11:7447`, `connect tcp/10.1.1.10:7447#iface=eth0`):

```
zenoh-router-sound:  listen tcp/10.1.1.12:7447,  connect tcp/10.1.1.10:7447#iface=eth0
сессии нод Sound Pi: peer → tcp/10.1.1.12:7447 (копия zenoh_session_config.json5 с заменой .11 → .12)
```

Путь `audio_node` (Sound) → `supervisor` (Vision) становится двухзвенным: Sound → Main → Vision по гигабиту. **Альтернатива** (В3): второй `connect`-endpoint у роутера Sound Pi напрямую на `10.1.1.11:7447`; Zenoh-роутеры образуют граф и маршруты считают сами, но это не проверено на нашей версии (`eclipse/zenoh:1.6.2`) — включать только после замера задержки голос→ТАРС на звезде.

Netplan — ровно как на Vision Pi: `01-netcfg.yaml` с eth0 metric 100 + wlan0 metric 600, cloud-init'овский файл не трогаем. Адрес `10.1.1.12` для eth0 закрепляется в DHCP роутера по MAC eth0 (на Main/Vision это так и работает: «роутер выдаёт»). Это делает Шифу на роутере (В1).

### 2.3 Compose: `docker/sound` — перенос, не копия

Новый проект `docker/sound/docker-compose.yaml` с сервисами `zenoh-router`, `voice-assistant`, `supercollider`, `voice-action-server`, `sensor-hub`, `cadvisor`, `promtail`. Правила (память `no-two-implementations`):

- Сервисные блоки `voice-assistant`, `supercollider`, `voice-action-server` **`git mv`-ятся** из `docker/vision/docker-compose.yaml`, не копируются; из vision они удаляются в том же PR, что добавляет их в sound (иначе один `depends_on supercollider` в двух местах и два источника правды).
- Конфиги и скрипты переезжают: `docker/vision/config/{audio,voice_assistant,wake_words.yaml}` → `docker/sound/config/…`, `docker/vision/scripts/{voice_assistant,supercollider}` → `docker/sound/scripts/…`. Пути **внутри** контейнеров не меняются (`/config/voice_assistant/*.yaml`, `/etc/asound.conf`, `/models`, `/data`), чтобы ноды и ресурсный пак ничего не заметили.
- `zenoh-router/Dockerfile`, `scripts/zenoh-router/start_zenoh_router.sh`, `scripts/ros_with_namespace.sh`, `config/zenoh_session_config.json5` сегодня уже лежат **двумя копиями** (main и vision). Третья копия для sound — честно заявленный долг: один общий `docker/common/` — отдельная карточка, не этот эпик (В4).
- `ollama`: `OLLAMA_BASE_URL=http://localhost:11434` перестанет работать. Переменная становится `${OLLAMA_HOST_URL:-http://10.1.1.11:11434}` в `.env` Sound Pi; `ollama` остаётся на Vision Pi под profile `ai` (сегодня не запущен — §0).
- `.image-versions.{dev,test,latest}` для `docker/sound` — пишет CI (как для vision/main).
- Данные состояния (`data/voice/*`, `cache/tts`) — **не в git**, их переносит cutover (§3.5), compose только монтирует те же относительные пути.

### 2.4 Нода `sensor-hub`: обычная ROS-нода, serial, без micro-ROS

- Пакет `src/rob_box_sensor_hub` (rclpy + `pyserial`): открывает `/dev/sensor_hub` (udev-симлинк на TTL-USB конвертер, правило ставит `setup_node.sh` роль `sound`), парсит кадры прошивки ESP32, публикует `/device/snapshot` (`DeviceSnapshot`), слушает `/device/command` (`DeviceCommand`). Протокол кадров — **открытый вопрос В6** (прошивки в репо нет).
- Контейнер `sensor-hub` в `docker/sound`: `devices: [/dev/sensor_hub:/dev/sensor_hub]`, `mem_limit 128m`, без `privileged`. Образ — **тот же `voice-assistant`** (build_context `.` уже собирает все `src/*`; добавить `pyserial` в его Dockerfile), по образцу `voice-action-server`: ноль новых build-job'ов. Отдельный образ — только если `voice-assistant` станет узким местом сборки (В7).
- `context_aggregator_node.py:170` — раскомментировать подписку, когда нода реально публикует (`ros2 topic hz /device/snapshot` в evidence).
- Хоронится: `rblog-agent` alias, micro-ROS в `setup_node.sh` (`setup_main_node`, `setup_sensor_hub`), `HARDWARE.md:104,111,171` → «нода `sensor_hub` на Sound Pi, serial через TTL-USB».

### 2.5 CI/CD: третий Pi как данные, а не как третья копия — где это дёшево

| Что | Решение |
|---|---|
| `build-manifest.yaml` | `pis.sound: {voice-base, voice-assistant, supercollider}` — **перенос** из `pis.vision` (`git diff` покажет −/+ одних и тех же блоков); guard `test_service_manifest_sync.py` обновляется в том же PR |
| Сборка | `L-Build Sound Pi Services.yml` — копия Vision-workflow с `--pi sound` (матрица из манифеста, так что копия почти вся — обвязка). Параметризация одного workflow по `pi` — отдельная карточка (В4); `L-Build-All-Services.yml` получает третий `uses:` |
| `L-Build Single Service.yml` | `pi_type` += `sound`; `case "$PI_TYPE"` += ветка; `.image-versions` путь уже параметризован |
| `L-Deploy and Verify.yml` | `sound_pi_ip=10.1.1.12` во все `case environment`; шаги `[Sound Pi] Pull / Stop / Start / Check Container Status / Cleanup` по образцу Vision; **ресурсный пак** `--only renardo-samples,dj-dave-samples,sonicpi-samples,muldjord-kit,score-library` и STT/TTS-модели гоняются **на Sound Pi**, Hailo `.hef` — остаются на Vision Pi; `check_container_status.sh` вызывается третий раз с `SCOPE=sound` |
| `G-Validate Docker Compose.yml` | третий путь в `paths:` и третий job (или матрица по трём каталогам — предпочтительно, это и есть «данные, не копия») |
| E2E и agent-flow | дефолты хоста voice-assistant → `10.1.1.22` (wlan, как сейчас `.21`) / `.12`: `L-E2E Voice Test.yml` (5 мест), `agent-flow-e2e-process.sh:254`, `padavan-step4-voice-smoke.sh:77`, `watchdog-tts.sh:32`, `hotpatch.sh:17`; `agent-flow-deploy-sweep.sh` += `SOUND_PI_IP` |
| Мониторинг | `NODE_MAPPING.md`, Grafana «Sound Pi CPU/Memory», promtail/cadvisor из `docker/sound` с теми же labels |
| Setup | роль `sound` в `setup_node.sh` + обёртка `setup_sound_pi.sh` — **сделано в этом PR** (§3.1); деплой-шаг «Ensure systemd unit» для Vision (`reinstall_vision_pi_systemd.sh`) на Sound не копируем, пока `robbox-sound.service` не понадобится в CI |

---

## 3. Детализация

### 3.1 Что уже в этом PR: `setup_node.sh` роль `sound`, `setup_sound_pi.sh`

- `scripts/setup/setup_node.sh`: пункт меню 6 / аргумент `sound`; параметры `NODE_IP=10.1.1.12`, `DOCKER_COMPOSE_DIR=docker/sound`, `SERVICE_NAME=robbox-sound`; `setup_sound_node()` → `setup_sound_network()` (пишет `/etc/netplan/01-netcfg.yaml` как на Vision Pi, `netplan apply`, печатает маршруты), `setup_sound_boot_config()` (`usb_max_current_enable=1`, `enable_uart=1`), `setup_sound_udev()` (`/dev/sensor_hub` для CP210x/CH340/FTDI/PL2303), `alsa-utils`, каталоги `/opt/rob_box/{models,samples,scores}`, `setup_zram_swap`. Всё идемпотентно.
- `scripts/setup/setup_sound_pi.sh`: обёртка как `setup_main_pi.sh` (`bash setup_node.sh sound`).
- Известное ограничение: `setup_autostart` создаст `robbox-sound.service` с `WorkingDirectory=~/rob_box_project/docker/sound`, которого до PR-2 нет — юнит включён, но при загрузке упадёт `compose up` (нет файла). Это допустимо до PR-2 (на Sound Pi нечего поднимать), в evidence PR-1 это надо показать честно.
- Найденная по пути несогласованность, не чиним здесь: роль `sensor-hub` в `setup_node.sh` имеет `NODE_IP=10.1.1.20` — это wlan0 Main Pi.

### 3.2 Zenoh-конфиги Sound Pi

`docker/sound/config/zenoh_router_config.json5` = vision-файл с `listen tcp/10.1.1.12:7447`, `connect tcp/10.1.1.10:7447#iface=eth0`. `zenoh_session_config.json5` = vision-файл с `connect tcp/10.1.1.12:7447`. `scouting.multicast.enabled: false` сохраняется — три роутера видят друг друга только явными endpoint'ами, никакого автодискавери по WiFi.

### 3.3 Порядок старта и healthcheck

`voice-assistant` зависит от `zenoh-router` (healthy) и `supercollider` (healthy) — оба на том же узле, зависимость сохраняется. Межузловые зависимости (ТАРС ждёт `/voice/*`) и сегодня не выражены compose'ом — ноды ретраят подключение к роутеру (`ZENOH_ROUTER_CHECK_ATTEMPTS=10`), ничего нового.

### 3.4 Деплой: порядок узлов

Сегодня: Vision → Main. Станет: Vision → Sound → Main (голосовой стек не должен подняться раньше роутера Main Pi? — нет: роутеры реконнектятся, порядок важен только для `check_container_status` grace-периодов). `Stop Containers` — на всех трёх параллельно, как сейчас cleanup.

### 3.5 Cutover на роботе (ручной, с Шифу, один раз)

1. `setup_sound_pi.sh` на SoundPi; кабель eth0; DHCP-резерв `.12`; `reboot` (config.txt).
2. Деплой PR-2…PR-4 **без** переключения ReSpeaker: Sound Pi поднимает только `zenoh-router`, `sensor-hub` (если ESP32 уже есть); voice-стек ещё на Vision — **нет**, так не выйдет: PR-2 переносит сервисы из vision-compose. Поэтому cutover атомарный: мёрж PR-2…PR-4 → `Build All` → **стоп** стека Vision Pi → `rsync -a` `docker/vision/data/voice/` и `docker/vision/cache/tts/` на Sound Pi в `docker/sound/…` (это `speakers.db`, `voice_memory.db`, `music_library`, `.mcp_token` — живые данные людей, память `robot-data-is-shared-with-live-people`) → переткнуть ReSpeaker и колонки → `Deploy and Verify`.
3. Evidence: `docker stats` на всех трёх; `ros2 topic hz /voice/stt/result` из supervisor-контейнера на Vision (сквозной путь Sound→Main→Vision); живая реплика «Робот, …» (память `live-check-by-topic-injection`) с ответом в динамики; LED-матрица реагирует на анимацию (`/panel_image` приходит с другого узла).

### 3.6 Что может сломаться при разнесении по хостам

| Риск | Где | Что делать |
|---|---|---|
| `/mcp/execute` аутентификация: токен лежит в `/data/.mcp_token` voice-assistant; `supervisor` на Vision Pi его файла не видит | `docker-compose.yaml:303–308` («Set ROB_BOX_MCP_TOKEN here instead if the nodes are ever split») | выяснить, как ТАРС зовёт тулы сегодня (В5); если через `/mcp/execute` с токеном — переносить токен в `.env.secrets` обоих узлов |
| `OLLAMA_BASE_URL=localhost` | §2.3 | env-переменная на хост Vision |
| `ipc: host` dmix между voice-assistant и supercollider | оба на Sound Pi — сохраняется | — |
| E2E/хотпатч ходят на `.21` за логами voice-assistant | §1.5 | PR-5 |
| Ресурсный пак кладётся на Vision Pi | deploy workflow | PR-4: models/samples/scores → Sound, hef → Vision |
| Второй hop для рации Quest → динамики | §2.2 | замер; при лаге — прямой endpoint Sound↔Vision |
| `promtail` labels/`job` одинаковые на трёх узлах | monitoring | hostname в labels уже есть у cadvisor; проверить promtail-конфиг |
| 5 ГБ voice + 0.5 sc на 8 ГБ без свопа | ADR-0111 | zram в `setup_sound_node` |

---

## 4. Приёмка

- `setup_sound_pi.sh` прогнан на SoundPi: raw `ip route` с двумя default (eth0 100, wlan0 600), `ls -l /dev/sensor_hub`, `grep usb_max_current /boot/firmware/config.txt`, `swapon --show`.
- `Build All` зелёный с третьим workflow; `docker/sound/.image-versions.dev` обновлён CI (`gh run view <id>`).
- `Deploy and Verify` зелёный на трёх узлах; `check_container_status` `SCOPE=sound` healthy.
- `docker stats` Vision Pi после переезда: нет `voice-assistant`/`supercollider`; CPU idle вырос против §1.1 — приложить обе таблицы.
- Сквозной голос: реплика в ReSpeaker (Sound) → ответ TTS в динамики (Sound) → анимация на LED (Vision) → запись в журнал ТАРС (Vision). `docker logs voice-assistant` (Sound) и `docker logs avatar-supervisor` (Vision) — последние 30 строк каждого.
- `ros2 topic hz /device/snapshot` > 0 из `perception` на Main Pi — только когда ESP32 подключён (иначе пункт переносится в карточку ноды).
- `grep -rn "micro-ros\|micro_ros\|microros" docker scripts docs/architecture src` — пусто, кроме истории в ADR и `test_service_manifest_sync.py` (он охраняет отсутствие).

## 5. План PR (каждый ≤ ~600 строк, ADR-0013)

| # | Содержимое | Зависит от |
|---|---|---|
| PR-1 (этот) | ADR-0159; `setup_node.sh` роль `sound`; `setup_sound_pi.sh` | — |
| PR-2 | `docker/sound/` (compose + config + scripts, `git mv` из vision); удаление трёх сервисов из `docker/vision/docker-compose.yaml`; `build-manifest.yaml` `pis.sound`; guard-тест манифеста; `G-Validate Docker Compose` третий путь | PR-1 |
| PR-3 | `L-Build Sound Pi Services.yml`; `L-Build-All-Services.yml` третий `uses:`; `L-Build Single Service.yml` `pi_type=sound` | PR-2 |
| PR-4 | `L-Deploy and Verify.yml`: `sound_pi_ip`, шаги Sound Pi, ресурсный пак на Sound; `agent-flow-deploy-sweep.sh` | PR-3 |
| PR-5 | E2E/agent-flow/hotpatch/watchdog дефолты хоста; `robbox_aliases.sh` (`rbstop-all` третий каталог, удалить `rblog-agent`); `NODE_MAPPING.md`, Grafana | PR-4 |
| PR-6 | `src/rob_box_sensor_hub` + сервис `sensor-hub` в `docker/sound`; `pyserial` в Dockerfile voice-assistant; `context_aggregator` подписка; похороны micro-ROS в `setup_node.sh`/`HARDWARE.md` | PR-2, протокол В6 |
| cutover | §3.5 с Шифу | PR-2…PR-5 влиты и собраны |

PR-2…PR-4 **мёржатся подряд перед одной сборкой**: после PR-2 compose Vision уже без голоса, а деплой ещё не знает Sound Pi — между ними робот без голоса. Либо держать их одной веткой-стеком и влить за один вечер (память `stacked-pr-breaks-after-squash`: пушить под новыми именами после сквоша нижнего).

## 6. Альтернативы

- **Резать `voice-assistant` на контейнеры на Vision Pi (ADR-0122 §3.1).** Не снимает 5 ГБ с узла и не разгружает CPU — переставляет мебель в той же комнате. Отклонено Шифу покупкой третьего Pi.
- **Перенести голос на Main Pi (16 ГБ).** Память есть, CPU — нет (`load 4`, `perception` 76 %); Main — узел движения, OOM/лаг там дороже, чем на Vision. Отклонено.
- **Один «общий» compose на три узла с профилями.** Один файл, три `docker compose --profile`; но `.image-versions`, deploy-шаги и guard'ы заточены на `docker/<pi>/` — миграция дороже третьего каталога. Отклонено сейчас; дубли `zenoh-router`/`ros_with_namespace.sh` вынести в `docker/common/` отдельной карточкой (В4).
- **micro-ROS agent для ESP32.** Отклонено Шифу 19.10: «нода будет просто нода». Плюс: нет ещё одного рантайма и образа; минус: протокол кадров — наш, его надо задокументировать (В6).
- **Sound Pi роутер напрямую к Vision Pi (mesh).** См. §2.2 — после замера.

## 7. Что удалить (с grep-доказательством, по PR)

- PR-2: три сервисных блока и их конфиги из `docker/vision/` — `grep -n "voice-assistant\|supercollider\|voice-action-server" docker/vision/docker-compose.yaml` → только комментарии-ссылки или пусто; `ls docker/vision/config/voice_assistant` → нет.
- PR-5: `alias rblog-agent` (`robbox_aliases.sh:45`).
- PR-6: micro-ROS из `setup_node.sh` (`setup_main_node`, `setup_sensor_hub`), `HARDWARE.md:104,111,171`; `grep -rn -i "micro.ros" scripts docs/architecture docker` → пусто.

## 8. Решения товарища Шифу и открытые вопросы

| # | Вопрос | Предложение координатора | Решение Шифу |
|---|---|---|---|
| В1 | Адреса Sound Pi `10.1.1.12` (eth0) / `.22` (wlan0), резерв `.12` по MAC на роутере | да, продолжить ряд .10/.11/.12 | — |
| В2 | Переезжают ли во второй волне `quest` и `telegram-bot` | решать по `docker stats` после первого переезда | — |
| В3 | Топология Zenoh: звезда через Main (как Vision) или + прямой линк Sound↔Vision | звезда; прямой линк после замера | — |
| В4 | Дубли `zenoh-router`/`ros_with_namespace.sh`/`zenoh_session_config.json5` ×3 и три build-workflow | принять долг, карточка «`docker/common` + workflow по `pi`» | — |
| В5 | Как ТАРС зовёт тулы `mcp_server` через хосты (`.mcp_token`) | выяснить до PR-2; если нужно — токен в `.env.secrets` обоих узлов | — |
| В6 | Протокол ESP32 ↔ хост по serial (кадр, скорость, где прошивка) | Шифу даёт прошивку/описание; нода пишется по ней | — |
| В7 | Образ для `sensor-hub`: тот же `voice-assistant` или свой | тот же, по образцу `voice-action-server` | — |

## 9. Что не проверено

См. §0. Дополнительно: ничего из §5 кроме PR-1 не написано; `setup_sound_pi.sh` на SoundPi не запускался (только `bash -n`); цифры §1.1 — один снимок через 20 минут после загрузки Vision Pi, не среднее за день.
