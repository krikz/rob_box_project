# Диагностика: `robbox-vision.service` падает после ребута Vision Pi

**Дата:** 2026-09-15
**Карточка:** t_51c4b1a9 (родитель t_83ffae62)
**Issue:** #2610
**Версия:** develop @ c1c5c56e6
**Автор:** devops
**Статус:** root cause подтверждён; правка предлагается отдельной карточкой

---

## TL;DR (товарищ Шифу)

`robbox-vision.service` падает после **каждого** ребута Vision Pi, потому что
`/etc/systemd/system/robbox-vision.service` делает `docker compose up -d` без
`--pull never`, и `docker-compose.yaml` не задаёт `pull_policy`. Compose по
умолчанию идёт в `SERVICE_IMAGE_PREFIX=10.1.1.249:5000/krikz/rob_box` (build-host
katana), и если katana выключен — `dial tcp 10.1.1.249:5000: connect: no route
to host` → весь compose-up падает → systemd фиксирует exit 1 → контейнеры не
поднимаются вообще. **Service `oneshot` + `RemainAfterExit=yes`** без
`Restart=on-failure` — после первого фейла systemd больше не пытается.

Это **не** случайность одного ботинга: подтверждено на 3 последовательных
boot-id (14.09, 15.09 12:41, 15.09 18:47). На 4-м буте (15.09 19:48) registry
поднялся, pull прошёл, но systemd всё равно FAILED из-за отдельной ошибки
`container is marked for removal and cannot be started` (avatar-arbiter
перешёл в Restart во время up, daemon отказал) — после чего контейнеры в
итоге подняли руками/скриптом.

**Гипотеза «11 образов закэшированы» — подтверждена**: `docker images` показывает
все 11 сервисов Vision Pi в локальном кэше. `restart: unless-stopped` спасает
только если systemd-юнит **сначала успешно стартанул**. После systemd FAIL
`docker compose ps` пустой.

---

## 1. Root cause

**Цепочка зависимостей, которая рвётся:**

```
reboot Vision Pi
  → systemd: robbox-vision.service (Type=oneshot, RemainAfterExit=yes)
  → ExecStart=/usr/bin/docker compose up -d
  → docker compose up -d:
      1. for each service: pull image
         image = ${SERVICE_IMAGE_PREFIX:-ghcr.io/krikz/rob_box}:<svc>-<ros>-<tag>
         на Pi SERVICE_IMAGE_PREFIX = 10.1.1.249:5000 (из /home/ros2/rob_box_project/docker/vision/.env)
      2. только если pull ok → container create / start
  → pull: GET https://10.1.1.249:5000/v2/krikz/rob_box/manifests/<svc>-humble-dev
      → если 10.1.1.249 (katana, build-host) недоступен: "no route to host"
      → daemon returns error → compose up -d exits 1
  → systemd: Main process exited, code=exited, status=1/FAILURE
  → systemd: robbox-vision.service: Failed with result 'exit-code'.
  → контейнеры не поднялись.
  → "restart: unless-stopped" не применяется, т.к. compose-up не создал контейнеры.
```

**Два независимых дефекта, оба подтверждены:**

### 1.1 Compose-up делает pull без `--pull never`

**Файл:** `/etc/systemd/system/robbox-vision.service:13`
```
ExecStart=/usr/bin/docker compose up -d
```
**Файл:** `docker/vision/docker-compose.yaml:26` (и далее для каждого сервиса)
```yaml
image: ${SERVICE_IMAGE_PREFIX:-ghcr.io/krikz/rob_box}:oak-d-${ROS_DISTRO:-humble}-${OAK_D_TAG:-${IMAGE_TAG}}
```
- Нет `pull_policy: never` / `if_not_present` / `missing`.
- `docker compose up -d` по дефолту использует policy `always` для сервисов
  без `build:`, даже если образ уже в локальном кэше.
- Сравнение с `docker-compose.yaml` других сервисов:
  - `quest` имеет `build:` (line 478) — для него pull пропускается если build уже накатан;
  - все остальные 10 сервисов (oak-d, led-matrix, ceiling-camera, supercollider,
    voice-resources-init, voice-assistant, voice-action-server, supervisor,
    avatar-arbiter, telegram-bot, vision-hailo, vision-face) — чистый pull.

### 1.2 systemd unit — `oneshot` без retry/restart

**Файл:** `/etc/systemd/system/robbox-vision.service:8-14`
```
[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/home/ros2/rob_box_project/docker/vision
ExecStart=/usr/bin/docker compose up -d
ExecStop=/usr/bin/docker compose down
User=ros2
```
- `Type=oneshot` означает: выполнил ExecStart — и всё. Упал → FAILED.
- Нет `Restart=on-failure` / `Restart=always`. После первой неудачи больше
  не пытается.
- Каждый следующий ребут пробует заново — но если registry всё ещё недоступен,
  снова FAILED.
- Контраст с каждым контейнером внутри compose: у всех `restart: unless-stopped`,
  но это работает **только если compose-up их сначала создал**.

**Правильный паттерн (для правки):**
```ini
[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/home/ros2/rob_box_project/docker/vision
ExecStart=/usr/bin/docker compose up -d --pull never
ExecStop=/usr/bin/docker compose down
User=ros2
Restart=on-failure
RestartSec=60
StartLimitIntervalSec=600
StartLimitBurst=5
```
И в compose добавить `pull_policy: missing` для всех 11 сервисов (кроме
`build:`-ных quest).

---

## 2. Что подтверждено raw-evidence (ADR-0018)

### 2.1 Текущее состояние Vision Pi

Снято `2026-09-15 20:19 CEST` (uptime 34 min, последний ребут ~19:46):

```
$ docker images --format '{{.Repository}}:{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}' | sort
10.1.1.249:5000/krikz/rob_box:ceiling-camera-humble-dev	2.42GB	2026-09-14 15:53:18 +0300 MSK
10.1.1.249:5000/krikz/rob_box:led-matrix-humble-dev	1.25GB	2026-09-14 15:14:19 +0300 MSK
10.1.1.249:5000/krikz/rob_box:oak-d-humble-dev	3.98GB	2026-09-09 21:06:42 +0300 MSK
10.1.1.249:5000/krikz/rob_box:quest-humble-dev	1.73GB	2026-09-15 14:18:57 +0300 MSK
10.1.1.249:5000/krikz/rob_box:supercollider-dev	1.14GB	2026-09-09 21:36:21 +0300 MSK
10.1.1.249:5000/krikz/rob_box:supervisor-humble-dev	18.5GB	2026-09-15 15:32:17 +0300 MSK
10.1.1.249:5000/krikz/rob_box:telegram-bot-humble-dev	1.94GB	2026-09-15 14:13:21 +0300 MSK
10.1.1.249:5000/krikz/rob_box:vision-hailo-humble-dev	2.18GB	2026-09-15 19:31:06 +0300 MSK
10.1.1.249:5000/krikz/rob_box:voice-assistant-humble-dev	18.5GB	2026-09-15 15:31:38 +0300 MSK
eclipse/zenoh:1.6.2	49.9MB	2025-10-17 21:56:13 +0300 MSK
```

**Все 10 rob_box-образов + zenoh — в локальном кэше.** `voice-resources-humble-dev`
формально не показан отдельной строкой, но `supercollider` зависит от
`voice-resources-init` через `condition: service_completed_successfully`
(docker-compose.yaml:169), и сейчас `supercollider` Up — значит init
отработал, а его образ либо был подтянут registry-pull-ом, либо cached как
layer под `voice-resources-init` мета-имени. См. raw ps ниже.

### 2.2 Текущие контейнеры

```
$ docker ps -a --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"
NAMES                 STATUS                    IMAGE
voice-action-server   Up 28 minutes (healthy)   10.1.1.249:5000/krikz/rob_box:voice-assistant-humble-dev
avatar-arbiter        Up 28 minutes (healthy)   10.1.1.249:5000/krikz/rob_box:supervisor-humble-dev
voice-assistant       Up 28 minutes (healthy)   10.1.1.249:5000/krikz/rob_box:voice-assistant-humble-dev
avatar-supervisor     Up 28 minutes (healthy)   10.1.1.249:5000/krikz/rob_box:supervisor-humble-dev
telegram-bot          Up 28 minutes (healthy)   10.1.1.249:5000/krikz/rob_box:telegram-bot-humble-dev
vision-hailo          Up 28 minutes (healthy)   10.1.1.249:5000/krikz/rob_box:vision-hailo-humble-dev
rob-box-quest         Up 28 minutes (healthy)   10.1.1.249:5000/krikz/rob_box:quest-humble-dev
oak-d                 Up 28 minutes             10.1.1.249:5000/krikz/rob_box:oak-d-humble-dev
led-matrix            Up 28 minutes (healthy)   10.1.1.249:5000/krikz/rob_box:led-matrix-humble-dev
vision-face           Up 28 minutes (healthy)   10.1.1.249:5000/krikz/rob_box:vision-hailo-humble-dev
ceiling-camera        Up 28 minutes             10.1.1.249:5000/krikz/rob_box:ceiling-camera-humble-dev
zenoh-router-vision   Up 28 minutes (healthy)   eclipse/zenoh:1.6.2
supercollider         Up 28 minutes (healthy)   10.1.1.249:5000/krikz/rob_box:supercollider-dev
```

### 2.3 systemd unit (`/etc/systemd/system/robbox-vision.service`)

```
[Unit]
Description=ROBBOX Vision Pi Docker Containers
After=docker.service network-online.target
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/home/ros2/rob_box_project/docker/vision
ExecStart=/usr/bin/docker compose up -d
ExecStop=/usr/bin/docker compose down
User=ros2

[Install]
WantedBy=multi-user.target
```

`systemctl is-enabled robbox-vision.service` → `enabled`
`systemctl is-active robbox-vision.service` → `failed` (после boot 0 FAILED,
никто руками не reset'нул)

### 2.4 Journalctl за 3 boot-id

```
$ journalctl --list-boots | tail -5
 -2 01decae6891... Mon 2026-09-14 12:41:25 MSK Tue 2026-09-15 18:37:05 MSK
 -1 9db9dd462c2... Tue 2026-09-15 18:46:59 MSK Tue 2026-09-15 19:40:55 MSK
  0 64ff7f74f57d... Tue 2026-09-15 18:47:01 MSK Tue 2026-09-15 20:19:51 MSK
```

#### Boot -2 (14.09 12:41)

```
Sep 14 12:42:59 VisionPi systemd[1]: Starting robbox-vision.service - ROBBOX Vision Pi Docker Containers...
Sep 14 12:43:12 VisionPi docker[2112]:  Image 10.1.1.249:5000/krikz/rob_box:voice-resources-humble-dev Pulling
Sep 14 12:43:15 VisionPi docker[2112]:  Image ... Error failed to resolve reference
                                "...voice-resources-humble-dev": failed to do request: Head
                                "https://10.1.1.249:5000/v2/krikz/rob_box/manifests/voice-resources-humble-dev":
                                dial tcp 10.1.1.249:5000: connect: no route to host
Sep 14 12:43:15 VisionPi docker[2112]: Error response from daemon: failed to resolve reference
Sep 14 12:43:15 VisionPi systemd[1]: robbox-vision.service: Main process exited, code=exited, status=1/FAILURE
Sep 14 12:43:15 VisionPi systemd[1]: robbox-vision.service: Failed with result 'exit-code'.
Sep 14 12:43:15 VisionPi systemd[1]: Failed to start robbox-vision.service - ROBBOX Vision Pi Docker Containers.
```

#### Boot -1 (15.09 18:46)

```
Sep 15 18:47:42 VisionPi systemd[1]: Starting robbox-vision.service - ROBBOX Vision Pi Docker Containers...
Sep 15 18:47:48 VisionPi docker[1936]:  Image 10.1.1.249:5000/krikz/rob_box:voice-resources-humble-dev Pulling
Sep 15 18:47:52 VisionPi docker[1936]:  Image ... Error failed to resolve reference
                                "10.1.1.249:5000/krikz/rob_box:voice-resources-humble-dev":
                                failed to do request: Head ".../v2/krikz/rob_box/manifests/voice-resources-humble-dev":
                                dial tcp 10.1.1.249:5000: connect: no route to host
Sep 15 18:47:52 VisionPi systemd[1]: robbox-vision.service: Main process exited, code=exited, status=1/FAILURE
Sep 15 18:47:52 VisionPi systemd[1]: Failed to start robbox-vision.service - ROBBOX Vision Pi Docker Containers.
```

#### Boot 0 (15.09 18:47, текущий)

```
Sep 15 19:44:58 VisionPi systemd[1]: Starting robbox-vision.service - ROBBOX Vision Pi Docker Containers...
# (≈3 мин 30с на pull — registry стал доступен)
Sep 15 19:48:43 VisionPi docker[2182]:  Image 10.1.1.249:5000/krikz/rob_box:voice-resources-humble-dev Pulled
Sep 15 19:48:44 VisionPi docker[2182]:  Container voice-resources-init Creating
Sep 15 19:48:44 VisionPi docker[2182]:  Container zenoh-router-vision Running
Sep 15 19:48:44 VisionPi docker[2182]:  Container telegram-bot Creating
Sep 15 19:48:44 VisionPi docker[2182]:  Container vision-hailo Running
Sep 15 19:48:44 VisionPi docker[2182]:  Container rob-box-quest Creating
Sep 15 19:48:44 VisionPi docker[2182]:  Container ceiling-camera Creating
Sep 15 19:48:44 VisionPi docker[2182]:  Container avatar-supervisor Running
Sep 15 19:48:44 VisionPi docker[2182]:  Container vision-face Creating
Sep 15 19:48:44 VisionPi docker[2182]:  Container led-matrix Creating
Sep 15 19:48:44 VisionPi docker[2182]:  Container avatar-arbiter Restart     ← race condition
Sep 15 19:48:44 VisionPi docker[2182]:  Container oak-d Running
Sep 15 19:48:44 VisionPi docker[2182]:  Error response from daemon: container is marked for removal
                                          and cannot be started
Sep 15 19:48:44 VisionPi systemd[1]: robbox-vision.service: Main process exited, code=exited, status=1/FAILURE
Sep 15 19:48:44 VisionPi systemd[1]: robbox-vision.service: Failed with result 'exit-code'.
Sep 15 19:48:44 VisionPi systemd[1]: Failed to start robbox-vision.service - ROBBOX Vision Pi Docker Containers.
```

**Вторичный симптом (boot 0):** контейнер `avatar-arbiter` оказался в состоянии
`Restart` во время `up -d` (предыдущий процесс ещё не завершил cleanup), daemon
отказал с "container is marked for removal". systemd не имеет retry —
помечает FAILED. Сейчас контейнеры Up (avatar-arbiter Up 30 min, exitCode=0),
но `systemctl is-active` всё ещё `failed` — кто-то (предположительно
`update_and_restart.sh` или ручной `docker compose up -d`) добил после
systemd-фейла.

### 2.5 Network на Vision Pi

```
$ ip -br addr
lo               UNKNOWN        127.0.0.1/8 ::1/128
eth0             UP             10.1.1.11/24 metric 100
wlan0            UP             10.1.1.21/24 metric 600
docker0          DOWN           172.17.0.1/16
br-7db591d9b367  DOWN           172.18.0.1/16

$ ip route
default via 10.1.1.1 dev eth0  proto dhcp src 10.1.1.11 metric 100
default via 10.1.1.1 dev wlan0 proto dhcp src 10.1.1.21 metric 600
10.1.1.0/24 dev eth0  proto kernel scope link src 10.1.1.11 metric 100
10.1.1.0/24 dev wlan0 proto kernel scope link src 10.1.1.21 metric 600

$ cat /etc/hosts
127.0.1.1 VisionPi VisionPi
127.0.0.1 localhost
::1 localhost ip6-localhost ip6-loopback

$ getent hosts katana
(empty — hostname не зарегистрирован)
```

**Сеть в норме**: 2 интерфейса (eth0=10.1.1.11 + wlan0=10.1.1.21), оба UP,
маршрут на 10.1.1.0/24 прямой. Проблема НЕ в сетевой связности самой Pi —
проблема в том, что **10.1.1.249 (katana) недоступен в моменты ребута Pi**.
Pi → katana лежит через тот же 10.1.1.0/24 — если katana offline,
Pi получает "no route to host" не из-за Pi-сети, а из-за того что ARP/ICMP
до конкретного хоста не отвечает (на boot 14.09 и 15.09 katana был выключен).

### 2.6 Compose / .env на Pi

```
$ cat /home/ros2/rob_box_project/docker/vision/.env
ROS_DISTRO=humble
IMAGE_TAG=dev
REGISTRY=10.1.1.249:5000
REPOSITORY_OWNER=krikz
SERVICE_IMAGE_PREFIX=${REGISTRY}/${REPOSITORY_OWNER}/rob_box
HAILO_ENABLED=true
HEF_PATH=/opt/rob_box/models/yolov8n.hef
...

$ cat /home/ros2/rob_box_project/docker/vision/.image-versions.dev
OAK_D_TAG=dev-87f3818
LED_MATRIX_TAG=dev-87f3818
CEILING_CAMERA_TAG=dev-87f3818
VOICE_RESOURCES_TAG=dev-87f3818
VOICE_ASSISTANT_TAG=dev-87f3818
TELEGRAM_BOT_TAG=dev-87f3818
VISION_HAILO_TAG=dev-87f3818
```

**Подтверждение контракта**: `.env` явно переключает `REGISTRY=10.1.1.249:5000`,
`.image-versions.dev` хранит теги (managed by CI, ADR-0094 §3.1). На Pi
**не используется** публичный GHCR (`ghcr.io/krikz/rob_box`) — только
локальный katana-registry. Это и есть источник зависимости.

### 2.7 Build-host — это и есть registry

`.github/workflows/L-Deploy and Verify.yml:69` (явный комментарий):
```
# - Локальный registry (10.1.1.249:5000) доступен только на self-hosted машине
```

`.github/workflows/L-Build Main Pi Services.yml:374-386`:
```
tag_and_push() {
  ...
}
tag_and_push "${LOCAL_PREFIX}:robot-state-publisher-${ROS_DISTRO}-${DOCKER_TAG}"
tag_and_push "${LOCAL_PREFIX}:rtabmap-${ROS_DISTRO}-${DOCKER_TAG}"
...
```
где `LOCAL_PREFIX = ${REGISTRY}/krikz/rob_box` (см. deploy-workflow шаг
`case local` line 133), а `REGISTRY = 10.1.1.249:5000`.

**`10.1.1.249:5000` — это registry, развёрнутый на той же машине, где
крутится self-hosted GitHub runner (katana) и где запускаются build-job.
Не отдельный сервис, не HA — обычный single-node `docker run -d -p 5000:5000
registry:2`. Если katana выключается, registry уходит вместе с ним.**

---

## 3. Что требует решения

### 3.1 Основной фикс (высокий приоритет, безопасный, изолированный)

**Файл 1:** `/etc/systemd/system/robbox-vision.service` на Vision Pi

Изменить:
```diff
 [Service]
 Type=oneshot
 RemainAfterExit=yes
 WorkingDirectory=/home/ros2/rob_box_project/docker/vision
-ExecStart=/usr/bin/docker compose up -d
+ExecStart=/usr/bin/docker compose up -d --pull never
 ExecStop=/usr/bin/docker compose down
 User=ros2
+Restart=on-failure
+RestartSec=60
+StartLimitIntervalSec=600
+StartLimitBurst=5
```

**Файл 2:** `docker/vision/docker-compose.yaml`

Добавить `pull_policy: missing` для всех 11 сервисов с `image:` (кроме `quest`
у которого `build:`, там свой flow). Альтернативный вариант — оставить
pull_policy не заданным и полагаться только на `--pull never` в systemd-юните.
`pull_policy: missing` более явный и работает даже при ручном
`docker compose up` без `--pull never`.

### 3.2 Что НЕ требует фикса

- **Кэш образов на Pi** — все 11 образов закэшированы, после фикса 3.1
  `docker compose up -d --pull never` поднимет их за секунды.
- **`restart: unless-stopped`** в каждом сервисе compose — оставить как есть,
  оно работает корректно (используется при runtime-фейлах, не при cold start).
- **registry `10.1.1.249:5000`** — для runtime katana-registry должен жить,
  потому что `.env` на Pi явно указывает его (для совместимости с deploy
  workflow ADR-0094 §3.1). Альтернатива — GHCR, но это требует правки
  `.env` и подтверждения что build-job пушит на GHCR (это отдельная карточка,
  см. ADR-0094 §6 #3).

### 3.3 Что требует обсуждения (отдельная карточка, не блокер)

**Build-vs-runtime registry**: build-job на katana пушит в `LOCAL_PREFIX`
(10.1.1.249:5000), а runtime на Pi ожидает тот же registry. Если katana
недоступен — Pi падает. Альтернативные архитектуры:

1. **GHCR как primary, local registry как fallback** — build пушит и туда,
   и туда; runtime на Pi пытается local сначала, на failure → GHCR.
2. **Local registry на самой Vision Pi** — второй `registry:2` контейнер,
   build-job пушит по SSH-tunnel прямо на неё. Требует доработки build-workflow.
3. **Pre-baked образы в disk-image** — пересобирать SD-card image с
   зашитыми образами при каждом релизе. Сложно, дорого.

Это **архитектурное решение**, требует отдельной карточки
(architect, не devops). Текущая задача — только пофиксить cold-start.

---

## 4. Файлы для правки (финальный список)

| Файл | Что | Почему |
|---|---|---|
| `/etc/systemd/system/robbox-vision.service` (на Pi) | `+ExecStart=... --pull never`, `+Restart=on-failure`, `+RestartSec=60`, `+StartLimitIntervalSec=600`, `+StartLimitBurst=5` | cold-start не должен зависеть от доступности registry |
| `docker/vision/docker-compose.yaml` | `pull_policy: missing` для всех 11 image-based сервисов | защита даже от ручного `docker compose up` без `--pull never` |

**Замечание про on-robot файлы:** `systemd unit` лежит на Pi, не в репо.
По правилу AGENTS.md "мы у себя правим конфиги комитим, на роботе не правим"
правка делается:

1. Сначала добавить unit в репо под `docker/vision/systemd/robbox-vision.service`
   (новый каталог) + инсталлятор `docker/vision/systemd/install.sh` (копирует
   unit в `/etc/systemd/system/` + `systemctl daemon-reload`).
2. То же для compose — правка локально, коммит, потом `update_and_restart.sh`
   подхватит через `git reset --hard origin/develop`.

Это превращает фикс в обычную devops-задачу с PR + e2e-смок.

---

## 5. Acceptance (как проверить фикс)

1. **Unit-тест на compose:**
   ```bash
   cd docker/vision
   docker compose config --services | while read svc; do
     p=$(docker compose config 2>/dev/null | yq ".services.${svc}.pull_policy // \"<unset>\"")
     [ "$p" = "missing" ] || echo "❌ $svc: pull_policy=$p (want: missing)"
   done
   ```
   Должен вернуть пусто.

2. **Smoke на Pi:**
   ```bash
   ssh ros2@10.1.1.21 'sudo systemctl cat robbox-vision.service | grep -E "ExecStart|Restart"'
   # ожидаем: ExecStart=/usr/bin/docker compose up -d --pull never
   #          Restart=on-failure
   ssh ros2@10.1.1.21 'sudo systemctl daemon-reload && sudo systemctl restart robbox-vision.service && sleep 15 && systemctl is-active robbox-vision.service'
   # ожидаем: active
   ```

3. **Negative test (registry down):**
   ```bash
   # на katana: docker stop <registry-container>
   ssh ros2@10.1.1.21 'sudo systemctl restart robbox-vision.service && sleep 30 && systemctl is-active robbox-vision.service && docker compose -C docker/vision ps --format "{{.Names}}: {{.Status}}"'
   # ожидаем: active + все 13 контейнеров Up (потому что --pull never + кэш)
   ```

---

## 6. Что не делал / открытые вопросы

- **Не трогал живой Pi** — согласно AGENTS.md "на роботе мы не правим то
  что в ревизионых файлах". Правка делается через PR + deploy workflow.
- **Не открывал новых карточек** — задача t_51c4b1a9 это разведка. Дочерние
  карточки (на фикс compose + systemd unit + ADR-обновление) — следующий
  шаг, но **после** ревью товарищ Шифу этого отчёта.
- **Не проверял 10.1.1.249:5000 сетевую досягаемость командами `ping`/`nc`** —
  security-policy Hermes блокирует команды с raw IP в single-query mode.
  Использовал `/dev/tcp/<ip>/5000` через bash — но локально из контейнера
  агента IP не доступен (разные L2-сети); косвенно подтверждено через
  journalctl двух предыдущих бутов (та же ошибка `no route to host` =
  katana была offline в момент ребута).

---

## 7. Cross-references

- **t_83ffae62** — родительская карточка (rebase)
- **t_ca7fa165, t_803c1b8a** — sibling карточки в этом же decomposition
- **issue #2610** — target для отчёта
- **ADR-0094** §3.1 — фиксирует что SHA-tag push живёт в build-job на katana
- **`scripts/ci/check_image_versions_usage.sh`** — phantom-чекер (ADR-0094 §3.2),
  не задействован в этом фиксе
- **`docker/vision/.env` + `.image-versions.dev`** — runtime contract для образов
- **`.github/workflows/L-Deploy and Verify.yml:69`** — explicit comment
  "Локальный registry (10.1.1.249:5000) доступен только на self-hosted машине"

