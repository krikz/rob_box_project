# Quest e2e smoke — ручная методичка + скрипт-помощник

> **Цель документа:** за один подход (~10 минут) проверить, что Phase 1.6
> (Caddy + Docker-сервис `rob-box-quest` на Vision Pi) живёт, отдаёт
> TLS, шлёт heartbeat, отдаёт видео + LiDAR, и что Quest на самом
> деле может через него рулить.
>
> Это **ручной** checklist для Шифу / шисюна после `docker compose up
> quest` на Vision Pi. Для автоматического smoke (DNS + TLS + WSS) есть
> [`local_test/quest_smoke.sh`](../../local_test/quest_smoke.sh) — он
> покрывает первые 4 пункта чеклиста и подъезжает к п. 5 (видео +
> LiDAR latency). Пункты 6–8 требуют живого шлема и оператора.
>
> **Источники истины:**
> - [`docs/architecture/meta-quest-api.md`](../architecture/meta-quest-api.md) — wire-протокол + heartbeat-тайминги
> - [`docs/adr/0027-meta-quest-ar-control.md`](../adr/0027-meta-quest-ar-control.md) — ADR на весь Quest-проект
> - [`docs/plans/2026-08-24-meta-quest-telepresence.md`](../plans/2026-08-24-meta-quest-telepresence.md) — Phase 1.x roadmap

## Содержание

1. [Что понадобится](#1-что-понадобится)
2. [Шаг 0 — `docker compose up quest` жив](#2-шаг-0--docker-compose-up-quest-жив)
3. [Шаг 1 — `quest_smoke.sh --all` (автоматический прогон)](#3-шаг-1--quest_smokesh---all-автоматический-прогон)
4. [Шаг 2 — Import self-signed cert в Quest](#4-шаг-2--import-self-signed-cert-в-quest)
5. [Шаг 3 — Открыть `https://quest.rob_box.local`, ввести PIN](#5-шаг-3--открыть-httpsquestrob_boxlocal-ввести-pin)
6. [Шаг 4 — Видеопоток камеры (latency)](#6-шаг-4--видеопоток-камеры-latency)
7. [Шаг 5 — LiDAR-оверлей](#7-шаг-5--lidar-оверлей)
8. [Шаг 6 — Teleop + safe-stop](#8-шаг-6--teleop--safe-stop)
9. [Шаг 7 — Wi-Fi обрыв → safe-stop + UI](#9-шаг-7--wi-fi-обрыв--safe-stop--ui)
10. [Шаг 8 — Скриншоты Quest-сцены](#10-шаг-8--скриншоты-quest-сцены)
11. [Что дальше](#11-что-дальше)

---

## 1. Что понадобится

- Vision Pi в той же Wi-Fi сети, что и Quest (для `https://quest.rob_box.local:8443`).
- Quest 2/3/Pro, **Developer Mode** включён + ADB-доступ (для импорта self-signed cert).
- Dev-машина с доступом к Vision Pi по SSH (`ros2@10.1.1.21`) — для проверки `docker logs`.
- (Опционально) `local_test/quest_smoke.sh` — должен быть уже в репо.

Проверить, что Docker-стек Quest жив:

```bash
ssh ros2@10.1.1.21
docker ps --filter name=rob-box-quest --filter name=zenoh-router
# Ожидаемо: 2+ контейнера в state Up+healthy.
```

---

## 2. Шаг 0 — `docker compose up quest` жив

```bash
ssh ros2@10.1.1.21
docker logs --tail=50 rob-box-quest 2>&1 | head -80
```

Что ищем в логах:

- `✓ Сертификат создан: /certs/selfsigned.crt` (или `уже существует`).
- `✓ Zenoh router доступен`.
- `caddy run` стартовал БЕЗ ошибок (нет `error: ... listen: address already in use`).
- `ros2 run rob_box_quest quest_node` стартовал (см. `Listening on http://0.0.0.0:8765` или аналог).
- Свежий PIN в логе `quest_node`: что-то вроде `[quest_node] active PIN = 123456`.

Если PIN не виден в `docker logs`, поищи через `grep PIN /var/log/*` или через
`docker exec rob-box-quest env | grep PIN`. Альтернативно — если start_quest.sh
дописал PIN в `/var/run/quest/pin` (как ожидается после #1641), то
`./quest_smoke.sh --all` прочитает его сам без `--pin`.

---

## 3. Шаг 1 — `quest_smoke.sh --all` (автоматический прогон)

С dev-машины:

```bash
cd /home/builder/hermes-share/rob_box_project   # или где у тебя worktree
./local_test/quest_smoke.sh --all --no-color
```

Ожидаемый отчёт:

```
rob_box_quest smoke report
  section          status  detail
  ---------------- ------  ----------------------------------------
  dns              PASS             quest.rob_box.local → 10.1.1.11
  tls              PASS             subject=CN = quest.rob_box.local; SAN DNS=['quest.rob_box.local']; expires in 363d
  healthz          PASS             status=ok, sessions_active=0, version=0.1.0
  wss_handshake    PASS             WELCOME received (session_id=…)
  heartbeat        PASS             n=5, median=200ms, max=210ms (target 200±50ms)
  subscribe_jpeg   PASS             SUBSCRIBE(camera_oak_color) ack OK (stream_id=…), got 18 BINARY_FRAME in 2.0s
  goodbye          PASS             server closed in ~1ms (close code=1000/1005)
  totals: PASS=7 FAIL=0 SKIP=0
EXIT=0
```

Если выход НЕ 0 — приложи **полный** лог (с `--json` для машино-читаемого),
это и есть твой raw-evidence. Самые частые причины:

| Секция FAIL   | Скорее всего                                     | Как чинить |
|---------------|--------------------------------------------------|------------|
| `dns`         | Vision Pi не в DNS, или firewall блокирует UDP/53 | Добавить запись в роутер или `/etc/hosts` на Quest + dev |
| `tls`         | `openssl x509` не находит `DNS:quest.rob_box.local` в SAN | Перегенерировать cert через `start_quest.sh` (проверь `addext subjectAltName=...`) |
| `healthz`     | Caddy не проксирует → aiohttp                     | Проверь `Caddyfile` (`reverse_proxy localhost:8765`) |
| `wss_handshake` | Неверный PIN, или aiohttp не стартовал          | Свежий PIN в `docker logs rob-box-quest` |
| `heartbeat`   | Метрики вышли за 200±50 ms                        | Проверь, что контейнер не под нагрузкой, CPU governor = performance |
| `subscribe_jpeg` | Нет фреймов (NoOpBridge)                       | Нормально на dev-машине — станет PASS на реальном Vision Pi с камерой |
| `goodbye`     | Сервер не закрыл сокет                            | Caddy буферизует → проверить `flush_interval -1` в Caddyfile |

---

## 4. Шаг 2 — Import self-signed cert в Quest

> **Один раз на шлем.** Cert действует 365 дней, потом start_quest.sh
> перегенерирует и придётся повторить.

```bash
# С dev-машины (Vision Pi в той же сети):
scp ros2@10.1.1.21:/certs/selfsigned.crt ~/Downloads/quest-rob_box-local.crt
# (или с Vision Pi: docker cp rob-box-quest:/certs/selfsigned.crt ~/Downloads/)

adb push ~/Downloads/quest-rob_box-local.crt /sdcard/Download/

# Внутри Quest (через ADB shell):
adb shell settings put global install_non_market_apps 1
adb shell pm grant com.android.settings android.permission.WRITE_SECURE_SETTINGS

# Через UI Quest (см. ADR-0027 §4.5):
#   Settings → Privacy & Security → Device unlock → Unknown sources: ON
#   Settings → Privacy & Security → Security → Trusted Sources → Add from Files → quest-rob_box-local.crt
```

Подтверждение: `Settings → Privacy & Security → Trusted Sources` — виден
сертификат `quest.rob_box.local`. Если его нет — повторно через `Files`
(стандартный проводник Quest), `sdcard/Download/quest-rob_box-local.crt`.

---

## 5. Шаг 3 — Открыть `https://quest.rob_box.local`, ввести PIN

В Quest-браузере: `https://quest.rob_box.local:8443/` (Caddy отдаёт
WebXR-клиент из `/srv/quest_static`).

- Должен загрузиться Quest-homescreen (4 floating panels + LiDAR-оверлей,
  см. Phase 1.5).
- Появится PIN-prompt. Введи PIN из `docker logs rob-box-quest` (см. шаг 0).
- Если PIN не принят — проверь что это **свежий** PIN (контейнер
  перезапускался → PIN сменился).

После успешного входа: сцена с 4 видео-панелями (`camera_rear`,
`camera_front`, `camera_oak_color`, `camera_ceiling`) + LiDAR-оверлей
в центре. В DevTools браузера (если включишь через ADB:
`adb shell am start -a android.intent.action.VIEW -d https://...`) видна
консоль без ошибок handshake.

---

## 6. Шаг 4 — Видеопоток камеры (latency)

Замер: на dev-машине `time` от момента, как оператор махнул рукой перед
камерой (`camera_oak_color`), до момента, как её изображение появилось в
Quest-сцене. Ориентир — **≤ 200 ms** (meta-quest-api.md §7).

Альтернативный замер (без человека):

1. На Vision Pi: `date +%s.%N` перед вспышкой лазерной указки в камеру.
2. В Quest: `date +%s.%N` по появлению пятна на сцене.
3. Разница = latency.

Если > 200 ms — проверить `flush_interval -1` в Caddyfile
(см. `docker/vision/quest/Caddyfile` §reverse_proxy).

---

## 7. Шаг 5 — LiDAR-оверлей

Включи в UI сцены (через кнопку на контроллере или DevTools). На
сцене должен появиться концентрический круг (2D-LiDAR-оверлей), с
точкой-роботом в центре и точками препятствий по периметру.

Ориентир: при свободном пространстве вокруг робота (>2 м во все
стороны) — оверлей пустой / с разреженными точками. При поднесении
объекта (рука, стул) — точки появляются в соответствующем секторе.

`topic_id = 0x1101` (см. `protocol/topics.py`), payload — sensor_msgs/LaserScan
в little-endian float32.

---

## 8. Шаг 6 — Teleop + safe-stop

В Quest-сцене нажми стик контроллера вверх (deadman — зажатый trigger +
стик):

- Робот начинает ехать вперёд. В HUD: `vel_linear` > 0.
- Отпусти trigger (или стик) — робот останавливается **за ≤ 100 ms**
  (ADR-0027 §3.3, deadman-watchdog).

Если отпускание trigger НЕ останавливает робота — safe-stop на сервере
не сработал. Возможные причины:

- `Bridge.publish_emergency` не вызывается → проверить `quest_node.py`.
- Watchdog на сервере не видит `JSON_EVENT{type:"ping"}` от клиента →
  проверить что клиент шлёт ping каждые 5 секунд (см. ADR-0027 §3.3).

---

## 9. Шаг 7 — Wi-Fi обрыв → safe-stop + UI

На Vision Pi: `sudo systemctl restart hostapd` (или выдерни шнур питания
Wi-Fi роутера, если нет systemd).

Ожидание:

1. Робот **останавливается за ≤ 500 ms** (Quest-сервер watchdog:
   нет ping от клиента > 600 ms → server-side close).
2. В Quest-сцене появляется баннер **«CONNECTION LOST»** (UI-state
   из Phase 1.5 client).
3. После восстановления Wi-Fi — автоматический reconnect (см.
   `webxr_client/src/wire/reconnect.ts` в Phase 1.5).

Если баннер не появился — Phase 1.5 client не подписан на `ws.close`.

---

## 10. Шаг 8 — Скриншоты Quest-сцены

Для ретро / changelog нужны 4 кадра из Quest-сцены:

1. **4 panels + LiDAR (нормальный режим):** `adb exec-out screencap -p > quest-home.png`
2. **PIN-prompt:** скриншот после `https://quest.rob_box.local/` но до ввода PIN.
3. **CONNECTION LOST:** скриншот после `systemctl restart hostapd`.
4. **После reconnect:** скриншот через ~10 секунд после восстановления Wi-Fi.

Все 4 прикрепить к issue #1643 как evidence после успешного прогона.

---

## 11. Что дальше

После успешного прогона:

- [ ] Все 4 скриншота прикреплены к issue #1643.
- [ ] Полный лог `quest_smoke.sh --all --json` приложен как artifact.
- [ ] Если есть FAIL — заводим sub-task в `t_XXXXXXX` (assignee =
      backend / devops / robocraft по принадлежности).
- [ ] Issue #1643 закрывается с пометкой `e2e-done` (только после
      Шифу-merge PR #1642 — deploy Phase 1.6).

Phase 2 (R10..R14) добавит:
- `stream_select` через UI (без UNSUBSCRIBE+SUBSCRIBE).
- Spatial audio + spatial mapping.
- mTLS + TOTP для multi-user.
- H.264 вместо JPEG.

Эти изменения не должны ломать контракт из этого checklist — smoke
прогоняется после каждого deploy как regression-guard.
