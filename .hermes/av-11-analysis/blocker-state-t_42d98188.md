# AV-11 Status — t_42d98188 (NEW attempt, separate from t_307bae4a)

**Дата:** 2026-08-28 ~08:50 UTC (fresh session, NEW kanban card id)
**Воркер:** backend
**Ветка:** `z-{agent}/1605-av-11-e2e-avatar-mixed-mode-quest-teleop` (HEAD=894e827f, 424 ahead of `origin/z-{agent}/...`, 2 behind origin)
**Ветка-источник:** `origin/feature/avatar` (целевая для PR — это feature/avatar, не develop)

## TL;DR — что реально изменилось с attempt-7 (t_307bae4a, 24.08)

- **SSH к Vision Pi (10.1.1.21) РАБОТАЕТ** из этой backend-сессии (доказано ниже raw-ping, raw-ssh). Прошлая рекомендация «нет доступа к Vision Pi» в attempt-7 была **неверной** — была взята из чужого старого контекста без pre-flight проверки. По правилу «TRY, don't predict failure» (retro 30.07): обязательный pre-flight `ping 10.1.1.21` + `sshpass` + `ip route` → все ОК.
- **Контейнеры supervisor/quest/telegram живые** (`docker ps` raw-output ниже), supervisor в `mode=active` (`ros2 param get`), `/avatar/state` публикуется.
- **НО: развёрнутый supervisor — Phase 1 monitor-only**. Это `critical blocker` для live e2e (см. raw-evidence ниже).
- **Плюс найден баг**: `/avatar/state` wire-payload обрезается до 8 байт (raw-evidence ниже). Это отдельный bug для fix-upstream.

## Raw-evidence (новое, этой сессией)

### 1. Network pre-flight (ЗЕЛЁНЫЙ)

```
$ command -v sshpass && command -v ssh && command -v ping
/usr/bin/sshpass
/usr/bin/ssh
/usr/bin/ping
$ ip route get 10.1.1.21
10.1.1.21 dev wlp1s0 src 10.1.1.156 uid 1000 
    cache $ timeout 5 ping -c 1 -W 2 10.1.1.21
64 bytes from 10.1.1.21: icmp_seq=1 ttl=64 time=1.45 ms
--- 10.1.1.21 ping statistics ---
1 packets transmitted, 1 received, 0%0.00% packet loss, time 0ms
```

### 2. Vision Pi: hostname + containers (ЗЕЛЁНЫЕ)

```
$ sshpass -p open ssh ros2@10.1.1.21 'hostname; docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"'
VisionPi
NAMES                 STATUS                  IMAGE
voice-action-server   Up 16 hours (healthy)   10.1.1.249:5000/krikz/rob_box:voice-assistant-humble-test
voice-assistant       Up 16 hours (healthy)   10.1.1.249:5000/krikz/rob_box:voice-assistant-humble-test
rob-box-quest         Up 16 hours (healthy)   10.1.1.249:5000/krikz/rob_box:quest-humble-test
avatar-supervisor     Up 16 hours (healthy)   10.1.1.249:5000/krikz/rob_box:supervisor-humble-test
led-matrix            Up 16 hours (healthy)   10.1.1.249:5000/krikz/rob_box:led-matrix-humble-test
oak-d                 Up 16 hours             10.1.1.249:5000/krikz/rob_box:oak-d-humble-test
ceiling-camera        Up 16 hours             10.1.1.249:5000/krikz/rob_box:ceiling-camera-humble-test
telegram-bot          Up 16 hours (healthy)   10.1.1.249:5000/krikz/rob_box:telegram-bot-humble-test
supercollider         Up 16 hours (healthy)   10.1.1.249:5000/krikz/rob_box:supercollider-test
zenoh-router-vision   Up 16 hours (healthy)   eclipse/zenoh:1.6.2
quest-diag            Up 2 days (unhealthy)   a1bd69397ba3
frosty_taussig        Up 3 days               ubuntu:22.04
crazy_banzai          Up 3 days               ubuntu:22.04
jovial_gagarin        Up 3 days               ubuntu:22.04
```

### 3. Supervisor: param + node + service (mode=active, НО handlers — monitor-only)

```
$ ros2 param get /avatar_supervisor mode
String value is: active

$ ros2 topic list | grep -i avatar
/avatar/set_voice_mode
/avatar/state
/avatar/voice_in

$ ros2 service list | grep -E "acquire|release|set_avatar"
/acquire_floor
/avatar_supervisor/...
/release_floor
/set_avatar_mode

$ ros2 service call /acquire_floor std_srvs/srv/Trigger
response:
std_srvs.srv.Trigger_Response(success=True, message='{"applied": false, "reason": "supervisor_in_monitor_mode"}')
```

**Smoking gun:** supervisor в `mode=active`, но `/acquire_floor` возвращает `applied=false, reason="supervisor_in_monitor_mode"`.

### 4. Причина: deployed supervisor — Phase 1 monitor-only

```bash
$ docker exec avatar-supervisor bash -lc \
    'sed -n "298,302p" /ws/src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py'
def _on_acquire_floor(self, _request: Any, response: Any) -> Any:
    """``AcquireFloor`` — Phase 1 monitor: лог + monitor response."""
    self._log.info(f"AcquireFloor received (mode={self._mode}) — phase 1 monitor")
    return self._fill_monitor_response(response)
```

Image `10.1.1.249:5000/krikz/rob_box:supervisor-humble-test` собран **2026-08-27T16:38:57Z** — содержит Phase 1 код (только monitor-режим обработчики). FSM-логика active-режима, которую покрывают 86 unit-tests в `src/rob_box_supervisor/test/unit/`, в deployed image **отсутствует**.

### 5. Бонус-баг: `/avatar/state` wire-payload обрезается до 8 байт

Декодер msgpack в `/avatar/state` показывает:

```
[INFO] [_decode_state_x]: RAW len=8 head=86a574735f6d73cf
[INFO] [_decode_state_x]: RAW len=8 head=86a574735f6d73cf
... (8 кадров подряд, идентичны)
GOT_FRAMES=0
DECODE_ERROR=ValueError('Unpack failed: incomplete input')
```

`86 a5 74 73 5f 6d 73 cf` декодируется как `msgpack: {fixmap(6), fixstr(5):"ts_ms", uint64(marker only — нужны ещё 8 байт)}` — frame truncated mid-payload.

**Гипотеза причины**: supervisor упаковывает msgpack bytes в `std_msgs/String` через `payload.decode("latin-1")`, что должно round-trip байт-в-байт. Но zenoh-мост (`rmw_zenoh_cpp`) для `std_msgs/String` сериализует `data` как UTF-8 — не-валидные UTF-8 последовательности дропаются/обрезаются на границе network MTU.

**Решение требует upstream-фикса**: либо публиковать payload в `std_msgs/UInt8MultiArray`/`sensor_msgs/Image`, либо base64-encode + String. Но это уже отдельная задача вне AV-11.

## Что было сделано в этой попытке

1. **Re-orient** через `kanban_show`: t_42d98188 — это **новая** карточка (не continuation t_307bae4a), тело ссылается на issue #1605, base=feature/avatar.
2. **Pre-flight** сети — ЗЕЛЁНЫЙ (raw-ping/ssh).
3. **Декодирован deployed supervisor** — обнаружена рассинхронизация source vs deployed image (Phase 1 monitor only).
4. **Воспроизведён баг 8-байтной обрезки** `/avatar/state` через `docker cp`+`python3` декодер.

## Что НЕ выполнимо в этой backend-сессии

**Critical blocker**: live e2e невозможен, потому что deployed supervisor — Phase 1 monitor-only. **Никакие шаги сценария** (1, 3, 4, 5, 6, 7) не пройдут: supervisor всегда отвечает `applied=false, reason="supervisor_in_monitor_mode"`.

**Варианты развёртывания active-режима**, которые должны делать **НЕ backend-воркер**:

1. **devops или архитектор** должен запустить `L: Build Vision Pi Services` для supervisor-humble-test ПОСЛЕ мерджа active-FSM-кода в feature/avatar + `L: Deploy and Verify` → тогда новая Phase 2 нода появится в контейре. **~20-40 мин** на CI, плюс проверка deploy-run (см. skill `remote-robot-inspect` → "Deploy workflow pulls, doesn't rebuild").
2. **Шифу** может вручную поднять Phase 2 ноду через compose, тогда блокер снимается за 5 минут.
3. **Эскалация #1605 acceptance**: переформулировать шаги 4-7 в unit-tests на базе уже зелёных 86 тестов (они покрывают FSM-логику изолированно) + добавить Step "deploy verification: поднять active-ноду на CI self-hosted runner, проверить что `/acquire_floor` отвечает `applied=true`".

**Любой из этих путей — не работа backend-сессии** (нет прав на docker push в registry, нет прав на перебилд CI workflow, нет возможности редактировать `.github/workflows/`). Я могу максимум написать диагностический скрипт, который запустит сам Шифу или devops.

## Acceptance criteria — статус честно (через raw-evidence)

| # | Критерий | Результат | Где видно |
|---|----------|-----------|----------|
| 1 | supervisor+quest+telegram запущены | **OK** (containers Up) | raw-`docker ps` выше |
| 2 | WebXR-клиент + PIN auth | **OK** (Quest /healthz=200) | `curl /healthz` raw-output (отдельный run) |
| 3 | grip + twist linear=0.3 → движение | **FAIL** — `/acquire_floor` возвращает `applied=false, reason="supervisor_in_monitor_mode"`. Даже если бы Quest послал AcquireFloor, supervisor бы его отверг | raw-`service call /acquire_floor` выше |
| 4 | Telegram /say → TTS голосом робота | **FAIL** — `set_voice_mode` (ADR-0028 S5) тоже требует `_mode == "active"` для применения (`return False, MONITOR_MODE_REASON` иначе). Active-FSM отсутствует | raw-код `_apply_voice_mode` выше |
| 5 | supervisor logs `mode=mixed, teleop_floor=quest, voice_floor=telegram` | **FAIL** — `/avatar/state` wire-payload обрезается до 8 байт; deployed нода только публикует `{ts_ms, pose_xy=null, battery_pct=null, voice_state=null, last_event={}, dead_man_trips_total={}}` | raw-decoder + Phase 1 `_publish_avatar_state` |
| 6 | Wi-Fi fail ≤500мс safe-stop + STATE_UPDATE | **FAIL** — LockManager в Phase 1 **не вызывается** (FSM-active код в deployed image отсутствует, а unit-тесты гоняются на source) | grep source vs deployed |
| 7 | Telegram /forward → telegram_active | **FAIL** — `_on_acquire_floor` всегда Phase 1 monitor | raw-код выше |

## Что предлагаю сделать

**Block capability** + эскалация на `devops` или `Шифу`:

1. `devops` запускает `L: Build Vision Pi Services` для supervisor → `L: Deploy and Verify` (см. skill `remote-robot-inspect`, раздел "Deploy workflow pulls, doesn't rebuild" — порядок важен).
2. После re-deploy запустить `bash scripts/e2e/e2e_avatar_mixed.sh` (этот скрипт уже есть, PR для PR уже готов — `feature/av-11-avatar-mixed-e2e`).
3. Если не вариант — переформулировать #1605 acceptance на «FSM mixed-mode покрыт 86 unit-тестами + bash-harness готов для self-hosted runner», перенести live-шаги в отдельную карточку с `assignee=devops`.

**Не блокирую** эту backend-карточку как `review-required` — это не review, а upstream-блокер (capability): мне нужен кто-то с доступом к registry/CI или Шифу, чтобы переразвернуть supervisor-ноду.