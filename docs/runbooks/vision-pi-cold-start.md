# Vision Pi Cold-Start Runbook

> Операционный runbook для алерта «Vision Pi: 0 running containers past grace».
> Источник: kanban-карточка `t_5ab5e44a` (наблюдаемость и алертинг на failed boot).
> Контекст: ретро-паттерн «silent re-starts» — `robbox-vision.service` падал,
> systemd рестартовал, никто не замечал, потому что `docker ps` показывал «Up»
> на init-стадии oneshot-юнита.

## Что значит алерт

`scripts/monitoring/robbox_vision_health_check.sh` запускается systemd-timerом
`robbox-vision-health.timer` каждые 5 минут. Если по прошествии
`ROBBOX_VISION_GRACE_SECS` (по умолчанию 300 = 5 минут) после старта
`robbox-vision.service` не появилось НИ ОДНОГО running-контейнера —
скрипт пишет:

- строку в `~/.local/state/robbox_vision_alerts.log` (alert-log)
- exit code 1 (systemd timer `OnFailure=`-цепочка, если она подключена)
- метрику `robbox_vision_health_alert 1` в textfile-коллектор Prometheus

Если `docker daemon` недоступен, скрипт НЕ алертит (verdict=`infra_error`,
exit=0) — это инфра-проблема, а не проблема стека.

Если `robbox-vision.service` ни разу не стартовал в этой сессии
(например, машина только что загрузилась в emergency.target), скрипт
НЕ алертит (verdict=`inactive`) — мы не можем отличить «cold boot» от
«выключен», и алертить «выключен» бессмысленно.

## Первые шаги (30 секунд)

```bash
# 1. Статус systemd-юнита: точно ли он active?
sudo systemctl status robbox-vision.service --no-pager

# 2. Что реально работает в docker? (не Up, а running)
ssh pi@vision-pi 'docker ps --filter status=running --format "table {{.Names}}\t{{.Status}}"'

# 3. Логи алерт-скрипта — там reason в human-readable виде
ssh pi@vision-pi 'cat ~/.local/state/robbox_vision_alerts.log'

# 4. Boot-summary (что поднялось, что нет, exit codes)
ssh pi@vision-pi 'tail -100 /var/log/robbox-vision-boot.log'

# 5. Live-метрика
ssh pi@vision-pi 'cat ~/.local/state/robbox_vision_health.prom'
```

Ожидаемые exit codes:

- `verdict=ok` → `robbox_vision_health_alert 0`, `running_containers > 0`.
- `verdict=alert` → `robbox_vision_health_alert 1`, `running_containers=0`,
  `grace_elapsed_seconds >= 300`.
- `verdict=infra_error` → `running_containers=-1` (docker daemon лежит).
- `verdict=inactive` → `grace_elapsed_seconds=-1` (юнит не стартовал).

## Типовые сценарии

### A. «Registry недоступен, docker pull падает»

**Симптомы:** `ExecStartPre=-docker compose pull --ignore-pull-failures`
вернул код ошибки; в journal видно `no such host`/`i/o timeout` для
`ghcr.io` или build-host (katana, 10.1.1.249:5000).

**Что делать:**

1. Проверить с хоста Vision Pi:
   ```bash
   ssh pi@vision-pi 'docker compose pull 2>&1 | head -20'
   ```
2. Если registry действительно лежит — это **ожидаемо**. `ExecStart`
   использует `--pull never`, поэтому стек поднимется на локальном кэше.
   Алерт false-positive: registry вернётся, следующий pull через
   `update_and_restart.sh` подтянет свежее.
3. Если registry недоступен **долго** (>30 минут) — пинговать
   katana-владельца или проверить firewall.

### B. «Образы есть, но сэмплов/моделей на хосте нет (Ресурсный пак не применён)»

**Симптомы:** `supercollider` и `voice-assistant` стартанули с пустым
bind-mount'ом `/opt/rob_box/samples`, в логах supercollider:
`bassball.scsyndef could not be opened`. STT/TTS без `/opt/rob_box/models`
не грузят Vosk/Silero.

Начиная с ADR-0125/ADR-0126 (21.09.2026) образа `voice-resources` и
init-контейнера **больше нет вовсе** — сэмплы и модели кладёт на хост шаг
деплоя «Ensure STT/TTS models + Renardo samples» ДО `docker compose up`,
compose про registry для этих ресурсов не знает и от него не зависит.

**Что делать:**

1. Это **нормальный частичный запуск** (issue #2610, ADR-0111 §2.1,
   уточнено ADR-0126). Музыкальные сэмплы не работают, но диалог/зрение — да.
2. Догнать Ресурсный пак на хосте (сеть нужна ЕМУ, не docker compose):
   ```bash
   ssh pi@vision-pi 'sudo bash ~/rob_box_project/docker/vision/scripts/resource_pack/apply_resource_pack.sh && \
     cd ~/rob_box_project/docker/vision && \
     docker compose restart supercollider voice-assistant'
   ```
3. НЕ алертить как failed-boot — это известный degraded-mode.

### C. «OAK-D или ceiling-camera не стартуют (USB-устройств нет)»

**Симптомы:** в journal видно `device or resource busy` или
`Cannot open /dev/video0`; контейнеры в `docker ps` показывают
`Restarting`.

**Что делать:**

1. Проверить, что USB-камера физически подключена:
   ```bash
   ssh pi@vision-pi 'ls -la /dev/video0 /dev/bus/usb/'
   ```
2. Если устройств нет (Vision Pi без камеры в dev-режиме):
   `docker compose stop oak-d ceiling-camera` — стек остальных сервисов
   должен подняться (частичный cold-start по ADR-0111).
3. Если устройства есть, но контейнер не видит — проверить
   `privileged: true` в compose и `MEM_LOW`/cgroup.

### D. «docker daemon лежит (verdict=infra_error)»

**Симптомы:** `running_containers=-1`, `docker ps` падает с
`Cannot connect to the Docker daemon`.

**Что делать:**

1. `sudo systemctl status docker` — проверить, что демон жив.
2. Если `inactive`: `sudo systemctl restart docker`.
3. Если `active`, но `docker ps` падает — проверить
   `/var/log/docker.log` или `journalctl -u docker`.

## Настройка алертинга

`robbox-vision-health.timer` создаётся функцией `setup_health_monitor()`
в `scripts/setup/setup_vision_pi.sh`. Идемпотентна — повторный запуск
безопасен. Что она делает:

1. Копирует `scripts/monitoring/robbox_vision_health_check.sh` в
   `/usr/local/bin/robbox_vision_health_check.sh`.
2. Создаёт systemd timer + service (`/etc/systemd/system/`) с
   интервалом 5 минут и `OnFailure=`-цепочкой (при желании).
3. Создаёт `/var/log/robbox-vision-boot.log` с правильными правами.
4. `systemctl daemon-reload && systemctl enable --now robbox-vision-health.timer`.

Если нужно изменить grace-период (по умолчанию 300s = 5 минут) — задать
через `/etc/systemd/system/robbox-vision-health.service.d/override.conf`:

```ini
[Service]
Environment="ROBBOX_VISION_GRACE_SECS=600"
```

Затем `sudo systemctl daemon-reload && sudo systemctl restart robbox-vision-health.service`.

## Метрики

Скрипт пишет в `$HOME/.local/state/robbox_vision_health.prom` (textfile
collector Prometheus):

| Метрика | Тип | Описание |
| --- | --- | --- |
| `robbox_vision_running_containers` | gauge | Число running-контейнеров на хосте |
| `robbox_vision_grace_elapsed_seconds` | gauge | Секунд с последнего старта `robbox-vision.service` |
| `robbox_vision_health_alert` | gauge | 1 = алерт, 0 = OK |
| `robbox_vision_health_check_timestamp_seconds` | gauge | Unix-time последнего прогона |

Для подключения к `node_exporter` textfile collector достаточно добавить
`--collector.textfile.directory=/home/pi/.local/state` в флаги
node_exporter и перезапустить его. Альтернативно — отдельный
`prometheus-pushgateway` через `curl` (см. `scripts/monitoring/README.md`).

## Где смотреть, если что-то не так с самим алерт-скриптом

- `journalctl -u robbox-vision-health.service --since "-1h"` — последние
  запуски health-check'а, со stderr (от скрипта).
- `~/.local/state/robbox_vision_alerts.log` — alert-events (JSONL-like).
- `/var/log/robbox-vision-boot.log` — summary после каждого
  `robbox-vision.service` старта (имя/статус/state всех контейнеров).

## Тесты

Регрессия покрыта в `tests/unit/scripts/test_robbox_vision_health_check.py`
(16 тестов: ok/grace/alert/inactive/infra_error, JSON-формат,
prom-textfile, dry-run, shellcheck -S error). Запускать как обычный pytest:

```bash
pytest -q tests/unit/scripts/test_robbox_vision_health_check.py
```

Для systemd unit'а — `tests/unit/scripts/test_setup_vision_pi_systemd_unit.py`.