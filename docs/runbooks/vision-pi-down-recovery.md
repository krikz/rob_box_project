# Vision Pi (10.1.1.21) — пошаговое восстановление после «No route to host / ARP FAILED»

> **Назначение**: операционный runbook для on-call инженера, который **физически
> подходит** к Vision Pi, когда ночной тик и/или e2e-процесс подтвердили, что
> хост 10.1.1.21 недоступен по WiFi (`ping` 100% loss, `ip neigh` = `FAILED`
> или `<incomplete>`), а сеть ROS2 при этом жива (Main Pi `10.1.1.10/20`,
> host builder `10.1.1.156`, ROS2-Katana `10.1.1.249` — все отвечают).
>
> **Контекст инцидента, который инициировал этот runbook**:
> issue #2648 (2026-09-16, ночной тик, `priority:high`). Vision Pi
> отвалился от AP `ESSID=ROS2` после того, как zram-swap + `MemoryLow` для
> контейнеров были развёрнуты (PR #2643, ADR-0111). Вероятная корневая
> причина — перегрев/перебой питания или kernel panic из-за OOM во время
> тестового раунда. Также релевантные `bug`-метки в issue #2621.
>
> **Правило 13.08** (фиксировано в `AGENTS.md` и ADR-0018): если устранить
> проблему можно только физическим вмешательством — **не лезем** в код и
> конфиги робота. Этот runbook — единственный путь: подойти → диагностика →
> безопасный power-cycle → подтверждение через `scripts/vision_pi_healthcheck.sh`.

## TL;DR

```text
1. (опц.) с builder/249: arp/ping/tail-логи — собрать удалённый baseline.
2. Подойти к Vision Pi. Безопасно снять питание.
3. Пауза 30 с (конденсаторы, watchdog reset).
4. Включить питание. Ждать ~60 с (boot + NetworkManager + ROS2).
5. С самого Pi: scripts/vision_pi_healthcheck.sh --local.
6. С builder: arp/ping/ssh — подтвердить, что Pi вернулся.
7. Если Pi не отвечает > 5 мин — диагностика SD/swap/журнала.
8. Если WiFi не поднимается — fallback на ethernet (eth0=10.1.1.11).
9. Если > 10 мин без прогресса — ЭСККАЛАЦИЯ Шифу (правило ниже).
```

---

## Что понадобится

- Физический доступ к Vision Pi (он в роботе-поле, оценочно ~5 мин пешком
  от рабочего места Шифу — координаты уточнить на месте).
- Кабель USB-C (для прямой консоли в крайнем случае; обычно не нужно).
- Ethernet-кабель **Cat5e/Cat6** (для fallback на `eth0=10.1.1.11`, если
  WiFi не поднимется после power-cycle).
- Ноутбук/телефон с SSH-клиентом (для подтверждения после подъёма).
- Опционально — HDMI-монитор + клавиатура (если нужно смотреть загрузку
  вживую, обычно не нужно: скрипт `--local` сам читает `journalctl`).

---

## Шаг 0 — Удалённый baseline (опционально, до подхода)

> Цель: зафиксировать «до», чтобы потом корректно подтвердить «после».
> Делается с builder (`10.1.1.156`) или с ROS2-Katana (`10.1.1.249`).
> **Не пытаться** чинить удалённо (правило 13.08).

```bash
# Хост: builder (156) или 249
HOST=156
TARGET=10.1.1.21

echo "== arp ==";   arp -n | grep -E "${TARGET//./\\.}" || true
echo "== ping ==";  ping -c 5 -W 2 "${TARGET}" || true
echo "== ip neigh =="; ip neigh show "${TARGET}" || true
echo "== iw station =="; iw dev wlp1s0 station dump 2>/dev/null | grep -E "${TARGET//./\\.}|SSID" || true
echo "== orchestrator tail =="
tail -n 80 /var/log/e2e-orchestrator.log 2>/dev/null | grep -E "${TARGET}|vision-pi" || true
```

Сохранить вывод в `/tmp/vision-pi-baseline-${HOST}-$(date -u +%FT%H%MZ).log`
(это пригодится для follow-up комментария в issue #2648).

---

## Шаг 1 — Физическая проверка индикаторов (без снятия питания)

| Индикатор | Норма | Аномалия → действие |
|-----------|-------|---------------------|
| Зелёный LED на корпусе (power) | горит непрерывно | мигает/не горит → питания нет, проверить БП и разъём |
| Красный LED на плате (Raspberry Pi) | горит | не горит → плата обесточена |
| WiFi-индикатор (или LED на USB-WiFi адаптере) | мигает при трафике | не мигает → см. Шаг 4 (fallback на ethernet) |
| Активность SD-карты (LED рядом с microSD) | редкие вспышки | постоянно горит → возможен kernel-panic loop |
| Температура радиатора/корпуса на ощупь | тёплый | очень горячий (>60 °C на ощупь) → возможен thermal throttle; дать остыть 5 мин перед power-cycle |

Если по индикаторам всё ок, но WiFi не светится — это типичный кейс
«WiFi-драйвер не подцепился после OOM/panic». Идём к Шагу 2.

---

## Шаг 2 — Безопасный power-cycle

> **Почему не «через ssh reboot»?** SSH недоступен (issue #2648). Любые
> watchdog-агенты (`ssh MemoryLow`, OOM-killer) уже отработали — хост
> в ступоре либо полностью. Физический power-cycle единственный способ.

```text
1. Выключить БП (вытащить питание USB-C / выключить тумблер на удлинителе).
2. Пауза 30 секунд (конденсаторы разрядились, watchdog отпустил).
3. Включить питание.
4. Засечь время (T0) — boot Vision Pi обычно занимает 45-75 секунд.
```

Зафиксировать время в `/tmp/vision-pi-powercycle-$(date -u +%FT%H%MZ).log`
формата:

```text
T0=2026-09-16T02:45:00Z  power-on
T1=…                     ssh opened  (после Шага 4)
ΔT=…                     (норма: 45-90 секунд)
```

---

## Шаг 3 — Ожидание загрузки (60 с)

Не трогать Pi в течение ~60 секунд. Параллельно можно запустить
удалённый мониторинг с builder:

```bash
# Хост: builder (156)
for i in $(seq 1 12); do
  if ping -c 1 -W 1 10.1.1.21 >/dev/null 2>&1; then
    echo "[$(date -u +%H:%M:%S)] ping OK (i=$i)"; break
  fi
  echo "[$(date -u +%H:%M:%S)] ping FAIL (i=$i)"
  sleep 5
done
```

Если за 60 с `ping` не ответил — НЕ паниковать, перейти к Шагу 4
(fallback на ethernet). Vision Pi обычно успевает подняться за 45-90 с.

---

## Шаг 4 — Подтверждение через скрипт `vision_pi_healthcheck.sh --local`

> **Сам Pi сейчас не доступен по WiFi**. Скрипт нужно запускать либо:
> а) с подключённой клавиатуры+монитора к Pi, либо
> б) с другой машины через SSH, **когда Pi ответит на Шаге 3** —
>    через `--remote` режим (скрипт сам подключится и соберёт данные).

### 4a. После того, как `ping 10.1.1.21` начал отвечать

С хоста `builder` (156):

```bash
ssh ros2@10.1.1.21 'bash -s' < scripts/vision_pi_healthcheck.sh --local
# (скрипт идемпотентный, ничего не правит, только читает состояние)
```

Скрипт напечатает секции:
- `[1] Питание/аптайм` (`uptime`, `vcgencmd get_throttled`)
- `[2] Сеть: ip a` (wlan0=10.1.1.21, eth0=10.1.1.11)
- `[3] ARP: ip neigh` (Pi видит роутер и Main Pi?)
- `[4] WiFi: iw dev wlan0 link` (ESSID=ROS2, signal)
- `[5] Docker: docker ps` (voice-assistant, hailort, zenoh)
- `[6] ROS 2: ros2 node list` (через контейнеры)
- `[7] Журнал: journalctl -u NetworkManager --since '-5 min'`
- `[8] swap: swapon -s` (после PR #2643 должен быть zram0)

### 4b. Если WiFi не поднялся → fallback на ethernet

Если через 5 минут после power-cycle `ping 10.1.1.21` не отвечает, но
светодиод на корпусе горит:

1. Подключить ethernet-кабель к `eth0` Pi (в роботе стационарный разъём,
   либо вытащить из Main Pi если нужно — НЕ надолго).
2. Проверить: `ssh ros2@10.1.1.11` (ethernet-адрес Vision Pi, см.
   `docs/architecture/NETWORK_TOPOLOGY.md` § 2).
3. Если 10.1.1.11 доступен — Pi жив, проблема только в WiFi. Запустить
   `scripts/vision_pi_healthcheck.sh --local` через ethernet-сессию.
4. **Оставить ethernet-кабель подключённым** до следующего визита
   (комментарий в issue #2648: «работает на ethernet-fallback»).

### 4c. Если ни WiFi, ни ethernet не отвечают > 5 минут

→ Перейти к Шагу 6 (SD/swap диагностика).

---

## Шаг 5 — Сетевая кросс-проверка с двух хостов

После того как `--local` отработал, **обязательно** подтвердить с двух
независимых машин (это тот же baseline, что и в issue #2648):

```bash
# С builder
ping -c 3 -W 2 10.1.1.21
ip neigh show 10.1.1.21 | grep -v FAILED
ssh -o ConnectTimeout=5 ros2@10.1.1.21 'echo OK; uname -a'

# С ROS2-Katana (249)
ssh builder@10.1.1.249 'ping -c 3 -W 2 10.1.1.21; ip neigh show 10.1.1.21'
```

Если все три зелёные — Vision Pi в строю. Закрыть issue #2648
(комментарий со ссылкой на этот runbook + raw-вывод Шага 0 + Шага 5).

---

## Шаг 6 — Если Pi не отвечает > 5 минут (SD/swap/журнал)

> Цель: понять, что именно сломалось (boot, kernel, файловая система,
> OOM-loop), **прежде чем** лезть в recovery (eMMC/SD reflash).
> Все действия — read-only.

### 6.1. Проверить, что Pi вообще грузится

Подключить HDMI-монитор + клавиатуру. Если нет картинки — питание
не доходит до GPU (проверить БП мультиметром, ≥5.1 V / ≥3 A).

### 6.2. Если есть картинка — посмотреть journal

```bash
# Войти под ros2 (или pi) на консоли
journalctl -b -p err          # все error+ с текущей загрузки
journalctl --list-boots       # видно ли предыдущие успешные загрузки
journalctl -k | grep -iE 'oom|panic|throttl' | tail -50
```

Типичные причины (в порядке убывания вероятности после PR #2643):

1. **zram0 не создан** → SWAP отсутствует → `voice-assistant` OOM-loop
   на старте. Лечится перезапуском `zram-swap.service`:
   `sudo systemctl restart zram-swap`.
2. **SD-карта read-only** (`EXT4-fs warning: mounting fs with errors`).
   Требует `sudo umount / && sudo fsck.ext4 -n /dev/mmcblk0p2` (если
   Pi вообще не в состоянии смонтировать `/` — нужна внешняя карта
   через USB-ридер).
3. **Kernel panic** во время init (обычно после OOM на этапе
   systemd). Лечится только power-cycle + наблюдение.
4. **Under-voltage** (`Under-voltage detected!` в `dmesg`). БП
   просажен, заменить/отключить USB-потребители (LED matrix, ReSpeaker).

### 6.3. Быстрый тест SD-карты (read-only)

```bash
# С самой Pi через консоль
sudo smartctl -A /dev/mmcblk0 2>/dev/null | grep -E 'Reallocated|Pending|Crc'
ls -la /var/log/              # логи пишутся? Если read-only — failed
mount | grep -E 'ro,|mmcblk'  # видна ли / в read-only?
```

Полная диагностика SD — отдельная карточка (issue #2625 содержит
backlog; **не** лезть сюда без согласования с Шифу).

---

## Шаг 7 — Когда эскалация

> **Порог: 10 минут без прогресса** с момента power-cycle → эскалация
> Шифу (`@GOODWORKRINKZ` в issue #2648, плюс алерт в `#devops-oncall`
> канал через `scripts/agent_flow/setup-devops-oncall.sh` —
> `docs/runbooks/devops-oncall-channel.md`).

Эскалация нужна, если:

- Pi не отвечает ни по WiFi, ни по ethernet > 10 мин после power-cycle.
- HDMI-консоль показывает kernel panic loop, который не уходит после
  второго power-cycle (boot → panic → reboot → panic).
- `journalctl -k` показывает аппаратные ошибки (`MCE`, `bus error`,
  `spurious 8259`).
- SD-карта не монтируется даже в USB-ридере (требуется перевыпуск
  образа, координация с Шифу).

В комментарии к issue приложить:

1. Raw-вывод `journalctl -b -p err` (если есть консоль).
2. Raw-вывод `scripts/vision_pi_healthcheck.sh --local` (если частично
   поднялся).
3. Точное время power-cycle и текущее время.
4. Фото индикаторов (если есть сомнения в питании).

---

## Шесть параллельных команд с builder (156)

Эти команды нужны **сразу** после того как Pi начнёт отвечать, чтобы
быстро подтвердить, что восстановление полное (не только Pi жив, но и
все зависимые сервисы):

```bash
HOST=builder
TARGET=10.1.1.21

echo "== 1. arp (было FAILED) =="
arp -n | grep -E "${TARGET//./\\.}" || ip neigh show "${TARGET}"

echo "== 2. ping 5x =="
ping -c 5 -W 2 "${TARGET}"

echo "== 3. ping роутер (10.1.1.1) =="
ping -c 3 -W 2 10.1.1.1

echo "== 4. ping Main Pi (10.1.1.20) — sanity что WiFi общий живой =="
ping -c 3 -W 2 10.1.1.20

echo "== 5. ssh на Pi, проверка docker =="
ssh ros2@10.1.1.21 'docker ps --format "table {{.Names}}\t{{.Status}}" | grep -E "voice-assistant|hailort|zenoh|oak-d"'

echo "== 6. orchestrator tail (последний ночной тик) =="
tail -n 50 /var/log/e2e-orchestrator.log 2>/dev/null | grep -iE 'vision-pi|10\.1\.1\.21' || true
```

Если все шесть зелёные → Vision Pi в строю, ночной тик можно перезапустить
вручную (`/var/log/e2e-orchestrator.log` будет видно, что подвисло).

---

## Связанные артефакты

- **Issue (инициатор runbook)**: [#2648](https://github.com/krikz/rob_box_project/issues/2648)
- **Helper-скрипт**: `scripts/vision_pi_healthcheck.sh`
  (идемпотентный, поддерживает `--local` (запуск на самом Pi) и
  `--remote` (запуск с builder, сам делает ssh + скрипт на Pi)).
- **Сеть/топология**: `docs/architecture/NETWORK_TOPOLOGY.md` § 2
  (Vision Pi: eth0=`10.1.1.11`, wlan0=`10.1.1.21`).
- **zram-swap ADR**: ADR-0111 + PR #2643 (zram-swap + MemoryLow для
  контейнеров; вероятный триггер текущего инцидента).
- **OOM-ретро**: `docs/investigations/929-oom-findings-wip.md` (12.08,
  ретро-таблица OOM-событий voice-assistant).
- **On-call канал**: `docs/runbooks/devops-oncall-channel.md` (для
  алертов в `#devops-oncall` после Шага 7).
- **Конвенция runbook'ов**: `docs/runbooks/stale-candidate-triage.md`,
  `docs/runbooks/devops-oncall-channel.md` (структура TL;DR → шаги →
  raw-evidence → эскалация → «связанные артефакты»).
- **Правило 13.08 + ADR-0018**: «Честный FAIL лучше красивого PASS» —
  если нельзя починить удалённо, **эскалировать**, а не выдавать
  желаемое за действительное.