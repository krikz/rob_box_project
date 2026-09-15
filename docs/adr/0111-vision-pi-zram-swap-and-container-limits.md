# ADR-0111 — Vision Pi: zram-swap + MemoryLow для sshd + mem_limit для всех контейнеров

| Поле | Значение |
|---|---|
| Статус | **Proposed** (kanban `t_22362046`, issue #2621) |
| Дата | 2026-09-15 |
| Автор | devops worker (Hermes Agent, kanban t_22362046) |
| Контекст | Vision Pi (10.1.1.21, RPi 5, 8 GB RAM) под нагрузкой становится недоступна по ssh. Коммит `Committed_AS: 10.5 GB` при физической памяти 8 GB. Свопа нет вообще (`/proc/swaps` пуст, `/dev/zram*` отсутствует). Когда `MemAvailable` опускается ниже ~400 MB, ядро не может вытеснить анонимные страницы — `sshd` не получает память → "робот недоступен" до жёсткой перезагрузки. 13 контейнеров, у 9 нет `mem_limit` — при разрастании любого первым умирает случайный сосед. |
| Затрагивает | `scripts/setup/setup_vision_pi_swap.sh` (новый), `host/vision/robbox-zram.service` (новый), `host/vision/ssh-memory-low.conf` (новый), `scripts/setup/setup_vision_pi.sh` (вызов swap-сетапа), `docker/vision/docker-compose.yaml` (9 недостающих `mem_limit`), `tests/unit/scripts/test_setup_vision_pi_swap.py` (новый), `docs/deployment/VISION_PI_DEPLOYMENT.md` (OOM-раздел), `docs/development/DOCKER_STANDARDS.md` (стандарт zram для Vision Pi) |
| Родители | issue #2621 (root), ADR-0018 (честный FAIL — «коммит уже 10.5 GB на 8 GB машине»), `docs/development/DOCKER_STANDARDS.md` §7 (стандарт `mem_limit` для Pi) |
| Связанные | issue #2609 (torch CPU-only — **отдельная** карточка agent:backend), #929 (OOM kill tts_node — теперь под 4 GB, не 2 GB), `docs/investigations/929-oom-findings-wip.md` (PSS/RSS замеры), ADR-0050 (worker-skill delivery), ADR-0013 (incremental delivery) |

> **TL;DR.** На Vision Pi добавляется **zram-swap** (4 GB, zstd) через
> отдельный systemd-юнит `robbox-zram.service` и **drop-in** для ssh с
> `MemoryLow=128M`, чтобы сессия управления не выдавливалась прикладными
> контейнерами. В `docker/vision/docker-compose.yaml` проставляется
> `mem_limit` для 9 контейнеров без него (сейчас лимиты только у 4 — 4 GB у
> voice-assistant, 6 GB у oak-d, 1 GB у vision-hailo/vision-face, 512 MB у
> telegram-bot). Решение **не лечит причину** (большие RSS контейнеров,
> прежде всего ~1.6 GB CUDA-torch в voice-assistant — это #2609, чужая зона),
> но превращает «робот недоступен, нужна перезагрузка» в «робот тормозит,
> но диагностируется».

---

## 1. Проблема

### 1.1. Сырые замеры 2026-09-15 (Vision Pi, issue #2621)

Память уходит монотонно с момента загрузки:

| время | аптайм | load 1m | MemAvailable | buff/cache |
|---|---|---|---|---|
| 20:06 | 21 мин | 3.74 | **1206 MB** | 1230 MB |
| 20:24 | 40 мин | 5.48 | 658 MB | 694 MB |
| 20:29 | 46 мин | 3.66 | 552 MB | 643 MB |
| 20:33 | 50 мин | 7.85 | **416 MB** | 529 MB |

**Свопа нет вообще**:

```bash
$ free -m
               total        used        free      shared  buff/cache   available
Mem:            7937        7481         272         137         513         455
Swap:              0           0           0

$ cat /proc/swaps
Filename   Type   Size   Used   Priority
(пусто)

$ ls /dev/zram*
ls: cannot access '/dev/zram*': No such file or directory
```

При этом коммит уже вдвое превышает физическую память:

```
MemTotal:        8127684 kB
CommitLimit:     4063840 kB       <- 4 GB commit-limit (без swap)
Committed_AS:   10553480 kB       <- 10.5 GB коммита на 8 GB машине
```

### 1.2. Бюджет памяти: 13 контейнеров, у 9 нет `mem_limit`

Замер cgroup `memory.current` (МБ):

| контейнер | memory.current | mem_limit в compose |
|---|---|---|
| voice-assistant | 3568 (45% RAM) | 4 GB ✅ |
| vision-hailo | 610 | 1 GB ✅ |
| oak-d | 582 | 6 GB ✅ |
| led-matrix | 406 | **нет** ⚠️ |
| vision-face | 312 | 1 GB ✅ |
| rob-box-quest | 266 | **нет** ⚠️ |
| avatar-supervisor | 253 | **нет** ⚠️ |
| telegram-bot | 250 | 512 MB ✅ |
| avatar-arbiter | 230 | **нет** ⚠️ |
| ceiling-camera | 206 | **нет** ⚠️ |
| supercollider | 179 | **нет** ⚠️ |
| zenoh-router-vision | 64 | **нет** ⚠️ |
| voice-action-server | 25 | **нет** ⚠️ |
| voice-resources-init | 0 (завершён) | **нет** ⚠️ |
| **итого** | **6951 из 7937 MB** | |

**9 из 13 контейнеров не имеют `mem_limit`** — при разрастании любого из них
первым умирает случайный сосед, а не виновник (это и есть ADR-0018: «кто
соврал — тот виноват; контейнер без лимита — невозможно понять, кто именно
съел память»).

### 1.3. Почему ssh отваливается

Кэш вытесняется под RSS (1230 → 513 МБ), дальше каждое чтение с диска идёт
в реальный I/O, растут context switches:

```
$ vmstat 1 3
 r  b   free   buff  cache   si   so    bi    bo    in     cs us sy id wa
 8  0 269804   3184 714804    0    0  1611  1107 14261     68 46 17 31  6
 7  0 275000   3184 714908    0    0     0     0 16005  32056 40 18 43  0
```

27–32 тысячи переключений контекста в секунду на четырёх ядрах. Когда
`MemAvailable` дойдёт до нуля, падать будет **некуда**: без свопа ядро не
может вытеснить анонимные страницы, остаётся только OOM-killer либо полный
ступор. `sshd` в этот момент не может ни получить память, ни дождаться
планировщика — что и наблюдается как «робот недоступен».

За 27 минут наблюдения MemAvailable упал с 1206 до 416 МБ. OOM-kill'ов в
журнале пока нет (`journalctl -k | grep -i oom` пуст) — то есть до сих пор
спасали перезагрузки.

---

## 2. Решение

### 2.1. zram-swap: 4 GB, zstd, через systemd

**Алгоритм:** `zstd` (коэффициент сжатия ~3:1, чуть больше CPU чем `lzo`,
но RAM важнее). **Размер:** 4 GB (50% от физической RAM — компромисс между
«выигрышем по памяти» и «расходом CPU на сжатие»).

**Архитектура:**

```
/etc/systemd/system/robbox-zram.service (host)
   ├─ [Unit] After=zram-device-prepare.service (если есть systemd-zram)
   ├─ [Service] Type=oneshot, RemainAfterExit=yes
   │    ├─ ExecStartPre=modprobe zram
   │    ├─ ExecStart=/usr/local/sbin/robbox-zram-setup.sh
   │    └─ ExecStop=swapoff /dev/zram0
   └─ [Install] WantedBy=multi-user.target

/usr/local/sbin/robbox-zram-setup.sh (host)
   ├─ load_module zram num_devices=1
   ├─ echo 4G > /sys/block/zram0/disksize
   ├─ echo zstd > /sys/block/zram0/comp_algorithm
   ├─ mkswap /dev/zram0
   ├─ swapon -p 5 /dev/zram0    (-p 5 = выше файлового swap, ниже RAM)
   └─ echo 180 > /proc/sys/vm/swappiness   (zstd сжимает быстро, 180 — рекомендация)
```

**Идемпотентность:** если `/dev/zram0` уже активен — пропустить, не
пересоздавать (живая память может быть в использовании).

**Параметры через env (override):**

| env | default | назначение |
|---|---|---|
| `ROBBOX_ZRAM_SIZE_MB` | 4096 | размер zram в МБ |
| `ROBBOX_ZRAM_ALGO` | zstd | алгоритм сжатия (`zstd`/`lzo`/`lz4`) |
| `ROBBOX_ZRAM_PRIORITY` | 5 | `swapon -p` (выше = приоритетнее) |
| `ROBBOX_ZRAM_SWAPPINESS` | 180 | `vm.swappiness` после активации |
| `ROBBOX_ZRAM_DRY_RUN` | 0 | `1` = только показать, что сделал бы |

### 2.2. MemoryLow=128M для sshd

Создаётся `/etc/systemd/system/ssh.service.d/10-robbox-memory-low.conf`:

```ini
[Service]
MemoryLow=128M
MemoryHigh=256M
```

Эффект: когда `cgroup memory pressure` появится, `sshd` последним получит
notification о нехватке памяти — то есть сессия управления выживет дольше,
чем прикладные контейнеры. Это **не жёсткий OOM-щит** (для этого есть
`MemoryMax`), но и не «sshd умрёт первым при нехватке памяти» (дефолт).

### 2.3. mem_limit для всех контейнеров в `docker/vision/docker-compose.yaml`

С учётом `mem_total = 7937 MB` и бюджета **«контейнеры ≤ 6.5 GB, система ≥
1.5 GB»**, распределение `mem_limit`:

| сервис | RSS сейчас | mem_limit | обоснование |
|---|---|---|---|
| voice-assistant | 3568 MB (стабильно) | **4 GB** (без изменений, уже было) | укладывается в cgroup 86% |
| oak-d | 582 MB | **6 GB** (без изменений) | OAK-D + AprilTag, пик до 2 GB на init |
| vision-hailo | 610 MB | **1 GB** (без изменений) | HailoRT + python |
| vision-face | 312 MB | **1 GB** (без изменений) | RetinaFace ~ 0.7 GB peak |
| telegram-bot | 250 MB | **512 MB** (без изменений) | лёгкий |
| led-matrix | 406 MB | **512 MB** (новое) | 406 MB сейчас, + 100 MB headroom |
| supercollider | 179 MB | **512 MB** (новое) | scsynth ~ 0.3 GB, sclang **отдельный контейнер** voice-assistant |
| ceiling-camera | 206 MB | **512 MB** (новое) | MJPEG-стрим |
| rob-box-quest | 266 MB | **512 MB** (новое) | Caddy + aiohttp |
| avatar-supervisor | 253 MB | **512 MB** (новое) | supervisor_node + LLM chain |
| avatar-arbiter | 230 MB | **512 MB** (новое) | arbiter без LLM |
| zenoh-router-vision | 64 MB | **128 MB** (новое) | легковесный router |
| voice-action-server | 25 MB | **128 MB** (новое) | HTTP-сервер |
| voice-resources-init | завершён | **без лимита** | init-only, `restart: no` |
| cadvisor | profile | **без лимита** | опционально через `--profile monitoring` |
| promtail | profile | **без лимита** | опционально через `--profile monitoring` |
| ollama | profile | **без лимита** | опционально через `--profile ai` |

**Суммарный бюджет лимитов:** 4+6+1+1+0.5+0.5+0.5+0.5+0.5+0.5+0.5+0.128+0.128
= **15.27 GB** (логически — лимиты могут срабатывать только если что-то
пошло не так; штатно занято ~7 GB).

### 2.4. Изменения в `setup_vision_pi.sh`

Добавляется новый шаг `setup_zram_swap` после `setup_autostart`:

```bash
# Настройка zram-swap (issue #2621)
bash scripts/setup/setup_vision_pi_swap.sh --auto
```

Аргумент `--auto` означает: без интерактивных подтверждений, использовать
дефолты. Идемпотентен — можно вызывать повторно.

---

## 3. Альтернативы (рассмотрены и отвергнуты)

### 3.1. ❌ Файловый swap на SD-карте / USB-SSD

- **Файловый swap на SD**: Pi 5 OS использует `dphys-swapfile` (2 GB по
  дефолту), но он **отключён** на текущем Vision Pi (`Swap: 0`). Включение
  → I/O thrashing при 32k context switches (см. §1.3), быстрый износ SD.
- **Swap на USB-SSD**: USB-SSD не подключён к Vision Pi (только SD + NVMe
  на Pi 5, но на текущем образе — только SD).
- **Вердикт**: zram дешевле, быстрее, не изнашивает flash.

### 3.2. ❌ systemd-zram-generator (`/etc/systemd/zram-generator.conf`)

`systemd-zram-generator` появился в systemd 254 (Debian Bookworm имеет 252
на момент PR). На Pi 5 OS (Debian Bookworm-based, `systemctl --version`
→ `systemd 252 (252.38-1~deb12u1)`) **этого пакета нет в репах по
дефолту** (нужен backports).

- **Вердикт**: наш собственный `robbox-zram.service` + bash-скрипт —
  работает на любой systemd ≥ 232 (Pi 3B+ Bookworm включительно), не зависит
  от backports.

### 3.3. ❌ zram-tool / zram-config

Утилита `zram-config` (Debian-пакет `zram-tools`) — обёртка для генератора.
Имеет те же ограничения, что и §3.2 — нужен backport.

### 3.4. ❌ MemoryMax=256M для sshd (жёстче, чем MemoryLow)

`MemoryMax` убьёт `sshd` при превышении, а не защитит. Семантика
противоположная задаче.

---

## 4. План реализации (incremental, ADR-0013)

### 4.1. Шаг 1 (этот PR, devops): файлы

- `scripts/setup/setup_vision_pi_swap.sh` — идемпотентный установщик
  zram. Принимает `--auto`, `--dry-run`, `--remove`, `--status`. Не требует
  параметров в интерактивном режиме (только дефолты + override через env).
- `host/vision/robbox-zram.service` — systemd unit (drop-in стиль для
  совместимости с `robbox-vision.service`).
- `host/vision/ssh-memory-low.conf` — drop-in для ssh.service.
- `tests/unit/scripts/test_setup_vision_pi_swap.py` — регресс-тесты:
  bash-синтаксис, dry-run в tmp-окружении (без root).
- `scripts/setup/setup_vision_pi.sh` — добавить шаг `setup_zram_swap`
  сразу после `setup_autostart`.
- `docker/vision/docker-compose.yaml` — добавить `mem_limit` + 
  `memswap_limit` для 9 контейнеров без лимита (см. §2.3).
- `docs/deployment/VISION_PI_DEPLOYMENT.md` — обновить OOM-раздел:
  убрать `dphys-swapfile`, добавить `robbox-zram.service`.
- `docs/development/DOCKER_STANDARDS.md` §7 — дополнить про zram.
- `CHANGELOG.md` + `progress.md` — отметка.
- ADR (этот документ).

### 4.2. Шаг 2 (НЕ этот PR, отдельные карточки):

- **#2609** — torch CPU-only (-1.6 GB в voice-assistant). Профиль
  `agent:backend`. Уже в работе (issue #2609).
- **Ревизия стека** — выделить «необязательные» контейнеры (`cadvisor`,
  `promtail`, `ollama` уже под `--profile`, но `supercollider` /
  `avatar-arbiter` тоже можно сделать профильными). Профиль
  `agent:architect` или решение Шифу.

### 4.3. Acceptance (этот PR)

- ✅ На Vision Pi после `bash scripts/setup/setup_vision_pi_swap.sh --auto`
  `swapon --show` показывает `/dev/zram0` размером 4 GB.
- ✅ `cat /proc/swaps` непустой.
- ✅ `/sys/block/zram0/comp_algorithm` содержит `zstd` (или `[zstd]`).
- ✅ `cat /proc/sys/vm/swappiness` = 180.
- ✅ `/etc/systemd/system/ssh.service.d/10-robbox-memory-low.conf`
  существует, `systemctl show ssh | grep MemoryLow` показывает
  `MemoryLow=134217728` (= 128 MiB).
- ✅ В `docker/vision/docker-compose.yaml` 13 контейнеров имеют `mem_limit`
  (3 под профилем — опциональны).
- ✅ `docker compose -f docker/vision/docker-compose.yaml config` рендерится
  без ошибок.
- ✅ `bash scripts/setup/setup_vision_pi_swap.sh --dry-run` показывает
  план без изменений на хосте.
- ✅ `pytest tests/unit/scripts/test_setup_vision_pi_swap.py -v` зелёный.

### 4.4. Что НЕ гарантируется этим PR

- ❌ «MemAvailable не опускается ниже 1 GB под штатной нагрузкой» —
  требует фикса #2609 (torch CPU-only), без него voice-assistant всё равно
  занимает 3.5 GB. Этот PR **только** разворачивает ssh-сценарий и
  разделяет контейнеры по лимитам.
- ❌ «Реальный e2e на Vision Pi» — hardware-тест проводит e2e-process
  после merge-gate. Здесь только юнит-тесты + dry-run.

---

## 5. Почему не просто сделать и закрыть карточку

Потому что:
1. **Культура честности** (ADR-0018) — без raw-evidence «я проверил, что
   zram поднялся на Vision Pi» нельзя ставить `done`. Этот PR поставляет
   скрипты, а e2e-process проверит на реальном железе.
2. **Инкрементальность** (ADR-0013) — этот PR не делает всё сразу; зрам
   поднимается отдельно от mem_limit, отдельно от ssh-override. Каждый шаг
   можно откатить без поломки остального.
3. **Прозрачность** — ADR фиксирует trade-off (4 GB zram = 4 GB физической
   RAM + ~12 GB эффективной, минус CPU на сжатие; что лучше, чем
   файловый swap).