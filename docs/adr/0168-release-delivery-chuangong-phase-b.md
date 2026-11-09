# ADR-0168: Доставка обновлений — релиз как артефакт и агент chuangong, фаза B (наблюдать и скачивать заранее)

| Поле | Значение |
|---|---|
| Статус | Proposed |
| Дата | 2026-11-09 |
| Автор | Claude Code (воркер) по брифу координатора, карточка #4375 |
| Родители | rbt_infra [ADR-0003](https://github.com/krikz/rbt_infra/blob/main/docs/adr/0003-robot-software-delivery.md) (pull-модель «传功», фазы A–D), [ADR-0001](https://github.com/krikz/rbt_infra/blob/main/docs/adr/0001-network-architecture.md) (сеть), [ADR-0002](https://github.com/krikz/rbt_infra/blob/main/docs/adr/0002-naming-school-spirit.md) (имена); ADR-0017 (zenoh REST `:8000`); ADR-0148; ADR-AF-0013 |
| Решения Шифу | 09.11.2026, `docs/shifu-decisions.md` → `d-20261109-chuangong-python-minisign`: **агент — на Python**, **подпись релиза — minisign** (в ADR-0003 это были U5 и U6) |
| Карточка | #4375, итог обсуждения — комментарий 09.11 в карточке |

Метки: **измерено** — есть вывод команды; **по коду** — прочитано; **оценка** — расчёт из измеренного.

## 1. Контекст

Сегодня робот обновляет `L: Deploy and Verify`: раннер katana ходит по ssh с паролем на `10.1.1.10/.11/.12`, на каждой Pi
делает `git reset --hard` ветки и `compose up` по тегам из `.image-versions.<tag>` (**по коду**). Понятия релиза нет,
откат — повторный деплой другой ветки. rbt_infra ADR-0003 выбрал pull-модель: подписанный релиз в локальном реестре
площадки, агент `chuangong` на каждой Pi, ритуал на доке, пульт — workflow, потом панель `dianjiang`.

Этот ADR — **фаза B** ADR-0003 §6 в этом репо: релиз как артефакт и агент, который **только** находит релиз, проверяет
подпись, скачивает заранее и докладывает. Нынешний путь деплоя не меняется ни в одной строке.

## 2. Релиз как артефакт — `scripts/build/release.py`

Формат — единственный модуль `host/chuangong/chuangong/release_format.py`; сборщик и агент импортируют его.

- **`release.yaml`** (схема 1): `release` (имя, оно же каталог на Pi и тег в реестре; только `[0-9A-Za-z._-]`),
  `channel`, `git`, `created`, `image_tag` (SHA-тег сборки), `docker_tag` (тег канала), `roles.<main|vision|sound>`,
  `compat.min_previous`.
- **Образы роли по digest.** Список — из `docker/<роль>/docker-compose.yaml` (то, что роль реально запускает, все
  профили); имя тега — из `docker/build-manifest.yaml` ровно как compose берёт сегодня: сервис с `*_TAG` — SHA-тег
  (`oak-d-humble-dev-b704404`), без поля — тег канала (`teleop-humble-dev`, `quest-humble-dev`, `supercollider-dev`).
  Digest — из реестра (`--resolve-registry`) или из JSON (`--digests`). Незакреплённый образ схема не пропускает.
  Сторонние образы compose (`eclipse/zenoh:1.6.2`, cadvisor, promtail, `ollama:latest`) записаны в `external_images`
  **по тегу, не закреплены** — честный пробел до фазы C.
- **Бандл роли** `bundle-<роль>-<релиз>.tar.gz` — отслеживаемые git файлы `BUNDLE_PATHS[роль]` (каталог роли,
  `docker/common`, `host/<роль>` и всё, на что compose ссылается через `../`: `src/rob_box_description` у main,
  `sound_pack`, `migrations` и скрипт ресурсного пака у sound). Содержимое — из блобов git (не из рабочей копии,
  чтобы autocrlf не менял байты), tar детерминирован (mtime 0, uid 0, сортировка): одинаковый коммит — одинаковый
  sha256. Файлы, похожие на секреты (`.env.secrets`, `.env.local`, `*.key`), сборка отвергает; `.env.secrets` и так
  не в git и остаётся на хосте. Гард в тестах сверяет: всё, на что ссылается compose роли через `../`, лежит в бандле.
  Формат gzip, а не zstd из ADR-0003, — чтобы агент обходился stdlib.
- **Ресурсный пак** — sha256 манифеста `docker/vision/scripts/resource_pack/manifest.yaml` у vision и sound
  (блобы пака и дальше раскладывает `apply_resource_pack.sh`).
- **Хостовые миграции** — список шагов, которые деплой сегодня делает до `compose up`
  (`scripts/setup/ensure_hailo_driver.sh` у vision). Исполнять их будет фаза C.
- **Подпись** — minisign (решение Шифу 09.11): `release.yaml.minisig`, Ed25519 над BLAKE2b-512, trusted comment
  `release:<имя> channel:<канал> git:<sha>`. Модуль `chuangong/minisign.py` (Ed25519 из `cryptography`) пишет и читает
  файлы утилиты `minisign`: подпись проверяется `minisign -Vm release.yaml -p release.pub`, и наоборот (тест
  совместимости в CI с `minisign` из apt). Секретный ключ CI — незашифрованный (`minisign -G -W`).
- **В реестре** релиз — OCI-артефакт `krikz/rob_box/release:<релиз>` (`artifactType
  application/vnd.robbox.release.v1`; слои: release.yaml, подпись, три бандла с аннотацией роли); **указатель
  канала** — тег `krikz/rob_box/release:<канал>` на тот же манифест. Агент верит только подписанному `release.yaml`:
  digest слоя бандла из неподписанного манифеста не используется, бандл сверяется с sha256 из `release.yaml`.
- **`.image-versions` на переходный период** — `release.py image-versions --role <роль>` печатает файл из релиза
  (поля те же, что в `docker/<роль>/.image-versions.dev` сегодня — гард в тестах; `--pinned` даёт `тег@sha256:…`).
  Текущие файлы в репо не трогаются.

Команды: `build`, `sign`, `verify` (подпись + схема + sha256 бандлов), `image-versions`, `publish` (стандартный
Distribution API, без oras). **В `L: Build` не подключено** — это следующий шаг с согласия Шифу (§5).

## 3. Агент `chuangong` — `host/chuangong/`

Python 3 stdlib + PyYAML + `cryptography`, без pip на хосте (на Main Pi есть Python 3.13.3, PyYAML 6.0.2,
cryptography 43.0.0 — проверено координатором 09.11, read-only). Один тик (`python3 -m chuangong run`) по
systemd-таймеру раз в 5 минут.

| Шаг | Что делает |
|---|---|
| fetch | манифест `krikz/rob_box/release:<канал>` → `release.yaml` и подпись → проверка minisign ключом `/etc/chuangong/release.pub` → схема. Нет указателя — `no_release`; реестр недоступен, подпись или схема не прошли — `fetch_error` с причиной |
| stage (только `--mode stage`) | место на диске (бандл ×3 + сжатые слои недостающих образов ×3 + запас 2 ГБ, по `releases_root` и `/var/lib/docker`) → бандл в `/opt/rob_box/releases/.partial/*.part` с докачкой по Range → sha256 → распаковка во временный каталог (только обычные файлы, без `..` и ссылок, ≤ 512 МБ) → rename в `/opt/rob_box/releases/<релиз>/<роль>` с маркером → `docker pull <repo>@sha256:…` недостающих образов. Обрыв, нет места, pull упал — `stage_error`, следующий тик продолжает; готовое не качается заново |
| report | `/var/lib/chuangong/state.json` (атомарно), `/var/lib/chuangong/metrics.prom` (`rbx_release_info{robot_id,node,release,state} 1`, `rbx_release_dirty`, `rbx_release_hil_locked`), PUT в zenoh REST своей Pi `robots/<id>/ops/chuangong/state/<узел>` |

- **Режимы.** `observe` (по умолчанию, так и в примере `chuangong.env`) — fetch + проверка + отчёт, пишет только в
  `/var/lib/chuangong`. `stage` — ещё бандл и образы. `--dry-run` не пишет ничего (ни файлов, ни pull, ни zenoh) и
  печатает запись тика с планом.
- **Переключений нет.** `current`/`previous` в фазе B всегда `null` (робот на старом деплое); симлинк `current`,
  `compose down/up`, миграции, prune старых релизов — фаза C. Поэтому `stage` не стоит держать включённым неделями:
  без prune каталоги релизов и образы по digest копятся (оценка дельты — §6).
- **Входы будущих условий** (в отчёт, ни на что не влияют): замок HIL `/tmp/hil.lock` на Main Pi
  (тот же, что проверяет `scripts/deploy/check_hil_lock.sh`); `dirty` — флаг `/var/lib/chuangong/dirty` (hotpatch его
  пока не ставит, фаза C) или контейнер с меткой `/ws/HOTPATCH` (её пишет `scripts/robot_dev/hotpatch.sh push`).
  Ещё — `legacy_running`: образы запущенных контейнеров, чтобы видеть, на чём робот реально стоит.
- **Ключ состояния по узлу**, а не один `robots/<id>/ops/chuangong/state` из ADR-0003: в фазе B старшего агента нет,
  три Pi писали бы в один ключ. Сводный ключ без узла оставлен за агентом Main Pi в фазе C. Ключ
  `robots/<id>/ops/<node>/release` и пинок `robots/<id>/ops/chuangong/cmd` в этой фазе не сделаны (только таймер).
- **Юниты** `host/chuangong/systemd/chuangong.{service,timer}` и пример `/etc/chuangong/chuangong.env` лежат в репо
  и **не установлены**: `User=ros2`, группа `docker`, `ProtectSystem=strict`, запись только в
  `/var/lib/chuangong` и (если есть) `/opt/rob_box/releases`.

## 4. Пульт — `.github/workflows/F-Update-Robot.yml`

Только `workflow_dispatch`: `robot_id` (пока один `RBXU100001`), `release`, `action` (в фазе B — только `status`),
`timeout_min`, `zenoh_url` (временно `http://10.1.1.10:8000`; фаза A — имя). `runs-on: [self-hosted, rob-box]`
(katana; фаза A — `zongguan, deploy`). Ходит к роботу **только** через zenoh REST: сначала GET
`robots/<id>/ops/chuangong/state/**`, затем подписка SSE до `timeout_min`. Хранилища в роутерах нет намеренно (#3669),
поэтому GET сегодня вернёт пусто, а данные придут подпиской при следующем тике агентов. Итог строит код
(`chuangong/status.py`): таблица трёх узлов в Job Summary; зелёный — только если отчитались все три без ошибок
(и видят ожидаемый релиз, если он задан). Никто не диспатчил.

## 5. Как включить (не сделано, каждый шаг — с согласия Шифу)

1. Ключ подписи: `minisign -G -W -p release.pub -s release.key`; секретный — в секрет CI (`CHUANGONG_SIGNING_KEY`),
   публичный — на Pi в `/etc/chuangong/release.pub` (ADR-0003: приватный ключ в `miji/`).
2. `L: Build All Services`: после сборки — `release.py build --resolve-registry http://localhost:5000` → `sign` →
   `publish` в registry katana (имя релиза — открытый вопрос В3).
3. Установка агента на три Pi (расширение `setup_node.sh` только добавлением): копия `host/chuangong` в
   `/opt/rob_box/chuangong`, `/etc/chuangong/chuangong.env` с узлом, юниты, `systemctl enable --now chuangong.timer`.
4. Потребитель `metrics.prom` (node_exporter textfile или Alloy, ADR-0001) — сейчас у файла потребителя нет.
5. По желанию — узкое хранилище zenoh на `robots/*/ops/chuangong/**` в роутере Main Pi (правка конфига роли):
   `status` ответит сразу, а не после тика.

## 6. Замеры (09.11, read-only)

| Что | Main Pi | Vision Pi | Sound Pi | Метка |
|---|---|---|---|---|
| `df -h /` (свободно из размера) | 83 из 114 ГБ (25 %) | 93 из 115 ГБ (16 %) | 199 из 228 ГБ (10 %) | измерено |
| `docker system df` — образы | 11 шт., 9,634 ГБ | 7 шт., 7,896 ГБ | 8 шт., 12,16 ГБ | измерено |
| Образы текущего `dev` в registry, сжато (состояние `.image-versions.dev` на 09.11) | 2,87 ГБ | 1,79 ГБ | 2,36 ГБ | измерено (registry katana) |
| Новые сжатые слои за день на `dev`, 08.11 (9 состояний `.image-versions.dev`) | 5 МБ | 108 МБ | 36 МБ | измерено |
| То же 07.11 (17 состояний; пересборка базы голоса) | ≥ 16 МБ | ≥ 288 МБ | ≥ 2 375 МБ | измерено, нижняя граница (часть старых тегов уже вычищена из registry) |

Вывод (**оценка**): обычный день `dev` — 0,1–0,2 ГБ фона на робота, день пересборки базы — до ~2,5 ГБ на одну роль;
полная заливка робота с нуля — ~7 ГБ сжатыми слоями. Места на SD хватит на два релиза с запасом; prune обязателен
до того, как `stage` включат надолго. Число «состояний» — это коммиты CI в `.image-versions.dev` (на 04.11 их 57),
а не деплои: агент с тиком 5 минут скачает не больше этого. Сырые выводы — в теле PR.

## 7. Сознательно не сделано

- Фаза C целиком: `current`/`previous`, симлинк, `compose down/up`, миграции, verify, откат, `biguan`/`chuguan`,
  дорога на док, prune, самообновление агента, `update/rollback/cancel` и тикет при провале в `F: Update Robot`.
- Фаза A: имена вместо `10.1.1.x`, реестр по https, ssh по ключам в `L: Deploy and Verify`, WireGuard, `robot.env`.
- Подключение к `L: Build`, установка на Pi, ключ подписи, потребитель метрики (§5).
- `L: Deploy and Verify`, `L: Build All Services`, `scripts/deploy/*`, `hotpatch.sh`, compose и конфиги ролей —
  без изменений. Единственная правка существующего workflow — шаг тестов в `G: Lint Code`.

## 8. Открытые вопросы Шифу

Из rbt_infra ADR-0003 §7 (ответов за Шифу нет):

| # | Вопрос |
|---|---|
| U1 | `fleet/fleet.yaml` и указатели каналов: каталог в `rbt_infra` или отдельный репо `rbt_fleet`? |
| U2 | Активация только по команде (панель/workflow) или допустить `auto` в окне для `dev` в мастерской? |
| U3 | Кто арбитр условий на борту: агент по фактам или ТАРС с правом вето? |
| U4 | Оставлять ghcr.io как зеркало CI? |
| ~~U5~~ | ~~Язык агента~~ — **решено Шифу 09.11: Python** |
| ~~U6~~ | ~~Подпись релиза~~ — **решено Шифу 09.11: minisign** |
| U7 | Док обязателен всегда, или оператор может разрешить обновление на месте (`update_here`)? Что на полигоне без дока? |
| U8 | Что показывает и говорит робот в `biguan`/`chuguan`, и кто это делает (Личность, ТАРС, LED)? |

Новые, из этой работы:

| # | Вопрос |
|---|---|
| В1 | Когда подключать `release.py` к `L: Build All Services` и заводить секрет CI с ключом подписи? |
| В2 | Чем снимать `metrics.prom`: node_exporter textfile на каждой Pi или сразу Alloy (ADR-0001)? |
| В3 | Имя релиза: дата и номер (`2026.11.09-1`, как в ADR-0003) или от коммита (`dev-b704404`)? |
| В4 | Узкое хранилище zenoh на `robots/*/ops/chuangong/**` в роутере Main Pi — заводить или ждать тика агента? |

## 9. Последствия

- Плюсы: релиз воспроизводим и проверяем (`verify` сверяет подпись и каждый байт бандла); агент можно поставить
  в `observe` без риска для робота и увидеть, что он видит; формат и ключи zenoh — те, на которые потом встанут
  ритуал фазы C и `dianjiang`.
- Цена: второй механизм рядом с деплоем на переходный период; ключ подписи в CI; зависимость агента от
  `python3-cryptography` и `python3-yaml` на хосте (на Main Pi есть; Vision и Sound не проверены).
- Риски: `stage` без prune копит образы; GET статуса пуст без хранилища (подписка ждёт тика); сторонние образы не
  закреплены по digest.
