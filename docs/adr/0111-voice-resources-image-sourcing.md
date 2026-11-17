# ADR-0111 — `voice-resources` образ: bake-into-voice-base (локальный build, без katana-зависимости в проде)

| Поле         | Значение                                                                |
|--------------|-------------------------------------------------------------------------|
| Статус       | Accepted (фиксация после ревью карточки t_803c1b8a, issue #2610)         |
| Дата         | 2026-09-15                                                              |
| Автор        | architect (Hermes Agent)                                                |
| Контекст     | Issue #2610 (Vision Pi не поднимает стек после перезагрузки без katana),  |
|              | parent t_83ffae62 (devops-fix), t_ca7fa165 (реализация)                 |
| Расширяет    | ADR-0094 (`.image-versions.*` SHA-tag push из build)                     |
| Связанные    | ADR-0018 (честность), issue #2095 (race init), `docs/architecture/SYSTEM_OVERVIEW.md` §Vision |
| Заменяет     | — (уточняет ADR-0094 только в части состава build-графа, не конфликтует) |

---

## 1. Контекст

### 1.1 Что ломается (raw evidence, issue #2610 / t_51c4b1a9)

`robbox-vision.service` после перезагрузки Vision Pi **не поднимает ни одного
контейнера** — `docker compose up -d` падает на первом же сервисе
(`voice-resources-init`), потому что пытается стянуть образ
`${SERVICE_IMAGE_PREFIX}:voice-resources-humble-${IMAGE_TAG}`,
а `SERVICE_IMAGE_PREFIX=10.1.1.249:5000/krikz/rob_box` (build-хост katana).

```
Sep 15 18:47:48 docker[1936]: Image 10.1.1.249:5000/krikz/rob_box:voice-resources-humble-dev Pulling
Sep 15 18:47:52 docker[1936]: Error ... failed to resolve reference
  "10.1.1.249:5000/krikz/rob_box:voice-resources-humble-dev": failed to do request:
  Head "https://10.1.1.249:5000/v2/krikz/rob_box/manifests/voice-resources-humble-dev":
  dial tcp 10.1.1.249:5000: connect: no route to host
```

Повторяется **на трёх последних boot-id** (10/14/15 сентября). На Pi
закэшировано 10 из 11 образов, не хватает ровно `voice-resources` — но
`depends_on: service_completed_successfully` (`docker-compose.yaml:169, 312`)
каскадно валит **весь стек**, потому что `voice-resources-init` обязан
проинициализировать named-volume `renardo_samples` (иначе scsynth не найдёт
WAV-файлы → `"Buffer UGen: no buffer data"`, `voice-assistant` не сможет
запустить Renardo).

### 1.2 Что такое `voice-resources-init`

Одноразовый init-контейнер (`docker/vision/voice_resources/Dockerfile` +
`init_resources.sh`). Принцип работы:

1. На этапе CI-build скачивает Renardo/FoxDot sample-паки
   (`docker/vision/voice_assistant/download_samples.py`) из публичного сервера
   `https://collections.renardo.org/samples`.
2. Бейкает их в `/renardo_samples_bundle` внутри образа.
3. При первом запуске на Pi копирует содержимое в named-volume
   `renardo_samples`, помечает `.initialized`, `exit 0`. Повторно — early-exit.
4. Если скачивание во время build упало (warning) — образ всё равно
   собирается, но `init_resources.sh` пишет `"WARNING: bundled Renardo
   samples not found — leaving volume empty"`, и downstream-сервисы идут
   в **synth-only mode** (Renardo сам себе синтезирует, scsynth не находит
   WAV, но стартует).

То есть это **не runtime-зависимость, а init-time батч сэмплов**.

### 1.3 Кто реально читает `renardo_samples`

| Потребитель              | Что делает с WAV                                       | Критичность init'а |
|--------------------------|--------------------------------------------------------|--------------------|
| `supercollider`          | `scsynth` читает WAV-путь для `Buffer UGen` при старте | **Hard** (без WAV падает с `Buffer UGen: no buffer data`) |
| `voice-assistant` Renardo| `MusicSkill.search_samples` → `search_renardo_samples` в `sample_search.py` (только для search-tool), runtime-синтез — через Python | **Soft** (есть `synth-only mode` fallback) |
| `voice-assistant` Renardo| Inline-синтез через FoxDot-паттерны (основной use-case)| **Soft** (без сэмплов играет синт) |

Init нужен **только ради supercollider**. Без него стек живёт, но музыка —
синт-only (текущий «хороший» fallback при провале скачивания в build).

### 1.4 Почему katana — это SPOF, а не фича

- `katana` (10.1.1.249:5000) — **build-хост и registry**. CI push'ит туда
  только во время работы `L-Build Vision Pi Services.yml` на self-hosted
  runner'е [self-hosted, rob-box] (см. `.github/workflows/L-Deploy and Verify.yml:69`).
- В проде Vision Pi **не должен** зависеть от того, включён ли dev-host.
- Текущая модель «build-push на katana → Vision Pi pull на каждом старте»
  превращает registry в обязательный шаг запуска стека — это инверсия
  ответственности (registry = dependency для production runtime).
- Каждое отключение katana на rebuild'е Vision Pi = стек лежит до ручного
  вмешательства (что и произошло 10/14/15 сентября, см. issue #2610).

### 1.5 Ограничения, которые уже зафиксированы

- `L-Deploy and Verify.yml:113-117` уже предупреждает: тег `local` не
  публикуется, нужно явно использовать `staging/test`. То есть
  deploy-flow **ожидает**, что registry во время boot может быть недоступен.
- ADR-0094 фиксирует SHA-tag push как часть build-job. То есть **на момент
  публикации SHA** образ уже есть в GHCR. Но на Vision Pi `.env` указывает
  на **локальный** `10.1.1.249:5000`, не на GHCR — и deploy-flow **не**
  переключает `SERVICE_IMAGE_PREFIX` при registry_source=github.

---

## 2. Решение

### 2.1 `voice-resources` больше не отдельный образ

**Сэмплы бейкаются в `voice-base`** (или в `voice-assistant`, если voice-base
станет «пустым base» сразу — см. §2.4), а init-логика переезжает в
**одноразовый build-step внутри `voice-assistant` образа** (или отдельный
**scratch-volume init** на стороне Pi, что ещё проще — см. §2.5).

Конкретный путь реализации — на усмотрение карточки `t_ca7fa165`
(devops-fix). Этот ADR фиксирует только **архитектурное** решение:

> `voice-resources` как **отдельный image-based init-сервис в проде —
> — ликвидируется**. Сэмплы доставляются через `voice-base` (или
> `voice-assistant`), один build-pipeline, одна версия SHA,
> init-логика живёт там же, где её используют.

### 2.2 Что меняется в compose

**Было** (`docker/vision/docker-compose.yaml:169, 183-186, 312`):

```yaml
supercollider:
  ...
  depends_on:
    voice-resources-init:
      condition: service_completed_successfully
  ...

voice-resources-init:
  image: ${SERVICE_IMAGE_PREFIX}:voice-resources-${ROS_DISTRO}-${VOICE_RESOURCES_TAG:-${IMAGE_TAG}}
  container_name: voice-resources-init
  volumes:
    - renardo_samples:/target
  restart: "no"
  ...
```

**Стало** (вариант с build-into-voice-base + init-step в voice-assistant):

```yaml
supercollider:
  ...
  # renardo_samples volume заполняется при первом старте
  # voice-assistant (встроенный init-script копирует из /renardo_samples_bundle
  # в volume и пишет .initialized; далее — no-op). См. ADR-0111.
  volumes:
    - renardo_samples:/root/.config/renardo/samples
  ...

voice-assistant:
  ...
  # init-step: запустить первым, скопировать сэмплы, exit; далее — основной процесс.
  command:
    - /bin/sh
    - -c
    - |
      /usr/local/bin/init_voice_resources.sh || echo "synth-only mode";
      exec /ros_scripts/ros_with_namespace.sh /bin/bash /scripts/start_voice_assistant.sh
  volumes:
    - renardo_samples:/root/.config/renardo/samples
  ...

# voice-resources-init — удалён. Шаг L-Deploy and Verify.yml:373-417 тоже
# удаляется (он существовал только ради гонки в init-контейнере).
```

**Ключевые изменения:**

1. `voice-resources-init` **удалён как сервис**. Один образ, один image tag.
2. `depends_on: service_completed_successfully` для init'а **исчезает** —
   больше нет каскадного краша. (см. ADR-0111 §2.3.)
3. Named-volume `renardo_samples` остаётся (его читают и voice-assistant, и
   supercollider по одному пути — см. `docker-compose.yaml:273,761`).
4. Init-script копирует сэмплы один раз (marker `.initialized`), далее
   exit 0 за <100 ms. Если bundle пустой — warning + продолжение (текущее
   поведение).

### 2.3 Что меняется в CI / build

**Было** (`L-Build Vision Pi Services.yml:239-265` + `:540, :588, :621`):

- Отдельный `build-voice-resources` job, тэги в оба registry
  (GHCR + local katana), `tag_and_push` в `combine` job, упоминание в
  `update-versions` job.

**Стало:**

- `build-voice-resources` job **удаляется**. Шаг скачивания Renardo сэмплов
  (`download_samples.py`) **добавляется в `build-voice-base`** (или
  `build-voice-assistant`, если voice-base становится «пустым») —
  см. §2.4.
- `voice-resources-${ROS_DISTRO}-${NEW_TAG}` строка в combine/версии —
  удаляется.
- `combine` job needs-list (`L-Build Vision Pi Services.yml:540, :688`)
  теряет `build-voice-resources`.

Эффект: **build-граф -1 job, -1 image tag, -1 registry push**. Время
cold-build `voice-base` растёт на ~30-60 сек (скачивание сэмплов),
но это происходит в CI, **не на Pi**.

### 2.4 Альтернатива А: bake в `voice-base`

`voice-base` — общий слой для `voice-assistant` и `supercollider`
(см. `L-Build Vision Pi Services.yml` — оба используют `BASE_IMAGE=
voice-base`). Бейк сэмплов в `voice-base`:

| Плюс                                              | Минус                                                          |
|---------------------------------------------------|----------------------------------------------------------------|
| Один слой — все downstream пользуются             | `voice-base` теперь ≠ «лёгкий base», он ~600 МБ тяжелее        |
| Init-логика естественно живёт в voice-assistant   | Если в будущем появится ещё один потребитель сэмплов без voice-assistant — нужна координация |
| Никаких новых сервисов в compose | — |

### 2.5 Альтернатива Б: scratch-volume init на стороне Pi (host-level)

Вместо переезда init'а в voice-assistant — **заполнять volume прямо
на хосте** во время deploy (аналог `L-Deploy and Verify.yml:373-417`,
но без `docker run init-контейнера`):

```
[Vision Pi] Initialize renardo_samples from GHCR voice-assistant image:
  docker run --rm \
    -v renardo_samples:/target \
    ${SERVICE_IMAGE_PREFIX}:voice-assistant-humble-${IMAGE_TAG} \
    /usr/local/bin/init_voice_resources.sh
  # или просто: docker cp <layers> ... (сложнее)
```

| Плюс                                              | Минус                                                          |
|---------------------------------------------------|----------------------------------------------------------------|
| Compose-файл минимально меняется (нет command-обёртки) | Нужен новый deploy-шаг, плодит сущности                  |
| Init полностью отдельно от voice-assistant lifecycle | Если init упал на этапе bundle-copy — voice-assistant запустится с пустым volume (synth-only) |
| Логика copy одна и та же (init_resources.sh) — переиспользуем без правок | — |

### 2.6 Какая альтернатива выбрана — на усмотрение devops

Этот ADR **не выбирает** между §2.4 и §2.5 — оба варианта убирают
`voice-resources` как отдельный образ в проде. Карточка `t_ca7fa165`
принимает решение на основе ревью, **оба варианта совместимы с этим ADR**.
Главное — **§2.1, §2.2 (compose), §2.3 (CI) — фиксируются**.

### 2.7 Что НЕ делаем

- **Не поднимаем registry на Vision Pi.** Сети робота — managed infra, новый
  сервис на Pi = новая attack surface. Registry должен жить в CI/infra.
- **Не переключаем `.env` Vision Pi на GHCR по умолчанию.** См. §2.8.
- **Не делаем voice-resources частью multi-arch manifest** для offline-режима.
  Это overengineering под несуществующий use-case (нет требования
  «полностью offline-robot»).

### 2.8 Что делаем с registry в проде

**Текущее (плохое):** `.env` на Pi указывает на `10.1.1.249:5000`. Vision Pi
при каждом старте тянет 11 образов с dev-машины.

**Целевое:** в проде Vision Pi должен использовать **GHCR** (облако,
доступно всегда с доступом в интернет). Локальный registry на katana —
**только для staging/тестов** на self-hosted runner'ах (см.
`L-Deploy and Verify.yml:69` — это уже зафиксировано).

Реализация — отдельный пункт deploy-flow (после §2.1-§2.3, **не блокирует**
этот ADR). Достаточно поменять `.env` на Pi:
```
REGISTRY=ghcr.io
REPOSITORY_OWNER=krikz
SERVICE_IMAGE_PREFIX=${REGISTRY}/${REPOSITORY_OWNER}/rob_box
```
и убрать build-push в local registry для production-веток (это отдельная
задача для devops-профиля, не архитектурная).

---

## 3. Последствия

### 3.1 Положительные

1. **Vision Pi поднимается при выключенной katana** (acceptance criteria
   issue #2610). 11 → 10 образов в compose, один источник правды.
2. **Нет каскадного краша** из-за одного отсутствующего образа.
   `depends_on: service_completed_successfully` остаётся только для
   real-runtime зависимостей (zenoh-router, supercollider health).
3. **Build-граф проще** — на 1 job меньше, на 1 image tag меньше в
   `.image-versions.*`. CI-time build voice-base +60 сек — это **в CI**,
   не в runtime.
4. **Устраняется race-condition workaround** (`L-Deploy and Verify.yml:
   373-417`, retro `t_d01fe536+t_9d35468d`) — отдельный init-контейнер
   больше не существует, race нечему гонраться.

### 3.2 Отрицательные / риски

1. **`voice-base` (или `voice-assistant`) становится тяжелее** на ~600 МБ
   (Renardo sample packs: 0_foxdot_default + 1_pitchglitch_samples).
   - Cold-pull образа с Pi: +10-15 сек (Pi — 100 Mbit сеть, ~600 МБ).
     Приемлемо: cold-pull бывает раз в N недель при обновлении версии.
2. **Init-step теперь в основном `voice-assistant`**, не в отдельном
   контейнере. Если init-step зависнет на старте voice-assistant — мы
   не сможем отделить «не скачались сэмплы» от «voice-assistant не стартует».
   - Митигация: init-step обёрнут в `|| echo synth-only mode` (как сейчас
     в `init_resources.sh` line 35). Если init упал — voice-assistant всё
     равно стартует.
3. **Размер сэмплов в build-cache:** CI теперь кэширует ~600 МБ сэмплов
   в каждом build voice-base. При регулярных ребилдах это +10-15 сек к
   cache-warm vs cold-cache.
   - Митигация: `download_samples.py` идемпотентен (использует marker
     `downloaded_at.txt` в `SAMPLES_DIR_PATH`). CI cache hit → 0 сек.

### 3.3 Что **не** меняется

- `docker compose pull --ignore-pull-failures` в `L-Deploy and Verify.yml:
  346` остаётся как defense-in-depth.
- `--pull never` (PR #2617) и `pull_policy: missing` для всех image-based
  сервисов остаются — это уже независимая защита от каскадного краша.
- `Restart=on-failure`, `RestartSec=60`, `StartLimitIntervalSec=600`,
  `StartLimitBurst=5` (PR #2617) остаются как defense-in-depth.
- Music API контракт (`search_renardo_samples`, `MusicSkill.search_samples`)
  не меняется — это публичный API для downstream-инструментов.

---

## 4. Альтернативы, которые отклонены

### 4.A «Поднять registry на Vision Pi» (вариант c из body карточки)

- **Идея:** на самой Vision Pi (или robo-hub) поднять `registry:2`, реплицировать
  туда образы из GHCR, и Vision Pi пулит с localhost:5000.
- **Почему отклонено:**
  - **Новая attack surface** на роботе (registry слушает порт, требует
    TLS-сертификат, требует аутентификации).
  - **Disk usage:** registry storage = N * (image size). На Pi SD-card 32 ГБ
    это ~20% места только под registry.
  - **Operational overhead:** репликация, очистка старых тегов, мониторинг —
    новый сервис для maintenance.
  - **KISS:** registry на каждом устройстве = «edge registry» паттерн, который
    оправдан только при тысячах устройств и bandwidth-ограничениях. У нас
    один робот с гигабитным Wi-Fi.
- **Trade-off:** устраняет зависимость от katana, но ценой нового слоя
  операционных рисков. Не стоит того.

### 4.B «GHCR по умолчанию + опциональный fallback на katana»

- **Идея:** `.env` указывает на GHCR; если он недоступен — fallback на
  katana.
- **Почему отклонено:**
  - Fallback в проде = два режима работы, два набора ошибок, двойное
    тестирование.
  - katana — dev-машина. Использовать её как fallback = «случайно доступный
    dev-env». Это anti-pattern: production должен зависеть от production
    infrastructure, не от dev-окружения.
- **Trade-off:** формально устраняет SPOF katana, фактически — маскирует
  проблему (когда katana online и GHCR недоступен, начинается хаос).

### 4.C «Оставить `voice-resources-init` + `pull_policy: missing` + cached bundle на Pi»

- **Идея:** один раз скачать `voice-resources` образ, держать его на Pi
  постоянно; compose при отсутствии — `pull_policy: missing` (не пулит,
  если образ есть).
- **Почему отклонено:**
  - Решает проблему «на втором старте», но не решает «на самом первом
    старте свежепрошитой Pi» — там образа нет, и нужно тянуть с registry.
  - Не убирает **архитектурный** SPOF: составной init-сервис остаётся,
    его зависимость от registry остаётся, гонки в `depends_on`
    (issue #2095, retro `t_d01fe536+t_9d35468d`) остаются.
  - Не уменьшает build-сложность: образ по-прежнему отдельный, registry
    по-прежнему отдельный.
- **Trade-off:** минимальное изменение, но архитектурно не решает
  корневую проблему (см. §1.4).

### 4.D «Bake сэмплы в supercollider образ»

- **Идея:** скачать сэмплы в supercollider, потому что он «главный
  потребитель».
- **Почему отклонено:**
  - Исторически supercollider — **только runtime scsynth**, не должен
    знать про Renardo Python pipeline. Сэмплы использует Renardo в
    voice-assistant (через `sample_search.py`), не только scsynth.
  - scsynth читает WAV по абсолютному пути `/root/.config/renardo/samples/...`.
    Если сунуть бандл в `/root/.sc_samples/` — придётся менять synthdef'ы
    или делать bind-mount на конкретные файлы (ещё один volume).
  - Создаёт дублирование: voice-base должен прокинуть сэмплы в
    voice-assistant и supercollider по-разному.

---

## 5. План реализации (для t_ca7fa165, devops-fix)

1. **CI (build):**
   - `L-Build Vision Pi Services.yml`: удалить job `build-voice-resources`
     (lines 239-265).
   - Добавить шаг скачивания Renardo сэмплов в `build-voice-base` Dockerfile
     (`docker/vision/voice_base/Dockerfile` или новое место).
   - `combine` job (`L-Build Vision Pi Services.yml:540, :688`):
     убрать `build-voice-resources` из needs.
   - `update-versions` job: убрать строку
     `"voice-resources-${ROS_DISTRO}-${NEW_TAG}"` (line 621).

2. **Compose:**
   - `docker/vision/docker-compose.yaml:169, 183-186, 312` — удалить
     `voice-resources-init` сервис.
   - `depends_on.voice-resources-init` — убрать из `supercollider` и
     `voice-assistant`.
   - Init-логика (`init_resources.sh`) копируется в `voice-assistant`
     образ (или вызывается из `start_voice_assistant.sh` через
     однократный marker-check).

3. **Deploy:**
   - `L-Deploy and Verify.yml:373-417` — удалить весь блок
     `[Vision Pi] Run voice-resources-init`. Race-workaround больше не
     нужен.

4. **Тесты (smoke):**
   - Локально: `docker compose config` валидируется.
   - `docker compose up -d` поднимает 10 контейнеров (на 1 меньше).
   - `supercollider` healthcheck → healthy без `voice-resources-init`
     (значит, сэмплы в volume).

5. **Документация:**
   - `docs/architecture/SYSTEM_OVERVIEW.md` §Vision — обновить «11
     контейнеров» → «10 контейнеров», описать что `voice-resources` больше
     не отдельный образ.
   - Этот ADR (`docs/adr/0111-voice-resources-image-sourcing.md`) —
     ссылка в README + CHANGELOG.

6. **Acceptance:**
   - Холодная перезагрузка Vision Pi с выключенной katana → стек
     поднимается полностью (10 контейнеров Up).
   - `systemctl is-active robbox-vision.service` → `active`.
   - `find /var/lib/docker/volumes/vision_renardo_samples -name '*.wav'
     | wc -l` > 0 (сэмплы в volume).
   - `scsynth` в логах supercollider не содержит
     `Buffer UGen: no buffer data`.
   - `voice-assistant` стартует (healthcheck `pgrep -f 'python3.*voice'`
     → 0).

---

## 6. Открытые вопросы (для Шифу)

1. **§2.4 vs §2.5:** devops (`t_ca7fa165`) выбирает между «bake в voice-base»
   и «scratch-volume init на стороне Pi». Оба совместимы с этим ADR.
   *Моя рекомендация:* §2.4 (bake в voice-base) — проще в сопровождении,
   init-логика естественно лежит рядом с потребителем.

2. **§2.8 (GHCR по умолчанию):** переключение `.env` на Pi на GHCR —
   отдельная задача или часть `t_ca7fa165`? *Моя рекомендация:* отдельная
   задача (это политика, не архитектура; требует проверки bandwidth с Pi
   до GHCR, возможно — VPN/proxy).

3. **Размер сэмплов:** в build-слое — `0_foxdot_default` (~250 МБ) +
   `1_pitchglitch_samples` (~350 МБ) = ~600 МБ. Если это проблема для
   Pi cold-pull — можно бейкать только `0_foxdot_default` (минимум для
   supercollider), а `1_pitchglitch_samples` — оставить опциональным
   через `INCLUDE_PITCHGLITCH_SAMPLES=true` build-arg. *Моя рекомендация:*
   сейчас — оба; опциональность — отдельный ADR если станет проблемой.

---

## 7. Референсы

- **Issue:** #2610 (Vision Pi не поднимается без katana)
- **Parent cards:** `t_83ffae62` (devops-fix), `t_ca7fa165` (реализация),
  `t_803c1b8a` (этот ADR)
- **Предыдущая попытка фикса (не завершилась):** PR #2617
  (закрыт с комментарием «вынести из compose → ADR»)
- **Релевантные:**
  - Issue #2095 / retro `t_d01fe536+t_9d35468d` — race-condition
    в `voice-resources-init` (init exit → supercollider ждёт → name-slot
    занят → systemd FAILED).
  - ADR-0094 §1.3 — build-push race-condition context.
  - `docs/architecture/SYSTEM_OVERVIEW.md` §Vision — общее описание стека.
  - `docs/architecture/diagnostics/2026-09-15-robbox-vision-pull-failure.md`
    — raw evidence, journalctl, .env.