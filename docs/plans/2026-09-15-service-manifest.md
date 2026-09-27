# План: манифест сервисов как единственный источник истины для CI-сборки

| Поле | Значение |
|---|---|
| Дата | 2026-09-15 |
| Автор | architect (Hermes Agent), по заданию архитектурного ревью 2026-09-09 (кандидат «Список сервисов как данные») |
| Статус | Proposed (план, реализация не начата) |
| Контекст | Состав docker-стека (11 сервисов Vision Pi + 8 Main Pi) продублирован минимум в шести местах и уже разошёлся; ревью просит один манифест-данные вместо ручного копирования списка по job'ам |
| Связанные ADR | ADR-0094 (`.image-versions.*` SHA-tag push, phantom-чекер), ADR-0111 (`voice-resources` — удаление сервиса), ADR-0057 (guard как hard gate в `G-Lint Code`), ADR-AF-0013 (инкрементальная поставка) |
| Параллельная задача | `docs/plans/2026-09-15-image-versions-seam.md` — **файл не найден в репозитории на момент написания этого плана** (см. §10) |
| Что НЕ делает этот документ | Не меняет код. Не создаёт манифест. Только план — следующая сессия реализует по этапам §5. |

---

## TL;DR

Манифест — `docker/build-manifest.yaml`, реализация читателя — `scripts/ci/gen_build_matrix.py`. Первый этап заводит манифест и guard-тест, которые **только проверяют**, что манифест совпадает с уже существующими workflow (манифест сперва догоняет реальность — это и есть проверяемый шов между «данными» и «фактическим поведением CI»); поведение CI не меняется. Только после того, как guard зелёный, второй этап переключает `prepare`-job на чтение манифеста и генерацию `matrix` через `fromJSON`. Двухуровневая цепочка `voice-base → voice-assistant → supervisor` не выражается через `needs: fromJSON(...)` — GitHub Actions такого не умеет (см. §3) — и на нынешней глубине графа (3 узла из 19 имеют `depends_on`) не стоит вводить общий механизм «волн»: рекомендация — оставить `voice-assistant` и `supervisor` именованными (генерируемыми, но не матричными) job'ами с явным `needs:`, а всё независимое (16 сервисов) собрать одной matrix-job на Pi. Алгоритм «волн» (топологическая сортировка по `depends_on`) документируется как готовый к использованию, если граф вырастет.

Как побочный результат ревью — при написании этого плана обнаружен и воспроизведён живой пример именно той болезни, которую манифест должен вылечить: guard-тест `tests/unit/test_workflow_refactor_acceptance.py::test_vision_workflow_all_build_jobs_present` **прямо сейчас красный** (список ожидаемых job'ов не содержит `build-vision-hailo`, добавленного позже) и не подключён ни к одному workflow — см. §1.7.

---

## §1. Инвентаризация дублирования

### 1.1 Обзор мест

| # | Место | Строк | Что именно знает о составе стека |
|---|---|---|---|
| 1 | `.github/workflows/L-Build Vision Pi Services.yml` | 710 | 11 job'ов `build-*` (тело каждого), `needs:` четырёх сводных job'ов, `tag_and_push`, `verify_in_registry`, `sed` по `.image-versions.*` |
| 2 | `.github/workflows/L-Build Main Pi Services.yml` | 508 | То же самое для 8 сервисов Main Pi |
| 3 | `.github/workflows/L-Build Single Service.yml` | 761/762 (см. 1.6) | `choice`-список сервисов дважды (per pi_type) + `case`-ветки конфигурации + `case`-ветки маппинга на `*_TAG` + echo-список в summary |
| 4 | `docker/vision/.image-versions.dev` | 8 полей | Список тегов, которые пишет `update-image-versions` |
| 5 | `docker/main/.image-versions.dev` | 8 полей | То же для Main Pi |
| 6 | `docs/guides/SINGLE_SERVICE_BUILD.md` | — | Документация вручную повторяет список сервисов (комментарий `L-Build Single Service.yml:6-13` явно требует держать её синхронной) |
| 7 | `tests/unit/test_workflow_refactor_acceptance.py` | 393 | Ещё один независимый список `EXPECTED_MAIN_JOBS` / `EXPECTED_VISION_JOBS` — guard-тест, сам являющийся седьмым местом дублирования (и уже разошедшийся, см. 1.7) |

Итого: не 4 повторения внутри одного файла (как было в первоначальной гипотезе ревью), а минимум **семь независимых копий** знания «какие сервисы есть» по всему дереву репозитория, плюс восьмая (неформальная) — комментарии, которые вручную объясняют исключения (supercollider, teleop).

### 1.2 Vision Pi — 11 `build-*` job'ов (проверено, `L-Build Vision Pi Services.yml`)

| Job | Строка |
|---|---|
| `build-oak-d` | 97 |
| `build-led-matrix` | 126 |
| `build-ceiling-camera` | 158 |
| `build-voice-assistant` | 187 (`needs: [prepare, build-voice-base]` — строка 188) |
| `build-voice-resources` | 239 |
| `build-voice-base` | 268 |
| `build-telegram-bot` | 297 |
| `build-supercollider` | 338 |
| `build-supervisor` | 373 (`needs: [prepare, build-voice-assistant]` — строка 383) |
| `build-quest` | 428 |
| `build-vision-hailo` | 479 |

Все номера строк совпадают с гипотезой ревью 1:1 — расхождений не обнаружено (перепроверено `wc -l` + построчным чтением файла целиком).

Список повторяется **четыре раза** внутри этого файла:
- `needs:` у `update-image-versions` — строка 540, все 11 job'ов.
- `tag_and_push` — строки 585–591 (**только 7** из 11: `oak-d`, `led-matrix`, `ceiling-camera`, `voice-resources`, `voice-assistant`, `telegram-bot`, `vision-hailo`; комментарий на строке 592 объясняет исключение только для `supercollider` — «supercollider excluded: tag naming is inconsistent»).
- `verify_in_registry` цикл — строки 619–626, тот же набор из 7.
- `needs:` у `summary` — строка 688, снова все 11.

### 1.3 Уже наблюдаемое расхождение — подтверждено и уточнено

Гипотеза ревью называла только `supercollider` как исключённый из `tag_and_push`. Построчная проверка (§1.2) показывает, что из `tag_and_push`/`verify_in_registry` **исключены четыре сервиса, а не один**: `voice-base`, `supercollider`, `supervisor`, `quest`. Комментарий в коде объясняет исключение только для `supercollider` (строка 358-360, 592). **Для `voice-base`, `supervisor`, `quest` в коде нет ни одного комментария, объясняющего исключение** — не подтверждено, намеренное это решение или забытое (см. §11, открытый вопрос).

`.image-versions.dev` (`docker/vision/.image-versions.dev`) содержит 8 полей:
```
OAK_D_TAG, RTABMAP_SYNC_TAG, LED_MATRIX_TAG, CEILING_CAMERA_TAG,
VOICE_RESOURCES_TAG, VOICE_ASSISTANT_TAG, TELEGRAM_BOT_TAG, VISION_HAILO_TAG
```
`sed` в `update-image-versions` (`L-Build Vision Pi Services.yml:640-648`) действительно пишет во все 8 полей. Но **`RTABMAP_SYNC_TAG` не соответствует ни одному `build-*` job'у Vision Pi** — нет job'а `build-rtabmap` в этом файле (rtabmap собирается только на Main Pi). Проверено:
```
grep -rn "RTABMAP_SYNC_TAG" --include=*.yaml --include=*.yml --include=*.sh docker/ scripts/ .github/
→ только docker/vision/.image-versions.dev и L-Build Vision Pi Services.yml:642 (сам sed),
  плюс L-Build Single Service.yml:579 (маппинг VERSION_VAR для pi_type=vision+service=rtabmap —
  но в case-списке сервисов vision (строки 202-317) ветки "rtabmap" НЕТ, то есть этот код
  недостижим для pi_type=vision)
```
Это ровно тот класс «phantom `*_TAG`», который уже описан в ADR-0094 §1.4 (там перечислены `MICRO_ROS_AGENT_TAG` для Main Pi и исторические `SUPERVISOR_TAG`/`QUEST_TAG`, которые к текущему моменту из `.image-versions.dev` уже убраны). `RTABMAP_SYNC_TAG` в этом списке ADR-0094 не упомянут — это **новый подтверждённый факт**, не отмеченный ранее ни в одном ADR.

### 1.4 Main Pi — 8 `build-*` job'ов (проверено, `L-Build Main Pi Services.yml`)

| Job | Строка |
|---|---|
| `build-robot-state-publisher` | 78 |
| `build-rtabmap` | 120 |
| `build-twist-mux` | 149 |
| `build-teleop` | 178 |
| `build-ros2-control` | 207 |
| `build-nav2` | 239 |
| `build-lslidar` | 268 |
| `build-perception` | 297 |

Ни один job не зависит от другого (все `needs: prepare`) — граф Main Pi плоский. `tag_and_push` (381-387) и `verify_in_registry` (410-417) содержат 7 из 8 — исключён `teleop`, с явным комментарием (строка 388: «teleop excluded: still uses IMAGE_TAG in docker-compose»). Это единственный документированный случай расхождения на Main Pi.

`docker/main/.image-versions.dev` содержит 8 полей, включая `MICRO_ROS_AGENT_TAG` — уже задокументированный в ADR-0094 §1.4 phantom (0 использований вне `.image-versions.*`, перепроверено тем же grep).

### 1.5 `L-Build Single Service.yml` — пятое-шестое места

Комментарий на строках 6-13 **явно формулирует обязательство синхронизации** как процессное правило, а не как архитектурный факт:

> «список `service` ниже (per pi_type) и `case "$SERVICE"` в job `prepare` обязаны оставаться зеркалом сервисов, которые собирает "L: Build Vision Pi Services" ... держи список сервисов там тоже синхронным [имеется в виду `docs/guides/SINGLE_SERVICE_BUILD.md`]»

Внутри этого одного файла знание о сервисах закодировано **четыре раза**:
1. `choice`-список опций инпута `service` (строки 42-68) — 21 пункт (main + vision + common + base), включая `apriltag`, которого нет ни в одном `build-*` job'е Vision Pi (см. 1.6).
2. `case "$SERVICE"` внутри `case "$PI_TYPE"` (строки 135-368) — по одной ветке на сервис, дублирует dockerfile/context/base_image из соответствующего `build-*` job'а другого файла.
3. `case "$SERVICE_NAME")` → `VERSION_VAR` (строки 572-588) — маппинг имени сервиса на имя переменной в `.image-versions.*`, включая недостижимую ветку `rtabmap) ... RTABMAP_SYNC_TAG` (см. 1.3) и `apriltag) VERSION_VAR="APRILTAG_TAG"` — переменной `APRILTAG_TAG` нет ни в одном `.image-versions.*` файле (проверено grep, ноль совпадений).
4. Echo-список в `summary` (строки 641-670) — текстовое перечисление сервисов по pi_type, вручную поддерживаемое в синхроне с (1) и (2).

### 1.6 `apriltag` — третий независимо найденный пример дрейфа

`apriltag` присутствует в `choice`-списке и в `case`-ветке `L-Build Single Service.yml` (строки 58-59, 245-251), у него есть `docker/vision/apriltag/Dockerfile`. Но:
- Нет `build-apriltag` job'а ни в одном матричном workflow.
- Комментарий в `G-Validate Docker Compose.yml` (косвенно, через список директорий на строке 254: `for service in oak-d zenoh-router supercollider` — apriltag не входит) и явный комментарий в `docker-compose.yaml` (`vision`, строка 58) прямым текстом: «This eliminates the need for a separate apriltag container» — сервис интегрирован в `oak-d` и как отдельный образ в проде не используется.
- Нет `APRILTAG_TAG` ни в одном `.image-versions.*`.

Вывод: `apriltag` в `L-Build Single Service.yml` — мёртвый пункт выбора, оставшийся от архитектуры до интеграции в `oak-d`. Он не входит в область этого плана (Single Service — открытый вопрос, см. §11), но подтверждает общий диагноз: без единого источника истины список расходится независимо в трёх разных местах тремя разными способами (`supercollider`/`voice-base`/`supervisor`/`quest` — недокументированное исключение из тегирования; `RTABMAP_SYNC_TAG` — phantom-поле без владельца; `apriltag` — мёртвый пункт выбора).

Про `L-Build Single Service.yml:761/762` (число строк): `wc -l` даёт 761, построчное чтение редактора показывает содержимое до строки 762 — файл не оканчивается символом перевода строки, поэтому `wc -l` (считает `\n`) и номер последней строки в редакторе расходятся на 1. Это не расхождение данных, а артефакт подсчёта; далее по тексту используется нумерация редактора (`cat -n`-подобная, как в выводе инструмента чтения файлов).

### 1.7 Guard-тест уже существует — и уже красный

`tests/unit/test_workflow_refactor_acceptance.py` (393 строки, последнее изменение — коммит `87b3c6f6`, 2026-09-09, PR #2306) уже делает ровно то, что нужно этому плану: парсит `L-Build Main/Vision Pi Services.yml` через `yaml.safe_load` и сравнивает множество `build-*` job'ов с зашитым в тест эталонным множеством (`EXPECTED_VISION_JOBS`, строки 58-69 файла).

Проверено запуском:
```
$ python -m pytest tests/unit/test_workflow_refactor_acceptance.py -q
.F...................................................................... [ 81%]
................                                                         [100%]
FAILED tests/unit/test_workflow_refactor_acceptance.py::test_vision_workflow_all_build_jobs_present
AssertionError: Vision Pi build jobs changed. Expected {...10 jobs без build-vision-hailo...}, got {...11 jobs...}
1 failed, 87 passed in 5.04s
```
`EXPECTED_VISION_JOBS` не содержит `build-vision-hailo` (добавлен позже коммитами `9b1b8e6e`/`83bcd5d5`/`fa8b11bf`, «feat(hailo): enable real inference Phase 1.5» — после последнего изменения тестового файла). Проверено также, что этот тестовый файл **не вызывается ни в одном workflow**:
```
$ grep -rn "tests/unit\b" .github/workflows/*.yml
→ пусто (только упоминания scripts/agent_flow/tests и src/*/test в комментариях)
```
Это живой, воспроизводимый прямо сейчас экземпляр ровно того класса дефекта, который описан в ADR-0057 §1.2 («скрипт написан, но в CI не подключён») — только для другого guard'а. Он не входит в объём этого плана как «то, что нужно чинить», но является сильным аргументом «за»: даже написанный вручную guard-тест на статический список деградирует за десять дней без единого источника истины и без подключения к CI.

---

## §2. Интерфейс манифеста

### 2.1 Модуль и его граница

Манифест — модуль в смысле «глубокий интерфейс, простая реализация читателя»: с внешней стороны (у воркеров, редактирующих Dockerfile или добавляющих сервис) видна одна декларативная точка правки — `docker/build-manifest.yaml`. Вся логика, которая раньше была *размазана* по шести файлам (как собрать команду `buildx`, как сформировать список тегов, как решить, кто зависит от кого), уходит либо в уже существующий `.github/actions/l-build-service` (реализация сборки — не меняется этим планом), либо в новый читатель `scripts/ci/gen_build_matrix.py` (реализация превращения данных в matrix/needs-граф). Сам манифест не содержит императивной логики — это швейный шов между «что собирать» (данные, версионируемые как обычный YAML, читаемые diff'ом в PR) и «как собирать» (код, который не должен меняться каждый раз, когда меняется состав стека).

### 2.2 Формат (YAML), поля

```yaml
version: 1

defaults:
  registry:
    ghcr: ghcr.io/krikz/rob_box
    local: localhost:5000/krikz/rob_box
    local_registry_host: localhost:5000
  base_registry: localhost:5000/krikz/rob_box_base
  ros_distro: humble
  platform: linux/arm64
  apt_proxy: http://host.docker.internal:3142

pis:
  <vision|main>:
    services:
      <имя-сервиса>:            # = имя build-* job'а без префикса "build-"
        dockerfile: <путь от корня репозитория>
        build_context: <путь от корня репозитория, "." допустим>
        base:
          family: <depthai|ros2-zenoh|rtabmap|pcl>   # → base_registry:<family>-<ros_distro>
          # ИЛИ
          service: <имя другого сервиса из этого же pi>  # → local.<service>-<ros_distro>-<docker_tag>
          # ИЛИ base: null                                # сервис без BASE_IMAGE (voice-resources, supercollider)
        depends_on: [<имя сервиса>, ...]   # пусто, если base.family или base: null;
                                            # для base.service — ОБЯЗАНО содержать тот же сервис
                                            # (инвариант проверяется guard-тестом, см. §6)
        tag:
          ros_distro: <true|false>   # false — только supercollider, "свой формат тега"
        image_versions: <ИМЯ_ПЕРЕМЕННОЙ_В_.image-versions.* | false>
        submodule_sha: <путь субмодуля | не указано>       # → compute-submodule-sha композит-экшена
        source_hash:
          arg: <имя build-arg, обычно SOURCE_HASH>          # см. §2.4 — не всегда SOURCE_HASH
          groups:
            - paths: [<путь>, ...]
              extensions: [<.py|.ts|.json|.html|.xacro|.urdf|CMakeLists.txt|package.xml|.msg>, ...]
        build_args: {<KEY>: <VALUE>, ...}   # статические build-args сверх APT_PROXY/BASE_IMAGE/*_HASH/*_SHA
        pre_build: <имя именованного хука | не указано>    # см. §2.5 — не всё декларативно
```

Поля выведены построчной сверкой с реальным кодом (не придуманы заранее): `dockerfile`/`build_context`/`base` — 1:1 с одноимёнными input'ами композит-экшена `l-build-service` (`action.yml:32-46`); `tag.ros_distro` — единственное различие в формуле тега между 18 сервисами (`${name}-${ROS_DISTRO}-${tag}`) и одним (`${name}-${tag}`, `L-Build Vision Pi Services.yml:364-365`); `image_versions` — точное соответствие присутствию/отсутствию сервиса в `tag_and_push`+`verify_in_registry`+`sed` (§1.2-1.4); `submodule_sha`/`source_hash` — прямое соответствие полям `compute-submodule-sha` и `SOURCE_HASH`/`URDF_FILES_HASH` build-arg, которые сегодня считаются вручную в pre-step каждого job'а (например `L-Build Vision Pi Services.yml:203-223` для voice-assistant, `L-Build Main Pi Services.yml:94-104` для robot-state-publisher).

### 2.3 `base.service` и `depends_on` — почему два поля, а не одно

`base.service: voice-base` уже однозначно определяет ребро графа (voice-assistant зависит от voice-base, потому что наследует его образ как `FROM`). Формально `depends_on` можно было бы вычислять автоматически из `base.service` и не заводить отдельное поле. Решение — оставить оба явно:
- `base.service` отвечает **какой образ взять за FROM** (влияет на формулу `BASE_IMAGE=...`);
- `depends_on` отвечает **в каком порядке собирать** (влияет на граф job'ов/волн, §3).

Сегодня оба значения у voice-assistant/supervisor совпадают 1:1 (`base.service: voice-base` ⇒ `depends_on: [voice-base]`). Разделение полей — задел на случай, если появится сервис, которому нужен порядок сборки без наследования образа (сегодня такого нет — не подтверждено ни одним примером в репозитории, чисто прогноз). Guard-тест (§6) проверяет инвариант «если `base.service` задан, он обязан быть единственным элементом `depends_on`» — это не даёт двум полям разойтись молча.

### 2.4 Расхождение имени build-arg для hash

`robot-state-publisher` считает хеш в build-arg `URDF_FILES_HASH` (`L-Build Main Pi Services.yml:94-104,118`), все остальные сервисы с source-hash — в `SOURCE_HASH`. Это исторически разное имя для одной и той же механики (инвалидация кэша buildx по хешу исходников). Манифест не пытается унифицировать имя (это было бы поведенческим изменением Dockerfile'а robot-state-publisher, вне рамок этого плана) — поле `source_hash.arg` явно хранит фактическое имя per-сервис, генератор подставляет его как есть.

### 2.5 `pre_build: fetch_hailort_wheel` — где манифест сознательно НЕ декларативен

`vision-hailo` перед сборкой копирует `hailort-4.24.0-cp310-cp310-linux_aarch64.whl` и `libhailort.so.4.24.0` из `/opt/rob_box/vendor` (с fallback на `/tmp`) в `docker/vision/vision-hailo/wheels/` (`L-Build Vision Pi Services.yml:507-522`). Это не параметризуемый build-arg, а императивный bash-шаг с двумя путями и явным `exit 1` при отсутствии файла. Пытаться выразить его в YAML-данных (список путей, список fallback'ов, текст сообщения об ошибке) — это не рычаг, а маскировка императивной логики под данные: она не станет проще для чтения, зато сломает предположение «манифест — только декларативные факты, разница видна в diff одной строкой».

Решение: `pre_build: fetch_hailort_wheel` — маркер-факт «у этого сервиса есть предсборочный хук», сама логика хука остаётся кодом генератора/workflow (по имени хука), не данными манифеста. Если появится второй сервис с похожей потребностью — тогда имеет смысл разбирать, есть ли общий адаптер (например, «скопировать вендорный артефакт из списка путей с fallback»); с одним примером обобщать преждевременно.

### 2.6 Полный манифест — Vision Pi (11 сервисов)

```yaml
pis:
  vision:
    services:
      oak-d:
        dockerfile: docker/vision/oak-d/Dockerfile
        build_context: docker/vision
        base:
          family: depthai
        tag:
          ros_distro: true
        image_versions: OAK_D_TAG

      led-matrix:
        dockerfile: docker/vision/led_matrix/Dockerfile
        build_context: "."
        base:
          family: ros2-zenoh
        submodule_sha: src/ros2leds
        tag:
          ros_distro: true
        image_versions: LED_MATRIX_TAG

      ceiling-camera:
        dockerfile: docker/vision/ceiling-camera/Dockerfile
        build_context: docker/vision
        base:
          family: ros2-zenoh
        tag:
          ros_distro: true
        image_versions: CEILING_CAMERA_TAG

      voice-base:
        dockerfile: docker/vision/voice_base/Dockerfile
        build_context: docker/vision/voice_base
        base:
          family: rtabmap
        tag:
          ros_distro: true
        image_versions: false   # НЕ в tag_and_push/verify сегодня; без комментария в коде — см. §1.3, §11

      voice-resources:
        dockerfile: docker/vision/voice_resources/Dockerfile
        build_context: docker/vision
        base: null
        tag:
          ros_distro: true
        image_versions: VOICE_RESOURCES_TAG
        # ADR-0111 (2026-09-15): сервис целиком подлежит удалению из прод-стека.
        # Манифест фиксирует ТЕКУЩЕЕ состояние; удаление узла — отдельный PR
        # после того, как devops-карточка t_ca7fa165 выберет §2.4 или §2.5 ADR-0111.

      voice-assistant:
        dockerfile: docker/vision/voice_assistant/Dockerfile
        build_context: "."
        base:
          service: voice-base
        depends_on: [voice-base]
        source_hash:
          arg: SOURCE_HASH
          groups:
            - paths:
                - src/rob_box_voice/rob_box_voice
                - src/rob_box_animations/rob_box_animations
                - src/rob_box_core
              extensions: [".py"]
        tag:
          ros_distro: true
        image_versions: VOICE_ASSISTANT_TAG

      telegram-bot:
        dockerfile: docker/vision/telegram_bot/Dockerfile
        build_context: "."
        base:
          family: ros2-zenoh
        source_hash:
          arg: SOURCE_HASH
          groups:
            - paths:
                - src/rob_box_telegram/rob_box_telegram
                - src/rob_box_core
              extensions: [".py"]
        tag:
          ros_distro: true
        image_versions: TELEGRAM_BOT_TAG

      supercollider:
        dockerfile: docker/vision/supercollider/Dockerfile
        build_context: docker/vision/supercollider
        base: null
        tag:
          ros_distro: false   # единственный сервис с нестандартным форматом тега
        image_versions: false   # документировано в коде: "tag naming is inconsistent"

      supervisor:
        dockerfile: docker/vision/supervisor/Dockerfile
        build_context: "."
        base:
          service: voice-assistant
        depends_on: [voice-assistant]
        source_hash:
          arg: SOURCE_HASH
          groups:
            - paths:
                - src/rob_box_supervisor/rob_box_supervisor
                - src/rob_box_core
              extensions: [".py"]
        tag:
          ros_distro: true
        image_versions: false   # НЕ в tag_and_push/verify сегодня; без комментария в коде — см. §1.3, §11

      quest:
        dockerfile: docker/vision/quest/Dockerfile
        build_context: "."
        base:
          family: ros2-zenoh
        source_hash:
          arg: SOURCE_HASH
          groups:
            - paths: [src/rob_box_quest/rob_box_quest]
              extensions: [".py"]
            - paths: [src/rob_box_quest/webxr_client]
              extensions: [".ts", ".json", ".html"]
            - paths: [src/rob_box_core]
              extensions: [".py"]
        tag:
          ros_distro: true
        image_versions: false   # НЕ в tag_and_push/verify сегодня; без комментария в коде — см. §1.3, §11

      vision-hailo:
        dockerfile: docker/vision/vision-hailo/Dockerfile
        build_context: "."
        base:
          family: ros2-zenoh
        source_hash:
          arg: SOURCE_HASH
          groups:
            - paths: [src/rob_box_perception/rob_box_perception]
              extensions: [".py"]
            - paths: [src/rob_box_perception_msgs]
              extensions: [".msg", "CMakeLists.txt", "package.xml"]
        build_args:
          HAILO_INSTALL_BINDING: whl
        pre_build: fetch_hailort_wheel
        tag:
          ros_distro: true
        image_versions: VISION_HAILO_TAG
```

### 2.7 Полный манифест — Main Pi (8 сервисов)

```yaml
pis:
  main:
    services:
      robot-state-publisher:
        dockerfile: docker/main/robot_state_publisher/Dockerfile
        build_context: "."
        base:
          family: ros2-zenoh
        source_hash:
          arg: URDF_FILES_HASH   # не SOURCE_HASH — см. §2.4
          groups:
            - paths: [src/rob_box_description]
              extensions: [".xacro", ".urdf", "package.xml", "CMakeLists.txt"]
        tag:
          ros_distro: true
        image_versions: ROBOT_STATE_PUBLISHER_TAG

      rtabmap:
        dockerfile: docker/main/rtabmap/Dockerfile
        build_context: docker/main/rtabmap
        base:
          family: rtabmap
        tag:
          ros_distro: true
        image_versions: RTABMAP_TAG

      twist-mux:
        dockerfile: docker/main/twist_mux/Dockerfile
        build_context: docker/main/twist_mux
        base:
          family: ros2-zenoh
        tag:
          ros_distro: true
        image_versions: TWIST_MUX_TAG

      teleop:
        dockerfile: docker/main/teleop/Dockerfile
        build_context: "."
        base:
          family: ros2-zenoh
        tag:
          ros_distro: true
        image_versions: false   # документировано в коде: "still uses IMAGE_TAG in docker-compose"

      ros2-control:
        dockerfile: docker/main/ros2_control/Dockerfile
        build_context: "."
        base:
          family: rtabmap   # не ros2-zenoh, несмотря на имя сервиса — L-Build Main Pi Services.yml:236
        submodule_sha: src/vesc_nexus
        tag:
          ros_distro: true
        image_versions: ROS2_CONTROL_TAG

      nav2:
        dockerfile: docker/main/nav2/Dockerfile
        build_context: docker/main/nav2
        base:
          family: ros2-zenoh
        tag:
          ros_distro: true
        image_versions: NAV2_TAG

      lslidar:
        dockerfile: docker/main/lslidar/Dockerfile
        build_context: docker/main
        base:
          family: pcl
        tag:
          ros_distro: true
        image_versions: LSLIDAR_TAG

      perception:
        dockerfile: docker/main/perception/Dockerfile
        build_context: "."
        base:
          family: ros2-zenoh
        tag:
          ros_distro: true
        image_versions: PERCEPTION_TAG
```

19 сервисов итого (11 + 8) — ровно те, у которых сегодня есть `build-*` job в двух матричных workflow. `zenoh-router` и base-образы (`ros2-zenoh`, `depthai`, `pcl`, `rtabmap` как base-образы, не как сервис Main Pi) присутствуют только в `L-Build Single Service.yml` и намеренно вне области этого манифеста первой версии — см. §11.

---

## §3. Matrix и двухуровневый граф зависимостей

### 3.1 Ограничение GitHub Actions (проверено по документированному поведению `fromJSON`/`needs`, не по эксперименту в этом репозитории — в репозитории ни одного примера динамической matrix нет, см. 3.2)

`strategy.matrix: fromJSON(needs.prepare.outputs.matrix)` разворачивает один job в N параллельных инстансов — по одному на элемент JSON-массива. Но `needs:` самого job'а фиксируется **один раз на весь job**, до разворачивания matrix, и не может ссылаться на конкретный элемент matrix другого job'а. То есть невозможно написать «инстанс matrix с `name: voice-assistant` должен ждать инстанс matrix с `name: voice-base` того же job'а» — `needs:` работает на уровне job ID, а не на уровне строки матрицы. Это подтверждённое (документированное) ограничение платформы, а не гипотеза.

Следствие: цепочку `voice-base → voice-assistant → supervisor` нельзя выразить одной matrix-job с зависимостями «внутри» матрицы. Нужен один из способов ниже.

### 3.2 В репозитории нет прецедента

Проверено по всем `.github/workflows/*.yml`:
```
$ grep -rln "fromJSON\|strategy:\s*$" .github/workflows/
→ codeql.yml (strategy.matrix.language — статический список языков, НЕ fromJSON,
  НЕ per-service build)
→ "L-Build Vision Pi Services.yml" — ложное совпадение (regex поймал слово
  "led-matrix" по подстроке "matrix", а не ключевое слово strategy.matrix)
```
То есть динамический `matrix: fromJSON(...)` в этом репозитории **нигде не используется** — это не рефакторинг существующего паттерна, а введение нового технического приёма. Это отдельный риск, который стоит явно взвесить при выборе между вариантами ниже (незнакомый механизм — дороже в отладке первое время).

### 3.3 Варианты

**Вариант A — «волны» (layered matrix).** Топологическая сортировка `depends_on` (алгоритм Кана) раскладывает сервисы по уровням: уровень 0 — узлы без зависимостей, уровень k — узлы, все зависимости которых лежат на уровне < k. Генератор эмитит N job'ов `build-wave-0`, `build-wave-1`, ..., каждый — `strategy.matrix: fromJSON(needs.prepare.outputs.wave_<k>)`, `needs: [prepare, build-wave-<k-1>]` (кроме волны 0, которая зависит только от `prepare`). Для сегодняшнего графа Vision Pi: волна 0 — 9 сервисов (все, кроме voice-assistant и supervisor), волна 1 — `voice-assistant`, волна 2 — `supervisor`. Main Pi — одна волна на все 8.

Плюс: общий, масштабируется на любую глубину графа без переписывания workflow-шаблона — глубина живёт только в манифесте (в `depends_on`), число job'ов в workflow-файле вычисляется генератором.

Минус — огрубление графа: `build-wave-1` (в котором сидит только `voice-assistant`) обязан ждать завершения **всей** волны 0 (9 сервисов), хотя `voice-assistant` реально зависит только от одного из них (`voice-base`). Сегодня `needs: [prepare, build-voice-base]` (`L-Build Vision Pi Services.yml:188`) — точный, а не огрублённый граф. Переход на волны — это осознанный откат точности графа ради масштабируемости механизма; на self-hosted раннере с ограничением «до 8 одновременно» (комментарий `L-Build Vision Pi Services.yml:4`) эффект скорее всего небольшой (сборки и так частично сериализуются очередью раннера), но это не измерено, а предположение.

**Вариант B — именованные job'ы для зависимых сервисов, matrix только для независимых.** Сервисы без `depends_on` (16 из 19: все Main Pi + 9 из 11 Vision Pi) собираются одной matrix-job (`build-vision`/`build-main` со `strategy.matrix: fromJSON(needs.prepare.outputs.matrix)`). Сервисы с `depends_on` (`voice-assistant`, `supervisor` — оба Vision Pi) генератор эмитит как отдельные именованные job'ы, по одному на сервис, с явным `needs: [prepare, build-<dep>]` — то есть **тем же способом, каким они устроены сегодня**, только текст job'а не пишется руками, а рендерится шаблоном из данных манифеста.

Плюс: нулевое огрубление графа (сохраняет сегодняшнюю точность `needs`), минимальный диф от текущего поведения, для читателя PR разница видна как «эти два job'а раньше были скопипащены руками, теперь генерируются» — остальное не меняется концептуально.

Минус: не масштабируется автоматически на произвольную глубину — если в будущем появится цепочка длиннее двух звеньев с ветвлением, придётся либо добавлять третий именованный job вручную в шаблон генератора (мелкое расширение при текущем размере), либо переключаться на вариант A.

### 3.4 Рекомендация

Взять **вариант B** для первой реализации. Обоснование: у графа сегодня ровно два узла с `depends_on` (`voice-assistant`, `supervisor`), оба — на Vision Pi, оба — линейная цепочка без ветвления. Вводить общий механизм волн (вариант A) для двух узлов — рычаг отрицательный: код генератора и workflow-шаблон становятся сложнее (два типа job'ов вместо одного plus именованные), а выигрыш (масштабируемость) не используется прямо сейчас нигде. Вариант B к тому же **не огрубляет** существующий build-граф — сохраняет то самое свойство «supervisor ждёт именно voice-assistant, не весь набор», из-за отсутствия которого уже случалась гонка (комментарий `L-Build Vision Pi Services.yml:374-382`, инцидент 09.09 — supervisor захватил старые слои `rob_box_core`, потому что собрался раньше voice-assistant).

Явно фиксируется: если граф вырастет (третий уровень зависимости, или зависимость с ветвлением — например, два сервиса зависят от одного и того же третьего), правильный следующий шаг — не городить третий именованный job вручную, а переключить генератор на вариант A (алгоритм уже описан в 3.3, реализуется как отдельная функция `topo_layers(services) -> list[list[str]]` в `scripts/ci/gen_build_matrix.py`, не задействованная в MVP, но покрытая unit-тестом — «когда понадобится, переключение — это замена шаблона, а не новое архитектурное решение»).

---

## §4. Скрипт-генератор

### 4.1 Расположение

`scripts/ci/gen_build_matrix.py`. Обоснование места: `scripts/ci/` уже держит скрипты именно этого класса — обвязку CI, читающую/пишущую `.image-versions.*` (`push-image-versions.sh`, `commit-image-versions.sh`, `check_image_versions_usage.sh`) и guard над `G-Run Tests.yml` (`validate_test_packages.py` — тоже Python, тоже парсит workflow YAML через `yaml.safe_load`, см. §6). Манифест логически ближе к семейству «данные о build-графе», чем к `scripts/agent_flow/` (agent-flow — про merge-gate и ADR-namespace, не про docker build).

### 4.2 Вход/выход

```
gen_build_matrix.py --manifest docker/build-manifest.yaml --pi vision --mode matrix
  → JSON на stdout: [{"name": "oak-d", "dockerfile": "...", ...}, ... 9 элементов без depends_on ...]

gen_build_matrix.py --manifest docker/build-manifest.yaml --pi vision --mode chained
  → JSON на stdout: [{"name": "voice-assistant", "depends_on": ["voice-base"], ...},
                     {"name": "supervisor", "depends_on": ["voice-assistant"], ...}]
  # топологически отсортировано — потребитель (workflow-шаблон) просто
  # проходит список по порядку и знает, что depends_on уже эмитирован раньше

gen_build_matrix.py --manifest docker/build-manifest.yaml --pi vision --mode tags
  → JSON: {"OAK_D_TAG": "oak-d", "LED_MATRIX_TAG": "led-matrix", ...}
  # используется update-image-versions вместо ручного списка tag_and_push/verify/sed
```
Библиотечная часть (`load_manifest`, `services_for_pi`, `topo_layers`, `independent_services`, `chained_services`, `image_versions_map`) — чистые функции без побочных эффектов, без сетевых вызовов, без обращения к `git`/`docker`. Это то, что делает генератор тестируемым без CI (см. 4.3).

### 4.3 Тестирование без запуска CI

Тесты — `scripts/ci/tests/test_gen_build_matrix.py`, запускаются `python -m pytest scripts/ci/tests/test_gen_build_matrix.py` локально или в `G-Lint Code.yml` (см. §6). Стиль — как у соседних тестов в `scripts/ci/tests/` (`test_check_image_versions_usage.sh` строит tempdir с фейковыми файлами и проверяет exit code/вывод) и как у `tests/unit/test_workflow_refactor_acceptance.py` (парсинг реального YAML через `yaml.safe_load`, `pytest.mark.parametrize` по списку сервисов). Конкретные тестовые случаи:
- манифест с одним независимым сервисом → `independent_services` вернул этот один сервис, `chained_services` пуст.
- манифест с `voice-base → voice-assistant → supervisor` (три сервиса, два ребра) → `chained_services` вернул `[voice-assistant, supervisor]` в этом порядке (топологически корректном); `independent_services` вернул `[voice-base]`.
- манифест с циклом (`a` зависит от `b`, `b` зависит от `a`) → `load_manifest`/`topo_layers` кидают понятную ошибку, а не зависают/падают в `RecursionError`.
- `base.service` задан, но `depends_on` не содержит того же сервиса (нарушение инварианта §2.3) → ошибка валидации с текстом, называющим оба поля и оба значения.
- `image_versions` содержит имя переменной, которая не является валидным идентификатором `*_TAG` (например, содержит пробел) → ошибка валидации.
- прогон на **реальном** будущем `docker/build-manifest.yaml` (после его появления в репозитории) — smoke-тест «манифест парсится, не падает, 19 сервисов на входе».

### 4.4 Что генератор НЕ делает

Не пишет workflow YAML-файлы на диск (не codegen-в-git). На первом этапе (§5, Phase 1) манифест существует рядом с уже написанными руками workflow и только сверяется с ними guard-тестом. На втором этапе (§5, Phase 2+) генератор вызывается **во время выполнения** `prepare`-job'а в CI (через `python scripts/ci/gen_build_matrix.py ... >> $GITHUB_OUTPUT`), а не во время правки репозитория — то есть у него нет режима «перезаписать workflow-файл», это осознанно не рассматривается в этом плане (альтернатива «codegen с диффом» обсуждалась и отклонена — см. §11, где перечислено как открытый вопрос на случай, если ревью не согласится).

---

## §5. Пошаговый план реализации

Правило ADR-AF-0013 (инкрементальная поставка): каждый этап — отдельный PR, e2e-проверяемый в своих рамках, без «одного большого коммита на всё».

### Phase 0 — подготовка (без кода)
Не входит в объём этого документа: решение по §11 «Открытые вопросы» (особенно — почему `voice-base`/`supervisor`/`quest` исключены из `image_versions`) должно быть получено ДО Phase 1, потому что манифест обязан либо честно зафиксировать `image_versions: false` с пометкой «причина не найдена», либо (если выяснится, что это баг) — сразу манифестировать `true` и одним PR чинить `tag_and_push`/`verify_in_registry`. Manifest, зафиксировавший баг как «намеренное поведение» без разметки, был бы хуже, чем нынешнее необъяснённое расхождение — потому что перестал бы выглядеть подозрительным.

### Phase 1 — манифест + guard, поведение CI не меняется
1. Создать `docker/build-manifest.yaml` (содержимое — §2.6/2.7, с пометками `image_versions: false # причина не найдена` там, где Phase 0 не даст ответа).
2. Создать `scripts/ci/gen_build_matrix.py` (библиотечные функции + CLI, §4).
3. Создать `scripts/ci/tests/test_gen_build_matrix.py` (§4.3).
4. Создать guard-тест «манифест ⇔ реальные workflow» (§6) — **этот тест обязан сначала СОЙТИСЬ** (манифест списан 1:1 с текущего состояния файлов, поэтому по построению должен пройти сразу; если не проходит — где-то в §2.6/2.7 ошибка транскрипции, а не расхождение в реальности).
5. Заодно (раз уже здесь) — актуализировать `EXPECTED_VISION_JOBS` в `tests/unit/test_workflow_refactor_acceptance.py`, добавив `build-vision-hailo` (см. §1.7) — иначе Phase 1 PR будет заведомо содержать заведомо известный красный тест по соседству, что плохо сочетается с «первый этап не меняет поведения» (тест на самом деле должен был быть зелёным всё это время). Подключить этот тестовый файл к `G-Lint Code.yml` (сейчас не подключён нигде — см. §1.7) — отдельным пунктом того же PR, hard gate, по образцу существующих шагов `python-lint`.
6. `docker/build-manifest.yaml` НЕ влияет ни на один workflow. Ни один `.yml` в `.github/workflows/` не редактируется, кроме добавления guard-шага и починки `tests/unit/test_workflow_refactor_acceptance.py`.

Acceptance Phase 1: CI зелёный, guard-тест подключён и зелёный, ручной прогон `L-Build Vision/Main Pi Services.yml` (workflow_dispatch) даёт те же образы, что и до PR (потому что файлы не менялись).

### Phase 2 — независимые сервисы (вариант B, §3.4) переходят на matrix
1. `prepare`-job в `L-Build Vision Pi Services.yml`/`L-Build Main Pi Services.yml` получает шаг, вызывающий `gen_build_matrix.py --mode matrix` и `--mode chained`, пишет оба JSON в `$GITHUB_OUTPUT`.
2. 9 (Vision) / 8 (Main) независимых `build-*` job'ов заменяются одной matrix-job, использующей `fromJSON(needs.prepare.outputs.matrix)` и композит-экшен `l-build-service` (без изменений в `action.yml`).
3. `voice-assistant` и `supervisor` (Vision Pi) остаются именованными job'ами, но их тело (checkout + composite call) теперь дословно совпадает с шаблоном, использованным для генерации остальных — различие только в источнике данных (не matrix, а прямая подстановка одного элемента `chained`-списка).
4. Guard-тест из Phase 1 (§6) на этом этапе меняет режим: раньше он сверял манифест с текстом workflow, теперь сверяет манифест с **параметрами**, с которыми реально были вызваны job'ы (доступно через `yaml.safe_load` — matrix-job тоже статически виден в YAML как `strategy.matrix: fromJSON(...)`, конкретные элементы — нет; здесь guard проверяет, что `needs.prepare.outputs.matrix` вычисляется из того же `docker/build-manifest.yaml`, не тестирует конкретные значения рантайма — те покрыты Phase 1 тестами генератора).

Acceptance Phase 2: `actionlint`/`yamllint` проходят (актуально добавить `actionlint` как новый hard-gate шаг — сейчас в репозитории есть только `yamllint` с `continue-on-error: true`, `actionlint` не используется нигде, проверено `grep -rn actionlint .github/ scripts/` → пусто); dispatch сборки одного сервиса (через matrix-фильтр или через `L-Build Single Service.yml`, не меняется этим планом) работает; полный прогон Vision-сборки → идентичные теги (побайтовое сравнение списка тегов `docker images` до/после).

### Phase 3 — `tag_and_push`/`verify_in_registry`/`sed` через манифест
1. `update-image-versions` job читает `gen_build_matrix.py --mode tags` вместо захардкоженного списка (§1.2 — 585-591, 619-626, 640-648).
2. Убирается ручное перечисление — цикл `for SVC_TAG in $(python scripts/ci/gen_build_matrix.py --mode tags ...)`.
3. Явно НЕ трогается: сам механизм push (SHA-тег, commit в develop, `scripts/ci/push-image-versions.sh`) — за пределами области этого плана, это территория ADR-0094 и параллельной задачи `docs/plans/2026-09-15-image-versions-seam.md` (см. §10).

Acceptance Phase 3: `.image-versions.dev` после прогона на develop содержит те же 7 полей (Vision)/7 полей (Main), что и раньше, с корректным SHA-тегом; `RTABMAP_SYNC_TAG`/`MICRO_ROS_AGENT_TAG` (phantom) — либо остаются нетронутыми (генератор их не пишет, потому что ни у одного сервиса манифеста `image_versions` не равно этим именам — это САМО ПО СЕБЕ фиксирует их как phantom без отдельного PR), либо удаляются отдельным PR по процедуре ADR-0094 §3.3 (осознанный выбор — не блокирует Phase 3).

### Phase 4 — `L-Build Single Service.yml` (опционально, отдельный PR)
Не детализируется в этом плане — см. §11. Помечено как логичное продолжение, не как обязательство этого плана.

---

## §6. Guard-тест

Стиль — по образцу `scripts/ci/validate_test_packages.py` (парсит `G-Run Tests.yml` через regex, сравнивает с discovery по файловой системе, hard gate в `G-Lint Code.yml:103-109`) и `tests/unit/test_workflow_refactor_acceptance.py` (парсит workflow через `yaml.safe_load`, параметризованные assert'ы по списку job'ов). Оба уже задают принятый в репозитории паттерн: **guard — это Python-тест, который читает workflow как данные (`yaml.safe_load`), а не текстовый grep**, за исключением мест, где YAML-парсинг неудобен (многострочные bash-блоки — там ADR-0057-стиль regex поверх сырого текста, как в `validate_test_packages.py`).

Новый файл: `scripts/ci/tests/test_service_manifest_sync.py` (Python, не bash — потому что нужен `yaml.safe_load` над тремя файлами разом, что естественнее в Python, чем в bash+grep). Он проверяет (Phase 1, до перехода воркфлоу на чтение манифеста):

1. `set(манифест.vision.services) == {job.removeprefix("build-") for job in L-Build Vision Pi Services.yml.jobs if job.startswith("build-")}` (и то же для main). Это буквально то же самое сравнение, которое уже делает `test_vision_workflow_all_build_jobs_present` (§1.7) — предлагается **не дублировать** его, а расширить существующий тестовый файл третьим источником истины (манифест) вместо жёстко вшитого `EXPECTED_VISION_JOBS`/`EXPECTED_MAIN_JOBS` — то есть после Phase 1 `EXPECTED_VISION_JOBS = set(manifest["pis"]["vision"]["services"])`, и эта константа перестаёт быть местом, которое можно забыть обновить (см. §7 — это тоже строки на удаление).
2. Для каждого сервиса с `depends_on`: реальный `needs:` соответствующего job'а в workflow содержит `prepare` + все элементы `depends_on` манифеста (сверка §2.3 инварианта).
3. Множество сервисов в `tag_and_push`/`verify_in_registry` (регэксп-парсинг текста, как в `validate_test_packages.py`, потому что это bash-heredoc, не YAML-структура) равно множеству сервисов с `image_versions != false` в манифесте.
4. Множество полей в `sed -e "s|^X_TAG=.*"` равно множеству значений `image_versions` в манифесте (обнаруживает phantom-поля вроде `RTABMAP_SYNC_TAG` автоматически — первый прогон теста ДОЛЖЕН упасть на этом пункте, если манифест не содержит фиктивного сервиса под phantom-тег, что и является ожидаемым и документированным поведением: тест ловит `RTABMAP_SYNC_TAG` как «есть в sed, нет владельца в манифесте» и требует явного исключения через whitelist-механизм, аналогичный `IV_KNOWN_PHANTOMS` в `check_image_versions_usage.sh`, ADR-0094 §3.2).
5. `tag.ros_distro: false` в манифесте есть ровно у тех сервисов, чей блок `tags:` в композит-вызове не содержит `${{ env.ROS_DISTRO }}` (сегодня — только `supercollider`).

Hard gate (без `continue-on-error`), подключается в `G-Lint Code.yml` рядом с `python-lint` (тот же паттерн, что ADR-0057 применил для `validate_adr_namespace.sh` — «рядом с уже существующим cc_budget.py», здесь — рядом с уже существующим `validate_test_packages.py`, тот же job, та же логика «почему hard gate, не warn-only»: ADR-0057 §1.2 явно документирует, что soft-warning уже испробован и не работает («ровно то, что есть сейчас — неэффективно»)).

---

## §7. Что удаляется

Deletion test (в духе «если функциональность не нужна, PR должен показывать отрицательный `git diff --stat`, а не только добавления»):

| Место | Строк сегодня | Что уходит после Phase 2/3 | Оценка удаления |
|---|---|---|---|
| `L-Build Vision Pi Services.yml`, тела 11 `build-*` job'ов | ≈423 строки (97-537, посчитано построчно по каждому job'у) | 9 из 11 тел (независимые сервисы) заменяются на одну matrix-job; 2 (voice-assistant, supervisor) остаются именованными, но их текст перестаёт отличаться копипастой | Ориентировочно −300…−330 строк (9 тел, средний размер ≈35 строк, минус ≈70 строк на саму matrix-job) — **оценка**, требует перемера после реализации, не окончательное число |
| `L-Build Vision Pi Services.yml`, `tag_and_push`/`verify_in_registry`/`sed` (585-591, 619-626, 640-648) | 24 строки ручного перечисления | Заменяются циклом по выводу `gen_build_matrix.py --mode tags` (§5 Phase 3) | ≈ −15 строк |
| `L-Build Main Pi Services.yml`, тела 8 job'ов | ≈205 строк (78-324) | Все 8 — независимые, вся секция схлопывается в одну matrix-job | Ориентировочно −150…−170 строк |
| `L-Build Main Pi Services.yml`, `tag_and_push`/`verify`/`sed` (381-437) | 23 строки | Тот же цикл, что для Vision | ≈ −13 строк |
| `tests/unit/test_workflow_refactor_acceptance.py`, `EXPECTED_MAIN_JOBS`/`EXPECTED_VISION_JOBS` (47-69) | 23 строки захардкоженных множеств | Заменяются чтением из манифеста | ≈ −18 строк (константы), но тест приобретает зависимость на манифест — не чистое удаление, а перенос источника истины |

Итоговая оценка по двум build-workflow: от **≈500 до ≈550 строк YAML** уходит из `.github/workflows/L-Build {Main,Vision} Pi Services.yml` (из 710+508=1218 суммарно, то есть заметная доля, но не большинство — сводные job'ы `prepare`/`update-image-versions`/`summary` и их комментарии про race conditions и historical gotchas остаются, потому что несут знание, не связанное с составом списка сервисов). Это оценка методом суммирования размеров существующих job-блоков; точное число нужно перемерить после Phase 2/3 реализации (`git diff --stat` того PR — самая честная метрика, эта таблица — предварительная, не финальная).

`L-Build Single Service.yml` и `docs/guides/SINGLE_SERVICE_BUILD.md` этим планом **не трогаются** (Phase 4, вне объёма — §11).

---

## §8. Acceptance

- [ ] `actionlint` проходит на обоих build-workflow после Phase 2 (новый hard-gate шаг — сегодня `actionlint` в репозитории не используется нигде, только `yamllint` с `continue-on-error: true`, `G-Lint Code.yml:184-223`; добавление `actionlint` — часть Phase 2 PR, не отдельная задача).
- [ ] `yamllint` проходит (уже есть, `continue-on-error: true` — не блокирует сегодня; для этого PR можно либо оставить как есть, либо снять `continue-on-error` для конкретно build-workflow — решение Phase 2, не блокер плана).
- [ ] Guard-тест `scripts/ci/tests/test_service_manifest_sync.py` (§6) зелёный на Phase 1, продолжает быть зелёным на всех последующих фазах.
- [ ] `python -m pytest tests/unit/test_workflow_refactor_acceptance.py` зелёный (сейчас красный — см. §1.7 — Phase 1 обязан это починить).
- [ ] Dispatch сборки одного сервиса через `workflow_dispatch` (после Phase 2, через фильтр matrix или руками поднятый один инстанс) — работает, образ собирается и пушится в local registry с тем же тегом, что раньше.
- [ ] Полный прогон `L-Build Vision Pi Services.yml` на ветке разработки до и после Phase 2/3 — списки тегов, реально запушенных в `localhost:5000` (`curl .../v2/krikz/rob_box/tags/list`), совпадают (с точностью до нового SHA — сравнивается формула тега, не конкретное значение SHA).
- [ ] `docker/vision/.image-versions.dev` и `docker/main/.image-versions.dev` после прогона на develop содержат то же множество полей (кроме явно удалённых phantom-полей, если Phase 3 их чистит — тогда сравнение «то же множество минус явно удалённые»).

---

## §9. Риски и откат

| Риск | Обоснование | Митигация |
|---|---|---|
| Динамическая matrix (`fromJSON`) — новый для репозитория механизм, нет локального прецедента (§3.2) | Первая отладка нестандартной ошибки (например, неверный JSON от генератора) будет медленнее, чем с уже знакомым паттерном именованных job'ов | Phase 2 запускается через `workflow_dispatch` на некритичной ветке перед merge в develop; guard-тест Phase 1 ловит большинство ошибок форматирования манифеста ДО того, как они попадут в реальный CI-прогон |
| Вариант B (§3.4) не масштабируется на ветвящийся граф без ручной правки шаблона | Осознанный выбор ради простоты при текущем размере графа (2 зависимых узла) | Вариант A (волны) документирован и частично закладывается в генератор (`topo_layers`) как задел, переключение — не архитектурное решение заново, а смена шаблона |
| Manifest фиксирует необъяснённое исключение (`voice-base`/`supervisor`/`quest` из `image_versions`) как `false` без причины | Может законсервировать баг вместо того, чтобы его найти | Phase 0 явно требует решения ДО Phase 1 (см. §5); если решения нет к моменту реализации — манифест помечает `false # причина не найдена`, что как минимум не хуже сегодняшнего состояния (где причина тоже не найдена, но не видна вообще) |
| Расхождение манифеста и реальности после Phase 2/3 (кто-то поправит `action.yml` или добавит сервис мимо манифеста) | Тот же класс риска, который уже реализовался с `tests/unit/test_workflow_refactor_acceptance.py` (§1.7) — guard написан, но не подключён | Guard подключается как hard gate в `G-Lint Code.yml` в САМОМ Phase 1 PR, не отложенным follow-up — учтён урок ADR-0057 |
| Откат | Phase 1 не меняет поведение CI → откат тривиален (удалить манифест + guard, ничего в поведении build не менялось). Phase 2/3 меняют реальные workflow → откат = `git revert` конкретного PR, оба build-workflow возвращаются к явным job'ам; манифест из Phase 1 при этом можно оставить (не мешает, просто не используется) или откатить тем же revert | — |

---

## §10. Связь с ADR

**ADR-0094** (`.image-versions.*` SHA-tag push). Прямо касается Phase 3 — механизм `push-image-versions.sh`/`commit-image-versions.sh` и phantom-чекер `check_image_versions_usage.sh` не меняются этим планом; манифест **дополняет** ADR-0094 §3.2 (phantom-чекер по всем `*_TAG` во всех `.image-versions.*` файлах, независимо от происхождения) более узким, но более точным guard'ом (§6 п.4 — конкретно сверяет `sed`-список с манифестом, а не только «используется ли переменная где-то ещё»). Конфликта нет — оба guard'а взаимно дополняющие (defense-in-depth, тот же принцип, что ADR-0057 §4 описывает для CI+merge-gate).

**ADR-0111** (`voice-resources` — bake-into-voice-base). Затрагивает состав манифеста напрямую: если devops-карточка `t_ca7fa165` реализует §2.1 ADR-0111 (сервис `voice-resources` удаляется целиком, `build-voice-resources` job уходит из `L-Build Vision Pi Services.yml:239-265,540,588,621`), это уменьшает манифест Vision Pi с 11 до 10 сервисов. Порядок работ: **манифест как единственный источник истины стоит вводить ПОСЛЕ или ОДНОВРЕМЕННО с ADR-0111** — если Phase 1 этого плана заведён раньше реализации ADR-0111, придётся редактировать манифест дважды (сначала зафиксировать 11 сервисов «как есть» с явной пометкой TODO на удаление — что и сделано в §2.6 у `voice-resources` — потом убрать сервис отдельным PR). Явного блокера нет, но это на заметку при планировании очерёдности PR (не блокер для Phase 0/1).

**ADR-0057** (ADR-namespace collision guard как hard gate). Не про этот манифест напрямую, но задаёт **процессный прецедент**, на который этот план опирается дважды: (1) в §6 — «guard рядом с уже существующим guard'ом того же класса, тот же job, hard gate, не warn-only»; (2) в §1.7/§5/§9 — живой найденный пример именно того паттерна отказа, который ADR-0057 описывает («скрипт написан, но не подключён к CI» — только здесь это не bash-скрипт коллизий ADR, а Python guard-тест build-workflow).

**ADR-AF-0013** (инкрементальная поставка). Структура §5 (4 фазы, каждая — отдельный PR, e2e-проверяемая сама по себе, Phase 1 не меняет поведения) прямо следует правилу ADR-AF-0013 §4 («каждый компонент — отдельный PR с e2e», применительно здесь не к переносу кода из старой ветки, а к переносу знания из workflow в данные).

**Параллельная задача `docs/plans/2026-09-15-image-versions-seam.md`.** Файл **не найден в репозитории** на момент написания этого плана (проверено: `find docs/plans -iname "*image-versions*"` → пусто, `ls docs/plans/ | grep 2026-09-15` → пусто — в `docs/plans/` вообще нет файлов с датой 2026-09-15, последний по дате в каталоге — `2026-09-10-theme-driven-arranger-design.md`). Это не подтвердившийся факт из задания — либо задача ещё не сдана в момент, когда пишется этот план (параллельная сессия), либо путь/имя указаны неточно. Стыковка по смыслу (не по прочитанному документу, а по тому, что уже известно из ADR-0094 о слое `.image-versions.*`): при любом исходе разбора `.image-versions` (например, смена схемы имён переменных, объединение файлов dev/test/latest, или их полная замена на что-то ещё) — граница ответственности этого плана заканчивается на поле `image_versions: <ИМЯ_ПЕРЕМЕННОЙ | false>` внутри `docker/build-manifest.yaml`. Если схема `.image-versions.*` поменяется, меняется **формат значения** этого поля (например, вместо голого имени переменной — структура `{file: dev, var: OAK_D_TAG}`), но не сам факт, что у каждого сервиса есть один булев/структурный признак «участвует ли в записи версии». Рекомендация: реализовывать Phase 3 этого плана (§5) уже после того, как параллельная задача о `.image-versions` будет видна в репозитории и её решение станет известно — чтобы не писать код для формата, который тут же поменяется.

---

## §11. Открытые вопросы

1. **Почему `voice-base`, `supervisor`, `quest` исключены из `tag_and_push`/`verify_in_registry`/`.image-versions.dev`, но `supercollider` — с объяснением, а эти три — без?** (§1.3, §2.6). Нужно решение до Phase 1 (см. §5 Phase 0) — это или намеренное решение (тогда — какое обоснование, чтобы вписать его в манифест как комментарий, а не как молчание), или забытое расхождение (тогда Phase 1 манифест сразу фиксирует `image_versions: true` и одним PR чинит три списка).
2. **Область манифеста первой версии — только два матричных workflow, или сразу включать `L-Build Single Service.yml` и base-образы (`ros2-zenoh`, `depthai`, `pcl`, `rtabmap`, `zenoh-router`)?** Этот план сознательно ограничил Phase 1-3 девятнадцатью сервисами, у которых есть `build-*` job. `L-Build Single Service.yml` содержит куда более богатую матрицу (main+vision+base, 21 пункт выбора, включая мёртвый `apriltag`, §1.6) — но это отдельный день работы (другая структура workflow, другой набор проблем — там нет параллельной сборки, есть выбор одного сервиса из dropdown). Рекомендация — Phase 4 отдельным PR после того, как Phase 1-3 этого плана докажут себя на двух матричных workflow.
3. **`apriltag` в `L-Build Single Service.yml` — удалить как мёртвый пункт, или он зарезервирован под будущее (например, отдельный от oak-d режим)?** Не решается этим планом (вне области, §1.6) — вопрос для владельца сервиса/devops, отдельная карточка вне этого документа.
4. **Codegen-с-диффом (writing workflow YAML files to disk from the manifest, checked-in and diff-guarded) как альтернатива runtime `fromJSON`.** Этот план выбрал runtime-генерацию (§4.4) по прямому указанию формулировки задания («GitHub Actions умеет `strategy.matrix` из `fromJSON`»), но вариант «манифест + шаблон → сгенерированный и закоммиченный `.yml`, guard проверяет `git diff` после regen» тоже валиден и местами предпочтительнее для читаемости PR (полный текст job'а виден в diff, а не только `fromJSON(...)` на рантайме). Не выбран в этом плане, но стоит explicitly зафиксировать как рассмотренную и отклонённую (в пользу задания) альтернативу, если ревью попросит пересмотреть.
5. **Нужно ли манифесту поле `platform` per-сервис, или общий `defaults.platform: linux/arm64` достаточен?** Сегодня все 19 сервисов используют один и тот же `linux/arm64` (`action.yml:43-46`, default). Поле в схеме (§2.2) присутствует на уровне `defaults`, per-service override не добавлен, потому что не подтверждён ни одним примером — если понадобится (например, кросс-платформенная сборка), добавляется как `override.platform` без миграции остальных полей.

---

## Словарь — предложение для `CONTEXT.md`

Термин **«манифест сервисов»** не встречается в текущем `CONTEXT.md` (проверено — `грep -in "манифест" CONTEXT.md` → пусто). Предлагаемая формулировка для добавления (раздел «Инструменты» или новый раздел «Сборка», по усмотрению следующего ревью CONTEXT.md, не решается этим планом):

> **Манифест сервисов** (`docker/build-manifest.yaml`) — интерфейс, из которого CI выводит matrix сборки, граф зависимостей между образами и список тегов `.image-versions.*`. Реализация (`scripts/ci/gen_build_matrix.py`) — единственное место, которое превращает эти данные в шаги workflow; сами workflow-файлы не хранят список сервисов, а читают его через этот модуль.
> _Не путать_ с `docker-compose.yaml`: манифест описывает **сборку** образов (CI, build-time), compose описывает **запуск** контейнеров (runtime, Pi). Один сервис обычно присутствует в обоих, но правится по разным причинам.

Использованный словарь этого документа: **модуль** (манифест + генератор как единица «интерфейс+реализация»), **интерфейс** (YAML-схема, §2.2), **реализация** (генератор + guard, §4, §6), **глубокий/мелкий** (§2.1 — манифест как глубокий интерфейс со скрытой сложностью формирования тегов/build-args; §2.5 — сознательный отказ делать `pre_build`-хук «глубоким» декларативно, чтобы не размазывать императив по данным), **шов** (§2.1 — манифест как шов между данными и поведением; §1.7 — сам guard-тест как шов, который уже был прорезан, но не подключён), **адаптер** (§4.4, §3.4 — workflow-job как тонкий адаптер поверх матрицы, не владеющий знанием о сервисах), **рычаг** (§3.4, §7 — обоснование выбора варианта B через отношение «сложность механизма / польза при текущем размере графа»), **локальность** (§2.1, §7 — один факт о сервисе живёт в одном месте манифеста вместо шести).

---

## §12. Статус реализации, 21.09.2026

| Фаза | Статус |
|---|---|
| Phase 0 | Ответа на §11 №1 (почему `voice-base`/`supervisor`/`quest` исключены из `image_versions`) так и нет. Манифест честно держит `image_versions: false # причина не найдена`. |
| Phase 1 | Сделана ранее (`4d2aa0cd8`). |
| Phase 2 | Сделана. |
| Phase 3 | Сделана. |
| Phase 4 (`L-Build Single Service.yml`) | НЕ делалась, как и планировалось. |

### §12.1. Отклонение от §5 Phase 2, принятое осознанно

План говорил: «9 из 11 Vision в matrix, именованными остаются
`voice-assistant` и `supervisor`». Так нельзя без изменения поведения.
`build-voice-assistant` имеет `needs: [prepare, build-voice-base]`, а
`needs:` ссылается только на job ID и не умеет ссылаться на элемент матрицы
(§3.1). Если `voice-base` уезжает в matrix, единственный вариант —
`needs: [prepare, build]`, и тогда:

1. `voice-assistant` ждёт все 8 независимых сервисов вместо одного —
   огрубление графа, против которого §3.4 сам и аргументирует;
2. падение **любого** сервиса матрицы отменяет `voice-assistant` и
   `supervisor` — сегодня падение `build-quest` им не мешает, то есть при
   частичном отказе менялся бы набор собранных образов.

Итог: именованными остались **три** job'а (`voice-base`,
`voice-assistant`, `supervisor`), граф `needs` побайтово прежний. В
генераторе это выражено явно: в matrix идут сервисы без `depends_on` и
не являющиеся целью ребра.

### §12.2. Deletion test (§7) — оценка не подтвердилась

| | оценка §7 | факт |
|---|---|---|
| `L-Build Vision Pi Services.yml` | — | 712 → 616 (**−96**) |
| `L-Build Main Pi Services.yml` | — | 508 → 391 (**−117**) |
| итого YAML | −500…−550 | **−213** |

Причины расхождения: (1) вариант B требует четырёх копий шаблона job'а на
Vision (matrix + три именованных), а оценка считала «9 тел уходят, остаётся
одна matrix-job»; (2) `prepare` вырос на ~45 строк (PyYAML + вызов
генератора); (3) знание из удалённых job'ов (race `supervisor` и
`voice-assistant`, ARCH-quest #2278, ADR-0089, «tag naming is inconsistent»)
перенесено в шапку секции, а не выброшено.

Главный результат не в строках: состав сборки перестал существовать в
workflow как список. Негативный контроль guard'а это фиксирует — порча
`depends_on` или `defaults.ros_distro` в манифесте валит тесты.

### §12.3. Проверено на живом стенде

`prepare` на katana, run 35617451547:

```
Docker Tag: test
PyYAML 6.0.3
```

Риск №1 («PyYAML может не оказаться на раннере») снят фактом: библиотека
там есть. Матрица развернулась в 8 job'ов с ожидаемыми именами
(`build-rtabmap`, `build-nav2`, `build-robot-state-publisher`,
`build-twist-mux`, `build-ros2-control`, `build-lslidar`, `build-teleop`,
`build-perception`) — динамическая matrix, прецедента которой в репозитории
не было (§3.2), работает.

### §12.4. Что осталось

- **Уточнение к Phase 4, найденное прогоном 21.09.2026:** `L-Build Single
  Service.yml` не просто дублирует список сервисов — он вообще НЕ использует
  композит `.github/actions/l-build-service`, а содержит собственный вызов
  `docker buildx build`. Следствие: всё, что добавлено в композит
  (registry-кеш Этапа 0, docker-container билдер, подмена host-gateway), на
  одиночные сборки НЕ распространяется. Обнаружено, когда проверочная сборка
  led-matrix через этот workflow не показала в логе ни одной строки `Cache:`.
- Phase 4 — `L-Build Single Service.yml` (четыре копии списка плюс мёртвый
  `apriltag`).
- Удаление phantom-полей отдельным PR по процедуре ADR-0094 §3.3.
- Расширение `actionlint`-гейта за пределы двух build-workflow (репозиторий
  целиком его сегодня не проходит: ~190 замечаний, 75 из них в
  `L-Deploy and Verify.yml`).
- Стухшие упоминания `build-oak-d`/`build-perception` в `docs/CI_CD_PIPELINE.md`.
- Если в branch protection настроены required checks по старым именам
  job'ов — их надо обновить: `build-oak-d` стал инстансом matrix-job `build`
  с display-name `build-oak-d`. Проверить может только владелец настроек.
