# План: Ресурсный пак — единый шов доставки бинарных ресурсов на Vision Pi

| Поле | Значение |
|---|---|
| Дата | 2026-09-15 (составлено 2026-09-16) |
| Статус | Черновик плана, к реализации не приступали |
| Автор | architect (агентское ревью, без реализации) |
| Ветка | `feature/cicd-pipeline-redesign` (документ, код не менялся) |
| Связанные ADR | ADR-0111 (расходится, см. §11), ADR-0104, ADR-0089, ADR-0099, ADR-0018, ADR-0112 |
| Связанные issue | #2610 (katana SPOF), #2398/#2599 (HEF lifecycle), #2499 (Hailo binding) |

---

## 0. Как читать этот документ

Каждый факт помечен `file:line` на момент написания (2026-09-16, ветка
`feature/cicd-pipeline-redesign`). Там, где бриф прислал предположение,
которое не подтвердилось при чтении кода — это явно написано, а не тихо
исправлено. Раздел 11 — самый важный: он честно фиксирует, что часть идеи
(Renardo-сэмплы) уже решена ADR-0111 **иначе**, чем предполагал бриф.

---

## 1. Контекст и факты

### 1.1 Открытые ресурсы (публичный URL, скрипт может скачать сам)

| Ресурс | Источник сейчас (file:line) | URL | Размер | Куда попадает в контейнере | Потребитель | Политика при провале скачивания сейчас |
|---|---|---|---|---|---|---|
| Vosk STT модель (`vosk-model-small-ru-0.22`) | BuildKit cache-mount, `docker/vision/voice_base/Dockerfile:129–165` | `https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip` | 45 МБ (по комментарию в Dockerfile) | `/models/vosk-model-small-ru-0.22` внутри образа `voice-base` → наследуется `voice-assistant` | `stt_node` — `model_path: /models/vosk-model-small-ru-0.22`, подтверждено в `docker/vision/config/voice_assistant/stt_node.yaml:8` и `src/rob_box_voice/config/stt_node.yaml:14` | **Build падает.** `wget -q -O ... && unzip ... && test -f .../README` — без `\|\| true`; сеть недоступна на build-хосте → вся сборка `voice-base` красная. |
| Silero TTS модель (`v5_ru.pt`) | BuildKit cache-mount, `docker/vision/voice_base/Dockerfile:129–165` | `https://models.silero.ai/models/tts/ru/v5_ru.pt` | 68 МБ | `/models/silero/v5_ru.pt` внутри образа `voice-base` | `tts_node` — `src/rob_box_voice/rob_box_voice/tts_node.py:1850–1854`, кандидатный путь №1 `/models/silero/v5_ru.pt` (основной, «встроено в Docker образ»), №2 `/cache/tts/silero_v5_ru.pt` (legacy volume, fallback) | **Build падает** (та же цепочка `&&`, без `\|\| true`). |
| Silero torch.hub прогрев (`snakers4/silero-models`, **не** веса, а репозиторий с кодом пакета) | `docker/vision/voice_base/Dockerfile:163–164` | `torch.hub.load(repo_or_dir='snakers4/silero-models', ...)` — **не закреплённый git-ref, тянет HEAD** | не указан | кэш `torch.hub` внутри образа | `tts_node.py:1874–1879` — **fallback третьего уровня**, только если оба файловых пути (см. выше) не найдены | `\|\| true` — тихо проглатывается. Отдельная неучтённая сетевая зависимость сборки; не воспроизводима (нет pin на коммит/тег). |
| YOLOv8n HEF | Хостовый скрипт, `docker/vision/scripts/vision-hailo/download_yolov8n_hef.sh` | `https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8/yolov8n.hef` | не указан в скрипте | `/opt/rob_box/models/yolov8n.hef` на хосте, монтируется `:ro` в `vision-hailo` — `docker/vision/docker-compose.yaml:730` (в брифе фигурировала строка 680 — **не подтвердилось**, актуальная строка 730, см. §12) | `vision-hailo` — `HEF_PATH=${HEF_PATH:-}` env + `docker/vision/config/hailo_models.yaml:26–28` (`hef_path`, Phase 1.5 путь `/opt/rob_box/models/yolov8n.hef` — в комментарии) | Скрипт **не падает** сам по себе (`set -eo pipefail`, но вызывается с `\|\| true` из деплоя, см. ниже), SHA256 не проверяется (`HEF_SHA256=""` по умолчанию — «Hailo не публикует официальный SHA», `download_yolov8n_hef.sh:31–33`). |
| RetinaFace HEF | Хостовый скрипт-копия, `docker/vision/scripts/vision-hailo/download_retinaface_hef.sh` | `https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8/retinaface_mobilenet_v1.hef` | 5 993 660 байт (зафиксировано в комментарии `:14`) | `/opt/rob_box/models/retinaface_mobilenet_v1.hef`, монтируется `:ro` в `vision-face` — `docker/vision/docker-compose.yaml:795` (в брифе — строка 744, **не подтвердилось**, см. §12) | `vision-face` — `FACE_HEF_PATH` env | SHA256 **проверяется** и жёстко зашит (`HEF_SHA256="1FBC7BE2...` — регистронезависимое сравнение после бага issue #2599, `download_retinaface_hef.sh:31–33,72–81`). |
| Звуковой пак (`sound_pack/`) | git checkout, корень репозитория | — (файлы в git, не скачиваются) | 884 КБ, 55 файлов `.mp3` (проверено `du -sh sound_pack`) | `../../sound_pack:/ws/sound_pack:ro` — `docker/vision/docker-compose.yaml:306` | `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/sound.py:43–49` — перебирает **5** кандидатных путей (`/ws/sound_pack/...`, Pi-путь, три относительных) | Не применимо — доставляется через `git reset --hard` вместе с кодом (см. `.github/workflows/L-Deploy and Verify.yml:298–317`), не через отдельный download-шаг. |

### 1.2 Ресурс, сознательно исключённый из таблицы «открытых»: Renardo/FoxDot сэмплы

Формально это тоже «биномный ресурс» (паттерн №2 из брифа: `docker/vision/voice_resources/Dockerfile` + `init_resources.sh` + `download_samples.py`, паки `0_foxdot_default` + `1_pitchglitch_samples`, ~600 МБ с `collections.renardo.org`). Он **не включён** в манифест этого документа как объект host-доставки — причина в §11: ADR-0111 (принят 2026-09-15, за день до этого документа) решил его **иначе** — bake-in в `voice-base`, без host-скачивания. Подробности, включая честный разбор расхождения, — §11.

### 1.3 Закрытые (gated) вендорные артефакты — второй класс ресурсов

Это НЕ то же самое, что «открытые» ресурсы выше: URL недоступен без аккаунта
Hailo Developer Zone, скрипт не может скачать их сам. Задача ревью явно
попросила разобрать этот класс отдельно — он не выдуман, в репозитории уже
есть **три** независимых места, которые упоминают один и тот же паттерн:

| Файл | Что делает | Откуда берёт артефакт |
|---|---|---|
| `docker/vision/vision-hailo/Dockerfile:113–131` | Устанавливает Python-биндинг `hailo_platform` из wheel + кладёт `libhailort.so.4.24.0` в `/usr/lib`, под флагом `HAILO_INSTALL_BINDING=whl`. Комментарий `:114`: «Wheel + libhailort.so.4.24.0 — артефакты Hailo Developer Zone (аккаунт), **НЕ в git**». | `docker/vision/vision-hailo/wheels/` — build-context, наполняется CI-шагом (следующая строка) |
| `.github/workflows/L-Build Vision Pi Services.yml:507–522` (шаг «Fetch HailoRT wheel + lib (ADR-0099 §2.2, Developer Zone artifact)») | Копирует `hailort-4.24.0-cp310-cp310-linux_aarch64.whl` и `libhailort.so.4.24.0` в build-context **из `/opt/rob_box/vendor/` на build-хосте** (katana), fallback — `/tmp/`. Если файла нет ни там, ни там — `echo "::error::..."` + **`exit 1`**, весь job падает. | `/opt/rob_box/vendor/*` на katana (self-hosted runner `[self-hosted, rob-box]`) |
| `.github/workflows/L-Build Single Service.yml:481–498` | Тот же паттерн, дословно (копия), для одиночной пересборки `vision_hailo`. | То же |
| `scripts/setup/setup_node.sh:694–745` (`setup_hailo_ai_hat()`) | На **самой Vision Pi** (не на build-хосте!) ставит драйвер `hailort-pcie-driver` (DKMS) и рантайм `hailort` из `.deb`, которые «ожидаем предзаложенными в `/opt/rob_box/vendor/`» (`:697,730`). Если `.deb` нет — `log_warning` + инструкция скачать вручную + `return 0` (**не падает**, идемпотентно перезапускаемо). | `/opt/rob_box/vendor/hailort-pcie-driver_4.24.0_all.deb`, `/opt/rob_box/vendor/hailort_4.24.0_arm64.deb` на **Vision Pi** |

Итого — **четыре** закрытых файла одной версии (4.24.0), два разных хоста
(katana — build-время, Vision Pi — install-время хостового драйвера), три
независимых куска кода, которые знают про соглашение «искать в
`/opt/rob_box/vendor/`», но нигде не описанное как формальный контракт:
нет манифеста, нет фиксированных sha256 (кроме отсутствующих — TODO ниже),
нет единой процедуры «как файл туда попал в первый раз».

| Файл | Версия | Где ожидается | Проверка целостности сейчас |
|---|---|---|---|
| `hailort-pcie-driver_4.24.0_all.deb` | 4.24.0 | `/opt/rob_box/vendor/` на **Vision Pi** | нет |
| `hailort_4.24.0_arm64.deb` | 4.24.0 | `/opt/rob_box/vendor/` на **Vision Pi** | нет |
| `hailort-4.24.0-cp310-cp310-linux_aarch64.whl` | 4.24.0 | `/opt/rob_box/vendor/` на **katana** (build-хост) | нет |
| `libhailort.so.4.24.0` | 4.24.0 | `/opt/rob_box/vendor/` на **katana** (build-хост) | нет |

**Честно про сегодняшнее состояние**: канонического места для «того самого»
файла нет. Ответ на вопрос «откуда он взялся в `/opt/rob_box/vendor/` на
katana» сегодня — «кто-то один раз скачал руками из Hailo Developer Zone и
положил на конкретную машину». Ни резервной копии, ни версии в манифесте,
ни sha256 для проверки «это точно тот файл» не существует. Это
недокументированное состояние одного хоста, и именно так это и надо
называть, а не «уже решено».

---

## 2. Интерфейс модуля «Ресурсный пак»

### 2.1 Название и место в дереве

**Ресурсный пак** (рабочее имя, термин для `CONTEXT.md` — см. §14) —
модуль, который знает, какие бинарные ресурсы нужны Vision Pi, откуда их
взять и куда положить, и умеет привести файловую систему хоста в
соответствие манифесту. Живёт как один скрипт +один манифест, предлагаемое
место: `docker/vision/scripts/resource_pack/` (`apply_resource_pack.sh` +
`manifest.yaml`), т.к. это host-side скрипт того же типа, что и
`download_yolov8n_hef.sh` сегодня — не питоновский пакет, не образ.

### 2.2 Что принимает

Один манифест (YAML, формат — §3) со списком записей вида «имя, тип
(open/gated), URL или ожидаемое место, sha256, куда положить, обязательность».
Аргументы командной строки/переменные окружения — только
`RESOURCE_PACK_MANIFEST` (путь к манифесту, default в репозитории),
`RESOURCE_PACK_ROOT` (default `/opt/rob_box`), `RESOURCE_PACK_FORCE`
(пере-верифицировать даже при наличии маркера).

### 2.3 Что гарантирует (контракт)

1. **Идемпотентность.** Повторный вызов при уже верном файле — no-op за
   доли секунды (сверка sha256 или маркера, без повторного скачивания).
   Это уже есть в двух из четырёх текущих реализаций (`download_*_hef.sh`
   `-f "${HEF_FILE}"` check, `init_resources.sh` `.initialized` marker) —
   модуль обобщает существующий паттерн, не изобретает новый.
2. **Проверка целостности.** Каждый **open**-ресурс проверяется по sha256
   после скачивания. Сегодня это делает только `download_retinaface_hef.sh`
   (регистронезависимое сравнение, issue #2599); Vosk/Silero/yolov8n —
   вообще без проверки. Модуль поднимает всех до одного стандарта.
3. **Версионирование.** Манифест — единственный источник версии/URL/sha256.
   Смена версии модели = один PR, меняющий одну строку в одном файле, а
   не Dockerfile + скрипт + комментарий россыпью.
4. **Поведение без сети.** Для **hard**-ресурсов — явный `exit 1` с
   понятным сообщением, что сломается и почему (сегодня Vosk/Silero
   роняют *всю сборку* без объяснения, какая ступень виновата — тонет в
   стандартном выводе `wget`). Для **soft**-ресурсов — warning +
   продолжение, деградация называется явно (см. §3), в духе ADR-0018
   (честный FAIL лучше красивого PASS, и наоборот — честная деградация
   лучше молчаливой).
5. **Открытые vs закрытые ресурсы — общий манифест, разное поведение
   добычи.** Открытый ресурс модуль умеет скачать сам. Закрытый —
   модуль умеет только *проверить* (наличие + sha256) и **честно
   сказать, чего не хватает и что с этим делать** — заполнение делает
   человек один раз (см. §5).

### 2.4 Режимы

- **Деплой** (CI, `L-Deploy and Verify.yml`) — заменяет сегодняшний шаг
  «[Vision Pi] Ensure HEF models» (`:320–353`), расширенный на весь
  манифест, а не только два HEF.
- **Первый boot / provisioning** свежей Pi — вызывается из
  `scripts/setup/setup_node.sh` (там уже живёт `setup_hailo_ai_hat()`,
  логика родственная, — не задача этого плана переписывать сам
  `setup_node.sh`, только показать, что вызов туда впишется естественно).
- **Ручной вызов** оператором/девопсом — `bash apply_resource_pack.sh
  --only vosk` для точечного обновления одного ресурса без полного
  прогона.

### 2.5 Что остаётся за швом (и почему интерфейс маленький, а реализация большая)

За швом: HTTP-клиент и ретраи (сейчас разные — `wget -q`, `curl -fL --retry
3`, питоновский `urllib` с `ThreadPoolExecutor`), формат распаковки (`zip`
vs «как есть» vs `.deb` через `apt`/`dpkg` вручную), разный слой хранения
(build-time cache-mount vs host-каталог vs named-volume), разная
семантика «уже стоит» (файл существует / маркер / `dpkg -l | grep ii` /
`modinfo` версия драйвера). Именно поэтому реализация — не двадцать строк:
внутри придётся аккуратно развести четыре разных способа «проверить, что
уже стоит» под одну функцию `ensure(resource)`. Интерфейс остаётся
маленьким («дай манифест — получи разложенную ФС») ровно потому, что вся
эта неоднородность спрятана внутри, а не потому что её нет.

---

## 3. Формат манифеста ресурсов

```yaml
# docker/vision/scripts/resource_pack/manifest.yaml
version: 1

resources:
  # ---------- открытые: STT/TTS модели (сегодня — BuildKit cache-mount) ----------
  - name: vosk-ru-small
    type: open
    kind: model-zip
    url: https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip
    sha256: ""  # TODO: посчитать один раз после скачивания текущего файла
    size_bytes: 47185920  # ~45 МБ, из комментария Dockerfile:148
    unpack: zip
    target: /opt/rob_box/models/vosk-model-small-ru-0.22
    required: hard  # без модели stt_node не стартует
    on_missing: fail-deploy
    consumers:
      - "stt_node (model_path, docker/vision/config/voice_assistant/stt_node.yaml:8)"

  - name: silero-tts-v5-ru
    type: open
    kind: model-file
    url: https://models.silero.ai/models/tts/ru/v5_ru.pt
    sha256: ""  # TODO
    size_bytes: 71303168  # ~68 МБ
    unpack: none
    target: /opt/rob_box/models/silero/v5_ru.pt
    required: hard
    on_missing: fail-deploy
    consumers:
      - "tts_node (tts_node.py:1850-1854, кандидатный путь №1)"

  # ---------- открытые: Hailo HEF (сегодня — download_*_hef.sh) ----------
  - name: yolov8n-hef
    type: open
    kind: hef
    url: https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8/yolov8n.hef
    sha256: ""  # Hailo официально не публикует — как и сегодня (download_yolov8n_hef.sh:31-33)
    unpack: none
    target: /opt/rob_box/models/yolov8n.hef
    required: soft
    on_missing: degrade
    degrade_note: >
      vision-hailo остаётся в stub-режиме (capability-honest, ADR-0018);
      /vision/hailo/events публикуется, но пуст. Не блокирует деплой.
    consumers:
      - "vision-hailo (HEF_PATH env, docker/vision/config/hailo_models.yaml:26-28)"

  - name: retinaface-hef
    type: open
    kind: hef
    url: https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8/retinaface_mobilenet_v1.hef
    sha256: "1fbc7be2554cceba18986cefa73983a59057ac5457bc75173fb2e09457bff472"  # уже зафиксирован сегодня
    size_bytes: 5993660
    unpack: none
    target: /opt/rob_box/models/retinaface_mobilenet_v1.hef
    required: soft
    on_missing: degrade
    degrade_note: "vision-face остаётся в stub, лицевая детекция недоступна."
    consumers:
      - "vision-face (FACE_HEF_PATH env)"

  # ---------- закрытые (gated): Hailo Developer Zone, версия 4.24.0 ----------
  - name: hailort-pcie-driver-deb
    type: gated
    kind: deb
    vendor_file: hailort-pcie-driver_4.24.0_all.deb
    version: "4.24.0"
    sha256: ""  # TODO: посчитать после ручной выгрузки, см. §5
    target_host: vision-pi          # НЕ katana — драйвер ставится на саму Pi
    target: /opt/rob_box/vendor/hailort-pcie-driver_4.24.0_all.deb
    required: soft
    on_missing: degrade
    degrade_note: "Hailo AI HAT не настраивается, vision-hailo/vision-face — stub."
    consumers:
      - "scripts/setup/setup_node.sh:setup_hailo_ai_hat() (host driver, DKMS)"

  - name: hailort-arm64-deb
    type: gated
    kind: deb
    vendor_file: hailort_4.24.0_arm64.deb
    version: "4.24.0"
    sha256: ""  # TODO
    target_host: vision-pi
    target: /opt/rob_box/vendor/hailort_4.24.0_arm64.deb
    required: soft
    on_missing: degrade
    consumers:
      - "scripts/setup/setup_node.sh:setup_hailo_ai_hat()"

  - name: hailort-whl
    type: gated
    kind: wheel
    vendor_file: hailort-4.24.0-cp310-cp310-linux_aarch64.whl
    version: "4.24.0"
    sha256: ""  # TODO
    target_host: katana              # build-контекст, НЕ Vision Pi
    target: docker/vision/vision-hailo/wheels/hailort-4.24.0-cp310-cp310-linux_aarch64.whl
    required: soft
    on_missing: degrade-build         # см. §11(доп.) — сегодня это НЕ degrade, а fail
    consumers:
      - "docker/vision/vision-hailo/Dockerfile:120-131 (HAILO_INSTALL_BINDING=whl)"
      - "L-Build Vision Pi Services.yml:507-522 (Fetch HailoRT wheel + lib)"

  - name: libhailort-so
    type: gated
    kind: shared-lib
    vendor_file: libhailort.so.4.24.0
    version: "4.24.0"
    sha256: ""  # TODO
    target_host: katana
    target: docker/vision/vision-hailo/wheels/libhailort.so.4.24.0
    required: soft
    on_missing: degrade-build
    consumers:
      - "docker/vision/vision-hailo/Dockerfile:124"

  # ---------- вне доставки Ресурсного пака, только для реестра ----------
  - name: renardo-samples
    type: build-time  # не open, не gated — печётся в образ на этапе CI-сборки
    note: >
      НЕ доставляется этим модулем. ADR-0111 решил печь эти сэмплы в
      voice-base на этапе сборки, без host-скачивания. Запись оставлена
      здесь только для полноты реестра ресурсов робота — см. §11.
    consumers:
      - supercollider (WAV buffers)
      - "voice-assistant Renardo (search_renardo_samples, synth-only fallback)"

  - name: sound-pack
    type: git  # уже версионируется в git, доставляется git reset --hard
    note: >
      Сознательно НЕ включён в host-доставку — 884 КБ, уже в репозитории,
      уже надёжно доставляется вместе с кодом. См. §6.4.
    consumers:
      - "sound.py:PlaySoundTool (5 кандидатных путей, sound.py:43-49)"
```

Поля `required` (`hard`/`soft`) и `on_missing` — рычаг раздела 2.3 п.4:
именно они определяют, роняет ли отсутствие ресурса деплой или переводит
потребителя в честно объявленную деградацию.

---

## 4. Закрытые артефакты — откуда берутся, где живут, как попадают дальше

### 4.1 Сегодняшнее состояние (честно)

Первый экземпляр каждого из четырёх файлов версии 4.24.0 появляется
**вручную**: человек с аккаунтом Hailo Developer Zone скачивает `.deb`/`.whl`
через браузер (регистрация обязательна, у скрипта нет доступа) и кладёт
файл в `/opt/rob_box/vendor/` на конкретной машине — либо на katana
(нужен для build-контекста), либо на самой Vision Pi (нужен для host-драйвера).
Никакой записи о том, кто это сделал, когда, и какой именно это билд —
не существует. Это ровно то незадокументированное состояние одного хоста,
о котором просил честно написать бриф: если katana потеряется (диск,
переезд, переустановка) — сборка `vision-hailo` с `HAILO_INSTALL_BINDING=whl`
теряет возможность собраться с реальным биндингом **безвозвратно**, пока
кто-то заново не зайдёт в Developer Zone.

### 4.2 Как файл сейчас попадает в три места

| Куда | Как | Файл-источник |
|---|---|---|
| (а) в сборку на katana | `L-Build Vision Pi Services.yml:507-522` копирует из `/opt/rob_box/vendor/` (или `/tmp/`) в build-context `docker/vision/vision-hailo/wheels/` перед `docker build` | wheel + `.so` |
| (б) на Vision Pi | `scripts/setup/setup_node.sh:setup_hailo_ai_hat()` — читает `.deb` из `/opt/rob_box/vendor/` **на самой Pi**, ставит через `dpkg`/DKMS | оба `.deb` |
| (в) на чистую новую машину-сборщик | **Ничего не делает этого автоматически.** Человек должен вручную повторить (а) для новой katana. Это и есть незадокументированная часть. |

### 4.3 Варианты зеркала для закрытых артефактов

| Вариант | Не нарушает ли лицензию Hailo | Переживает потерю katana | Работает на чистой машине | Новых сущностей |
|---|---|---|---|---|
| **(а) OCI-артефакт в приватном GHCR** (`oras push ghcr.io/krikz/rob_box-vendor:hailort-4.24.0 ./hailort...whl`) | **Вопрос к человеку** — Hailo Developer Zone EULA обычно запрещает редистрибуцию третьим лицам; приватный registry снижает риск (не публичный), но не снимает вопрос юридически | Да — GHCR внешний относительно katana | Да — `docker`/`oras` уже есть в тулчейне (используется для всех остальных образов), но `oras` как таковой в репозитории сегодня **не используется** (проверено — нет ни одного вызова `oras` в `.github/workflows/*.yml`), т.е. это новый CLI-инструмент в CI | 1 (новый registry-репозиторий + новый CLI в раннере) |
| **(б) `/opt/rob_box/vendor` как официальная точка засева + манифест sha256 + документированная процедура** | Не решает вопрос лицензии сам по себе, но и не создаёт нового способа распространения — файл остаётся там, где и был | **Нет** — если это единственная копия и katana теряется, теряется и файл; вариант снимает только незадокументированность, не SPOF | Частично — на чистой машине нужно повторить ручную выгрузку заново (тем же способом, что и сегодня), но теперь по чек-листу, а не по памяти | 0 (переиспользует существующий каталог и соглашение, только формализует) |
| **(в) git-lfs** | Каждый push копии `.whl`/`.deb` в LFS-хранилище — тоже редистрибуция; репозиторий сегодня **не использует** git-lfs нигде (проверено — `.gitattributes` в корне не содержит lfs-фильтров) | Да, если LFS-хранилище отдельное от katana | Да — `git lfs pull` работает на любой машине с доступом к репо | 1 (новая зависимость тулчейна, новый storage-биллинг) |
| **(г) отдельный tar-пак как GitHub Release asset приватного репозитория** | Тот же вопрос редистрибуции, что и (а), но менее auditable (Release assets не versioned так же строго, как OCI-теги) | Да | Да — `gh release download` есть в стандартном тулчейне GH Actions | 1 (новый приватный репозиторий или новый release-поток в существующем) |

**Рекомендация:** начать с **(б)** — не потому что она технически
элегантнее (а решает эту часть плана явно), а потому что она **не
принимает решение за человека**: она не копирует вендорные бинарники
никуда за пределы того единственного места, где они лежат сегодня, а
просто перестаёт быть молчаливой (манифест с sha256, чек-лист «как
заполнить», явная деградация при отсутствии). Вопрос лицензии Hailo
блокирует (а)/(в)/(г) — они физически размножают копии файла за пределы
исходной машины, а Hailo EULA на редистрибуцию третьим лицам этот план
проверить не может и не должен: **это решение владельца проекта**, вынесено
в открытые вопросы (§13). Если лицензия окажется разрешающей —
пересмотреть в пользу (а): OCI в приватный GHCR устраняет реальный SPOF
(потеря katana), чего (б) не делает.

### 4.4 Что меняется для manifest.yaml из §3

Записи `type: gated` получают обязательные поля `sha256` и `version`
(в примере — плейсхолдеры `""`, реальные значения — задача человека,
единожды, при следующем контакте с Developer Zone) — это прямое требование
задачи: «для закрытых — обязательные sha256 и версия, чтобы файл был
проверяем». Модуль (§2) для `gated`-записи не скачивает — только
`test -f` + сверка sha256 + осмысленное сообщение, что делать, если файла
нет или sha256 не совпал (типичная причина — файл от другой версии HailoRT).

---

## 5. Раскладка на диске

Ключевое ограничение брифа подтвердилось: `/opt/rob_box/models` уже занят
(два `.hef`, монтируется `:ro` в `vision-hailo` — `docker-compose.yaml:730`
— и в `vision-face` — `:795`). Переименовывать каталог — ломать оба
монтирования и комментарии в `docker/vision/config/hailo_models.yaml:12-13`.

**Решение: не создавать новый каталог, а доращивать два уже существующих.**
Оба уже употребляются в прод-паттерне «host-каталог, скрипт кладёт файл»
(HEF в `/opt/rob_box/models`, вендорные `.deb`/`.whl` — в `/opt/rob_box/vendor`,
уже конвенция трёх мест из §4.2). Это меньший интерфейс: существующие
volume-mount'ы `vision-hailo`/`vision-face` не меняются вообще.

```
/opt/rob_box/
├── models/                                  # УЖЕ существует, монтируется :ro
│   ├── yolov8n.hef                          # без изменений
│   ├── retinaface_mobilenet_v1.hef          # без изменений
│   ├── vosk-model-small-ru-0.22/            # НОВОЕ — переезжает из образа voice-base
│   │   └── ... (содержимое zip, как сегодня внутри образа)
│   └── silero/
│       └── v5_ru.pt                         # НОВОЕ — переезжает из образа voice-base
└── vendor/                                   # УЖЕ существует как неформальная конвенция
    ├── hailort-pcie-driver_4.24.0_all.deb   # НОВОЕ формально — уже кладётся туда руками
    ├── hailort_4.24.0_arm64.deb
    ├── hailort-4.24.0-cp310-cp310-linux_aarch64.whl   # на katana, не на Vision Pi
    ├── libhailort.so.4.24.0                            # на katana, не на Vision Pi
    └── manifest.lock.json                    # НОВОЕ — sha256 + version + timestamp засева
```

Примечание: `/opt/rob_box/vendor` на **katana** (build-хост) и
`/opt/rob_box/vendor` на **Vision Pi** — два разных каталога на двух
разных хостах с одинаковым именем и частично разным содержимым (katana
держит wheel+so для сборки, Pi держит два `.deb` для host-driver). Манифест
(§3) различает их полем `target_host`. Это не противоречие, а то же самое
свойство, что и сегодня («канонично — `/opt/rob_box/vendor`», без уточнения
«на каком хосте» — планом уточняется явно).

`sound_pack/` — остаётся в git-корне репозитория, без изменений (§6.4).

---

## 6. Как ресурсы попадают в контейнеры

### 6.1 Vosk/Silero — из `voice-base` build-time в host bind-mount

**Убрать** из `docker/vision/voice_base/Dockerfile:126-165` весь блок
скачивания (RUN-ступень «Скачивание STT/TTS моделей»). **Добавить** в
`docker/vision/docker-compose.yaml` для сервиса `voice-assistant` новый
volume:

```yaml
volumes:
  - /opt/rob_box/models:/models:ro   # НОВОЕ — вместо запечённых в образ
```

Ключевое наблюдение: путь внутри контейнера (`/models`) **не меняется** —
`stt_node.yaml:8` (`model_path: /models/vosk-model-small-ru-0.22`) и
`tts_node.py:1853` (`/models/silero/v5_ru.pt`) продолжают читать то же
самое место, просто теперь это bind-mount, а не слой образа. **Ни один
Python-файл менять не нужно.** `ENV SOUND_PACK_DIR=/ws/sound_pack` и
`ENV TTS_CACHE_DIR=...` (`Dockerfile:193-194`) не затрагиваются.

### 6.2 HEF — без изменений

`vision-hailo`/`vision-face` уже монтируют `/opt/rob_box/models:/opt/rob_box/models:ro`
целиком (не по файлам) — `docker-compose.yaml:730,795`. Добавление
Vosk/Silero в этот же каталог **не требует правки** этих двух сервисов —
они просто не видят новые файлы (или видят, но не используют — не мешает).

### 6.3 Renardo `renardo_samples` named-volume — не трогать в рамках этого плана

Решение ADR-0111 (§11) не требует изменений от Ресурсного пака.
`renardo_samples` volume, `voice-resources-init` (сейчас `profiles: ["init"]`,
`required: false` — недавняя мера issue #2610, коммит `9bfc56c9`) —
остаются как есть до отдельной реализации ADR-0111 (карточка `t_ca7fa165`).

### 6.4 `sound_pack` — не менять

`../../sound_pack:/ws/sound_pack:ro` (`docker-compose.yaml:306`) остаётся.
Файлы уже в git, уже 884 КБ, уже надёжно доставляются `git reset --hard`
(`L-Deploy and Verify.yml:298-317`) вместе с кодом — до Ресурсного пака.
Заводить сюда ещё один механизм доставки того, что и так работает,
означало бы тратить рычаг модуля не туда: два реально болящих паттерна
(build-time скачивание, дублирующиеся shell-скрипты) важнее, чем
косметика над тем, что не ломается. Пятистрочный список кандидатных путей
в `sound.py:43-49` — отдельный, не связанный с доставкой техдолг (можно
оставить на отдельную карточку, не в этом плане).

### 6.5 Что удаляется из compose (детали — §7)

`docker/vision/voice_base/Dockerfile` теряет ступень скачивания моделей
(`:126-165`) — 40 строк. `docker-compose.yaml` не теряет ничего, только
получает один новый volume-mount (§6.1).

---

## 7. Пошаговый план реализации

Каждый этап — отдельный PR, каждый безопасен для мержа поодиночке
(инкрементальность, ADR-AF-0013), ни один этап не требует, чтобы
следующий уже случился.

### Этап 1 — манифест + скрипт, без переключения потребителей

1. Создать `docker/vision/scripts/resource_pack/manifest.yaml` (формат §3),
   первая версия — **только записи для двух уже существующих открытых HEF**
   (yolov8n, retinaface), т.к. на них проще всего проверить модуль без риска
   сломать что-то новое.
2. Создать `docker/vision/scripts/resource_pack/apply_resource_pack.sh` —
   читает манифест, для каждой `open`-записи: если файл есть и sha256
   (когда задан) совпадает — no-op; иначе скачивает, проверяет, кладёт.
   Для `gated` — только проверяет и печатает статус.
3. Юнит/smoke-тест скрипта на фикстуре-манифесте (без реальной сети) —
   аналог существующих tests для `download_*_hef.sh`, если такие есть
   (проверить `docker/vision/scripts/vision-hailo/` на предмет теста —
   не найдено ни одного `test_download*` в этой директории при
   первичном просмотре, значит и этого сегодня нет — тоже стоит завести).
4. **Не менять** `download_yolov8n_hef.sh`/`download_retinaface_hef.sh` и
   шаг в `L-Deploy and Verify.yml` — они продолжают работать как раньше.
   Этап 1 доказывает, что новый механизм работает, не трогая прод.

### Этап 2 — переключить деплой HEF на новый скрипт

1. `L-Deploy and Verify.yml:320-353` («[Vision Pi] Ensure HEF models») —
   заменить цикл по двум hardcoded скриптам на один вызов
   `apply_resource_pack.sh --only yolov8n-hef,retinaface-hef`.
2. Оставить `download_yolov8n_hef.sh`/`download_retinaface_hef.sh` в
   репозитории ещё один релиз-цикл (быстрый откат — см. §10), пометить
   комментарием «заменено Ресурсным паком, см. docs/plans/2026-09-15-resource-pack.md,
   удалить после N успешных прогонов».
3. Acceptance: холодный деплой (issue #2610 сценарий) отрабатывает,
   `/opt/rob_box/models/*.hef` на месте, деплой зелёный.

### Этап 3 — Vosk/Silero: манифест + удаление из Dockerfile

1. Добавить записи `vosk-ru-small`, `silero-tts-v5-ru` в манифест.
2. Добавить шаг `apply_resource_pack.sh --only vosk-ru-small,silero-tts-v5-ru`
   в `L-Deploy and Verify.yml` рядом с HEF-шагом (тот же паттерн, тот же
   шаг — просто больше записей в `--only`, или вовсе убрать `--only` и
   гонять весь манифест одним вызовом, если Этап 2 показал, что это
   безопасно).
3. Добавить `volumes: - /opt/rob_box/models:/models:ro` в `voice-assistant`
   (`docker-compose.yaml`, секция `voice-assistant`).
4. Убрать ступень скачивания из `docker/vision/voice_base/Dockerfile:126-165`.
   Оставить `mkdir -p /models` **не нужно** — каталог больше не нужен внутри
   образа, он придёт из bind-mount.
5. Acceptance: `voice-assistant` стартует, `stt_node` находит модель
   (см. §9), TTS озвучивает тестовую фразу без «model not found».

### Этап 4 — закрытые артефакты: формализация без смены механизма

1. Один раз посчитать sha256 существующих файлов в `/opt/rob_box/vendor/`
   на katana и на Vision Pi (человек, не CI — файлы недоступны скрипту).
2. Заполнить `sha256`/`version` в manifest.yaml для четырёх `gated`-записей.
3. Добавить `/opt/rob_box/vendor/manifest.lock.json` (или переиспользовать
   тот же `manifest.yaml` с `type: gated`) — записывает, какая версия
   сейчас реально лежит и когда была туда положена (руками, при засеве).
4. `apply_resource_pack.sh --check-gated` — новый режим, только проверка
   (используется в CI **до** шага «Fetch HailoRT wheel + lib», чтобы
   ошибка была понятной, а не голым `::error::...whl не найден`).
5. **Не менять** сам механизм копирования в build-context
   (`L-Build Vision Pi Services.yml:507-522`) и в `setup_node.sh` — они
   остаются, только получают проверку sha256 перед использованием.
6. Решение по §4.3 (OCI-зеркало) — **отдельный этап**, только после ответа
   человека на вопрос лицензии (§13). Не блокирует Этапы 1-4.

### Этап 5 — уборка (после стабилизации на проде)

1. Удалить `download_yolov8n_hef.sh`, `download_retinaface_hef.sh` (заменены
   Этапом 2, при условии N успешных прогонов без отката).
2. Обновить `docker/vision/config/hailo_models.yaml` комментарии,
   `docs/architecture/SYSTEM_OVERVIEW.md` §Vision (если там перечислены
   4 способа доставки — не проверялось, требует отдельного грепа при
   реализации).

---

## 8. Что удаляется (deletion test)

| Удаляется | Куда переезжает сложность |
|---|---|
| `docker/vision/voice_base/Dockerfile:126-165` (40 строк: скачивание Vosk+Silero+torch.hub прогрев) | В `manifest.yaml` (2 записи, декларативно) + `apply_resource_pack.sh` (общая логика скачивания для всех open-ресурсов, написана один раз) |
| `docker/vision/scripts/vision-hailo/download_yolov8n_hef.sh` (87 строк) | В ту же запись `manifest.yaml` + общий скрипт. Специфика (SHA256 optional для yolov8n) — поле `sha256: ""` в манифесте, не отдельная ветка кода |
| `docker/vision/scripts/vision-hailo/download_retinaface_hef.sh` (87 строк, **дословная копия** yolov8n-скрипта с другим URL/sha) | Туда же — устраняется именно дублирование «копия скрипта на новый файл», которое бриф просил не плодить дальше |
| Шаг «[Vision Pi] Ensure HEF models» в `L-Deploy and Verify.yml:320-353` (34 строки bash-цикла по двум hardcoded именам) | В один вызов `apply_resource_pack.sh` с манифестом — добавление пятого ресурса **не требует правки workflow**, только новой записи в YAML |
| `RUN mkdir -p /models ...` и связанные `ENV`/слои в voice_base, специфичные под запечённые модели | Не переезжает — исчезает вовсе: каталог создаёт bind-mount с хоста, а не Dockerfile |

Что **не** удаляется и почему: `voice-resources` образ, `init_resources.sh`,
`voice-resources-init` сервис — не в скоупе этого плана (см. §11, ADR-0111
решает их отдельно). `download_samples.py` — туда же.

---

## 9. Влияние на сборку

- `docker build` для `voice-base` перестаёт делать 2 сетевых запроса
  (45 МБ + 68 МБ) и один best-effort git-based `torch.hub.load` —
  экономия по комментарию Dockerfile ровно на объём этих скачиваний;
  время экономии зависит от сети build-хоста, но BuildKit cache-mount
  (`--mount=type=cache,target=/root/.cache,sharing=locked`,
  `Dockerfile:129`) и так уже кэширует между сборками — **выигрыш реален
  в основном на cold cache** (новый build-хост, первая сборка после
  `docker builder prune`), не на каждой пересборке.
- Слой `voice-base`, который сегодня содержит 113 МБ моделей внутри
  образа, перестаёт их содержать — образ `voice-base` (и, соответственно,
  `voice-assistant`, наследующий его) легче на ~113 МБ. Это уменьшает
  время `docker pull` при холодном старте Vision Pi (тот самый сценарий
  ADR-0111 §1 — «Vision Pi не поднимает стек после перезагрузки»), хотя и
  по другой причине (не registry недоступен, а просто меньше байт).
- Инвалидация слоёв: сегодня слой моделей стоит **после** apt/pip-слоёв,
  но **до** клонирования `audio_common_msgs` (`Dockerfile:126-175`) — то
  есть смена версии Vosk/Silero сегодня не требует пересборки apt-слоя,
  но требует пересборки/перескачивания `audio_common_msgs` (`git clone`
  ниже по файлу зависит только от предыдущих RUN, не от контента моделей,
  так что на самом деле не инвалидируется — Docker слои кэшируются по
  предыдущей инструкции, а не по её результату). После переноса моделей
  на хост правки версии модели **вообще не трогают Dockerfile** —
  0 инвалидированных слоёв на образе `voice-base` при апдейте модели.
- HEF: сегодня скачивание — уже вне build (host-скрипт), поэтому сборка
  `vision-hailo`/`vision-face` не меняется. Влияние Ресурсного пака на
  сборку этих образов — нулевое, только на деплой-шаг.

---

## 10. Acceptance

Проверяемые команды (Vision Pi, если не указано иное):

1. **Холодный boot Vision Pi при выключенной katana** (сценарий issue #2610):
   ```
   sudo reboot
   # после подъёма:
   systemctl is-active robbox-vision.service   # ожидание: active
   docker compose -f docker/vision/docker-compose.yaml ps   # все не-init сервисы Up
   ```
2. **Отсутствие `Buffer UGen: no buffer data` в логах supercollider**:
   ```
   docker logs supercollider 2>&1 | grep -c "Buffer UGen: no buffer data"   # ожидание: 0
   ```
   (Примечание: это проверяет Renardo-сэмплы, за которые в этом плане
   отвечает **не** Ресурсный пак, а существующий `voice-resources-init` /
   будущий ADR-0111 — команда сохранена из брифа как есть, честно
   помечено, что относится к другому шву.)
3. **`stt_node` находит Vosk-модель**:
   ```
   docker logs voice-assistant 2>&1 | grep -i "vosk\|model not found\|Folder does not contain model files"
   # ожидание: НЕТ строки "Folder does not contain model files"
   ls -la /opt/rob_box/models/vosk-model-small-ru-0.22/README   # файл существует на хосте
   ```
4. **HEF на месте и с правильным sha256 (после Этапа 2)**:
   ```
   sha256sum /opt/rob_box/models/retinaface_mobilenet_v1.hef
   # сравнить с manifest.yaml: 1fbc7be2554cceba18986cefa73983a59057ac5457bc75173fb2e09457bff472
   ```
5. **CI: манифест валиден и скрипт идемпотентен**:
   ```
   bash docker/vision/scripts/resource_pack/apply_resource_pack.sh --dry-run
   bash docker/vision/scripts/resource_pack/apply_resource_pack.sh   # первый прогон
   bash docker/vision/scripts/resource_pack/apply_resource_pack.sh   # второй — должен быть no-op, < 1с на ресурс
   ```
6. **Сборка `vision-hailo` на машине с пустым `/opt/rob_box/vendor`**
   (добавлено по запросу ревью, см. §4 и §11-доп.):
   ```
   rm -rf /opt/rob_box/vendor/*   # на изолированной/тестовой копии, НЕ на katana!
   # прогнать L-Build Vision Pi Services.yml вручную (workflow_dispatch) на этой машине
   ```
   **Ожидание по факту сегодняшнего кода — сборка `vision-hailo` job
   ПАДАЕТ** (`exit 1` на шаге «Fetch HailoRT wheel + lib»,
   `L-Build Vision Pi Services.yml:518-519`) — это **не** капабилити-честная
   деградация на уровне CI-job, хотя рантайм-деградация ноды (если бы
   образ всё же собрался без биндинга) — честная (`ADR-0099 §2.1`,
   `start_vision_hailo.sh` уже это умеет). Это расхождение с тем, что
   можно было бы ожидать по духу ADR-0099 §2.2, и оно зафиксировано здесь
   как факт, а не додумано — см. §11-доп.

---

## 11. Связь с ADR

### 11.1 ADR-0111 — прямое расхождение, разобрано честно

ADR-0111 (`docs/adr/0111-voice-resources-image-sourcing.md`, Accepted
2026-09-15, за день до этого документа) решает судьбу Renardo/FoxDot
сэмплов **иначе**, чем предполагала первоначальная идея брифа
«довести паттерн №3 до модуля и завести под него остальные три ресурса»
(в брифе Renardo — один из «остальных трёх»).

**В чём именно расхождение:**

- Бриф (до чтения ADR-0111) предполагал, что сэмплы тоже должны стать
  host-доставляемым ресурсом (паттерн «скрипт + манифест + volume»,
  как HEF).
- ADR-0111 §2.1 решает прямо противоположное: **«`voice-resources` как
  отдельный image-based init-сервис в проде — ликвидируется. Сэмплы
  доставляются через `voice-base` (или `voice-assistant`)»** — то есть
  **назад к bake-in на этапе сборки**, а не вперёд к host-доставке.
- Причина расхождения — разные проблемы. Ресурсный пак (этот план) решает
  «не тащить сеть в build» и «не дублировать скрипты». ADR-0111 решает
  «Vision Pi не должен зависеть от доступности katana как registry во
  время **старта**» (issue #2610) — для init-контейнера, который тянет
  **отдельный образ** с registry при каждом `docker compose up`, а не
  для файла, который качается один раз скриптом. Это разные оси: у
  Ресурсного пака сеть нужна один раз (при деплое/апдейте ресурса), у
  `voice-resources-init` в его нынешнем виде — при каждом старте стека
  (`pull` образа).

**Что из ADR-0111 остаётся в силе безусловно:** §2.1 (ликвидация отдельного
образа `voice-resources`), §2.3 (упрощение build-графа CI), весь список
«что НЕ делаем» из §2.7 (не поднимаем registry на Pi, не переключаем на
GHCR в рамках этого решения, не делаем multi-arch offline-пак). Ни один
из этих пунктов Ресурсный пак не трогает и не обязан трогать.

**§6 вопрос 3 ADR-0111 (открыт) касается этого плана косвенно:** ADR-0111
честно оставляет открытым размер бейкнутых сэмплов (~600 МБ, вопрос —
не сделать ли `1_pitchglitch_samples` опциональным через build-arg). Если
это станет проблемой (частые cold-pull `voice-base` на слабой сети Pi) —
у Ресурсного пака **уже был бы** готовый паттерн для «опционального
host-доставляемого большого файла» (soft-ресурс с деградацией), и тогда
имело бы смысл вернуться к этому вопросу. Но **не сейчас** и **не в этом
плане** — это меняло бы уже принятое ADR-0111 решение без нового triggera
(нет свежего инцидента вроде issue #2610).

**Нужен ли новый/дополняющий ADR?** Да, но не для того, чтобы
переспорить ADR-0111 — Ресурсный пак его не отменяет и не сужает, он
просто про **другой набор ресурсов** (Vosk/Silero/HEF/вендорные
артефакты), которые ADR-0111 не рассматривал вовсе (в его тексте нет ни
слова про Vosk/Silero/HEF). Предлагаемая формулировка нового ADR:

> **ADR-0118** (следующий свободный номер на момент написания —
> **подтвердить перед созданием файла**, в репозитории уже случались
> коллизии номеров, см. преамбулу ADR-0099) — «Ресурсный пак: единый шов
> host-доставки STT/TTS-моделей, HEF и закрытых Hailo-артефактов».
> Зафиксировать: (1) манифест-формат (§3) как SSoT версий/URL/sha256;
> (2) `/opt/rob_box/models` и `/opt/rob_box/vendor` как два канонических
> host-каталога, без создания третьего; (3) явную границу с ADR-0111 —
> Renardo-сэмплы **не** входят в зону ответственности этого шва;
> (4) явную границу вопроса лицензии Hailo (§13) как решение владельца
> проекта, не архитектурное.

### 11.2 ADR-0104 — «Взгляд»

ADR-0104 (`docs/adr/0104-perception-gaze-seam.md`) — про шов выбора
источника кадра (`gaze_source`), не про доставку HEF-файлов напрямую, но
**упоминает** HEF lifecycle как один из «уровень ниже» пунктов проблемы
(`:48-49`) и фиксирует его как **done** touchpoint (`:223`:
«10 | HEF lifecycle: скрипт скачивания | ✅ done (download_yolov8n_hef.sh)»).
Замена скрипта на манифест-модуль **не противоречит** ADR-0104 — сам
факт «HEF lifecycle решён» остаётся истинным, меняется только чем именно
он решён. Формально стоит обновить touchpoint `:223`, если заводить
новый ADR-0118 (косметика, не архитектурный конфликт).

### 11.3 ADR-0089 — AI HAT+ deployment

§3 touchpoint #11 (HEF lifecycle) и #9 (Python binding) — оба затрагиваются
этим планом: #11 меняет реализацию (манифест вместо hardcoded-скрипта),
#9 остаётся нетронутым по механизму (§4), только получает sha256-проверку.
Phase 1/1.5 деление ADR-0089 не меняется.

### 11.4 ADR-0099 — vision-hailo binding install strategy

§2.2 (Phase 1.5, `HAILO_INSTALL_BINDING=whl`) описывает **рантайм**-контракт
(«если binding недоступен — нода в stub, не падает build, не падает
рантайм» — но это про сам процесс vision_hailo_node при отсутствии
`hailo_platform` **внутри уже собранного образа**, не про CI build-job
целиком). Расхождение, найденное при проверке (см. Acceptance №6, §10):
**сегодняшний CI build-job падает целиком** (`exit 1`,
`L-Build Vision Pi Services.yml:518-519`), если `/opt/rob_box/vendor`
пуст на katana — это НЕ то же самое, что «capability-honest degraded
build», это `fail-fast` на этапе подготовки build-context, **до** того,
как Dockerfile вообще получает шанс собраться в stub-режиме
(`HAILO_INSTALL_BINDING=none` дал бы честный degraded build, но workflow
сегодня всегда жёстко передаёт `HAILO_INSTALL_BINDING=whl`, не оставляя
себе пути назад к `none` при нехватке вендорных файлов). Это не
противоречит тексту ADR-0099 (он писал только про §2.1/Phase-1-default
поведение и *будущий* Phase 1.5 без уточнения именно build-job
fail-fast'а), но стоит явно зафиксировать в новом ADR-0118 или отдельным
патчем к ADR-0099: должен ли build-job при пустом vendor **деградировать**
до `HAILO_INSTALL_BINDING=none` (собрать образ без биндинга, честно
залогировать) вместо `exit 1` — это архитектурное решение, не техническое,
и в рамках этого документа не принимается, только называется.

### 11.5 ADR-0018 — культура честности

Весь план опирается на принцип «честный FAIL лучше красивого PASS» — как
раз в его пользу говорит наблюдение из §11.4: сегодняшний `exit 1` для
build-job формально честный (не маскируется), но обвязка вокруг HEF-шага
в деплое (`L-Deploy and Verify.yml:340-350`) уже прошла через похожий урок
(issue не по номеру, но комментарий `:341-347` явно ссылается на то, что
раньше `\|\| echo warning` красил шаг зелёным при реально не вставшем
`retinaface_mobilenet_v1.hef` — тот же класс ошибки, которого стоит
избежать и в Ресурсном паке: `on_missing: degrade` должно **логировать
проверяемый факт** (test -f после попытки), а не полагаться на код
возврата скрипта.

---

## 12. Расхождения брифа с фактическим кодом (сводка)

Явно вынесено отдельным разделом, как просил бриф («если факт не
подтвердился — так и напиши»):

| Утверждение брифа | Фактическая проверка | Вывод |
|---|---|---|
| `docker-compose.yaml:680` — монтирование `/opt/rob_box/models` в `vision-hailo` | Найдено на строке **730** | Не подтвердилось дословно, суть верна |
| `docker-compose.yaml:744` — монтирование в `vision-face` | Найдено на строке **795** | Не подтвердилось дословно, суть верна |
| «менять `/opt/rob_box/models` = ломать ADR-0104» | ADR-0104 упоминает путь как **факт текущего состояния** (`:48`) и как **done**-touchpoint (`:223`), но не декларирует путь как часть своего решения — конфликт не архитектурный, а с фактом «уже сделано» | Частично подтвердилось — риск реален (сломать ссылку `✅ done`), но природа риска не «нарушение решения ADR», а «устаревание одной строки таблицы» |
| Паттерн №3 (HEF) — «уже наполовину построенный модуль» | Подтвердилось полностью: идемпотентность, опциональный SHA256, понятная инструкция при отсутствии curl/wget — всё уже есть в `download_*_hef.sh` | Подтвердилось |
| ADR-0111 «решает про Renardo-сэмплы иначе» | Подтвердилось полностью, см. §11.1 | Подтвердилось |

---

## 13. Открытые вопросы

1. **Лицензия Hailo на редистрибуцию.** Разрешает ли EULA Hailo Developer
   Zone копировать `.deb`/`.whl` за пределы машины, на которую их скачал
   человек с аккаунтом (даже в приватный GHCR/LFS)? Это блокирует выбор
   между вариантом (б) (без копирования) и (а)/(в)/(г) (с копированием) в
   §4.3. **Решает владелец проекта**, не архитектор и не этот план.
2. **Номер нового ADR.** Предложен ADR-0118 (§11.1) — подтвердить перед
   созданием, в репозитории уже случались коллизии номеров близко к
   моменту мержа (см. преамбулу ADR-0099).
3. **CI build-job fail-fast vs degrade при пустом vendor** (§11.4, §10
   Acceptance №6). Нужно ли менять поведение `L-Build Vision Pi Services.yml`
   на деградацию до `HAILO_INSTALL_BINDING=none` при отсутствии вендорных
   файлов, или текущий жёсткий `exit 1` — осознанный выбор (чтобы никто
   не задеплоил случайно stub-версию `vision-hailo`, думая что собрал
   real-inference)? Решение затрагивает ADR-0099, не только этот план.
4. **sha256 для сегодняшних Vosk/yolov8n.** У обоих сегодня нет
   зафиксированного sha256 нигде в репозитории. Кто и когда считает
   эталонное значение для manifest.yaml — при реализации Этапа 1/3, или
   раньше отдельной карточкой?
5. **Судьба `torch.hub.load` прогрева** (§1.1, таблица, третья строка).
   Он не закреплён (тянет HEAD чужого GitHub-репозитория без pin), это
   единственный несоответствующий манифест-модели источник во всём
   пакете. Оставить как есть (fallback третьего уровня, редко
   срабатывает), закрепить конкретным git-ref, или убрать вовсе (раз
   основной путь — файловый, а не torch.hub)? Не решено в этом плане.
6. **`sound.py` — пять кандидатных путей.** Не входит в Ресурсный пак
   (§6.4), но замечено как соседний техдолг — заводить отдельной
   карточкой или оставить как есть? Не решено в этом плане (сознательно,
   не задача брифа).
7. **Кто выполняет Этап 4 п.1 (посчитать sha256 вендорных файлов).**
   Нужен доступ к обеим машинам (katana + Vision Pi) — операционный
   вопрос, не архитектурный.

---

## 14. Термин для `CONTEXT.md`

В `CONTEXT.md` сегодня нет раздела про сборку/доставку (проверено —
структура файла: Агенты, Голос, Инструменты, Движок, Блокировки,
Восприятие, Интерфейсы, §160-174). Предлагаемая формулировка нового
раздела (вставить после «Интерфейсы» или как отдельный раздел «Доставка»):

```markdown
## Доставка

**Ресурсный пак** — модуль, приводящий файловую систему Vision Pi в
соответствие манифесту бинарных ресурсов (STT/TTS-модели, HEF, закрытые
Hailo-артефакты): открытый ресурс модуль умеет скачать и проверить сам,
закрытый — только проверить (наличие + sha256), заполняет человек один
раз. Раскладывает файлы в `/opt/rob_box/models` и `/opt/rob_box/vendor`
на хосте; контейнеры получают их через bind-mount, не через слой образа.
_Не путать_ с `renardo_samples` named-volume — тот заполняется
build-time-бейкнутым содержимым `voice-base` (ADR-0111), не Ресурсным
паком.
```

---

## 15. Резюме решения

1. Ресурсный пак — маленький модуль (манифест + один скрипт), не новая
   инфраструктура: переиспользует два уже существующих host-каталога
   (`/opt/rob_box/models`, `/opt/rob_box/vendor`) вместо третьего нового.
2. Закрывает три из четырёх сегодняшних путей доставки (Vosk/Silero
   build-time скачивание, дублирующиеся HEF-скрипты) манифестом с
   sha256-проверкой и явной hard/soft-деградацией.
3. Четвёртый путь (Renardo-сэмплы) **сознательно не трогается** — ADR-0111
   уже решил его иначе (bake-in), за день до этого плана; расхождение
   разобрано честно в §11.1, предложена формулировка нового ADR-0118,
   который фиксирует границу между двумя решениями, а не отменяет ADR-0111.
4. Добавлен второй, ранее нигде не формализованный класс — закрытые
   вендорные Hailo-артефакты (2 `.deb` + `.whl` + `.so`, версия 4.24.0),
   сегодня существующие как незадокументированная конвенция трёх мест
   кода. План формализует их как `type: gated` в том же манифесте, но
   явно не решает вопрос лицензии на копирование — это вынесено к
   владельцу проекта.
5. Пятый ресурс (`sound_pack`) сознательно исключён из доставки — git
   уже справляется, добавлять сюда механизм было бы тратой рычага не по
   адресу.
