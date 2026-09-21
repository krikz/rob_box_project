# Сборочный шов: жирный builder → тонкий runtime — план реализации

**Дата:** 2026-09-15
**Статус:** Draft (план, без изменений кода)
**Тип:** архитектурный план (docker build, CI/CD)
**Кандидат:** архитектурное ревью 2026-09-15, тема «сборочный шов builder→runtime»
**Связанный параллельный план:** `docs/plans/2026-09-15-resource-pack.md` (доставка закрытых Hailo-артефактов — на момент написания этого документа файл ещё не существует в репозитории; при реализации сверить актуальность ссылки)

---

## 0. Резюме

31 из 32 Dockerfile в `docker/` — одностадийные. Из них **только 12** реально запускают `colcon build`
(остальные 20 — чистый `apt-get install` поверх готовой базы или FROM специализированного образа без
компиляции своего кода; для них сборочный шов **неприменим**, потому что нечего выносить). Из этих 12
кандидатов реальный, измеримый выигрыш от разделения builder/runtime дают **6**: `vesc_nexus`, `ros2_control`,
`led_matrix`, `perception`, `vision-hailo`, `lslidar`. Для `voice_assistant` и `supervisor` выигрыш **на уровне
погрешности** — они наследуют базу через `FROM ghcr.io/krikz/rob_box:...-latest` от `voice_base`
(11.3 GB измерено локально, `docker history`), и их **собственный** слой `colcon build` весит единицы МБ
(измерено: 11.3 MB у voice_assistant). Трогать `voice_base` дорого не по объёму тулчейна, а по цене
инвалидации ~10 GB кеша для всех потребителей — здесь совпадают дух ADR-0100 и коммит `337548a38`.

**Главный технический риск плана** (раздел 2): у большей части кандидатов рантайм-стадия сегодня
наследуется от `ghcr.io/krikz/rob_box_base:{ros2-zenoh,pcl,rtabmap}`, а эти базовые образы сами
наследуются от `ros:humble-ros-base` (upstream OSRF), который **уже содержит** `build-essential` +
`python3-colcon-common-extensions` + `python3-rosdep` + `python3-vcstool` + `git` — подтверждено чтением
исходного Dockerfile OSRF. Значит builder-стадия, которая просто переиспользует ту же базу
(`FROM ${BASE_IMAGE} AS builder` / `FROM ${BASE_IMAGE}` без замены базы для рантайма), **не уберёт**
унаследованный тулчейн из финального слоя — она уберёт только то, что **сама** сервисная Dockerfile
добавляет поверх базы (например, `build-essential`/`python3-dev`/`libffi-dev` в `led_matrix`). Полное
устранение тулчейна из рантайма требует либо (а) отдельной, более лёгкой рантайм-базы без
`build-essential`/`colcon`/`rosdep`, либо (б) остаётся частичным. Это разбирается честно в разделе 2 и
влияет на реалистичные ожидания в разделах 3, 5 и 8.

Пилот: **`docker/vision/led_matrix/Dockerfile`** (раздел 5). Не самый сложный (в отличие от
`vision-hailo` нет закрытых бинарников) и не тривиальный (реальный C-тулчейн: `build-essential` +
`python3-dev` + `libffi-dev` под сборку `spidev`), самодостаточный (не висит в FROM-цепочке
`voice-assistant`/`voice-base`), с уже существующим healthcheck и малым blast radius (используется только
на Vision Pi, ничто от него не наследуется).

---

## 1. Инвентаризация

### 1.1. Файлы без `colcon build` (20 из 32) — сборочный шов неприменим

Эти Dockerfile не компилируют собственный ROS-пакет: либо чистый `apt-get install` поверх готовой базы
(рантайм и так тонкий, добавить нечего вынести), либо `FROM` специализированного образа без сборки в
принципе. Разделять их builder/runtime — **выдумывать работу**: тулчейна, который стоило бы убрать,
здесь либо нет вовсе, либо он целиком унаследован от базового образа (см. раздел 2.1) и не устраняется
локальной правкой этого файла.

| Dockerfile | Строк | Что делает | Вывод |
|---|---|---|---|
| `docker/base/Dockerfile.depthai` | 77 | `FROM luxonis/depthai-ros:v2.12.2-humble` — сторонняя специализированная база | не трогать |
| `docker/base/Dockerfile.pcl` | 49 | `apt-get install libpcl-dev` и т.п. поверх `ros2-zenoh` | не трогать |
| `docker/base/Dockerfile.ros2-zenoh` | 80 | `FROM ros:humble-ros-base` + `.deb` `rmw_zenoh_cpp` (vendor) | не трогать — это и есть общая база, см. §2.1 |
| `docker/base/Dockerfile.rtabmap` | 73 | `FROM introlab3it/rtabmap_ros:humble-latest` + zenoh | не трогать |
| `docker/build/test/Dockerfile` | 32 | `FROM ubuntu:22.04`, `apt-get install git curl` — тестовый образ CI-инфраструктуры | не трогать |
| `docker/main/nav2/Dockerfile` | 57 | apt-only (nav2-bringup и т.д.) | не трогать |
| `docker/main/robot_state_publisher/Dockerfile` | 31 | apt-only (robot-state-publisher, xacro) | не трогать |
| `docker/main/rtabmap/Dockerfile` | 15 | только `FROM ${BASE_IMAGE}:rtabmap`, ни одного `RUN` сверх | не трогать |
| `docker/main/twist_mux/Dockerfile` | 27 | apt-only (twist-mux) | не трогать |
| `docker/main/zenoh-router/Dockerfile` | 9 | apt-only (rmw-zenoh-cpp) | не трогать — эталон «уже тонкий» |
| `docker/vision/apriltag/Dockerfile` | 22 | apt-only (apriltag-ros) | не трогать |
| `docker/vision/ceiling-camera/Dockerfile` | 28 | apt-only (usb-cam) | не трогать |
| `docker/vision/oak-d/Dockerfile` | 25 | apt-only поверх `depthai` базы | не трогать |
| `docker/vision/supercollider/Dockerfile` | 31 | `FROM ubuntu:26.04`, apt-only (supercollider-server) | не трогать |
| `docker/vision/test/mock_llm/Dockerfile` | 11 | `FROM python:3.11-slim`, тестовый двойник | не трогать |
| `docker/vision/test/ollama/Dockerfile` | 7 | `FROM ollama/ollama:latest` + entrypoint | не трогать |
| `docker/vision/test/scenario_runner/Dockerfile` | 16 | `FROM voice-assistant-humble-latest`, pip-only поверх | зависит от voice_assistant, см. §1.2 |
| `docker/vision/voice_resources/Dockerfile` | 29 | `FROM python:3.11-slim`, скачивает Renardo sample packs | не сборка кода — другая задача (доставка ассетов), не путать с этим швом |
| `docker/vision/zenoh-router/Dockerfile` | 9 | apt-only (rmw-zenoh-cpp) поверх depthai | не трогать |
| `docker/ci/Dockerfile` | 107 | Тулчейн-образ для CI (`build-essential`, `cmake`, `python3-colcon-common-extensions`, `ros-dev-tools`, `clang`, `lcov` — строки 55-88), но **сам не вызывает `colcon build`** — комментарий строк 14-15 лишь документирует, как им пользуются потребители | не сборка, это уже готовый прообраз общего builder-образа, см. §4.3 |

Факт-проверка `docker/ci/Dockerfile`: несмотря на «established facts» в задании, где он упомянут в списке
«тулчейн ставится и не удаляется», сам файл **не запускает `colcon build`** (`grep -n "colcon build"
docker/ci/Dockerfile` находит только закомментированные примеры на строках 14-15). Это CI-раннер-образ,
переиспользуемый workflow'ами как `container: image: ghcr.io/krikz/rob-box-ci:humble` — потребитель сам
монтирует исходники и гоняет `colcon build`/`colcon test` внутри контейнера. Тулчейн здесь установлен
**намеренно** и не подлежит вырезанию — это и есть весь смысл образа. Уточнение факта, установленного
в задании.

### 1.2. Файлы с `colcon build` (12 из 32) — кандидаты

| Dockerfile | Строк | `colcon build` | Пакеты | build_type | Свой тулчейн в файле | COPY src (свой) | Реальный выигрыш от разделения |
|---|---|---|---|---|---|---|---|
| `docker/main/vesc_nexus/Dockerfile` | 95 | :72 | `vesc_msgs`, `vesc_nexus` | ament_cmake (оба, проверено `package.xml`) | `ros-dev-tools` :22 (мета-пакет, тянет colcon+сборочные инструменты) | :50-68, из субмодуля `src/vesc_nexus/src/...` | **высокий** — настоящий C++ компилятор нужен, пакет собирается **повторно** в `ros2_control` (см. ниже) |
| `docker/main/ros2_control/Dockerfile` | 96 | :73 | `vesc_msgs`, `vesc_nexus`, `rob_box_description` | ament_cmake (все три) | явного `build-essential`/`cmake` нет, только rosidl-generators :33-34 | :60-64 | **высокий** — тот же C++-код, что и в `vesc_nexus`, собирается **второй раз** отдельно; общий builder убирает дублирование |
| `docker/vision/led_matrix/Dockerfile` | 106 | :80 | `led_matrix_driver`, `led_matrix_compositor` | ament_python (оба) | `python3-dev`+`build-essential`+`libffi-dev` :28,30,31 (под pip `spidev`, C-расширение без prebuilt wheel) | :74-76 | **средний-высокий** — свой тулчейн явно лишний в рантайме, самодостаточный образ (см. §5) |
| `docker/main/perception/Dockerfile` | 69 | :55 | `rob_box_perception_msgs` (ament_cmake), `rob_box_perception` (ament_python) | смешанный | нет explicit build-essential (наследуется от базы) | :50-51 | **средний** — rosidl codegen для `_msgs` реален; тот же исходник собирается повторно в `vision-hailo` |
| `docker/vision/vision-hailo/Dockerfile` | 138 | :108 | те же `rob_box_perception_msgs`+`rob_box_perception`, что и perception | смешанный | нет explicit | :95,103 | **средний**, но интерфейс шва здесь **шире**: кроме `/ws/install` нужен закрытый `libhailort.so.4.24.0` + `hailo_platform` wheel (см. §2.3, дополнение по vision-hailo) |
| `docker/main/lslidar/Dockerfile` | 40 | :37 | внешний драйвер `lslidar_ros2_driver` (git clone :26, ветка M10P/N10P) | не проверено (исходник не в этом дереве, склонирован в момент сборки) | нет explicit build-essential (наследуется), но `libboost-dev`/`libboost-thread-dev` :18-19 | нет (COPY отсутствует — источник это `git clone` в теле Dockerfile) | **средний** — builder изолирует сетевую зависимость (git clone от Lslidar) от рантайма, плюс настоящий C++ |
| `docker/main/teleop/Dockerfile` | 34 | :27 | `rob_box_teleop` | ament_python | нет своего тулчейна сверх базы | :16 (`COPY --chown=ros2:ros2 src/rob_box_teleop ...` — не поймано простым `grep "^COPY src/"`, но источник копируется) | **низкий** — чистый Python, своего тулчейна не добавляет, экономить особо нечего на уровне этого файла |
| `docker/vision/telegram_bot/Dockerfile` | 70 | :67 | `rob_box_core`, `rob_box_telegram` | ament_python (оба) | нет своего тулчейна сверх базы | :... (2 пути) | **низкий** — тот же случай, что teleop |
| `docker/vision/quest/Dockerfile` | 290 | :263 | `rob_box_supervisor_msgs` (ament_cmake), `rob_box_core`+`rob_box_quest` (ament_python) | смешанный | нет своего тулчейна сверх базы, только runtime ROS-пакеты (ament-cmake/rosidl-generators ставятся временно только для сборки IDL, строки 138-140 — но не убираются) | :220-251 | **низкий** — свой `colcon build` уже «лёгкий» (комментарий :256: «~30-60 сек»); уже применил правильный урок (сменил базу с voice-assistant на ros2-zenoh, ARCH-quest #2278) в другом месте шва (внешние тулы, не colcon) |
| `docker/vision/supervisor/Dockerfile` | 107 | :92 | `rob_box_supervisor` | ament_python | нет | :... | **нулевой на уровне этого файла** — база `voice-assistant-humble-latest` (:22) тянет весь 11.3 GB; разделение здесь ничего не даст, пока не тронут выше по цепочке |
| `docker/vision/voice_assistant/Dockerfile` | 317 | :284 | `rob_box_core`, `rob_box_perception_msgs`, `rob_box_mcp_tools`, `rob_box_llm`, `rob_box_harness`, `rob_box_voice`, `rob_box_animations` | смешанный | нет своего build-essential (наследует voice_base) | 42 путей | **нулевой на практике** — измерено `docker history` на локально закешированном образе: этот слой `colcon build` весит **11.3 MB** (!) против 5.05 GB pip-слоя и 2.73 GB apt-слоя того же образа. Резать тулчейн здесь бессмысленно — экономия тонет в погрешности |
| `docker/vision/voice_base/Dockerfile` | 198 | :183 | `audio_common_msgs` (ament_cmake) | ament_cmake | `python3-dev` :39, `ros-dev-tools` :57, `ros-humble-ament-cmake` :65 | нет (пакет тянется `git clone` :168-172) | **реальный, но опасный** — измерено `docker history`: слой с `ros-dev-tools`+`ros-humble-ros-base` весит **700 MB**, соседний слой с `ament-cmake`+`rosidl-default-generators`+прочим тулчейном voice_base весит **374 MB** (но в него же попадают нужные в рантайме `std-msgs`/`sensor-msgs`, так что не все 374 МБ уходят). Цена правки — инвалидация ~10 GB кеша для `voice_assistant`, `supervisor`, `scenario_runner` (тест) |

Числа `docker history` получены **локально** командой `docker history --no-trunc --format "{{.Size}}\t{{.CreatedBy}}" ghcr.io/krikz/rob_box:voice-assistant-humble-latest` на образе, уже закешированном в Docker Desktop на этой машине (arm64, 11 321 026 313 байт = 11.3 GB, тег `voice-assistant-humble-latest`, id `bc50db363589`, дата pull — 4 недели назад). Это не «established facts» из задания, а измерение, сделанное в ходе подготовки этого документа — рекомендуется перепроверить перед реализацией (`docker pull` мог обновиться).

---

## 2. Интерфейс шва

### 2.1. Ключевой факт, меняющий ожидания: тулчейн живёт в общей базе, не в сервисных Dockerfile

`ros:humble-ros-base` (апстрим OSRF, `FROM` в `docker/base/Dockerfile.ros2-zenoh:3`) по официальному
Dockerfile OSRF (`osrf/docker_images`, `ros/humble/ubuntu/jammy/ros-base/Dockerfile`) устанавливает в
первом же `RUN`:

```
build-essential, git, python3-colcon-common-extensions, python3-colcon-mixin,
python3-rosdep, python3-vcstool
```

Затем — метапакет `ros-humble-ros-base`. Наши образы `rob_box_base:ros2-zenoh` (`docker/base/Dockerfile.ros2-zenoh`),
`rob_box_base:pcl` (`FROM rob_box_base:ros2-zenoh`, `docker/base/Dockerfile.pcl:3-4`) и, по всей видимости,
`introlab3it/rtabmap_ros:humble-latest` (внешний образ, `docker/base/Dockerfile.rtabmap:3` — содержимое не
проверено, но `rtabmap_ros` тоже собирает C++ и типично строится поверх такого же `ros-base`) **наследуют**
этот тулчейн, а не переустанавливают его.

Практическое следствие: если builder-стадия и runtime-стадия одного и того же Dockerfile обе делают
`FROM ${BASE_IMAGE}` (одна и та же `rob_box_base:X`), то `build-essential`/`colcon`/`rosdep`/`git`/`vcstool`
**останутся в рантайм-образе** независимо от того, сколько стадий в файле — они пришли с базой, общей
для обеих стадий. Локальная builder-стадия в этом случае убирает только то, что **сама сервисная
Dockerfile** добавляет поверх базы (например, `python3-dev`+`build-essential`+`libffi-dev` в `led_matrix`,
или `ros-dev-tools` в `vesc_nexus`/`voice_base`) — и не трогает унаследованное. Это не делает локальную
builder-стадию бесполезной (реальный, измеримый кусок веса убирается — см. §1.2, колонка «реальный
выигрыш»), но означает, что цифры вида «образ стал в 5 раз тоньше» для большинства кандидатов —
несостоятельны. Полное избавление от тулчейна потребовало бы отдельной, более лёгкой рантайм-базы
(`rob_box_base:ros2-zenoh-runtime` без build-essential/colcon/rosdep/git/vcstool) — это отдельная, более
крупная инициатива, вне рамок этого документа (см. §12, открытый вопрос).

Второе следствие того же факта: `apt-get purge -y build-essential ... && apt-get autoremove` **в отдельном
`RUN`-слое после** `colcon build`, но **в той же стадии**, не уменьшает финальный размер образа. Docker
слои аддитивны — предыдущий слой с уже установленными файлами зафиксирован в истории образа
(`docker history` покажет оба слоя, суммарный размер не уменьшится). Единственный способ реально убрать
байты — начать **новую** стадию (`FROM` заново) и скопировать в неё только нужное через `COPY --from`.
Отсюда и востребованность многостадийной сборки, а не in-place purge.

### 2.2. Что реально пересекает `COPY --from` для ROS2/colcon — интерфейс не глубже `/ws/install`

Для чистого `ament_python`-пакета (`led_matrix_driver`, `led_matrix_compositor`, `rob_box_core`,
`rob_box_telegram`, `rob_box_teleop`, `rob_box_perception`, `rob_box_quest`, `rob_box_supervisor` —
все проверены по `package.xml`, тег `<build_type>ament_python`) `colcon build` — это, по сути,
`setup.py install` плюс генерация `ament_index`-маркеров. Компилятор не участвует. Артефакт: каталог
`/ws/install/<pkg>/{lib,share,bin}` — Python-код и entry points, обычный `site-packages`-путь плюс
`share/<pkg>/package.xml`+`ament_index`-маркер. Runtime-зависимости — рантайм-пакеты ROS (`sensor_msgs`,
`std_msgs` и т.п.), которые **не нужно** переносить через `COPY --from`, потому что они ставятся `apt`
и общие для builder/runtime (если рантайм-стадия ставит их сама).

Для `ament_cmake`-пакета с IDL (`rob_box_supervisor_msgs`, `rob_box_perception_msgs`, `vesc_msgs`,
`vesc_nexus`, `rob_box_description`, `audio_common_msgs`) картина сложнее — здесь **есть** реальная
компиляция и настоящая граница между build-time и runtime зависимостями `rosidl`:

- `rosidl-default-generators` (и его транзитивные `rosidl-generator-c`/`-cpp`/`-py`) — это **кодогенератор**,
  вызывается только во время `colcon build` для превращения `.msg`/`.srv` в `.c`/`.cpp`/`.py`. **Не нужен**
  в рантайме — сгенерированный код уже лежит в `/ws/install`.
- `rosidl-default-runtime` (и `rosidl-runtime-c`/`-cpp`, `rosidl-typesupport-c`/`-cpp`/`-fastrtps-c`/`-cpp`) —
  это **рантайм-библиотеки типовой поддержки** (typesupport `.so`), к которым линкуется/подключается
  сгенерированный код. **Обязательно** нужен в рантайме, иначе `import rob_box_supervisor_msgs.msg` или
  аналог на C++ падает с ошибкой линковки/импорта в момент старта ноды, а не на этапе сборки — то есть
  ошибка обнаружится не в CI сборки образа, а при первом запуске ноды на роботе. Это тот самый риск,
  который раздел 10 просит ловить в CI, а не на Pi.
- `ament-cmake` (и CMake/build-essential/gcc/g++) — build-time инструмент сборки. Не нужен в рантайме.

Это не выдумано для документа, а стандартная архитектура `rosidl` в ROS2 (генераторы/рантайм разнесены
по разным `.deb`-пакетам сознательно, начиная с ROS2 Dashing). В этом репозитории прямого подтверждения
через тестовую сборку builder/runtime не проводилось — при реализации пилота (раздел 5) первый же прогон
CI подтвердит или опровергнет это на практике, поэтому раздел 9 (acceptance) требует именно `ros2 node
list`/healthcheck на реальном контейнере, а не просто «образ собрался».

Итого — интерфейс шва для colcon-кандидатов:

| Что копируется `COPY --from=builder` | Обязательно в рантайме? |
|---|---|
| `/ws/install/**` (весь colcon workspace install-base) | да, это и есть продукт сборки |
| `ros-*-rosidl-default-runtime` + typesupport `.so` (`ros-*-rosidl-runtime-c/cpp`, `ros-*-rosidl-typesupport-*`) | да — apt-пакет, ставится в рантайм-стадии напрямую (не через COPY — apt проще и надёжнее, чем копировать `/opt/ros` вручную) |
| `ros-*-rosidl-default-generators` | нет — build-only |
| `ros-*-ament-cmake`, `build-essential`, `cmake` | нет — build-only |
| Рантайм-версии сообщений (`ros-*-std-msgs`, `ros-*-sensor-msgs` и т.п.) | да, но это уже стоит в базе или явно ставится в runtime-стадии — не через COPY |
| pip-пакеты с C-расширениями (пример: `spidev` в led_matrix) | да, но **осторожно**: копировать собранный `.so` можно только если ABI builder- и runtime-стадии совпадают (одна и та же Python/glibc версия — гарантировано, если обе стадии используют один и тот же `${BASE_IMAGE}`) |

### 2.3. Дополнение: интерфейс шва для `vision-hailo` шире, чем `/ws/install`

Для `docker/vision/vision-hailo/Dockerfile` через шов должно пройти больше, чем ROS-workspace:

1. **`/ws/install`** — как у всех colcon-кандидатов (`rob_box_perception_msgs`, `rob_box_perception`).
2. **`libhailort.so.4.24.0`** — рантайм shared-библиотека HailoRT. Сейчас устанавливается в текущем
   единственном слое командой `install -m 0755 /wheels/libhailort.so.4.24.0 /usr/lib/` +
   `ln -sf libhailort.so.4.24.0 /usr/lib/libhailort.so` + `ldconfig`
   (`docker/vision/vision-hailo/Dockerfile:120-123`). Это **обязательно** в рантайм-стадии — без неё
   `import hailo_platform` упадёт при старте. Сама библиотека не компилируется в этом образе (она
   приходит уже собранной из закрытого `.deb`/архива Hailo Developer Zone, распакованного воркфлоу —
   см. п.4 ниже), поэтому она не «продукт builder-стадии» в привычном смысле — она такой же внешний
   бинарный артефакт, как `caddy` в `docker/vision/quest/Dockerfile:169` (`COPY --from=caddy-builder`).
   Технически правильное место для неё в multi-stage схеме — отдельная псевдо-стадия
   `FROM scratch AS hailo-artifacts` с `COPY docker/vision/vision-hailo/wheels/ /` (по аналогии со
   `Stage 1.5: caddy-builder` в quest, docker/vision/quest/Dockerfile:69), из которой обе стадии (builder
   и runtime) берут нужное через `COPY --from=hailo-artifacts`, не тряся build-context дважды.
3. **`hailo_platform`** (Python wheel `hailort-4.24.0-cp310-cp310-linux_aarch64.whl`) — устанавливается
   `pip3 install /wheels/hailort-4.24.0-cp310-cp310-linux_aarch64.whl` (Dockerfile:119). Это готовый wheel
   (cp310, ABI-совместимый под конкретный Python 3.10 из базового образа), компиляции не требует — значит
   его можно ставить **прямо в рантайм-стадии**, минуя builder, если рантайм-стадия использует тот же
   Python 3.10 ABI, что и сегодня (что гарантировано, если обе стадии — производные одной и той же
   `rob_box_base:ros2-zenoh-humble`). Это упрощает интерфейс: wheel не обязан быть продуктом builder-стадии,
   он идёт напрямую в runtime.
4. **`hailort-pcie-driver_4.24.0_all.deb`** — это **драйвер ядра**, ставится на **хост** (Vision Pi), а не
   в образ. Подтверждается ADR-0089 §4: «Docker | ⚠️ Custom base | `--device /dev/hailo0` (preferred) или
   `privileged: true`; ARM64 HailoRT deb» (`docs/adr/0089-ai-hat-plus-deployment.md:192`) и §7 R1/R2 —
   раздел 3 «Hardware-совместимость» описывает `/dev/hailo0` как устройство, пробрасываемое в контейнер,
   а не собираемое внутри него. В самом репозитории явного текста «pcie-driver ставится на хост, не в
   образ» **не найдено** (`grep -rn "hailort-pcie-driver" docs/ docker/ .github/` находит только три
   упоминания в ADR-0089:191, ADR-0099:54, и устаревшем `docs/reports/AI_HAT_UPGRADE_ANALYSIS.md:275` —
   ни одно не формулирует явно «на хосте, не в образе»), но паттерн `/dev/hailo0` как pass-through
   устройство однозначно подразумевает host-side kernel-модуль. **Вывод**: `hailort-pcie-driver` — вне
   зоны ответственности сборочного шва, независимо от builder/runtime разделения. Доставка и версионирование
   вендорских артефактов (включая соотношение driver/runtime/wheel версий) — предмет параллельного плана
   `docs/plans/2026-09-15-resource-pack.md`.

**Как разделение builder/runtime не должно сломать текущий capability-honest режим.** Сегодня в
`docker/vision/vision-hailo/Dockerfile:113-129` установка обёрнута в `if [ "${HAILO_INSTALL_BINDING}" =
"whl" ]`, иначе — печатает предупреждение и **не падает** (собирается stub-режим, ADR-0099 §2.2). Но CI
workflow (`.github/workflows/L-Build Vision Pi Services.yml:507-522`, шаг «Fetch HailoRT wheel + lib»)
делает `exit 1` (строка 519), если файлов нет ни в `/opt/rob_box/vendor`, ни в `/tmp` на build-машине —
то есть **сегодня production CI жёстко падает** при отсутствии закрытых артефактов на katana, а не
деградирует. При этом `HAILO_INSTALL_BINDING=whl` передан безусловно (строка 537) — деградация в
stub предусмотрена в самом Dockerfile (на случай локальной сборки без CI-обвязки), но в текущем CI-пайплайне
не задействована. **Требование к разделению**: builder/runtime-схема обязана сохранить оба поведения —
CI на katana (или self-hosted `rob-box`) продолжает жёстко падать при отсутствии вендорских файлов
(это намеренно, чтобы не выкатить на робота молча деградировавший до stub образ под видом «полного»),
а сборка **вне** этого CI (`docker build` без прокинутых wheels, `HAILO_INSTALL_BINDING=none` по
умолчанию) — продолжает собираться и работать в stub-режиме без ошибки. Формализовано в acceptance §9.

---

## 3. Кандидаты на разделение, по убыванию выигрыша

Порядок — по произведению «реальный вес убираемого тулчейна» × «безопасность/изолированность» ÷ «цена
инвалидации кеша». Файлы без `colcon build` (раздел 1.1) в список не входят — там нечего разделять.

1. **`docker/main/vesc_nexus/Dockerfile`** — настоящий C++ (`ament_cmake`), явный `ros-dev-tools` (:22),
   изолирован (не в FROM-цепочке с voice_assistant/rtabmap-тяжёлыми образами — база `ros2-zenoh`), а
   главное — **тот же код собирается второй раз** в `ros2_control` (см. следующий пункт). Общий builder
   убирает дублирование компиляции, не только тулчейн.
2. **`docker/main/ros2_control/Dockerfile`** — компилирует `vesc_msgs`+`vesc_nexus` (те же пакеты, что и
   выше) плюс `rob_box_description`. База `rtabmap` (:9) уже тяжёлая сама по себе (не трогаем — раздел 1.1
   базовых образов вне скоупа), поэтому относительный процент выигрыша меньше, чем у `vesc_nexus`, но
   абсолютная экономия сопоставима, и убирается второе дублирование той же C++ компиляции.
3. **`docker/vision/led_matrix/Dockerfile`** — **пилот** (раздел 5). Самодостаточный, свой тулчейн explicit
   (`build-essential`+`python3-dev`+`libffi-dev`, :28,30,31), умеренная сложность (`ament_python` + один
   C-extension `spidev`), нет закрытых бинарников, нет FROM-цепочки на что-то тяжёлое.
4. **`docker/main/perception/Dockerfile`** — реальный `rosidl`-кодогенератор для `rob_box_perception_msgs`
   (`ament_cmake`), плюс тот же исходник (`rob_box_perception`+`rob_box_perception_msgs`) собирается
   **повторно** в `vision-hailo` (см. следующий пункт) — тот же аргумент про дублирование, что у
   vesc_nexus/ros2_control.
5. **`docker/vision/vision-hailo/Dockerfile`** — та же пара пакетов, что и perception (дублирование),
   но интерфейс шва **шире** (раздел 2.3: закрытый `.so` + wheel + host-driver-исключение) и сборка
   зависит от вендорских артефактов на katana — усложняет пилотирование, поэтому не первый кандидат,
   но не последний по значимости: это единственный образ с полноценным примером «builder производит
   код, а бинарный сторонний артефакт приходит отдельным путём», близким к тому, что уже делает quest
   для caddy.
6. **`docker/main/lslidar/Dockerfile`** — настоящий внешний C++-драйвер (`git clone` в теле сборки,
   :26), сетевая зависимость от `github.com/Lslidar` изолируется в builder-стадию (рантайм-стадия не
   требует сетевого доступа к GitHub при пересборке — только к своему registry за артефактом builder'а).
7. **`docker/main/teleop/Dockerfile`** и **`docker/vision/telegram_bot/Dockerfile`** — чистый
   `ament_python`, свой тулчейн сверх базы не ставят. Разделение уберёт близкий к нулю объём на уровне
   этого файла (см. §2.1 — унаследованный тулчейн не устраняется локальной стадией). Делать **низкий
   приоритет**, разве что заодно с базовой рефакторизацией (§12).
8. **`docker/vision/quest/Dockerfile`** — **не трогать сейчас**. Уже прошёл собственную оптимизацию
   (ARCH-quest #2278, комментарии :12-20) — сменил базу с `voice-assistant` (~10 GB) на `ros2-zenoh`
   (~250 MB, по собственному комментарию :19) и уже использует multi-stage для внешних инструментов
   (caddy, webxr). Его **собственный** `colcon build` — маленький и быстрый (комментарий :256: «~30-60
   сек»), выносить в отдельную стадию почти нечего.
9. **`docker/vision/supervisor/Dockerfile`** и **`docker/vision/voice_assistant/Dockerfile`** — **не
   окупается на уровне этих файлов**. Оба наследуют FROM-цепочку от `voice-base-humble-latest`
   (11.3 GB измерено). Их собственный слой `colcon build` — единицы МБ (у voice_assistant измерено:
   11.3 MB). Тратить время на builder-стадию здесь бессмысленно, пока не тронута цепочка выше
   (`voice_base` → см. следующий пункт). Честно: выигрыш **копеечный**.
10. **`docker/vision/voice_base/Dockerfile`** — реальный тулчейн-вес измерен (~700 MB + частично 374 MB,
    раздел 1.2), но это низкий приоритет **не по размеру, а по риску**: любая правка этого файла
    инвалидирует ~10 GB кеша для `voice_assistant`+`supervisor`+`scenario_runner` (тест), и это тот же
    принцип цены, что зафиксирован в ADR-0100 (§3.2: «build cache voice_base не инвалидируется при
    апгрейдах aiohttp — экономия ~10 GB pytorch/etc cache rebuild», `docs/adr/0100-aiohttp-apt-vs-pip-voice-assistant.md:126`)
    и в исходном коммите `337548a38` («Kept off voice_base on purpose: changing voice_base would
    invalidate the ~10 GB pytorch etc. cache for every consumer», процитировано в ADR-0100:34). Делать
    это отдельной, узкой, тщательно протестированной карточкой — не попутно с остальным планом.

**Явно не трогать**: все 20 файлов раздела 1.1 (нечего разделять) плюс `docker/main/zenoh-router/Dockerfile`
(9 строк, уже минимален — прямо назван в задании как ориентир «не усложнять то, что и так тонкое»).

---

## 4. Общий builder-образ или локальная builder-стадия в каждом Dockerfile?

### 4.1. Вариант (а) — общий builder-образ, отдельный job, `FROM ${BUILDER_IMAGE} AS builder`

Один образ (например, `rob_box_base:ros2-zenoh-builder`) со всем тулчейном (`build-essential`, `cmake`,
`python3-dev`, `ros-dev-tools`, `libboost-dev` и т.п. — объединение того, что сегодня разбросано по
`vesc_nexus`, `led_matrix`, `perception`, `lslidar`), собирается одним CI job'ом, кешируется в registry,
и переиспользуется как `ARG BUILDER_IMAGE=... / FROM ${BUILDER_IMAGE} AS builder` в каждом сервисном
Dockerfile.

- **Глубина интерфейса**: мельче для потребителя — сервисный Dockerfile не описывает СВОЙ тулчейн, только
  `ARG BUILDER_IMAGE` + `COPY src` + `colcon build --packages-select X`. Вся сложность тулчейна спрятана
  за одной точкой правды.
- **Локальность**: хуже — правка тулчейна в одном месте синхронно затрагивает все Dockerfile, которые на
  него ссылаются; при добавлении нового build-зависимости для одного сервиса (например, специфичный
  `libfoo-dev` только для `lslidar`) приходится либо раздувать общий builder всем, либо городить
  build-arg-переключатели внутри общего образа.
- **Время сборки**: builder собирается один раз, переиспользуется всеми — экономит суммарное CI-время
  при частых билдах нескольких сервисов подряд (типичный сценарий `L-Build Main Pi Services.yml` /
  `L-Build Vision Pi Services.yml`, где в одном workflow-run собираются 5-10 образов). Но builder-образ
  сам должен быть закеширован (registry cache), иначе каждый job тянет его заново.
- **Инвалидация кеша**: любое изменение общего builder-образа (даже для одного сервиса) требует пересборки
  builder-образа целиком и **инвалидирует кеш-referencing слой** у всех потребителей на следующей сборке
  (хотя их собственные COPY/build-слои по хешу source остаются кешированными отдельно — страдает только
  первый `FROM ${BUILDER_IMAGE}` слой).
- **Прецедент в репозитории**: `docker/ci/Dockerfile` — уже реализованный общий тулчейн-образ, просто
  для CI-тестов, а не для рантайм-сборки сервисов (раздел 1.1). Это готовый шаблон, из которого можно
  унаследовать список пакетов, а не изобретать заново.

### 4.2. Вариант (б) — локальная builder-стадия в каждом Dockerfile

`FROM ${BASE_IMAGE} AS builder` в каждом сервисном Dockerfile отдельно, со своим набором build-зависимостей.

- **Глубина интерфейса**: глубже для читателя одного файла (весь контракт «что нужно для сборки» виден
  в одном месте, не надо смотреть в отдельный builder-образ), но мельче системно — 12 файлов, 12 почти
  одинаковых списков `build-essential`/`cmake`/`rosidl-generators`, разъезжающихся по мелочи (как уже
  происходит сегодня: `led_matrix` ставит `libffi-dev`, `vesc_nexus` — `ros-dev-tools`, `perception` —
  ничего своего).
- **Локальность**: лучше — правка тулчейна для одного сервиса не трогает остальные 11 файлов, PR
  ограничен одним файлом (см. AF-0013, принцип «инкрементальная поставка», раздел 11).
- **Время сборки**: каждый Dockerfile тянет и кеширует свою builder-стадию отдельно — при параллельных
  job'ах (как сегодня в `L-Build Vision Pi Services.yml`/`L-Build Main Pi Services.yml`, где сервисы
  собираются параллельными job'ами на одном self-hosted runner'е) это не хуже общего builder-образа,
  потому что job'ы и так параллельны, а Docker-layer-cache на **одном и том же** self-hosted runner'е
  (`[self-hosted, rob-box]`, `.github/workflows/L-Build Vision Pi Services.yml:481` и др.) переиспользует
  слои между job'ами через общий локальный daemon (при условии `--load`, не `--push` в чистый registry
  без layer-cache — см. раздел 6).
- **Инвалидация кеша**: изменение тулчейна для одного сервиса не трогает кеш остальных — узкий blast
  radius.
- **Дублирование сборки исходников**: не решает проблему из раздела 3 (vesc_msgs/vesc_nexus собираются
  дважды, rob_box_perception(_msgs) — дважды) — каждый Dockerfile со своей builder-стадией всё равно
  компилирует те же исходники заново.

### 4.3. Рекомендация

**Локальная builder-стадия (вариант б) — по умолчанию**, с одним отступлением под конкретную проблему
дублирования (раздел 3, пункты 1-2 и 4-5): для пар `vesc_nexus`↔`ros2_control` и
`perception`↔`vision-hailo`, где один и тот же исходник компилируется дважды, — рассмотреть **общий
builder-образ именно для этой пары**, а не для всех 12 кандидатов сразу. Причины:

1. Self-hosted раннер один (`[self-hosted, rob-box]`, katana) — Docker layer cache на локальном демоне
   уже частично даёт эффект общего builder-образа без явной архитектуры под него: если два job'а на том
   же демоне используют одинаковый `FROM ${BASE_IMAGE}` + одинаковые ранние слои, buildx их переиспользует
   локально. Полноценный общий builder-образ добавляет ценность прежде всего там, где он реально убирает
   **повторную компиляцию одного и того же исходника**, а не только повторную установку одного и того же
   apt-пакета (которую локальный layer cache и так решает бесплатно).
2. AF-0013 (раздел 11) явно требует инкрементальной поставки, ≤50 коммитов / ≤3000 строк на PR — общий
   builder-образ для всех 12 файлов разом означает один большой рефакторинг с широким blast radius
   (правка тулчейна ломает 12 файлов одновременно при ошибке), что плохо ложится на это правило. Локальные
   builder-стадии позволяют мержить по одному сервису за раз — раздел 7 построен именно так.
3. `docker/ci/Dockerfile` уже даёт прецедент и список пакетов для переиспользования — общий builder-образ
   для пары vesc_nexus/ros2_control и perception/vision-hailo может быть реализован как узкий overlay
   поверх существующего паттерна, не с нуля.

---

## 5. Пилот: `docker/vision/led_matrix/Dockerfile`

### 5.1. Почему именно этот файл

- Самодостаточный: `FROM ghcr.io/krikz/rob_box_base:ros2-zenoh` (:14), не висит в FROM-цепочке на
  `voice-assistant`/`voice-base` — правка не рискует ничьим ещё образом.
- Реальный, explicit тулчейн в самом файле (`python3-dev`, `build-essential`, `libffi-dev`, :28,30,31) —
  есть что реально убрать из рантайма (в отличие от `teleop`/`telegram_bot`, где своего тулчейна нет).
- Умеренная сложность: `ament_python` (не требует `rosidl`-кодогена, раздел 2.2 упрощается), один
  C-extension (`spidev`, у него нет prebuilt wheel под arm64 на PyPI — нужен реальный компилятор), два
  ROS-пакета.
- Нет закрытых бинарников (в отличие от vision-hailo) — пилот проверяет саму механику шва, не решает
  параллельно проблему доставки вендорских артефактов.
- Уже есть `HEALTHCHECK` (:99-100) — готовый критерий acceptance «нода стартует» без дополнительной
  разработки.
- Изолированный blast radius: используется только сервисом `led-matrix` в
  `docker/vision/docker-compose.yaml:68-101`, ничто от него не наследуется.

### 5.2. Будущий Dockerfile целиком

```dockerfile
# Dockerfile для LED Matrix Driver и Compositor
# Управление NeoPixel LED матрицами через SPI на Vision Pi (Raspberry Pi 5)
#
# Архитектура:
#   - led_matrix_driver: низкоуровневый драйвер (SPI → LED через pi5neo)
#   - led_matrix_compositor: композитор панелей (sensor_msgs/Image → driver)
#
# Физическая конфигурация:
#   - 4× панели 8×8 (фары передние/задние)
#   - 5× панели 5×5 (main_display 5×25)
#   - Общая цепочка: 381 LED (256 + 125)
#
# Сборочный шов (docs/plans/2026-09-15-builder-runtime-seam.md): builder-стадия
# ставит build-essential/python3-dev/libffi-dev (нужны только для компиляции
# C-расширения pip-пакета spidev — под arm64 нет prebuilt wheel) и собирает
# colcon workspace. Рантайм-стадия НЕ содержит компилятор и заголовки — только
# то, что нужно для запуска: apt runtime-пакеты ROS, pip site-packages
# (скопированы как готовые .so/.py, ABI гарантированно совпадает — обе стадии
# используют один и тот же BASE_IMAGE) и /ws/install.
#
# ВНИМАНИЕ (см. §2.1 плана): BASE_IMAGE сам наследует build-essential/colcon от
# ros:humble-ros-base (апстрим OSRF) — эта стадия НЕ убирает унаследованный
# тулчейн, только то, что этот файл добавляет сверх базы. Полное устранение
# тулчейна требует отдельной облегчённой рантайм-базы (вне скоупа пилота).

ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh
ARG ROS2LEDS_SHA=unknown

# ============================================================
# Stage 1: builder — тулчейн + компиляция colcon workspace
# ============================================================
FROM ${BASE_IMAGE} AS builder

ARG ROS_DISTRO
ARG ROS2LEDS_SHA

# Тулчейн для сборки C-расширения spidev (нет prebuilt wheel под arm64 на
# PyPI) и для colcon build. build-essential/python3-dev/libffi-dev ставятся
# ТОЛЬКО в этой стадии — в рантайм они не попадут.
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    libffi-dev \
    && rm -rf /var/lib/apt/lists/*

# pip-пакеты ставим с --prefix, чтобы потом скопировать чистый каталог в
# рантайм-стадию (без кеша pip, без build-логов, без самого pip/setuptools).
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir --prefix=/pyinstall \
    pi5neo \
    spidev \
    numpy

WORKDIR /ws
RUN mkdir -p src

RUN echo "Building with ros2leds SHA: ${ROS2LEDS_SHA}" && \
    echo "${ROS2LEDS_SHA}" > /tmp/ros2leds_build_sha

# 1. package.xml — изменяются редко
COPY src/ros2leds/led_matrix_driver/package.xml /ws/src/led_matrix_driver/
COPY src/ros2leds/led_matrix_compositor/package.xml /ws/src/led_matrix_compositor/

# 2. ROS-зависимости через rosdep (кэшируется, если package.xml не менялся)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true

# 3. setup.py
COPY src/ros2leds/led_matrix_driver/setup.py /ws/src/led_matrix_driver/
COPY src/ros2leds/led_matrix_compositor/setup.py /ws/src/led_matrix_compositor/

# 4. исходный код (горячий слой)
COPY src/ros2leds/led_matrix_driver /ws/src/led_matrix_driver
COPY src/ros2leds/led_matrix_compositor /ws/src/led_matrix_compositor

# 5. Сборка
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select led_matrix_driver led_matrix_compositor \
    --symlink-install \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# ============================================================
# Stage 2: runtime — та же база, БЕЗ build-essential/python3-dev/libffi-dev
# ============================================================
FROM ${BASE_IMAGE}

ARG ROS_DISTRO

LABEL org.opencontainers.image.title="LED Matrix System for Vision Pi"
LABEL org.opencontainers.image.description="ROS2 LED matrix driver and compositor with pi5neo"
LABEL org.opencontainers.image.source="https://github.com/krikz/ros2leds"

# Только runtime ROS-зависимости (sensor_msgs/std_msgs нужны на импорт в
# рантайме, python3-spidev/pip3 — runtime-обвязка, БЕЗ python3-dev/build-essential).
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-spidev \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# pip site-packages из builder-стадии — уже собранные .so/.py, ABI совпадает
# (обе стадии — один BASE_IMAGE → одна версия Python/glibc).
COPY --from=builder /pyinstall /usr/local

# colcon workspace из builder-стадии.
COPY --from=builder /ws/install /ws/install

WORKDIR /ws
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV RUST_LOG=zenoh=info
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV ZENOH_CONFIG=/config/shared/zenoh_session_config.json5

HEALTHCHECK --interval=15s --timeout=5s --start-period=10s --retries=3 \
    CMD ros2 node list | grep -q led_matrix || exit 1

ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]

CMD ["bash", "-c", "source /ws/install/setup.bash && ros2 launch led_matrix_compositor led_matrix_compositor_launch.py"]
```

**Отличие от текущего файла**: `rosdep install` в builder-стадии подтягивает `ros-${ROS_DISTRO}-sensor-msgs`/
`std-msgs` через `package.xml`-зависимости (нужны для сборки IDL-совместимости), но в рантайм-стадии они
**переустанавливаются явно** (`apt-get install ros-${ROS_DISTRO}-sensor-msgs ros-${ROS_DISTRO}-std-msgs`),
а не копируются из builder — так надёжнее (apt разрешает транзитивные зависимости заново для чистой
стадии, а не полагается на то, что скопированный `/opt/ros/...` файл окажется консистентным без
остального дерева пакетов). Это единственная сознательная асимметрия интерфейса: часть runtime-зависимостей
идёт через `COPY --from`, часть — через собственный `apt-get` в рантайм-стадии.

**Реалистичное ожидание по размеру** (см. §2.1): убирается вес `build-essential` + `python3-dev` +
`libffi-dev` + pip build-артефакты (кеш pip, `.c`/`.o` промежуточные файлы компиляции `spidev`) —
унаследованный из `ros2-zenoh` базы тулчейн (`git`, `colcon`, `rosdep`, `vcstool`, тот же
`build-essential`, что уже сидит в базе) **останется** в обеих стадиях, потому что обе `FROM
${BASE_IMAGE}`. Ожидаемая экономия — не в разы, а на десятки-сотни МБ (точная цифра — предмет измерения
до/после, раздел 8, а не декларации в этом документе).

---

## 6. Кеширование

### 6.1. Текущее состояние

`.github/actions/l-build-service/action.yml` (переиспользуемый composite action для всех сервисных сборок)
**не содержит** ни `--cache-from`, ни `--cache-to` — подтверждено полным чтением файла: единственная
команда сборки — `docker buildx build --platform ... --file ... --tag ... --load --add-host=... --build-arg=...
--progress=... <context>` (строки 198-206), без единого кеш-флага. Комментарий в описании action'а
(строки 13-22) объясняет почему `--load`, а не `--push`: GHCR-теги пушить нельзя (раннер не залогинен,
issue #1503, run #34368750126), а `update-image-versions` job-у нужен образ в **локальном** Docker daemon
для `docker tag`. Поэтому пушится только `localhost:5000/...` (локальный registry на том же
self-hosted раннере).

Registry-кеш (`--cache-from type=registry,ref=...`) используется **только** при сборке базовых образов:
`.github/workflows/L-Build Base Images.yml:76,116,156,197` — четыре по числу баз (`ros2-zenoh`, `rtabmap`,
`depthai`, `pcl`). Единственное использование `type=gha` (кеш GitHub Actions) — `.github/workflows/B-Build
CI Image.yml:62-63`, для сборки `docker/ci/Dockerfile`.

### 6.2. Что даёт добавление `--cache-from type=registry` в `l-build-service/action.yml`

Раннер один и тот же физический хост (`katana`, self-hosted `[self-hosted, rob-box]`), и Docker daemon
на нём один — значит buildx уже переиспользует слои **локально** между последовательными job'ами через
локальный build-кеш демона, без явного `--cache-from`. Явный `--cache-from type=registry,ref=localhost:5000/...`
даёт выигрыш в первую очередь в двух случаях, которых сегодня локальный кеш не покрывает:

1. **Раннер потерял локальный кеш** (перезапуск демона, `docker builder prune`, смена раннера) —
   registry-кеш переживает такие события, локальный build-кеш демона — нет.
   **Cold build** сегодня платит полную цену пересборки заново.
2. **Явная параллельность builder+runtime стадий** после реализации плана — при разделении на стадии
   (раздел 5) builder-стадия одного сервиса (например, `vesc_nexus`) структурно совпадает с
   builder-стадией другого (`ros2_control`), если реализован общий builder-образ (раздел 4.3, вариант
   для пары vesc_nexus/ros2_control) — тогда `--cache-from` на общий builder-тег даёт реальный cross-job
   реюз, которого без явного кеш-флага полагаться на удачу локального демона рискованно (порядок job'ов
   в workflow не гарантирован, GC демона может вытеснить нужный слой раньше, чем второй job его
   переиспользует).

### 6.3. Совместимость с builder-стадией

Для локальной builder-стадии (раздел 4.3, вариант по умолчанию) `--cache-from` работает без изменений —
buildx кеширует **все** стадии multi-stage сборки по отдельности, если явно указать
`--cache-from type=registry,ref=<tag>` (кеш кодирует граф стадий, не только финальный слой). Для этого
нужен BuildKit inline-кеш или `--cache-to type=registry,ref=<tag>,mode=max` при сборке (сегодня в
`l-build-service/action.yml` нет ни того, ни другого) — `mode=max` обязателен, иначе кешируются только
слои финального образа, а промежуточные builder-слои — нет (это ключевая деталь: без `mode=max`
добавление `--cache-from` без соответствующего `--cache-to,mode=max` не даст выигрыша для builder-стадии
вообще, потому что нечего будет из чего кешировать).

### 6.4. Нужен ли отдельный cache-образ

Нет, отдельный образ не нужен — `type=registry` кеш записывается как обычный (уже существующий)
`localhost:5000/<tag>` образ с `--cache-to type=registry,ref=<тот же или отдельный тег>,mode=max`.
Практика в `L-Build Base Images.yml` уже использует такой паттерн (`ref=${LOCAL_PREFIX}:ros2-zenoh-${ROS_DISTRO}`
и т.п.) — то есть кеш **это и есть** предыдущий тег того же образа в локальном registry, не отдельная
сущность. Для builder-стадий по аналогии: `--cache-from type=registry,ref=localhost:5000/krikz/rob_box:led-matrix-builder-cache`
+ `--cache-to type=registry,ref=localhost:5000/krikz/rob_box:led-matrix-builder-cache,mode=max`.

### 6.5. Что делать с `--load`

`--load` кладёт собранный образ **в локальный Docker daemon**, не в registry. Между **разными**
job'ами (даже на одном раннере, если GitHub Actions создаёт для каждого job'а изолированный executor —
что на self-hosted раннерах **не обязательно так**: `[self-hosted, rob-box]` — это один и тот же процесс
`actions-runner` на одном хосте, значит Docker daemon физически общий между job'ами одного workflow-run'а,
и `--load` образ одного job'а **виден** следующему через тот же daemon без дополнительного кеш-флага, при
условии, что job'ы выполняются последовательно/на одном раннере, а не на разных self-hosted машинах).
Между **разными ранами** (двумя запусками workflow, отдельными коммитами) `--load` **не помогает** —
daemon кеш переживает только пока жив контейнер/VM раннера, при первой сборке нового коммита с нуля от
`--load` толку нет, тут и нужен `--cache-from type=registry`, который переживает между ранами, потому
что читает из `localhost:5000` — постоянного локального registry на том же хосте (не эфемерного).

**Итог для раздела 6**: добавление `--cache-from`/`--cache-to type=registry,mode=max` в
`l-build-service/action.yml` — независимое, самостоятельно ценное изменение (снижает cold-build риск
и даёт устойчивый layer-cache между ранами), которое стоит делать **до или параллельно** с
builder/runtime-разделением, а не после — иначе первые прогоны нового multi-stage Dockerfile будут
платить полную цену пересборки builder-стадии на каждом коммите.

### 6.6. Особый случай `vision-hailo`: build-context меняется от хоста

У `vision-hailo` build-context **не детерминирован source-кодом репозитория**: CI job
(`.github/workflows/L-Build Vision Pi Services.yml:507-522`) копирует `hailort-4.24.0-cp310-cp310-linux_aarch64.whl`
и `libhailort.so.4.24.0` в `docker/vision/vision-hailo/wheels/` **из `/opt/rob_box/vendor` на build-хосте
katana** непосредственно перед вызовом `l-build-service`. Это значит:

- Build-context (а значит и хеш контекста, на котором buildx строит cache key для `COPY docker/vision/vision-hailo/wheels/
  /wheels/`, Dockerfile:119) зависит от состояния файловой системы конкретной build-машины, а не только
  от git-дерева. Кеш **инвалидируется**, если файлы в `/opt/rob_box/vendor` поменялись (например, обновили
  версию HailoRT) — это правильное поведение, не баг.
- Но если у `/opt/rob_box/vendor` **нестабильная mtime** (файл перекопирован без изменения содержимого —
  например, при переустановке раннера) — buildx может решить, что слой `COPY .../wheels/` изменился, даже
  если байты идентичны, и инвалидировать кеш зря. Рекомендация: если после реализации builder/runtime
  разделения для `vision-hailo` кеш стабильно не работает, проверить `COPY` с `--link` (BuildKit, снижает
  чувствительность к порядку) или явный `sha256sum` в отдельном build-arg (тот же паттерн, что уже
  используется для `SOURCE_HASH` в этом же Dockerfile, ARG :101).
- Доставка и стабилизация самих вендорских артефактов (mirroring, версионирование, детерминированный
  путь) — предмет параллельного плана `docs/plans/2026-09-15-resource-pack.md`, здесь фиксируется только
  влияние на кеш сборочного шва.

---

## 7. Пошаговый план реализации

Каждый этап — отдельный PR, мержится независимо, не ломает прод (правило AF-0013: ≤50 коммитов / ≤3000
строк на PR, раздел 11).

**Этап 0 — кеширование (независимо, можно первым).**
Добавить `--cache-from`/`--cache-to type=registry,mode=max` в `.github/actions/l-build-service/action.yml`
(раздел 6). Не меняет ни один Dockerfile. Проверяется тем, что повторная сборка того же коммита ощутимо
быстрее (раздел 8).

**Этап 1 — пилот `led_matrix`.**
Реализовать Dockerfile из раздела 5. Один PR, один файл. Смотреть раздел 9 (acceptance) и раздел 10
(риски) перед мержем.

**Этап 2 — снять урок с пилота, обновить чек-лист.**
После мержа этапа 1 — короткое ретро (по аналогии с AF-0013): что в интерфейсе шва (раздел 2) оказалось
верно/неверно на практике, скорректировать шаблон для следующих файлов.

**Этап 3 — `vesc_nexus` + `ros2_control` (общий builder, раздел 4.3).**
Один builder-образ (или общая builder-стадия через `ARG BUILDER_IMAGE`) для обоих, убирающий дублирование
компиляции `vesc_msgs`/`vesc_nexus`. Два отдельных PR — сначала общий builder-образ + `vesc_nexus`
(наименьший из двух потребителей), потом `ros2_control` на него же.

**Этап 4 — `perception` + `vision-hailo` (общий builder, та же логика).**
Отдельно от этапа 3, потому что `vision-hailo` тянет дополнительную сложность закрытых артефактов
(раздел 2.3) — не смешивать с чисто ROS-задачей vesc_nexus/ros2_control в одном PR.

**Этап 5 — `lslidar`.**
Изолирует сетевую зависимость от `github.com/Lslidar` в builder-стадию.

**Этап 6 (опционально, низкий приоритет) — `teleop`, `telegram_bot`.**
Только если решено параллельно делать облегчённую рантайм-базу (раздел 12) — иначе выигрыш на уровне
этих файлов отдельно близок к нулю (раздел 3, пункт 7).

**Не планировать сейчас**: `quest` (уже достаточно оптимален), `supervisor`/`voice_assistant`/`voice_base`
(раздел 3, пункты 9-10 — низкий приоритет по разным причинам: первые два — незначительный выигрыш,
третий — высокий риск инвалидации кеша).

---

## 8. Как проверять, что стало лучше

### 8.1. Baseline — снять ДО начала работ

```bash
# Размер финального образа (сравнивать по тегу до/после)
docker image ls ghcr.io/krikz/rob_box:led-matrix-humble-latest --format "{{.Size}}"

# Разбивка по слоям — сколько весит apt-слой с build-essential/python3-dev/libffi-dev
docker history --no-trunc --format "{{.Size}}\t{{.CreatedBy}}" ghcr.io/krikz/rob_box:led-matrix-humble-latest \
  | sort -rh | head -20

# Список установленных toolchain-пакетов внутри текущего (одностадийного) образа —
# сравнить dpkg-list ДО и ПОСЛЕ, чтобы подтвердить, что build-essential/python3-dev/
# libffi-dev реально исчезли из рантайма, а не просто "не видны" в истории
docker run --rm ghcr.io/krikz/rob_box:led-matrix-humble-latest \
  dpkg -l | grep -E "build-essential|python3-dev|libffi-dev|gcc|g\+\+"

# Время job'а сборки в Actions — взять из истории конкретного run'а
gh run list --workflow "L-Build Vision Pi Services.yml" --limit 5
gh run view <run-id> --json jobs --jq '.jobs[] | select(.name | contains("led-matrix")) | {name, startedAt, completedAt}'

# Время `docker compose pull` на Vision Pi (реальная доставка на робота — то, ради
# чего всё затевается). Через SSH на Vision Pi:
ssh vision-pi "cd /path/to/docker/vision && time docker compose pull led-matrix"
```

### 8.2. После реализации — те же команды на новом теге

Сравнить:
- `docker image ls` size (ожидание: меньше, но не в разы — см. §2.1 и §5.2 «реалистичное ожидание»).
- `docker history` — слой с `build-essential`/`python3-dev`/`libffi-dev` должен **отсутствовать** в
  runtime-стадии (в `docker history` многостадийного образа видна только финальная стадия — это и есть
  прямое доказательство, что слои builder-стадии не попали в итоговый образ).
- `dpkg -l | grep -E "build-essential|python3-dev|libffi-dev|gcc|g\+\+"` внутри контейнера — **НЕ пусто,
  и не может быть пустым**. Исправлено 2026-09-16 по факту реальной сборки пилота: `build-essential`,
  `gcc`, `g++`, `python3-dev` запечены в `ghcr.io/krikz/rob_box_base:ros2-zenoh` (наследство
  `ros:humble-ros-base`) — ровно то, что говорит §2.1 этого же документа. Из трёх пакетов, которые
  `led_matrix/Dockerfile` ставит явно, шов убирает только **`libffi-dev`**. Проверять надо его, а не
  весь список.
- Время job'а в Actions — по этапу 0 (кеширование) ожидается **улучшение** на повторных сборках того же
  коммита; по этапу 1 (multi-stage) первая сборка может быть **медленнее** (два `FROM`, больше слоёв),
  но повторные (с кешем builder-стадии) — не хуже текущих.
- `docker compose pull` на Pi — меньше данных по сети, если образ реально стал легче (значимо только если
  `docker image ls` показал ощутимую разницу, не единицы МБ).

---

## 9. Acceptance (для пилота `led_matrix`, обобщается на следующие этапы)

> **ЗАМЕРЕНО НА ARM64 2026-09-16** (run 35028743625, katana, сборка под qemu).
> Acceptance пилота пройден полностью:
>
> | Проверка | Результат |
> |---|---|
> | Сборка arm64 на self-hosted | зелёная, ~3.5 мин (22:00:56 → 22:04:12) |
> | `spidev` компилируется из исходников под arm64 | `Successfully built spidev` — главный риск пилота снят |
> | `/ws/src` в рантайм-образе | отсутствует |
> | `libffi-dev` в рантайм-образе | отсутствует (0 пакетов) |
> | `import spidev, pi5neo, numpy, led_matrix_driver, led_matrix_compositor` | OK |
> | Размер: было (`led-matrix-humble-dev`, одностадийный) | **829 МБ** |
> | Размер: стало (`led-matrix-humble-test`, многостадийный) | **823 МБ** |
> | Дельта | **~6 МБ** |
>
> Шесть мегабайт — меньше, чем ~13 МБ, оценённых на amd64, и это честный
> масштаб выигрыша для этого файла. Пилот подтверждает **корректность
> механизма**, а не экономию: `led_matrix` выбирался как безопасный первый
> шаг. Деньги — в дедупликации компиляции (`vesc_nexus`/`ros2_control`,
> `perception`/`vision-hailo`) и в облегчённой рантайм-базе, см. §3 и §12.
>
> Побочный результат, к шву не относящийся, но полезный: `led_matrix`
> никогда не объявлял `ARG APT_PROXY`, хотя раннер его передаёт — apt ходил
> в интернет мимо apt-cacher-ng (51.3 МБ индексов за 1m37s). Подключение
> прокси починило падение `Hash Sum mismatch` (run 35027911983) и убрало
> этот трафик. Проверить остальные Dockerfile на ту же дыру — отдельный
> пункт, см. §12.

- [ ] `docker buildx build` по новому Dockerfile завершается успешно на self-hosted `[self-hosted, rob-box]`
      (реальный CI-прогон, не только локальная сборка на amd64-машине разработчика — arm64-специфичные
      проблемы компиляции `spidev` могут не проявиться под эмуляцией).
- [ ] Контейнер стартует, `ros2 node list | grep -q led_matrix` — зелёный (тот же критерий, что уже в
      `HEALTHCHECK`, Dockerfile текущей версии :99-100).
- [ ] `HEALTHCHECK` контейнера — `healthy` в `docker ps` после `start_period` (15s).
- [ ] `docker run --rm <образ> dpkg -l | grep libffi-dev` — **пусто** в рантайм-образе (единственный из
      трёх явно ставящихся пакетов, который шов реально убирает). `build-essential`/`gcc`/`g++`/
      `python3-dev` останутся — они из базового образа, см. §2.1. Прежняя формулировка требовала пустого
      вывода по всему списку и противоречила §2.1; исправлено 2026-09-16 после реальной сборки пилота.
- [ ] `docker run --rm <образ> which colcon` — **не найдено** (colcon — часть унаследованного из базы
      тулчейна и per §2.1 не обязан исчезнуть; если требование «нет colcon» жёстче реального технического
      предела варианта (б) — зафиксировать это явно как известное ограничение пилота, не мнимый provided).
- [ ] `docker run --rm <образ> ls /ws/src` — **не существует** (в рантайм-стадии не должно быть `/ws/src`,
      только `/ws/install`).
- [ ] NeoPixel реально загорается на Vision Pi (ручная/полевая проверка, не автоматизируется в CI — SPI
      требует реального железа).
- [ ] Для `vision-hailo`-этапа дополнительно: сборка **без** прокинутых `/opt/rob_box/vendor`-файлов
      (`HAILO_INSTALL_BINDING=none` явно) собирается успешно и нода стартует в stub-режиме без падения
      (раздел 2.3) — регрессионный тест на сохранение capability-honest поведения.

---

## 10. Риски и откат

**Главный риск**: срезали нужную runtime-зависимость (например, забыли `ros-*-rosidl-default-runtime`,
раздел 2.2) — нода падает на старте с `ImportError`/`ModuleNotFoundError`/ошибкой линковки не в CI, а на
роботе, после того как образ уже задеплоен.

**Как ловить в CI, а не на Pi**:

1. **Смоук-тест контейнера в самом CI job'е**, не только `docker buildx build --load`. Сегодня
   `l-build-service/action.yml` не запускает построенный образ вообще — только собирает. Добавить шаг
   `docker run --rm <образ> <healthcheck-команда>` (для led_matrix: `ros2 node list | grep -q led_matrix`
   после старта) **после** buildx-шага, до `update-image-versions`. Без реального SPI-железа полный
   `ros2 launch` может не подняться — тогда минимум `python3 -c "import led_matrix_driver"` /
   `python3 -c "import rob_box_supervisor_msgs.msg"` (импорт сгенерированных типов — самый частый
   класс поломок из раздела 2.2, специфично ловит отсутствие `rosidl-default-runtime`).
2. **`docker history` diff в CI** — автоматическая проверка, что в финальном образе нет
   `build-essential`/`python3-dev`/colcon-специфичных строк в `dpkg -l` (тот же список, что в acceptance
   §9), как gate перед `update-image-versions`.
3. Прецедент такого класса регрессии уже есть в кодовой базе: issue #2081 (регрессия `quest`:
   `ModuleNotFoundError rob_box_core.avatar_command` из-за устаревшего закешированного `rob_box_core` —
   `docker/vision/quest/Dockerfile:187-198`) и issue #2089 (`ModuleNotFoundError
   rob_box_supervisor_msgs` — `docker/vision/quest/Dockerfile:204-218`). Оба — тот же класс ошибки
   («забыли пересобрать/скопировать нужный кусок между стадиями/слоями»), обе исправлены явным
   документированием, что копировать и пересобирать. Builder/runtime-шов входит в ту же категорию риска
   по своей природе (граница между тем, что «уже должно быть» и тем, что «нужно явно перенести»)
   — это ADR-0018-типичный случай: «честный FAIL в CI лучше красивого PASS», который на самом деле
   молча деградировал до `ImportError` на роботе.

**Откат**: каждый этап (раздел 7) — отдельный PR с одним изменённым Dockerfile. Откат — `git revert`
конкретного PR, пересборка предыдущего тега через `docker compose pull` с зафиксированной версией в
`docker/vision/.image-versions.*env*` (или аналог для main). Поскольку `pull_policy: if_not_present`
уже используется в `docker/vision/docker-compose.yaml:72` (и аналогично, судя по недавней истории
коммитов, добавлено в другие сервисы) — откат на Pi не требует принудительного `docker pull`, если тег
не совпадает с новым.

---

## 11. Связь с ADR

- **ADR-0100** (`docs/adr/0100-aiohttp-apt-vs-pip-voice-assistant.md`) — формально о другом (apt vs pip
  версия `aiohttp`), но фиксирует **тот же принцип**, что раздел 3 (пункт 10) и раздел 10 этого документа
  применяют к `voice_base`: цена правки `voice_base` — инвалидация ~10 GB кеша для всех потребителей
  (§3.2, цитата коммита `337548a38`: «Kept off voice_base on purpose…», ADR-0100:34). Прямого конфликта
  нет — ADR-0100 не запрещает трогать `voice_base` вообще, он документирует **почему конкретно aiohttp**
  туда решили не переносить. Этот план применяет ту же логику по аналогии к тулчейну `voice_base`
  (раздел 1.2, раздел 3 пункт 10) — не как формальное требование ADR-0100, а как перенос уже принятого
  архитектурного принципа. Совместимо.
- **ADR-0099** (`docs/adr/0099-vision-hailo-binding-install-strategy.md`) — определяет capability-honest
  контракт для `vision-hailo` (§2.1: `hailo_platform` опциональный, build-arg `HAILO_INSTALL_BINDING`).
  Раздел 2.3 этого документа прямо основан на §2.2 ADR-0099 и обязывает сохранить это поведение при
  разделении на стадии (acceptance §9, последний пункт). Совместимо, без конфликтов — этот план **не
  предлагает** менять сам контракт `HAILO_INSTALL_BINDING`, только его физическое размещение по стадиям.
- **ADR-0089** (`docs/adr/0089-ai-hat-plus-deployment.md`) — §4 и §7 (R1/R2) описывают
  `hailort-pcie-driver` как хостовый kernel-модуль и `/dev/hailo0` как проброс устройства. Раздел 2.3
  этого документа использует этот факт, чтобы явно вывести `hailort-pcie-driver` за границы сборочного
  шва. Совместимо.
- **ADR-0018** (`docs/adr/0018-agent-honesty-culture.md`) — «честный FAIL лучше красивого PASS».
  Раздел 10 этого документа (риски и откат) прямо следует этому принципу: явно требует смоук-теста
  контейнера **в CI**, а не полагаться на «образ собрался = всё работает». Формулировки честности по
  разделам 1 и 3 («выигрыш копеечный», «на уровне погрешности») — тоже следствие этого принципа.
  Совместимо, без конфликтов.
- **AF-0013** (`docs/adr/AF-0013-incremental-delivery-over-big-bang.md`) — правило размера PR (≤50
  коммитов / ≤3000 строк) и возраста ветки (≤7 дней). Раздел 7 (пошаговый план) построен как серия узких,
  независимо мержимых PR именно из-за этого правила — ни один этап не пытается переписать больше одного
  Dockerfile за раз (кроме этапов 3-4, где два файла осознанно объединены **общим builder-образом**, но
  даже там предложено разбить на два PR: сначала builder-образ + меньший потребитель, потом второй
  потребитель отдельным PR). Совместимо.

**Нужен ли новый ADR**: да, рекомендуется — после пилота (этап 1, раздел 7), не до него. Причина:
ADR должен фиксировать **проверенное** архитектурное решение (что реально сработало на практике —
раздел 2.2 содержит непроверенные в этом репозитории утверждения про `rosidl-default-runtime`, которые
пилот либо подтвердит, либо опровергнет). Предлагаемое содержание будущего ADR:
- Контракт интерфейса шва для colcon/ROS2 (раздел 2.2 — что обязано копироваться, что нет), подтверждённый
  реальной сборкой.
- Решение по разделу 4.3 — локальная builder-стадия по умолчанию, общий builder-образ только для пар с
  дублированием компиляции.
- Explicit список файлов, которые сознательно **не** переводятся на builder/runtime (раздел 1.1 и раздел
  3, пункты 8-10) — чтобы следующий архитектурный ревьюер не открывал этот вопрос заново без нового факта.

---

## 12. Открытые вопросы

1. **Облегчённая рантайм-база.** Раздел 2.1 показал, что полное устранение тулчейна из рантайма требует
   отдельного, более лёгкого базового образа (`rob_box_base:ros2-zenoh-runtime` без `build-essential`/
   `colcon`/`rosdep`/`git`/`vcstool`) — это отдельная, более крупная инициатива, которая меняет
   `docker/base/Dockerfile.ros2-zenoh` (и, возможно, требует parallel-варианта, а не замены — многие
   образы, включая builder-стадии, продолжат нуждаться в текущей «толстой» базе). Делать ли это отдельным
   планом после накопления опыта пилота, или включить в roadmap этого плана как Phase 2? Не решено здесь.
2. **`introlab3it/rtabmap_ros:humble-latest`** (внешний образ, база для `rob_box_base:rtabmap`,
   `docker/base/Dockerfile.rtabmap:3`) — его внутренний состав (есть ли там build-essential/colcon
   аналогично `ros:humble-ros-base`) не проверен в рамках этого документа (сторонний Docker Hub образ,
   не проверялся ни чтением исходного Dockerfile, ни `docker history` — в отличие от `voice-assistant`,
   этот образ не был локально закеширован на момент подготовки документа). Учитывая, что `ros2_control`
   и `vesc_nexus`(частично, через свой собственный) опираются на эту базу — стоит подтвердить перед
   этапом 3 раздела 7.
3. **`docker/vision/voice_resources/Dockerfile`** — не является сборочным швом в смысле этого документа
   (не компилирует код, качает Renardo sample packs), но структурно похож (init-контейнер, отдающий
   готовые данные). Не идентифицировано, входит ли эта проблема в скоуп параллельного плана
   `docs/plans/2026-09-15-resource-pack.md` или это отдельная, третья задача — стоит свериться при
   реализации.
4. **Общий builder-образ для vesc_nexus/ros2_control и perception/vision-hailo** (раздел 4.3) — не
   проработан на уровне конкретного Dockerfile/тега в этом документе (только направление). Требует
   отдельного маленького дизайн-решения на этапе 3/4 (раздел 7): один тег на пару, или переиспользование
   расширенного `docker/ci/Dockerfile`-подобного образа?
5. **Числа `docker history` из раздела 1.2** получены на локально закешированном образе (4 недели
   давности на момент написания). Требуют переизмерения непосредственно перед реализацией, если пройдёт
   значительное время — состав `voice_assistant`/`voice_base` мог измениться.
6. **`docker/ci/Dockerfile` как основа общего builder-образа** (раздел 4.1, 4.3) — сейчас это ROS 2 +
   тестовые пакеты (flake8-плагины, pytest-расширения), не заточенные под сборку рантайм-сервисов
   (нет `libboost-dev`, нет специфичных для `led_matrix`/`vesc_nexus` пакетов). Использовать его
   напрямую как `BUILDER_IMAGE` не получится без изменений — вопрос, стоит ли развивать `ci/Dockerfile`
   в двух направлениях (тесты + рантайм-сборка) или заводить отдельный образ, не решён здесь.

---

## Предложение термина для `CONTEXT.md`

В `CONTEXT.md` сегодня нет секции про сборку образов (проверено: `grep -n "^##" CONTEXT.md` — секции
«Агенты», «Голос», «Инструменты», «Движок», «Блокировки», «Восприятие», «Интерфейсы»; ни одна не про
Docker/CI). Предлагается добавить новую секцию `## Сборка образов` с термином:

> **Сборочный шов** — граница `COPY --from=<builder>` между тяжёлой стадией сборки (тулчейн: компилятор,
> `colcon`, заголовочные `-dev`-пакеты, сетевые зависимости вида `git clone`) и тонкой стадией запуска
> (только то, что нужно ноде в рантайме: скомпилированные артефакты `/ws/install`, runtime-библиотеки
> `rosidl-default-runtime`, разделяемые `.so`). Для ROS2/colcon-пакетов интерфейс шва **не** глубокий
> сам по себе — требует явного знания, какие ROS-пакеты являются build-time-инструментом (`rosidl-default-generators`,
> `ament-cmake`) и какие обязательны в рантайме (`rosidl-default-runtime`), см.
> `docs/plans/2026-09-15-builder-runtime-seam.md` §2. Адаптер №1 этого шва в репозитории —
> `docker/vision/quest/Dockerfile` (стадии `webxr-builder` и `caddy-builder`), но он закрывает только
> внешние (не-ROS) инструменты; свой `colcon build` quest **не** выносит за шов (собственный ROS-пакет
> собирается прямо в рантайм-стадии).
> _Не путать_ с базовым образом (`rob_box_base:*`) — тот общий для builder- и runtime-стадий одного
> и того же сервиса и сам по себе не является швом.

---

## Неподтвердившиеся / уточнённые факты из задания

- **«Тулчейн ставится и не удаляется в `docker/ci/Dockerfile`»** — уточнено: тулчейн там **есть и
  намеренно не удаляется**, но `docker/ci/Dockerfile` **не запускает `colcon build` сам** (это CI-раннер-образ,
  потребители гоняют сборку внутри него сами). Формально факт верен (тулчейн стоит), но контекст
  («ставится ради чего») в задании не был уточнён — это не сборка сервиса, это заготовленный инструмент
  для чужой сборки. См. раздел 1.1.
- **«Тулчейн ставится... в `docker/main/{lslidar,perception,ros2_control,teleop,vesc_nexus}/Dockerfile`»**
  — частично уточнено: `lslidar`, `perception`, `teleop` **не устанавливают** `build-essential`/`cmake`/
  `python3-dev` явным `RUN apt-get install` в своём файле — этот тулчейн приходит из базового образа
  (`ros:humble-ros-base`, см. раздел 2.1). Явно свой тулчейн в файле ставят только `vesc_nexus`
  (`ros-dev-tools`, :22) и, частично, `ros2_control` (только rosidl-generators, не полноценный
  build-essential/cmake — те наследуются). См. таблицу раздела 1.2.
- **«Тулчейн ставится... в `docker/vision/{led_matrix,quest,supervisor,vision-hailo,voice_assistant,voice_base}/Dockerfile`»**
  — уточнено по каждому: `led_matrix` и `voice_base` — да, явно (подтверждено, файл:строка в разделе 1.2).
  `quest`, `supervisor`, `vision-hailo`, `voice_assistant` — **не устанавливают** свой build-essential/
  python3-dev/cmake явно; их тулчейн — унаследованный от базы (`quest`, `vision-hailo`) либо от
  FROM-цепочки `voice-assistant`/`voice-base` (`supervisor`, `voice_assistant`). Разница важна для
  раздела 3 (порядок кандидатов) — «явно свой тулчейн» и «унаследованный тулчейн» дают разный реальный
  выигрыш от локальной builder-стадии (раздел 2.1).
- Прочие established facts задания (многостадийность `quest` и три его стадии с точными номерами строк,
  список файлов с `COPY src/...` в рантайм, `git clone` в `voice_base`, комментарий про ~10 GB на строке
  `voice_assistant/Dockerfile:123`, отсутствие `--cache-from` в `l-build-service/action.yml`, наличие
  `--cache-from type=registry` только в `L-Build Base Images.yml:76,116,156,197`, единственный `type=gha`
  в `B-Build CI Image.yml:62-63`, self-hosted `[self-hosted, rob-box]`, `linux/arm64`) — **все
  подтверждены** прямым чтением файлов, номера строк совпали с точностью до строки.

---

## 13. Ретро (Этап 2 плана) — что шов дал на самом деле, 21.09.2026

Этап 2 требовал «снять урок с пилота и скорректировать шаблон». Уроков
набралось на шесть файлов (led_matrix, vesc_nexus, ros2_control, lslidar,
perception, vision-hailo), и главный из них — про ожидания.

### 13.1. Экономия размера оказалась на порядок меньше обещанной

| файл | ожидание плана | факт |
|---|---|---|
| led_matrix (пилот) | ~13 МБ (оценка на amd64) | **~6 МБ** (arm64, run 35028743625) |
| perception | «средний выигрыш» за счёт rosidl | **~9 МБ** (842 → 833 МБ, amd64-проба) |
| vision-hailo | «средний выигрыш» | тот же порядок |
| ros2_control | «абсолютная экономия сопоставима с vesc_nexus» | **≈ 0** |
| vesc_nexus | самый крупный кандидат | ~55 MiB (`ros-dev-tools`, 59 пакетов) |
| lslidar | — | Boost-заголовки + клон драйвера |

Причины, каждая подтверждена измерением, а не рассуждением:

1. **§2.1 сильнее, чем казалось.** `dpkg -l` внутри `ros:humble-ros-base`:
   `ament-cmake`, `ament-cmake-python`, `rosidl-default-generators`,
   `rosidl-default-runtime`, `nav-msgs` уже стоят в базе. Для
   perception/vision-hailo шов **не убирает с уровня apt ничего** — §1.2 и §3
   п.4-5 оценивали выигрыш неверно.
2. **Данные пересекают шов в обе стороны.** `rob_box_description/models` — 59 МБ,
   и `install(DIRECTORY models ...)` тащит их в `share/` при любом раскладе.
   Сегодня `--symlink-install` держал их симлинками на `/ws/src` (байты лежали
   один раз), после шва это настоящие копии в `/ws/install`, зато `/ws/src`
   уходит. Нетто по ros2_control ≈ ноль.
3. Реально шов убирает `/ws/src`, `/ws/build`, `/ws/log` и промежуточные
   артефакты pip — единицы мегабайт, если сервис не ставит своего тулчейна.

**Вывод для следующих этапов:** мерить выигрыш надо не «сколько весит
тулчейн», а «сколько этот Dockerfile ставит СВЕРХ базы». Там, где ответ
«ничего», шов даёт корректность и изоляцию, но не байты, и продавать его
как экономию нельзя.

### 13.2. `--symlink-install` ломает шов ТИХО — и это главный риск, а не размер

Проверено экспериментом (сборка с флагом и без, amd64, настоящий
`rob_box_description`):

```
--- битые symlink в /ws/install (--symlink-install): ---
/ws/install/rob_box_description/share/ament_index/resource_index/packages/rob_box_description
/ws/install/rob_box_description/share/rob_box_description/models/.../whiteboard.jpg
--- source /ws/install/setup.bash ---
not found: ".../local_setup.bash"
source вернул 0                 <-- падения НЕТ
--- ros2 pkg prefix rob_box_description ---
Package not found
```

В perception таких симлинков наружу **91**. То есть «образ собрался» и даже
«контейнер запустился» этот класс регрессии не ловят: `source setup.bash`
возвращает 0, падает уже нода. §9 (acceptance) обязан требовать
`ros2 pkg prefix` / импорт — и теперь это не формальность, а единственная
проверка, которая работает.

### 13.3. Поправка, которую нашёл только реальный CI: `apt-get update` перед rosdep

Run 35617451547, `build-ros2-control`:

```
E: Unable to locate package ros-humble-xacro
ERROR: the following rosdeps failed to install
```

Apt-блок builder-стадии заканчивается `rm -rf /var/lib/apt/lists/*`, а
`rosdep install` внутри вызывает `apt-get install`. До шва этого не было
видно: `xacro` ставился явным apt-блоком того же образа, и rosdep'у нечего
было доустанавливать; шов увёл его в рантайм-стадию. В четырёх других
файлах (включая смерженный пилот) rosdep стоит с `|| true` — там эта же
дыра молча превращала его в no-op. Починено везде.

**В шаблон §5 добавлено обязательное:** `apt-get update &&` первой строкой
RUN с `rosdep`.

### 13.4. Что ещё вылезло

- **APT_PROXY отсутствовал во ВСЕХ пяти файлах**, хотя раннер его передаёт
  (та же дыра, что чинил `dc7ed68a2` в пилоте). build-arg молча
  игнорировался, apt ходил напрямую в packages.ros.org.
- **Зависимости, приходившие транзитивно через `rosdep`, через шов не
  проходят** и требуют явного объявления в рантайм-стадии:
  `rclcpp-lifecycle`, `std-msgs`, `robot-state-publisher`, `nav-msgs`,
  `libboost-thread1.74.0`.
- **pip-layout:** у ament_cmake+rosidl пакетов
  `/ws/install/<pkg>/local/lib/python3.10/dist-packages`, у ament_python —
  `/ws/install/<pkg>/lib/python3.10/site-packages`. Оба резолвятся через
  `setup.bash`; в `/opt/ros`, `/usr/lib/python3/dist-packages` и `/usr/local`
  не осело ничего.
- **`docker/main/vesc_nexus/Dockerfile` не собирается ни одним workflow** и
  отсутствует в `docker/build-manifest.yaml` — половину Этапа 3 CI не
  проверит в принципе. Судьбу этого образа надо решить отдельно.

### 13.5. Статус этапов

| Этап | Статус |
|---|---|
| 0 — кеширование | Сделан. `--cache-from` работает, `--cache-to` на katana пропускается: драйвер buildx `docker` не умеет экспорт кеша. Нужен `docker-container` на раннере — решение владельца стенда, не кода. |
| 1 — пилот led_matrix | Сделан и смержен ранее. |
| 2 — ретро | Этот раздел. |
| 3 — vesc_nexus + ros2_control | Локальные builder-стадии сделаны. Общий builder-образ для пары — НЕ сделан. |
| 4 — perception + vision-hailo | Локальные builder-стадии сделаны. Общий builder — НЕ сделан. |
| 5 — lslidar | Сделан, сборка на katana зелёная. |
| 6 — teleop, telegram_bot | Не делался (низкий приоритет, §3 п.7). |

**Про общий builder-образ (§4.3) — аргумент сменился.** Раз локальный шов для
пар даёт ~9 МБ и ноль на уровне apt, общий builder надо оценивать как
**экономию CI-времени** (повторная компиляция rosidl — минуты под qemu), а не
размера образа: размер он не изменит вовсе. Осложнение: `perception` и
`vision-hailo` собираются в РАЗНЫХ workflow, значит это отдельный job + тег +
`--cache-from` поверх Этапа 0.

---

## 14. Этап 0 доведён до конца, 21.09.2026 — и это заняло три прогона

§13.5 писался, когда Этап 0 был «сделан, но кеш на стенде не пишется». Теперь
пишется. Путь туда стоит записать целиком: два из трёх препятствий нельзя было
найти чтением.

### 14.1. Драйвер нельзя переключить «на стенде»

Раннеры на katana — это **контейнеры** (`myoung34/github-runner`, восемь штук,
с проброшенным `/var/run/docker.sock`). У каждого свой `~/.docker`, поэтому:

- `docker buildx create`, сделанный по ssh на хосте, сборкам **не виден**
  (проверено: `docker buildx ls` на хосте показывает только `default`,
  а в job'е драйвер был свой);
- сделанный внутри контейнера — умирает вместе с контейнером при
  пересоздании раннера.

Единственное место, которое переживает и то, и другое, — шаг в самом
composite action. Там билдер и заводится: `docker-container`,
`--driver-opt network=host`, `buildkitd.toml` с `http=true` для локального
registry без TLS.

Факт, который выяснился по логу: hostname у всех восьми раннер-контейнеров
**одинаковый** (`ros2-Katana-GF66-11UD`), поэтому билдер получается ОБЩИЙ, а
не по одному на раннер. Для кеша это лучше; гонки при одновременном создании
восемью job'ами не случилось — `create` обёрнут в `|| echo warning`, и
остальные просто взяли готовый. Все 7 завершившихся job'ов получили
`(from + to)`.

### 14.2. `host-gateway` — фича демона, а не buildx

Первый прогон с новым билдером уронил **все восемь** job'ов за две минуты:

```
ERROR: unable to derive the IP value for host-gateway:
       host-gateway is not supported by the docker-container driver
```

`--add-host=host.docker.internal:host-gateway` работал ровно потому, что
сборка шла на драйвере `docker`. На `docker-container` его надо заменять
реальным IP шлюза bridge-сети (`docker network inspect bridge`). После замены
apt-прокси остался достижим: `OK: APT proxy http://host.docker.internal:3142`
в builder-стадии lslidar.

### 14.3. Как выглядит работающий кеш в логе

```
add-host: host-gateway → 172.17.0.1 (драйвер docker-container не умеет host-gateway)
Cache: localhost:5000/krikz/rob_box:twist-mux-buildcache (from + to, mode=max, ignore-error, driver=docker-container)
#4 importing cache manifest from localhost:5000/krikz/rob_box:twist-mux-buildcache
#10 exporting cache to registry
```

До этого в каждом job'е стояло `(from only)` — то есть Этап 0 с момента
реализации и до 21.09 **не писал кеш вообще**, только безопасно читал пустой
ref. Если бы никто не пошёл смотреть в лог, «кеширование сделано» жило бы
как факт.

### 14.4. Цена, о которой надо помнить

- Переключение драйвера обнуляет видимость старого кеша демона: первые сборки
  после него холодные. На katana это стоило ~13 ГБ диска и десятков минут
  (холодный `build-nav2` — 20 минут, `build-ros2-control` — больше 38).
- Диск на katana хронически полон (97% и до, и после чистки). Главный едок —
  не кеш сборки, а сам локальный registry: **62.5 ГБ** в `/var/lib/registry`,
  потому что каждый SHA-тег хранится вечно и сборки мусора там нет. Это
  отдельная задача, кеш её только подсвечивает.

---

## 15. Ловушка, которую шов не создал, но удвоил: apt + конвейер под qemu

Хронологически это случилось сразу после §14 и стоило дороже всего — 68 минут
висящей сборки и один отменённый прогон.

### 15.1. Симптом

`build-ros2-control`, рантайм-стадия, `apt-get install` семнадцати
ROS-пакетов. Лог не двигался. Выглядело как «медленно под эмуляцией» —
оказалось дедлоком:

```
PID      ELAPSED   TIME      COMMAND
3822692  4075      00:00:00  apt-get                      ← 68 минут, НОЛЬ секунд CPU
3826151  4151      00:00:04  qemu-binfmt/aarch64 .../apt/methods/store

ss -tnp:
CLOSE-WAIT  Recv-Q=1  172.17.0.1:42884 → 172.17.0.1:3142
CLOSE-WAIT  Recv-Q=1  172.17.0.1:42888 → 172.17.0.1:3142
```

Ноль секунд CPU за 68 минут — процесс не работает, а ждёт. Два сокета к
apt-cacher-ng в `CLOSE-WAIT` с непрочитанным байтом. При этом сам хост
здоров: с katana `packages.ros.org` отдаёт 200 за 0.96 с, через прокси —
за 0.37 с.

**Проверять надо именно `TIME`, а не `ELAPSED`.** «Долго висит» и «долго
работает» под qemu выглядят одинаково, различает их только накопленное
процессорное время.

### 15.2. Причина и починка

HTTP-конвейер: и apt, и apt-cacher-ng по умолчанию держат `Pipeline-Depth 10`,
а метод apt под qemu-user на закрытии соединения застревает. Слова
`Pipeline-Depth` в репозитории не было ни в одном из 11 мест, где пишется
`/etc/apt/apt.conf.d/02proxy`.

`Acquire::http::Pipeline-Depth "0"` добавлен во все 13 стадий семи сервисных
Dockerfile'ов. Базовые образы (`docker/base/Dockerfile.*`) в тот заход
сознательно НЕ тронуты: ловушка там та же, но правка инвалидирует кеш всех
потребителей (voice_base тянет за собой ~10 ГБ пересборки) — отдельной
карточкой.

Карточка закрыта следом: та же строка добавлена в блок APT_PROXY четырёх
базовых образов — `Dockerfile.ros2-zenoh`, `Dockerfile.rtabmap`,
`Dockerfile.depthai`, `Dockerfile.pcl`. Теперь `Pipeline-Depth "0"` стоит во
всех местах, где пишется `02proxy`, кроме `docker/build/test/Dockerfile` — он
ни на один workflow не завязан и под qemu не собирается.

### 15.3. Результат

Тот же шаг, та же машина, та же эмуляция:

```
#10 [stage-1 3/6] RUN apt-get update && apt-get install -y can-utils ...
#10 DONE 124.7s
```

124.7 секунды вместо 68 минут без конца. Весь прогон
(run 35635018766) зелёный, включая `update-image-versions`.

### 15.4. Вклад шва

Шов эту ловушку не создал — она ждала в репозитории с тех пор, как появился
блок APT_PROXY. Но он **удвоил шанс в неё попасть**: раньше на образ
приходился один `apt-get install`, после разделения — два. Это честная
строка в цену сборочного шва, наравне с §13.1 (экономия оказалась меньше
обещанной).
