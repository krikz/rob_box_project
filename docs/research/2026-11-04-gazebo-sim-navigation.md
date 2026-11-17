# Gazebo-симуляция РОББОКСА для подбора навигации — разведка и план

> Дата: 2026-11-04. Статус: разведка (этап 1), код не писался.
> Задача товарища Шифу: поднять систему в Gazebo, покатать робота в виртуальном мире, подобрать настройки навигации.
> Целевая машина (слово Шифу 04.11): песочница `ros2@10.1.1.250`.
>
> Метки достоверности: **[код]** — прочитано в репо (файл:строка, develop `e5b524929`);
> **[измерено]** — снято командой в этой разведке; **[док]** — официальная документация (ссылка);
> **[догадка]** — вывод без проверки.

## 1. Коротко

- В репо есть заготовки под Gazebo, но **рабочей симуляции нет**. `rob_box.xacro` годится как основа
  (inertial и collision есть), но в симуляторе его не запустить: в нём только железный плагин VESC,
  нет плагина колёс, нет IMU, лидар описан на ±80° вместо 360°.
- Рекомендация: **Gazebo Fortress + `ros_gz` + `gz_ros2_control`** — это официальная пара для Humble,
  ставится бинарями из репозитория ROS. Колёсами в симе управляет **тот же** `diff_drive_controller`
  с тем же `controller_manager.yaml`, что и на роботе.
- Где запускать: песочница 10.1.1.250 (i7-11800H, 16 потоков, 31 ГБ ОЗУ, 763 ГБ свободно, RTX 3050 Ti).
  Gazebo без окна (headless) + Foxglove/RViz по сети. Отдельный amd64-контейнер «sim»,
  конфиги навигации берутся из `docker/main/config/**` как есть.
- По ходу чтения кода нашлись три вещи в настоящем стеке навигации, их стоит проверить в симе в первую очередь (§2.4).

## 2. Что уже есть в репо

### 2.1 Описание робота (`src/rob_box_description`)

| Что | Где | Состояние для Gazebo |
|---|---|---|
| Главный xacro | `src/rob_box_description/urdf/rob_box.xacro:1-366` | основа годится |
| Корень TF | `base_footprint` → `base_link` на высоте `wheel_radius`, `rob_box.xacro:26-71` | ок (REP-120) |
| Шасси: inertial + collision | `rob_box.xacro:39-61`: масса 62.65 кг, collision = меш `base_link.stl` | collision мешем — тяжело для физики [догадка], лучше box |
| Колёса | макрос `rob_box.xacro:75-109`, 4 шт. `continuous`, оси ±Y, `rob_box.xacro:324-344` | collision = STL-меш (`rob_box.xacro:95-101`); для контакта колесо–пол нужен цилиндр [догадка, обычная практика]. **Масса колеса 21.68 кг** (`rob_box.xacro:10`), итого по URDF ≈ 62.65 + 4×21.68 ≈ **149 кг**, а в комментарии контроллера «62 кг скид-стир» (`docker/main/config/controllers/controller_manager.yaml:116`). Нужна реальная масса |
| Радиус колеса | 0.1143 м, `rob_box.xacro:5`, замер 23.02 | ок |
| Колея геометрическая | ±0.195 м → 0.39 м, `rob_box.xacro:325,343` | — |
| Лидар LSLIDAR N10 | `rob_box.xacro:112-173`, вызов `:348` (frame `lslidar_n10`, топик `/scan`) | `gpu_lidar`, **640 точек на ±80°** (`:148-151`), 10 Гц (`:144`), 0.08–10 м (`:161-162`). У настоящего драйвера угол не обрезается (`angle_disable_min/max: 0.0`, `docker/main/config/lslidar/lsx10_custom.yaml:14-15`), диапазон 0.15–10 м (`:16-17`) → в симе нужно 360° и min 0.15. Плагин `ignition::gazebo::systems::Sensors` стоит внутри `<gazebo reference>` ссылки (`:169-171`); в Fortress это системный плагин мира [догадка, проверить на шаге 1] |
| Yaw лидара | сейчас `0 0 0` (`rob_box.xacro:348`); в `docs/fixes/LIDAR_180_ROTATION_FIX_2025-11-20.md:102` предлагался `0 0 3.14159` | расхождение; в симе это неважно, но живой `/scan` надо сверить (без робота не проверено) |
| OAK-D | только фреймы, `rob_box.xacro:176-273`; collision мешем | сенсора камеры нет. Для Nav2 не нужна: rtabmap OAK-D не использует (`docker/main/docker-compose.yaml:143-144`), local costmap берёт только `/scan` (`docker/main/config/nav2/nav2_params.yaml:155-157`) |
| IMU | только фрейм `camera_imu_frame` (`rob_box.xacro:267-272`) | rtabmap ждёт `/camera/imu/data` (`docker-compose.yaml:162`, `wait_imu_to_init:=false` `:163`) → в симе добавить `imu`-сенсор |
| ros2_control | `src/rob_box_description/urdf/rob_box_ros2_control.xacro:14-115`, плагин `vesc_nexus/VescSystemHardwareInterface` (`:16`), CAN `can0`, `control_mode=duty` (`:40`) | для сима нужен `xacro:arg sim` → `gz_ros2_control/GazeboSimSystem` + плагин `gz_ros2_control-system` |
| Старый экспорт | `src/rob_box_description/URDF_EXPORT/` (ROS1, `gazebo.launch` на `gazebo_ros spawn_model`, `URDF_EXPORT/launch/gazebo.launch:1-11`), есть `COLCON_IGNORE` | мусор, не использовать |
| Миры | `worlds/office_cpr.world` (SDF 1.6, ODE, меш `model://cpr_office/meshes/office.dae`, `office_cpr.world:87`), `worlds/coridor.world` (188 строк, свой коридор, коммит `3a1e6f359`) | можно открыть в Fortress после правки плагинов [догадка]. Офис 59 МБ (`models/`), `model.config` автора называет, «office from clearpath robotics», но лицензии не указывает (`models/cpr_office/model.config:1-11`) — уточнить |
| Пакет | `src/rob_box_description/package.xml:13-17` — зависимостей `ros_gz*` нет; `CMakeLists.txt` ставит `meshes models rviz urdf worlds` | — |
| Bringup | `src/rob_box_bringup/launch/complete_system.launch.py:10` — аргумент `use_sim_time` есть; `rob_box_control.launch.py:98-146` — controller_manager/RSP/rviz | в продакшене не используется: контейнеры запускают свои скрипты |

### 2.2 Привод

- Скид-стир 4×4, управляется как дифф: `diff_drive_controller`, 2 колеса на борт
  (`docker/main/config/controllers/controller_manager.yaml:46-72`). Именно этот файл подаётся контейнеру
  (`docker-compose.yaml:213`, `CONTROLLER_CONFIG=/config/controllers/controller_manager.yaml`).
  `docker/main/config/vesc_nexus/robot_controller.yaml` с `wheel_separation: 0.380` (`:50`) — **не тот**,
  что запускается.
- **Эффективная колея 1.11 м** при геометрической 0.39 м — калибровка поворотом на 360°
  (`controller_manager.yaml:60-66`). **`open_loop: true`** — одометрия считается из команд, а не из энкодеров
  (`controller_manager.yaml:86-88`). TF `odom → base_footprint` (`:80-84`).
- Железо: VESC по CAN, duty-режим, `gear_ratio 2.17`, `min_duty 0.05`
  (`rob_box_ros2_control.xacro:29,34,40`). Открытая находка #4004 (по данным issue): на 1 м/с колёса крутятся
  на ~18–22 % медленнее команды, `exp 8.74` → `meas ≈ 6.8–7.2 рад/с`.

### 2.3 Стек навигации (Main Pi, `docker/main/docker-compose.yaml`)

| Узел | Где | Нужен в симе |
|---|---|---|
| `zenoh-router` | `docker-compose.yaml:15-18` | по выбору RMW (§5.2) |
| `robot-state-publisher` | `:73-93`, `use_sim_time: false` зашит (`docker/main/scripts/robot_state_publisher/start_robot_state_publisher.sh:28`) | да |
| `ros2-control` | `:193-217`; скрипт сначала проверяет CAN (`docker/main/scripts/ros2_control/start_ros2_control.sh:58-73`) | **нет** — заменяется плагином `gz_ros2_control` |
| `lslidar` | `:233-256` | **нет** — `/scan` даёт Gazebo |
| `twist-mux` | `:40-59`, вход Nav2 = `cmd_vel` priority 10 (`docker/main/config/twist_mux/twist_mux.yaml:45-48`), выход `/diff_drive_controller/cmd_vel_unstamped` (`docker/main/scripts/twist_mux/start_twist_mux.sh:72`), `use_sim_time:=false` (`:73`) | да (те же приоритеты и блокировки) |
| `rtabmap` | `:100-187`: ICP-одометрия по лидару (`:153`), guess от `odom` (`:154`), `frame_id base_footprint` (`:156`), база `/maps/rtabmap.db` (`:175`), **всегда стартует в localization** (`docker/main/scripts/rtabmap/start_rtabmap.sh:6-22`), `use_sim_time:=false` (`docker-compose.yaml:178`) | да; в симе сначала режим картирования |
| `nav2` | `:321-341`, `start_nav2_direct.sh` поднимает узлы по одному (`docker/main/scripts/nav2/start_nav2_direct.sh:35-89`), параметры `docker/main/config/nav2/nav2_params.yaml` | да |
| `perception`, `teleop`, мониторинг | `:264-306`, `:391-413`, профиль `monitoring` | нет (teleop — по желанию) |
| Vision Pi: OAK-D, Hailo, лица, LED, ollama | `docker/vision/docker-compose.yaml:47-364` | нет |
| Голос, звук, музыка (Sound Pi) | `docker/sound/` | нет |

`use_sim_time: false` зашит в **каждой** секции `nav2_params.yaml` (`:7,66,131,180,216,226,252,260,271,286`)
и в обоих `*_costmap_params.yaml` (`global_costmap_params.yaml:11`, `local_costmap_params.yaml:11`).
Для сима нужен один переключатель (переменная окружения → override-файл параметров).
Копию `nav2_params.yaml` под сим заводить нельзя: правило «не держать две реализации» и смысл задачи —
крутить **тот же** файл.

### 2.4 Особенности текущей Nav2-конфигурации, которые сим проверит первым делом [код]

1. **velocity_smoother, похоже, работает вхолостую.** В Humble `navigation_launch.py` переназначает
   `controller_server: cmd_vel → cmd_vel_nav` и `velocity_smoother: cmd_vel_nav → cmd_vel`
   ([navigation_launch.py, ветка humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/navigation_launch.py)) [док].
   В `start_nav2_direct.sh:43-89` таких переназначений нет. Значит, DWB публикует прямо в `/cmd_vel`
   (twist_mux), а smoother подписан на тот же `/cmd_vel` и выдаёт `/cmd_vel_smoothed`, который никто не слушает.
   Лимиты smoother-а (`nav2_params.yaml:269-282`) на робота, скорее всего, не действуют. Это вывод по коду:
   на роботе не проверено, в симе проверяется одной командой `ros2 topic info /cmd_vel_smoothed -v`.
2. **Глобальная карта не берёт карту rtabmap.** В `global_costmap` нет `static_layer`, есть только
   `obstacle_layer` из `/scan` и окно 50×50 м, которое едет за роботом (`nav2_params.yaml:187-192`).
   Планировщик видит только то, что лидар заметил за эту поездку. Сознательное ли это решение —
   вопрос к Шифу (§8).
3. **Одометрия без обратной связи (`open_loop: true`) при скорости колёс ниже команды на 20 % (#4004).**
   ICP-одометрия rtabmap это исправляет, но её задержку видно по множеству «Increased … tolerate ICP latency»
   (`nav2_params.yaml:67,107,251`). В симе можно отдельно проверить два режима: с честной одометрией колёс
   и с одометрией, искажённой как на роботе.

Контроллер — DWB (`nav2_params.yaml:85`), `max_vel_x 0.15` (`:89`), `robot_radius 0.45` (`:140`),
`inflation 0.55` (`:145`), планировщик NavFn A* (`:219-221`). В образ уже поставлены альтернативы
(`docker/main/nav2/Dockerfile:29-45`: `nav2-smac-planner`, `nav2-regulated-pure-pursuit-controller`).

## 3. Выбор симулятора [док]

| Вариант | Статус для Humble | EOL | Вывод |
|---|---|---|---|
| Gazebo Classic 11 | бинари есть, на нём `tb3_simulation_launch.py` в Nav2 Humble ([исходник](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/tb3_simulation_launch.py): `gzserver`, `spawn_entity.py`) | **29.01.2025** ([classic.gazebosim.org](https://classic.gazebosim.org/)) | не брать: мёртв уже почти два года |
| **Gazebo Fortress** (gz-sim 6) | «рекомендовано» для Humble, `apt install ros-humble-ros-gz` ([ros_installation](https://gazebosim.org/docs/latest/ros_installation/)) | май 2027 ([releases](https://gazebosim.org/docs/latest/releases/)) | **брать** |
| Gazebo Harmonic | «можно с усилием», пакеты `ros-humble-ros-gzharmonic` не официальные и конфликтуют с `ros-humble-ros-gz*` ([ros_installation](https://gazebosim.org/docs/latest/ros_installation/)); `gz_ros2_control` для Humble+Harmonic — только сборка из исходников ([gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)) | май 2029 | запасной путь при переходе на Jazzy |

`gz_ros2_control` для Humble+Fortress ставится бинарём `ros-humble-gz-ros2-control` ([README](https://github.com/ros-controls/gz_ros2_control)) [док].
С ним в симе работает **тот же** `diff_drive_controller` с тем же yaml, что на роботе.
Встроенный DiffDrive-плагин Gazebo этого не даёт.

Fortress умирает в мае 2027. Когда стек перейдёт на Jazzy, сим переедет на Harmonic.
Описание робота и миры (SDF) при этом почти не меняются [догадка].

Headless с рендер-сенсорами (`gpu_lidar` рендерится): флаг `--headless-rendering` (EGL), нужен ogre2 и доступ
к GPU (`/dev/dri`, группа `render`) ([gz-sim 6 headless_rendering](https://gazebosim.org/api/sim/6/headless_rendering.html)) [док].
Без GPU OGRE уходит в программный рендер (там же).

## 4. Песочница 10.1.1.250 — что на ней есть [измерено, 04.11.2026, только чтение по ssh]

```
test-Katana-GF66-11UD / Ubuntu 22.04.5 LTS / x86_64
CPU: 11th Gen Intel i7-11800H, nproc=16
RAM: 31Gi total, 12Gi used, 21Gi available
Disk /: 916G, 106G used, 763G free
GPU: Intel TigerLake-H GT1 UHD (00:02.0) + NVIDIA GA107M RTX 3050 Ti Mobile (01:00.0)
nvidia-smi: RTX 3050 Ti Laptop GPU, 4096 MiB, 4 MiB used, driver 550.107.02
Docker runtimes: runc io.containerd.runc.v2 (Default: runc) — nvidia-container-toolkit НЕТ (nvidia-ctk не найден)
Дисплей: graphical.target, сессия ros2 на tty2, Type=wayland, /tmp/.X11-unix: X0 X1; DISPLAY в ssh пуст
docker ps: e2e-minio (Up 4 days), e2e-postgres (Up 3 weeks), expertise-observability-observability-1 (Up 3 weeks)
образы: ros:humble-ros-base
load average: 0.76 0.61 0.60, uptime 21 days
```

Выводы:
- По ресурсам хватает с запасом: Gazebo + Nav2 + rtabmap ≈ 4–6 ядер [догадка].
- Пробросить NVIDIA в контейнер нельзя без `nvidia-container-toolkit`: это `sudo apt` и правка
  `/etc/docker/daemon.json`, то есть системная настройка, решает Шифу.
  **Обходной путь без изменений хоста:** отдать контейнеру `--device /dev/dri` (iGPU Intel, Mesa EGL).
  Для одного 2D-лидара на 10 Гц его, скорее всего, хватит [догадка, проверить первым шагом].
  Третий вариант — программный рендер (медленнее, но работает).
- Смотреть: в контейнере `foxglove_bridge` (ws :8765), на Windows — Foxglove;
  или RViz2 в WSLg с `ROS_DOMAIN_ID`/zenoh до песочницы. Окно Gazebo не нужно,
  на нём есть живая Wayland-сессия Шифу — её не трогаем.
- Образы робота собраны под arm64: базовый `ghcr.io/krikz/rob_box_base:ros2-zenoh-humble-latest`
  (`docker/main/nav2/Dockerfile:3`), см. память «песочница: образы arm64». Для сима нужен свой
  amd64-образ (§6, шаг 2).

## 5. Мир

- В репо уже есть `office_cpr.world` и `coridor.world` (§2.1). Карты квартиры или мастерской в репо нет:
  `docker/main/maps` не отслеживается git; карта живёт на роботе в `/maps/rtabmap.db`
  (`docker-compose.yaml:122,175`).
- **Лучший мир — сама мастерская:** снять **копию** `rtabmap.db` с робота (с разрешения Шифу, только
  чтение), выгрузить 2D-сетку (`rtabmap-export`/`/map` → pgm) и выдавить стены высотой 1 м в SDF.
  Тогда сим катается по тем же проходам, что и робот [догадка о трудоёмкости: ~0.5–1 дня].
- Стандартные миры: `aws-robomaker-small-house-world` **заархивирован 21.07.2026**, RoboMaker закрыт
  10.09.2025, в README лицензия не указана ([GitHub](https://github.com/aws-robotics/aws-robomaker-small-house-world)) [док].
  Брать только после проверки LICENSE, как запасной.
- Синтетический «полигон» для замеров: коридоры 1.2 / 1.0 / 0.9 м, дверь 0.8 м, тупик, Г-поворот.
  Робот имеет `robot_radius 0.45` + `inflation 0.55`, то есть ему нужно ≥ 0.9 м
  (`nav2_params.yaml:140,145`), и на этих размерах сразу видно, где у него кончается проходимость.

## 6. План (каждый шаг — отдельный PR в develop, ADR-0013)

| # | PR | Что внутри | Готово, когда (с raw-выводом) | Оценка |
|---|---|---|---|---|
| 0 | — (решение Шифу) | доступ к песочнице для `docker build`; выбрать GPU-путь (§4) | — | — |
| 1 | `feat(sim): rob_box в Gazebo Fortress` | `xacro:arg sim`: железо → `gz_ros2_control/GazeboSimSystem`; колёса — collision-цилиндры, `mu1/mu2`; лидар 360°, 0.15–10 м, 10 Гц; IMU → `/camera/imu/data`; новый пакет `rob_box_sim` (launch, bridge yaml `/clock /scan /imu`, `worlds/`); spawn через `ros_gz_sim create` | `gz sim -s -r --headless-rendering` поднимается; `ros2 topic hz /scan` ≈ 10; с `teleop_twist_keyboard` → `/odom` едет; `xacro`-тест на железный режим (сим-ветка не ломает робота) | 1.5–2 дня |
| 2 | `feat(sim): docker-профиль sim (amd64)` | `docker/sim/Dockerfile` (`ros:humble` + `ros-humble-ros-gz`, `gz-ros2-control`, `nav2-*`, `rtabmap-ros`, `twist-mux`, `foxglove-bridge`); `docker/sim/docker-compose.yaml` монтирует `docker/main/config` **без копий**; переключатель `USE_SIM_TIME=true` в стартовых скриптах (RSP, twist_mux, nav2, rtabmap) через override-yaml | `docker compose -f docker/sim/... up` на песочнице: `ros2 node list`, `/clock` идёт, Nav2 lifecycle `active`, в Foxglove видны карта, скан, путь | 2 дня |
| 3 | `feat(sim): миры` | полигон с размерами из §5; мастерская из копии `rtabmap.db` (если Шифу даст); при совместимой лицензии — `office_cpr` | rtabmap в режиме картирования строит карту мира; скриншот или дамп `/map` | 1 день |
| 4 | `feat(sim): сценарии и метрики навигации` | `scripts/sim/nav_bench.py` на `nav2_simple_commander`: сценарии в yaml (точки, проходы, разворот в тупике, цель за препятствием); метрики: успех/провал, время до цели, длина пути / длина прямой, мин. зазор (по `/scan` и по контактам gz), столкновения (contact-сенсор), осцилляции (смены знака `w` в `/cmd_vel` за метр), число recovery (BT-лог / feedback `NavigateToPose`); вывод JSON/CSV; тест `pytest -v` на подсчёт метрик по записанному bag | один прогон текущего `nav2_params.yaml` → таблица метрик в PR | 2–3 дня |
| 5 | `feat(sim): похожесть на робота` | слой «как VESC» между `cmd_vel` и колёсами: недобор ~20 % (#4004), мёртвая зона `min_duty`, `open_loop`-одометрия; подобрать трение так, чтобы поворот на 360° в симе давал ту же «эффективную колею 1.11»; сверка с bag-ом с робота на земле (когда Шифу поставит его на пол) | расхождение сим/робот по времени поворота и прямой ≤ 10–15 % на одном сценарии (цифры в PR) | 1.5–2 дня + робот |
| 6 | `tune(nav2): параметры по результатам сима` | перебор сетки (`max_vel_x`, `inflation`, критики DWB; DWB vs RPP/MPPI); исправить §2.4 п.1 (переназначение smoother); PR меняет только `docker/main/config/nav2/*.yaml` | таблица «до/после» по метрикам шага 4; потом проверка на роботе (решение Шифу) | 2–3 дня |

**Итого:** ~10–13 рабочих дней воркера до первого PR с подобранными параметрами. Шаги 1–4 —
это рабочий сим (~6–8 дней); шаги 5–6 — польза для настоящего робота. Оценка — [догадка].

## 7. Риски

1. **Физика скид-стира.** Скид-стир с четырьмя колёсами в любом симуляторе поворачивает иначе, чем живой,
   а эффективная колея 1.11 против геометрической 0.39 (`controller_manager.yaml:60-66`) — это и есть
   проскальзывание. Без шага 5 параметры, подобранные в симе, перенесутся на робота с ошибкой,
   особенно по `max_vel_theta`/`acc_lim_theta` и поведению при развороте на месте [догадка].
2. **Привод.** Недобор скорости ~20 % (#4004), мёртвая зона `min_duty` и `open_loop`-одометрия —
   в чистом симе их нет. Если их не смоделировать, сим покажет навигацию лучше, чем она есть на самом деле.
3. **Масса и инерция.** По URDF 149 кг против «62 кг» в комментарии — разгон и торможение в симе
   получатся неверными. Нужен замер (весы).
4. **Задержки.** На роботе Zenoh между Pi и ICP-латентность задают `transform_tolerance`. В симе всё
   в одном контейнере и быстрее, поэтому «слишком щедрые» допуски там не проявятся [догадка].
5. **Real-time factor.** Если песочница не тянет 1.0, все тайм-ауты Nav2 считаются по `/clock` —
   это нормально, но метрику «время до цели» нужно пересчитывать в sim-time.
6. **Две реализации.** Велик соблазн завести `nav2_params_sim.yaml`. Запрещено: только override
   `use_sim_time` и топиков.
7. **Fortress EOL май 2027.** Миграция на Harmonic — при переходе на Jazzy.

## 8. Что нужно от товарища Шифу

1. **Песочница:** можно ли собирать и запускать там контейнер `sim` (без sudo)? Ставить ли
   `nvidia-container-toolkit` (sudo, правка docker daemon — системная настройка), или начинаем
   на iGPU через `/dev/dri`?
2. **Мир:** можно ли снять **копию** `/maps/rtabmap.db` с робота (только чтение), чтобы сделать мастерскую?
   Или есть планировка с размерами?
3. **Масса робота** на весах: в URDF 149 кг, в комментарии 62 кг.
4. **Лидар:** реально 360°? Какая ориентация (0 или π по yaw, §2.1)?
5. **global_costmap без карты rtabmap** (§2.4 п.2) — так задумано?
6. **Что подбирать:** только параметры DWB или можно сравнить с RPP (уже в образе) и MPPI (в образе нет, `nav2-mppi-controller`)?
7. **RMW в симе:** zenoh как на роботе (ближе к правде) или cyclonedds в одном контейнере (проще)?
   Предлагаю zenoh, как на роботе.
