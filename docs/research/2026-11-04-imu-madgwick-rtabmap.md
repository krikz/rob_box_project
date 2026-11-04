# IMU OAK-D → imu_filter_madgwick → rtabmap: замер в Gazebo (04.11.2026)

Решение Шифу 04.11 (TG): «да, подключи» гироскоп/IMU в оценку движения, вариант (а) —
madgwick перед rtabmap. Здесь — что подключено, как выключить и что показал сим.

## Итог одной строкой

IMU подключён (по умолчанию включён), но **коридор он не чинит**: 0/6 целей и с IMU, и без,
ошибка позы в конце того же порядка (5–15 м). Причина — по физике задачи [догадка, не
проверялась отдельным опытом]: в гладком коридоре ICP теряет **продольный сдвиг**, а IMU даёт
только поворот. На polygon IMU ничего не сломал.

## Откуда IMU на роботе [по коду]

- Источник — **OAK-D** на Vision Pi: `i_enable_imu: true` (`docker/vision/config/oak-d/oak_d_config.yaml:36`),
  топик `/camera/camera/imu`, frame `camera_imu_frame`. Ориентация в сообщении фиктивная
  (единичный кватернион), поэтому нужен фильтр. Магнитометра нет → курс по гироскопу (дрейфует).
- VESC IMU в стек не выведен (в compose и `src/` IMU-топиков VESC нет).
- До этого PR `imu_topic` rtabmap смотрел на `/camera/imu/data`, которого на роботе никто не публикует.

## Что сделано

| где | что |
|---|---|
| `docker/main/docker-compose.yaml` | новый сервис `imu-filter` (образ rtabmap): `/camera/camera/imu` → madgwick → `/imu/data`; переменная `RTABMAP_IMU_TOPIC=${RTABMAP_IMU_TOPIC-/imu/data}` у `rtabmap` и `imu-filter` |
| `docker/main/scripts/rtabmap/start_imu_filter.sh` | запуск `imu_filter_madgwick_node`; пустой флаг → `sleep infinity` |
| `docker/main/config/rtabmap/imu_filter_madgwick.yaml` | `use_mag: false`, `world_frame: enu`, `publish_tf: false`, `gain: 0.1` |
| `docker/main/scripts/rtabmap/start_rtabmap.sh` | флаг → `imu_topic:=… wait_imu_to_init:=true`; пусто → `imu_topic:=/camera/imu/data wait_imu_to_init:=false` (как до 04.11) |
| `docker/sim/compose.yaml`, `src/rob_box_sim/config/bridge.yaml` | сим публикует сырой IMU под именем робота `/camera/camera/imu`, `imu-filter` наследуется из боевого compose |
| `src/rob_box_description/urdf/rob_box.xacro` (только `sim:=true`) | шум IMU: гироскоп σ = 0.002 рад/с, смещение 0.0005 ± 0.0002 рад/с; акселерометр σ = 0.02 м/с². Порядок BMI270, **на роботе не мерил** |

**Откат:** `RTABMAP_IMU_TOPIC=` (пусто) в `.env` Main Pi и пересоздать `rtabmap` + `imu-filter`.

### Находка: без `wait_imu_to_init:=true` IMU не доходит до ICP-одометрии [измерено]

С `wait_imu_to_init:=false` (как было в compose) `ros2 topic info /imu/data -v` показал одного
подписчика — `/rtabmap/rtabmap`, а `/rtabmap/icp_odometry` на IMU не подписан. В `rtabmap`
IMU идёт только в гравитационные рёбра графа, а их выключает `Optimizer/GravitySigma=0`
(`docker/main/config/rtabmap/rtabmap.ini`). То есть при старом флаге IMU не работал бы вообще.
С `wait_imu_to_init:=true` в логе `odometry: Subscribing to IMU topic /imu/data` и
`Updated initial pose … rpy=0.000281,0.000471,0.041133 with IMU orientation`.

Цена: пока не пришло ни одного IMU-сообщения, ICP-одометрия **не стартует**.

## Замеры (песочница 10.1.1.250, образ `shidi-imu-rob-box-sim:imu`)

Каждый прогон идёт на чистом стеке (`docker/sim/run_bench.sh`). Прогоны «с IMU» и «без»
шли **парами одновременно**, чтобы нагрузка была одинаковой. Параллельно шли 4 стека шага 6
другого воркера, load average 77–100 на 16 ядрах. Абсолютные цифры поэтому шумнее, чем в #4324.
«Без IMU» — это `RTABMAP_IMU_TOPIC=` (пусто), то есть как на develop до PR.

### corridor (гладкий коридор 10 м), ошибка позы `map→base_footprint` против ground truth, м

| прогон | цель | статус Nav2 | ошибка в конце | макс. | макс. ошибка курса, рад | до цели по истине |
|---|---|---|---|---|---|---|
| imu1 | smooth_10m | TIMEOUT | 8.01 | 8.06 | 0.35 | 1.46 |
| imu2 | smooth_10m | FAILED | 5.48 | 5.51 | 0.16 | 4.67 |
| imu3 | smooth_10m | TIMEOUT | 9.48 | 9.48 | 0.15 | 1.05 |
| off1 | smooth_10m | TIMEOUT | 10.50 | 10.54 | 0.19 | 0.64 |
| off2 | smooth_10m | FAILED | 8.53 | 8.56 | 0.22 | 1.07 |
| off3 | smooth_10m | FAILED | 1.57 | 1.57 | 0.17 | 7.11 |
| imu1 | back_8m | SUCCEEDED* | 14.06 | 14.47 | 0.42 | 13.93 |
| imu2 | back_8m | FAILED | 8.93 | 8.96 | 0.17 | 6.94 |
| imu3 | back_8m | TIMEOUT | 14.98 | 15.14 | 0.26 | 14.75 |
| off1 | back_8m | FAILED | 13.75 | 13.89 | 0.16 | 11.92 |
| off2 | back_8m | TIMEOUT | 13.80 | 13.86 | 0.33 | 13.54 |
| off3 | back_8m | FAILED | 4.34 | 4.80 | 0.28 | 3.66 |

\* Nav2 сообщил SUCCEEDED, но по истине робот в 13.9 м от цели: оценка позы уехала, Nav2 «доехал» в неё.

Сводка (среднее по 3 прогонам):

| | smooth_10m: в конце / макс. | back_8m: в конце / макс. | макс. ошибка курса | успех по истине |
|---|---|---|---|---|
| с IMU (madgwick) | 7.66 / 7.68 м | 12.66 / 12.86 м | 0.25 рад | 0/6 |
| без IMU | 6.87 / 6.89 м | 10.63 / 10.85 м | 0.22 рад | 0/6 |
| база #4324 (без IMU в ICP, меньше нагрузка) | — / 6.19 м (8.32 / 1.74 / 8.51) | — / 6.73 м (10.19 / 1.66 / 8.35) | — | 0/6 |

В #4324 ошибка в конце опубликована только для base3 (8.33 м), поэтому база сравнивается по максимуму.

Разброс внутри варианта (1.6–10.5 м) больше разницы между вариантами. Вывод «IMU помогает»
или «IMU вредит» по трём парам сделать нельзя. Курс с IMU в среднем чуть хуже
[догадка: заложенное в сим смещение гироскопа 0.0005 рад/с плюс начальный курс madgwick,
см. `rpy … 0.041` выше]. В imu1 видно, что истинный x = 15.9 при цели 2.0: оценка
продольной позы стоит, робот едет дальше.

Карта rtabmap в конце (precision из `map_check`): с IMU 0.83 / 0.97 / 0.91, без 0.95 / 0.74 / 0.84.

### polygon (контроль)

| прогон | gap_1.2 | gap_1.0 | ошибка позы макс., м | precision карты |
|---|---|---|---|---|
| с IMU | OK, 22.6 с | TIMEOUT (25 recovery) | 0.03 / 0.05 | 0.92 |
| без IMU | OK, 30.8 с | TIMEOUT (11 recovery) | 0.04 / 0.08 | 0.92 |

gap_0.9 и gap_0.8 в обоих — SKIPPED (стенд останавливается после провала). В #4324 gap_1.0
проходился 2/3, здесь оба варианта 0/1. Это похоже на нагрузку песочницы, а не на IMU, но не доказано.
Ошибка позы на polygon с IMU не выросла (≤ 0.05 м).

## Raw

На песочнице `ros2@10.1.1.250:~/shidi-imu/results/` (2.5 МБ):
`{imu1,imu2,imu3,off1,off2,off3}/corridor.{json,csv,md,log,stack.log}`, `corridor_map.pgm`;
`{imu-poly,off-poly}/polygon.*`; `*.driver.log` — полный вывод `run_bench.sh`.
Код там же (`~/shidi-imu`, `git archive` коммита ветки `feat/imu-madgwick-rtabmap`), драйвер пар — `~/shidi-imu/drive.sh`.

## Что дальше [догадки, не проверены]

1. Продольную потерю в коридоре должна закрывать колёсная одометрия, а не IMU. `odom_guess_frame_id:=odom`
   уже подаёт её как guess, но point-to-point ICP (`Icp/PointToPlane=false` в `rtabmap.ini`) в коридоре
   «стягивает» guess обратно к нулю. Кандидат на опыт в симе: `Icp/PointToPlane=true` и
   `Icp/PointToPlaneMinComplexity` / `Icp/PointToPlaneLowComplexityStrategy`. Тогда в бедном
   направлении берётся guess.
2. `icp_odometry` публикует в `/odom` (`odom_topic:=/odom`, видно в `ros2 node info /rtabmap/icp_odometry`).
   Это то же имя, что у колёсной одометрии робота. Проверить, кто ещё читает `/odom` и что получает.

## Что не проверено

- **На роботе ничего не запускалось.** Ориентация осей `camera_imu_frame` в URDF
  (`rpy="3.14159 0 1.5708"`, `rob_box.xacro`) против реальных осей BMI270 OAK-D не сверялась.
  В симе сенсор висит на этом же фрейме, поэтому совпадает по построению. На роботе при
  несовпадении madgwick увидит гравитацию «вбок».
- Пакет `imu_filter_madgwick` есть в боевом базовом образе: `ghcr.io/krikz/rob_box_base:rtabmap`
  (arm64, sha256:00af7c1d…, создан 2026-08-07) содержит `lib/imu_filter_madgwick/imu_filter_madgwick_node`
  (проверено `docker cp` из созданного контейнера, без запуска). Dockerfile поэтому не менялся.
  Запуск узла на arm64 не проверялся.
- QoS/транспорт: IMU 248 Гц с Vision Pi на Main Pi по zenoh раньше на Main Pi не шёл (подписчика не было).
- Шум IMU в симе выбран по порядку величины, а не измерен.
- По 3 прогона на вариант при сильной фоновой нагрузке.
