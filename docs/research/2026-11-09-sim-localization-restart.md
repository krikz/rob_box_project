# Локализация после перезапуска в Gazebo: своя и чужая база, сдвиг, похищение, тег в карте (#4346, вариант D)

Дата: 09.11.2026. Карточки: #4346 (вариант D), #4468 (блок «Локализация»).
Метки: **[измерено]** — прогон на песочнице 10.1.1.250 с raw ниже; **[по коду]** — по исходникам;
**[догадка]** — не проверено.

## 1. Коротко

РЕЗУЛЬТАТЫ_КОРОТКО

## 2. Стенд

Что добавлено в репо (PR):

- **Камера OAK-D в сим-URDF** (`rob_box.xacro`, только `sim:=true` и `sim_camera:=true` / `SIM_CAMERA=true`):
  сенсор `camera` Gazebo на `camera_rgb_camera_frame`, кадр сообщений `camera_color_optical_frame` (тот же TF
  `base_link → camera_color_optical_frame`, что у робота), HFOV 1.133 рад (по K робота), 640×360, 5 Гц, near 0.45 м.
  Мост — `config/bridge_camera.yaml`, топики как у OAK-D: `/camera/camera/color/{image_raw,camera_info}`.
  `SIM_CAMERA_YAW_ERROR` (рад) поворачивает ось камеры относительно URDF — для гипотезы «калибровка камера↔лидар».
- **Теги tag36h11 21, 22, 23** в мире `polygon_tags` (`rob_box_sim.apriltag_world`): текстура рисуется из
  открытого формата (коды и раскладка битов `tag36h11.c`, BSD-2), чужих картинок нет. Размер — как у тегов Шифу:
  чёрный квадрат 0.119 м, с рамкой 0.1785 м. Высота центра 0.55 м. 21 — на перегородке x=2 (лицом на запад,
  виден со спавна), 22 — на западной стене (виден по пути домой), 23 — на восточной стене (у дальней цели).
- **apriltag_node** в `docker/sim/compose.yaml`, профиль `tags`: тот же узел и тот же
  `docker/vision/config/apriltag/apriltag_config.yaml`, что в контейнере oak-d робота; выход `/detections`, который
  слушает боевой rtabmap (`tag_topic:=/detections`). Отличия сима: `image_transport raw`, `detector.decimate 1.0`
  (кадр вдвое меньше боевого, где 2.0), `size` из `SIM_APRILTAG_SIZE` (0.119).
- **Вес тега** `SIM_TAG_LINEAR_VARIANCE` → `start_rtabmap.sh` → `tag_linear_variance:=` (только при
  `USE_SIM_TIME=true`; на роботе отказ). Без переменной — 0.0001 из `rtabmap.launch.py`, как на роботе.
- **Стенд** `rob_box_sim.loc_bench` + метрики `rob_box_sim.loc_metrics`, сценарии `scenarios/localization.yaml`,
  оркестратор `docker/sim/run_loc_bench.sh` (`map` — картирование, `loc` — «перезапуск»), оверлей
  `docker/sim/compose.loc.yaml` (база rtabmap — каталог хоста).

Как устроен прогон:

1. `map <метка> <мир>`: rtabmap в mapping с пустой базой, Nav2 ведёт робота по `scenarios/mapping.yaml`
   (через все проёмы полигона до восточной стены и домой, курс π). Спавн (0, 0, 0), поэтому `map` = мир Gazebo.
   После маршрута `docker stop -t 60` rtabmap — он пишет базу и последнюю позу; база и истинная поза в конце
   маршрута сохраняются.
2. `loc <метка> <мир> <база> <сценарий>`: свежая копия базы, Gazebo со спавном по сценарию, rtabmap в
   localization (как на роботе: `start_rtabmap.sh`, тот же `patch_rtabmap_launch.py`), без Nav2.
   С момента `/clock` стенд 10 Гц пишет истину (`/sim/ground_truth/odom`) и оценку (TF `map → base_footprint`).
   Проба: 20 с стоит, круг влево 0.5 рад/с, 10 с, круг вправо, 15 с (~70 с sim). «Похищение» —
   `ign service /world/<мир>/set_pose` посреди пробы.
3. Метрики (`loc_metrics.run_metrics`) от старта стека (или от телепорта): ошибка первой оценки; «сошлось» —
   первый момент, после которого ошибка ≤ 0.15 м и ≤ 5° держится ≥ 5 с; сколько проехал до этого; медиана ошибки
   за последние 5 с; скачки оценки (за шаг оценка сдвинулась на 0.25 м больше истины) и «ложные» скачки (после
   скачка ошибка выросла); RTF.

Сценарии (`scenarios/localization.yaml`):

| сценарий | база | спавн после «перезапуска» |
|---|---|---|
| (а) `restart_same` | своя | там же, где робот стоял при остановке |
| (б) `restart_shift05` | своя | сдвиг 0.5 м по x |
| (б') `restart_shift05_turn30` | своя | сдвиг (0.4, 0.3) м и поворот 30° |
| (б'') `restart_far` | своя | другая комната, (5, 0, 0) |
| (в) `foreign_same` / `foreign_far` | чужая (карта мира `turn90`) | там же / (5, 0, 0) |
| (г) `kidnap` | своя | на месте; через 20 с телепорт в (0.5, −1.5, 90°) |

## 3. Результаты

РЕЗУЛЬТАТЫ

## 4. Что не проверено

НЕ_ПРОВЕРЕНО

## 5. Рекомендация по A / B / C

РЕКОМЕНДАЦИЯ

## 6. Как повторить

```bash
# на песочнице, в копии репо (CRLF почищен: docs/runbooks/hil-video.md §1)
docker build -f docker/sim/Dockerfile -t shidi-loc-rob-box-sim:4346 .
export SIM_IMAGE=shidi-loc-rob-box-sim:4346 SIM_DRI_CARD=/dev/dri/card1
docker/sim/run_loc_bench.sh map m_notag polygon
docker/sim/run_loc_bench.sh map m_turn90 turn90
for i in 1 2 3; do docker/sim/run_loc_bench.sh loc a$i polygon m_notag restart_same; done
# с тегами: SIM_CAMERA=true, мир polygon_tags; вес тега: SIM_TAG_LINEAR_VARIANCE=0.01
SIM_CAMERA=true docker/sim/run_loc_bench.sh map m_tag polygon_tags
SIM_CAMERA=true docker/sim/run_loc_bench.sh loc ta1 polygon_tags m_tag restart_shift05
# прототип варианта B: SIM_CAMERA=true SIM_RELOC=true
```

## 7. Raw

RAW
