# Потолочная камера: калибровка и AprilTag

Потолочная камера: USB «HD USB Camera» (32e4:9310), контейнер `ceiling-camera` на Vision Pi.
Камера смотрит вверх. Нужна для меток на потолке: по ним робот получит абсолютную позу
(исследование «докинг и потолок», раздел B, 04.11.2026).

## 1. Что есть сейчас

| Что | Значение | Откуда |
|---|---|---|
| Режим usb_cam | 640×480, YUYV (`yuv422_yuy2`), 15 к/с | замер 04.11: `ros2 topic echo /ceiling_camera/image_raw` |
| Режимы камеры | MJPG до 1280×720@30; YUYV 640×480@30, 800×600@20, 960×720@15, 1280×720@10 | замер 04.11: `v4l2-ctl --list-formats-ext` |
| `camera_info` | `docker/vision/config/ceiling-camera/ceiling_camera.yaml`: 640×480, fx = fy = 1316, центр кадра, дисторсия 0 | **оценка по одному кадру, не калибровка** |
| `frame_id` кадров | `ceiling_camera_optical_frame` (до 04.11 был `default_cam`: usb_cam 0.8 читает `frame_id`, а не `camera_frame_id`) | по коду, после деплоя проверить |
| TF | `ceiling_camera_optical_frame`: Z вверх, верх кадра — вперёд робота | по коду; поворот вокруг оси не измерен, см. шаг 5 |
| Позиция на корпусе | (−0,171; 0,065; 0,4615) м от `base_link` | экспорт Fusion 360, рулеткой не мерили |

Откуда оценка фокуса. Тег 36h11 ID 49 висел в 0,82 м перед камерой с небольшим смещением,
чёрный квадрат 0,119 м, сторона в кадре ~191 px. Отсюда f ≈ 191 · 0,82 / 0,119 ≈ 1316 px.
Смещение тега и дисторсию оценка не учитывает.

## 2. Калибровка шахматкой

Делается один раз для режима 640×480. Если режим usb_cam сменится (`camera_params.yaml`:
`image_width`, `image_height`), калибровку нужно повторить: у режимов может быть разное
кадрирование сенсора.

### Доска

- Шахматка, например 9×7 клеток, то есть **8×6 внутренних углов**, клетка 25–30 мм.
  Печатать без масштабирования и наклеить на ровное жёсткое основание.
- Сторону клетки измерить линейкой на отпечатке. Принтер искажает масштаб, а от этой цифры
  зависит фокус.
- Фокус ~1300 px даёт узкий угол, примерно 27° по горизонтали. На 0,8 м в кадр попадает поле
  ~0,4 × 0,3 м. Доска A4 должна занимать от трети кадра до почти всего кадра, поэтому держать
  её в 0,5–1,2 м от камеры.

### Вариант А: без графики, скриптом (рекомендуется)

Скрипт `docker/vision/scripts/ceiling-camera/ceiling_calib.py` смонтирован в контейнер
как `/scripts/ceiling_calib.py`.

1. **Снять кадры** (на Vision Pi, камера продолжает работать):

   ```bash
   ssh ros2@10.1.1.21
   docker exec -it ceiling-camera bash -c "source /opt/ros/humble/setup.bash && \
       python3 /scripts/ceiling_calib.py capture --out /tmp/ceiling_calib --count 40 --every 2"
   docker cp ceiling-camera:/tmp/ceiling_calib ./ceiling_calib
   ```

   Пока идёт съёмка (кадр раз в 2 с, 40 кадров), водить доску над камерой:
   - в центре и по всем четырём углам кадра;
   - с наклоном 30–45° в разные стороны;
   - ближе и дальше.

   Доску держать неподвижно в момент кадра: смаз портит углы.

2. **Посчитать** (на любой машине с Python и OpenCV, хоть на Windows):

   ```bash
   pip install opencv-python numpy pyyaml
   scp -r ros2@10.1.1.21:ceiling_calib .
   python docker/vision/scripts/ceiling-camera/ceiling_calib.py calibrate \
       --images ceiling_calib --cols 8 --rows 6 --square 0.025 --out ceiling_camera.yaml
   ```

   Скрипт печатает fx, fy, cx, cy, дисторсию и RMS. Хорошая калибровка: **RMS < 0,5 px**,
   доска найдена хотя бы на 15 кадрах (меньше скрипт не примет). Если RMS > 1 px,
   переснять кадры.

### Вариант Б: `camera_calibration` с окном

Для машины с ROS 2 Humble, дисплеем и Zenoh-сессией к роботу (например, katana):

```bash
sudo apt install ros-humble-camera-calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
    --ros-args -r image:=/ceiling_camera/image_raw -p camera:=/ceiling_camera
```

Водить доску, пока полоски X, Y, Size, Skew не позеленеют, затем нажать **CALIBRATE** и
**SAVE**. Результат ляжет в `/tmp/calibrationdata.tar.gz`, из архива нужен `ost.yaml`.
**COMMIT** не нажимать: он через сервис `set_camera_info` пишет файл внутри контейнера,
а `ceiling_camera.yaml` смонтирован только на чтение.

> Разница вариантов. `cameracalibrator` кладёт в `projection_matrix` матрицу для
> ректифицированного кадра. Скрипт кладёт туда `P = [K | 0]`, потому что apriltag_node
> работает на сыром кадре без ректификации и берёт fx, fy, cx, cy из `P`. При малой
> дисторсии разница мала. Если дисторсия заметна, лучше вариант А.

## 3. Куда положить YAML и как он подключён

1. Заменить файл **`docker/vision/config/ceiling-camera/ceiling_camera.yaml`** результатом.
   В шапке оставить дату, RMS и число кадров. Проверить, что
   `image_width: 640`, `image_height: 480`, `camera_name: ceiling_camera`.
2. Подключение уже есть, менять его не нужно:
   - compose монтирует файл в контейнер:
     `./config/ceiling-camera/ceiling_camera.yaml:/root/.ros/camera_info/ceiling_camera.yaml:ro`;
   - usb_cam читает его по `camera_info_url: file:///root/.ros/camera_info/ceiling_camera.yaml`
     (`camera_params.yaml`) и публикует в `/ceiling_camera/camera_info`.
3. Отдать изменение через PR. На роботе после деплоя:
   `docker compose restart ceiling-camera`. Затем проверить:
   `ros2 topic echo --once /ceiling_camera/camera_info` — должны прийти 640×480 и новые fx, fy.

Тест `tests/unit/docker/test_ceiling_camera_apriltag.py::test_camera_info_matches_usb_cam_mode`
проверяет, что размер в YAML совпадает с режимом usb_cam и что `P` согласована с `K`.

## 4. AprilTag на потолке

Выключено по умолчанию. Включение — переменная в `docker/vision/.env` на Vision Pi:

```bash
CEILING_APRILTAG_ENABLED=true
```

Затем `docker compose up -d ceiling-camera`. Нужен образ с `ros-humble-apriltag-ros`:
его даёт сборка **Build All Services** после мёржа PR.

Цепочка внутри контейнера `ceiling-camera`
(`docker/vision/config/ceiling-camera/launch/ceiling_apriltag.launch.py`):

```
usb_cam  /ceiling_camera/image_raw (yuv422_yuy2, 15 к/с) + /ceiling_camera/camera_info
   │
ceiling_tag_feed.py   ≤ 5 Гц, Y-канал YUYV → mono8
   │  /ceiling_camera/apriltag/image + /ceiling_camera/apriltag/camera_info
   │
apriltag_node «ceiling_apriltag»   threads 1, decimate 2, 36h11, size 0.119
   ├─ /ceiling_camera/apriltag/detections   (apriltag_msgs/AprilTagDetectionArray)
   └─ /ceiling_camera/apriltag/tf           (позы тегов от ceiling_camera_optical_frame)
```

- **Почему не `/detections`.** Это топик AprilTag OAK-D, его читают rtabmap и
  context_aggregator. У камер разные `frame_id`, смешивать их нельзя.
- **Почему не общий `/tf`.** apriltag_node называет кадры тегов `tag36h11:<id>`, так же, как
  детектор OAK-D. Если одну метку видят обе камеры, у её кадра в общем дереве будет два
  родителя.
- **Почему своя кормушка, а не image_proc.** Она прореживает поток до 5 Гц и берёт яркость
  YUYV одним срезом, без перевода цвета. Умеет ли cv_bridge 3.2.1 сам переводить
  `yuv422_yuy2` в mono8, не проверено. С кормушкой этот вопрос не важен.
- **Потребителя детекций пока нет.** Им станет нода «потолочная поза» (relocalize), это
  следующий шаг. До неё смотреть так: `ros2 topic echo /ceiling_camera/apriltag/detections`.
- **Нагрузка не измерена.** Оценка: +10–20 % одного ядра Vision Pi на детектор и
  кормушку. Замер после включения: `docker stats ceiling-camera`, температура
  `vcgencmd measure_temp`.

## 5. Проверка TF (нужна Шифу)

В URDF принято: оптическая ось Z — вверх, верх кадра — вперёд робота, правый край кадра —
влево. Это допущение, его надо проверить.

1. Повесить тег над камерой, включить AprilTag (шаг 4).
2. Подвинуть тег **вперёд** по ходу робота. В кадре (`/ceiling_camera/image_raw` в RViz или
   Quest) тег должен уйти **вверх**. По позе в `/ceiling_camera/apriltag/tf` координата
   **−Y** должна вырасти.
3. Подвинуть тег **влево** от робота. Тег должен уйти **вправо** в кадре, координата **+X**
   должна вырасти.

Если направления не совпали, поправить `optical_rpy` у `ceiling_camera` в
`src/rob_box_description/urdf/rob_box.xacro`:

| Верх кадра смотрит | `optical_rpy` |
|---|---|
| вперёд (+X) | `0 0 1.5708` (сейчас) |
| назад (−X) | `0 0 -1.5708` |
| влево (+Y) | `0 0 3.14159` |
| вправо (−Y) | `0 0 0` |

Затем перегенерировать `src/rob_box_description/test/golden/rob_box_real.urdf` (`xacro`),
поправить ожидание в `src/rob_box_description/test/test_ceiling_camera_tf.py` и, если камера
стоит с наклоном, измерить его.

Позицию (−0,171; 0,065; 0,4615) м тоже стоит проверить рулеткой: от центра между осями
колёс на полу до центра объектива. Для меток на потолке 2–3 м ошибка позиции в 1 см почти
не важна. Наклон камеры важнее: 1° на 2 м до потолка даёт ~3,5 см.
