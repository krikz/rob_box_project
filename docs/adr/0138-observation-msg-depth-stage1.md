# ADR-0138: Наблюдение (`Observation.msg`) и глубина OAK-D → 3D — этап 1 ADR-0130

| Поле | Значение |
|---|---|
| Статус | Proposed (этап 1 `v2.0.0`; код в том же PR, на роботе не проверен) |
| Дата | 2026-09-28 |
| Автор | Claude Code (воркер этапа 1) по ADR-0130 товарища Шифу |
| Домен | RT (восприятие, Vision Pi → Main Pi) |
| Контекст | Детекции человека и лица (`VisionEvent`) не несут геометрии: bbox в долях кадра, `distance_m = −1` всегда (`vision_hailo_loader.py:1027`, `vision_face_loader.py:519`), нет `frame_id` кадра, нет 3D, `stamp` = время публикации. Трекеру этапа 2 нечего собирать в треки, safety stop ADR-0089 Phase 1 нечем мерить дистанцию. OAK-D Lite уже отдаёт глубину, выровненную по RGB 1280×720 (`oak_d_config.yaml`: `i_align_depth: true`), и `camera_info` с K. |
| Затрагивает | `src/rob_box_perception_msgs/msg/Observation.msg` (новый), `CMakeLists.txt`, `VisionEvent.msg` (только комментарий deprecated); `src/rob_box_perception/rob_box_perception/{gaze.py, observation_geometry.py (новый), vision_hailo_node.py, launch_factory.py}`; `docker/vision/config/hailo_models.yaml`; `docker/vision/scripts/vision-hailo/start_vision_{hailo,face}.sh`; тесты `src/rob_box_perception/test/unit/test_{observation_geometry,observation_parity,gaze_depth,vision_hailo_observation}.py` |
| Родители | ADR-0130 §7 этап 1 (а также §1.1, §1.3, §2.1, §2.2, §2.5, §2.11, §5.1), ADR-0013 (один этап — один PR), ADR-0018 (честный FAIL, `capability-honest`) |
| Связанные | ADR-0104 («Взгляд» — шов источника кадра, сюда же приезжает глубина), ADR-0089 Phase 1 (safety stop по дистанции — потребитель `distance_m`, в этом PR не делается), ADR-0123 (приватность лица: `candidate_id` — ссылка, не имя), #2826 (поза RTAB-Map — нужна трекеру этапа 2 для `map`, не этому этапу) |

> **TL;DR.** Vision Pi начинает публиковать `rob_box_perception_msgs/Observation` на `/perception/observations`: детекция `person` (YOLOv8n) или `face` (RetinaFace) с bbox в пикселях кадра, 3D-точкой в **optical frame камеры**, дистанцией, разбросом глубины и честным статусом «почему глубины нет». `header.stamp` — время **кадра**, `header.frame_id` — фрейм кадра. Имени нет; лицо несёт только `candidate_id` — ссылку на запись хранилища примет. Глубина приходит через шов «Взгляд» (`OakDSource` подписывается на выровненную глубину и `camera_info`, связывает с RGB по ближайшему stamp). `VisionEvent.distance_m` заполняется той же оценкой. TF на Vision Pi не подключается — в `base_link`/`map` переводит трекер этапа 2. `Track.msg` и трекер — этап 2.

---

## 1. Контекст: что было до этого PR (проверено по коду 28.09.2026)

| Что | Факт | Где |
|---|---|---|
| bbox | нормирован в [0,1] исходного кадра (после снятия letterbox) | `vision_hailo_loader.py:980-995`, `VisionEvent.msg` |
| `distance_m` | всегда −1 у реальных детекций; у stub — выдуманный 1.0 | `vision_hailo_loader.py:1028`, `vision_face_loader.py:519`, `:278` |
| `stamp` | `get_clock().now()` в момент публикации | `vision_hailo_node._publish_event` |
| фрейм кадра | в сообщение не попадает (кроме `source_camera = frame_id`, который ставит loader) | `vision_hailo_loader.py:616` |
| глубина | драйвер настроен (`i_align_depth: true`, 1280×720, 5 fps, `i_enable_lazy_publisher: true` — публикует, только пока есть подписчик) | `docker/vision/config/oak-d/oak_d_config.yaml` |
| RTAB-Map | LiDAR-only: `depth:=false`, `subscribe_depth: false`, `subscribe_rgbd: false` — глубину **не** читает | `docker/main/docker-compose.yaml:137`, `docker/main/config/rtabmap/rtabmap.yaml:23` |
| кто читает глубину | только по запросу и только `compressedDepth`: Quest-панель (`quest_node.py:1952`) и фото в Telegram (`telegram_node.py:105`), оба на Vision Pi | см. файлы |

Постоянного потребителя raw-глубины в репозитории нет, RTAB-Map доказательством «raw depth ходит через Zenoh» не служит. Косвенное доказательство существования raw-топика — комментарий `quest_node.py:155-165`: 08.09.2026 на роботе `/camera/camera/depth/image_rect_raw/compressedDepth` имел `format: '16UC1; compressedDepth'`. `compressedDepth` — плагин image_transport, он висит на том же издателе, что и raw-топик `/camera/camera/depth/image_rect_raw` с кодировкой `16UC1`. Мною на роботе не перепроверено. См. §7 п.1.

### 1.1 Какой depth-топик публикует драйвер (по исходникам, не на роботе)

`depthai_ros_driver` v2.12.2-humble, `src/dai_nodes/sensors/stereo.cpp` и `depthai_bridge/src/ImageConverter.cpp` (прочитаны 28.09.2026 с GitHub, тег `v2.12.2-humble`):

- топик — `~/<имя стерео-ноды>` + суффикс `/image_rect_raw` при `i_rs_compat: true`; с `namespace=camera`, `name=camera` и переименованием `stereo→depth` это `/camera/camera/depth/image_rect_raw` — так и записано в шапке `oak_d_config.yaml`;
- `i_publish_compressed: false` → публикуется обычный `sensor_msgs/Image`, а не только compressed;
- `i_low_bandwidth: true` у стерео → на устройстве MJPEG-кодируется **8-битный disparity**, хост декодирует и переводит в глубину `baseline·10·fx / disparity`, кодировка `16UC1` (мм), `0` = нет данных.

Вывод (исходники драйвера + замер 08.09 из `quest_node.py`): raw-топик `16UC1` есть, декодер `compressedDepth` не нужен и не пишется. Но глубина **квантована** (8 бит disparity) и прошла через сжатие с потерями (MJPEG, `quality: 90`): погрешность растёт примерно как Z², на краях объектов возможны артефакты. Числа погрешности на роботе не замерены (§7 п.2).

---

## 2. Решение

### 2.1 Поток данных

```
OAK-D ─► /camera/camera/color/image_raw ──────┐
      ─► /camera/camera/depth/image_rect_raw ─┤  gaze.OakDSource: RGB + ближайший по stamp depth
      ─► /camera/camera/color/camera_info ────┘  (допуск depth_sync_tolerance_sec) + K → Frame
                                                        │
                           HEF (YOLOv8n | RetinaFace) ◄─┘ тот же Frame
                                                        │
            observation_geometry.build_observation(event, Frame)
                 │                                   │
   /vision/hailo/events (VisionEvent,        /perception/observations (Observation,
   distance_m теперь заполнен)               header = stamp + frame_id КАДРА)
```

`VisionHailoNode._tick` фиксирует кадр **один раз**: на нём инференс, с его stamp и глубиной публикуется наблюдение (ADR-0130 §2.5 — время наблюдения, а не доставки). `vision-hailo` и `vision-face` публикуют одинаково — код в базовой ноде.

### 2.2 `Observation.msg` — минимальное общее ядро

| Поле | Тип | Смысл |
|---|---|---|
| `header` | `std_msgs/Header` | `stamp` = время кадра; `frame_id` = optical frame камеры из header кадра |
| `source` | `string` | id датчика = имя адаптера «Взгляда» (`oak_d`, `ceiling_camera`) |
| `class_name` | `string` | класс **детекции**: `person` \| `face` |
| `confidence` | `float32` | уверенность детектора |
| `bbox_{cx,cy,w,h}_px` | `float32` | bbox в пикселях исходного кадра |
| `image_width`, `image_height` | `uint32` | размер этого кадра |
| `position` | `geometry_msgs/Point` | центр детекции в optical frame, м; NaN×3 при невалидной глубине |
| `distance_m` | `float32` | \|position\|, м; −1 при невалидной глубине |
| `position_valid` | `bool` | true только при `position_status = "ok"` |
| `position_status` | `string` | `ok`, `no_depth_stream`, `depth_out_of_sync`, `depth_size_mismatch`, `depth_encoding_unsupported`, `no_camera_info`, `camera_info_size_mismatch`, `bbox_outside_frame`, `depth_invalid` |
| `position_stddev_m` | `float32` | σ глубины инлаеров в ядре бокса; −1 — нет оценки |
| `candidate_id` | `string` | ссылка на биометрическую улику: сейчас `person_id` FaceStore, после этапа 3 — `Acquaintance.id`; пусто — улики нет |
| `candidate_similarity` | `float32` | сходство с `candidate_id`; −1 — не посчитано на этом кадре |

Словарь статусов живёт в одном месте — `observation_geometry.STATUS_*`; тест `test_status_vocabulary_documented_in_idl` не даёт ему разойтись с комментарием IDL.

**Пиксели + размер кадра, а не доли.** 3D считается через K, а K задана в пикселях кадра конкретного разрешения: доли всё равно пришлось бы умножать на размер. С `image_width/height` сообщение без потерь даёт и пиксели, и доли (`bbox_*_px / image_*`), и позволяет потребителю проверить, что K и bbox — про один кадр. У `VisionEvent` доли остаются как были.

**Отличия от эскиза ADR-0130 §2.14** (там сказано «окончательно — в дочернем ADR»):

| Эскиз | Здесь | Почему |
|---|---|---|
| `string modality` + `string class` | один `class_name` (`person` \| `face`) | в этапе 1 модальность однозначно следует из класса детекции; `modality` без второго производителя (голос, DOA, лидар) — поле без потребителя. `class` — ключевое слово Python: сгенерированный Python-класс с таким полем как минимум неудобен (как на него реагирует rosidl, не проверялось — не рискуем) |
| `geometry_msgs/PointStamped position` | `header` + `geometry_msgs/Point position` | фрейм и время уже в `header`; второй stamp мог бы разойтись с первым |
| `float64[9] position_covariance` | `float32 position_stddev_m` | откалиброванной ковариации нет; 9 чисел, из которых честно известно одно, выглядели бы точнее, чем есть. Разброс по боксу — это разброс **сцены** в боксе, не ошибка датчика (§5) |
| `float32 bearing_rad` | нет | нужен DOA (этап 5) — производителя нет |
| `string embedding_ref` | `candidate_id` + `candidate_similarity` | трекеру нужна улика со сходством, а не сырой указатель на вектор |
| `string attributes_json` | нет | потребителя нет; маркер Встречи остаётся в `VisionEvent` |

### 2.3 Геометрия bbox + depth → 3D (`observation_geometry.estimate_position`)

1. Ядро бокса — центральные 50 % по ширине и высоте (края person-бокса — фон между рук и за плечами).
2. Валидные пиксели: `0.1 м ≤ Z ≤ 20 м` (0 = «нет данных» у драйвера). Меньше 5 % валидных в ядре → `depth_invalid`.
3. Медиана; выбросы — дальше `max(3·1.4826·MAD, 5 см)` от медианы; `Z` = медиана инлаеров, `stddev` = σ инлаеров.
4. Центр бокса `(u, v)` через K: `x = (u−cx)·Z/fx`, `y = (v−cy)·Z/fy`, `z = Z` (глубина OAK-D — Z-глубина, REP-118). `distance_m = |(x, y, z)|`.

Модуль чистый (numpy лениво, без rclpy/cv2). Отдельный модуль, а не `gaze.py` и не нода: у шва «Взгляд» нет bbox (он отвечает за кадр), а `vision_hailo_node.py` импортирует `rclpy` на уровне модуля — без ROS его тесты пропускаются, и контракт геометрии не проверялся бы в CI.

### 2.4 Глубина через шов «Взгляд» (ADR-0104)

- `Frame` получил необязательные поля `depth_mm` (HxW uint16, мм, выровнена по RGB), `intrinsics` (fx, fy, cx, cy) и `depth_status`. По умолчанию — `None/None/no_depth_stream`: `CeilingCameraSource` и `StubSource` глубины не имеют и так и говорят.
- `OakDSource` дополнительно подписан на `/camera/camera/depth/image_rect_raw` (`sensor_msgs/Image`, `16UC1`/`mono16`) и `/camera/camera/color/camera_info`. Держит 8 последних depth-сообщений; к RGB-кадру прикладывается ближайшее по stamp, если `|Δt| ≤ depth_sync_tolerance_sec` (по умолчанию 0.15 с — при 5 fps соседние кадры идут через 0.2 с). Если depth пришёл **после** своего RGB, связывание повторяется при его приходе (только для статусов `no_depth_stream`/`depth_out_of_sync` — остальное новым depth не лечится, а декод 1280×720 стоит CPU). `message_filters` не нужен.
- Размер depth ≠ размер RGB → `depth_size_mismatch` (выравнивание сломано — не масштабируем молча). Размер из `camera_info` ≠ размер RGB → `camera_info_size_mismatch`.
- **TF на Vision Pi не подключается** (ADR-0130 §2.11): наблюдения уезжают в optical frame, перевод в `base_link`/`odom`/`map` — у трекера на Main Pi (этап 2), где живут TF, лидар и карта. Это уточняет строку §7 «→ `base_link`/`map`»: этап 1 доводит наблюдение до optical frame, преобразование — этап 2.
- Параметры ноды (и launch/YAML/скрипты, как у `output_topic`): `observation_topic` (`/perception/observations`), `depth_enabled` (`true`; `false` — рычаг, если глубина съест CPU: подписки нет, все наблюдения с `no_depth_stream`), `depth_sync_tolerance_sec` (`0.15`).

### 2.5 Что меняется у `VisionEvent`

- Поля не меняются; в шапке `.msg` — «deprecated с ADR-0130, замена — `Observation.msg`».
- `distance_m` у реальных детекций `person`/`face` берётся из той же оценки, что `Observation.distance_m` (−1 при невалидной глубине — как и было в контракте «negative = unknown»; `perception_projection.py:190` уже отдаёт LLM `None` для отрицательных).
- Stub-события (ADR-0089 §2.2) не трогаются и в наблюдения не попадают — кадра за ними нет. Объекты (`cup`, `chair`…) — только `VisionEvent` (ADR-0130 §2.2: профили кроме `person` не пишутся).

---

## 3. Контракт топика `/perception/observations` (шаблон `docs/design/ARCHITECTURE_AUDIT.md`)

| Пункт | Значение |
|---|---|
| **name** | `/perception/observations` (параметр `observation_topic`) |
| **type** | `rob_box_perception_msgs/msg/Observation` |
| **semantic owner** | `rob_box_perception` / `VisionHailoNode` — «датчик видел детекцию здесь-то» |
| **publishers** | `vision_hailo` (класс `VisionHailoNode`, контейнер `vision-hailo`, `class_name=person`) и `vision_face` (класс `VisionFaceNode` ⊂ `VisionHailoNode`, контейнер `vision-face`, `class_name=face`). Два писателя одного топика — **намеренно**: каждое сообщение самодостаточно (`source`, `class_name`, `header`), порядок между писателями не важен. Одна реализация публикации в базовом классе |
| **subscribers** | пока нет. Потребитель — камерный трекер на Main Pi (ADR-0130 этап 2) |
| **purpose** | мгновенный факт датчика со временем кадра, фреймом, геометрией и погрешностью — вход трекера; дистанция для safety stop (ADR-0089 Phase 1) |
| **what it must NOT mean** | не «кто это» — имени нет, `candidate_id` — ссылка на улику, не решение; не «здесь один человек» — один человек в одном кадре даёт до двух наблюдений (`person` + `face`), склейка — дело трекера; не трек — нет памяти и id объекта; `position_valid=false` не значит «рядом никого» и не значит «далеко» — значит «неизвестно»; не `base_link`/`map`; `header.stamp` — не время публикации; не для прямой подачи в LLM (для этого — Встреча/трек) |
| **static evidence** | publisher: `vision_hailo_node.py` (`create_publisher(ObservationMsg, self.observation_topic, 10)` в `__init__`); IDL: `Observation.msg`; поля: `observation_geometry.OBSERVATION_FIELDS` (паритет — `test_observation_parity.py`). `tools/architecture_audit.py` имя **не видит**: он читает только литеральные имена топиков, а здесь имя из параметра — ровно как у `/vision/hailo/events` (у которого по той же причине в находках висит `subscribe_without_static_publisher`) |
| **runtime evidence** | **нет** — на роботе не запускалось. Ожидаемо в `L: Architecture Audit` (runtime-findings): `multiple_writers` (два контейнера, см. выше — намеренно) и `dead_output` (нет подписчика до этапа 2) |

**Почему не заводим фиктивного подписчика.** Находка «публикуется, но никто не читает» — правда до этапа 2. Подписчик-заглушка спрятал бы реальный разрыв и добавил бы ноду без ответственности (вопросы «Node review questions»: «What problem does this node solve?» — никакой). Временная находка лучше выдуманного потребителя; закрывается трекером этапа 2. В `architecture/ownership.yml` и `runtime-baseline.json` этим PR ничего не пишется — только после ручной проверки Шифу.

**Zenoh.** В `docker/{vision,main}/config/zenoh_{router,session}_config.json5` нет ни ACL (`access_control` закомментирован), ни `downsampling`; единственные правила — `congestion_control: "drop"` для `rt/camera/**`, `rt/ceiling_camera/**`, `rt/rtabmap/**`, `rt/*_costmap/**`. `/perception/observations` под них не попадает и должен доходить до Main Pi как любой топик. **Не проверено на роботе.** Depth под `rt/camera/**` попадает: при перегрузке кадры глубины могут отбрасываться — тогда наблюдения честно получат `no_depth_stream`/`depth_out_of_sync`.

---

## 4. Инварианты и как они проверяются

| Инвариант (ADR-0130 §5.1) | Проверка |
|---|---|
| п.1 Наблюдение не содержит имени | `test_observation_parity.py::test_idl_carries_no_name` (в IDL нет `display_name`/`name`/…); `test_observation_geometry.py::test_face_observation_carries_candidate_without_name`, `::test_observation_fields_have_no_identity_name`; `test_vision_hailo_observation.py::test_face_observation_has_candidate_but_no_name` (VisionEvent с именем → Observation без него) |
| п.5 Модальность не выдаёт свой `id` личности | `candidate_id` — только копия `embedding_id` записи хранилища примет, нода ничего не генерирует; те же тесты |
| п.3 (в части этапа 1) у наблюдения есть время наблюдения | `test_vision_hailo_observation.py::test_person_publishes_vision_event_with_distance_and_observation` (header = stamp кадра до наносекунд), `::test_tick_uses_same_frame_for_infer_and_observation` |
| п.7 Отказ канала виден | `test_observation_geometry.py` (нет глубины / нет K / все нули / вне кадра / статус шва пробрасывается); `test_gaze_depth.py` (рассинхрон, размеры, нет `camera_info`, depth выключен, потолочная и stub — без глубины); `test_vision_hailo_observation.py::test_no_depth_keeps_minus_one_and_honest_status` |
| ADR-0130 §2.2 только профиль `person` | `test_build_observation_skips_other_classes`, `test_object_class_gets_no_observation` |
| ADR-0089 §2.2 stub не выдаёт себя за детекцию | `test_stub_event_untouched_and_not_observed`, `test_tick_stub_mode_publishes_no_observations` |
| IDL ↔ Python | `test_observation_parity.py` (порядок и состав полей, типы ключевых полей, `Observation.msg` в `rosidl_generate_interfaces`) |

---

## 5. Отвергнутые альтернативы

| Альтернатива | Почему нет |
|---|---|
| TF на Vision Pi и публикация сразу в `base_link` | TF `base_link → camera_*` публикует Main Pi; тащить TF-буфер через Zenoh на Vision Pi ради того, что трекер всё равно делает на Main Pi, — лишняя зависимость и лишний CPU на уже загруженном Pi (ADR-0130 §1.3, §2.11) |
| `message_filters.ApproximateTimeSynchronizer` | даёт колбэк только на пару; нам нужен RGB-кадр и без пары (с честным статусом), и повторное связывание, когда depth опоздал. 20 строк кода проще и тестируются без rclpy |
| Глубина в точке центра бокса | один пиксель — ноль/артефакт на границе; для person-бокса центр часто между рук |
| Глубина по всему боксу | в боксе человека много фона; медиана по ядру устойчивее |
| Масштабировать bbox, если depth другого размера | размер ≠ RGB означает, что выравнивание не то, которое мы ожидаем; молча масштабировать = молча врать о точности |
| Декодер `compressedDepth` «на всякий случай» | raw-топик есть по исходникам драйвера (§1.1); декодер без потребности |
| `position_covariance float64[9]` | нечего честно положить в 8 из 9 чисел (§2.2) |
| Тащить `Track.msg` сейчас | ни производителя, ни потребителя до этапа 2; формат трека решается вместе с трекером |
| Фиктивный подписчик ради аудита | §3 |
| Отдельный параметр/лог «глубина есть» вместо статуса в каждом сообщении | потребитель (трекер, safety stop) должен знать про **это** наблюдение, а не про ноду в целом |

---

## 6. Что сознательно НЕ делается на этапе 1

- `Track.msg`, трекер, «Встреча» из треков — этап 2.
- TF-подписки, перевод в `base_link`/`odom`/`map` — этап 2 (трекер на Main Pi).
- Safety stop ADR-0089 Phase 1 (`safety_stop_node`, twist_mux) — отдельный PR; этот этап только даёт ему `distance_m`. Требование к нему уже сейчас: `position_valid=false` — «неизвестно», не «свободно».
- Покадровое `candidate_similarity`: `FaceRecognizer._annotate_identities` кладёт в `VisionEvent` только `embedding_id`/`display_name`; сходство есть лишь на кадре Встречи (`attributes_json.similarity`). Логику узнавания (`face_store.py`, `face_tracker.py`, `face_recognition.py`) этот этап не трогает — поэтому на остальных кадрах `candidate_similarity = −1` («не посчитано»). См. §7 п.4.
- Миграция потребителей с `VisionEvent` (`context_aggregator_node`, `dialogue_node`, `mcp_tools`) — по мере появления трекера.
- Изменения `oak_d_config.yaml` (в т.ч. `i_low_bandwidth` у depth) — вопрос к Шифу (§7 п.2).
- Потолочная камера — глубины нет, статус честный; участвует ли она в восприятии людей — ADR-0130 §8 п.2.

---

## 7. Открытые вопросы (для Шифу)

1. **Реальный depth-топик и нагрузка — не проверено на роботе.** Сейчас raw-depth постоянно никто не читает (Quest/Telegram — `compressedDepth` по запросу), драйвер с `i_enable_lazy_publisher: true` в покое её, скорее всего, не публикует. После этого PR подписываются **две** ноды (`vision-hailo`, `vision-face`): драйвер начнёт хостовый декод MJPEG disparity → depth (CPU контейнера `oak-d`), а каждая нода будет копировать и десериализовать 1280×720×2 ≈ 1.8 МБ × 5 Гц. На Vision Pi (load ≈ 5 на 4 ядра, ADR-0130 §1.3) это нужно замерить (этап 0, замер CPU). Рычаг отката без пересборки — `DEPTH_ENABLED=false` / `depth_enabled: false`.
2. **Точность глубины при `i_low_bandwidth: true`.** 8-битный disparity через MJPEG: квантование растёт ~Z², ближняя граница по disparity — порядка десятков сантиметров (не посчитано для конкретной калибровки). Для safety stop на 1.5 м может хватить, для трекера на 4–5 м — вопрос. Решение `i_low_bandwidth: false` для depth — после замера: трафик Vision→Main (причина включения в марте 2026) не затрагивается, пока на Main Pi никто не подписан на depth, но растёт USB/CPU.
3. **Межпайный TF (ADR-0130 §8 п.5)** — переносится на этап 2: трекер будет искать `base_link ← <optical frame>` на Main Pi по `header.stamp` наблюдения; задержка TF и расхождение часов (chrony — этап 0) там и замеряются. Реальный `frame_id` RGB-кадра OAK-D с `i_publish_tf_from_calibration: false` и то, есть ли для него статический TF в URDF, **не проверены** — трекеру этапа 2 он нужен.
4. **Покадровое сходство лица.** Нужно ли `candidate_similarity` на каждом кадре (одна строка в `_annotate_identities` — но это файл логики узнавания, этапы 3–4), или трекеру хватит `candidate_id` + сходства на Встрече?
5. **Задержка depth относительно RGB** и уместность допуска 0.15 с — не замерены. Метрики доли статусов (`ok` / `depth_out_of_sync` / …) в Prometheus этим PR не добавляются; первая проверка на роботе — `ros2 topic echo /perception/observations --field position_status`.
6. **Разрыв статического аудита.** `tools/architecture_audit.py` не видит топики, имя которых берётся из параметра (`self.output_topic`, `self.observation_topic`) или атрибута класса (`OakDSource.depth_topic`). Поэтому этот PR даёт 0 новых статических находок, хотя по сути добавляет publisher без подписчика. Стоит ли учить аудит дефолтам `declare_parameter` — отдельная задача.

---

## 8. Проверка (raw-вывод — в PR)

- `python -m pytest src/rob_box_perception/test/unit -q -p no:cacheprovider --no-cov --ignore=src/rob_box_perception/test/unit/core`
- `python tools/architecture_audit.py …` + `tools/architecture_findings.py …` до и после (сравнение по ключам `(type, subject)`).
- `flake8 --max-line-length=99` и `pydocstyle --convention=pep257` (игноры ament) по изменённым файлам.
- **Не проверено:** colcon build `rob_box_perception_msgs` с новым `.msg`, тесты под настоящим `rclpy`, запуск на роботе (топики, stamp, `frame_id`, задержка, CPU), доставка через Zenoh до Main Pi.
