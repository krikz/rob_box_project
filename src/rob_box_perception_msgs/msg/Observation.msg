# Observation.msg — одно наблюдение датчика (ADR-0130 §2.1/§2.2, ADR-0138).
#
# «Датчик X в момент t видел детекцию класса C вот здесь, с такой
# погрешностью». Мгновенный факт, без памяти и без личности.
#
# Инварианты (ADR-0130 §5.1, проверяются test_observation_*):
#   * НЕ несёт имени человека (п.1). Поля display_name нет и не будет.
#   * НЕ выдаёт собственный id личности (п.5): candidate_id — ссылка на
#     биометрическую улику в хранилище примет, решение «кто это» принимает
#     не Observation.
#   * Отказ глубины виден (п.7): position_valid=false + position_status,
#     а не нули/тишина.
#
# Публикует: VisionHailoNode (контейнеры vision-hailo и vision-face) на
# /perception/observations. Потребитель — трекер (ADR-0130 этап 2).
# Заменяет VisionEvent (deprecated) для потребителей геометрии.

# stamp    = время КАДРА, на котором сделана детекция (header кадра камеры),
#            а не время публикации (ADR-0130 §2.5).
# frame_id = optical frame камеры из header кадра (REP-103 optical:
#            z вперёд, x вправо, y вниз). В base_link/map переводит
#            потребитель (трекер на Main Pi, ADR-0130 §2.11), не Vision Pi.
std_msgs/Header header

# Id датчика = имя адаптера шва «Взгляд» ("oak_d", "ceiling_camera").
string source

# Класс ДЕТЕКЦИИ по словарю детектора: "person" (YOLOv8n) | "face"
# (RetinaFace). Прочие классы не публикуются (ADR-0130 §2.2).
string class_name

# Уверенность детектора [0, 1].
float32 confidence

# Bbox в ПИКСЕЛЯХ исходного кадра (после снятия letterbox), центр + размер.
# image_width/image_height — размер этого кадра; нормированный bbox
# = bbox_*_px / image_*.
float32 bbox_cx_px
float32 bbox_cy_px
float32 bbox_w_px
float32 bbox_h_px
uint32 image_width
uint32 image_height

# 3D-точка центра детекции в optical frame камеры (header.frame_id), метры.
# NaN во всех трёх координатах, когда position_valid=false.
geometry_msgs/Point position

# Расстояние от оптического центра камеры до position, метры.
# -1.0, когда position_valid=false.
float32 distance_m

# Честный статус глубины (ADR-0018 capability-honest). position_valid=true
# только при position_status="ok". Словарь — observation_geometry.STATUS_*:
#   ok | no_depth_stream | depth_out_of_sync | depth_size_mismatch |
#   depth_encoding_unsupported | no_camera_info | camera_info_size_mismatch |
#   bbox_outside_frame | depth_invalid
bool position_valid
string position_status

# Разброс глубины (стандартное отклонение, м) по валидным пикселям
# центральной части бокса после отсечения выбросов. Оценка разброса
# сцены внутри бокса, а НЕ откалиброванная ковариация. -1.0 = нет оценки.
float32 position_stddev_m

# Биометрическая улика без имени: id записи в хранилище примет
# (сейчас FaceStore person_id; после этапа 3 ADR-0130 — Acquaintance.id).
# Пусто = улики нет (person-bbox, лицо без эмбеддинга, нет совпадения).
string candidate_id

# Сходство с candidate_id [0, 1]; -1.0 = не посчитано на этом кадре.
float32 candidate_similarity
