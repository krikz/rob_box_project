# История попыток ездить по стереокамере (OAK-D) и лидару — что пробовали и почему не вышло

> Дата: 2026-11-04. Статус: разведка по git/issue/PR, код не менялся, на робота не ходили (Main Pi не опрашивался).
> Запрос товарища Шифу: «стоит стерео-камера, цель была ездить по ней и по лидару, но не вышло — глянуть, поднять историю».
> Метки достоверности: **[измерено]** — цифра из коммита/issue/замера, снятая на роботе; **[код]** — прочитано в репо;
> **[догадка]** — вывод без проверки. Хэши — из `git log` на develop `e5b524929`.

## 1. Коротко

- **Ездить по стерео-камере (визуальная одометрия) не пробовали по-настоящему.** `visual_odometry:=true` стоял в compose два дня (08–10.10.2025, коммиты `7763b7935` -> `0ab57ae12`) и ни разу не работал на железе устойчиво. `rgbd_odometry` / `stereo_odometry` / `subscribe_stereo` в рабочем конфиге не включались никогда [код, `git log -S`].
- **Что пробовали реально — RGB-D (цвет + глубина OAK-D) как добавка к лидару в rtabmap**: два захода — октябрь 2025 (4 дня) и март 2026 (8 дней, `feature/rtabmap-rgbd-lidar-3d`, PR #646). Оба откачены в «только лидар».
- **Сейчас ездим на лидаре:** ICP-одометрия по `/scan`, guess от колёсной одометрии; rtabmap `Grid/Sensor=0`, глубину не читает; Nav2 local/global costmap берут только `/scan` [код: `docker/main/docker-compose.yaml:143-163`, `docker/main/config/rtabmap/rtabmap.yaml:21-27`, `nav2_params.yaml:157,196`].
- **Главные причины, по которым RGB-D не взлетел:** (1) нагрузка и канал Pi→Pi по Zenoh (rtabmap 87 % ядра, 55 МБ/с на eth0, задержка 0.65–1.1 с) [измерено, коммит `3b74ce532`]; (2) rtabmap с RGB-D рвал Zenoh-транспорт и портил базу (`519752841`, `50d2f73ed`) [измерено по логу краша]; (3) TF/timestamp-цепочка камеры не сошлась (серия алиасов фреймов, `approx_sync`, рассинхрон 15/10 Гц); (4) на живой машине сыпались отказы loop closure и «Not found word» (issue #688, #699, #675) [измерено].
- **Лидар-ICP тоже не идеален:** в коридорах с ровными стенами «думает, что стоит» (14–15.12.2025), чинилось подбором параметров, а не заменой датчика [измерено/по коммитам].
- **Для сима стоит проверить гипотезу «причина — нагрузка/канал/синхронизация, а не алгоритм»**: в Gazebo всё на одной машине без Zenoh и без Pi (§6).

## 2. Железо и текущее состояние

| Что | Значение | Источник |
|---|---|---|
| Лидар | LSLIDAR N10 2D, высота ~0.477 м (из коммита `b73943fae`: «lidar at 0.477m height misses low obstacles (<47cm)») | [код/коммит] |
| Камера | OAK-D Lite на **Vision Pi**, Main Pi — rtabmap/Nav2; между ними Zenoh по eth0 | [код] |
| Режим камеры | `i_pipeline_type: RGBD`, stereo/depth 1280x720, `i_fps: 5`, `i_align_depth: true`, `i_rs_compat: true`, RGB `low_bandwidth: true`, depth `low_bandwidth: false` | `docker/vision/config/oak-d/oak_d_config.yaml` |
| Что читает глубину сейчас | только восприятие v2 на Vision Pi (`Observation.msg`, ADR-0138) и Quest/Telegram через `compressedDepth`; **навигация глубину не читает** | ADR-0138 §1, `docker-compose.yaml` |
| rtabmap | `depth:=false`, `subscribe_rgbd:=false`, `icp_odometry:=true`, `odom_guess_frame_id:=odom`, `Reg/Strategy=1`, `Grid/Sensor=0`, `Reg/Force3DoF` | `docker/main/docker-compose.yaml:149-184` |
| IMU | rtabmap ждёт `/camera/imu/data`, камера публикует `/camera/camera/imu` — IMU в rtabmap **не доходит** (issue #3106 закрыт, но в compose всё ещё `imu_topic:=/camera/imu/data`, `wait_imu_to_init:=false`) | [код] |
| Колёсная одометрия | `open_loop: true`, колёса отстают от команды на ~20 % (#4004) — поэтому ICP по лидару важен | `controller_manager.yaml`, #4004, док Gazebo (PR #4303) |
| Нагрузка Vision Pi | `oak-d` ~72 % ядра (67–76 %), весь Pi 51 % при load 8–11 на 4 ядра, троттлинга нет [измерено 03.11, `docs/research/2026-11-04-perception-load-measure.md` на ветке `docs/2026-11-04-perception-load`]. Main Pi **не замерен** |
| Расхождение в доках | `docs/architecture/ICP_ODOMETRY.md` называет лидар «C16», в рабочих конфигах — N10 | [код] |

## 3. Хронология

### Эпизод A. Октябрь 2025 — RGB-D + visual odometry на Pi (08–10.10.2025)

| Дата | Коммит | Что |
|---|---|---|
| 04–07.10 | `4476efb02`, `4a9dbcce6`, `2f1cdd1c8` | первые коммиты oakd / rtabmap |
| 08.10 | `7763b7935` | «Оптимизация OAK-D-Lite + RTAB-Map для Raspberry Pi»: RGB 720p, depth 640x400, 5 Гц, compressed (JPEG/PNG), compose с `rgbd_sync:=true`, `visual_odometry:=true`. Ожидания в сообщении: CPU 85–95 % -> 30–45 %, сеть 80–100 -> 8–15 Мбит/с — **это прогноз, не замер** |
| 08.10 | `cbebde97b`, `b62fcb213`, `07881740f` | камера молчит: lazy publisher ждёт подписчиков на базовые топики, а все подписаны на compressed |
| 08–09.10 | `ea300afe3`, `451d5d621`, `e98b8935d` | compressed в rgbd_sync, пути zenoh_cpp_vendor для rtabmap, переход на `rmw_zenohd` |
| 09.10 | `af224000e` | «Emergency memory optimization for Vision Pi»: **7.2 ГБ из 7.75 ГБ (92 %)**, лимит контейнера 6 ГБ [измерено в сообщении коммита] |
| 10.10 | `0f2a4f07b` | «Vision Pi: OAK-D AprilTag-only mode» — rtabmap/SLAM снят с Vision Pi |
| 10.10 | `0ab57ae12`, `ff76c95c8`, `3c8bb5b4f` | rtabmap на Main Pi переключён на **лидар + AprilTag**, `icp_odometry=true`, `visual_odometry=false`; долго воевали с тем, что `rtabmap.launch.py` по умолчанию `depth=true` и подписывается на камеру |
| 11.10 | `bb3530721` | аналитическая записка про RGB-D SLAM — «future reference»; рекомендует RealSense D435i / ZED 2 |

**Чем кончилось:** RGB-D снят за 2–4 дня. **Причины:** память Vision Pi (92 %), канал/CPU [измерено только по памяти; по CPU/сети — ожидания, не замеры], плюс возня с настройкой launch/image_transport/lazy publisher. Явного замера «visual odometry не держит трек» нет — **её не успели ни запустить как следует, ни померить**.

### Эпизод B. Ноябрь–декабрь 2025 — геометрия и ICP по лидару (без камеры в SLAM)

| Дата | Коммит | Что |
|---|---|---|
| 19–21.11 | `c2cda4457`, `d718a150d`, `6354bb905`, `2cfcf384d`, `1069ca7ac` | URDF: позиции лидара и OAK-D из Fusion 360, повороты +90°, ориентация камеры |
| 20.11 | `8c1f3c80d`, `0b923241e`, `1b3bc207a` | `frame_id` rtabmap: `base_link` -> `base_footprint` |
| 20.11 | `028772369` | `Icp/CorrespondenceRatio` 0.4 -> 0.05 («fix odometry with low point matches») |
| 14.12 | `56a001574` / `1cd9c432e` / `e968bb27f` | флип-флоп: ICP on -> off -> on; loop closure off («2D lidar only mode») |
| 15.12 | `3b872189b` | «ICP odometry was drifting in uniform corridors, robot thought it stayed at origin while moving» -> колёсная одометрия [измерено на роботе, по тексту коммита] |
| 15.12 | `8aa0f2296`, `308974ec7`, `eeb3b98d4`, `466ba6317`, `337b8f028` | ещё раз off/on, `odom_guess_frame_id`, IMU-топик, «disable loop closure and visual features … without camera» |
| 16.12 | `90b95a0ae`, `3511753a5`, `ffcff9761`, `2d6b0b65f`, `dda7fcb7e` | ICP остаётся; `RGBD/OptimizeMaxError` 3 -> 5, `transform_tolerance` под задержку ICP 30–125 мс, `approx_sync`, DWB без RotationShim. Док: `docs/fixes/NAV2_NAVIGATION_TUNING_2025-12-16.md` |
| 15.12 | `d82571f6f` | `docs/architecture/ICP_ODOMETRY.md` |

**Чем кончилось:** остановились на **ICP-одометрии по лидару с guess от колёс** — рабочая схема до сегодняшнего дня. **Проблемы ICP** [измерено по текстам коммитов/дока]: дрейф в однородных коридорах, задержка 30–125 мс, `std ~7–10 мм`, отказ loop closure при ошибке графа > порога.

### Эпизод C. Январь 2026 — ROADMAP

`55a571dcd` (13.01.2026): в `ROADMAP_SIMPLE.md` упомянуты ORB-SLAM3 и 3D-локализация как планы. Кода нет [код].

### Эпизод D. Март 2026 — RGB-D + лидар, `feature/rtabmap-rgbd-lidar-3d` (02–10.03.2026)

Цель, из коммита `b73943fae` (02.03): лидар на 0.477 м не видит низкое (<47 см) и «мягкое» (диваны) — «OAK-D depth camera covers the blind zone». План: ICP-одометрия остаётся основной, `Grid/Sensor=2` (лидар + глубина), `subscribe_rgbd:=true`.

| Дата | Коммит | Что |
|---|---|---|
| 02.03 | `b73943fae` | Vision Pi: `rtabmap-sync` (`rgbd_sync`) шлёт `/rgbd_image` в Main Pi; `Grid/Sensor 0 -> 2` |
| 02–03.03 | `cbdd5dfb9`, `c01e6af26`, `a54bafa53`, `e761edf1e`, `681ebd69d`, `b8f6df3ef` и др. | depthai v3 -> v2.12.2-humble, `i_rs_compat`, двойной namespace `/camera/camera/`, переименования rgb->color, stereo->depth — **десяток коммитов на имена топиков и фреймов**; `93961c14f` alias TF `camera_rgb_camera_optical_frame -> camera_color_optical_frame` |
| 03.03 | `e59c48021` | сырой `RGBDImage` по Zenoh = **~89 МБ/с**; переход на compressed (~8–13 МБ/с) [измерено] |
| 03.03 | `b0857880e`, `4d69fdaa7`..`cdd20f4de` | «200x speedup» (WM 500 узлов, `BinDataKept=false`, `Kp/MaxFeatures=200`); `Grid/Sensor=2` не применялся — `rtabmap.launch.py` перезаписывает, пришлось патчить launch на старте (`patch_rtabmap_launch.py`) |
| 05.03 | `3b74ce532` | замер до правки: **eth0 RX 23 + TX 31 МБ/с = 55 МБ/с; rtabmap CPU 87 %; задержка avg 0.65 с, max 1.10 с; RGB 15 Гц vs depth 10 Гц — рассинхрон, `approx_sync` теряет кадры** [измерено]; fps 15 -> 10 |
| 05.03 | `2a3a30203` | GTSAM падал `IndeterminantLinearSystemException` на узле 5235 (граф с битым VWDictionary) -> g2o + robust |
| 06.03 | `519752841`, `50d2f73ed`, `7e1e03fbe`, `4b097c8ea`, `fe691ba4e` | **loop closure публикует mapData ~200–400 МБ, Zenoh-транспорт закрывается и убивает ВСЕ топики (камера, лидар, цели Nav2) + повреждение БД; в логе краша WM=203** [измерено]. Лечили: MaxNodes 200 -> 100, `congestion_control=drop`, буферы 16 МБ, voxel 0.1 -> 0.3, fps 10 -> 5 |
| 09.03 | `2a35c15b5`, `e2faa6613`, `4c0f362f2`, `d640db095`, `3218f5575`, `c27c10d56`, `090586dfe` | ещё TF-алиас `camera_stereo_camera_frame -> camera_depth_frame`, возврат к presynced `/rgbd_image`; в local costmap добавлен `/rtabmap/cloud_obstacles` и depth->laserscan; «relax localization links», «stop reanchoring map on startup» |
| 09.03 | issues #675, #688, #699 | живые логи: `Rejecting localization … max graph error ratio 29.4 (RGBD/OptimizeMaxError 5)`; `VWDictionary::addWordRef Not found word 77723 (dict size=0)`; `OptimizerG2O hessian not valid` [измерено] |
| 10.03 | `b99309fd1`, `bc93f9f6b`, `cb409c971` | **откат: `Grid/Sensor=0`, `depth:=false`, `subscribe_rgbd:=false`; убраны запуск depth->laserscan и TF-алиасы; тесты переделаны под lidar-only.** Комментарий в compose: «OAK-D intentionally excluded from Main Pi mapping until TF/timestamp path is stable again» |
| 10.03 | `8ac11cd03`, `ebcea5cdf`, `76c1899c0`+Revert | `rtabmap-sync` удалён из Vision compose; камера перенастроена под AprilTag (720p, ISP) |
| 30.09 | PR #3209 | из образа Nav2 убран `depthimage-to-laserscan` (хвост отката, тест давно требовал этого) |

**Чем кончилось:** RGB-D+лидар не стал «стабильным» за 8 дней, и Шифу/разработчик откатил на лидар. Комментарий отката называет причину: **TF/timestamp-путь**. Но реальные поломки были шире (таблица выше).

### Эпизод E. Сентябрь 2026 — глубина OAK-D нужна восприятию, не навигации

- PR #3184 (29.09): `depth.i_low_bandwidth: false` — low-bw глубина завышена ×2.0–2.5 (1 м -> 2.52 м, 4 м -> 9.52 м) [измерено рулеткой]. Причина — [по исходникам depthai-ros v2.12.2-humble]: при `low_bandwidth` хост переводит 8-битный disparity в глубину по `fx` RGB-камеры 1280x720, а disparity посчитан в пикселях моно-сенсора.
- **Следствие для навигации:** в **марте 2026 глубина, шедшая в rtabmap с `i_low_bandwidth: true` (и тогда RGB-D шёл 640x400), могла быть неверной по масштабу — Grid/Sensor=2 строил бы препятствия не там [догадка: в марте драйвер и конфиг были другими (640x400, `depth.i_low_bandwidth` не читался из тех же секций), поэтому прямой связи не доказано].** Это — самый важный непроверенный кандидат на «причину, которую никто не видел».
- ADR-0138: постоянного потребителя raw-глубины, кроме нового восприятия, в репо нет.

## 4. Причины «не вышло» по эпизодам

| # | Причина | Эпизод | Достоверность |
|---|---|---|---|
| 1 | Память Vision Pi 92 % при RGB-D стеке | A | [измерено, `af224000e`] |
| 2 | Канал Pi->Pi: сырые RGB-D = 89 МБ/с; после compressed 55 МБ/с на eth0 суммарно | A, D | [измерено, `e59c48021`, `3b74ce532`] |
| 3 | CPU rtabmap 87 %, задержка 0.65–1.1 с | D | [измерено, `3b74ce532`] |
| 4 | Рассинхрон потоков: RGB 15 Гц vs depth 10 Гц -> `approx_sync` теряет кадры; затем всё в 5 Гц | D | [измерено, `3b74ce532`] |
| 5 | Zenoh-транспорт закрывается на огромном mapData, гасит все топики, портит БД | D | [измерено, лог краша 06.03] |
| 6 | TF-цепочка камеры: двойной namespace, `i_rs_compat`, три TF-алиаса, camera_link/optical-frames; в сумме 15+ коммитов правок имён | D | [код/коммиты]; точная ошибка TF в логах не сохранена |
| 7 | Loop closure отвергается (ошибка графа 29x), `Not found word`, GTSAM падает на битом словаре | D | [измерено, #688, #699, #675, `2a3a30203`] |
| 8 | Глубина OAK-D с `low_bandwidth` неверна ×2–2.5 | D (возможно) | [измерено в сентябре; связь с мартом — догадка] |
| 9 | IMU камеры не доходит до rtabmap (топик не совпадает) | A–сейчас | [код, #3106 закрыт, но compose прежний] |
| 10 | Однородные коридоры: ICP по 2D-лидару «залипает» | B | [измерено по коммиту `3b872189b`] |
| 11 | Колёсная одометрия `open_loop` и -20 % по скорости (#4004) — guess для ICP плохой | сейчас | [измерено, #4004] |
| 12 | Нагрузка Vision Pi: `oak-d` ~72 % ядра уже при выключенном SLAM; добавлять тяжёлое некуда | сейчас | [измерено 03.11] |

**Чего не знаем:** (а) измерения Main Pi (load, CPU rtabmap в лидарном режиме) — нет; (б) логов TF-ошибок марта — нет, в issue (#675…#726) только предупреждения rtabmap; (в) почему именно Шифу решил откатить — только текст коммита; (г) ни разу не запускали `stereo_odometry`/`rgbd_odometry` и не мерили их на этих Pi.

## 5. Что показывает история в целом

1. **Ни одна попытка не упёрлась в «алгоритм не работает».** Все упирались в инженерные потолки: память/CPU Pi, канал Zenoh между Pi, синхронизация времени/TF, размер mapData.
2. **Схема «камера на одном Pi, SLAM на другом» — главная слабость.** RGB-D идёт через сеть (55–89 МБ/с), rtabmap на Main Pi, камера и лидар в разных часовых доменах. Это ровно тот шов, который сим уберёт.
3. **Мотивация (слепая зона лидара ниже 47 см и «мягкие» препятствия) осталась** — её ничем не закрыли; сейчас Nav2 видит только 2D `/scan`.
4. **Визуальная одометрия по стерео — не опробована.** Выводов «стерео не годится» сделать нельзя.

## 6. Что проверить в симуляции Gazebo (план PR #4303)

Гипотеза: «стерео/глубина не сломана, ломали Pi, Zenoh и синхронизация». Сим эти факторы выключает. Что мерить (в порядке ценности):

1. **RGB-D камера в симе** (`gz` `rgbd_camera`, 640x400 и 1280x720, 5/10/15 Гц, **с теми же** `camera_info`/фреймами, что у OAK-D Lite: baseline ~7.5 см, FOV, высота и наклон из `rob_box.xacro`).
2. **Режимы rtabmap** на одном и том же маршруте и мире (`office_cpr`, `coridor.world`, а также мир «ровный коридор» и мир «с низкими препятствиями <47 см и диванами»):
   - A: «только лидар» (текущий: `icp_odometry`, `Grid/Sensor=0`);
   - B: «лидар + depth» (`subscribe_depth:=true`, `Grid/Sensor=2`);
   - C: «depth-only visual odometry» (`rgbd_odometry`, лидар только в grid) — **ни разу не пробовали**;
   - D: `stereo_odometry` на левой/правой камерах (в OAK-D Lite есть mono pair; ни разу не пробовали).
3. **Метрики:** ошибка позы к ground truth симулятора (APE/RPE), доля отвергнутых loop closure, число `Rejecting localization`, время цикла rtabmap, CPU, задержка `odom->base_footprint`, процент успешных Nav2-целей, столкновения с препятствиями **ниже уровня лидара**.
4. **Стресс-тесты, которые воспроизводят потолки Pi:** (i) fps камеры 5/10/15 и **рассинхрон** RGB/depth с задержкой, (ii) лимит CPU контейнера rtabmap (`--cpus`), (iii) ограничение канала между «Vision» и «Main» контейнерами (`tc netem`/`tbf`) при использовании Zenoh, (iv) тот же глубокий квантованный disparity с ошибкой масштаба ×2.3 — **проверить, портит ли неверный масштаб глубины карту**.
5. **Проверка нашей догадки о причине #8** (масштаб глубины) можно сделать **только** симом с контролируемым искажением, на роботе уже лежит корректная конфигурация.
6. **Смежное, вытекающее из истории:** IMU в rtabmap не доходит (#3106) — в симе включить IMU и сравнить с/без; `open_loop`+дефицит -20 % скорости (#4004) — сравнить «честная колёсная одометрия» vs «искажённая как на роботе» (уже в плане #4303 §2.4).
7. **Критерий входа обратно на робота:** режим B/C/D даёт заметно меньше столкновений с низкими препятствиями **и** укладывается в потолок CPU/канала Main Pi (нужен замер Main Pi — его нет).

## 7. Рекомендация

1. **Не возвращать RGB-D в rtabmap на роботе до результатов сима.** Все прошлые провалы — потолки Pi/сети; без замера Main Pi вернуть нечем.
2. **Замерить Main Pi** (load, `docker stats rtabmap`, eth0) — у Шифу нет ключа/пароля для агента; это блокер и для этой темы, и для восприятия v2 (Q13).
3. **В симе первым делом сделать режим B (лидар+depth) и отдельно режим C/D** (визуальная/стерео-одометрия) — это то, что **никогда не пробовали**.
4. **Дёшево закрыть слепую зону без rtabmap:** в local costmap добавить `PointCloud2`/`DepthImage` слой из OAK-D (то, что делали в `4c0f362f2`), но только после того как сим покажет приемлемую нагрузку; глубина уже идёт с `low_bandwidth: false`.
5. **Исправить `imu_topic`** (#3106: compose всё ещё `/camera/imu/data`) или убрать IMU — иначе любое сравнение будет нечестным.
6. Вопрос Шифу: откат 10.03 был решением по нагрузке или по нестабильности? (в коммите только «until TF/timestamp path is stable again»).

## Источники

- Коммиты и PR перечислены в таблицах; сообщения коммитов: `git show <hash>`.
- `docker/main/docker-compose.yaml`, `docker/main/config/rtabmap/rtabmap.yaml`, `docker/main/config/nav2/nav2_params.yaml`, `docker/vision/config/oak-d/oak_d_config.yaml`.
- `docs/architecture/ICP_ODOMETRY.md`, `docs/fixes/NAV2_NAVIGATION_TUNING_2025-12-16.md`, `docs/adr/0138-observation-msg-depth-stage1.md`.
- PR #646 (feature/rtabmap-rgbd-lidar-3d), #3184, #3209, #4303; issues #3106, #4004, #675, #688, #699.
- `docs/research/2026-11-04-perception-load-measure.md` (ветка `docs/2026-11-04-perception-load`), `docs/research/2026-11-04-gazebo-sim-navigation.md` (ветка `docs/2026-11-04-gazebo-sim-navigation`).
