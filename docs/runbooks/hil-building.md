# Прогон «большая карта здания» в HIL (#4468)

Как одной командой снять в HIL карту этажа `building`, проехать по кабинетам по сохранённой карте и проверить
локализацию: рестарт стека, старты из случайных точек, «похищенный робот», при желании — голос. Мир этажа —
`rob_box_sim.building_floor` (шаг 1, PR #4563), включение HIL — `scripts/hil/hil.sh` (runbook `hil-video.md` §2).

Метки: **[код]** — по исходникам, **[оценка]** — прикинуто по плану, на стенде не мерено. На 09.11.2026 этот
прогон **ни разу не запускался**: мир `building` в Gazebo ещё не грузили (RTF и `/scan` не проверены).

## 0. Что меряем и кто решает

Решения принимает код `rob_box_sim.building_run`, скрипт только исполняет шаги и снимает raw [код].

| Блок | Что делаем | Вердикт (`VERDICT #4468 <блок> …`) |
|---|---|---|
| `map` | объезд 24 точек этажа ближайшим соседом по коридору от холла и назад в холл, rtabmap в mapping | доехал до каждой точки |
| `registry` | `start_mapping` / `finish_mapping` с именем «здание» и `save_waypoint` на каждой точке маршрута — тулы напрямую, мимо LLM | каждый тул ответил `success` |
| `restart` | рестарт стека (`hil.sh restart rtabmap`, режим локализации) после картографии и в конце маршрута | нашёл себя |
| `route` | те же 24 точки по сохранённой карте: время, стопы защиты, восстановления | доехал до каждой точки |
| `drift` | ошибка позы истина Gazebo против `map` на всём маршруте: средняя, p95, максимум | максимум ≤ 1 м |
| `global_loc` | N стартов: телепорт в случайную свободную позу → рестарт стека → поворот 360° → к ближайшей точке | нашёл себя в каждом старте |
| `similar` | скачки оценки по всем отрезкам, отдельно в одинаковом восточном крыле | ни одного ложного скачка |
| `kidnap` | посреди поездки телепорт: на два кабинета в одинаковом крыле и из западного крыла в холл | нашёл себя после телепорта |
| `voice` | (выкл.) реплика «поезжай в деканат» через `inject.sh`, доезд по истине | доехал |

Определения [код, пороги — предложение, не решение Шифу]:

- **доехал** — Nav2 вернул SUCCEEDED, и истина Gazebo не дальше 0.5 м от точки. Nav2 «дошёл», а робот дальше —
  `FALSE_SUCCESS`, это FAIL, а не успех.
- **нашёл себя** — после события (конец рестарта или телепорт) ошибка оценки держится в 0.35 м и 10° хотя бы 5 с,
  и это случилось не позже 120 с. Позже — `LATE`; не сошлось и в конце мимо больше 1 м — `WRONG` (уверенно «не там»);
  не сошлось — `NOT_FOUND`; оценки не было — `NO_ESTIMATE`.
- **ложный скачок** — за шаг записи (10 Гц) оценка сдвинулась сверх движения истины больше 0.5 м, и после этого
  ошибка больше 1 м и выросла. Телепорт истины и первая оценка после рестарта скачком не считаются.

Числа «нашёл себя», скачков и дрейфа считает `rob_box_sim.loc_metrics` стенда #4346 (**PR #4568**). Пока он не
влит, `run_building.sh map|loc|all|full` отказывает с сообщением о #4568, а тесты `test_sim_building_loc.py`
пропускаются. Порядок мёржа: #4568, потом этот прогон.

## 1. Перед запуском

1. Замок стенда — свой: первое слово `stand.lock` = `BUILDING_OWNER`. Скрипт замок не берёт, только сверяет
   (как `hil-checks`). Деплой не идёт, робот на кирпичах.
2. Ключи в `authorized_keys` Main Pi, Vision Pi, Sound Pi и песочницы. Пароли нельзя.
3. Образ `rob_box_sim:hil` на песочнице есть (тот же, что для `hil-video`). Новые модули `rob_box_sim`
   (план, вердикты, рекордер) едут копией из чекаута (`pylib`), образ пересобирать не нужно.
4. Чекаут оператора — develop с влитым #4568 (`src/rob_box_sim/rob_box_sim/loc_metrics.py` есть).
5. На машине оператора — Python 3.11 с `numpy`, `PyYAML`, `Pillow` (анимации).

## 2. Запуск

```bash
export MSYS_NO_PATHCONV=1
scripts/hil/building/run_building.sh plan                         # порядок, старты, похищения, оценка; стенд не нужен
BUILDING_OWNER=<имя> scripts/hil/building/run_building.sh full    # всё: prep → hil.sh on → прогон → hil.sh off
```

`full` делает [код]:

1. `prep` — копия частей репо в `/home/ros2/rob_box_hilbuilding` на песочнице: `docker/sim`, конфиги Main,
   `docker/common`, миры, `route.py`, `building_step.py`, `building_remote.sh`, `pylib/rob_box_sim`; CRLF правится.
   Без Depot и без `patch_compose.py`: мир — `world:=/sim_worlds/building.sdf` из обычного `compose.hil.yaml`.
2. `hil.sh on` с `HIL_SANDBOX_REPO=/home/ros2/rob_box_hilbuilding`, `HIL_SIM_WORLD=building`,
   `HIL_RTABMAP_MODE=mapping` и свежим `HIL_MAPS_DIR=/home/ros2/rob_box_hil/maps_building_<MMDD_HHMM>`.
3. `all` — блоки `map`, `route`, `loc`, `voice` (если `BUILDING_VOICE=1`) и отчёт.
4. `hil.sh off` — всегда, и при ошибке тоже (trap).

По частям (HIL включён руками, как в `hil-video.md` §2, с теми же переменными):

```bash
scripts/hil/building/run_building.sh prep
HIL_SANDBOX_REPO=/home/ros2/rob_box_hilbuilding HIL_SIM_WORLD=building HIL_RTABMAP_MODE=mapping \
  HIL_MAPS_DIR=/home/ros2/rob_box_hil/maps_building_$(date +%m%d_%H%M) scripts/hil/hil.sh on
export BUILDING_OWNER=<имя> BUILDING_OUT=$TMPDIR/hil_building/run1     # один каталог на все блоки
scripts/hil/building/run_building.sh map      # затем route, loc, voice — каждый отдельно
scripts/hil/building/run_building.sh stop     # остановить рекордер поз, скачать raw
scripts/hil/building/run_building.sh report "$BUILDING_OUT"
scripts/hil/hil.sh off
```

Повторить локализацию без новой картографии: `hil.sh on` с **тем же** `HIL_MAPS_DIR` и
`HIL_RTABMAP_MODE=localization` (режим mapping стирает базу на старте), затем `run_building.sh loc`.

Остановить шаг: `ssh ros2@10.1.1.250 'docker exec rob-box-hil-gazebo pkill -INT -f /building_step.py'`. Не
`pkill -f run_building` через ssh: шаблон совпадёт с самой ssh-командой.

## 3. Что делает каждый блок [код]

- **Шаг** — один вызов `building_step.py` в контейнере `rob-box-hil-gazebo`: `goto`, `mark`, `teleport`, `probe`,
  `kidnap`, `arrive`. Запускается в фоне на песочнице (`building_remote.sh start`), обрыв ssh его не убивает;
  оркестр ждёт файл `out/building/steps/<метка>.rc`. Лог шага — строки `route.py` (`GOAL_START/GOAL_END`, `TRACK`,
  `EM_ON`) и `EVENT <вид> sim=…`. Отрезки прогона размечены `EVENT SEG_START name= kind=` / `SEG_END`.
- **Рекордер поз** — `rob_box_sim.map_recorder` в контейнере: истина Gazebo и `map → base_footprint` по 10 Гц
  sim time. На картографии пишет и снимки `/rtabmap/map` (`maprec_map/maps`), дальше — только позы
  (`maprec_loc/poses.csv`): снимки большой карты за часы — сотни МБ [оценка].
- **map.** `start_mapping(«здание»)` → объезд → `finish_mapping(«здание»)` → рестарт стека в локализацию
  (отрезок `restart_after_map`) → поворот 360°. Список файлов карты на Main Pi — `map_files.txt`.
- **Рестарт стека** — `hil.sh restart rtabmap` с `HIL_RTABMAP_MODE=localization`: `docker stop --time 60`
  (rtabmap дописывает базу на остановке), затем `compose up --force-recreate` той же командой, что у `on`;
  каталог карты берётся из замка HIL. Что перезапускать — `BUILDING_RESTART` (например `"rtabmap nav2"`).
  `hil.sh restart rtabmap` в режиме mapping (умолчание `hil.conf`) отказывает: он стёр бы карту.
- **route.** Те же точки по одной; после каждой — `save_waypoint` с русским названием точки («деканат»):
  сохраняется **оценка** позы в момент доезда, как у человека, который говорит «запомни это место».
- **loc.** Рестарт на месте (конец маршрута, отрезок `restart_here`), потом `BUILDING_STARTS` стартов
  (`building_run.random_starts`: свободные позы, зазор корпуса ≥ 0.35 м, разнос ≥ 6 м, попеременно запад и
  одинаковое крыло, сид `BUILDING_SEED`), потом похищения `kidnap_similar` (из `room_203` к `room_210`, через
  15 с — на два кабинета вдоль крыла) и `kidnap_far` (из деканата к библиотеке, через 15 с — в холл). Телепорт —
  `ign service /world/building/set_pose` для модели `rob_box`, шаг ждёт скачка истины.
- **voice** (`BUILDING_VOICE=1`). Реплики из `BUILDING_VOICE_TARGETS` (`имя=фраза|…`) уходят в диалог через
  `scripts/robot_dev/inject.sh`; шаг `arrive` ждёт до 300 с, пока истина окажется у точки. Робот не поехал
  (модель не позвала тул) — `NOT_TRIGGERED`, а не PASS.

## 4. Сколько длится [оценка]

`run_building.sh plan` (6 стартов): картография ~45 мин, маршрут ~45 мин, локализация ~49 мин, всего ~2.3 ч
стены; плюс `prep` ~2 мин, `hil.sh on/off` по 2–3 мин, голос до 5 мин на фразу. Расчёт: объезд 223 м по оценке
пути через коридор, 0.10 м/с, 15 с на цель, рестарт 90 с, поворот 45 с, RTF 1.0 без камер (замер 09.11 на Depot).
Шаги-метки добавляют по 3–10 с на ssh и старт rclpy. На здании не мерено.

## 5. Переменные (умолчания — в шапке `run_building.sh`)

| Переменная | Зачем |
|---|---|
| `BUILDING_OWNER`, `STAND_LOCK` | чей стенд; без совпадения — отказ с кодом 3 |
| `BUILDING_OUT` | каталог raw на машине оператора (`$TMPDIR/hil_building/<UTC>`) |
| `BUILDING_SANDBOX_DIR` | копия частей репо на песочнице (`/home/ros2/rob_box_hilbuilding`) |
| `BUILDING_STARTS`, `BUILDING_SEED` | число случайных стартов (6) и сид (4468) |
| `BUILDING_REGISTRY` | 1 — карта и точки в HIL-реестре тулами; 0 — без реестра |
| `BUILDING_MAP_NAME` | имя карты в реестре («здание») |
| `BUILDING_RESTART` | сервисы рестарта стека (`rtabmap`) |
| `BUILDING_VOICE`, `BUILDING_VOICE_TARGETS` | голосовой сценарий (0) и пары `точка=фраза` через `\|` |
| `BUILDING_STEP_TIMEOUT_S` | потолок одного шага, с стены (7200) |
| `HIL_RESTART_STOP_S` | сколько ждать `docker stop` при рестарте (60, `hil.conf`) |

## 6. Что приложить в #4468

Каталог `BUILDING_OUT`:

```
verdicts.txt        VERDICT - preflight … и VERDICT #4468 <блок> <статус>: … по одной на блок
report.md           те же строки и таблица отрезков
anim/<отрезок>.gif  вид сверху: зелёный — истина Gazebo, красный — оценка rtabmap, полоса ошибки внизу
steps/*.log         raw шагов (GOAL_END, EVENT, TRACK, EM_ON)
maprec_*/poses.csv  t, истина, оценка; maprec_map/maps — снимки карты
tools.log           ответы тулов реестра (JSON /mcp/result)
restart.log         вывод hil.sh restart; map_files.txt — файлы карты на Main Pi
preflight/          stand_lock, hil_status, inspect, clock, world (аргументы gazebo, режим rtabmap)
run.log             ход прогона
```

В комментарий: строки `verdicts.txt`, `git rev-parse HEAD`, тег образа сима и образов Main, `plan`-вывод (порядок,
старты), анимации `route`, одного `start_*` в одинаковом крыле и обоих похищений, CAN-проверку «команд на VESC
нет» (`hil-video.md` §6). Закрывать карточку только по PASS; `NOT_TRIGGERED` — «не проверено», так и писать.

## 7. Ограничения [код]

- Пороги вердиктов (0.5 м, 0.35 м / 10°, 120 с, 1 м) — предложение по замерам Depot, не решение Шифу.
- `save_waypoint` пишет оценку позы, не истину: если локализация поехала, в реестр попадёт кривая точка. Это и
  видно: строка `registry` и ошибка `route` у той же точки.
- Мебель восточного крыла одинакова до сантиметра — это нарочно: так проверяются похожие места.
- Пока стек перезапускается, оценки нет: в `poses.csv` пустые `ex,ey,eyaw`, на анимации красной позы нет.
- `hil.sh restart` пересоздаёт только сервисы контура HIL на Main Pi; Gazebo, реестр и Vision не трогает.
