# Ролик из HIL: РОББОКС едет в Gazebo

Как снять ролик (MP4 и GIF ≤ 2 МБ), где стек настоящего робота (Nav2, rtabmap, защита) едет по данным
Gazebo на песочнице. Первый прогон 06.11.2026: мир Depot, картография с нуля. Что получилось и что нет,
записано в §9. Эпик HIL — #4371, стенд и подводные камни — `docs/research/2026-11-05-nav2-hil-tuning.md`.

Метки в тексте: **[измерено]** — на прогоне 06.11, **[код]** — по исходникам.

## 0. Что нужно и сколько это стоит

- Ключ в `authorized_keys` Main Pi, Vision Pi, Sound Pi и песочницы `10.1.1.250` (пароли нельзя).
- Замок стенда у координатора (`stand.lock`), деплой не идёт: `hil.sh on` ставит `/tmp/hil.lock` на Main Pi,
  и деплой на нём останавливается. Включать HIL только после сообщения «деплой готов».
- Время [измерено]: подготовка песочницы 5 мин (Depot 82 МБ), `hil.sh on` 2–3 мин, картография пяти целей
  ~8 мин стены, монтаж 2 мин.
- RTF [измерено]: камеры 1280×720 по 10 Гц на программном рендере (EGL `dri2` не создаётся, это llvmpipe) дали
  RTF **0.12**, цели не достигались за таймаут. Камеры 960×540 и 480×270 по 3 Гц — RTF **0.7**. Не поднимай камеры.

## 1. Подготовка песочницы (один раз на ролик)

```bash
scripts/hil/video/sandbox_prep.sh        # из корня репо; копия частей репо в /home/ros2/rob_box_hilvideo
```

Что делает скрипт и на какие грабли наступили:

1. **Отдельная копия** `/home/ros2/rob_box_hilvideo`, а не `/home/ros2/rob_box_hil`: боевой чекаут песочницы
   лежит на старом коммите, миры в нём старые, и мы его не правим. Скрипт копирует `docker/sim`,
   `docker/main/{config,docker-compose.yaml}`, `docker/common`, `src/rob_box_sim/worlds`.
   `docker/sim/compose.yaml` делает `extends ../main/docker-compose.yaml`: без него `compose up` падает
   «open …/docker/main/docker-compose.yaml: no such file or directory».
2. **CRLF.** Файлы, собранные на Windows, приходят с `\r`. `ros_with_namespace.sh` падает
   `/bin/bash^M: bad interpreter`, контейнер gazebo выходит с кодом 127, `hil.sh on` пишет «gazebo не работает
   (exited 127)». Скрипт делает `sed -i 's/\r$//'` по `*.sh *.py *.yaml *.json5 *.sdf`.
3. **Depot из Fuel**: `https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot/6/Depot.zip`, 82 МБ, лицензия
   CC-BY 4.0 (автор Open Robotics) [измерено: API Fuel, `license_name`]. В репо не кладём, лежит в
   `fuel/Depot` на песочнице. Размер здания около 30×15 м, проходы между стеллажами 1.5–2.3 м.
4. **`trim_depot.py`** делает `Depot_vid`: без крыши, вентиляторов и окон (иначе сверху виден потолок), **без
   collision**. Последнее важно: в Depot есть только плоскость-пол в collision, стен и стеллажей для физики
   нет, робот физически проедет сквозь них. Лидар видит визуалы, Nav2 их объезжает, но столкновений с
   препятствиями физика не даст. Меши и текстуры — жёсткие ссылки (`cp -al`): симлинки на путь хоста внутри
   контейнера не видны («Unable to find file …/Depot.dae»).
5. **`patch_compose.py`** правит копию `compose.hil.yaml`: gazebo сначала строит мир `mkdepot.py` (шапка
   `polygon.sdf`, Depot, две камеры), потом запускает `hil_run.sh` со спавном в (0, 0).
6. **Карта = мир.** rtabmap кладёт начало кадра `map` в стартовую точку робота. Если спавнить робота не в
   (0, 0), цели в координатах мира уезжают на смещение (первая попытка 06.11: цель «в (−6, 4)» оказалась
   за стеной). Поэтому здание сдвинуто на `DEPOT_POSE=10.5 0 0` так, что спавн (0, 0) стоит на свободном
   полу, а цели считаются в тех же координатах.

Параметры мира и камер — переменные `DEPOT_POSE`, `DEPOT_TOP`, `DEPOT_CHASE`, `DEPOT_TOPW`, `DEPOT_CHW`,
`DEPOT_RATE` (см. `mkdepot.py`). Камеры: сверху над центром здания (центр кадра в мире `(10.5, 0)`, 40 пикс/м
после масштаба до 1280×720) и 3D-вид из угла здания.

## 2. Включение HIL

Только после «деплой готов» от координатора и с записью в `stand.lock` (`shidi hil-video <UTC>`):

```bash
export HIL_SANDBOX_REPO=/home/ros2/rob_box_hilvideo
export HIL_MAPS_DIR=/home/ros2/rob_box_hil/maps_depot_$(date +%H%M)   # СВЕЖИЙ каталог: карта пустая
scripts/hil/hil.sh on > /tmp/hil_on.log 2>&1; tail -5 /tmp/hil_on.log
scripts/hil/hil.sh status                    # в графе ровно один /controller_manager
```

- `hil.sh on` не стартует при существующем замке (`/tmp/hil.lock`); если прошлый `on` упал на песочнице, сначала
  `hil.sh off`, потом `on` снова.
- Каждому прогону новый `HIL_MAPS_DIR`: иначе rtabmap откроет старую базу и карта будет с хвостом. Боевую
  `./maps` и `/data` скрипт и так не примет.
- Реестр: `voice-assistant` на Sound Pi пересоздаётся с `WAYPOINT_DB_PATH=/data/hil/waypoints.db`, боевой
  `maps` до и после сверяется (`hil.sh off` пишет «боевой реестр карт … не изменился»).
- CAN только на чтение, параллельно со всей работой: `ssh ros2@10.1.1.10 'nohup timeout 3000 candump -t a -L can0
  > /tmp/can_video.log 2>&1 &'`. Остановить: `pkill -x candump` (не `-f`).

## 3. Картография и маршрут по точкам

```bash
ssh ros2@10.1.1.250 'cd /home/ros2/rob_box_hilvideo && (nohup ./runvideo.sh > out/runvideo.out 2>&1 < /dev/null &)'
# ждать конца: файл out/run_end.txt; прогресс:
ssh ros2@10.1.1.250 'grep -E "GOAL_END|RECOV|EM_ON|TIMEOUT" /home/ros2/rob_box_hilvideo/out/route.log'
```

`runvideo.sh` внутри контейнера `rob-box-hil-gazebo`: подписчики камер (`ign topic -e`, без ROS: сенсор
рендерит кадр только при подписчике), `rob_box_sim.map_recorder` (снимки `/rtabmap/map` и позы) и `route.py`.
`route.py` шлёт цели `NavigateToPose` по одной (цели M — картография по пустой карте, R — маршрут по карте)
и печатает `GOAL_START/GOAL_END` с sim time, номером кадра камеры, истинной позой, ошибкой до цели,
числом стопов защиты (эпизоды `/cmd_vel_emergency`) и восстановлений. Список целей — `GOALS` в начале `route.py`.

Камеры и RTF: `CAMERAS=1|0` (по умолчанию 1 у `runvideo.sh`, **0** у `runcoverage.sh`). Подписчики камер заставляют
сенсор рендерить, на программном рендере песочницы RTF падает до 0.33–0.34; без них RTF 1.0 [измерено 09.11].
Ролик пылесоса строится из TRACK-лога (`pose_anim.py`), кадры камер ему не нужны. При `CAMERAS=0` скрипт ещё
гасит подписчиков прошлого прогона. В контейнере процесс подписчика называется `ign-transport-topic`, а не
`ign topic`: гасить `pkill -f ign-transport-topic` (скрипты так и делают). `CAMERAS=1 ./runcoverage.sh` вернёт камеры.

Подводные камни:

- Печать через `| grep > файл` буферизуется: нужен `grep --line-buffered` (в скрипте есть).
- `sandbox_prep.sh` не падает на файлах root в `out/` (контейнеры пишут туда от root): CRLF-шаг (`fix_crlf.sh`)
  обходит `out*` и `fuel`, и шаги 4–5 (Depot, `patch_compose.py`) выполняются.
- Остановить прогон: `docker exec rob-box-hil-gazebo pkill -f /route.py`. **Не** `pkill -f runvideo.sh` через
  ssh: шаблон совпадёт с самой ssh-командой и сессия оборвётся.
- «STATUS_SUCCEEDED» говорит про карту, а не про мир. Сверяй `err=` в `GOAL_END` (истинная поза против цели).
  Ошибка позы rtabmap/ICP видна в `maprec/poses.csv` (колонки `t,tx,ty,tyaw,ex,ey,eyaw`).

### 3a. Обход покрытием («пылесос», #4560) [код; на складе — две попытки 09.11, места разворота ещё не ездили]

После картографии (карта `/rtabmap/map` уже есть) вместо `runvideo.sh` — `runcoverage.sh`. Модули плана
и метрики берутся из `pylib/` копии (кладёт `sandbox_prep.sh`), а не из образа.

```bash
# сначала только план: out/coverage/plan.png, plan.json, строка PLAN и UNREACHABLE в out/coverage.log
ssh ros2@10.1.1.250 'cd /home/ros2/rob_box_hilvideo && ./runcoverage.sh --plan-only'
# обход (можно --max-poses 20 для пробы); конец — строка COVERAGE_DONE
ssh ros2@10.1.1.250 'cd /home/ros2/rob_box_hilvideo && (nohup ./runcoverage.sh > out/runcoverage.out 2>&1 < /dev/null &)'
ssh ros2@10.1.1.250 'grep -E "PLAN|UNREACHABLE|GOAL_END|EM_ON|COVERAGE_" /home/ros2/rob_box_hilvideo/out/coverage.log'
```

- Лог в формате `route.py` (TRACK, GOAL_START/GOAL_END, EM_ON/EM_OFF, RECOVERY). Полоса (start → промежуточные →
  end) уходит **одной целью `NavigateThroughPoses`** (`GOAL_START L0007_c3l2 target=[…]`, после `GOAL_END` —
  `LANE_FEEDBACK … remaining=`): робот не тормозит в каждой позе. Позами по одной (`--per-pose`) на попытке 3
  09.11 вышло 0.06 м/с и 59 % времени в движении. Пройденную точку убирает `RemovePassedGoals` в
  `docker/main/config/nav2/behavior_trees/navigate_through_poses.xml` (радиус 0.35 м); **без деплоя этого
  файла на Main Pi** (`L: Deploy and Verify`) Nav2 перепланирует через уже пройденные точки и возвращается к ним.
  Таймаут полосы — `--timeout` (45 с) на позу × поз + путь / 0.15 м/с (потолок `max_vel_x` Nav2).
  Неудачную цель исполнитель пропускает и считает; после `--max-fail` (5) неудач подряд останавливается,
  остаток — `NOT_SENT`.
- Итог `COVERAGE_DONE`: `coverage=` — доля **покрываемого** пола (до чего корпус дотягивается из достижимых
  центров) под корпусом по правде Gazebo; `coverage_free=` — доля **всего свободного** пола карты (с узкими
  местами и полосой у стен). Первая говорит, как исполнен план, вторая — сколько склада реально проехано.
  В строке `PLAN` то же для плана: `plan_coverage` и `plan_coverage_free`. Исходы целей — короткие имена
  (`SUCCEEDED`, `ABORTED`, `TIMEOUT`, `INTERRUPTED`…): `STATUS_*` из `route.run_goal` нормализует
  `GoalOutcome.from_route` (09.11 без этого 20 успехов засчитались неудачами). `goals=` — целей (полос),
  `poses=`/`poses_passed=` — поз плана в них и пройдено (успех — все, иначе по feedback Nav2).
  Движение по треку истинной позы: `moving_s`/`moving_pct` — время в движении (быстрее 0.03 м/с или 0.10 рад/с),
  `moved_m` — путь, `avg_mps` — путь / всё время обхода, `moving_mps` — путь / время в движении.
- **Остановить обход**: `docker exec rob-box-hil-gazebo pkill -INT -f 'python3 -u /coverage_run.py'` (или
  `-TERM`). Исполнитель отменяет текущую цель (`INTERRUPTED cancel` в логе, исход `INTERRUPTED`), шлёт нулевую
  скорость в `/cmd_vel` (`HALT`), остаток — `NOT_SENT`, и пишет `COVERAGE_DONE … stop=interrupted_SIGINT
  interrupted=yes` по пройденному. Второй сигнал — выход сразу, без итога.
- Пересчёт покрытия по логу без ROS: `python3 -m rob_box_sim.coverage_metric out/coverage/plan_map.npz
  out/coverage.log --out cov.png` (зелёное — покрыто, красное — пропущено, оранжевое — свободно, но корпусу
  недоступно: запас padding, узкие места, пол за проходом уже габарита).
- Ручки: `--spacing` (0.55 м), `--axis auto|x|y`, `--waypoint-step` (промежуточные позы на полосе, 0 — нет),
  `--timeout` на позу (45 с стены), `--per-pose`. Ось при `auto`: из планов, что накрывают не меньше 98 % лучшего,
  код берёт самый быстрый по оценке: путь / 0.15 м/с (`max_vel_x` Nav2) + 5 с на позу (5 с — догадка).
- **Места разворота** (`--turn-clearance`, умолч. 0.66 м = описанный радиус 0.43 + стоп защиты 0.20 + padding
  0.03). Полоса сбоку идёт по сужению 0.336 (у стеллажа корпус в 3 см), а концы полос, где робот крутится,
  стоят только там, где до занятого ≥ 0.66 м. Строку вдоль внешней стены и тупик уже 1.32 м план не берёт:
  развернуться там негде, задом план не выезжает. `--turn-clearance 0` — старый план (концы в 0.34 м от стены:
  попытка 1 на Depot 09.11, 58 стопов защиты за 10 мин). В симе центр вращения — задняя ось (−0.144 м),
  углам нужно 0.69 м: если у стен всё равно стопы — `--turn-clearance 0.69`. `--erosion 0.65` больше не нужен.
- **Поперечный проход** (по умолчанию; `--no-cross` — без него): полосы другой оси от места разворота до места
  разворота, только где они накрывают ≥ 0.25 м² пола, не накрытого основными полосами (торцы у колонн).
- Строка `PLAN`: `turn_clearance`, `min_turn_clear_m` (самая тесная поза разворота до занятого — должна быть
  ≥ `turn_clearance`), `lanes_dropped_no_turn` (строки без места разворота), `clipped_m` (срезано с концов
  полос), `cross_lanes`.

## 4. Запись видов

Камеры пишут PNG сами (`<save enabled="true">`) в `out/frames/video_top` и `video_chase`, по sim time.
Номер кадра ↔ sim time `compose.py` берёт из линейной подгонки по `GOAL_START/GOAL_END`.

## 5. Монтаж и экспорт

Скилл `vizavi-motion` (раскадровка, подписи, ритм, проверка экспорта) для оформления, а материал только
настоящие кадры прогона. Сборка — `make_video.py`: титульная карточка, основной отрезок (вид сверху с
растущей картой rtabmap, истинной позой, следом, 3D-камера в углу, плашка «sim время, ускорение, фаза, цель»),
итоговая карточка с честной сводкой по целям из `route.log`, затем MP4 и GIF.

```bash
ssh ros2@10.1.1.250 'docker run --rm --entrypoint python3 \
  -v /home/ros2/rob_box_hilvideo/out:/run:ro -v /home/ros2/rob_box_hilvideo:/s:ro -v /tmp/vid:/o \
  rob_box_sim:hil /s/make_video.py /run /o <T0_sim> <T1_sim> --gif-mb 2'
scp ros2@10.1.1.250:/tmp/vid/hil_depot.mp4 ros2@10.1.1.250:/tmp/vid/hil_depot.gif .
```

`T0..T1` — sim время удачного отрезка (из `GOAL_START` первой цели до `GOAL_END` последней хорошей).
Выходы [измерено]: MP4 1280×720, 20 к/с, H.264, yuv420p, без звука, 2.5 МБ, 55 с (ускорение ×5);
GIF 640 px, 10 к/с, ×8, 1.2 МБ — `make_video.py` понижает ширину, кадры и палитру, пока GIF не влезет в предел.
Проверка экспорта: `ffprobe` (кодек, размер, `pix_fmt`, число кадров) и просмотр кадров начала, середины, конца.

## 6. Выключение и проверки «после»

```bash
scripts/hil/hil.sh off                      # сим гаснет первым; реестр карт сверяется со снапшотом
scripts/hil/hil.sh status                   # замка нет, на Main работают ros2-control и lslidar
# md5 боевых конфигов nav2 совпали с замером «до»
ssh ros2@10.1.1.10 'cd /home/ros2/rob_box_project/docker/main/config/nav2 && md5sum *.yaml'
ssh ros2@10.1.1.10 'docker inspect nav2 --format "{{range .Config.Env}}{{println .}}{{end}}" | grep -c NAV2_PARAMS_OVERLAYS'  # 0
# CAN: только статус-кадры VESC (команды 9, 14, 15, 16, 27), команд 0..8 нет
ssh ros2@10.1.1.10 "python3 - /tmp/can_video.log" <<'EOF'
import sys, collections
c = collections.Counter()
for l in open(sys.argv[1]):
    p = l.split()
    if len(p) > 2:
        try: c[(int(p[2].split('#')[0], 16) >> 8) & 0xFF] += 1
        except ValueError: pass
print(dict(sorted(c.items())))
EOF
rm "$STAND_LOCK"                            # замок стенда; написать координатору «стенд свободен»
```

Каталоги `HIL_MAPS_DIR` на Main Pi (`/home/ros2/rob_box_hil/maps_depot*`) остаются: это не боевые карты.
Каталог `out/` на песочнице принадлежит root (кадры пишет контейнер): чистить только контейнером —
`docker run --rm --entrypoint bash -v …/out:/o rob_box_sim:hil -c 'rm -rf /o/*; chmod 777 /o'`.

## 7. Подсказки по миру

- Мир Depot: на кадре сверху большое свободное место на западе, стеллажи и ящики на востоке, коридор между
  рядами ящиков и стеллажей шириной около 1.7 м. Колонны здания в 3D-виде заслоняют обзор.
- Другие миры Fuel: AWS RoboMaker Small House, офис, больница (лицензии проверять на карточке модели в Fuel;
  Apache-2.0 и CC-BY подходят). Не проверялись.
- Большое здание — свой мир `building` (#4468, `rob_box_sim.building_floor`): этаж 58 × 15 м, 20 кабинетов,
  стены с collision, в репо. Готовые миры разобраны в PR шага 1 карточки #4468. Запуск — `HIL_SIM_WORLD=building`
  (обычный `compose.hil.yaml`, без Depot), точки-кабинеты — `ROUTE_GOALS_FILE=/sim_worlds/building.goals.yaml
  ./runvideo.sh aud_101 dean …` (имена из файла, порядок — как в аргументах). В Gazebo ещё не запускался [код].
  План с точками: `docs/runbooks/img/hil-building-plan.png` (коробки, прочитанные из `building.sdf`).

## 8. Чего не делать

- Не включать HIL до «деплой готов». Не оставлять HIL включённым: деплой остановится на замке.
- Не писать боевые карты: `HIL_MAPS_DIR` свежий, не `./maps` и не `/data`.
- Не называть цель достигнутой по `STATUS_SUCCEEDED`, не сверив с истинной позой.

## 9. Что вышло 06.11.2026 [измерено]

Мир Depot, спавн (0, 0), здание сдвинуто на +10.5 по X. Логи прогонов — в каталоге оператора
(`scratchpad/hilvideo/run1_route.log`, `run2_route.log`).

| прогон | цели | итог |
|---|---|---|
| 1 | M1–M5 (картография) | все 5 достигнуты, ошибка 0.19–0.28 м, стопов 0, 1 восстановление; ошибка позы rtabmap до 0.33 м |
| 1 | R1 | Nav2: «дошёл» за 65 с, но истинная поза в 6.4 м от цели, 4 стопа защиты; ICP-оценка ушла (на t≈270 с ошибка 4.6 м, к 430 с 12.7 м) |
| 2 | M1, M2 | дошли, ошибка 0.19 и 0.23 м |
| 2 | M3–M5, R1 | «дошёл» по карте, истинная ошибка 3.6–11.4 м: оценка позы ушла между M2 и M3 (sim 112–148 с), 3 стопа и 6 восстановлений на R1 |

Вывод: картография в Depot работает, пока ICP-одометрия держится, но надёжного возврата по готовой карте
**нет**: оценка позы уходит на метры в зале с редкими опорными поверхностями. Причина ICP-ухода не
исследована [догадка: большой открытый зал, лидар видит мало геометрии]. Маршрут по готовой карте в
ролик не вошёл. Первый запуск на 1280×720/10 Гц (RTF 0.12) и вторая попытка с неверным смещением карты в
счёт полных прогонов не вошли: цели в них не выполнялись.

Причина найдена позже (#4495, разбор — `docs/research/2026-11-05-nav2-hil-tuning.md` §9) [измерено]:
ICP-одометрия теряется на быстром развороте и без `Odom/ResetCountdown` не возвращается до конца прогона.
С `Odom/ResetCountdown=1` у `icp_odometry` 12 целей Depot дали ошибку 0.17–0.34 м.
