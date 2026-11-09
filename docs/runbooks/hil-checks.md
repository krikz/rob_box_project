# HIL-проверки группы А: одна команда, raw и вердикты кода

Как одним запуском проверить в HIL карточки, у которых фикс уже влит и не хватает только живой проверки:

| Карточка | Что проверяем | Сценарий |
|---|---|---|
| #3789 | честные фразы тулов движения: по ABORTED и у `stop_navigation` («Я и так стою.» / «Останавливаюсь.») | `nav_aborted`, `stop_idle`, `stop_active` |
| #4435, #4456 | после провала или отмены NavigateToPose FollowPath погашен и выход twist_mux = 0 | `nav_aborted`, `stop_active` (вердикт `tool_halt`), `orphan_natural` |
| #4458 | сторож сирот гасит FollowPath, если цель NavigateToPose прислал не наш тул | `orphan` |
| #4365 | в движении у препятствия: стоп, писк, «Осторожно!», маячок вместе | `zone` |
| #4470 | `HIL_GUARD_SOUND=on`: писк уходит на настоящего робота; `off`: только строка в журнале, стоп и маячок есть | `zone` (on), `guard_off` (off) |

Скрипты лежат в `scripts/hil/checks/`: `run_checks.sh` (оркестр), `probe.py` (rclpy в контейнере gazebo песочницы),
`verdicts.py` (вердикты кода, тесты `tests/unit/scripts/test_hil_checks_verdicts.py`). HIL включает и выключает
`scripts/hil/hil.sh`, а не этот набор.

Метки: **[код]** — по исходникам, **[оценка]** — время прикинуто по таймаутам скрипта, на стенде ещё не мерено.

## 0. Перед запуском

1. Возьми замок стенда сам: `stand.lock` координатора, первое слово — твоё имя (например
   `hil-checks #4365 2026-11-10T09:00:00Z`). Скрипт замок не берёт, он только сверяет первое слово с
   `HIL_CHECKS_OWNER` и отказывает, если стенд чужой.
2. Включи HIL: `scripts/hil/hil.sh on` (робот на кирпичах, звук защиты по умолчанию включён, решение Шифу 05.11).
3. Деплой должен включать фикс сторожа (#4567). До него контейнер `followpath-watchdog` стоит в `Restarting (126)`,
   и скрипт откажет на предусловиях: так и задумано, сторожа без него не проверить.
4. Ключи в `authorized_keys` Main Pi, Sound Pi и песочницы. Пароли нельзя.

## 1. Запуск

```bash
export MSYS_NO_PATHCONV=1
HIL_CHECKS_OWNER=hil-checks scripts/hil/checks/run_checks.sh            # все сценарии по порядку
HIL_CHECKS_OWNER=hil-checks scripts/hil/checks/run_checks.sh zone orphan   # выборочно
```

Сценарии по умолчанию идут так: `orphan zone guard_off nav_aborted stop_idle stop_active orphan_natural`.
`orphan_natural` без `ORPHAN_GOAL` пропускается со строкой `NOT_TRIGGERED`: цель зависит от мира.

Переменные (умолчания — в шапке `run_checks.sh`):

| Переменная | Зачем |
|---|---|
| `HIL_CHECKS_OUT` | куда писать raw; по умолчанию `$TMPDIR/hil_checks/<UTC>` |
| `STAND_LOCK` | путь к замку стенда координатора |
| `ABORT_PHRASE` | реплика для ABORTED: точка вне карты, план не строится (по умолчанию x=40, y=40) |
| `STOP_PHRASES` | фразы через `\|`: пробуются по очереди, пока модель не позовёт `stop_navigation` |
| `ORPHAN_GOAL` | `x,y,yaw°` в `map` для сценария 3b. Мир `polygon`: подход наискось к проёму 1.2 м с востока на запад, например `0.5,0.0,180` |
| `ORPHAN_LOAD_S` | нагрузить Main Pi на столько секунд (4 пустых цикла под `timeout`), как в 3b при load 10–18 |
| `ORPHAN_GRACE_S` | за сколько секунд sim после провала цели FollowPath должен погаснуть (2.0 = `orphan_timeout_s` 1.5 + такт) |

После прогона: `scripts/hil/hil.sh off` и снять свой замок.

## 2. Что делает каждый сценарий [код]

- **Предусловия.** Скрипт сохраняет `hil.sh status`, `docker inspect` нужных контейнеров и часы четырёх машин.
  Отказ с кодом 3 будет, если: замок чужой; HIL выключен; `/controller_manager` в графе не ровно один; контейнер не
  `running`, нездоров или поднялся меньше 15 с назад; часы расходятся больше 5 с.
- **`orphan`** (#4458). Проба шлёт цель NavigateToPose в свободную сторону, через 2 с отменяет её и сразу шлёт
  свою цель FollowPath по прямой. По правилу сторожа это «сирота»: последняя NavigateToPose терминальна, а FollowPath
  активен. PASS, если цель кончилась CANCELED не позже `ORPHAN_GRACE_S` с sim, в журнале `followpath-watchdog` есть
  строка с её `goal_id` и через 0.5 с после отмены выход twist_mux = 0. Если сторож не погасил, проба через 10 с
  отменяет цель сама (`FP_SELF_CANCEL`), и это FAIL.
- **`orphan_natural`** (#4435, 3b). Цель `ORPHAN_GOAL` из скрипта, по желанию под нагрузкой. Сироту здесь нельзя
  заказать: если BT сам погасил FollowPath, вердикт будет `NOT_TRIGGERED` («сироты не было»), а не PASS.
- **`zone`** (#4365, #4470 on). Проба поворачивается по правде Gazebo к ближайшему препятствию (до 3.5 м) и едет
  на него телеопом `/cmd_vel_web` 0.1 м/с. После `EM_ON` она толкает ещё 4 с. PASS для #4365: защита вошла в `stop`,
  робот за удержание сдвинулся не больше 3 см, на Sound Pi есть частый писк не позже 2 с после стопа и «Осторожно!»
  не позже 6 с, а на Main Pi пойман хотя бы один запрос маячка. Имена писка, маячка и фразы берутся из
  `rob_box_teleop/proximity.py` (`ALARMS`).
- **`guard_off`** (#4470 off). Проба отъезжает назад. Затем `proximity-guard` пересоздаётся с `HIL_GUARD_SOUND=off`
  той же командой compose, что у `hil.sh on`, только для одного сервиса, и делается тот же наезд. PASS: стоп есть,
  маячок есть, писка и фразы на Sound Pi нет, в журнале защиты есть строка «HIL: зона stop, на настоящий робот не
  уходит». В конце, и при любом выходе скрипта тоже, защита возвращается в `on`.
- **`nav_aborted`** (#3789, #4435). Реплика `ABORT_PHRASE` уходит в диалог (`scripts/robot_dev/inject.sh`), проба
  слушает статусы. PASS: тул навигации вернул ABORTED, а `tts_node` сказал фразу ключа `motion.rejected.aborted`
  или `motion.not_reached.aborted` из `phrases/ru.yaml`. Если в окне есть «Приехал», это FAIL. Если исход не ABORTED
  (таймаут, модель выбрала другой тул), вердикт `NOT_TRIGGERED`. Вердикт `tool_halt`: через `ORPHAN_GRACE_S` после
  провала цели FollowPath не активен и mux = 0.
- **`stop_idle` / `stop_active`** (#3789). Стоп-слова уходят в Tier-1 `stop_all`, и тул так не позвать, поэтому
  фразы — «отмени навигацию» и подобные. PASS: вызван `stop_navigation` и сказано «Я и так стою.» (`motion.stop.idle`),
  а во время поездки из скрипта «Останавливаюсь.» (`motion.stop.confirmed`) с итогом цели CANCELED. Если модель не
  позвала тул ни на одну фразу, это `NOT_TRIGGERED`, а не PASS.

## 3. Сколько длится [оценка]

Предусловия ~0.5 мин, `orphan` ~1 мин, `zone` ~1.5 мин, `guard_off` ~3–4 мин (пересоздание защиты ×2),
`nav_aborted` до 4.5 мин (ждёт исход тула до 180 с), `stop_idle` до 2 мин, `stop_active` до 2.5 мин,
`orphan_natural` до 4 мин. Всего ~15–20 мин, плюс `hil.sh on/off` 2–3 мин каждый. На стенде ещё не мерено.

## 4. Что приложить в карточку

Каталог `HIL_CHECKS_OUT`:

```
verdicts.txt                       строки VERDICT <карточка> <сценарий> <статус>: <факты> — по одной на вердикт
preflight/{hil_status,inspect,clock,stand_lock}.txt
orphan/{probe.log,watchdog.log,nav2.log}
zone/{probe.log,guard.log,voice.log,expr.log,env.txt}          guard_off/ — то же + recreate_*.txt
nav_aborted/{probe.log,voice.log,nav2.log,inject.txt}          stop_*/{voice.log,inject.txt[,probe.log]}
```

В комментарий к карточке: строку `VERDICT` из `verdicts.txt`, `git rev-parse HEAD` и тег образа с робота,
выдержки из raw по фактам вердикта (строки `tts_node`, `followpath_watchdog`, `proximity_guard`, `NAV_STATUS`/`MUX`
пробы) и CAN-проверку, что на VESC за окно HIL нет команд, как в прошлых прогонах (#4365). Закрывать карточку можно
только по PASS. `NOT_TRIGGERED` значит «не проверено»: так и пиши.

## 5. Ограничения [код]

- Время пробы — часы песочницы, время логов — часы Pi. Сдвиг больше 5 с отсекают предусловия, окна вердиктов
  берут запас в 1–6 с.
- Маячок ловит `ros2 topic echo /voice/expression/request` внутри контейнера `proximity-guard`: это запрос
  на мигалку, а не снимок лица на Vision Pi.
- `zone` и `orphan` выбирают направление по скану, мир им не важен. В мире Depot нет collision у стен: физика
  не остановит, остановить может только защита. Для `zone` это даже строже.
- Выбор тула в `nav_aborted` и `stop_*` делает модель, поэтому `NOT_TRIGGERED` здесь честный исход, а не сбой скрипта.
