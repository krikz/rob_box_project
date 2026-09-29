# Фаза клока Renardo и старт формы трека (issue #3112)

Статус: **диагностика включена по умолчанию, фикс — кандидат за флагом
`ROB_BOX_MUSIC_ALIGN_CLOCK=1` (по умолчанию ВЫКЛ)**. На роботе не проверено.

## Проблема (по исходникам renardo_lib 0.9.13)

| Что | Где | Следствие |
|---|---|---|
| `TimeVar.get_current_index`: `time = now - self.start_time`, `start=0` | `TimeVar.py:150`, `:33` | `var`/`linvar`/`Pvar` (гейты секций, `gflt`, `Root.default`) считают фазу от доли 0 клока |
| `Player.count`: `acc = now - (now % total_dur)`, `n = len(durs) * acc / total_dur` | `Players.py:702-728` | индекс нот/атрибутов плеера — тоже от доли 0 |
| `TempoClock.clear()` не трогает `beat` | `TempoClock.py:686` | после `Clock.clear()` клок продолжает идти |
| Плеер встаёт на `next_bar()` | `Players.py:892`, `TempoClock.py:648` | старт формы = `next_bar mod F` (F — длина формы в долях) |

Трек на давно идущем клоке (DJ-сет, второй трек) начинается с середины формы.

## Диагностика (по умолчанию ВКЛ)

`ComposeMusicTool._execute_with_clock_phase` (`tools/music.py`) до и после
`execute_code` снимает `core.clock_phase.clock_phase_snapshot(Clock, F)`:
`clock_beat`, `start_beat = next_bar(clock_beat)`, `phase_offset_beats = start_beat mod F`.
Результат попадает в `result.data["clock_phase"]` и `result.data["clock_phase_offset_beats"]`
(снимок после exec, иначе до), а в лог уходит INFO со строкой `[#3112] фаза клока: ...`.
Все ошибки проглатываются: снимок `None`, музыка играет.

Оговорка: это оценка. Плееры ставятся на `next_bar` в момент своей строки
`>>`; если между строкой и снимком клок перешёл через границу такта (exec
плюс 50 мс паузы `/g_freeAll`), оценка ошибётся на такт.

## Фикс-кандидат: `Clock.set_time` сразу после `Clock.clear()`

```
Clock.clear()
Clock.set_time(((Clock.now() + 2) // F + 1) * F - 2)
Clock.bpm = ...
```

Почему так (все ссылки на `TempoClock.py`):

1. `set_time(beat)` (`:420`) пишет `beat`, `bpm_start_beat`, `bpm_start_time = time.time()`,
   **чистит `queue`** и перезапускает `count` у `self.playing` (после `clear()` пуст).
   При float-bpm `_now()` (`:463`) дальше считает `bpm_start_beat + elapsed*bpm/60`.
2. **Строка должна стоять до `Clock.bpm = ...`**: `__setattr__('bpm')` → `update_tempo`
   (`:218-249`) ставит смену темпа в очередь на `next_bar`. `set_time` после неё
   стёр бы смену темпа, и трек играл бы в темпе прошлого.
3. **Цель `T ≡ F-2 (mod F)`, середина такта.** `next_bar = beat + (4 - beat % 4)` даёт
   `T + 2 = k·F`, пока сдвиг `beat` после `set_time` лежит в (-2, +2). Сдвиг есть:
   `_now()` берёт `get_time()` с `nudge + hard_nudge` (`:390`), а `set_time` пишет
   `time.time()` без них. `Clock.clear()` не сбрасывает `nudge`, который `Clock.swing`
   прошлого трека поставил TimeVar-ом (`:290`). При `swing ≤ 0.3` и `bpm ≤ 180`
   (клампы аранжировщика) nudge ≤ 0.45 с, это ≤ 1.35 доли, то есть < 2.
4. **Прыжок только вперёд (`T > now`).** `set_time(-4)` дал бы старт с доли 0,
   но есть два риска. (a) `TimeVar.get_current_index` монотонен (`if time >= self.next_time`,
   состояние `next_time/next_index` не откатывается): TimeVar, переживший `clear`
   (например `nudge` от swing), замёрз бы до возврата клока на старую долю.
   (b) Гонка с потоком `run()` (`:564`): он мог посчитать `beat = _now()` до `set_time`
   и вытолкнуть только что поставленные блоки со старой большой долей. При прыжке
   вперёд устаревшая доля всегда меньше новых блоков, и ничего не выталкивается.
5. Цена: фаза 0 гарантирована только для периодов, которые делят F. По golden-фикстурам
   (`test/fixtures/arranger_golden.json`, 88 кейсов с `Clock.future`) это 1079 из 1083
   `dur`/`var`-периодов. Исключение — `mozart40`, сумма `dur` 8.0001 (артефакт округления).
   В club `linvar(..., 31/61)` к форме не привязаны. Плюс сопряжение длины нот и
   длины `dur` у плеера (`event_n mod len(notes)`) этим не проверено.
6. `Clock.future(F, Clock.clear)` считается от `now ≈ T`, а плееры стартуют на
   `T + 2`. Поэтому при флаге конец ставится на `F + 2`: форма доигрывает целиком.
   Без флага срезалось 0–4 доли, случайно.

Второй вариант из issue — `start=` во все `var`/`linvar`. Отвергнут: он не сдвигает
паттерны `Player` (`count` без `start`) и `Pvar`-мотивы, так что фикс вышел бы частичным.

Санитайзер (`renardo_sanitizer.sanitize_renando`) строку пропускает без изменений
(токены `import/os/eval…` в ней не встречаются, dunder-ов нет). Это покрыто тестом.

## Эксперимент без сервера

Настоящие `TempoClock.py`/`TimeVar.py` (серверные модули заглушены, поток не запущен)
для старта 0 / 37.3 / 500.9 / 1021 / 127.99: после prelude `next_bar % 128 == 0`,
смена темпа в очереди стоит на k·F, свежий `var` на `next_bar + 0.01` даёт первое значение.
При nudge ±0.45 с `next_bar` тоже k·F. Сырой вывод — в отчёте шиди к #3112.

## Проверить на роботе (не сделано)

См. чек-лист в отчёте. Главное: INFO `[#3112]` с `смещение в форме ≠ 0` на
втором треке без флага и `= 0` с флагом; на слух — интро с начала;
темп нового трека применяется (нет залипания на старом bpm).

## Включение на роботе

Решение владельца 28.09: live-прогон показал `[#3112]` смещение ≠ 0 во всех 13
треках, флаг включается на роботе. В коде дефолт остаётся ВЫКЛ до
live-подтверждения.

Путь переменной до процесса:

1. `docker/vision/docker-compose.yaml`, сервис `voice-assistant`, `environment`:
   `ROB_BOX_MUSIC_ALIGN_CLOCK=${ROB_BOX_MUSIC_ALIGN_CLOCK:-1}`. Если в `.env`
   ничего не задано, значение 1.
2. ENTRYPOINT образа (`bash -c 'source … && exec "$@"'`), затем
   `ros_with_namespace.sh` (`exec "$@"`), затем `start_voice_assistant.sh`
   (`exec ros2 launch …voice_assistant_headless.launch.py`). Ни `env -i`, ни
   `unset` по пути нет.
3. `Node(package='rob_box_mcp_tools', executable='mcp_server')` без `env=`:
   launch_ros передаёт процессу окружение launch целиком.
4. `mcp_server` → `tools/music.py:music_align_clock_enabled()` читает
   `os.environ` при каждом `compose_music`/DJ-треке.

Коммитнутый `docker/vision/.env` переменную не задаёт. Guard:
`src/rob_box_voice/test/unit/core/test_issue_3112_music_align_clock_env.py`.

**Откат:** добавить `ROB_BOX_MUSIC_ALIGN_CLOCK=0` в `docker/vision/.env` и
перезапустить сервис: `docker compose up -d voice-assistant`. Одного
`restart` мало, он не перечитывает окружение.
