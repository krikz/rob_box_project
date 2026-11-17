# scsynth block size (`-z`) на Vision Pi — теория, затронутые синты, процедура замера

Issue: #3114 (perf), процедура для #3115. Статус: **подготовлено, не замерено**.
Дефолт `-z` остаётся 1024 — менять его без замера CPU на Pi issue запрещает.

## 1. Что на самом деле значат `-z`, `-Z` и период jackd

Первоисточник — исходники SuperCollider, тег `Version-3.13.0` (в `develop` на
28.09.2026 та же логика, номера строк сдвинуты на ±1). Строки, помеченные
(develop), сверялись только с `develop`. Какая версия `supercollider-server` стоит в образе
(`docker/vision/supercollider/Dockerfile`, `FROM ubuntu:26.04`) — **не
проверено**: на Pi выполнить `docker exec supercollider scsynth -v`.

| Факт | Где |
|---|---|
| `-z <block-size>` → `options.mBufLength` (размер блока обработки, округляется вверх до степени 2), дефолт 64 | `server/scsynth/scsynth_main.cpp:69,226-229`, `include/server/SC_WorldOptions.h:48` (develop) |
| `-Z <hardware-buffer-size>` → `mPreferredHardwareBufferFrameSize` | `scsynth_main.cpp:70,230-233` |
| JACK-драйвер берёт размер колбэка у jackd: `*outNumSamples = jack_get_buffer_size(mClient)`; `mPreferredHardwareBufferFrameSize` в `SC_Jack.cpp` не встречается ни разу (используется только в CoreAudio/PortAudio-драйверах, `SC_CoreAudio.cpp:705,2255` (develop)) | `server/scsynth/SC_Jack.cpp:259` |
| На каждый JACK-колбэк scsynth считает `numBufs = numSamples / mBufLength` блоков подряд | `SC_Jack.cpp:401-403, 430` |
| `-D <load synthdefs? 1 or 0>` — загружать ли SynthDef'ы с диска при старте (НЕ realtime) | `scsynth_main.cpp` Usage + `case 'D'` → `mLoadGraphDefs` |

URL: <https://github.com/supercollider/supercollider/blob/Version-3.13.0/server/scsynth/SC_Jack.cpp>,
<https://github.com/supercollider/supercollider/blob/Version-3.13.0/server/scsynth/scsynth_main.cpp>.

Выводы для нашего `start_supercollider.sh`:

1. Аппаратный буфер определяет `jackd -p 1024` (он и должен совпадать с
   `period_size` dmix в `asound.conf`). `-Z` при `-H jack` ни на что не влияет.
2. `-z 1024` задаёт размер **блока**, т.е. control rate. Старый комментарий
   «`-z` = period_size dmix (JACK требует совпадения)» был неверен — исправлен.
3. Ограничение: `-z` должен делить период jackd. Если `-z` > 1024, то
   `numBufs = 0` и сервер ничего не считает (тишина). Поэтому допустимы
   только 64/128/256/512/1024.
4. Попутно: комментарий «`-D 0` — отключить realtime scheduling» тоже был
   неверен (`-D 0` = не грузить synthdefs с диска). Поведение не меняли,
   только комментарий.

## 2. Control rate по block size при 16 кГц

| `-z` | control rate | шаг `.kr` | блоков на JACK-колбэк (1024) | Найквист для `.kr`-модулятора |
|---|---|---|---|---|
| 1024 | 15.6 Гц | 64 мс | 1 | 7.8 Гц |
| 512 | 31.3 Гц | 32 мс | 2 | 15.6 Гц |
| 256 | 62.5 Гц | 16 мс | 4 | 31.3 Гц |
| 128 | 125 Гц | 8 мс | 8 | 62.5 Гц |
| 64 | 250 Гц | 4 мс | 16 | 125 Гц |

Задержка (latency) от `-z` не зависит: она задаётся периодом jackd
(1024 × 3 периода / 16 кГц).

## 3. Что затронуто (по коду, не прослушано)

**Наши патчи** (`src/rob_box_voice/rob_box_voice/core/renardo_synthdef_patches.py`):
- `tb303`: `filtEnv = EnvGen.kr(...)` с `dec` от 0.08 с → 1–2 шага по 64 мс,
  плюс `Lag.kr(..., 0.01)` на `baseFreq`/`filtStart`/`filtPeak` — лаг 10 мс
  короче шага, т.е. по факту ступенька.
- `organ`: `gate = EnvGen.kr(...)` с `atk` от 0.01 с — атака органа
  квантуется до 64 мс; `Lag.kr` на ширину/фильтр.
- `brass`: `Lag.kr(freq, rate)`.
- `fuzz`: модулятор уже `.ar` (#3008) — не затронут, `Lag.kr(cutoff, 0.05)` затронут.

**Кастомные** (`docker/vision/voice_assistant/custom_synthdefs/*.scd`):
`imperialbrass, marchstrings, retrobass, strangerarp, strangerbrass,
strangerpulsepad, supersawlead, warmpad` — `Lag.kr` фильтра и
`SinOsc.kr(rate, 0, vib)` (вибрато: при `rate` > 7.8 Гц — алиасинг на
15.6 Гц control rate). `masterfilter/masterlimiter` — `Lag.kr(gain, lag)`
мастер-фейдера (ступеньки громкости при смене gain).

**Renardo** (проверено по `renardo_lib` 0.9.13 из PyPI; в образе версия
не закреплена, `requirements.txt: renardo-lib`): в 278 `.scd` синтов/эффектов
(без `tmp_code/`) 210× `In.kr`, 27× `LFNoise1.kr`, 26× `SinOsc.kr`,
25× `Line.kr`, 24× `EnvGen.kr`, 11× `LFPulse.kr` и т.д. Заметные эффекты: `chop`
(`LFPulse/LFTri/LFSaw.kr(chop / sus)` — при частоте > 7.8 Гц алиасинг),
`swell` (`EnvGen.kr`), `bpnoise` (`LFNoise1.kr`), `chorus` (`LFPar.kr`).
`lpf` — это `RLPF.ar(osc, lpf, lpr)`, `spf` — `EnvGen.ar`: от `-z` не зависят.

**НЕ связано с `-z` (поправка к тексту issue):** `lpf=linvar(...)`.
`linvar` — Python-объект `TimeVar`; Renardo вычисляет его значение
**на каждое событие** (`Players.py:1394-1398`, renardo_lib 0.9.13: `if isinstance(item, TimeVar):
item = item.now()`) и шлёт в `/s_new` константой. Ступеньки свипа —
по нотам, а не по control rate; уменьшение `-z` их не уберёт.

Начало нот `-z` не квантует: `makeSound` пишет в выход через `OffsetOut`
(`renardo_lib/SynthDefManagement/sclang_code/sceffects/makeSound.scd`), а scsynth выставляет
`mSampleOffset` по таймстемпу бандла (`SC_Jack.cpp:449-458`).

## 4. CPU trade-off

Меньше блок → больше накладных на блок: вызов calc-функции каждого UGen
и обход дерева нод происходят `16000 / z` раз в секунду вместо 15.6, а все
`.kr`-UGen'ы считаются чаще в `1024 / z` раз. Аудио-UGen'ы (основная
нагрузка) обрабатывают то же число сэмплов. Насколько это дорого на
Vision Pi — **неизвестно, поэтому и нужен замер**. Компромисс 256 (62.5 Гц) уже
убирает ступеньки у огибающих ≥ 50 мс.

## 5. Процедура замера на Pi (для #3115)

Требует: ветку с этим коммитом на Pi (`start_supercollider.sh`
монтируется bind-mount'ом, пересборка образа НЕ нужна).

```bash
ssh ros2@10.1.1.21
cd ~/rob_box_project && git fetch && git checkout <ветка #3114> && git pull
docker exec supercollider scsynth -v           # версия SC для отчёта
bash scripts/music/scsynth_block_bench.sh --dry-run
bash scripts/music/scsynth_block_bench.sh --sizes "1024 512 256 128 64" \
     --duration 60 --out /tmp/scsynth_block_bench.tsv 2>&1 | tee /tmp/scsynth_block_bench.log
docker logs supercollider --tail 50            # после восстановления: "-z 1024"
```

Что делает скрипт: на время прогона **останавливает voice-assistant**
(робот молчит), для каждого размера пересоздаёт контейнер `supercollider`
с `SCSYNTH_BLOCK_SIZE=<N>` (тот же образ, `--pull never`), проверяет по
`/proc/<pid>/cmdline`, что `-z` применился, меряет CPU scsynth/jackd
в idle и под нагрузкой (6 плееров `tb303,organ,fuzz,pluck,bass,warmpad`
из `scripts/music/scsynth_bench_stimulus.py`, молча — в приватные шины),
берёт avg/peak CPU из OSC `/status` scsynth и считает строки `xrun` в
`docker logs`. По `trap` на выходе (в т.ч. Ctrl-C) возвращает исходный
block size и запускает voice-assistant. Репо и `.env` не меняет.

Ограничение стимула: нет эффект-синтов Renardo и `makeSound`, поэтому
абсолютный CPU на реальном треке выше; для сравнения размеров между собой
годится. Если нужна проверка «на живом треке» — после выбора размера
выставить его (п. 6) и прогнать обычный 6-плеерный трек через музыкальный
tool, снимая `top -b -d 1 -n 60 -p $(pgrep -x scsynth)` на хосте.

### Правило решения

Взять **наименьший** `-z` из {64, 128, 256}, для которого одновременно:

1. `xruns` = 0 за весь прогон;
2. `load_sc%` (под нагрузкой) ≤ 1.5 × `load_sc%` при 1024 **и** ≤ 50 %
   одного ядра;
3. `sc_peakCPU` (из `/status`) < 60.

Если ни один не проходит — оставить 1024 и закрыть #3114 с замером как
evidence. Затем прослушать tb303/organ/fuzz до/после (запись) — это
п. 3 issue, скриптом не автоматизировано.

## 6. Переключение после замера (один флаг)

- Временно на Pi: `SCSYNTH_BLOCK_SIZE=256` в `docker/vision/.env`, затем
  `docker compose up -d --force-recreate supercollider` и
  `docker compose restart voice-assistant`.
- Насовсем (PR): поменять дефолт `1024` в трёх местах —
  `start_supercollider.sh` (`SCSYNTH_BLOCK_SIZE_DEFAULT` и `${...:-N}`),
  `docker-compose.yaml` (`${SCSYNTH_BLOCK_SIZE:-N}`) и `EXPECTED_DEFAULT`
  в `src/rob_box_voice/test/unit/core/test_issue_3114_scsynth_block_size.py`
  (тест проверяет, что все три совпадают). Там же добавить assert
  `EXPECTED_DEFAULT <= 256` — acceptance issue.
