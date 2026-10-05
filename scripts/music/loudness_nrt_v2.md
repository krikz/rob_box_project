# loudness_nrt_v2 — замер громкости и полос слоёв (ADR-0152 §3.1, issue #3422)

`loudness_nrt_v2.py` рендерит программу Renardo в `scsynth -N` на 16 кГц и меряет dB RMS и доли энергии
< 150 / 150–2000 / ≥ 2000 Гц по одному слою. Числа идут в `knowledge.LAYER_MEASURED_DB`, `AMP_EXPONENT`
и `LAYER_BANDS`. Это модель, а не замер на роботе (`knowledge.LOUDNESS_SOURCE`).

## Где запускать

На **katana**. Робот для этого не нужен, на Windows SuperCollider нет. `loudness_nrt_v2.sh` поднимает
одноразовый контейнер (`docker run --rm`) из образа `voice-assistant` локального реестра:

- в образе есть `sclang` и `scsynth`, numpy и renardo_lib 0.9.13 **с патчами робота** (`fix_brass_scd.py`).
  Получаются те же SynthDef-ы, что грузит робот. В образе `supercollider` есть только `scsynth`, `sclang` в нём нет;
- образ собран под arm64 и на katana (x86_64) работает через qemu-binfmt. Один слой рендерится примерно за 17 с;
- сэмплы `0_foxdot_default` лежат в `$WORK/samples`. Харнесс сам докачивает папки нужных символов
  (`X`, для `--track` ещё `-` и `*`) с `collections.renardo.org`;
- `custom_synthdefs/` и `masterfilter.scd` берутся из чекаута. Мастер: `gain 0.5`, `dyn 0`.

```bash
# на katana, из любого чекаута репо (или REPO=/путь/к/чекауту)
scripts/music/loudness_nrt_v2.sh --regress                 # рамка 29.09 против таблицы, ±1.5 дБ; exit 1 — FAIL
scripts/music/loudness_nrt_v2.sh --sweep lead sitar epiano # строки таблицы: dB, наклон amp, полосы
scripts/music/loudness_nrt_v2.sh --track 7                 # трек v2: слой против Part.level_db модели
```

`WORK` (по умолчанию `/tmp/nrt_v2`) — каталог рендеров (`out/*.wav`, `out/*.osc`, `out/defs`) и сэмплов.
Файлы в нём создаёт root из контейнера. `IMAGE` переопределяет образ.

Без контейнера (свой SuperCollider и распакованное колесо):
`pip download renardo_lib==0.9.13 --no-deps` и `python -m zipfile -e <whl> rl`, затем
`python scripts/music/loudness_nrt_v2.py --renardo rl/renardo_lib --samples <0_foxdot_default> --sclang … --scsynth …`.
В колесе нет патчей робота (brass/organ/tb303/fuzz), поэтому эти синты звучат иначе, чем на роботе.

## Рамка 29.09

`loudness_frame_0929.json` содержит программы `club_loudness_nrt.py --sweep` старого харнесса на коммите
`1835264a5`, после санитайзера: seed 0, `dj_dave_32`, 124 BPM, тоники A#, D, A, E, G, один слой с постоянным
`amp`. Из неё сняты числа 29.09. Новые синты меряются в той же рамке, только с подменой синта, поэтому новые
строки таблицы стоят в той же шкале, что и старые. dB по тонике усредняется арифметически. Наклон `p` считается
по первой тонике на уровне замера и на его половине: `p = Δ / 6.02`.

## Пайплайн

1. `render.events.program_events`: семантика `Players.py` (`amp·amplify`, `sus = dur`, ступень → MIDI).
2. Партитура `score()` строит бандл ноты так же, как `ServerManager.get_bundle`: `/g_new`, затем `startSound`
   (`rate` = частота, `sus·8`), синт, эффекты `hpf, lpf, echo, room` (только ненулевые), `makeSound`.
3. `scsynth -N`: 16 кГц, float, стерео. Замер идёт по левому каналу на окне формы (в `--track` — по секциям роли).

Не моделируются `Clock.latency`, джиттер, ReSpeaker/ALSA и стерео-голоса `Mix.stereo`: рамка 29.09 моно.
Слои `loop()` (DJ_Dave) в `--track` пропускаются. Бочки по ADR-0152 §3.1 меряются на роботе.

## Тесты

`python -m pytest scripts/music/test_loudness_nrt_v2.py -v --no-cov` проверяет партитуру из
`program_events` и разбор полос без SuperCollider. CI этот каталог не собирает.
Таблицу `LAYER_BANDS` проверяет `src/rob_box_music/test/test_loudness_tables.py`.
