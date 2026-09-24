# ADR-0129 — fuzz SynthDef: анти-алиасинг + step → lin + `lpf`-арг на 16 kHz scsynth

**Дата:** 2026-09-24
**Статус:** Accepted — реализуется в рамках issue #3008
**Автор:** architect-agent (фаза S13, волна voice-music-fuzz-16khz)
**Родители:** Issue [#3008](https://github.com/krikz/rob_box_project/issues/3008) (live 24.09.2026, трек «nirvana rape me» через compose_music)
**Связанные:** ADR-0000 (архитектурные принципы), ADR-0126 (renardo-samples-host-delivery), `src/rob_box_voice/rob_box_voice/core/renardo_synthdef_patches.py` (тот же патч-канал, что fix_brass_scd.py и fix_tb303_scd.py), `docker/vision/voice_assistant/custom_synthdefs/masterfilter.scd` (паттерн «срез от SampleRate», правило 3), `src/rob_box_voice/test/unit/core/test_music_stack_validation.py` (регресс-тесты на патчи), `src/rob_box_mcp_tools/test/test_critical_synths_cover_palette.py` (инвариант палитры)
**Реализуют:** `src/rob_box_voice/rob_box_voice/core/renardo_synthdef_patches.py` (`patch_fuzz_scd_content` + регистрация в `apply_renardo_synthdef_patches`)

## 1. Контекст

Live-сессия 24.09.2026 15:00 UTC: `compose_music` сгенерировал трек «nirvana rape me»
(`bass_synth=fuzz`, `lead_synth=pluck`). Бас-линия `p1 >> fuzz(...)` «пердела» и
щёлкала. Перебор `amplify`, `amp`, `oct=+2` не помогли; `oct=-2` починил звук, но
это не опция — палитра промпта рассчитана на `oct>=0`.

### 1.1 Корень проблемы — три причины складываются

Источник `renardo_lib/SynthDefManagement/sclang_code/scsynth/fuzz.scd`:

```supercollider
SynthDef.new(\fuzz, {
  |amp=1, sus=1, pan=0, freq=0, vib=0, fmod=0, rate=0, bus=0, blur=1,
   beat_dur=1, atk=0.01, decay=0.01, rel=0.01, peak=1, level=0.8|
  ...
  osc=LFSaw.ar(LFSaw.kr(freq, 0, freq, (freq * 2)));   // (1) пила + пилообразная FM
  env=EnvGen.ar(Env(times: [(sus * 0.8), 0.01],
                      levels: [(amp * 1), (amp * 1), (amp * 0.01)],
                      curve: 'step'), doneAction: 0); // (2) step-огибающая
  ...
}).add;
```

1. **Алиасинг.** `LFSaw.ar` — идеальная пила, спектр уходит в `∞`. Пилообразная
   FM (`LFSaw.kr` модулирует фазу опорной пилы) уплотняет спектр ещё сильнее.
   На 44.1/48 kHz это терпимо (мастер-фильтр `masterfilter.scd` режет выше
   Найквиста), но scsynth на роботе работает на **16 kHz** (потолок ReSpeaker
   UAC1.0), Найквист — 8 kHz. Обертоны выше 8 kHz зеркалятся обратно в
   слышимую полосу негармоничным «пердением». `oct=-2` сдвигает основной
   тон вниз (с учётом внутреннего `freq/2` — на 8×): обертоны уходят на
   номера гармоник выше, амплитуда пилы падает как `~1/n`, и алиасинг
   оказывается ниже слышимости — отсюда и «вылечилось».
2. **Step-огибающая.** `curve:'step'` означает жёсткий on/off между уровнями
   без атаки/релиза. На каждой ноте — щелчок (это тот же эффект, что был у
   organ/tb303 в issues #2716/#2747 и лечился `Env.perc`/`Env.asr` +
   `HPF.ar(LeakDC.ar(...), 35)`). Аргументы `atk/decay/rel` из шапки
   **игнорируются** — огибающая захардкожена.
3. **`lpf=` — no-op.** В шапке нет ни `lpf`, ни `hpf`. Renardo шлёт
   `lpf=523.3` как OSC-аргумент, синтезатор его не читает. Поэтому
   «починить фильтром» нельзя — фильтра в SynthDef нет.

### 1.2 Альтернативы, которые не подошли

- **BLSaw / DPW4 из SC-Plugins** — band-limited осциллятор в стандартной
  поставке. Чище пила, но требует регистрации плагина в sclang, а это
  ещё один слой «есть/нет на роботе», который надо поддерживать. KISS:
  тот же `LFSaw`, но срез ниже Найквиста прямо в SynthDef — паттерн
  уже есть в `masterfilter.scd` (правило 3, стр. 65–68).
- **Полный запрет `bass_synth=fuzz` в arranger** — да, в ADR это
  зафиксировано как fallback (см. §5), но это **уменьшает** палитру
  промпта и нарушает инвариант
  `test_critical_synths_cover_palette.py` («палитра ⊆ CRITICAL_SYNTHS»).
  Если синтезатор можно починить — чинить, а не выбрасывать.
- **Перевод scsynth на 48 kHz** — отдельный большой рефактор (P7 из
  аудита 30.08), не входит в scope #3008.

## 2. Решение

Патч `fuzz.scd` через **тот же механизм**, что `brass.scd` / `organ.scd`
/ `tb303.scd` — функция `patch_fuzz_scd_content` в
`renardo_synthdef_patches.py`, регистрация в `apply_renardo_synthdef_patches`.
Файл переписывается на известную стабильную версию при старте voice-assistant
(см. `docker/vision/voice_assistant/fix_brass_scd.py`).

### 2.1 Новое тело SynthDef

```supercollider
SynthDef.new(\fuzz, {
    |amp=1, sus=1, pan=0, freq=0, vib=0, fmod=0, rate=0, bus=0, blur=1,
     beat_dur=1, atk=0.01, decay=0.01, rel=0.01, peak=1, level=0.8,
     lpf=4000|                                  // ← новый арг (default ≈ центр речи)
    var osc, env, nyquist, cutoff, baseFreq;
    sus = sus * blur;
    baseFreq = Lag.kr(In.kr(bus, 1).max(20), 0.01);   // ← (a) anti-click на смене ноты
    freq = [baseFreq, baseFreq + fmod];
    freq = (freq / 2);
    amp = (amp / 6);

    // ── Правило 1: NaN/Inf не должны выйти из SynthDef ─────────────────
    // (аналогично masterfilter.scd §правило 1, чтобы один сломанный вход
    // не убивал всю пачку)
    osc = LFSaw.ar(LFSaw.kr(freq, 0, freq, (freq * 2)));
    osc = CheckBadValues.ar(osc, 0, 0);            // пост-режим 0: тихо
    osc = Select.ar(osc, [DC.ar(0), DC.ar(0), DC.ar(0), DC.ar(0)]);
    // ВНИМАНИЕ: Select.ar на bool-результат отбрасывает весь сигнал,
    // правильнее через `if(osc==0)` или `(osc * (1-bad)).sum` — см.
    // обновлённый вариант в §2.2.

    // ── Правило 3 (masterfilter.scd): срез от SampleRate ────────────────
    // На 16 kHz потолок РеСпикера — 8 kHz. Срез ниже Найквиста чистит
    // зеркальные обертоны пилы.
    nyquist = SampleRate.ir * 0.5;
    cutoff = min(lpf, nyquist * 0.45);             // 0.45 = запас от Найквиста
    osc = LPF.ar(osc, Lag.kr(cutoff, 0.05));       // (b) Lag — без щелчка при смене lpf=

    // ── Правило 2 (issue #2716): реальные atk/rel, не 'step' ───────────
    env = EnvGen.ar(
        Env([0, amp, amp, 0], [atk.max(0.005), sus.max(0.05), rel.max(0.05)],
            curve: -4),
        doneAction: 0
    );
    osc = (osc * env);
    osc = Mix(osc) * 0.5;
    osc = Pan2.ar(osc, pan);
    ReplaceOut.ar(bus, osc)
}).add;
```

### 2.2 Защита от NaN — финальный вариант

В §2.1 «если CheckBadValues плохо» — это место прокомментировано, но
финальная версия в коде использует **корректный шаблон** из
`masterfilter.scd`:

```supercollider
bad = CheckBadValues.ar(osc, 0, 0);             // 0=ok, 1=NaN, 2=Inf, 3=denorm
osc = Select.ar(bad > 0, [osc, DC.ar(0)]);      // NaN/Inf/denorm → тишина
```

Это ровно то, что делает `masterfilter.scd:59` (`Select.ar(bad, [sig, ...])`,
где `bad==0` — ок, остальные — нули).

### 2.3 Контракт патча

| Свойство                              | Старое (`renardo_lib` upstream) | Новое (патч) |
|---------------------------------------|--------------------------------|--------------|
| Осциллятор                            | LFSaw(LFSaw.kr) сырой          | LFSaw + CheckBadValues + LPF(SampleRate) |
| Огибающая                             | `Env(..., curve:'step')`       | `Env(atk, sus, rel, curve:-4)` |
| Аргумент `lpf=`                       | нет                            | есть, default=4000 |
| Аргумент `atk`/`rel`                  | есть, но игнорируются          | есть, реально используются |
| `Lag.kr` на входе freq                | нет                            | есть (anti-click) |
| Поведение на `oct>=0` 16 kHz          | «пердит» + щелчки              | чистый fuzz |
| Поведение на `oct=-2`                 | работает                        | работает (плюс защита) |
| `lpf=523.3` из compose_music          | no-op                          | реально режет |

## 3. Структура кода

В `src/rob_box_voice/rob_box_voice/core/renardo_synthdef_patches.py`:

```python
FUZZ_SYNTHDEF = """SynthDef.new(\\fuzz, {
    |... lpf=4000|                       // ← новый арг
    ...
}).add;
"""

def patch_fuzz_scd_content(content: str) -> str:
    if "\\fuzz" not in content and "fuzz" not in content:
        return content
    if content == FUZZ_SYNTHDEF:
        return content
    return FUZZ_SYNTHDEF
```

И в `apply_renardo_synthdef_patches`:

```python
if scd_file.name == "fuzz.scd":
    updated = patch_fuzz_scd_content(updated)
```

`fix_brass_scd.py` уже вызывает `apply_renardo_synthdef_patches` — никаких
новых entrypoints не нужно.

## 4. Регресс-тесты

Добавляются в `test_music_stack_validation.py` (там же, где тесты на
brass/organ/tb303):

1. `test_patch_fuzz_scd_content_replaces_step_envelope_with_real_attack_release`
   — патченный вариант содержит `curve: -4` и `atk.max(0.005)`, **не**
   содержит `curve: 'step'`.
2. `test_patch_fuzz_scd_content_adds_anti_aliasing_lowpass` — содержит
   `LPF.ar` и `SampleRate.ir`, **не** содержит голого `LFSaw.ar` без
   фильтра (как минимум — рядом с LFSaw идёт LPF).
3. `test_patch_fuzz_scd_content_adds_lpf_arg` — в шапке есть `lpf=4000`,
   в теле — `LPF.ar(osc, ..., lpf)`.
4. `test_apply_renardo_synthdef_patches_patches_fuzz_file_in_place` —
   аналог теста для tb303: записать «битый» `fuzz.scd` в `tmp_path`,
   вызвать `apply_renardo_synthdef_patches`, проверить, что файл
   перезаписан и в `patched_files` появилось `'fuzz.scd'`.

`test_critical_synths_cover_palette.py` — `fuzz` уже в палитре промпта и
в `CRITICAL_SYNTHS`, ничего не трогаем.

## 5. Fallback (если патч не зайдёт)

Если на live-роботе `fuzz` всё равно ведёт себя плохо (например,
пользовательский код шлёт странные `lpf=`-значения выше Найквиста),
в `src/rob_box_mcp_tools/rob_box_mcp_tools/core/arranger.py` запретить
`synth=fuzz` при `oct>=0` — по аналогии с уже существующим ограничением
на `_cap_amp` / `max_oct` в `music.py` (см. `masterfilter.scd` §РАЗМЕЩЕНИЕ).

Это **не делается сейчас** — патч SynthDef дешевле и не уменьшает
палитру.

## 6. Чего НЕ делаем

- Не переписываем arranger/harmonize до подтверждения фикса на роботе.
- Не трогаем `masterfilter.scd` — он и так режет правильно (правило 3).
- Не вводим зависимость от SC-Plugins (BLSaw/DPW4).
- Не переключаем scsynth на 48 kHz — это P7 из аудита 30.08, отдельная задача.

## 7. Метрика приёмки

- `pytest -q src/rob_box_voice/test/unit/core/test_music_stack_validation.py` — зелёный.
- `pytest -q src/rob_box_mcp_tools/test/test_critical_synths_cover_palette.py` — зелёный.
- `flake8` для изменённых файлов — без новых предупреждений.
- На live-роботе после merge: `bass_synth=fuzz`, `oct=0`, `dur=0.5`,
  `scale=chromatic` — играет без «пердения» и щелчков. Проверка
  трека «nirvana rape me» через `compose_music` (e2e-process после merge).
