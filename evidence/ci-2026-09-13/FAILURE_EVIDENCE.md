# CI failure evidence — runs 34779436302, 34780571796, 34781633844

**Source CI runs** (all on develop HEAD `4ab3a0a59fa5534fa67b1ecd3349c54c9ad2e3e8`,
workflow `L: E2E Voice Test`, runner `rob-box-e2e-1`):

| Run id | Started UTC | Duration | Conclusion | Scenario (TRIGGERED_BY_REASON) |
| --- | --- | --- | --- | --- |
| 34779436302 | 2026-09-13 20:01:06 | 20m 51s | **failure** | night marathon act 2: Знакомство и прогрев голосов |
| 34780571796 | 2026-09-13 20:22:49 | ~18m 15s | **failure** | night marathon act 3: Кто там бубнил на фоне |
| 34781633844 | 2026-09-13 20:43:20 | ~17m 38s | **failure** | night marathon act 4: Робот учится говорить |

All three runs returned the same GitHub Action `conclusion=failure` and were terminated
by the harness after `GATE-1` aggregate acceptance check (`E2E_GATE1_FAIL` /
`E2E_VERDICT FAIL`). Raw logs are committed alongside this report:

```
evidence/ci-2026-09-13/run_34779436302.log   (1322 lines)
evidence/ci-2026-09-13/run_34780571796.log   (1431 lines)
evidence/ci-2026-09-13/run_34781633844.log   (1400 lines)
```

## Tag-mapping decision (per body requirement)

Body asked for sections tagged `music`, `compose_music`, `lead_synth`, or `detect_key`.
A direct grep for those literal tags across the 3 runs found:

* `compose_music` — appears **once** in run `34780571796`, as part of the
  `n310_execute_backlog_lru` pattern check (see Run 2 below).
* `lead_synth`, `detect_key` — **not present** in any of the 3 run logs.
* The closest umbrella term in the harness is `RULE #MUSIC`, referenced by every
  GATE-1 failure message as a possible-cause hint (verbatim text below).

Therefore rows are classified by the closest music / voice-control subsystem that
the failing scenario belongs to:

| Run | Subsystem | Why |
| --- | --- | --- |
| 34779436302 | voice-registration (music-RULE hint in GATE-1) | `night_marathon_act2` is the registration/warmup phase; final GATE-1 fail cites `register_speaker` and references `RULE #MUSIC` |
| 34780571796 | music-control (compose_music / get_music_state) | `night_marathon_act3` is the backlog/music scenario; uses `compose_music` pattern and ends on `n313_silence_restored` expecting `get_music_state` |
| 34781633844 | voice-core (RULE #MUSIC hint in GATE-1, but tool actually missed is `list_tts_voices` / `set_speed`) | `night_marathon_act4` is the voice-core suite; final GATE-1 message text still points to `RULE #MUSIC` as the suspect |

## Summary table — one row per (run, failing scenario)

| Run | Scenario id | Scenario name | Step / failure | Expected (per harness) | Actual | Stacktrace | Harness verdict |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 34779436302 | act2 | n207_recall_sasha — "Робот, как меня зовут и что ты про меня уже знаешь?" | acceptance keyword check | keyword `['Саш']` present in voice cycle | keyword missing (voice cycle OK, but TTS did not name the speaker) | none (bash harness, no Python traceback) | `ACCEPTANCE[n207_recall_sasha]: ❌` → `STEP ❌ проверка не прошла после retry` → `E2E_STEP n207_recall_sasha FAIL` |
| 34779436302 | act2 | n209_recall_boris — "Робот, а про меня что помнишь?" | acceptance keyword check | keyword `['Борис\|Спартак\|пицц']` present | keyword missing | none | `ACCEPTANCE[n209_recall_boris]: ❌` → `E2E_STEP n209_recall_boris FAIL` |
| 34779436302 | act2 | n211_who_do_you_know — "Робот, перечисли всех, кого ты сегодня запомнил по голосу." | acceptance keyword check | keyword `['Саш']` present | keyword missing | none | `ACCEPTANCE[n211_who_do_you_know]: ❌` → `E2E_STEP n211_who_do_you_know FAIL` |
| 34779436302 | act2 | GATE-1 (aggregate, after all 11 steps) | tool-call enforcement (soft-fail) | `register_speaker` tool invoked at least once during the run | `register_speaker` never invoked (TTS x14, speak_text x301, LLM gave verbal-only answer) | none | `GATE-1 ❌` → `GATE-1 ❌ aggregate acceptance FAIL` → `E2E_VERDICT FAIL` |
| 34780571796 | act3 | n310_execute_backlog_lru — "Робот, ну ты слышал, что просили. Сделай." | pattern check on backlog flush | pattern `[backlog] flushed to LLM backlog_handled=true` AND pattern `execute_music_code\|compose_music\|set_vibe_preset\|generate_music` | **PASS** — both patterns found | none | `STEP n310_execute_backlog_lru: ✅ паттерны найдены` → `ACCEPTANCE ✅ all checks passed` (this step is green; included for completeness because it is the only `compose_music` occurrence) |
| 34780571796 | act3 | n313_silence_restored — "Робот, теперь тихо?" | acceptance tool-call check | tool `get_music_state` invoked at least once during the run | `get_music_state` never invoked (TTS x12, speak_text x159, verbal-only answer) | none | `ACCEPTANCE[n313_silence_restored]: ❌` → `STEP ❌` → `E2E_STEP n313_silence_restored FAIL` → `GATE-1 ❌ aggregate acceptance FAIL` → `E2E_GATE1_FAIL` → `E2E_VERDICT FAIL` |
| 34781633844 | act4 | n401_list_voices — "Робот, какими голосами ты умеешь говорить? Перечисли, что у тебя есть." | acceptance tool-call check | tool `list_tts_voices` invoked | tool never invoked (TTS x36, speak_text x909, verbal-only answer) | none | `ACCEPTANCE[n401_list_voices]: ❌` → `STEP ❌` → `E2E_STEP n401_list_voices FAIL` |
| 34781633844 | act4 | n410_speed_up — "Робот, ладно, хватит тянуть, теперь наоборот — тараторь." | acceptance tool-call check | tool `set_speed` invoked | tool never invoked | none | `ACCEPTANCE[n410_speed_up]: ❌` → `STEP ❌` → `E2E_STEP n410_speed_up FAIL` |
| 34781633844 | act4 | n412_skazka_multivoice — multi-voice fairy tale | acceptance pattern check (retry) | pattern `voice_used` present in logs | pattern missing both attempts | none | `ACCEPTANCE ✅` then `STEP ❌ проверка не прошла — retry 1/1` → on retry `STEP ❌ проверка не прошла после retry` → `E2E_STEP n412_skazka_multivoice FAIL` |
| 34781633844 | act4 | GATE-1 (aggregate, after all 12 steps) | tool-call enforcement (soft-fail) | tool `list_tts_voices` invoked during run | never invoked (GATE-1 message cites TTS x36, speak_text x909) | none | `GATE-1 ❌` → `GATE-1 ❌ aggregate acceptance FAIL` → `E2E_GATE1_FAIL` → `E2E_VERDICT FAIL` |

> Note on stacktraces: the e2e harness is a bash script (`/tmp/e2e_voice_test.sh`
> invoked via `RUN_CMD` at the top of each log). It does not emit Python
> tracebacks for these particular failure modes — the failures are reported as
> plain `❌` lines from acceptance / GATE-1 checks. There are no `Traceback`
> blocks in any of the 3 logs.

## Per-run verbatim evidence (10–30 lines per failure)

Line numbers below are 1-indexed line numbers in the raw `.log` file.

### Run 34779436302 — `night_marathon_act2_acquaintance_v1`

**Scenario header** (run_34779436302.log:46):

```
2026-09-13T20:01:15.1379102Z   TRIGGERED_BY_REASON: night marathon act 2: Знакомство и прогрев голосов
```

**n207_recall_sasha — keyword `['Саш']` missing** (run_34779436302.log:390–397):

```
2026-09-13T20:14:42.1773079Z >>> STEP n207_recall_sasha: cmd_n207_recall_sasha.wav отсутствует — повторный синтез (cleanup-resilience)
2026-09-13T20:14:59.3803273Z >>> STEP n207_recall_sasha: робот молчит 15s — команду можно играть
2026-09-13T20:14:59.5921192Z >>> STEP n207_recall_sasha: PLAY attempt 1/3
2026-09-13T20:15:45.4427754Z >>> STEP n207_recall_sasha: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)
2026-09-13T20:15:45.9869687Z >>> TRANSCRIPT[n207_recall_sasha]: STT не вернул фразу (нет '✅ ПРИНЯТО')
2026-09-13T20:15:46.5908000Z >>> ACCEPTANCE[n207_recall_sasha]: ❌ expected keywords missing in logs: ['Саш']
2026-09-13T20:15:46.5910205Z >>> STEP n207_recall_sasha: ❌ проверка не прошла после retry (см. /tmp/e2e_v2_34779436302/acceptance.json)
2026-09-13T20:15:46.5911237Z E2E_STEP n207_recall_sasha FAIL
```

**n209_recall_boris — keyword `['Борис|Спартак|пицц']` missing** (run_34779436302.log:410–417):

```
2026-09-13T20:17:07.8765977Z >>> === STEP n209_recall_boris (safe=n209_recall_boris): voice=ermil text="Робот, а про меня что помнишь?" ===
2026-09-13T20:17:30.7038357Z >>> STEP n209_recall_boris: PLAY attempt 1/3
2026-09-13T20:18:14.8428196Z >>> STEP n209_recall_boris: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)
2026-09-13T20:18:15.3472803Z >>> TRANSCRIPT[n209_recall_boris]: STT не вернул фразу (нет '✅ ПРИНЯТО')
2026-09-13T20:18:15.8232846Z >>> ACCEPTANCE[n209_recall_boris]: ❌ expected keywords missing in logs: ['Борис|Спартак|пицц']
2026-09-13T20:18:15.8233782Z >>> STEP n209_recall_boris: ❌ проверка не прошла после retry (см. /tmp/e2e_v2_34779436302/acceptance.json)
2026-09-13T20:18:15.8234248Z E2E_STEP n209_recall_boris FAIL
```

**n211_who_do_you_know — keyword `['Саш']` missing** (run_34779436302.log:430–437):

```
2026-09-13T20:19:42.3708378Z >>> === STEP n211_who_do_you_know (safe=n211_who_do_you_know): voice=anton text="Робот, перечисли всех, кого ты сегодня запомнил по голосу." ===
2026-09-13T20:19:59.6909692Z >>> STEP n211_who_do_you_know: PLAY attempt 1/3
2026-09-13T20:20:45.8912284Z >>> STEP n211_who_do_you_know: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)
2026-09-13T20:20:46.3937972Z >>> TRANSCRIPT[n211_who_do_you_know]: STT не вернул фразу (нет '✅ ПРИНЯТО')
2026-09-13T20:20:46.9154483Z >>> ACCEPTANCE[n211_who_do_you_know]: ❌ expected keywords missing in logs: ['Саш']
2026-09-13T20:20:46.9155256Z >>> STEP n211_who_do_you_know: ❌ проверка не прошла после retry (см. /tmp/e2e_v2_34779436302/acceptance.json)
2026-09-13T20:20:46.9155742Z E2E_STEP n211_who_do_you_know FAIL
```

**Aggregate GATE-1 — `register_speaker` tool never invoked** (run_34779436302.log:439–444):

```
2026-09-13T20:20:47.4886423Z [hint] GATE-1 soft-fail: voice-cycle OK but tool skipped — register_speaker
2026-09-13T20:20:47.5112645Z >>> GATE-1: ❌ expected tool calls not invoked during run: register_speaker; voice cycle completed (TTS finished x14, speak_text x301) but expected tool call(s) skipped: register_speaker. LLM сделал verbal-only answer (RULE #MUSIC для stop_music / RULE #VOICE-MULTI для multi-voice могут не enforce). Проверь master_prompt_compact.txt и/или добавь explicit tool-call enforcement в LLM-system reminder.
2026-09-13T20:20:47.5114578Z >>> GATE-1: ❌ aggregate acceptance FAIL (см. /tmp/e2e_v2_34779436302/acceptance.json)
2026-09-13T20:20:47.5115140Z E2E_GATE1_FAIL
2026-09-13T20:20:47.5799214Z >>> WARN: /tmp/e2e_v2_34779436302/recording.wav не найден — audio_metrics.json не пишется (recorder не запустился?)
2026-09-13T20:20:47.5799522Z E2E_VERDICT FAIL
```

The same GATE-1 block is repeated 2 more times later in the log (lines 574–575
and 724–725), with the same verbatim message — the harness re-emits the verdict
three times, presumably once per scenario / after sub-step reporting.

### Run 34780571796 — `night_marathon_act3_bg_chatter_v1`

**Scenario header** (run_34780571796.log:46):

```
2026-09-13T20:22:49.3178746Z   TRIGGERED_BY_REASON: night marathon act 3: Кто там бубнил на фоне
```

**n310_execute_backlog_lru — the only `compose_music` pattern in any of the 3 logs** (run_34780571796.log:424–435):

```
2026-09-13T20:34:11.2686681Z >>> === STEP n310_execute_backlog_lru (safe=n310_execute_backlog_lru): voice=anton text="Робот, ну ты слышал, что просили. Сделай." ===
2026-09-13T20:34:28.6584811Z >>> STEP n310_execute_backlog_lru: PLAY attempt 1/3
2026-09-13T20:35:14.1331184Z >>> STEP n310_execute_backlog_lru: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)
2026-09-13T20:35:14.6687600Z >>> TRANSCRIPT[n310_execute_backlog_lru]: STT не вернул фразу (нет '✅ ПРИНЯТО')
2026-09-13T20:35:14.6992554Z >>> STEP n310_execute_backlog_lru: проверка паттернов (2): \[backlog\] flushed to LLM backlog_handled=true execute_music_code|compose_music|set_vibe_preset|generate_music
2026-09-13T20:35:15.2431049Z   PATTERN_OK: \[backlog\] flushed to LLM backlog_handled=true
2026-09-13T20:35:15.2471134Z   PATTERN_OK: execute_music_code|compose_music|set_vibe_preset|generate_music
2026-09-13T20:35:15.2472359Z >>> STEP n310_execute_backlog_lru: ✅ паттерны найдены
2026-09-13T20:35:15.7785211Z >>> ACCEPTANCE[n310_execute_backlog_lru]: ✅ all checks passed
2026-09-13T20:35:15.7786204Z >>> STEP n310_execute_backlog_lru: ✅ acceptance PASS
2026-09-13T20:35:15.7786565Z E2E_STEP n310_execute_backlog_lru OK
```

This is the only **PASSING** row in the table above and is included because it
is the only direct hit for the body-required tag `compose_music`. It demonstrates
that the backlog-flush → music-tool-call pipeline works end-to-end when asked
explicitly.

**n313_silence_restored — `get_music_state` tool never invoked** (run_34780571796.log:452–477):

```
2026-09-13T20:38:19.4685319Z >>> === STEP n312_stop_music (safe=n312_stop_music): voice=anton text="Робот, всё, спасибо, выключай музыку, курьер уехал." ===
2026-09-13T20:39:57.6140385Z >>> STEP n312_stop_music: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)
2026-09-13T20:39:58.7364514Z >>> ACCEPTANCE[n312_stop_music]: ✅ all checks passed
2026-09-13T20:39:58.7366396Z >>> STEP n312_stop_music: ✅ acceptance PASS
2026-09-13T20:39:58.7366912Z E2E_STEP n312_stop_music OK
2026-09-13T20:39:59.0006698Z >>> === STEP n313_silence_restored (safe=n313_silence_restored): voice=anton text="Робот, теперь тихо?" ===
2026-09-13T20:40:16.2727648Z >>> STEP n313_silence_restored: PLAY attempt 1/3
2026-09-13T20:41:00.2362419Z >>> STEP n313_silence_restored: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)
2026-09-13T20:41:00.7337072Z >>> TRANSCRIPT[n313_silence_restored]: STT не вернул фразу (нет '✅ ПРИНЯТО')
2026-09-13T20:41:01.2884899Z >>> ACCEPTANCE[n313_silence_restored]: ❌ expected tool calls not invoked: ['get_music_state']
2026-09-13T20:41:01.2886262Z >>> STEP n313_silence_restored: ❌ проверка не прошла после retry (см. /tmp/e2e_v2_34780571796/acceptance.json)
2026-09-13T20:41:01.2886760Z E2E_STEP n313_silence_restored FAIL
2026-09-13T20:41:01.9043208Z [hint] GATE-1 soft-fail: voice-cycle OK but tool skipped — get_music_state
2026-09-13T20:41:01.9269582Z >>> GATE-1: ❌ expected tool calls not invoked during run: get_music_state; voice cycle completed (TTS finished x12, speak_text x159) but expected tool call(s) skipped: get_music_state. LLM сделал verbal-only answer (RULE #MUSIC для stop_music / RULE #VOICE-MULTI для multi-voice могут не enforce). Проверь master_prompt_compact.txt и/или добавь explicit tool-call enforcement в LLM-system reminder.
2026-09-13T20:41:01.9271461Z >>> GATE-1: ❌ aggregate acceptance FAIL (см. /tmp/e2e_v2_34780571796/acceptance.json)
2026-09-13T20:41:01.9272000Z E2E_GATE1_FAIL
2026-09-13T20:41:01.9940176Z >>> WARN: /tmp/e2e_v2_34780571796/recording.wav не найден — audio_metrics.json не пишется (recorder не запустился?)
2026-09-13T20:41:01.9940596Z E2E_VERDICT FAIL
```

Same GATE-1 message is then repeated twice more (lines 641–645 and 825–829),
verbatim — the harness re-emits the verdict after each per-step reporting
block.

Adjacent **non-music** failures in this run (for completeness, also failure rows
that are not music-tagged but contribute to the verdict):

- `n302_bg_sasha`, `n303_bg_boris`, `n308_bg_command_boris` — backlog-pattern
  failure (`speaker='Саш'/'Борис'`) at lines 336, 348, 409 respectively.
- `n311_who_asked` — `❌ NO_ACCEPT после 3 попыток` at line 449.

### Run 34781633844 — `night_marathon_act4_voice_core_v1`

**Scenario header** (run_34781633844.log:46):

```
2026-09-13T20:43:20.3886965Z   TRIGGERED_BY_REASON: night marathon act 4: Робот учится говорить
```

**n401_list_voices — `list_tts_voices` tool never invoked** (run_34781633844.log:319–326):

```
2026-09-13T20:43:31.5944441Z >>> === STEP n401_list_voices (safe=n401_list_voices): voice=anton text="Робот, какими голосами ты умеешь говорить? Перечисли, что у тебя есть." ===
2026-09-13T20:43:31.5945728Z >>> STEP n401_list_voices: cmd_n401_list_voices.wav отсутствует — повторный синтез (cleanup-resilience)
2026-09-13T20:43:48.8689160Z >>> STEP n401_list_voices: робот молчит 15s — команду можно играть
2026-09-13T20:43:49.1231914Z >>> STEP n401_list_voices: PLAY attempt 1/3
2026-09-13T20:44:36.5384934Z >>> STEP n401_list_voices: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)
2026-09-13T20:44:37.0600468Z >>> TRANSCRIPT[n401_list_voices]: STT не вернул фразу (нет '✅ ПРИНЯТО')
2026-09-13T20:44:37.6167004Z >>> ACCEPTANCE[n401_list_voices]: ❌ expected tool calls not invoked: ['list_tts_voices']
2026-09-13T20:44:37.6167775Z >>> STEP n401_list_voices: ❌ проверка не прошла после retry (см. /tmp/e2e_v2_34781633844/acceptance.json)
2026-09-13T20:44:37.6168225Z E2E_STEP n401_list_voices FAIL
```

**n410_speed_up — `set_speed` tool never invoked** (run_34781633844.log:414–421):

```
2026-09-13T20:54:15.2422255Z >>> === STEP n410_speed_up (safe=n410_speed_up): voice=anton text="Робот, ладно, хватит тянуть, теперь наоборот — тараторь." ===
2026-09-13T20:54:15.2423836Z >>> STEP n410_speed_up: cmd_n410_speed_up.wav отсутствует — повторный синтез (cleanup-resilience)
2026-09-13T20:54:32.6812642Z >>> STEP n410_speed_up: PLAY attempt 1/3
2026-09-13T20:55:19.2004518Z >>> STEP n410_speed_up: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)
2026-09-13T20:55:19.7779248Z >>> TRANSCRIPT[n410_speed_up]: STT не вернул фразу (нет '✅ ПРИНЯТО')
2026-09-13T20:55:20.3302983Z >>> ACCEPTANCE[n410_speed_up]: ❌ expected tool calls not invoked: ['set_speed']
2026-09-13T20:55:20.3303839Z >>> STEP n410_speed_up: ❌ проверка не прошла после retry (см. /tmp/e2e_v2_34781633844/acceptance.json)
2026-09-13T20:55:20.3304282Z E2E_STEP n410_speed_up FAIL
```

**n412_skazka_multivoice — pattern `voice_used` missing** (run_34781633844.log:438–459):

```
2026-09-13T20:57:20.5424982Z >>> === STEP n412_skazka_multivoice (safe=n412_skazka_multivoice): voice=anton text="Робот, расскажи сказку про Красную Шапочку и говори за каждого персонажа своим голосом — за бабушку одним, за волка другим." ===
2026-09-13T20:57:43.7635607Z >>> STEP n412_skazka_multivoice: PLAY attempt 1/3
2026-09-13T20:58:34.4056690Z >>> STEP n412_skazka_multivoice: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)
2026-09-13T20:58:35.0874929Z >>> TRANSCRIPT[n412_skazka_multivoice]: STT не вернул фразу (нет '✅ ПРИНЯТО')
2026-09-13T20:58:35.1356138Z >>> STEP n412_skazka_multivoice: проверка паттернов (1): voice_used
2026-09-13T20:58:35.6994231Z   PATTERN_MISS: voice_used
2026-09-13T20:58:36.3007394Z >>> ACCEPTANCE[n412_skazka_multivoice]: ✅ all checks passed
2026-09-13T20:58:36.3009528Z >>> STEP n412_skazka_multivoice: ✅ acceptance PASS
2026-09-13T20:58:36.3010464Z >>> STEP n412_skazka_multivoice: ❌ проверка не прошла — retry 1/1
2026-09-13T20:58:46.5922244Z >>> === STEP n412_skazka_multivoice (safe=n412_skazka_multivoice): voice=anton text="Робот, расскажи сказку про Красную Шапочку и говори за каждого персонажа своим голосом — за бабушку одним, за волка другим." ===
2026-09-13T21:00:05.2562879Z >>> STEP n412_skazka_multivoice: PLAY attempt 1/3
2026-09-13T21:00:55.7402579Z >>> STEP n412_skazka_multivoice: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)
2026-09-13T21:00:56.4824944Z >>> TRANSCRIPT[n412_skazka_multivoice]: STT не вернул фразу (нет '✅ ПРИНЯТО')
2026-09-13T21:00:56.5154811Z >>> STEP n412_skazka_multivoice: проверка паттернов (1): voice_used
2026-09-13T21:00:57.1127500Z   PATTERN_MISS: voice_used
2026-09-13T21:00:57.7309466Z >>> ACCEPTANCE[n412_skazka_multivoice]: ✅ all checks passed
2026-09-13T21:00:57.7313617Z >>> STEP n412_skazka_multivoice: ✅ acceptance PASS
2026-09-13T21:00:57.7314833Z >>> STEP n412_skazka_multivoice: ❌ проверка не прошла после retry (см. /tmp/e2e_v2_34781633844/acceptance.json)
2026-09-13T21:00:57.7315343Z E2E_STEP n412_skazka_multivoice FAIL
```

**Aggregate GATE-1** (run_34781633844.log:458–466):

```
2026-09-13T21:00:57.7315343Z E2E_STEP n412_skazka_multivoice FAIL
2026-09-13T21:00:58.5370907Z [hint] GATE-1 soft-fail: voice-cycle OK but tool skipped — list_tts_voices
2026-09-13T21:00:58.5598823Z >>> GATE-1: ❌ expected tool calls not invoked during run: list_tts_voices; voice cycle completed (TTS finished x36, speak_text x909) but expected tool call(s) skipped: list_tts_voices. LLM сделал verbal-only answer (RULE #MUSIC для stop_music / RULE #VOICE-MULTI для multi-voice могут не enforce). Проверь master_prompt_compact.txt и/или добавь explicit tool-call enforcement в LLM-system reminder.
2026-09-13T21:00:58.5600699Z >>> GATE-1: ❌ aggregate acceptance FAIL (см. /tmp/e2e_v2_34781633844/acceptance.json)
2026-09-13T21:00:58.5601141Z E2E_GATE1_FAIL
2026-09-13T21:00:58.6276457Z >>> WARN: /tmp/e2e_v2_34781633844/recording.wav не найден — audio_metrics.json не пишется (recorder не запустился?)
2026-09-13T21:00:58.6277043Z E2E_VERDICT FAIL
2026-09-13T21:00:58.6277254Z E2E_FEATURE_FAIL
2026-09-13T21:00:58.9450986Z E2E_ARTIFACTS /tmp/e2e_v2_34781633844
2026-09-13T21:00:58.9456513Z >>> RECORDING: stop (SIGTERM pid=635264)
2026-09-13T21:01:00.1732802Z >>> RECORDING_DONE: /tmp/e2e_v2_34781633844/recording.wav (33535790 bytes)
```

Same GATE-1 block is repeated twice more (lines 619–623 and 792–796) verbatim.

## Cross-run observations for the QA reviewer

1. **All three runs have an identical `WARN: recording.wav не найден — audio_metrics.json не пишется`** immediately before `E2E_VERDICT FAIL`. The recorder log line shows the recording was eventually written (`/tmp/e2e_v2_<run_id>/recording.wav`, 33535790 bytes for run 34781633844), so the warning appears to be a race between the recording-cleanup step and the audio_metrics emission. Not the root cause, but worth flagging.

2. **All three runs end on the same harness hint text**:
   `LLM сделал verbal-only answer (RULE #MUSIC для stop_music / RULE #VOICE-MULTI для multi-voice могут не enforce). Проверь master_prompt_compact.txt и/или добавь explicit tool-call enforcement в LLM-system reminder.`
   This is a generic template the harness prints for every GATE-1 soft-fail, regardless of which tool was actually skipped — that is why the same line appears for `register_speaker` (run 1), `get_music_state` (run 2) and `list_tts_voices` (run 3). The hint does not mean `lead_synth` / `detect_key` are involved; it is the harness' stock remediation pointer.

3. **Music-tagged hits are sparse.** Only one literal occurrence of `compose_music` was found across the 3 logs (run 34780571796, n310_execute_backlog_lru) and that step **passed**. The other music-adjacent hits (`n312_stop_music`, `n313_silence_restored`) are described by tool-name (`get_music_state`) rather than by the `lead_synth` / `detect_key` tags the body asked for. `lead_synth` and `detect_key` do not appear anywhere in any of the 3 logs.

4. **Each `E2E_GATE1_FAIL` verdict is emitted 3 times per run** (once per reporting block), which is why the GATE-1 lines repeat at L440/574/724 in run 34779436302, at L473/641/825 in run 34780571796, and at L462/619/792 in run 34781633844. Verbatim quotes above cover the first occurrence in each run; later repeats carry identical text.

## How to re-verify (line numbers)

* Run 34779436302: per-step fails at L336 (n202 NO_ACCEPT), L361 (n204), L376 (n205), L397 (n207), L417 (n209), L437 (n211). Aggregate GATE-1 at L440–444.
* Run 34780571796: per-step fails at L336 (n302), L348 (n303), L409 (n308), L449 (n311 NO_ACCEPT), L470 (n313). `compose_music` pattern PASS at L432. Aggregate GATE-1 at L473–477.
* Run 34781633844: per-step fails at L326 (n401), L421 (n410), L448/L459 (n412 retry). Aggregate GATE-1 at L462–466.

All line numbers above are 1-indexed positions in the raw `.log` files committed alongside this report.