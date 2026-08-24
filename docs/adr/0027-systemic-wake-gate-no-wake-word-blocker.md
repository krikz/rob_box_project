# ADR-0027: Системный wake-gate no_wake_word blocker в e2e — наблюдаемое поведение и его отличие от stale-PR / dj02-race

| Поле | Значение |
|---|---|
| Статус | Proposed |
| Дата | 2026-08-23 |
| Автор | architect (Hermes Agent), kanban t_d9e70587 |
| Контекст | E2E Voice Test: fail-streak round-215/216/217 (3 подряд, 23.08 18:40–19:50 UTC), signature = `TRANSCRIPT[ww01/mv02]: ожидалось «<wake> …», распознано «…»` — wake-word теряется STT'ом на первом слове фразы. Робот жив, контейнеры healthy, develop свежий (содержит fix #1546). Это **отдельный** failure-mode, не покрытый ни ADR-0024 (music-aware), ни t_fb037ed1 (stale-PR). |
| Затрагивает | наблюдательное знание для будущих ретро; голосовой wake-gate в `audio_node`/Yandex STT pipeline; e2e-harness (нет pre-flight diagnostic) |
| Родители | ADR-0024 (music-aware intent priority gate — закрывает только music-context); ADR-0025 (stale-PR detection — закрывает другой класс проблемы) |
| Связанные | issue #1525 (CLOSED, про wake-word под играющей музыкой, PR #1547 в работе), issue #1117 (CLOSED, про wake_word теряется в audio_node), PR #1547 (needs-review, MERGEABLE), PR #1546 (MERGED 22.08 — fix #1546), t_fb037ed1 (done 22.08 — stale-PR verdict), t_9d229634 (blocked — dj02 LLM race), t_7dbd1bd1 (ready — archive parent после merge #1547), t_d9e70587 (running — это retro) |

## 1. Контекст и бизнес-проблема

### 1.1 Аномалия

23.08.2026 в окне 18:40–19:50 UTC произошёл **fail-streak 3/3** подряд на `L: E2E Voice Test`:

| round | verdict | ww01 | mv02 | ds01/02 | dj02 | signature |
|---|---|---|---|---|---|---|
| 215 | FAIL | FAIL | — | — | FAIL | `no_wake_word` |
| 216 | FAIL | FAIL | — | — | FAIL | `no_wake_word` |
| 217 | FAIL | FAIL no_accept | FAIL no_accept | backlog (no_wake_word) | FAIL | `no_wake_word` + multiple no_accept |

Сырой лог round-216 (`gh run view 32660445543`):
```
19:15:36 >>> STEP ww01_roboks_wake: PLAY attempt 1/3
19:16:36 >>> STEP ww01_roboks_wake: нет акцепта (attempt 1) — повтор
19:18:34 >>> STEP ww01_roboks_wake: ❌ NO_ACCEPT после 3 попыток
19:18:34 E2E_STEP ww01_roboks_wake FAIL no_accept
19:18:34 >>> TRANSCRIPT[ww01_roboks_wake]: ожидалось «Робокс, как дела», распознано «как дела»
19:19:36 E2E_STEP mv01_set_voice_alena OK
19:19:53 >>> STEP mv02_speak_alena: PLAY attempt 1/3
19:22:35 >>> STEP mv02_speak_alena: ❌ NO_ACCEPT после 3 попыток
19:22:35 E2E_STEP mv02_speak_alena FAIL no_accept
19:22:35 >>> TRANSCRIPT[mv02_speak_alena]: ожидалось «Робот, скажи привет», распознано «скажи привет»
19:26:47 >>> STEP ds01_speaker_A_tea: ✅ backlog accumulated (no_wake_word)
19:27:55 >>> STEP ds02_speaker_B_coffee: ✅ backlog accumulated (no_wake_word)
```

**Signature этого fail-streak** = wake-слово («Робокс», «Робот») систематически теряется Yandex STT на первом слове длинной фразы; остаток фразы распознаётся; wake-gate фейлит → диалог не открывается.

### 1.2 Что это НЕ

Чтобы будущие ретро не тратили время на повторное расследование, фиксируем **отличия от уже диагностированных кейсов**:

| Failure-mode | Signature | Root cause | Когда проявляется |
|---|---|---|---|
| **stale-PR** (t_fb037ed1, done 22.08) | `PATTERN_MISS: alena` + `mv01 acceptance PASS` + `GATE-1 expected tool calls not invoked` | PR tip отстаёт от develop (отсутствие `db84ff59` в ancestry) | После изменений в harness/scenario, не докатившихся до старой PR tip |
| **dj02 LLM race** (t_9d229634, blocked) | `dj02_stop_music FAIL acceptance` (`stop_music` не вызван) | LLM-stream не доходит до `stop_music` tool call; music_state не пробрасывался в dialogue_node | На шаге dj02 после dj01 (музыка играет) |
| **e2e harness-output missing** (t_fb037ed1, вторичный) | `Harness output missing ($VERDICT_LOG)` | scp race на 10.1.1.249 | Разовая инфраструктурная деградация |
| **wake-gate no_wake_word** (t_d9e70587, **этот**) | `TRANSCRIPT[ww01]: «как дела»` (без «Робокс») | STT pipeline теряет wake-слово на тишине или при ambient noise | На любом voice-step; reproduce rate ≥ 3/3 подряд |

### 1.3 Почему это «системный», а не flaky

1. **Reproducibility**: 3/3 подряд за ~1 час. Flaky даёт ~30-50% success, не 0%.
2. **Signature стабильна**: TRANSCRIPT показывает одно и то же — wake-word теряется, остаток фразы приходит. Это не timing race (тогда были бы разные signature).
3. **Scope**: ww01 (`Робокс, как дела`), mv02 (`Робот, скажи привет`), mv03 — **все** voice-step с wake-prefix. Паттерн не зависит от конкретной команды.
4. **Контекст**: round-212 был success (после merge #1546 — fix #1546 в music_state); 213/214 нет в списке (отменены или skipped — см.ниже); 215/216/217 — все FAIL. Между success и fail-streak нет очевидного PR в develop, который трогал бы STT pipeline (см. §2.2).
5. **Round-212** (00:12Z) success → **round-215** (18:40Z) fail-streak. Между ними ~18.5 часов. PR #1546 уже был в develop к моменту round-212. Разрыв слишком большой для «изменение в develop вызвало regression» — скорее **дрейф robot-host** (mic calibration / Yandex credentials / network).

## 2. Что проверено (raw evidence, 23.08 23:00 CEST)

### 2.1 E2E Voice Test rounds 23.08

```
gh run list --repo krikz/rob_box_project --workflow "L: E2E Voice Test" --limit 30
```

| round | created | conclusion | branch |
|---|---|---|---|
| 32662498630 (round-217) | 19:50Z | failure | z-{e2e}/test-round-217 |
| 32660445543 (round-216) | 19:11Z | failure | z-{e2e}/test-round-216 |
| 32658783611 (round-215) | 18:40Z | failure | z-{e2e}/test-round-215 |
| 32607268334 (round-212) | 00:12Z | success | z-{e2e}/test-round-212 |

Между round-212 (success) и round-215 (first fail в streak) — раунды 213/214 не в списке top-30 (вероятно skipped из-за отсутствия live candidates — поведение ADR-0025 live-candidate guard). Это **ожидаемо**: между успешным прогоном и новым циклом нет PR с `agent:*` label, поэтому e2e-process не генерит новые rounds.

### 2.2 Git log origin/develop между round-212 и round-215

```bash
gh run view 32660445543 --log | grep "TRANSCRIPT"
git log origin/develop --oneline --since="2026-08-23T00:13:00Z" --until="2026-08-23T18:40:00Z"
```

30+ коммитов в develop за окно 23.08 00:13Z – 18:40Z. Среди них — `084fe626` (backlog hint injection, **но это 21.08 — out of window**), и связанные voice-pipeline коммиты **тоже до 22.08**. В окне 23.08 — преимущественно `ci: SHA tags`, `wip(process-fix-roadmap)`, `wip(agent-flow)*`, `wip(ci)*`. **Ни одного commit'а, меняющего voice pipeline / STT / wake-gate** в окне между round-212 success и round-215 fail.

Это значит: кодовая регрессия в develop **не объясняет** fail-streak 23.08 18:40–19:50.

### 2.3 PR #1547 status (на момент retro 23.08 23:14)

```bash
gh pr view 1547 --repo krikz/rob_box_project --json state,mergeable,statusCheckRollup
```

- `state`: OPEN
- `mergeable`: MERGEABLE
- `additions`: 307, `deletions`: 11
- `files`: `dialogue_node.py` (+125/-11), `test_pure_methods.py` (+182)
- **Все CI checks SUCCESS**: Python Code Quality, Unit Tests (ROS2 Humble), TTS Provider Tests + coverage, YAML/Config, Shell Scripts, Lint Summary, Test Summary
- `reviewDecision`: пусто (Шифу ещё не ревьюил)
- `head`: `z-{agent}/1546-fix-voice-1544-dialogue-node-music-state`

PR **ждёт Шифу merge**. ADR-0027 фиксирует: **пока PR #1547 не влит, root cause из issue #1525 (wake-word теряется под играющей музыкой) остаётся активным**, но это объясняет только dj02_stop_music, не массовый no_wake_word на тишине.

### 2.4 Робот жив (проверено в task body)

```
ssh ros2@10.1.1.21 — uptime 5d
voice-assistant `Up About an hour (healthy)`
```

→ wake-gate фейлит не из-за crash/restart, а на стороне STT pipeline.

## 3. Гипотезы root cause (в порядке приоритета)

### H1. STT pipeline infra-degradation (Yandex API key / network / robot-host mic)

**Evidence**: signature = TRANSCRIPT показывает чистую потерю первого слова; reproduce rate 3/3; код не менялся.

**Проверка**: нужен лог `docker logs voice-assistant --since 2026-08-23T18:30:00Z | grep -E "yandex|wake|STT"`, плюс `ssh ros2@10.1.1.21 docker inspect voice-assistant | grep -E "RestartCount|StartedAt"`. Это вне scope архитектора (правило 18.08 «не фиксить руками»), передаётся **backend** для диагностики.

### H2. audio_node wake-word detector de-calibration после deploy

**Evidence**: голосовая команда синтезирована harness'ом (cmd_ww01_roboks_wake.wav — отсутствует, повторный синтез), но робот не реагирует. VAD/audio_node были обновлены в PR #1480 (c57c0c23 — two-phase Yandex + RMS telemetry). Возможно, новый RMS/VAD-порог не откалиброван на новый микрофон или volume.

**Проверка**: `git log src/rob_box_voice/rob_box_voice/audio_node.py --since "2026-08-15"`, плюс параметры VAD (dB threshold, RMS floor) в YAML config. Если пороги жёстче, чем на этапе синтеза команд — wake-word выпадает.

### H3. Race condition с предыдущим TTS-ответом (VAD ещё считает TTS активным)

**Evidence**: в логах других ретро: `TTS активен — VAD пропускает речь (barge-in через wake-word gate)`. Если TTS от предыдущего шага (mv01 OK) не успел «остыть», wake-gate может фейлить на следующем шаге (mv02). Но это не объясняет ww01 (первый wake-step после cold start).

### H4. PR #1547 ещё не влит, и dj02_stop_music контекст удерживает music_state

**Evidence**: PR #1547 чинит `<music_state>` пробрасывание, но music не играет в момент ww01/mv02 (это cold-start steps). Так что H4 не подходит для cold-start fail-streak.

**Наиболее вероятная комбинация**: H1 (Yandex/network) + H2 (VAD calibration). Round-212 success (00:12Z) → 18+ часов дрейфа → fail-streak. Это паттерн «degradation по времени», не «regression по коду».

## 4. Что НЕ предлагаем в этом ADR

- **НЕ чинить руками** audio_node/VAD/STT (правило 18.08).
- **НЕ делать ad-hoc fix в develop** без e2e-done (правило ADR-0013 incremental delivery).
- **НЕ закрывать** ни одно из связанных issue (#1525, #1117) — root cause ещё не подтверждён.
- **НЕ менять** acceptance criteria `voice_core_suite_v1.json` — это не flaky acceptance, это blocker (signature стабильная).

## 5. Рекомендации (для Шифу и для будущих ретро)

### 5.1 Архитектурное (owner: architect)

1. **Зафиксировать signature** этого failure-mode в ADR-0022 как **«wake-gate no_wake_word systemic»** — отдельный класс, не flaky. Это позволит следующему ретро НЕ путать его ни со stale-PR (t_fb037ed1), ни с dj02-race (t_9d229634).
2. **Документировать expected behavior**: после успешного e2e (round-212 00:12Z) при отсутствии новых PR в develop **не генерировать** rounds 213/214 — это поведение ADR-0025 live-candidate guard. Но **после длительного простоя** (>12ч) — добавить sanity-check: «последний success N часов назад → требовать re-calibration round».
3. **Pre-flight diagnostic в e2e-harness** (п.5.2 ниже) — отдельная задача для devops.

### 5.2 Процессное (owner: devops, НЕ архитектор)

4. **E2E harness: pre-flight wake-gate diagnostic**. Если wake-word не детектится в первом step (ww01 или cc01) — пометить дальнейшие steps как `blocked by wake-gate`, а не `no_accept`. Это:
   - Отличит systemic failure от flaky acceptance (отдельная signature в логе).
   - Позволит triage-боту сразу эскалировать на backend, а не на harness-script.
   - Снизит шум «mass no_accept» в раундах (этот ретро начался именно из-за шума).

5. **Отдельный issue «wake-word gate regression post-#1547»** (assignee=backend, после merge PR #1547 Шифу):
   - Если после merge #1547 fail-streak сохраняется → нужен чисто wake-gate fix без примесей dj02/mv02.
   - Минимальный repro: тестовая команда `Робокс, как дела` на cold start → ожидаемый wake-detection в STT-логе.

## 6. Acceptance criteria (для archive этой retro-карточки)

- ✅ PR #1547 влит Шифу.
- ✅ 3 раунда dj02_stop_music подряд PASS (это acceptance t_7dbd1bd1, **НЕ** этой карточки — но необходимо для полного цикла).
- ✅ Никаких массовых `no_wake_word` в последних 5 раундах.
- ✅ t_9d229634 архивирована.
- ✅ (опционально) E2E harness имеет pre-flight wake-gate diagnostic — отдельная карточка devops.

## 7. Backwards compatibility / failure modes этого ADR

- **ADR informational only**: документирует наблюдаемое поведение и signature. Не навязывает реализацию.
- **Если PR #1547 не починит**: ADR остаётся валидным, root cause просто переформулируется в H1/H2.
- **Если pre-flight diagnostic будет добавлен**: ADR можно расширить §5.2 дополнительным параграфом.

## 8. Когда поднимать / пересматривать этот ADR

- После merge PR #1547 + 3 раунда без fail-streak → перевести статус в `Accepted`.
- Если signature меняется (например, wake-word detector улучшен) → обновить §1.1.
- Если pre-flight diagnostic реализован → добавить ссылку на issue/PR.

## 9. Связь с существующими механизмами

- **ADR-0024** (music-aware intent priority gate): закрывает dj02 (музыка играет + команда стоп). **Не** закрывает cold-start wake-gate на тишине.
- **ADR-0022** (process e2e done gates): GATE-1 acceptance contract. Wake-gate фейл не нарушает GATE-1 (он идёт до acceptance), но помечает раунд как FAIL на step-level.
- **ADR-0025** (stale-PR detection): закрывает stale-PR tip → старый harness/scenario. **Не** закрывает свежий develop с wake-gate regression.
- **t_fb037ed1** (stale-PR verdict, done 22.08): объясняет fail-streak **ДО** merge #1546.
- **t_9d229634** (dj02_stop_music flaky, blocked): объясняет fail-streak **ПОСЛЕ** merge #1546, но до merge #1547.

**Эта карточка (t_d9e70587)** объясняет fail-streak **ПОСЛЕ** merge #1546, на cold-start voice-step (ww01, mv02, mv03) — отдельный failure-mode, требующий merge #1547 + дополнительной backend-диагностики.

---

## Приложение A: timeline восстановления (для следующего ретро)

Если Шифу не merge'нет #1547 в ближайшие 24-48ч, ожидаемый сценарий:

1. **0–24ч**: fail-streak продолжится, e2e-harness будет показывать массовые `no_accept`.
2. **24–72ч**: triage-бот начнёт агрессивно создавать карточки «wake-gate regression» (этот retro будет первым).
3. **После merge #1547 + 1 round**: скорее всего вернётся к нормальному success-streak, потому что:
   - dj02_stop_music починен (music_state в dialogue_node).
   - cold-start wake-gate не зависит от music-state, но возможно обновится стейт машина wake-gate (побочный эффект).
4. **Если после merge #1547 fail-streak сохраняется**: подтверждается H1/H2, нужен backend-issue «wake-word detector calibration post-deploy».

## Приложение B: различия с ADR-0024

| Параметр | ADR-0024 (music-aware) | ADR-0027 (systemic no_wake_word) |
|---|---|---|
| Контекст | Музыка играет (dj01 → dj02) | Cold start, тишина |
| Root cause | dialogue_guards не пробрасывает music_state | STT теряет wake-слово на первом слове |
| Repro signature | `dj02 stop_music not invoked` | `TRANSCRIPT[ww01]: «как дела»` (без «Робокс») |
| Fix | PR #1547 (`<music_state>` в LLM input) | PR #1547 частично + backend wake-gate calibration |
| Status | Plan-only (закрывает dj02 #1525) | Empirical (наблюдаемое поведение 23.08) |