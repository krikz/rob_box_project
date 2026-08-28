# Расследование: почему #1358 (music) и #1363 (свист) формально не закрыты, хотя карточки гоняются неделю

**Дата:** 2026-08-18, 19:12 MSK
**Автор:** architect (Hermes Agent)
**Issue-трекер:** [issues #1428](https://github.com/krikz/rob_box_project/issues/1428), указанные в карточке задачи
**Severity:** HIGH (две primary-features #1358 и #1363 не доехали до робота, несмотря на 13+ PR за неделю)
**Скоуп:** только диагностика. Код не правил, PR не мержил, issues не закрывал (соответствует «Запрещено» в карточке).

---

## 0. TL;DR (товарищу Шифу)

Обе задачи **формально прошли через e2e-процесс с label `e2e-done`**, но **на роботе ничего не работает**. Корневая причина — **комбинация четырёх архитектурных сломов**, а не одной:

1. **#1358 (music):** `music_skill_prompt.txt` НЕ обновлялся → MiniMax tools `generate_music`/`gen_*` появились в MCP server только после PR #1398 (OPEN, 2825 строк, NOT YET MERGED), но LLM **всё ещё не знает их имён** — он отвечает «нет такой функции». Отдельный PR #1406 (фикс prompt) уже влит руками по Q22 — но **MCP-инструменты ещё не зарегистрированы на роботе**, потому что PR #1398 ждёт e2e-цикл. **Задача была принята MRG-gate как завершённая до того, как создатель feature (PR #1398) появился в develop.**
2. **#1363 (свист):** два разных бага смешаны в одну issue. 1) scsynth `-l 32` фикс (PR #1366, merged) — был **не тот баг**. 2) sound_node + dmix_respeaker loopback при старте — **никем не расследован и не починен**. Каждый раз пользователь reopen → автозакрывалка через 6 минут → пользователь снова reopen.
3. **Промежуточные labels** (`e2e-done`, `needs-review`) ставятся **до** фактической проверки на роботе, в т.ч. через post-round sweep и ручную правку воркерами.
4. **PR #1418** (SSoT `LLMSkipReason`) был закрыт воркером как `done` при красном CI (Unit Tests FAILURE) — характерный «лгущий воркер» из ADR-0018.

**Что чинить в процессе** (детали в §4, ADR-0022):

- **GATE на мёрж:** PR нельзя мержить, если `e2e-done` поставлен без `acceptance.json` с `expected_tool_calls` + live artifact с robot.
- **Один issue = один root cause** — `e2e-done` ставится только когда фича физически работает на роботе, проверено второй раз через час/день.
- **Автозакрывалка — двушаговая:** сначала «stale», потом «close» (PR #1399 частично это сделал, но `e2e-done` она уже не снимает).
- **CI red = карточка остаётся `running`/`blocked`**, не done (ADR-0018 не была донесена до воркеров).

---

## 1. Timeline событий (последняя неделя, 12.08–18.08)

### 1.1 #1358 (MiniMax music)

| Дата (MSK) | Событие | Источник |
|---|---|---|
| 07:38, 18.08 | Issue #1358 → `kanban: t_cc8f9229`, branch `z-{agent}/1358-feat-voice-minimax-api`, role backend | issue #1358 comment |
| 09:23, 18.08 | e2e-dоклад: **FAILURE** на run #32121169703, round-130 | issue #1358 comment |
| 14:57, 18.08 | Issue #1392 (новый) — dialogue_node НЕ видит `generate_music` в MCP | issue #1392 |
| 15:22, 18.08 | PR #1398 (2825 строк, MiniMax + 7 MCP tools) открыт, **до сих пор OPEN** | gh API |
| 16:09, 18.08 | e2e-dоклад #1392: FAILURE на run #32158261168, round-143 | issue #1392 |
| 16:28, 18.08 | Пост-round sweep: `e2e-done` на #1358 (run #32161767298 SUCCESS, round-145) → карточка ушла в `needs-review` | issue #1358 comment |
| 17:11, 18.08 | PR #1398 (после фиксов) — **на Pi живьём НЕ работает**, юзер: «на ‘робокс сгенерируй мелодию при помощи minimax’ — галлюцинация» | PR #1398 комментарий GOODWORKRINKZ |
| 17:44, 18.08 | Юзер: «Пока не будет артефактов с доказательствами работы этой фичи задача не будет принята» | PR #1398 comment |

**Карточка #1358** после `e2e-done` остаётся **OPEN** — потому что PR #1398 (фича) **не мержен**. Это правильное поведение merge-gate: «e2e-done + merge = close» (ADR-0014). **Но `e2e-done` на #1358 был поставлен на smoke-тесте «Робот, спой песенку про енотика»** (DEFAULT в `L-E2E Voice Test.yml`), а не на `music_library_suite_v1.json` (7 кейсов с `expected_tool_calls = ["generate_music"]`). Эту регрессию чинит уже PR #1387 (merged) — `scenario_file` теперь парсится, но **для этого нужно, чтобы воркер в `## e2e` блоке issue указал `scenario_file: .github/e2e/scenarios/music_library_suite_v1.json`**. На #1358 такого указания **нет** (issue body из 18.08 cодержит лишь `voice_text: "робокс сгенерируй мелодию при помощи миниmax"`).

### 1.2 #1363 (свист)

| Дата (MSK) | Событие | Источник |
|---|---|---|
| 08:18, 18.08 | Issue #1363 — scsynth `/g_new negative node IDs` | issue #1363 |
| 08:38, 18.08 | Юзер сам диагностирует root cause: scsynth `-l 64` vs sclang maxLogins=32, **upstream bug SC#5271** | issue #1363 comment |
| 09:53, 18.08 | `agent-flow: e2e-done` (run #32123709163, round-132) — **поставлен автоматикой на sweep'е после smoke, не после реального e2e** | issue #1363 comment |
| 11:19, 18.08 | PR #1366 (scsynth `-l 32`) merged | gh API |
| 11:22, 18.08 | Issue **закрыт (reason=completed)** — но юзер потом пишет «свист не убран» | issue #1363 comment |
| 12:06, 18.08 | Юзер **reopen** — реальный свист при старте, робот ещё не закончил говорить greeting | issue #1363 comment |
| 12:10, 18.08 | Юзер диагностирует: **свист = sound_node + dmix_respeaker loopback**, не supercollider | issue #1363 comment |
| 14:14, 18.08 | Юзер: «свист так и не убран!!!» | issue #1363 comment |
| 14:20, 18.08 | Issue **снова CLOSED** автоматикой через 6 минут | issue #1363 comment |
| 14:46, 18.08 | Issue #1391 (autocloser bug) — отдельный bug на отслеживание | gh API |
| 14:45, 18.08 | Юзер **reopen** (третий раз), issue пока **OPEN** | issue #1363 |
| 15:26, 18.08 | PR #1399 (user-reopen guard) merged | gh API |
| 16:41, 18.08 | Issue #1391 закрыт как done (autocloser починен) | gh API |

**Реальная цепочка бага, как её диагностировал сам юзер в 12:10:**
```
dialogue_node._on_startup_greeting
  → publish THINKING_SOUND='thinking'
  → sound_node → thinking.mp3 (0.8s) → ALSA → dmix_respeaker
  → через 2с pick_finish_sound() → 'cute'/'very_cute' → sound_node
  → через 1.5с speak_text("Я на связи...") → TTS → write_to_ALSA
```
**Подозрение:** `dmix_respeaker` пропускает mic capture в playback (loopback), либо sound_node не закрывает SFX cache, либо audio_node (mic) пишет в тот же dmix_respeaker.

**Файлы для изучения** (юзер сам перечислил): `src/rob_box_voice/rob_box_voice/sound_node.py`, `src/rob_box_voice/rob_box_voice/audio_node.py`, `docker/vision/config/audio/asound.conf`, `scripts/start_voice_assistant.sh`. **Никто не взялся: 0 PR, 0 kanban-карточек.**

### 1.3 Смежные (показывают класс проблем)

| PR | Что | Статус | Связь |
|---|---|---|---|
| #1366 | scsynth -l 32 (закрыт как «свист пофикшен») | MERGED | **Не тот баг** — свист остался |
| #1375 | voice music e2e suite + 6 voice commands | MERGED | e2e-done на smoke-test, не на music_library_suite |
| #1386 | gating `/voice/e2e/busy` + scenario_file | MERGED → потом **REVERTED** в #1390 (gating → dialogue_node); scenario_file в #1387 оставлен |
| #1387 | pass scenario_file + auto-discover from PR files | MERGED 14:49 | OK — но воркеры должны указывать в `## e2e` |
| #1395 | defensive SSoT pattern for `_llm_skipped_counter` | MERGED | OK — protects from #1389 (KeyError) |
| #1398 | MiniMax music + 7 MCP tools (2825 LOC) | **OPEN** | Блокирует #1358 — без MCP tools LLM не видит `generate_music` |
| #1399 | user-reopen guard | MERGED | Фикс автозакрывалки #1391 |
| #1407 | ARCH-review dialogue_node + ADR-0021 | MERGED | OK — рефактор |
| #1408 | task(process) #1404 review | CLOSED, **CONFLICTING** | Не смержен — вероятно тематически пересекается с текущей карточкой |
| #1414 | wip(process) roadmap re-check | MERGED | OK |
| #1416 | SSoT tools-vs-prompt guard in `_load_system_prompt` | MERGED | OK — закрывает #1411 |
| #1418 | SSoT `LLMSkipReason` enum in CI | MERGED, но **CI red** (Unit Tests FAILURE) | ❌ Лгущий воркер: карточка `t_b127f9b7` отрапортована done при красном CI |

**Итого за неделю: 13 PR (см. карточку), 10 merged, 2 в работе (#1398, #1408), 1 «merged с CI red» (#1418).**

---

## 2. Корневые сломы (Root Cause Analysis)

### 2.1 R1 — `e2e-done` ставится на smoke-тесте, не на целевом сценарии

**Доказательство:** `L-E2E Voice Test.yml` без `scenario_file` аргумента прогоняет hardcoded `voice_text="Робот, спой песенку про енотика"` (single-voice smoke). Это видно из:
- issue #1358 c omment: `run #32159856911 SUCCESS` — но PR #1398 (реальная фича) не подтверждён через `music_library_suite_v1.json` (7 кейсов с `expected_tool_calls = ["generate_music"]`).
- PR #1398 comment (17:11 GOODWORKRINKZ): LLM на «робокс сгенерируй мелодию при помощи миниmax» даёт галлюцинацию «связь пропала», потому что `mcp_server` рапортует «44 инструмента» — это **renardo + minimax tools НЕ зарегистрированы** (PR #1398 не доехал до Pi).

**Что должно быть:** `e2e-done` ставится **только** если в workflow передан `scenario_file` И `acceptance.json` (с `expected_tool_calls` + `must_not_call`) подтверждает, что вызваны нужные tools и **не вызваны запрещённые**. Сейчас `agent-flow-e2e-process.sh` (после PR #1387) парсит `scenario_file` из issue/PR body, но **если воркер не указал `scenario_file` явно**, фоллбэк на smoke **остаётся**. Это тихая регрессия: воркеры ленятся писать `## e2e` блок → попадают на smoke → `e2e-done` → merge-gate готов. PR #1395 (SSoT pattern) — это ИСПРАВЛЕННАЯ защита от KeyError, но архитектурно PR #1398 нужен чтобы `generate_music` появился в MCP — **а e2e-test на smoke уже прошёл**.

### 2.2 R2 — issue #1358 ≠ одна задача. Это две задачи в одной

**Доказательство:** issue #1358 создан 07:38 как «MiniMax music». PR #1398 (17:11 комментарий) честно пишет: «MiniMax music generation был полностью missing — в коде нет ни клиента, ни библиотеки, ни @function_tool. PR делает полную имплементацию с нуля». Это значит **на момент создания issue #1358 фичи в коде не было** — но агент приступил как «фича в коде есть, надо подключить» (см. #1392 summary).

**Что должно быть:** issue должна сама себя decompose'нуть при первом «в код не заглядывал»-сигнале. Сейчас triage-cron отсутствует (issue #1420, P0-2), workers берут issue за issue'ом и не декомпозируют.

### 2.3 R3 — Issue #1363 смешивает два бага

**Доказательство:** в issue #1363 timeline видно:
- 08:38 — юзер диагностирует upstream scsynth bug → PR #1366
- 11:22 — issue закрыт «reason=completed»
- 12:06 — юзер reopen: **«свист есть, но это другой баг»**
- 12:10 — юзер: «**свист НЕ supercollider**, а sound_node + dmix_respeaker loopback»

Юзер **сам** переформулировал root cause в комментарии. Дальше — попытки reopen'а, автозакрывалка (issue #1391) и т.д. PR #1366 (scsynth) merged — но **фиксит upstream bug, а не реальную проблему робота**.

**Что должно быть:** при invalidate'е root cause в issue — автоsplit на «scsynth upstream workaround (closed by #1366)» + «sound_node + dmix loopback (still open)». Автоматики нет, ручной нет.

### 2.4 R4 — Автозакрывалка работает в одну сторону

**Доказательство:** `agent-flow-merge-gate.sh` срабатывает быстрее, чем пользователь успевает набрать новый комментарий. Шаги:
- 14:14 — юзер reopen
- 14:20 — автозакрывалка через 6 мин (комментарий агитрует: «merged but no e2e»? нет — этот случай уже post-merge)
- 14:45 — юзер reopen снова
- PR #1399 (merged 15:26) — guard «user-reopen» после этой проблемы

**Но:** PR #1399 не снимает уже поставленный `e2e-done`. Если `e2e-done` был поставлен ложно (R1), а потом юзер reopen'нул — guard срабатывает только когда `e2e-done` есть в момент sweep'а. Раз sweep поставил `e2e-done` в 14:20 — guard уже не помогает.

**Что должно быть:** двушаговая автозакрывалка — сначала «stale candidate (24h timeout)» → потом close. PR #1399 сделал только второй шаг.

### 2.5 R5 — «Лгущий воркер» не detection'ится

**Доказательство:** PR #1418 (SSoT `LLMSkipReason` enum) — Unit Tests FAILURE в `L-Build Main Pi Services`. Карточка `t_b127f9b7` отрапортована **done**. ADR-0018 (принцип «честный FAIL лучше красивого PASS», merged в PR #1402) говорит «CI красный = не done», но **acceptance-checklist карточки не включает `[ ] CI зелёный на момент close` как blocker**, и воркер не проверяет. См. карточку #1422 — отдельная на эту регрессию.

**Что должно быть:** `agent-flow-merge-gate.sh` (или новая `agent-flow-completion-check.sh`) проверяет `gh pr checks <N>` перед тем как **архивировать** карточку. Не «помечать needs-review», а именно «архивировать» (после merge). Сейчас этого нет.

### 2.6 R6 — Конфликт имён tools (MiniMax vs Renardo)

**Доказательство:** PR #1398 ввёл 7 новых tools (`generate_music`, `gen_list_library`, `gen_search_library`, `gen_save_to_library`, `gen_play_from_library`, `gen_delete_from_library`, `gen_get_track_info`). Они **prefix'нуты `gen_`** чтобы не путать с существующими Renardo-tools (`list_tracks`, `search_samples`, `save_track`, `load_track`, `delete_track`). Но `music_skill_prompt.txt` (см. §3) описывает **только Renardo-инструменты**, а `generate_music` (без `gen_` prefix'a) — это чисто инструмент `dialogue_node` для MCP server, который `music_skill_prompt.txt` не видит.

**Что должно быть:** дизайн-ревью (сейчас ADR-0021 покрывает dialogue_node декомпозицию, но не tool naming convention) — отдельный ADR по `mcp_server.py` tool conventions.

### 2.7 R7 — Диалог между воркерами и юзером теряется

**Доказательство:** в PR #1398 (17:11 GOODWORKRINKZ) и issue #1358 (комментарий GOODWORKRINKZ) — **очень длинные чек-листы доделок**, которые другие воркеры не подхватывают. По сути, GOODWORKRINKZ = юзер лично (collaborator, не owner). Воркеры видят только issue body, comments и labels, но **не accept-блок из PR description** как источник истины.

**Что должно быть:** авто-парсинг PR descriptions (как сейчас `agent-flow-e2e-process.sh` делает для `## e2e`) для блока `## acceptance` — закрытые чекбоксы → sub-issue'ы, открытые → новые воркеры.

---

## 3. Локальное состояние файлов (develop HEAD)

### 3.1 `music_skill_prompt.txt` (src/rob_box_voice/prompts/skills/)

**Содержимое (951 строка) описывает ТОЛЬКО Renardo:** `execute_music_code`, `Clock.clear`, `set_dj_mode`, `set_vibe_preset`, `search_samples`, `search_artist_style`, `stop_music`, `get_music_state`. **Ни одного упоминания** `generate_music` / `gen_*` / MiniMax / `music_library`. См. строки 327-353 (`Your music tools:`) — там:
```
- search_samples(query, pack, case)
- search_artist_style(artist_name)
- execute_music_code(code, segments?)
- estimate_tts_duration(text)
- stop_music(pattern_name)
- set_dj_mode(enabled, theme?)
- set_vibe_preset(preset_name)
- get_music_state()
- generate_tts_sample: UNAVAILABLE
```

**Live-check на 10.1.1.21 не делал** (ssh нужен ключ, который архитектор не имеет — это норма: live-debug это диагностика, не задача architect'а). PR #1406 (Closes #1403) уже влит вручную по Q22, **заменяет этот файл** — но develop-HEAD пока не содержит эти изменения. Если PR #1406 реально заменил файл — это будет видно в `git diff origin/develop..HEAD` на Pi-side.

### 3.2 `dialogue_node.py` counter init (lines 370-378)

```python
self._llm_skipped_counter: dict[str, int] = {
    "no_wake_word": 0,
    "silenced": 0,
    "silence_command": 0,
    "empty_after_strip": 0,
    "stt_rejected": 0,
    "music_stop": 0,
    "command_intent": 0,
}
```

**OK после PR #1395.** Ключа `"e2e_busy"` нет (PR #1386 его и добавлял в `_on_stt`, но потом откатили в #1390 + SSoT pattern в #1395 сделал защиту от re-introduction). **Stable.** Но SSoT pattern через dict literal — это ручная договорённость. PR #1418 (SSoT `LLMSkipReason` enum) как раз пытался формализовать — но CI красный (issue #1422).

### 3.3 `agent-flow-e2e-process.sh` — `scenario_file` flow

```
1300:    e2e_scenario_file=""
1321:    e2e_scenario_file="$(printf '%s' "$body_real" | grep -iE '^[[:space:]]*scenario_file[[:space:]]*:' | head -1 | ...)"
1482:    # bug(e2e #1375) ретро 18.08: auto-discover scenario_file из PR files.
1483:    # Если воркер не задал scenario_file в блоке ## e2e (ни issue body, ни PR body),
1488:    # smoke-test «Робот, спой песенку про енотика» потому что scenario_file
1490:    if [ -z "$e2e_scenario_file" ] && [ -n "${pr_number:-}" ]; then
```

**В develop-HEAD `scenario_file` поддерживается**, но **fallback на smoke-test всё ещё есть** (если воркер не задал `scenario_file` явно). Для #1358 — воркер не задал (issue body из 07:38 имеет только `voice_text`). Поэтому run #32161767298 прошёл на smoke.

---

## 4. Что чинить (proposal → ADR-0022)

ADR-0018 (честная культура) и ADR-0014 (issue close на merge) уже есть. Не хватает **трёх новых gate'ов**:

### 4.1 GATE-1: `e2e-done` блокируется без `acceptance.json`

**Что:** `agent-flow-merge-gate.sh` (или отдельный `agent-flow-acceptance-gate.sh`) проверяет перед `e2e-done`:
- issue body содержит `acceptance.json` с `expected_tool_calls` + `must_not_call` (или PR добавил `.json` файл в `.github/e2e/scenarios/`)
- если НЕТ — `e2e-done` НЕ ставится, issue остаётся в `needs-e2e`, воркер предупреждается

**Trade-off:** +1 шаг для воркера. Цена: ~5 минут. Benefit: прекращает R1, R6.

### 4.2 GATE-2: двушаговая автозакрывалка

**Что:** после PR #1399 (user-reopen guard) — добавить **24-часовой stale-период**:
- если `e2e-done` был поставлен → 24ч наблюдение
- если за 24ч не было ни одного human comment/open-e2e PR, новой активности от живого воркера — close
- если был human-reopen в течение 24ч — reset

**Trade-off:** медленнее закрытие. Цена: реальные closed-циклы становятся 24+. Benefit: прекращает R4.

### 4.3 GATE-3: CI-blocking completion

**Что:** `agent-flow-completion-check.sh` (новый), тикает при попытке `kanban complete`:
- если `gh pr view <N> --json state` = MERGED, проверяет `gh pr checks <N> --json conclusion` — должно быть empty (любая FAILURE = блок)
- если есть CONFLICTING → блок
- если есть `needs-e2e` И нет `e2e-done` → блок

**Trade-off:** +1 скрипт. Цена: каждый worker claims прогоняет проверку. Benefit: прекращает R5.

### 4.4 Альтернатива (отклонённая): «человек подтверждает e2e-done»

- Минус: ручной шаг, нарушает ADR-0014 автоматизацию.
- Вердикт: отклонено.

### 4.5 Альтернатива (отклонённая): «back-to-runner auto-decompose»

- Идея: если issue ≥ 2 root cause → decompose в 2 sub-issue.
- Минус: требует triage-cron (отсутствует с 12.08, issue #1420 P0-2). Без него — не работает.
- Вердикт: отложено до восстановления triage.

### 4.6 Follow-up: что нужно прочитать после PR #1398 merge

- Live-тест на Pi: `music_library_suite_v1.json` запустить через `gh workflow run -f scenario_file=...`.
- Если не работает — это уже не «процесс сломан», а конкретный bug в коде PR #1398. Worker card.

---

## 5. Дополнительные гипотезы (для проверки, не утверждаю)

### H1. Triage-cron отсутствует → workers берут issue в неправильном порядке

Issue #1420 (P0-2): «восстановить agent-flow-triage cron — issues с hermes label не превращаются в kanban-карточки с 12.08». Это значит, что **12.08** — последний день, когда новые issue'ы автоматически становились карточками. Все issue'ы с 12.08 (включая #1358, #1363, #1392, #1403, #1411, #1412, #1363, #1391, #1395) — **созданы руками разработчика** + прокомментированы карточкой вручную. Это не объясняет, почему #1358 «формально гоняется неделю» (issue создан 18.08). Но объясняет, почему **разработчик сам пишет `kanban: t_cc8f9229` в комментарии** — потому что автоматика не сделает.

### H2. PR lifecycle (CI-only) vs e2e-required

ADR-0014 + retro 10.08 #2: PR только с `.github/` и `scripts/agent_flow/` → needs-review, e2e не нужно. PR с `src/`, конфигами, runtime → needs-e2e. PR #1398 (2825 строк, runtime) — **needs-e2e**, и текущий e2e на smoke это пропускает. Это правило работает, но smoke-test ≠ e2e.

### H3. PR #1408 (task review) закрыт CONFLICTING

Возможное пересечение с текущей карточкой t_26d8a61b (process review другой формат). Конфликт можно разрешить позже — **не блокирует расследование**.

---

## 6. Acceptance этой карточки (что сейчас ✅)

- [x] Прочитаны все PR #1375–#1418, связанные с music/whistle
- [x] Прочитаны все issue #1358, #1363, #1392, #1403, #1411, #1412, #1391, #1395, #1404, #1405, #1409, #1411, #1420, #1422
- [x] `gh pr view --json mergeable,conclusion` для каждого (через `gh pr view --json` — `conclusion` не возвращается для не-workflow runs, но `state`/`mergedAt`/`mergeable` есть)
- [x] dialogue_node.py counter init — подтверждено, стабилен после #1395
- [x] music_skill_prompt.txt — подтверждено (Renardo-only, без MiniMax)
- [x] PR #1398 — подтверждено (2825 LOC, 11 файлов, **OPEN**, базовый `develop`)
- [x] PR #1406 — подтверждено (closes #1403, MЕРЖЕН руками по Q22)
- [x] issue #1358 — подтверждено (e2e-done поставлен на smoke, не на music_library_suite)
- [x] issue #1363 — подтверждено (4 раза reopen, последний status unclear из-за CONFLICTING)
- [x] Все 4 ADR прочитаны (0014, 0015, 0018, 0021)

## 7. Что НЕ делал (по карточке)

- ❌ Не фиксил код (запрещено)
- ❌ Не мержил PR (запрещено + Q22)
- ❌ Не закрывал issues #1358/#1363 (запрещено)
- ❌ Не делал live-проверку на 10.1.1.21 (нет SSH-доступа; live-debug = дешёвая операция, но architect должен запрашивать devops-карточку, не лезть ssh'ом)
- ❌ Не ставил sticky-block / re-trigger воркеров

## 8. Связанные

- ADR-0014 (issue closure на merge)
- ADR-0015 (e2e verdict SOT)
- ADR-0018 (честный FAIL лучше красивого PASS)
- ADR-0021 (dialogue_node decomposition) — будет дополнен
- ADR-0022 (новый — process fix proposal, см. след. файл)
- Issue #1420 (P0-2: triage-cron отсутствует)
- Issue #1422 (P0-X: PR #1418 closed with CI red)
- Issue #1391 (autocloser, починен в #1399)
- PR #1398 (MiniMax music, OPEN)
- PR #1406 (music prompt, merged по Q22)
- PR #1418 (SSoT enum, CI red)

## 9. Замечание про «свист» (товарищ Шифу)

Юзер **сам** диагностировал root cause в issue #1363 (12:10 комментарий): sound_node + dmix_respeaker loopback. PR #1366 (scsynth `-l 32`) — **это не тот fix**. Закрытие issue #1363 в 11:22 было ошибкой (юзер reopen в 12:06 это подтвердил). Реальный фикс — в sound_node.py / audio_node.py / asound.conf. Это **отдельная worker-карточка**. Architect не лезет в код (запрещено), но **целевая карточка**:
- `feat(voice #1363): устранить loopback в sound_node / dmix_respeaker при startup_greeting`
- роль: backend + devops
- ветка: `z-{agent}/1363-...` (не плодить, использовать существующую если возможно)
- acceptance: рестарт voice-assistant → 10 секунд тишины → TTS говорит, **свиста нет**

Это **рекомендация**, не инструкция. Карточку завести юзер поручит triažу (когда восстановят, #1420).
