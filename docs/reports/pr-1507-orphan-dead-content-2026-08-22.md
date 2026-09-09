# PR #1507 orphan-dead после rebase: ретро-анализ

**retkey:** pr-1507-orphan-dead-content-after-rebase
**Дата:** 2026-08-22
**Автор:** architect (t_944df2c5)
**Issue:** #1506 (e2e-валидация голосовых фич)
**PR:** #1507, #1508, #1509, #1510, #1511
**Связанные ретро-карточки:** t_887c5c64, t_d407c723, t_4cf2811e, t_a8fb9137, t_9e61d788

---

## TL;DR

PR #1507 (`z-{agent}/1506-task-voice-e2e-command-gate-new-session-`) после rebase на develop стал **orphan-dead**: diff vs develop содержал только 6 voice .ogg (≈94KB), а вся смысловая часть (suite / acceptance / analysis) уже была в develop (4 коммита `bd1ca0ef / 7f02b593 / 1abd4465 / 947ffd8e`, ветка `develop-suite-v1` слита ранее). `merge-gate signal 3` (PR #1509, смержен 22.08 07:42Z) разрешил лимб — нашёл sibling PR #1508 с `e2e-done`, поставил ему `needs-review`, issue #1506 закрыт 07:45Z. **Но 8 acceptance-пунктов #1506 физически не прогонялись на реальном e2e** (round-166/167/168 — инфра-фейлы, round-169 — инфра-фикс #1508, не голосовые фичи). Процесс прошёл по правилам, но **acceptance был подменён инфра-прогоном** — silent skip валидации.

Корневая причина: **add/add конфликт при rebase был разрешён в пользу develop без guard'а "что осталось в PR"**. PR стал формально MERGEABLE без меток и без CI-фейлов, но **потерял смысл**.

---

## Timeline (фактический, raw evidence)

| Время (UTC) | Событие | Артефакт |
|---|---|---|
| 21.08 21:08 | Issue #1506 создан (type:testing + needs-e2e) | issue #1506 |
| 21.08 21:33 | kanban карточка t_76c783eb создана | comment |
| 21.08 21:55 | PR #1507 создан: 6 .ogg + suite + acceptance + analysis | gh pr view 1507 |
| 21.08 22:12 | round-166 e2e FAIL (инфра: «No such file» в артефактах) | run #32531463344 + comment "## 📊 e2e-доклад #1506 — FAILURE" |
| 21.08 22:12 | issue #1506 → e2e:rejected + needs-e2e снят | timeline event |
| 21.08 22:26 | re-test → needs-e2e | timeline event |
| 21.08 23:06 | PR #1508 создан: инфра-фикс upload-artifact '?' | gh pr view 1508 |
| 22.08 01:51 | PR #1509 создан: signal 3 | gh pr view 1509 |
| 22.08 03:27 | retro t_d407c723 разморозка (signal 3 merge готов) | comment |
| 22.08 03:32 | signal 3 смержен в develop (7a6f633b) | commit 7a6f633b |
| 22.08 04:46 | retro t_4cf2811e → CRLF fix → PR #1510 | comment |
| 22.08 06:19 | PR #1511 stale-rebase watchdog | gh pr view 1511 |
| 22.08 07:42 | PR #1511 → merge develop | merge 92acea3f |
| 22.08 07:42 | PR #1509 → merge develop (signal 3 в кроне) | merge 86044a04 |
| 22.08 07:43 | PR #1508 → merge develop (инфра-фикс) | merge 6a5b653f |
| 22.08 07:44 | PR #1510 → merge develop (CRLF fix) | merge 2d4fc072 |
| 22.08 07:45 | issue #1506 CLOSED (через merge-gate signal 3 → needs-review на #1508 → merge Шифу) | timeline event |

---

## Diff vs base: что реально осталось в PR #1507 после rebase

```bash
$ gh pr diff 1507 --name-only
.github/e2e/voice_commands/rabot_gde_ty.ogg
.github/e2e/voice_commands/rabot_govori_golosom_aleny.ogg
.github/e2e/voice_commands/rabot_rasskazhi_skazku_raznymi_golosami.ogg
.github/e2e/voice_commands/rabot_sbros_vsyo.ogg
.github/e2e/voice_commands/rabot_skazhi_privet.ogg
.github/e2e/voice_commands/rabotoks_kak_dela.ogg
```

`additions: 0`, `deletions: 0`, `changedFiles: 6` (только binary .ogg).

**Сравнение с develop (что уже там)**:

```bash
$ git log origin/develop --oneline -- .github/e2e/scenarios/voice_core_*.json | head
947ffd8e test(e2e): voice core suite — dispute→arbiter (multi-voice, sequential replies)
1abd4465 test(e2e): voice core suite v2 — add set_voice steps, drop OOS backlog/multi-speaker
7f02b593 test(e2e): voice core suite — music start via Renardo before stop, exclude MiniMax generation + delete
bd1ca0ef test(e2e): voice core suite — command gate, new-session reset, wake word, music stop
```

→ PR принёс в develop только 6 orphan .ogg; вся `.json` + `.md` инфраструктура уже была.

---

## Round branches (что реально гонялось)

```bash
$ git log origin/z-{e2e}/test-round-{166,167,168,169} --oneline -3
round-166: 8a4b1dd3 agent-flow: merge z-{agent}/1506-task-voice-e2e-command-gate-new-session- for issue #1506
round-167: b4aafeee agent-flow: merge ... for issue #1507
round-168: 37586295 agent-flow: merge ... for issue #1507
round-169: f0c3f33b agent-flow: merge z-devops/t_a8fb9137-e2e-upload-artifact-question-mark-legacy-collect for issue #1508
```

- **round-166** — мержит PR #1507, инфра-FAIL «No such file»
- **round-167** — ретрай PR #1507 (тоже инфра)
- **round-168** — ретрай PR #1507 (тоже инфра)
- **round-169** — мержит PR #1508 (инфра-фикс upload-artifact), SUCCESS — но это **фикс инфры**, не прогон 8 voice acceptance-пунктов

**acceptance.json ни в одном раунде не выполнился на реальном роботе для 8 пунктов #1506.**

---

## Архитектурный анализ (3 корня)

### Корень 1 — dead-content PR после rebase (главный)

**Что произошло**: PR #1507 был основан на develop-SHA, в котором ещё не было suite/acceptance (они пришли позже через PR develop-suite-v1). После rebase на develop add/add конфликт был разрешён через `git checkout origin/develop -- <files>` (ретро-комментарий воркера t_887c5c64 — «PR потерял смысл, 6 orphan .ogg»). **Никто не проверил «что осталось в PR после rebase»**.

**Почему merge-gate не заметил**: guard'ы merge-gate проверяют только:
- PR state = OPEN
- base = develop
- mergeable = MERGEABLE
- merge_state = CLEAN
- checks = SUCCESS
- additions > 0 / files > 0

Последний пункт формально пройден (6 .ogg ≠ 0 файлов), но **смысловой дифф = 0** (suite/acceptance уже в develop).

**Impact**: PR висит OPEN MERGEABLE без меток. Если бы merge-gate не нашёл sibling #1508 с e2e-done (signal 3), PR #1507 висел бы вечно (как rebase-orphan).

**Fix (proposed)** — merge-gate: dead-content guard после rebase.

```bash
# После rebase (или после push), если diff vs base содержит ТОЛЬКО binary-файлы
# (голос, картинки, шрифты) И ни одного meaningful-файла (.py/.json/.yaml/.md/.sh/.ts/.cpp/.h):
#   - пометить PR label="dead-content"
#   - прокомментировать в issue: "PR потерял смысл после rebase — только N binary, 0 meaningful"
#   - снять e2e-done/e2e:rejected не нужно, но ПОСТАВИТЬ блок на merge
#
# Реализация (agent-flow-merge-gate.sh, в районе 1657 — после signal 3):
_dead_flag=""
if [ "$pr_state" = "OPEN" ] && [ "$pr_mergeable" = "MERGEABLE" ]; then
    _diff_files_json="$(gh pr diff "$pr_number" --repo "$GH_REPO" --name-only 2>/dev/null || true)"
    _meaningful_count="$(printf '%s\n' "$_diff_files_json" \
        | grep -cE '\.(py|json|ya?ml|toml|md|sh|ts|cpp|h|hpp|launch\.xml|setup\.py)$' || true)"
    if [ "${_meaningful_count:-0}" -eq 0 ] && [ -n "$_diff_files_json" ]; then
        _dead_flag=1
    fi
fi
if [ -n "$_dead_flag" ]; then
    log "issue #${number}: PR #${pr_number} — DEAD-CONTENT (только binary после rebase)"
    gh pr edit "$pr_number" --repo "$GH_REPO" --add-label "dead-content" >/dev/null 2>&1 || true
    gh issue comment "$number" --repo "$GH_REPO" --body \
        "agent-flow: ⚠️ PR #${pr_number} помечен dead-content — после rebase diff vs base содержит только binary (голосовые .ogg), 0 meaningful файлов. Suite/acceptance уже в develop. Рекомендация: закрыть PR без merge." >/dev/null 2>&1 || true
fi
```

**Почему важно**: 6 orphan .ogg ≈94KB — мёртвый груз, который при merge тянет в develop артефакты без сценария.

### Корень 2 — signal 3 для type:testing (позволяет инфра-фиксу закрыть тест-issue)

**Что произошло**: signal 3 (PR #1509) нашёл sibling PR #1508 с `e2e-done` (round-169 SUCCESS), поставил ему `needs-review`, issue #1506 закрыт по merge #1508. **Но #1508 — инфра-фикс upload-artifact '?'**, не голосовая фича. Acceptance-пункты #1506 не прогонялись.

**Почему**: signal 3 проверяет только `e2e-done` на PR, **не различает data-only / infra-fix / feature**. Issue закрывается через merge любого sibling PR с e2e-done.

**Impact**: e2e-issue с `type:testing` может закрыться через инфра-фикс, который сам по себе SUCCESS, но acceptance исходной задачи — не выполнен.

**Fix (proposed)** — signal 3: type:testing gate.

```bash
# В signal 3 (после нахождения _sibling_done_pr), если issue имеет type:testing:
#   - НЕ закрывать через sibling, а ставить needs-review на data-only PR (не на sibling)
#   - комментарий в issue: "issue #N — type:testing; signal 3 sibling-прогон не валидирует
#     acceptance, требуется прямой e2e для исходного PR. Если sibling инфра-фикс — закрытие
#     отменяется, требуется ручная верификация Шифу"
#
# Реализация (вокруг строки 1690):
_issue_has_testing="$(printf '%s' "$labels_norm" | grep -Ec 'type:testing' || echo 0)"
if [ "${_issue_has_testing:-0}" -gt 0 ] && [ -n "${_sibling_done_pr:-}" ]; then
    # Для type:testing закрытие через sibling невалидно — sibling может быть инфра-фиксом.
    # Ставим needs-review на исходный PR (data-only артефакт) + warning.
    gh pr edit "$pr_number" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1 || true
    gh pr edit "$pr_number" --repo "$GH_REPO" --remove-label "$REJECTED_LABEL" >/dev/null 2>&1 || true
    gh issue comment "$number" --repo "$GH_REPO" --body \
        "agent-flow: ⚠️ issue #${number} имеет type:testing — закрытие через signal 3 sibling
НЕ валидирует acceptance-пункты. PR #${pr_number} (data-only) поставлен на needs-review;
требуется ручная проверка Шифу, что 8 acceptance-пунктов #${number} выполнены (или будут
выполнены при следующем e2e-раунде)." >/dev/null 2>&1 || true
    labeled=$((labeled+1)); continue
fi
```

**Альтернатива (более чистая архитектура)**: signal 3 вообще не должен закрывать type:testing — только помечать на ручную проверку. Закрытие type:testing — **всегда через ручной мерж Шифу + подтверждённый acceptance.json** (т.е. через `e2e-done` на ИСХОДНОМ PR, не на sibling).

### Корень 3 — e2e-process не различает type:testing vs feature

**Что произошло**: e2e-process фильтрует issue по `needs-e2e` (line 580+ skip-rule) и `e2e-done / e2e:rejected`. **Не различает type:testing** — issue с type:testing обрабатываются как обычные feature-задачи.

**Почему**: для type:testing требуется:
- реальный e2e-прогон на роботе (не smoke-default «спой песенку про енотика»)
- per-step acceptance.json (expected_tool_calls, must_not_call, expected_artifacts)
- baseline ASR/RMS сравнение

E2e-process сейчас падает обратно на smoke-default, если scenario_file пустой (см. e2e_voice_test.sh). Для type:testing это **silent skip валидации** — issue типа "протестируй X" закрывается через прогон "спой песенку".

**Impact**: type:testing задачи в принципе не могут быть валидно пройдены через текущий e2e-process без явного scenario_file + acceptance_file + реального робота.

**Fix (proposed)** — e2e-process: type:testing gate.

```bash
# В e2e-process (после skip-rule e2e-done/e2e:rejected, line 1533), если issue type:testing:
#   - требовать e2e_scenario_file И e2e_acceptance_file (иначе skip с warning)
#   - не использовать smoke-default fallback для type:testing
#
# Реализация:
_has_testing="$(printf '%s' "$labels_norm" | grep -Ec 'type:testing' || echo 0)"
if [ "${_has_testing:-0}" -gt 0 ]; then
    if [ -z "$e2e_scenario_file" ] || [ -z "$e2e_acceptance_file" ]; then
        log "issue #${number}: type:testing, но scenario_file=${e2e_scenario_file:-empty} acceptance_file=${e2e_acceptance_file:-empty} — skip (нужен явный e2e артефакт)"
        skipped=$((skipped+1)); continue
    fi
fi
```

**Дополнительно** — фикс smoke-default fallback в `e2e_voice_test.sh` (line 30): если `voice_text=""` И передан `scenario_file` — не падать в default "енот", а возвращать ошибку "scenario_file без voice_text не поддерживается, нужно явно прогнать scenario".

---

## Решения, которые были приняты правильно

Сигнал 3 (PR #1509) — **архитектурно правильное решение для не-тестинг задач**. Суть: e2e:rejected issue разблокируется, если есть sibling PR с e2e-done. Это работает для:
- feature-issue, где есть data-only PR (голос/сценарий) + infra-fix PR (фикс пайплайна). Инфра-фикс не должен блокировать закрытие feature-issue.
- general процесс, где rebase-orphan больше не висит вечно.

**Проблема только в type:testing**: для тест-задачи signal 3 подменяет acceptance инфра-прогоном.

---

## Процессные фиксы (приоритизированные)

### P0 — process-fix (атомарно, develop)

1. **merge-gate: dead-content guard** (корень 1)
   - После rebase: если diff vs base содержит 0 meaningful файлов (.py/.json/.yaml/.md/.sh/.ts), label=`dead-content`
   - Комментарий в issue + PR
   - assignee=devops, фикс в `agent-flow-merge-gate.sh` (после signal 3, ~строка 1697)
   - Тест: `test_merge_gate_dead_content.sh` — L1: только binary → dead-content, L2: meaningful + binary → ok

2. **merge-gate: type:testing gate в signal 3** (корень 2)
   - Для issue с `type:testing` signal 3 НЕ закрывает, а ставит needs-review на data-only PR + warning
   - assignee=devops, фикс в `agent-flow-merge-gate.sh` (вокруг строки 1690)
   - Тест: `test_merge_gate_type_testing.sh` — L1: type:testing + e2e-done sibling → needs-review на data-only, L2: feature + e2e-done sibling → close (старое поведение)

3. **e2e-process: type:testing gate** (корень 3)
   - Для type:testing требовать scenario_file + acceptance_file, иначе skip
   - assignee=devops, фикс в `agent-flow-e2e-process.sh` (после line 1533)
   - Тест: `test_e2e_process_type_testing.sh` — L1: type:testing без scenario_file → skip, L2: с обоими → прогон

### P1 — наблюдение (без кода)

4. **round-169 verdict re-classification**: round-169 прогонял PR #1508 (инфра), но в журнале имеет "issue #1508" — недвусмысленно. Если бы лог-парсер выделял «issue #N = прогон acceptance N», сейчас видно «прогон инфры ≠ acceptance». Достаточно grep'ать `git log origin/z-{e2e}/test-round-N --merges` на subject "for issue #N" и сверять с acceptance.

### P2 — после фиксов

5. **Re-run реального e2e для 8 acceptance-пунктов #1506**: после фиксов выше создать fresh PR с обновлённой suite + acceptance + явным label `type:testing`, прогнать на роботе через `gh workflow run` с явным scenario_file.

---

## Немедленные операционные шаги (assignee: architect)

1. **Закрыть PR #1507** (assignee: devops) — orphan-dead, 6 .ogg без suite/acceptance (suite/acceptance уже в develop). Действие:
   ```bash
   gh pr close 1507 --comment "$(cat <<'EOF'
   Закрываю как orphan-dead: diff vs develop = только 6 voice .ogg (≈94KB),
   0 meaningful файлов. Suite/acceptance в develop-коммитах
   bd1ca0ef/7f02b593/1abd4465/947ffd8e уже содержат всю сценарную
   инфраструктуру. Issue #1506 закрыт через merge-gate signal 3 +
   sibling PR #1508 (инфра-фикс upload-artifact '?').

   ⚠️ Фактический e2e-прогон 8 acceptance-пунктов #1506 на роботе
   НЕ был выполнен (round-166/167/168 — инфра-фейлы, round-169 —
   инфра-фикс #1508, не голосовые фичи). Рекомендую воркеру
   повторно прогнать voice_core_suite_v1.json с явным acceptance.json
   на реальном роботе в следующих e2e-раундах.

   Retro: t_944df2c5, ref-key pr-1507-orphan-dead-content-after-rebase
   EOF
   )"
   ```
2. **Удалить ветку** `z-{agent}/1506-task-voice-e2e-command-gate-new-session-` после закрытия PR
3. **Создать дочернюю карточку** в kanban на процессные фиксы P0 (dead-content guard + type:testing gate) — assignee=devops
4. **Создать дочернюю карточку** в kanban на повторный e2e-прогон 8 acceptance #1506 — assignee=devops (отдельный PR + явный scenario_file + явный acceptance_file)

---

## Артефакты для трассировки

- issue #1506 events timeline (метки, комменты, close)
- PR #1507 diff vs develop (6 .ogg)
- PR #1509 commit 7a6f633b (signal 3)
- PR #1511 commit deb29669 (stale-rebase watchdog)
- rounds 166/167/168/169 log
- branch `z-{agent}/1506-task-voice-e2e-command-gate-new-session-` (для удаления)
- ветки develop-suite-v1 (4 коммита suite/acceptance) — merged
- test script (предлагаемый): `scripts/agent_flow/tests/test_merge_gate_dead_content.sh`
- test script (предлагаемый): `scripts/agent_flow/tests/test_merge_gate_type_testing.sh`
- test script (предлагаемый): `scripts/agent_flow/tests/test_e2e_process_type_testing.sh`

---

## Связанные ретро

- `t_887c5c64` — rebase done, воркер заметил «PR потерял смысл»
- `t_d407c723` — signal 3 retro, рекомендация merge порядка #1507+#1508+#1509
- `t_4cf2811e` — CRLF fix для round-166 «No such file»
- `t_a8fb9137` — upload-artifact '?' fix
- `t_9e61d788` — signal 3 implementation, привёл к merge #1509
- `t_944df2c5` — этот ретро-анализ