# Отчёт: [process #2159] воркеры сохраняют отчёты в docs/reports/kanban/<task_id>.md + skills в body карточки

**Task ID:** t_77f8ebd8
**Assignee:** devops
**Issue:** #2159  — https://github.com/krikz/rob_box_project/issues/2159
**PR:** #2167  — https://github.com/krikz/rob_box_project/pull/2167
**Branch:** `z-{devops}/2159-feat-process-kanban-reports-v2-with-skills`
**Started:** 2026-09-08T11:41:14+00:00 UTC (claimed event)
**Completed:** 2026-09-08T12:30:00+00:00 UTC (PR opened)
**Duration:** ~50m

## Что сделано

- Создан `scripts/agent_flow/report_template.md` — канонический шаблон отчёта воркера (Что сделано / Файлы / Git log / Raw-evidence / Skill results / PR / Caveats).
- Создан `scripts/agent_flow/kanban-report-write.sh` — генератор отчёта: подставляет git log/diff/branch/дату; воркер дописывает свободные секции руками; НЕ коммитит/НЕ пушит.
- Создан `scripts/agent_flow/tests/test_kanban_report_write.sh` — регресс-тест (8 сценариев: defaults, --title, --pr, --output, no-task-id, no-git-worktree, bad-base-ref, idempotent-overwrite; 11/11 ассертов PASS).
- Обновлён `scripts/agent_flow/install.sh::EXPECTED[]` — добавлен `kanban-report-write.sh` для раскладки в 6 target-папок (drift-detect контролирует).
- Добавлен `scripts/agent_flow/agent-flow-merge-gate.sh::warn_no_worker_report` — мягкий WARN в лог (НЕ блокирующий), если PR ≥3 файлов и нет `docs/reports/kanban/*.md`. Hot-fix ≤2 файлов exempt; 0 файлов silent; null fail-open.
- Вызов `warn_no_worker_report` вставлен в строке MERGEABLE+CLEAN (прямо перед dead-content check).
- Создан `docs/adr/0077-kanban-worker-report-file.md` — ADR (статус Accepted; process change по наказу Шифу 18.08: «не делай руками»).
- rebase на origin/develop (6321dc1b → e17e5bca) — clean.
- PR #2167 создан, base=develop, CI 8 SUCCESS / 1 SKIPPED (Integration Tests, нормально).

## Файлы изменены

```
 docs/adr/0077-kanban-worker-report-file.md         | 187 +++++++++++++++
 scripts/agent_flow/agent-flow-merge-gate.sh        |  48 ++++
 scripts/agent_flow/install.sh                      |   8 +
 scripts/agent_flow/kanban-report-write.sh          | 252 +++++++++++++++++++++
 scripts/agent_flow/report_template.md              | 132 +++++++++++
 .../agent_flow/tests/test_kanban_report_write.sh   | 195 ++++++++++++++++
 6 files changed, 822 insertions(+)
```

## Git log

```
cf00852e wip(process #2159): kanban worker report file — шаблон + генератор + soft-warn в merge-gate
6321dc1b wip(adr #2142): ADR-0075 — расширение ROOM_D для варианта E (4 опции, рекомендую R1) (#2156)
72c27b82 fix(supervisor #2131): MultiThreadedExecutor — on_result callback не голодает (#2153)
6b9d6951 fix(quest #2136): убрать isinstance-guard на msg.data в _on_avatar_tts_audio — шлем оператора молчит из-за array.array (#2154)
598f6128 fix(quest #2135): сегментировать wake-поток шлема в фразы до STT (#2155)
```

## Raw-evidence (pytest / CI / логи)

### pytest (локально)

```
Running test_kanban_report_write.sh against /home/builder/rob_box_project/.worktrees/t_77f8ebd8/scripts/agent_flow/kanban-report-write.sh
--- Scenario 1: A_defaults ---
  OK
--- Scenario 2: B_title_assignee ---
  OK
--- Scenario 3: C_pr_issue ---
  OK
--- Scenario 4: D_custom_output ---
  OK
--- Scenario 5: E_no_task_id ---
  OK
--- Scenario 6: F_no_git_worktree ---
  OK
--- Scenario 7: G_bad_base_ref ---
  OK
--- Scenario 8: H_idempotent_overwrite ---
  OK

===== Result: pass=11 fail=0 =====
fail_msg:
```

### bash -n (синтаксис)

```
$ bash -n scripts/agent_flow/kanban-report-write.sh && echo "syntax OK"
syntax OK
$ bash -n scripts/agent_flow/tests/test_kanban_report_write.sh && echo "syntax OK"
syntax OK
$ bash -n scripts/agent_flow/install.sh && echo "syntax OK"
syntax OK
$ bash -n scripts/agent_flow/agent-flow-merge-gate.sh && echo "syntax OK"
syntax OK
```

### install.sh --list-files (sanity-check EXPECTED)

```
$ bash scripts/agent_flow/install.sh --list-files | grep kanban-report
kanban-report-write.sh
```

### CI (gh pr checks)

- Run #34212584052 (Code Quality / Lint / Shell) — SUCCESS
- Run #34212584039 (TTS Provider Tests) — SUCCESS
- Run #34212584234 (Unit Tests ROS2 Humble) — SUCCESS (2m 6s)
- Test Summary — SUCCESS
- Dockerfile Best Practices — SUCCESS
- YAML/Config Files — SUCCESS
- Lint Summary — SUCCESS
- Integration Tests — SKIPPED (нормально, не наш scope)
- Python Code Quality — SUCCESS

PR #2167 mergeable: MERGEABLE, mergeStateStatus: CLEAN.

### Smoke-тест warn_no_worker_report (offline, без gh)

```
TEST 1 (has report, 2 files): out=''
  PASS
TEST 2 (empty PR → silent): out=''
  PASS
TEST 3 (null fail-open): out=''
  PASS
TEST 4 (3 files no report → WARN): out=WARN: PR #99 has no docs/reports/kanban/...
  PASS
TEST 5 (hot-fix 1 file → silent): out=''
  PASS
TEST 6 (hot-fix 2 files → silent): out=''
  PASS
pass=6 fail=0
```

## Skill results

### verification-before-completion

- [x] `bash -n` на всех 4 .sh файлах — OK.
- [x] `test_kanban_report_write.sh` — 11/11 ассертов PASS (raw-вывод выше).
- [x] `gh pr checks 2167` — 8 SUCCESS / 1 SKIPPED (Integration, нормально).
- [x] `git status` — clean после каждого коммита.
- [x] rebase на origin/develop — clean.
- [x] честный FAIL лучше красивого PASS (ADR-0018): если бы хоть один ассерт упал, я бы НЕ вызывал kanban_complete, а сначала починил.

### code-review (self-review)

Применил самопроверку:
- **Standards** (CONTRIBUTING.md / AGENTS.md): код следует существующему стилю agent-flow (set -euo pipefail, локальные переменные, python3-парсеры для JSON — здесь заменены на grep+wc для sandbox-friendly).
- **Smell baseline**: дублирования нет (один warn_no_worker_report вместо inline), мистических имён нет, `local var=$(...)` для subshell-isolation.
- **Spec** (issue #2159 + #2162):
  - Требование «шаблон отчёта» → `report_template.md` ✅
  - Требование «генератор отчёта» → `kanban-report-write.sh` ✅
  - Требование «тест» → `test_kanban_report_write.sh` ✅
  - Требование «не блокировать kanban_complete» → soft WARN в merge-gate, не exit 1 ✅
  - Требование «не менять hermes-kanban CLI» → НЕ трогал kanban_db / kanban_complete ✅
  - Требование «не менять другие скилы/скрипты» → НЕ трогал triage / handoff / e2e-process ✅
  - Требование issue #2162 «skills в body» → `## Skill results` секция в шаблоне, воркер дописывает ✅
  - ADR-0077 создан ✅

### writing-for-agents

Body этой карточки содержит чёткие секции (Skills, Skills-порядок, Контекст, Что нужно, DoD, Связанные, Контракт, Что НЕ делать). Следующий воркер получит:
- Готовый шаблон отчёта (не нужно изобретать).
- Готовый генератор (не нужно писать bash-обвязку).
- Готовый регресс-тест (не нужно придумывать, что тестировать).
- ADR-0077 со ссылками на issue #2159 / #2162 / AGENTS.md / ADR-0018.
- Контракт из 4 шагов (generate → edit → push → kanban_complete).

Что улучшить: можно добавить в `agent-flow-triage.sh` шаблон с `## Skills` секцией по умолчанию (issue #2162 фикс), но это **отдельная карточка** (issue #2159 явно: «не менять другие скилы»).

### senior-devops

CI/CD best practices:
- **Immutable infrastructure**: генератор отчёта лежит в репо (SOT), раскладывается через `install.sh` hardlink-копиями (cp -al), drift-detect контролирует 6 target-папок (паттерн из ADR-AF-0022).
- **Observability**: WARN в stderr + лог merge-gate (без gh issue comment — не шумим). Если по retро увидим «50% PR без отчёта» → ужесточим в `ADR-0077+1`.
- **Fail-open**: gh вернул null → silent (не ломаем merge-gate на flake). BAD_REF base → fallback на HEAD~1..HEAD с пометкой в файл.
- **Sandbox-friendly**: без `python3 -c` (sandbox в hermes блокирует `-c` flag, см. ADR-0077 §2.4). Только `grep -oE` + `wc -l`.
- **Test coverage**: 8 сценариев / 11 ассертов для генератора + 6 сценариев для warn_no_worker_report.

## PR / Issue ссылки

- PR #2167 — https://github.com/krikz/rob_box_project/pull/2167
- Issue #2159 — https://github.com/krikz/rob_box_project/issues/2159
- Issue #2162 — https://github.com/krikz/rob_box_project/issues/2162 (комплементарный, не закрыт)
- kanban t_84434d4c — предыдущая итерация этой карточки (PARSE_ERROR при автотесте, был сброшен — этот PR учитывает фейлы)

## Замечания / Caveats

- **Soft gate, не жёсткий**: по наказу Шифу 18.08 «не делать руками» (= не ломать существующий flow жёсткой блокировкой). Если в следующем ретро увидим <30% PR с отчётами — ужесточим в `ADR-0077+1`.
- **Integration Tests skipped**: это нормально для нашего PR (мы не трогаем ros2 workspace, только process scripts). См. statusCheckRollup.
- **python3 -c исключён**: в текущей sandbox среде `python3 -c` блокируется. В CI-runner и в hermes-builder он работает. Поэтому я убрал python3 из `warn_no_worker_report` (только grep+wc).
- **Не проверено вживую на железе**: e2e-процесс не запускал (это сделает e2e-process после merge). По контракту — не наш scope.
- **Follow-up карточки (НЕ делал сам)**:
  - Жёсткий gate (если через месяц ретро покажет <30%): `kanban-create ADR-0077+1`.
  - Авто-вставка `## Skills` секции в `agent-flow-triage.sh` (issue #2162 фикс).
  - Расширение `af_skill_for_profile` на multi-skill (per t_aafad606, kanban).
- **Что НЕ сделано (по DoD карточки)**: всё сделано, кроме «CI прошёл» — это требует merge, что вне scope воркера. Сделает merge-gate.
