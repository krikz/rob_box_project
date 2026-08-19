# Ретро-отчёт: process-fix issues (hermes) stuck OPEN при merged PR — t_498dc624

**Дата:** 2026-08-19 (01:00 MSK — наблюдение, 03:30 MSK — фикс)
**Архитектор наблюдения:** architect
**Исполнитель фикса:** devops
**Карточка:** kanban t_498dc624
**retro-key:** `process-fix-hermes-stuck-open`

## 1. Симптом

5 issues в krikz/rob_box_project имели status OPEN, хотя соответствующие PR были MERGED в `develop`:

| Issue | PR | Когда PR merged | Kanban-карточка | Labels |
|------:|----:|------------------|-----------------|--------|
| #1421 | #1425 | 2026-08-18 17:51 UTC | t_8d6b7268 done | `bug, priority:high, process, hermes, agent:devops, e2e` |
| #1419 | #1424 | 2026-08-18 17:20 UTC | t_88f3ec13 archived | `bug, priority:high, process, hermes, agent:devops` |
| #1404 | #1414 | 2026-08-18 16:56 UTC | t_e068b88f done | `process, hermes, agent:architect, task, refactor` |
| #1412 | #1430 | 2026-08-18 18:13 UTC | t_79e9417c done | `voice, hermes, agent:devops, task, startup` |
| #1444 | — (нет PR) | — | — (отсутствует) | `bug` only |

Все 4 issues с PR имели общий признак: метка `hermes` (= `ISSUE_LABEL`) + process-fix PR (только CI/scripts/docs).

#1444 — отдельная аномалия: тикет без triage, body = "Please see local body file".

## 2. Root cause

В `scripts/agent_flow/agent-flow-merge-gate.sh` (SOT) ретро-путь (строки 2688-2759) имел
два независимых дефекта:

### Дефект 2.1 — `hermes` skip'ает ретро-путь безусловно

Условие skip:

```bash
elif has_label "$_r_labels_norm" "$ISSUE_LABEL" \   # ISSUE_LABEL=hermes
    || has_label "$_r_labels_norm" "$NEEDS_E2E_LABEL" \
    || has_label "$_r_labels_norm" "$DONE_LABEL" \
    || has_label "$_r_labels_norm" "$NO_E2E_LABEL" \
    || has_label "$_r_labels_norm" "$NEEDS_REVIEW_LABEL"; then
    log "retro-path: issue #${r_issue} уже в process-цикле — skip"
    continue
fi
```

Проблема: наличие одной лишь `hermes` (= "это issue в нашем pipeline") приравнивалось
к "уже в process-цикле". Но `hermes` — это **identity-метка**, не workflow-метка. Workflow-
метки — это `needs-e2e` (ждёт e2e), `e2e-done` (e2e прошла), `no-e2e-required` (CI-only),
`needs-review` (на ручном ревью).

Petля:
- issue с `hermes` (только) попадает в основной цикл → основной цикл требует `e2e-done`
  для close (ретро-путь основного цикла, ветка "MERGED but no e2e-done → leave OPEN",
  см. `merge-gate.sh:~1100`).
- `e2e-process` берёт issue в ротацию, но для process-fix PR (только `.github/`,
  `scripts/agent_flow/`, `docs/`) e2e-run технически бессмысленен — код не меняет
  поведение робота, и e2e-process скипает его.
- `e2e-done` никогда не ставится → issue вечно OPEN.
- Ретро-путь мог бы закрыть через PASS-доказательство (e2e run SUCCESS или CI-only +
  зелёный CI), но skip по `hermes` блокировал.

Исторически фикс для проблемы 1 в `merge-gate.sh:2610-2625` (t_68607832) покрыл только
issues БЕЗ меток. Для issues С меткой `hermes` gap остался.

### Дефект 2.2 — CI-only фильтр не покрывает `.hermes/plans/`

CI-only фильтр (используется и в ретро-пути, и в clean-pr-sweep) считал CI-only только
PR, меняющие файлы в `.github/`, `scripts/agent_flow/`, `docs/`:

```python
f.startswith(".github/") or f.startswith("scripts/agent_flow/") or f.startswith("docs/")
```

PR #1414 (`.hermes/plans/process-fix-roadmap.md`) — процессный фикс roadmap — не
считался CI-only → issue #1404 висел OPEN при зелёном CI.

Аналогично потенциально другие процессные PR, меняющие `.hermes/profiles/` или
другие process-meta файлы.

## 3. Решение

### 3.1 Расширение skip-логики ретро-пути

`ISSUE_LABEL=hermes` больше не в skip-условии. Skip ТОЛЬКО при наличии workflow-метки
(needs-e2e / e2e-done / no-e2e-required / needs-review) — это и есть взаимоисключаемость
с основным циклом.

```diff
-    elif has_label "$_r_labels_norm" "$ISSUE_LABEL" \
-        || has_label "$_r_labels_norm" "$NEEDS_E2E_LABEL" \
+    elif has_label "$_r_labels_norm" "$NEEDS_E2E_LABEL" \
         || has_label "$_r_labels_norm" "$DONE_LABEL" \
         || has_label "$_r_labels_norm" "$NO_E2E_LABEL" \
         || has_label "$_r_labels_norm" "$NEEDS_REVIEW_LABEL"; then
         log "retro-path: issue #${r_issue} уже в process-цикле — skip"
         continue
     fi
+    # Ретро 19.08 t_498dc624: hermes БЕЗ workflow-меток = process-fix
+    # issue. Ретро-путь должен иметь шанс закрыть через PASS-доказательство.
+    # Workflow-метки выше по-прежнему skip'ают.
```

### 3.2 Расширение CI-only фильтра

```diff
         ok = bool(files) and all(
-            f.startswith(".github/") or f.startswith("scripts/agent_flow/") or f.startswith("docs/")
+            f.startswith(".github/")
+            or f.startswith("scripts/agent_flow/")
+            or f.startswith("docs/")
+            or f.startswith(".hermes/plans/")
             for f in files
         )
```

Покрытие: `.hermes/plans/process-fix-roadmap.md` (PR #1414) теперь считается CI-only.

### 3.3 Тесты — `test_merge_gate_retro_path.sh`

Добавлено 4 теста (M/N/O/P), 28/28 проходят:

- **M.** hermes process-fix + `.hermes/plans/` CI-only green → close (#1404 + PR #1414)
- **N.** hermes bug + `scripts/agent_flow/` CI-only green → close (#1421 + PR #1425, #1419 + PR #1424)
- **O.** hermes + needs-e2e → skip (regression-guard: активный e2e-цикл не трогаем)
- **P.** hermes + не-CI-only PR + нет e2e → needs-e2e (e2e-process возьмёт в ротацию)

Существующие 12 retro-path тестов прошли без изменений — регрессии нет.

### 3.4 #1444 — отдельная карточка для триажа

#1444 — аномалия: тикет без triage, body = "Please see local body file", нет kanban-
маркера, нет PR. Не относится к root cause выше. Создаём отдельную kanban-карточку
на триаж.

### 3.5 Комментарии на #1421/#1419/#1404/#1412

После merge фикса merge-gate следующий 5m-тик увидит эти issues и закроет их по
ретро-пути (PASS-доказательство: CI-only PR + зелёный CI). Шаги:

1. Merge PR с фиксом.
2. Дождаться следующего тика merge-gate (5m).
3. Проверить issues — должны быть CLOSED с комментарием "✅ ретро-путь".

Если по какой-то причине тик не закрыл issues в течение часа — fallback: ручной
`gh issue close <N> --reason completed` с комментарием-ссылкой на PR.

## 4. Изменённые файлы

| Файл | Изменения |
|------|-----------|
| `scripts/agent_flow/agent-flow-merge-gate.sh` | skip-логика ретро-пути, CI-only фильтр |
| `scripts/agent_flow/tests/test_merge_gate_retro_path.sh` | +4 теста (M/N/O/P), переупорядочен run-test блок |

После merge: `bash scripts/agent_flow/install.sh` разложит hardlinks по
`~/.hermes/profiles/{agent-flow,architect}/scripts/` и `~/.hermes/scripts/`.

## 5. Lessons / follow-up

1. **Identity-метки ≠ workflow-метки.** Skip-условия в ретро-пути (и любых подобных
   "уже в process-цикле" guard'ах) должны разделять эти категории. Identity-метки
   (hermes, agent:*) не блокируют ретро-путь сами по себе; блокируют только
   workflow-метки (needs-e2e, e2e-done, no-e2e-required, needs-review).

2. **CI-only фильтр — это whitelist, а не open-set.** Если мы добавляем новые
   директории процессных файлов (`.hermes/profiles/`, `.cursorrules/`, ...), их
   тоже нужно добавлять в CI-only. Альтернатива: blacklist — `not in
   [src/robot_*/]` — может быть проще в долгосрочной перспективе (не требует
   обновления при добавлении директорий). Решение отложено.

3. **Issue без triage — отдельный класс проблем.** #1444 (body "Please see local
   body file", 1h ago) — это не process-fix, а triage-gap. Нужна отдельная
   kanban-карточка для triage-spec профиля.

4. **Pre-existing fail в `test_merge_gate_clean_pr_sweep.sh::C5`** — не связан с
   этим фиксом (reproduce на чистом origin/develop). Оставить для отдельной
   карточки.
