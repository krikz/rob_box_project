# RECON-1571 — worktree stale HEAD drift (48+ commits)

> Status: investigation complete, implementation in progress.
> Issue: krikz/rob_box_project#1571
> Reporter: architect (23.08.2026)
> Author of fix: devops (t_07799ca7)

## Симптом

`_ensure_git_worktree` создаёт новую ветку от **локального `HEAD`** главного worktree
(`/home/builder/hermes-share/rob_box_project`). Если главный worktree давно не делал
`git fetch origin develop && git pull --rebase` — его HEAD отстаёт от origin/develop
на десятки коммитов (наблюдалось 48–50+ коммитов на четырёх последних карточках).

## Корневая причина

`/home/builder/.hermes/hermes-agent/hermes_cli/kanban_db.py:8215`:

```python
cmd = [
    "git", "-C", str(repo_root), "worktree", "add", "-b", branch_name,
    str(target), "HEAD",          # ← вот тут баг
]
```

Когда ветка `branch_name` НЕ существует (новая карточка) — `worktree add -b <new>`
берёт base из локального `HEAD`, который может быть устаревшим.

## Архитектура

- `scripts/agent_flow/vendor/hermes-agent-*.patch` — vendor patches.
- `install.sh` (line ~454) автоматически применяет ВСЕ `vendor/hermes-agent-*.patch`
  в `/home/builder/.hermes/hermes-agent/`. Не нужно править EXPECTED list.
- Существующий `hermes-agent-spawn-worktree-precheck.patch` уже добавляет:
  - `_find_worktree_for_branch`
  - `WorktreeBranchBusyError`
  - force_trip=True routing в dispatch_once (ready + review loops)
- Регрессионный тест `tests/test_spawn_worktree_collision_reuse_block.sh` —
  рабочий шаблон: применяет patch в чистый origin/main worktree, гоняет
  функциональные сценарии через `python3 -c`, проверяет apply/reverse idempotency.

## Acceptance из issue #1571

- [ ] Перед `git worktree add` делать `git fetch origin develop` в главном worktree.
- [ ] Если `origin/develop` newer чем HEAD — использовать `origin/develop` как base.
- [ ] Явная опция `--branch-base origin/develop` в `hermes kanban create` — отложено
      до отдельной задачи (требует изменения CLI, scope выходит за пределы bug fix).
- [ ] При dispatch карточки с drift > 10 коммитов — auto-rebase или warning.

## Реализация (план)

1. Новый vendor patch: `hermes-agent-spawn-base-origin-develop.patch`.
2. Helper `_resolve_worktree_base_ref(repo_root, base_branch="develop")`:
   - `git fetch origin <base_branch>` (таймаут 30с)
   - OK → возвращает `f"origin/{base_branch}"`
   - FAIL → возвращает `"HEAD"` + warning в stderr
     (`WORKTREE_BASE_FETCH_FAILED: ...`)
3. Helper `_warn_worktree_base_drift(repo_root, threshold=10)`:
   - `git rev-list --count HEAD..origin/develop`
   - если > threshold → warning в stderr
     (`WORKTREE_BASE_DRIFT: HEAD..origin/develop = N commits`)
4. Изменение в `_ensure_git_worktree` else-ветке:
   - `base_ref = _resolve_worktree_base_ref(repo_root)`
   - `cmd = [..., base_ref]`
   - Вызвать `_warn_worktree_base_drift(repo_root)` ДО вызова `worktree add`
     (когда branch не существует — самое опасное место).
5. НЕ трогаем ветку-уже-существует (там checkout существующего branch).
6. Backward-compat: при fetch fail — fallback на HEAD + warning. Никаких regressions.

## Acceptance #4 — частичное покрытие

`auto-rebase при drift > 10` в этом PR — только warning в dispatch log.
Полный auto-rebase требует логики в worker startup (`kanban.py:dispatch_once`)
которая знает контекст карточки — отдельный PR. Issue #1571 это допускает.

## Out-of-scope

- `--branch-base` CLI flag для `hermes kanban create` (отдельная карточка).
- Auto-rebase в воркере (отдельная карточка).