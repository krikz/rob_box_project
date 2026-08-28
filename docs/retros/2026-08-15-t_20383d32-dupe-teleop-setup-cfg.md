# Retro t_20383d32 — дубль-фикс teleop setup.cfg: PR #1262 и PR #1267 (2026-08-15)

**Kanban:** t_20383d32 (run 1587, devops profile)
**Ретро-key:** dupe-teleop-setup-cfg-pr
**Источник:** надзор архитектора 15.08, тик ~07:10+02:00

## TL;DR

Две параллельные карточки пришли к одному корневому фиксу: **PR #1262**
(`z-devops/t_392d6000-teleop-setup-cfg`, needs-review, CLEAN) и **PR #1267**
(`z-{agent}/1266-...-round-114`, e2e-done+needs-review, CLEAN) **оба добавляют
`src/rob_box_teleop/setup.cfg` с ИДЕНТИЧНЫМ содержимым** (blob
`66dad82270ea249b801b375028dbf54397457319`). Фикс задвоен; при merge первого
второй получит add/add конфликт или пустой diff.

**Корень:** триаж/e2e-процесс не dedup-ят PR по изменяемым файлам (одинаковый
blob в двух открытых PR не детектится).

**Что сделано (этот PR):** в merge-gate добавлен guard `duplicate_file_scan_all()`
— для open PR с `needs-review`/`needs-e2e` тянет `pulls/N/files` (filename+sha),
находит пару (filename, sha) в РАЗНЫХ PR и постит **инфо-коммент на оба PR**
(dedup 24h), чтобы при ревью Шифу было видно дубль. Guard НЕ блокирует CI и НЕ
снимает `needs-e2e` — решение «какой влить, какой закрыть» остаётся за Шифу.

## Факты (верифицировано 15.08 ~07:20+02:00)

| Поле | PR #1262 | PR #1267 |
|---|---|---|
| Ветка | `z-devops/t_392d6000-teleop-setup-cfg` | `z-{agent}/1266-deploy-issues-on-z-e2e-test-round-114-te` |
| Лейблы | `needs-review` | `e2e-done`, `needs-review` |
| Mergeable | MERGEABLE / CLEAN | MERGEABLE / CLEAN |
| Файлы | только `src/rob_box_teleop/setup.cfg` | `setup.cfg` + `.github/scripts/deployment_issue_dedup.py` + тест |
| setup.cfg blob | `66dad82270ea249b801b375028dbf54397457319` | `66dad82270ea249b801b375028dbf54397457319` |
| setup.cfg patch | `[develop] script_dir=...` + `[install] install_scripts=...` (4 строки) | идентично |

Проверка blob-ов:
```
gh api repos/krikz/rob_box_project/pulls/1262/files --jq '.[]|select(.filename=="src/rob_box_teleop/setup.cfg")|.sha'  # 66dad822...
gh api repos/krikz/rob_box_project/pulls/1267/files --jq '.[]|select(.filename=="src/rob_box_teleop/setup.cfg")|.sha'  # 66dad822...
```

## Root cause

- `rob_box_teleop` переведён на ament_python (791f18b9), но без `setup.cfg`
  console_scripts ставятся в `bin/`, а `ros2 run` ищет в `lib/<pkg>/` →
  crash-loop `No executable found` (issue #1266, деплой round-114).
- Две независимые карточки (t_392d6000 deploy-teleop и t_a14ac65d deploy-issue
  round-114) диагностировали один корень и каждая добавила **один и тот же**
  файл. Триаж при создании новой ветки не проверяет «этот путь уже изменён в
  открытом PR» → PR-дубль не замечен до ревью.

## Guard: duplicate_file_scan_all() в agent-flow-merge-gate.sh

Поведение (ретро 15.08 t_20383d32):

1. `gh pr list --state open --json number,headRefName,labels` — все open PR.
2. Python-блок фильтрует PR с метками `needs-review`/`needs-e2e` (кандидаты на
   ревью/мерж) и тянет `gh api pulls/N/files` (REST, filename+sha).
3. Строит map `(filename, sha) → [PR...]`; если пара (filename, sha) встречается
   в ≥2 РАЗНЫХ PR — это дубль идентичного контента.
4. Коммент на **оба** PR (dedup 24h по подстроке `duplicate file detected`):
   «файл X (blob Y) уже изменён в открытом PR #N с ИДЕНТИЧНЫМ содержимым».
5. НЕ блокирует CI, НЕ снимает needs-e2e, НЕ трогает лейблы.

Вызов добавлен рядом с `stale_branch_scan_all` (основной путь + no-issues путь
сходятся в одну точку).

Rate-limit: 1 REST-вызов `pulls/N/files` на PR-кандидат за тик (~5 мин) —
при ~10 кандидатах это ~120 вызовов/час, в пределах лимита 5000/час.

## Что делать Шифу при ревью (решение по РЕШЕНИЮ карточки)

1. **Влить ОДИН из PR** — рекомендуется **#1267** (шире: включает
   dedup-фиксы `deployment_issue_dedup.py` + тест, уже e2e-done).
2. **Второй (#1262) закрыть как дубль** или rebase на develop после merge
   первого (diff станет пустым — файл уже в develop).

## Тесты

`scripts/agent_flow/tests/test_merge_gate_duplicate_file.sh` (5 сценариев):

- A. Два open PR (needs-review) с ИДЕНТИЧНЫМ blob → коммент на ОБА PR.
- B. Два PR, один файл, РАЗНЫЕ blob → не дубль (нет коммента).
- C. Один PR, уникальный файл → нет коммента.
- D. PR без needs-review/needs-e2e → файлы не тянутся, ложного дубля нет.
- E. Dedup: 2 тика с одним дублем → коммент постится один раз за 24h.

Mock-поддержка: `tests/lib/mock_env.sh` — case `repos/*/pulls/*/files*` →
`PR_<n>_FILES_JSON` (массив `[{"filename","sha"}]`).

Прогон (все 10 merge-gate тестов зелёные):
```
bash scripts/agent_flow/tests/test_merge_gate_duplicate_file.sh  # 5/5
bash scripts/agent_flow/tests/test_merge_gate_stale_branch.sh    # 7/7
bash scripts/agent_flow/tests/test_merge_gate_retro_path.sh      # 12/12
... (остальные merge-gate тесты OK)
```

## Ссылки

- PR #1262 (узкий): https://github.com/krikz/rob_box_project/pull/1262
- PR #1267 (широкий, e2e-done): https://github.com/krikz/rob_box_project/pull/1267
- Issue #1266 (deploy round-114, crash-loop): https://github.com/krikz/rob_box_project/issues/1266
- Карточки: t_392d6000 (deploy-teleop), t_a14ac65d (deploy-issue round-114)
- Предыдущий аналогичный guard: stale-branch re-commit (ретро 12.08 t_d3aeaa9b),
  escape-hatch аддитивных PR (ретро 13.08 t_a3f170fe, t_7d6b4b65)
