# AF-0066 — Deploy-issue "premise-obsolete" verification procedure

**Дата:** 2026-09-15
**Автор:** devops worker (ретро-карточка t_0fe85763, issue #2256)
**Контекст:** kanban-карточки assignee=devops от авто-созданных issue с label `deployment` (L-Deploy and Verify.yml → "🚨 Create Deployment Issue")
**Статус:** proposed
**Связанные тикеты:** t_0fe85763 (issue #2256), t_16325ddd (premise-obsolete class), t_cce8616a (т\u0435\u043a\u0443\u0449\u0438\u0435 devops-карточки на deploy-fail)
**Связанные коммиты:** 6b3ef64a1 (race fix), 78bbbe16d (SOURCE_HASH rob_box_core)

## Контекст

`L-Deploy and Verify.yml` создаёт issue с label `deployment` каждый раз, когда
deploy-roll на develop/main завершается со статусом `issues` (не success).
Hermes triage ловит эту issue и создаёт kanban-карточку assignee=devops.

Проблема (наблюдалось на t_0fe85763 / issue #2256): в CI deploy-фейлы
**не всегда означают баг в текущем develop** — часто фикс уже влит ранее,
а issue висит OPEN пока воркер не разберётся. На каждый фикс race-condition
в CI может висеть 1-2 stale deploy-issue, которые triage повторно триажит
(потому что issue с label `deployment` всегда рождает карточку).

**Реальный кейс:** issue #2256 от 2026-09-09 11:08:50 UTC
(`deploy-fail:develop:staging:2026-09-09`): avatar-supervisor
restart-looped с `ModuleNotFoundError: No module named 'rob_box_core.utterance'`.

Корень бага был починен в 6b3ef64a1 (16:10 UTC того же же дня) —
`needs: [prepare, build-voice-assistant]` на build-supervisor job
предотвращает гонку build-supervisor ↔ build-voice-assistant.
На kanban-доску карточка попала только 2026-09-15 02:55 (triage re-scan),
то есть через **~6 дней** после фикса.

## Acceptance

Воркер assignee=devops на карточку с label `deployment` ОБЯЗАН:

1. **Сверить issue's `Workflow Run` с текущим develop HEAD:**
   - `gh run view <run_id>` → если не из текущего HEAD, фикс может быть уже в develop.
   - `git log --grep "<bug keyword>" origin/develop — нашёл фикс-коммит?"
2. **Если фикс в develop, проверить последние 5 develop-deploy'ов:**
   - `gh run list --workflow="L: Deploy and Verify" --limit 20 — json | jq`
   - все SUCCESS → премис obsolete, завершаем без PR.
3. **Сверить статус avatar-supervisor на последнем SUCCESS deploy:**
   - `gh run view <latest_dev_run> --log | grep avatar-supervisor:`
   - `status=running restarting=false health=healthy restarts=0` — подтверждение.
4. **Только при наличии регрессии в свежих run'ах — открывать PR с фиксом.**

## Process

- **Перед открытием PR** — проверить, что фикс уже не в develop. Это
  типичный случай для race-fix / cache-fix / dedup-fix — коммит маленький,
  быстро вливается, а deploy-issue продолжает висеть OPEN до следующего
  re-fail (или re-триажа).
- **Закрытие issue** — комментарий с raw-evidence (run_id + SHA fix-коммита
  + статус avatar-supervisor из последнего deploy). Без raw-evidence —
  фикс не доказан (ADR-0018).

## Anti-pattern (НЕ делать)

- ❌ Сразу открывать PR с «улучшением», не проверив, что фикс уже в develop.
- ❌ Коммитить WIP, если премис obsolete — лишний шум в PR-cleanup.
- ❌ Закрывать issue без raw-evidence.
- ❌ Слать PR в `develop` от старого deploy-issue, если fix уже merged
  ранее — это создаёт дубликат фикса.

## Related

- ADR-0014 §8 (conservative guard) — аналогичный anti-double-create guard
  для merge-gate.
- ADR-AF-0065 — scan-all-prs race-guard для merge-gate.
- `agent-flow-triage.sh::fp_for_pr_file` — дедуп фиксов в PR-волне.

## Acceptance для этой ADR

- Воркер, получивший deploy-issue карточку, применяет procedure выше
  и в `kanban_complete.metadata.findings` указывает один из вариантов:
  - `premise_obsolete: <fix_sha> уже в develop, последний develop-deploy <run_id> SUCCESS, avatar-supervisor healthy`
  - `regression: текущий develop-deploy <run_id> падает с тем же симптомом, открываю PR`