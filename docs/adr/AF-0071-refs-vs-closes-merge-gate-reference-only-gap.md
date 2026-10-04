# ADR-AF-0071: closingIssuesReferences пустой у PR с `Refs #N` — расширить merge-gate fallback на reference-only intent

| Поле | Значение |
|---|---|
| Статус | **Proposed** (на принятие Шифу после слияния PR) |
| Дата | 2026-10-04 |
| Автор | architect (Hermes Agent), карточка `t_b9ac8c4e` |
| Контекст | Ночной ревью 2026-10-03 (карточка `t_bf8216cb`) выявил recurring gap: после merge PR в develop `closingIssuesReferences` остаётся пустым, если PR body содержит `Refs #N`, а не `Closes #N`. Issue остаётся OPEN до ручного close Шифу или до watchdog (≤94 секунд). Issue #3000 — второй наблюдаемый кейс (см. ниже); retro-card t_b87fdcf6 от 2026-09-24 говорит «17 из 36 PR за 24.09.2026 пишут Fixes #N только в title PR». |
| Родители | ADR-0014 (контракт close=e2e-done ∧ MERGED), ADR-0018 (raw-evidence обязателен), ADR-AF-0063 (issue-auto-close-after-merge — основной fallback §4.1/4.2), ADR-AF-0066 (`pr-orphan-after-issue-merged-guard`, `pr-redundant-after-umbrella-merge-guard`). |
| Затрагивает | (a) `scripts/agent_flow/agent-flow-merge-gate.sh` — расширение fallback-ветки `Refs`-keyword на intent-close; (b) `scripts/agent_flow/agent-flow-triage.sh` — авто-промпт воркера на конвертацию `Refs` → `Closes` в worktree ветке до push (для следующих PR); (c) `docs/architecture/contributing-issue-pr-linking.md` — канонический гайд; (d) ретро-карточка `t_bf8216cb` — этот ADR. **НЕ затрагивает** `agent-flow-issue-close-fallback.sh` (ADR-AF-0063 §4.2 — второй эшелон, на случай недоступности merge-gate). |
| Связанные | issue #3000 (DJ-persona swap — fix), PR #3280 (MERGED 2026-10-01 в develop, `Refs #3000` в body), PR #3367 (OPEN, MERGEABLE, `Refs #3000`), PR #3371 (OPEN, MERGEABLE, `Refs #3000`), kanban `t_33b7d8f8` (architect done), `t_3026e750` (tester done), retro-card `t_b87fdcf6` (2026-09-24 fingerprint `8123b1b135e2` missing-auto-close), issue #3013 (тот же класс — открыто). |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (на 2026-10-04)

Nightly-review за 2026-10-03 (`t_bf8216cb`, develop HEAD `db40fd5ef`) зафиксировал: issue #3000 остаётся **OPEN уже 9 дней** после merge PR #3280 (commit `96a962c9a`, 2026-10-01T09:03Z) с фиксом `dj_set_boundary.py` + `clear_history(keep=…)` + `<dj_state>` stamp. Причина — PR body использует keyword `Refs #3000` (намеренно, чтобы не закрыть issue до живого e2e), но **никто не конвертировал** это в `Closes #3000` после зелёного ручного прогона архитектора (`t_33b7d8f8` done 2026-10-03 21:04:15).

Это **третий наблюдаемый кейс** того же класса (первые два — `t_b87fdcf6` 2026-09-24, issue #3013).

### 1.2 Сырые доказательства

**Issue #3000 timeline** (`gh api repos/krikz/rob_box_project/issues/3000/timeline?per_page=100`):

```
event=cross-referenced  source=PR #3007 (ADR)        2026-09-24T15:11:55Z  (user=GOODWORKRINKZ)
event=cross-referenced  source=PR #3017 (WIP fix)    2026-09-25T09:54:56Z  (user=GOODWORKRINKZ)
event=referenced        commit=5e70d9fbb              2026-09-27T11:21:39Z  (user=krikz, PR #3007 merge)
event=closed            (auto-sweep, 27.09)          2026-09-27T14:56:12Z  (user=krikz)
event=reopened          (Шифу 01.10)                 2026-10-01T08:11:45Z  (user=GOODWORKRINKZ)
event=cross-referenced  source=PR #3261              2026-10-01T08:11:45Z
event=cross-referenced  source=PR #3280 (Refs #3000)  2026-10-01T08:50:32Z
event=referenced        commit=96a962c9a              2026-10-01T09:03:18Z  (user=krikz, PR #3280 merge)
```

**PR #3280 body** (key fragment):
```
Refs #3000 — реализация ADR-0129 (PR #3007 добавил только ADR).
```

**PR #3280 closingIssuesReferences** (`gh api repos/krikz/rob_box_project/pulls/3280 --jq '.closingIssuesReferences'`):
```
null  # ← GitHub не связывает PR с issue, потому что нет keyword Closes/Fixes/Resolves
```

**PR #3367 body** (e2e-сценарий, architect):
```
Refs #3000 — e2e-сценарий для проверки фикса ADR-0129 (PR #3280).
```

**PR #3371 body** (e2e-сценарий, tester):
```
Refs: #3000, #3367, PR #3280 (ADR-0129)
```

**Issue #3000 events** (полный набор закрытий и переоткрытий): 19 комментариев; последний на 2026-10-04 — ссылка на issue #3375 (текущая kanban-карточка), issue #3377 (e2e-streak), PR #3379 (nightly-report).

**Repo settings** (`gh api repos/krikz/rob_box_project`):
```
allow_squash_merge:                true
squash_merge_commit_message:       COMMIT_MESSAGES  ← body PR теряется в squash-commit
```

### 1.3 Почему текущий merge-gate не закрывает

`scripts/agent_flow/agent-flow-merge-gate.sh` line 4725-4750 (ADR-AF-0063 §4.1 fallback) проверяет PR-body **только** на keyword `Closes/Fixes/Resolves`:

```bash
_kw_pat="(?im)(?:closes|fixes|resolves)\\s+#${number}\\b"
if printf '%s' "$_pr_body" | grep -qE "$_kw_pat"; then
    log "issue #${number}: PR #${pr_number} body has Closes/Fixes/Resolves keyword → fallback auto-close (ADR-AF-0063 §4.1)"
    gh issue close "$number" --repo "$GH_REPO" --reason completed
fi
```

PR #3280 body — `Refs #3000` → keyword не матчится → fallback не срабатывает → issue остаётся OPEN.

### 1.4 Семантическое различие `Refs # vs Closes #`

| Keyword | Семантика воркера | Семантика GitHub | Merge-gate fallback (ADR-AF-0063 §4.1) |
|---|---|---|---|
| `Closes #N` | «закрыть issue по факту merge» | auto-close issue + close commit ref | срабатывает ✅ |
| `Fixes #N` | то же (синоним) | auto-close + close commit ref | срабатывает ✅ |
| `Resolves #N` | то же (синоним) | auto-close + close commit ref | срабатывает ✅ |
| `Refs #N` | «ссылаюсь, но не закрываю сам» | reference, **НЕ** auto-close | **НЕ** срабатывает ❌ |
| `partially addresses #N` | «частично, остальное — follow-up» | reference, не auto-close | НЕ срабатывает ❌ |
| упоминание `#N` (plain) | «для контекста» | reference, не auto-close | НЕ срабатывает ❌ |

Проблема: **воркер не может на 100% доверять** собственному `Refs #N`, потому что между `Refs` и merge проходит 5-20 минут (CI), за которые:

- может прилететь новый fix → `Refs` правильно
- может пройти только этот fix → должно стать `Closes`

## 2. Решение

**Два комплементарных подхода**, синхронно:

### 2.1 Вариант A (основной, merge-gate) — контекстно-зависимый fallback на `Refs`

Расширить fallback в `agent-flow-merge-gate.sh` line 4725-4750: если PR-body содержит **только** `Refs #N` (без `Closes/Fixes/Resolves`) **И** issue выполняет **все три** условия-контекста:

1. Issue **OPEN**, у issue **нет** process-меток (`agent-flow-error`, `stale-candidate`, `needs-input`, `agent-flow-triage` watcher-active). Источник — `gh issue view N --json labels`.
2. С момента merge прошло **> E2EFD** (default 24ч, настраиваемо через env `REFS_FALLBACK_HOURS`). Источник — `gh pr view <pr> --json mergedAt`.
3. Issue упоминается в PR-body как `#N` **только в одном PR** за lookback-окно (default 30 дней). Источник — `gh search prs --query "<N>"`.

→ fallback auto-close issue с маркером `whoami_close_issue` «🔁 refs-fallback auto-close (ADR-AF-0071 §2.1): PR #N MERGED >24ч назад, issue без process-меток, #N упомянут только в этом PR за 30д».

**Почему это безопасно**: PR с `Refs` намеренно отказывается от GitHub-native auto-close (worker хочет сначала прогнать e2e). Но если 24ч прошло и никто не закрыл руками и не подвесил процесс-метку — **либо** фикс признан ок (тогда Шифу ожидал бы закрытие), **либо** worker потерял kanban-карточку (см. `t_b87fdcf6` — это recurring). В обоих случаях лучше auto-close + audit-комментарий, чем 9 дней OPEN.

### 2.2 Вариант B (дополнительный, triage) — авто-промпт воркера перед push

В `scripts/agent_flow/agent-flow-triage.sh` (либо новый `agent-flow-pr-close-keyword-check.sh`, на этапе pre-push в worktree) добавить проверку: если в worktree-ветке issue из kanban-card имеет **только** `Refs #N` в PR-body, и коммиты уже ≥1, и e2e-prognose = PASS или `no-e2e-required` — emit **warning** в kanban-карточку воркера:

> «PR использует `Refs #N` вместо `Closes #N`. Если фикс окончательный, замени `Refs` → `Closes` через `git commit --amend` и `gh pr edit <rev> --body "..."`. Иначе merge-gate fallback ADR-AF-0071 §2.1 закроет issue через 24ч автоматически.»

**Не блок** (воркер может сознательно обойти — например, для архитектурного PR с двумя issue), но **видимая** подсказка с правилом.

### 2.3 Документация

В `docs/architecture/contributing-issue-pr-linking.md` (если файла нет — создать) каноническая таблица: какой keyword когда использовать. Ссылка на этот ADR как источник правила.

## 3. Trade-off

### 3.1 Плюсы

- **Автоматический cleanup** recurring gap (issue #3000, #3013, #3014 — все из nightly-review 2026-10-03).
- **Сохранение семантики воркера**: `Refs` остаётся «не закрывать сам» (worker явно говорит «не доверяю»), но fallback через 24ч добавляет safety net.
- **Audit trail**: `whoami_close_issue` + `🔁 refs-fallback` маркер позволяет post-hoc разобрать, почему issue закрылся.
- **Минимальный diff**: ≈40 строк в `agent-flow-merge-gate.sh` (расширение существующего if), ≈15 строк в `agent-flow-triage.sh`.

### 3.2 Минусы

- **False positive** при сценарии: issue не требует закрытия этим PR (например, PR-рефактор ссылается на issue, но issue живёт отдельно). Защита: правило (3) «issue упомянут только в этом PR за 30д» — резко снижает FP.
- **+24ч latency** между merge и auto-close. Для пользователя это неудобно (issue «висит» сутки после merge), но лучше 9 дней OPEN. Можно тюнить `REFS_FALLBACK_HOURS` env.
- **Ложное срабатывание на 30-дневное окно** при двух PR: один `Refs`, второй `Closes`. Правило (3) смотрит «только в этом PR» → если другой PR `Closes` — этот PR ничего не закроет (уже закрыто), pass-through. Если оба `Refs` и оба MERGED — fallback сработает на первом по времени merge (idempotent для второго, helper `_gm_recent_commented` 2ч окно).

### 3.3 Что если НЕ делать?

- Каждый новый фикс с `Refs #N` будет лежать OPEN до ручного close Шифу (текущий SLA из retro: Шифу закрывает за ≤94 секунды, **но только если он онлайн**).
- Process gap будет повторяться (3+ раза за неделю = recurring по ADR-AF-0049).
- Triage cron будет триггерить на issue #N как orphan (см. `t_b87fdcf6` fingerprint `missing-auto-close`).

## 4. План реализации (фазы)

### Phase 1 — Fallback в merge-gate (1 PR, 40 строк, тестируемо)

**Diff outline** (`scripts/agent_flow/agent-flow-merge-gate.sh`, branch `~line 4725`):

```bash
# === ADR-AF-0071 §2.1: расширяем §4.1 fallback на `Refs #N` с context guards ===
# Существующая §4.1: Closes/Fixes/Resolves keyword → close
# Новая ветка (refs-fallback): только Refs + 3 условия-контекста → close с marker

REFS_FALLBACK_HOURS="${REFS_FALLBACK_HOURS:-24}"
LOOKBACK_DAYS="${REFS_FALLBACK_LOOKBACK_DAYS:-30}"

# После существующего `_kw_pat` block (line ~4750), добавляем:
if [ "$_issue_state" = "OPEN" ] \
   && [ "$_has_e2e_done" = "0" ] \
   && [ "$_has_no_e2e" = "0" ]; then
    _refs_pat="(?im)\\brefs\\s+#${number}\\b"
    if printf '%s' "$_pr_body" | grep -qE "$_refs_pat" \
       && ! printf '%s' "$_pr_body" | grep -qE "(?im)\\b(closes|fixes|resolves)\\s+#${number}\\b"; then
        # Condition 1: no process-labels on issue
        _ilabels="$(gh issue view "$number" --repo "$GH_REPO" --json labels --jq '.labels[].name' 2>/dev/null || echo '')"
        _proc_label_hit="$(printf '%s\n' "$_ilabels" | grep -E '^(agent-flow-error|stale-candidate|needs-input|triage)$' || true)"
        # Condition 2: >24h since merge
        _merged_at="$(gh pr view "$pr_number" --repo "$GH_REPO" --json mergedAt --jq '.mergedAt' 2>/dev/null || echo '')"
        _now="$(date -u +%s)"
        _merged_ts="$(date -u -d "$_merged_at" +%s 2>/dev/null || echo 0)"
        _age_h=$(( (_now - _merged_ts) / 3600 ))
        # Condition 3: #N упомянут только в этом PR за 30 дней (search prs)
        _other_prs="$(gh search prs --repo "$GH_REPO" --query "${number}" --limit 50 --json number --jq '.[] | .number' 2>/dev/null | grep -v "^${pr_number}$" || true)"
        if [ -z "$_proc_label_hit" ] && [ "$_age_h" -ge "$REFS_FALLBACK_HOURS" ] && [ -z "$_other_prs" ]; then
            _gm_recent_commented "issue" "$number" \
                "🔁 refs-fallback auto-close (ADR-AF-0071 §2.1)" 86400 contains || \
            whoami_close_issue "$number" "refs-fallback auto-close (ADR-AF-0071 §2.1): PR #${pr_number} MERGED ${_age_h}h ago, issue без process-меток, #${number} упомянут только в этом PR за 30д"
            if gh issue close "$number" --repo "$GH_REPO" --reason completed >/dev/null 2>&1; then
                log "issue #${number}: CLOSED via refs-fallback (ADR-AF-0071 §2.1)"
                _issue_state="CLOSED"
            fi
        fi
    fi
fi
```

**Тесты** (integration, не unit — это bash):
- Создать временный issue `test: ref-only-PR gap` → создать PR с `Refs #N` → вручную `pr_merge_label_sweep` (mock) → дождаться merge-gate → проверить что через 24ч закрылось.
- Реальный test: добавить mock `REFS_FALLBACK_HOURS=0` в env и проверить на issue #3000 (предварительно откатив e2e-done / no-e2e-required метки если есть).

### Phase 2 — Triage pre-push промпт (1 PR, 15 строк)

В `scripts/agent_flow/agent-flow-triage.sh` (или новом `agent-flow-pr-close-keyword-check.sh`): при создании kanban-карточки через triage, если в worktree issue из тела использует `Refs` и прошло >N коммитов, emit warning в comment.

### Phase 3 — Документация

Создать `docs/architecture/contributing-issue-pr-linking.md` с таблицей keyword → семантика → merge-gate fallback.

## 5. Acceptance criteria

- [ ] Phase 1 diff в `agent-flow-merge-gate.sh` ≤40 строк (line 4725-4760 area)
- [ ] Тест: при `REFS_FALLBACK_HOURS=0` issue #3000 закрывается в течение ≤1 тика merge-gate (5 мин)
- [ ] Тест: при наличии process-метки `agent-flow-error` на issue — fallback **НЕ** срабатывает (защита от false-positive)
- [ ] Тест: при наличии **двух** PR с `Refs #N` за 30 дней — fallback срабатывает только на первом по времени merge (idempotent для второго через `_gm_recent_commented` 24ч окно)
- [ ] Audit marker `whoami_close_issue "🔁 refs-fallback auto-close (ADR-AF-0071 §2.1)"` виден в issue history
- [ ] Документ `docs/architecture/contributing-issue-pr-linking.md` создан и ссылается на этот ADR
- [ ] PR-нота в `agent-flow-triage.sh` видна воркерам (warning, не блок)
- [ ] После merge PR все 11 обязательных CI checks зелёные
- [ ] Ретро-карточка `t_bf8216cb` обновлена: «gap missing-auto-close: process-fix в ADR-AF-0071 + PR #<TBD>»

## 6. Открытые вопросы (на решение Шифу)

1. **`REFS_FALLBACK_HOURS` default**: 24ч (предлагаю) vs 48ч (безопаснее) vs 72ч (Шифу в выходные, автосвип медленнее)?
2. **Condition 3 (`только в этом PR`)**: оставляем как строгий фильтр (предлагаю) vs ослабить до «нет другого `Closes #N` за 30д» (позволит несколько `Refs #N` fallback)?
3. **Phase 1 vs Phase 2+3**: одиночный PR на весь ADR или три отдельных PR? Прагматика: один (всё логически связано), но PR-размер риск >200 строк → можно разделить.

## 7. Альтернативы (отклонённые)

### 7.1 Запретить `Refs #N` совсем

Заменить на: `Closes #N` для одного PR, `Refs #N` не использовать. **Минус**: ломает семантику воркера «не закрывать пока e2e не пройдёт». Отклонено.

### 7.2 Шифу закрывает руками каждый раз

Текущий SLA: ≤94 секунды (см. retro 2026-09-06 `t_bfd19ffb`). **Минус**: не масштабируется, требует онлайна. Уже recurring 3+ раза. Отклонено.

### 7.3 Cron «чистильщик» как ADR-AF-0063 §4.2 (variant B)

Существующий §4.2 fallback-cron. **Минус**: ещё один тик, ещё один потенциальный single-point-of-failure, ADR-AF-0063 §4.2 уже отвергнут Шифу 2026-09-08 (принят только §4.1). Phase 1 этого ADR **расширяет** §4.1, а не вводит новый cron — **это и есть синтез §4.1 и §4.2 подходов**, которого не хватало.

## 8. Rollout plan

1. **2026-10-04** — этот ADR отправлен Шифу на review (через PR в develop с docs/adr/AF-0071-*.md).
2. **2026-10-05** — после review Шифу, открыть PR для Phase 1 (`agent-flow-merge-gate.sh`).
3. **2026-10-06** — Phase 1 merge. На следующий день мониторим `gh issue list --label=agent-flow-error` — нет ли regression (новые false-positive).
4. **2026-10-07** — Phase 2 + Phase 3 (triage промпт + docs).
5. **2026-10-08** — retro через неделю: сравнить `gh api .../issues?labels=agent-flow-error&since=2026-10-04` vs baseline до ADR (recurring gap должен упасть до 0).

## 9. References (raw)

- `gh api repos/krikz/rob_box_project/issues/3000` → state=open, 19 comments
- `gh api repos/krikz/rob_box_project/issues/3000/timeline?per_page=100` → 13 events (см. §1.2)
- `gh api repos/krikz/rob_box_project/pulls/3280 --jq '.closingIssuesReferences'` → null
- `gh api repos/krikz/rob_box_project/pulls/3367 --jq '.closingIssuesReferences'` → null
- `gh api repos/krikz/rob_box_project/pulls/3371 --jq '.closingIssuesReferences'` → null
- `git log origin/develop --all --grep=#3000` → 5 коммитов: 5e70d9fbb (ADR), 96a962c9a (PR #3280), 9d23d680c (release), 0a9847fcd (PR #3367), 94f793196 (PR #3371)
- `docs/adr/AF-0063-issue-auto-close-after-merge.md` §4.1, §4.2
- `scripts/agent_flow/agent-flow-merge-gate.sh` line 4725-4760 (existing §4.1 fallback)
- Retro: `docs/reports/nightly-review/2026-09-24.jsonl` fingerprint `8123b1b135e2` (missing-auto-close, same class)
- Kanban: `t_33b7d8f8` (architect done 2026-10-03 21:04:15), `t_3026e750` (tester done 2026-10-03 21:19:21), `t_b87fdcf6` (2026-09-24 retro), `t_bf8216cb` (2026-10-03 nightly-review, parent)