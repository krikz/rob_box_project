# ADR-0035: merge-gate auto-detect "stale-after-upstream-fix" для diagnostic-карточек

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge PR в develop → Accepted) |
| Дата | 2026-08-31 |
| Автор | architect (Hermes Agent); карточка `t_8fdc62ee`, родительская ретро-карточка `t_9d375e3e` |
| Контекст | merge-gate (PR #1743, retro `t_e00f448d`) научился создавать **diagnostic-карточки** для CI UNSTABLE с classification `unit_lint` (= реальная регрессия в коде PR, а не rebase-need). Но эти карточки **не умеют auto-detect**, когда upstream-фикс уже в `develop` или включён в тот же PR → backend-воркер открывает карточку, читает «зелёные» failed-tests, не видит работы, сидит `max_runtime` впустую. Реальный кейс 31.08 03:25Z: t_8f764875 (PR #1740) и t_5c524b12 (PR #1741) — обе stale после upstream-фикса 06b83b01b (PR #1748, merged 00:45:28Z), Шифу закрыл PR #1740 в 00:56:57Z, но карточки висят в `todo`. |
| Затрагивает | `scripts/agent_flow/agent-flow-merge-gate.sh` (новый блок `stale_after_upstream_fix_scan_all` рядом со `stale_branch_scan_all` / `stale_conflicting_scan_all`; маркеры в body diagnostic-карточек в UNSTABLE-блоке ~line 2516 и в scan-all-prs UNSTABLE-блоке ~line 3188), `scripts/agent_flow/tests/test_merge_gate_stale_after_upstream_fix.sh` (новый), `CONTRIBUTING.md` (упоминание про маркеры `<!-- diag-* -->` в diagnostic-body для grep'абельности). |
| Родители | ADR-0018 (честность — auto-block лучше молчаливого waste'а), ADR-0014 (agent-flow issue closure contract), ADR-0026 (recovery contract — worker отвечает за следствие), `t_e00f448d` (PR #1743 — diagnostic-классификация), `t_9d375e3e` (ретро, родитель этой карточки). |
| Связанные | PR #1743 (UNSTABLE-классификатор, merged), PR #1748 (upstream-фикс `06b83b01b`, merged 31.08 00:45:28Z), PR #1740 / #1741 (stale PR'ы), issues #1730 / #1736, карточки `t_8f764875` / `t_5c524b12` (stale diagnostic). |

## TL;DR

Diagnostic-карточки (создаваемые merge-gate при `mergeStateStatus=UNSTABLE + classification=unit_lint`) сегодня «вечно живые»: после того как upstream-фикс уже в `develop` или включён в тот же PR, они продолжают висеть в `todo`/`ready`, пока backend-воркер не сядет читать и не закроет руками. Это (а) тратит worker-time на чтение «уже-зелёных» тестов, (б) шумит в Kanban UI Шифу, (в) риск false-FIX (воркер пытается «починить» то, что уже починено, и ломает фикс).

Решение — добавить в merge-gate **новый блок `stale_after_upstream_fix_scan_all`**, который перед каждым тиком (или раз в N минут) сканирует все live diagnostic-карточки, **парсит из body маркеры `<!-- diag-* -->`** (PR-номер, PR head SHA, имена importer-атрибутов, failing-tests файлы), и при наличии upstream-фикса в `develop` или в самом PR — вызывает `hermes kanban block --kind transient` с reason=`stale-after-upstream-fix: <sha>` и дописывает в body ссылку на upstream-коммит + объяснение «почему фикс больше не нужен». Существующая логика классификации (UNSTABLE → diagnostic, CONFLICTING → rebase, dead-content guard, lint-only routing) — **не трогаем**. Юнит-тест на 6 сценариев (по образцу `test_merge_gate_drift_pr_done.sh`).

**Не делаем:** централизованный upstream-фикс store (overkill, git log уже single source of truth), ML-based similarity detector (overkill, тело diagnostic-карточки и так парсится regex), изменение логики создания diagnostic-карточек (retro `t_e00f448d` уже сделала это правильно в PR #1743).

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем

31.08 03:25Z, retro `t_9d375e3e`:

```
30.08 22:18Z  merge-gate → t_8f764875 (PR #1740), t_5c524b12 (PR #1741) — diagnostic
30.08 23:55Z  t_5e06c47d (developer, ретро upstream-regression) → __new__ фикстуры → PR #1748
31.08 00:45:28Z  PR #1748 merged в develop (06b83b01b) — upstream-фикс уже в
31.08 00:56:57Z  PR #1740 CLOSED (Шифу закрыл, т.к. upstream починил)
31.08 03:25Z  t_8f764875 + t_5c524b12 всё ещё в `todo`
```

Backend-воркер возьмёт одну из них, откроет failed-tests в CI (там «уже-зелёные»), не увидит работы, потратит `max_runtime` (1800s = 30 мин) на «доказательство, что фикс уже есть», и либо сделает no-op close, либо (хуже) начнёт «поверхностный фикс», ломающий upstream.

### 1.2 Почему это блокер (а не косметика)

1. **Worker-time waste.** Каждая stale diagnostic-карточка = 30 мин CPU + 5+ GB tmp-disk + 1 PR или abort-цикл. При текущем rate'е (4-6 diagnostic-карточек в день после PR #1743) — 2-3 часа worker-time в день впустую.
2. **False-FIX risk.** Воркер, читающий «зелёные тесты» + рекомендацию «почини тестовые фикстуры в `test_barge_in_policy.py`», может не понять что upstream уже починил, и начать cherry-pick'ить или менять `__init__` поверх уже-починенного → rebase conflict / CI regression / новый цикл UNSTABLE.
3. **Signal-noise в Kanban UI.** Шифу видит 4-6 «почти одинаковых» diagnostic-карточек в `ready`/`todo`, тратит 5-10 мин на их разбор при triage. Психологический долг (ретро 18.08 уже фиксировал это для других категорий — `auto_decompose`).
4. **Скрытый root cause.** PR #1743 (retro `t_e00f448d`) сделал **создание** diagnostic-карточек правильным — классификация `unit_lint` vs `e2e` vs `lint` уже работает. Но **lifecycle** diagnostic-карточки не закрыт: создал → воркер взял → … → Шифу руками блокирует. Нет feedback-loop «PR закрылся / upstream починил → diagnostic stale».

### 1.3 Гипотеза (root cause)

Три независимых дыры в lifecycle diagnostic-карточки:

1. **Нет upstream-фикс детектора.** merge-gate знает про PR (`pr_number`, `pr_head_ref`, `pr_sha`), знает про check-runs (через REST API), но не имеет git-доступа к `origin/develop` для проверки «был ли фикс этого PR или upstream уже в develop». Даже когда `REPO_DIR` доступен (как в post-merge-build на line 1811), merge-gate не использует `git log -S` / `git rev-list` для diagnostic-карточек.

2. **Нет сигнатуры в body diagnostic-карточки.** PR #1743 создаёт diagnostic-карточки с подробным body (failed-tests список, AttributeError имена, «Что делать (НЕ rebase)» чек-лист) — **но не имеет машино-читаемых маркеров**. Greppать «`_track_mode_music_active`» по всему телу работает только если знаешь этот термин; новый diagnostic пишется с другими именами атрибутов / тестов.

3. **Нет feedback от PR lifecycle на diagnostic-карточку.** Существующая логика merge-gate работает с PR-метками (`needs-e2e` → set, `e2e-done` → reconcile) — но diagnostic-карточка **привязана к PR, а не к PR-метке**. Когда PR закрывается (ретро `t_16325ddd` это уже уважает для UNSTABLE-блока — skip if CLOSED), или когда PR мерджится (фикс уже в develop), или когда в самом PR появляется коммит с фикс-сигнатурой — diagnostic-карточка не получает сигнала.

### 1.4 Бизнес-последствие (если НЕ фиксить)

- Каждый major CI UNSTABLE инцидент (типа PR #1740 / #1741 → upstream-fix через PR #1748) → 2-6 stale diagnostic-карточек на 30+ мин каждая = 1-3 часа worker-time в день.
- Рост false-FIX incidents (воркер «поверхностно правит» уже-починенное) → новые rebase-циклы → новые diagnostic-карточки → рост backlog.
- Шифу продолжает блокировать stale-карточки руками → bottleneck → медленнее фиксы PR-блокеров → медленнее e2e-rotation.

## 2. Принятое решение

### 2.1 Маркеры в body diagnostic-карточки (мини-DSL)

При создании diagnostic-карточки (UNSTABLE-блок в основном цикле ~line 2516, и UNSTABLE-блок в scan-all-prs ~line 3188) добавляем **в конец `body`** HTML-комментарии — невидимые при рендере, grep'абельные, стабильные (PR #1743 retro `t_e00f448d` уже парсит такие маркеры для классификации, см. `<!-- kind: ... -->` если есть):

```markdown
...существующий текст diagnostic-карточки (failed jobs, failed tests, контекст, что делать)...

<!-- diag-pr: 1740 -->
<!-- diag-pr-sha: f924ad6c47bcf7deb66d2080dcee067c66cf5792 -->
<!-- diag-pr-base: develop -->
<!-- diag-sig: _track_mode_music_active, _code_speech_retry_used -->
<!-- diag-tests: src/rob_box_voice/test/unit/node/test_barge_in_policy.py, \
              src/rob_box_voice/test/unit/node/test_issue_1195_tg_source.py, \
              src/rob_box_voice/test/unit/node/test_issue_1343_empty_speak_text.py, \
              src/rob_box_voice/test/unit/node/test_service_text_leak_via_speaker_tag.py -->
<!-- diag-classification: unit_lint -->
<!-- diag-created-ts: 1756598400 -->
```

Где:
- `diag-pr` — PR номер (из `pr_number`).
- `diag-pr-sha` — head SHA PR на момент создания карточки (`gh pr view N --json commits --jq '.commits[-1].oid'`).
- `diag-pr-base` — base ref (обычно `develop`, но возможны исключения).
- `diag-sig` — список importer-символов / атрибутов, упомянутых в body (regex-экстракция из `AttributeError'ы` секции).
- `diag-tests` — список failing-test-файлов из `### Failed tests` секции.
- `diag-classification` — `unit_lint` / `e2e` / `lint` (по PR #1743).
- `diag-created-ts` — `date +%s` на момент создания карточки (для rate-limit и freshness).

**Legacy diagnostic-карточки** (созданные до PR этой ADR — `t_8f764875`, `t_5c524b12`) **не имеют** маркеров → новый блок `stale_after_upstream_fix_scan_all` их **пропускает** (см. 2.5 graceful-degradation). Backward-compatible: маркеры опциональны, без них блок не падает.

### 2.2 Новый блок `stale_after_upstream_fix_scan_all`

Расположение: в `agent-flow-merge-gate.sh` рядом со `stale_branch_scan_all` (line 3068), `stale_conflicting_scan_all` (line 3073), `duplicate_file_scan_all` (line 3076), `pr_without_marker_scan_all` (line 3079) — т.е. в **scan-all-prs секции**, вызывается **перед основным циклом** (как все остальные scan'ы).

Сигнатура вызова (по аналогии с существующими scan-функциями):

```bash
# Ретро 31.08 t_9d375e3e, ADR-0035: stale-diagnostic-after-upstream-fix detector.
stale_after_upstream_fix_scan_all
```

Псевдокод (полная реализация — в `tests/test_merge_gate_stale_after_upstream_fix.sh` и PR):

```bash
stale_after_upstream_fix_scan_all() {
    local diag_cards_json _card_id _card_status _body _pr_num _pr_sha _pr_base \
          _sig_list _tests_list _created_ts _marker _reason
    # 1. Получить все live diagnostic-карточки (todo/ready/running).
    diag_cards_json="$(hermes kanban --board "$KANBAN_BOARD" list --json 2>/dev/null | python3 -c '
import json, sys
try:
    data = json.loads(sys.stdin.read())
except Exception:
    sys.exit(0)
for t in data:
    title = t.get("title", "")
    body = t.get("body", "") or ""
    status = t.get("status", "")
    # Сигнатура diagnostic-карточки из PR #1743 (ретро t_e00f448d):
    # title начинается с "🐛 CI UNSTABLE:" или "🔀 rebase PR #..."
    is_diag = (title.startswith("🐛 CI UNSTABLE:") or
               title.startswith("🔀 rebase PR #"))
    has_marker = "<!-- diag-pr:" in body
    if is_diag and has_marker and status in ("todo", "ready", "running", "blocked"):
        print(t["id"], status)
')"

    [ -z "$diag_cards_json" ] && { log "stale-after-upstream-fix: no live diagnostic cards"; return 0; }

    while IFS=' ' read -r _card_id _card_status; do
        [ -z "$_card_id" ] && continue

        # 2. Достать body карточки (нужны маркеры).
        _body="$(hermes kanban --board "$KANBAN_BOARD" show "$_card_id" --json 2>/dev/null | python3 -c '
import json, sys
try:
    data = json.loads(sys.stdin.read())
    print(data.get("body", "") or "")
except Exception:
    pass
')"

        # 3. Парсинг маркеров (grep + sed).
        _pr_num="$(printf '%s' "$_body" | grep -oE '<!-- diag-pr: [0-9]+ -->' | head -1 | grep -oE '[0-9]+')"
        _pr_sha="$(printf '%s' "$_body" | grep -oE '<!-- diag-pr-sha: [a-f0-9]+ -->' | head -1 | grep -oE '[a-f0-9]+')"
        _pr_base="$(printf '%s' "$_body" | grep -oE '<!-- diag-pr-base: [^ ]+ -->' | head -1 | sed 's/<!-- diag-pr-base: //;s/ -->//')"
        _sig_list="$(printf '%s' "$_body" | grep -oE '<!-- diag-sig: [^>]+-->' | head -1 | sed 's/<!-- diag-sig: //;s/ -->//' | tr ',' '\n')"
        _tests_list="$(printf '%s' "$_body" | grep -oE '<!-- diag-tests: [^>]+-->' | head -1 | sed 's/<!-- diag-tests: //;s/ -->//' | tr ',' '\n')"
        _created_ts="$(printf '%s' "$_body" | grep -oE '<!-- diag-created-ts: [0-9]+ -->' | head -1 | grep -oE '[0-9]+')"

        [ -z "$_pr_num" ] && { log "stale-after-upstream-fix: ${_card_id} — no diag-pr marker, skip (legacy)"; continue; }
        [ -z "$_pr_base" ] && _pr_base="$DEVELOP_BRANCH"

        # 4. Rate-limit: не блокировать одну и ту же карточку чаще 1 раза в 2ч.
        _marker="stale-after-upstream-fix:${_pr_num}"
        _last_ts="$(kanban_last_reminder_ts "$_card_id" "$_marker")"
        _now_ts="$(date +%s)"
        if [ -n "$_last_ts" ] && [ $(( _now_ts - _last_ts )) -lt 7200 ]; then
            log "stale-after-upstream-fix: ${_card_id} — rate-limited (last=${_last_ts})"
            continue
        fi

        # 5. PR уже CLOSED? (быстрый skip — retro t_16325ddd)
        if [ "$(pr_state_now "$_pr_num")" = "CLOSED" ]; then
            log "stale-after-upstream-fix: ${_card_id} — PR #${_pr_num} CLOSED, blocking as transient"
            _reason="stale-after-upstream-fix: PR #${_pr_num} CLOSED (починка upstream или неактуален, ретро t_9d375e3e / ADR-0035)"
            _commit_sha=""
        else
            # 6. Стратегия A: PR head SHA — ancestor of origin/develop (PR слит).
            if [ -n "$_pr_sha" ] && [ -n "${REPO_DIR:-}" ] && [ -d "$REPO_DIR" ]; then
                if git -C "$REPO_DIR" merge-base --is-ancestor "$_pr_sha" "origin/${_pr_base}" 2>/dev/null; then
                    _reason="stale-after-upstream-fix: PR #${_pr_num} head ${_pr_sha:0:8} уже в origin/${_pr_base} (ретро t_9d375e3e / ADR-0035)"
                    _commit_sha="$_pr_sha"
                fi
            fi

            # 7. Стратегия B (если A не сработал): upstream-фикс по сигнатуре / failing-tests.
            if [ -z "${_reason:-}" ] && [ -n "${REPO_DIR:-}" ] && [ -d "$REPO_DIR" ]; then
                # 7a. По атрибутам (diag-sig): git log -S <attr> origin/develop.
                _upstream_sha=""
                if [ -n "$_sig_list" ]; then
                    while IFS= read -r attr; do
                        [ -z "$attr" ] && continue
                        # Ищем коммит в origin/develop, который добавил/инициализировал атрибут,
                        # после создания карточки.
                        _hit="$(git -C "$REPO_DIR" log "origin/${_pr_base}" \
                            --since="@${_created_ts:-0}" -S "$attr" \
                            --pretty=format:'%H' 2>/dev/null | head -1)"
                        if [ -n "$_hit" ]; then
                            _upstream_sha="$_hit"
                            break
                        fi
                    done <<< "$_sig_list"
                fi

                # 7b. По failing-tests (diag-tests): git log origin/develop -- <file>.
                if [ -z "$_upstream_sha" ] && [ -n "$_tests_list" ]; then
                    while IFS= read -r test_file; do
                        [ -z "$test_file" ] && continue
                        _hit="$(git -C "$REPO_DIR" log "origin/${_pr_base}" \
                            --since="@${_created_ts:-0}" -- "$test_file" \
                            --pretty=format:'%H' 2>/dev/null | head -1)"
                        if [ -n "$_hit" ]; then
                            _upstream_sha="$_hit"
                            break
                        fi
                    done <<< "$_tests_list"
                fi

                if [ -n "$_upstream_sha" ]; then
                    _reason="stale-after-upstream-fix: upstream-фикс ${_upstream_sha:0:8} уже в origin/${_pr_base} (после создания карточки, ретро t_9d375e3e / ADR-0035)"
                    _commit_sha="$_upstream_sha"
                fi
            fi

            # 8. Стратегия C (если A+B не сработали): фикс в самом PR (PR не слит, но файлы из diag-tests уже изменены + CI зелёный).
            if [ -z "${_reason:-}" ] && [ -n "$_tests_list" ]; then
                _pr_files="$(gh pr view "$_pr_num" --repo "$GH_REPO" --json files --jq '[.files[].path]' 2>/dev/null || echo '[]')"
                _pr_checks_ok="$(gh pr checks "$_pr_num" --repo "$GH_REPO" --json state --jq '[.[] | select(.state != "SUCCESS")] | length' 2>/dev/null || echo 999)"
                # Если все failing-test файлы уже в текущем PR-diff И CI SUCCESS → фикс внутри PR, не в develop.
                if [ "$_pr_checks_ok" = "0" ]; then
                    _all_in_pr=1
                    while IFS= read -r test_file; do
                        [ -z "$test_file" ] && continue
                        if ! printf '%s' "$_pr_files" | grep -qF "$test_file"; then
                            _all_in_pr=0
                            break
                        fi
                    done <<< "$_tests_list"
                    if [ "$_all_in_pr" = "1" ]; then
                        _reason="stale-after-upstream-fix: фикс уже в самом PR #${_pr_num} (failing-tests файлы в PR-diff + CI SUCCESS, ждать merge в develop, ретро t_9d375e3e / ADR-0035)"
                        _commit_sha=""
                    fi
                fi
            fi

            # 9. Fallback через REST API (если REPO_DIR недоступен — например, при cron-tick с workdir=host).
            if [ -z "${_reason:-}" ] && [ -n "$_pr_sha" ]; then
                # REST compare: ahead_by == 0 и mergeStateStatus == MERGEABLE+CLEAN → вероятно слит.
                _compare="$(gh api "repos/${GH_REPO}/compare/${_pr_base}...${_pr_sha}" --jq '{a:.ahead_by,b:.behind_by,s:.status}' 2>/dev/null || echo '{}')"
                _a="$(printf '%s' "$_compare" | grep -oE '"a":[0-9-]+' | grep -oE '[0-9-]+' | head -1)"
                _b="$(printf '%s' "$_compare" | grep -oE '"b":[0-9-]+' | grep -oE '[0-9-]+' | head -1)"
                _s="$(printf '%s' "$_compare" | grep -oE '"s":"[^"]+"' | cut -d'"' -f4)"
                if [ "${_a:-999}" = "0" ] && [ "${_b:-999}" = "0" ] && [ "$_s" = "identical" ]; then
                    _reason="stale-after-upstream-fix: PR #${_pr_num} уже в origin/${_pr_base} (REST compare identical, ретро t_9d375e3e / ADR-0035)"
                    _commit_sha="$_pr_sha"
                fi
            fi
        fi

        # 10. Если ни одна стратегия не сработала — карточка ещё не stale, skip.
        if [ -z "${_reason:-}" ]; then
            log "stale-after-upstream-fix: ${_card_id} (PR #${_pr_num}) — upstream-фикс пока не найден, skip"
            continue
        fi

        # 11. DRY_RUN: только лог.
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: block ${_card_id} with reason: ${_reason}"
            continue
        fi

        # 12. Auto-block + body patch.
        hermes kanban --board "$KANBAN_BOARD" block --kind transient \
            "$_card_id" "$_reason" >/dev/null 2>&1 \
            && log "stale-after-upstream-fix: ${_card_id} auto-blocked (PR #${_pr_num}, sha=${_commit_sha:-none})" \
            || log "stale-after-upstream-fix: WARNING block ${_card_id} failed"

        # 13. Patch body: добавить секцию "Upstream-фикс (auto-detected)".
        if [ -n "$_commit_sha" ]; then
            _gh_url="https://github.com/${GH_REPO}/commit/${_commit_sha}"
            _patch=$(printf '\n\n### ✅ Upstream-фикс уже в develop (auto-detected, merge-gate ADR-0035, %s)\n\n**Причина блокировки:** %s\n\n**Upstream-коммит:** [%s](%s)\n\n**Что делать:** карточка может быть закрыта как `done` (stale-diagnostic-after-upstream-fix). Воркеру не нужно ничего чинить — регрессия upstream-починена, тесты на develop уже зелёные.\n' "$(date -u +%H:%M:%SZ)" "$_reason" "${_commit_sha:0:8}" "$_gh_url")
            hermes kanban --board "$KANBAN_BOARD" comment "$_card_id" "$_patch" >/dev/null 2>&1 \
                || log "stale-after-upstream-fix: WARNING body patch comment failed for ${_card_id}"
        fi
    done <<< "$diag_cards_json"

    log "stale-after-upstream-fix: scan complete"
}
```

### 2.3 Вызов блока (как остальные scan'ы)

В `agent-flow-merge-gate.sh` рядом с существующими scan-вызовами (line 3068-3082) добавляем:

```bash
stale_branch_scan_all
stale_conflicting_scan_all
duplicate_file_scan_all
pr_without_marker_scan_all
deploy_issue_reconcile_all
# Ретро 31.08 t_9d375e3e / ADR-0035: stale-diagnostic-after-upstream-fix detector.
# Безопасно вызывать в начале секции: не зависит от issues_json, сканирует
# только kanban board. Дешёвая проверка (≤10 live diagnostic-карточек типично).
stale_after_upstream_fix_scan_all
```

Также добавляем вызов в **no-issues path** (ранний exit перед основным циклом) — там уже сходятся `stale_branch_scan_all` / `duplicate_file_scan_all` и т.д. (ретро `t_d3aeaa9b`, `t_20383d32`).

### 2.4 Что НЕ трогаем

| Компонент | Почему |
|---|---|
| Логика создания diagnostic-карточки (UNSTABLE-блок ~line 2516, scan-all-prs UNSTABLE-блок ~line 3188) | PR #1743 (retro `t_e00f448d`) сделал правильно. Меняем ТОЛЬКО добавление маркеров `<!-- diag-* -->` в конец `body` (sed-append). Никакой change-detection logic. |
| CONFLICTING-блок (~line 2472, scan-all-prs ~line 3344) | CONFLICTING = rebase-need, не regression. upstream-фикс не делает merge conflict. Логика другая — `stale_conflicting_scan_all` уже существует (retro `t_cd32788f`). |
| Dead-content guard (line 2675+) | Отдельная забота (retro `t_e8d52cb7` / `t_944df2c5`), другой failure mode. |
| Clean-PR sweep (line 3421+) | CLEAN/MERGEABLE PR — успешный путь, не наш случай. |
| Stale-branch re-commit scan (`stale_branch_scan_all`) | Retro `t_d3aeaa9b` — другая проблема (z-{agent}/... ветки с устаревшим PR). |
| Throttle / idempotency guards | ADR-0030, ADR-0032 — другая семантика. |
| Rate-limit 2ч | Унаследован от UNSTABLE-reminder rate-limit (line 2517). |

### 2.5 Graceful degradation (legacy diagnostic без маркеров)

Карточки, созданные **до** внедрения маркеров (`t_8f764875`, `t_5c524b12` и любые другие diagnostic-карточки в текущем board), **не имеют** маркеров `<!-- diag-* -->`. Для них:

1. **Skip по умолчанию** (см. строку `[ -z "$_pr_num" ] && continue` в 2.2): новый блок их не трогает — они остаются в `todo`/`ready`/`blocked`, как сейчас.
2. **Одноразовый backfill** (опционально, в том же PR): script `scripts/agent_flow/backfill_diag_markers.sh` проходит по всем live diagnostic-карточкам, пытается regex-извлечь PR-номер (из title — `PR #NNNN`), SHA (через `gh pr view N --json commits --jq '.commits[-1].oid'`), failing-tests (из `### Failed tests` секции), sig (из `AttributeError'ы` секции), и **перезаписывает body** с добавленными маркерами в конец. DRY-RUN по умолчанию. После backfill — следующий тик `stale_after_upstream_fix_scan_all` их подхватит и заблокирует.

### 2.6 Конфигурируемость

Env-vars (по аналогии с существующими):

```bash
# ADR-0035: stale-after-upstream-fix detector
STALE_AFTER_UPSTREAM_FIX_SCAN="${STALE_AFTER_UPSTREAM_FIX_SCAN:-true}"
STALE_AFTER_UPSTREAM_FIX_COOLDOWN_SECONDS="${STALE_AFTER_UPSTREAM_FIX_COOLDOWN_SECONDS:-7200}"  # 2ч
STALE_AFTER_UPSTREAM_FIX_REPO_DIR="${STALE_AFTER_UPSTREAM_FIX_REPO_DIR:-$REPO_DIR}"  # fallback на REPO_DIR
```

`STALE_AFTER_UPSTREAM_FIX_SCAN=false` → блок не вызывается (для incident-response, если upstream-фикс детектор начнёт false-positive'ить — отключаем без правки кода).

## 3. Альтернативы (рассмотренные и отклонённые)

### 3.1 Альтернатива 1: PR-close watcher

Идея: когда PR закрывается, найти все diagnostic-карточки, ссылающиеся на этот PR, и auto-block.

**Почему нет:** решает только 1 из 3 стратегий (стратегия A). Случай `t_5c524b12` (PR #1741 ещё OPEN, но фикс уже в PR в виде коммита `afc94e2b`) — НЕ ловится. Случай upstream-фикса в develop без PR close — тоже не ловится. Решение 2.2 покрывает все три.

### 3.2 Альтернатива 2: check-runs success watcher

Идея: для каждой live diagnostic-карточки периодически опрашивать `gh pr checks <pr_num>` — если SUCCESS, блокировать.

**Почему нет:** check-runs success ≠ upstream-фикс. PR может быть SUCCESS но с тем же кодом что и при создании diagnostic (например, rebase ничем не помог). Нужна проверка **содержимого** — а это и есть git log по сигнатуре.

### 3.3 Альтернатива 3: централизованный fingerprint store (Redis / SQLite)

Идея: каждый upstream-фикс оставляет fingerprint в общем store'е; diagnostic-карточка проверяет «есть ли мой fingerprint в store».

**Почему нет:** overkill. `git log origin/develop -- <file>` и `git log -S <attr>` уже single source of truth — git history и есть тот самый store. Добавлять ещё один слой — лишний moving part, лишний failure mode.

### 3.4 Альтернатива 4: ML-based content similarity

Идея: сравнивать текст body diagnostic-карточки с сообщениями merge-коммитов в develop.

**Почему нет:** overkill и unreliable. Тело diagnostic-карточки структурировано (PR-номер, имена атрибутов, failing-tests файлы) — regex достаточен. ML в cron-tick'е = latency + cost.

### 3.5 Альтернатива 5: блокировать diagnostic-карточку после merge PR в develop (стратегия A only)

Идея: минимальное изменение — только проверка `git merge-base --is-ancestor <pr_sha> origin/develop`. Если PR слит → block.

**Почему нет:** пропускает случай `t_5c524b12` (PR #1741 ещё OPEN, но фикс уже в PR). Стратегия C (фикс в самом PR) обязательна, иначе после мержа PR #1741 в develop карточка t_5c524b12 останется stale ещё на 1 тик.

## 4. Последствия

### 4.1 Положительные

1. **Zero-waste diagnostic lifecycle.** Каждая diagnostic-карточка, для которой upstream-фикс уже в develop или в PR, автоматически блокируется в течение 1 тика merge-gate (≤5 мин) после фикса. Backend-воркер не тратит 30 мин на чтение «уже-зелёных» тестов.
2. **Защита от false-FIX.** Воркер не сможет случайно «починить поверх» уже-починенного — карточка уже `blocked (transient)`, воркер сразу видит «upstream-фикс уже в develop, см. upstream-commit».
3. **Прозрачность для Шифу.** В Kanban UI stale-карточки сразу маркируются как `blocked` с kind=`transient` + reason содержит sha upstream-коммита → Шифу может быстро аудитить «что auto-detect заблокировал, корректно ли».
4. **PR-close case решается автоматически.** PR #1740 (CLOSED 00:56:57Z после upstream-фикса) — карточка t_8f764875 auto-blocks на следующем тике merge-gate. Никаких ручных блокировок Шифу.

### 4.2 Отрицательные / риски

1. **False-positive: блокировка карточки при ложном upstream-фикс детекте.**
   - **Сценарий:** разработчик починил `_track_mode_music_active` в PR #X (НЕ upstream, НЕ в develop, а в feature-ветке, которую потом отменили). `git log origin/develop -S "_track_mode_music_active"` показывает коммит X. diagnostic-карточка блокируется.
   - **Митигация:** rate-limit 2ч + в reason указан конкретный upstream-sha + в body patch — ссылка на коммит. Если false-positive — Шифу или воркер снимает блокировку (`hermes kanban unblock <id> --reason "false-positive: коммит X отменён"`). Без auto-close (transient, не permanent).
   - **Дополнительная митигация:** `--since="@${_created_ts:-0}"` фильтрует коммиты ПОСЛЕ создания карточки — не ловит upstream-фиксы, которые были до создания (они уже учтены при создании).
2. **Git-history не доступен в cron-workdir=host.**
   - **Сценарий:** merge-gate cron запускается в `/home/builder` (не в repo workdir) — `REPO_DIR` пуст → git-команды не работают.
   - **Митигация:** fallback через REST compare API (стратегия A только, без стратегий B+C). Если REPO_DIR пуст — логируем "stale-after-upstream-fix: REPO_DIR empty, only REST fallback active" и продолжаем.
3. **Diagnostic-карточка создана с неполными маркерами (например, только diag-pr, без diag-tests).**
   - **Сценарий:** PR был с classification `e2e` (не `unit_lint`), failing-tests секция пустая (e2e logs в другом месте) → `diag-tests` пустой.
   - **Митигация:** если `diag-sig` и `diag-tests` оба пустые, fallback к стратегии A (PR-merge через `git merge-base`). Если и A не сработал — skip с логом "no upstream-fix signal available, skipping".
4. **Цена каждого тика растёт на ~5-10 git-команд.**
   - **Сценарий:** 10 live diagnostic-карточек × 2 стратегии × 3 атрибута = 60 git-команд. На тик 5 мин — это ≤2 сек total.
   - **Митигация:** git log на remote ref'е — дешёвая операция (ref-кеш), реальная стоимость — сетевой I/O к GitHub для fetch ref'ов. Если cost растёт — можно throttle на 1 раз в 10 мин (отдельный env `STALE_AFTER_UPSTREAM_FIX_THROTTLE_MINUTES=10`).

### 4.3 Что НЕ делаем (negative scope)

- Не внедряем новые метки / не меняем существующие PR-метки.
- Не закрываем карточки автоматически (только `block --kind transient`).
- Не модифицируем diagnostic-карточки в `done` / `archived` (они терминальные — retro `t_42741511`).
- Не трогаем UNSTABLE-блок основного цикла (только добавляем маркеры в конец `body`).
- Не внедряем новые зависимости (только `git`, `gh`, `hermes kanban` — уже в наличии).

## 5. План реализации

### 5.1 Backend-воркеру (developer / backend)

| # | Что | Файл | Строк | Тест |
|---|---|---|---|---|
| 1 | Helper-функция `stale_after_upstream_fix_scan_all` (по псевдокоду из 2.2) | `scripts/agent_flow/agent-flow-merge-gate.sh` (новый блок после `pr_without_marker_scan_all`, ~line 3080) | +200 / -0 | unit-тест D1-D6 |
| 2 | Env-vars в начале файла | `scripts/agent_flow/agent-flow-merge-gate.sh` (после `STALE_REBASE_*` ~line 82) | +5 / -0 | n/a |
| 3 | Вызов `stale_after_upstream_fix_scan_all` в основном пути (рядом с другими scan'ами) | `scripts/agent_flow/agent-flow-merge-gate.sh` (line 3068+) | +2 / -0 | n/a |
| 4 | Вызов в no-issues path | `scripts/agent_flow/agent-flow-merge-gate.sh` (где другие scan'ы в early-exit пути) | +2 / -0 | n/a |
| 5 | Маркеры `<!-- diag-* -->` в конец body UNSTABLE-блока (основной цикл) | `scripts/agent_flow/agent-flow-merge-gate.sh` (~line 2662 — `hermes kanban create --body "$_reminder"`) | +10 / -0 | unit-тест M1-M2 |
| 6 | Маркеры в body UNSTABLE-блока scan-all-prs | `scripts/agent_flow/agent-flow-merge-gate.sh` (~line 3324 — `hermes kanban create --body "$_reminder"`) | +10 / -0 | unit-тест M1-M2 |
| 7 | Backfill script (опционально, но рекомендую) | `scripts/agent_flow/backfill_diag_markers.sh` (новый файл) | +80 / -0 | unit-тест B1-B3 |
| 8 | Юнит-тест | `scripts/agent_flow/tests/test_merge_gate_stale_after_upstream_fix.sh` (новый) | +300 / -0 | self |

### 5.2 Тест-сценарии (для `test_merge_gate_stale_after_upstream_fix.sh`)

По образцу `test_merge_gate_drift_pr_done.sh` (D1-D6):

| # | Сценарий | Что проверяет |
|---|---|---|
| **D1** | Diagnostic-карточка + upstream-fix в develop (стратегия B) | Карточка auto-blocks, body содержит `### ✅ Upstream-фикс`, reason содержит sha upstream-коммита |
| **D2** | Diagnostic-карточка + PR head SHA — ancestor of origin/develop (стратегия A) | Карточка auto-blocks, reason содержит PR SHA |
| **D3** | Diagnostic-карточка + PR CLOSED | Карточка auto-blocks с reason "PR CLOSED", sha пустой |
| **D4** | Diagnostic-карточка + фикс в самом PR + CI SUCCESS (стратегия C) | Карточка auto-blocks с reason "фикс в самом PR", sha пустой |
| **D5** | Diagnostic-карточка без маркеров (legacy) | Skip с логом "no diag-pr marker, skip (legacy)", НЕ блокируется |
| **D6** | DRY_RUN=true | Без real `hermes kanban block`, только лог `DRY-RUN would: block ...` |
| **D7** | Rate-limit: 2й тик подряд, тот же PR | На 2й тик — skip с логом "rate-limited" |
| **D8** | Diagnostic в `done` / `archived` | Skip (терминальные статусы, retro `t_42741511`) |
| **D9** | REPO_DIR пуст (fallback REST compare) | Стратегия A работает через `gh api compare`, B+C — skip с логом |
| **D10** | diag-sig + diag-tests оба пустые | Только стратегия A (PR-merge), без B+C |
| **M1** | Маркеры корректно записываются в body при создании diagnostic-карточки (UNSTABLE-блок основного цикла) | body содержит все `<!-- diag-* -->` маркеры |
| **M2** | Маркеры корректно записываются в body scan-all-prs diagnostic | То же для scan-all-prs пути |
| **B1** | Backfill script: legacy diagnostic без маркеров → маркеры добавляются | body содержит маркеры после backfill |
| **B2** | Backfill script: diagnostic с маркерами → НЕ дублирует (idempotent) | Количество маркеров остаётся тем же |
| **B3** | Backfill script: DRY_RUN=true | Файл body не изменяется |

### 5.3 Метрики успеха

После merge PR в develop (через 7 дней):

1. **Stale diagnostic-карточки в `todo`/`ready`:** ≤1 (исключения: легитимные, где upstream ещё не починил).
2. **Backend-воркер no-op closes (stale-diagnostic):** ≤5% от всех closes (остальные — реальная работа).
3. **False-positive rate:** ≤2% (измеряется как «карточка unblocked воркером после auto-block» / «все auto-blocks»). Цель — 0%, бюджет 2% на старт.
4. **Время от upstream-merge до auto-block:** ≤10 мин (≤2 тика merge-gate).

### 5.4 Rollout

1. **Этап 1 (PR с маркерами):** добавить маркеры в body diagnostic-карточек (п.5.1 #5, #6). Тест M1-M2. Маркеры безвредны без detector'а — старый merge-gate их игнорирует (HTML-комментарии в markdown — невидимы).
2. **Этап 2 (PR с detector'ом):** добавить `stale_after_upstream_fix_scan_all` (п.5.1 #1, #2, #3, #4). Тест D1-D10.
3. **Этап 3 (PR с backfill'ом):** опционально — добавить `backfill_diag_markers.sh` (п.5.1 #7). Применить вручную к `t_8f764875` и `t_5c524b12` для разовой очистки (Шифу решит). Тест B1-B3.

Каждый этап — отдельный PR (для review). Если этап 2 покажет false-positive rate > 2% за первые 24 часа — `STALE_AFTER_UPSTREAM_FIX_SCAN=false` откатывает без правки кода.

### 5.5 Связанные ретро и follow-up

- **Эта ретро-карточка:** `t_8fdc62ee` (architect → plan/ADR).
- **Родительская ретро:** `t_9d375e3e` (зафиксировала паттерн stale diagnostic, создала child-карточку).
- **Базовый PR:** #1743 (ретро `t_e00f448d`, классификация diagnostic vs rebase). Без него маркеры не имели бы смысла.
- **Upstream-фикс:** PR #1748 (commit `06b83b01b`, merged 31.08 00:45:28Z) — триггер этого ADR.
- **Следующий шаг после merge этого PR:** Шифу вручную вызывает `bash scripts/agent_flow/backfill_diag_markers.sh --apply` для `t_8f764875` и `t_5c524b12` — следующий тик merge-gate их auto-blocks.

## Приложение А: Полная цепочка событий (для контекста ADR)

```
30.08 22:18Z  merge-gate tick: PR #1740 (unit_lint UNSTABLE) → t_8f764875 создан
              merge-gate tick: PR #1741 (unit_lint UNSTABLE) → t_5c524b12 создан
30.08 23:55Z  t_5e06c47d (developer, upstream-regression): __new__ фикстуры → PR #1748
31.08 00:45:28Z  PR #1748 merged в develop (06b83b01b) — upstream-фикс уже в
31.08 00:56:57Z  PR #1740 CLOSED (Шифу: upstream починил)
31.08 02:50:42Z  PR #1741 → CI run #33337022448 (после рекомендаций t_5c524b12):
              developer push afc94e2b wip(voice t_eade262b): getattr-guard
              → CI SUCCESS, MERGEABLE+CLEAN, label needs-e2e
31.08 03:25Z  t_9d375e3e (backend, retro): обе diagnostic-карточки stale после
              upstream-фикса → создана t_8fdc62ee (этот ADR)
31.08 03:27Z  комментарии в t_8f764875 / t_5c524b12 от backend с raw-доказательствами
              + рекомендация "закрыть как done (stale-diagnostic-after-upstream-fix)"

С этим ADR:
- t_8fdc62ee → ADR-0035 merge → backfill_diag_markers → следующий тик merge-gate:
  * t_8f764875: PR #1740 CLOSED → auto-block (стратегия D: PR CLOSED)
  * t_5c524b12: PR #1741 → upstream-фикс afc94e2b в PR-diff + CI SUCCESS →
    auto-block (стратегия C: фикс в самом PR)
- Backend-воркеры больше не тратят runtime на эти 2 карточки.
- После мержа PR #1741 в develop — карточка остаётся blocked (transient),
  Шифу закрывает её вручную (или по batch-sweep).
```

## Приложение Б: Сравнение с существующими решениями

| Аспект | Текущее поведение | После ADR-0035 |
|---|---|---|
| Diagnostic-карточка после upstream-фикса в develop | Висеть в `todo`/`ready`, пока воркер / Шифу не закроет руками | Auto-block (transient) ≤10 мин после upstream-merge |
| Diagnostic-карточка после фикса в самом PR | Висеть, воркер открывает failed-tests (уже зелёные) | Auto-block с reason "фикс в самом PR" |
| Diagnostic-карточка после PR CLOSED | Висеть (retro `t_16325ddd` только skip нового создания) | Auto-block с reason "PR CLOSED" |
| Diagnostic-карточка без upstream-фикса (легитимная) | Воркер чинит → close | Без изменений (стратегии A/B/C не срабатывают — skip) |
| Worker-time на stale diagnostic | 30 мин × N карточек = часы в день | ≤2 сек на tick (git log — дешёвая операция) |

---

ADR-0035 готов к review. После merge → status Accepted, env `STALE_AFTER_UPSTREAM_FIX_SCAN=true` по умолчанию. Rollout — 3 этапами (маркеры → detector → backfill), каждый — отдельный PR с тестами.
