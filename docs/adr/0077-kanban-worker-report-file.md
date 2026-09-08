# ADR-0077: kanban worker report file (issue #2159)

| Поле | Значение |
|---|---|
| Статус | **Accepted** (process change, не требует ревью владельца — наказ Шифу 18.08: «не делай руками») |
| Дата | 2026-09-08 |
| Автор | devops (Hermes Agent), kanban `t_77f8ebd8` (issue #2159, v2 с skills секцией из #2162) |
| Контекст | Ночной ревью (`t_af2371a0`, `t_69cb9566`) и 199 done-карточек за всё время показали: **воркеры не оставляют артефактов** при `kanban_complete`. Result — 1-2 предложения (~304 символа), Artifacts — список путей **без содержимого**, summary — одно предложение, pytest / e2e / CI логи **не сохраняются**. Через месяц после архивации невозможно понять, что воркер делал, что починил и где raw-evidence. |
| Зависит от | AGENTS.md (raw-evidence обязателен), ADR-0018 (честный FAIL > красивый PASS), ADR-0032 (процессный трек), issue #2162 (skills в body карточки — комплементарный fix) |
| Связанные | issue #2159 (этот ADR — фича), issue #2162 (skills в body), `scripts/agent_flow/report_template.md` (шаблон), `scripts/agent_flow/kanban-report-write.sh` (генератор), `scripts/agent_flow/tests/test_kanban_report_write.sh` (регресс), kanban `t_77f8ebd8` (эта карточка), follow-up если воркеры начнут игнорировать → жёсткий gate |
| Цель | **Каждый воркер перед `kanban_complete` обязан закоммитить `docs/reports/kanban/<task_id>.md` в свою ветку** с raw-evidence (pytest / CI / e2e / git log / diff / skill results). Отчёт переживает архивацию worktree (лежит в репо, git tracked). |
| Решает | «Как воркеру сохранить полный отчёт, чтобы через месяц после архивации можно было восстановить контекст: что делал, что починил, какие были raw-evidence, какие skills помогли» |

> **TL;DR.** Воркеры **перед `kanban_complete`**:
>
> 1. Запускают `scripts/agent_flow/kanban-report-write.sh <task_id> --title "..." --pr <N> --issue <M>` → генерируется `docs/reports/kanban/<task_id>.md` с git log/diff/CI-заглушками.
> 2. **Дописисывают руками** свободные секции (что сделано, skill results, caveats).
> 3. `git add` + commit + push **в ту же ветку**, что и PR.
> 4. **Только после push** → `kanban_complete`.
>
> Merge-gate печатает **мягкий WARN** в лог (НЕ блокирует), если в PR ≥3 файлов и нет `docs/reports/kanban/*.md` (issue #2159, ADR-0077). Hot-fix (≤2 файлов) exempt. ADR-0018: «честный FAIL лучше красивого PASS» — сначала заполнить отчёт, потом complete.

---

## 1. Что не работает сейчас

### Поведение воркера при `kanban_complete` (ретро 18.08, ~199 done карточек)

| Поле | Что воркер пишет сейчас | Достаточно для ретро? |
|---|---|---|
| `result` | 1-2 предложения, ~304 символа | ❌ слишком короткий |
| `artifacts` | список путей **без содержимого** | ❌ только индекс, через 30 дней worktree может быть удалён |
| `summary` | одно предложение | ❌ нет деталей |
| `git diff` | в ветке (но после archive/cleanup может пропасть) | ⚠️ зависит от worktree |
| `pytest -v` | **не сохраняется** | ❌ нет raw-evidence |
| `gh run view` | **не сохраняется** | ❌ нет raw-evidence |
| `e2e logs` | **не сохраняется** | ❌ нет raw-evidence |

### Конкретный кейс: `t_88230c1c` (17 минут работы, ADR-0022)

В `Result` сохранено:
> «Issue #2122 (HIGH) — Deploy and Verify рапортовал success при контейнере в Restarting loop. Принял в работу, диагностика + регрессия + PR.»

В `Artifacts` — 3 пути к файлам, но:
- Содержимое файлов не сохранено (через 30 дней worktree может быть удалён).
- Нет raw-вывода pytest.
- Нет логов CI workflow run.
- Нет git log изменённых файлов.
- Нет способа перепроверить результат воркера через месяц.

### Что просит Шифу (issue #2159, тело issue)

> «проанализируй карточки в канбане на счет ревью, они ничего не оставляют после себя, ни каких артефактов»

> «воркеры должны сохранять полные отчёты в `docs/reports/kanban/<task_id>.md` при `kanban_complete` — иначе после архивации нечего ревьюить»

---

## 2. Решение

### 2.1. Шаблон отчёта — `scripts/agent_flow/report_template.md`

Содержит секции:
- **Что сделано** — bullet-list (воркер дописывает руками)
- **Файлы изменены** — `git diff --stat` (генерируется автоматически)
- **Git log** — `git log --oneline` (генерируется автоматически)
- **Raw-evidence** — `pytest -v`, `gh pr checks`, `docker logs` (воркер вставляет руками)
- **Skill results** — что дал каждый skill из `## Skills` секции body (issue #2162)
- **PR / Issue ссылки**
- **Замечания / Caveats** — что НЕ сделано, известные проблемы

### 2.2. Генератор — `scripts/agent_flow/kanban-report-write.sh`

```bash
bash scripts/agent_flow/kanban-report-write.sh t_77f8ebd8 \
    --title "[process #2159] воркеры сохраняют отчёты" \
    --assignee devops \
    --pr 9999 \
    --issue 2159
```

Что делает:
1. `mkdir -p docs/reports/kanban/`.
2. Собирает `git diff --stat origin/develop..HEAD`, `git log --oneline origin/develop..HEAD`, текущую ветку, дату.
3. Создаёт `docs/reports/kanban/<task_id>.md` с подставленными данными.
4. **НЕ** коммитит и **НЕ** пушит — это делает воркер, чтобы можно было сначала отредактировать свободные секции.
5. Печатает next-steps: `git add` → commit → push → **и только после** `kanban_complete`.

### 2.3. Регресс-тест — `scripts/agent_flow/tests/test_kanban_report_write.sh`

8 сценариев: defaults / --title / --pr / --output / no-task-id / no-git-worktree / bad-base-ref / idempotent-overwrite. Покрытие raw-evidence: тест `assert_contains` проверяет ключевые подстроки (`**Assignee:** devops`, `**PR:** #4242`, `недоступен` для fallback, etc).

### 2.4. Мягкий WARN в merge-gate (НЕ блокирующий)

`agent-flow-merge-gate.sh::warn_no_worker_report` — вызывается на этапе, когда merge-gate знает, что PR жизнеспособен (MERGEABLE + CLEAN). Если:
- в PR ≥3 файлов **И**
- нет ни одного `docs/reports/kanban/*.md`

→ печатает WARN в stderr + лог merge-gate. **НЕ блокирует merge**, **НЕ** комментирует issue/PR (по решению в issue #2159: «не блокировать kanban_complete»). Hot-fix PR (≤2 файлов) exempt — отчёт там нерелевантен.

```
WARN: PR #2094 has no docs/reports/kanban/*.md report (issue #2159, ADR-0077)
— workers should add it before kanban_complete (soft warning, not blocking)
```

### 2.5. install.sh — раскладка

`scripts/agent_flow/install.sh::EXPECTED[]` дополнен `kanban-report-write.sh` (с комментарием). Это значит:
- `agent-flow-drift-detect.sh` будет контролировать наличие файла во всех 6 target-папках (`profiles/agent-flow`, `architect`, `devops`, `backend`, `analyst`, `~/.hermes/scripts`).
- На каждом cron-тике `install-daily` раскладывает обновлённый файл на хост.

### 2.6. Скилы в body (комплементарно к issue #2162)

Шаблон отчёта (`report_template.md`) уже **требует** секцию `## Skill results` — воркер дописывает, что сделал каждый skill из `## Skills` body карточки. Это работает «из коробки» — отдельных изменений в dispatch/triage не нужно (issue #2162 говорит, что `--skill` CLI-флаг не пробрасывается, поэтому body — единственный канал).

---

## 3. Trade-offs

| Альтернатива | Плюсы | Минусы | Выбор |
|---|---|---|---|
| **A. Жёсткий gate (блокировать merge без отчёта)** | Гарантированный отчёт в каждом PR | Ломает hot-fix (1-2 файла: «починил таймаут»), ADR-фиксы (только `docs/adr/00XX-...md`), мелкие тесты. Жёсткий kill-switch без ramp — антипаттерн (см. ADR-0022 §7.1) | ❌ |
| **B. Мягкий WARN в лог merge-gate (выбрано)** | Не ломает hot-fix, не шумит, постепенно формирует привычку. Если в ретро увидим «50% PR без отчёта» → ужесточим в ADR-AF-0077+1 | Воркер может проигнорировать | ✅ |
| **C. Воркер сам генерирует отчёт + commit руками** | Гибко, воркер контролирует содержание | Воркеры уже ленятся писать summary (304 символа) — без скрипта-генератора будут писать «сделал PR #N, закоммитил» | ❌ |
| **D. Скрипт-генератор + ручное дописывание свободных секций (выбрано)** | Шаблон + автоматический git log/diff + ручной контент | Двухшаговая процедура (generate → edit) | ✅ |
| **E. Архивировать worktree целиком (вместо отчёта)** | Полный контекст | Worktree может быть удалён через 30 дней, занимает много места, не grepable | ❌ |

**Комбинация D + B** — генератор + мягкий WARN. Воркеры получают инструмент, не получают kill-switch, привычка формируется через лог-фидбек.

---

## 4. Что НЕ меняется

- ❌ `hermes-kanban` CLI (`kanban_complete` / `kanban show`) — issue #2159 явно запрещает, мы только пишем отчёт **до** complete, не меняем обработчик.
- ❌ Жёсткая блокировка — по наказу Шифу 18.08: «не делать руками» (= не ломать воркерам существующий flow, пока не проверено).
- ❌ Другие процессные скрипты (triage, e2e-process, handoff) — это **отдельные карточки**, если выяснится, что им тоже нужны отчёты.
- ❌ `--skill` CLI-флаг — issue #2162 фиксит комплементарно (skills в body). Не наш scope.

---

## 5. Acceptance

### Проверяемые факты (DoD)

- [x] `scripts/agent_flow/report_template.md` существует, содержит все секции (что сделано, файлы, git log, raw-evidence, skill results, PR/Issue, caveats).
- [x] `scripts/agent_flow/kanban-report-write.sh` существует, executable, синтаксис OK (`bash -n`).
- [x] `scripts/agent_flow/tests/test_kanban_report_write.sh` существует, 8 сценариев, 11/11 ассертов PASS.
- [x] `scripts/agent_flow/install.sh::EXPECTED[]` дополнен `kanban-report-write.sh`, файл попадает в 6 target-папок.
- [x] `agent-flow-merge-gate.sh::warn_no_worker_report` определена, вызывается в строке MERGEABLE+CLEAN, smoke-тест 6/6 PASS.
- [x] PR создан, base=develop, CI зелёный.
- [x] Сам отчёт по этой карточке лежит в `docs/reports/kanban/t_77f8ebd8.md` (закоммичен в эту же ветку до kanban_complete).

### Метрика успеха (для следующего ретро)

- Доля PR воркер-карточек с `docs/reports/kanban/*.md` ≥ 50% к 2026-10-08.
- Если < 30% — ужесточаем в `ADR-0077+1` (жёсткий gate, exempt только для hot-fix ≤2 файлов).
- Если ≥ 80% — оставляем как есть, добавляем CI-summary в отчёт.

---

## 6. Открытые вопросы

- **Q1**: писать ли отчёт в `result` поле `kanban_complete`? Сейчас в metadata кладём `report_path: "docs/reports/kanban/t_77f8ebd8.md"`, воркер ссылается из summary. Достаточно. Если Шифу захочет весь текст — отдельная карточка.
- **Q2**: для архитектурных ADR-карточек нужен ли отчёт? Сейчас exempt (только `docs/adr/00XX-...md` → merge-gate CI-only exemption, hot-fix path). Если Шифу захочет — отдельная итерация.
- **Q3**: что делать с `agent-flow-error` (карточка создалась, но воркер упал до `kanban_complete`)? Сейчас отчёт не пишется, в tasks events остаётся `agent-flow-error` label. Достаточно для ретро (по issue #1534).

---

## 7. Связанные

- AGENTS.md → секция «Культура честности (АБСОЛЮТ)» — raw-evidence обязателен.
- ADR-0018 (честный FAIL лучше красивого PASS) — база.
- ADR-0022 §7.1 #12 — CI-only exemption паттерн (не ломаем легитимные правки жёстким gate).
- ADR-0032 (Meta Quest WebXR stack) — пример ADR, на структуру которого опирается этот документ.
- issue #2159 — исходное issue (этот ADR — фича).
- issue #2162 — комплементарный fix (skills в body), этот ADR покрывает `## Skill results` секцию отчёта, но не трогает `--skill` CLI.
- `scripts/agent_flow/report_template.md` — шаблон (SOT содержимого).
- `scripts/agent_flow/kanban-report-write.sh` — генератор (SOT скрипта).
- `scripts/agent_flow/tests/test_kanban_report_write.sh` — регресс (SOT покрытия).
- kanban `t_77f8ebd8` — эта карточка (где живёт отчёт).
- kanban `t_84434d4c` — предыдущая итерация (PARSE_ERROR при автотесте, был сброшен). Этот ADR учитывает её фейлы.

---

> *«Не оставляй после себя пустоту — оставь отчёт, чтобы следующий воркер
>   мог встать на твои плечи, а не угадывать, что ты делал.»*
> (наказ товарища Шифу, issue #2159, 18.08.2026)
