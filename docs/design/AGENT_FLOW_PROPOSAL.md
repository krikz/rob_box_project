# Agent Flow Proposal — Issue → branch+PR → e2e round → manual merge → close

| Поле | Значение |
|------|----------|
| Документ | `docs/design/AGENT_FLOW_PROPOSAL.md` |
| Статус | **v2, decisions applied (Q1..Q24; Q20..Q24 закрыты 08.08 по итогам live-теста #1050/#1051)** |
| Дата | 2026-08-06 (v1) / 2026-08-08 (v2) |
| Автор | PM / юзер (krikz) → оформил: Hermes |
| Issue | **#1038** |
| Связанное | `docs/design/E2E_TESTING_DESIGN_v2.md`, `docs/design/CHILD_TASKS_PROPOSAL.md`, `.github/workflows/L-E2E Voice Test.yml`, канбан `t_*`, issue #1050/#1051 (live-тест) |

---

## 0. TL;DR

Сквозной поток для агент-воркера, **от ручного создания Issue до ручного merge юзером**, с **автоматизацией** в середине:

```
human → Issue[hermes+agent:role]
   ↓ cron `agent-flow-triage` (every 5m)
kanban-карточка (assignee, инструкция, branch-naming)
   ↓ sub-agent по карточке
worktree → рабочая ветка `z-{agent}/<id>-<slug>` → push + **комменты в issue о ходе** (Q23) → `kanban complete` → карточка в блок (ждёт CI)
   ↓ cron `agent-flow-merge-gate` (every 5m)
CI красный → коммент в карточку «⛔ красный» + `kanban unblock` → воркер чинит в ТОЙ ЖЕ карточке (Q21)
CI зелёный → merge-gate ничего не делает (карточка в блоке, ждёт e2e)
   ↓ cron `e2e-process` (rolling-round, every 1h)
resolver берёт задачу → мержит в `z-{e2e}/wip-<id>-<slug>` → PR в `z-{e2e}/test-round-N` → CI → merge в test-round-N → e2e прогон
   ↓ артефакты
issue-коммент: verdict + run + log + audio + ASR + diff + acceptance + timing + RMS + baseline
   ↓ человек (юзер)
ручной merge `z-{agent}/<id>-<slug>` → `feature/harness-p0-foundation`
   ↓ cleanup (ТОЛЬКО после merge — R6)
удалить `z-{agent}/<id>-<slug>` и `z-{e2e}/wip-<id>-<slug>`, `kanban complete`, issue close
```

**Ручное:** Issue creation (юзер) и merge approval (юзер). Остальное — автомат.

**Ключевое правило статусов (Q21/Q22):** карточка **НЕ создаёт дочерних карточек** на починку. Красный CI → `block` → воркер чинит в той же → зелёный → `unblock`. После e2e → `block` «ожидает merge». `done` — **ТОЛЬКО после ручного merge юзером**.

---

## 1. Что УЖЕ работает (08.08, точки опоры)

| Компонент | Где | Статус |
|-----------|-----|--------|
| Kanban Hermes | `~/.hermes/kanban/boards/robbox/` | ✅ карточки `t_*` |
| Sub-agents | `hermes` (19 профилей с контрактом в SOUL) | ✅ |
| triage cron | `agent-flow-triage.sh` (every 5m) | ✅ live-тест #1048/#1049 |
| merge-gate cron | `agent-flow-merge-gate.sh` (every 5m) | ✅ live-тест #1050 → PR #1051; Q21 block/unblock — в работе |
| e2e-process | `agent-flow-e2e-process.sh` (every 1h) | ⏳ Q22 (block до e2e, done после merge) — в работе |
| G: Run Tests | `.github/workflows/G-Run Tests.yml` | ✅ зелёный на develop (run 31251726974, 2m9s) |
| G: Auto-merge to Main | `.github/workflows/G-Auto-merge to Main.yml` | 🔧 чинится (workflow_call + inputs.*) |
| GH access | `gh` (GOODWORKRINKZ) | ✅ |
| Репо | `krikz/rob_box_project`, работа на **develop** | ✅ |

---

## 2. Маркеры

### Issue-side

| Маркер | Что значит |
|--------|-----------|
| `hermes` | триаж cron берёт |
| `agent:backend \| agent:architect \| agent:devops \| agent:reviewer` | роль исполнителя в канбан-карточке |
| default роли (если нет `agent:*`) | **`hermes`** (Гермес сам себе делает) |
| `priority:P0 \| P1 \| P2` | канбан-приоритет |
| `e2e:now` | срочно — пропустить волну (Phase 4) |

### PR-side

| Маркер | Что значит |
|--------|-----------|
| База ветки `z-{e2e}/test-round-N` | PR из `z-{e2e}/wip-<id>-<slug>` в test-round-N |
| `needs-e2e` | auto от merge-gate когда CI зелёные |
| `e2e-done` | после успешного e2e (с артефактами) |
| `e2e:rejected` | юзер отказал merge'у |

### Именование веток (Q20, обязательное)

Тильда `~` (ASCII 0x7E) в **начале** имени → служебные ветки сортируются **внизу** списка (после `z`), рабочие — сверху.

| Тип | Формат | Пример |
|-----|--------|--------|
| Агентские (воркер) | `z-{agent}/<id>-<slug>` | `z-{agent}/1050-bug-voice-assistant` |
| Служебные e2e | `z-{e2e}/test-round-N`, `z-{e2e}/wip-<id>-<slug>` | `z-{e2e}/wip-1050-audio-buffer` |

**Правила:**
- **ВСЕ ветки автоматизированного процесса начинаются с `z-{`** (ASCII 0x7E) — в начале имени, чтобы сортировались внизу списка: `z-{agent}/...`, `z-{e2e}/...`.
- Тильда строго в начале имени (`z-{agent}/...`, не `agent/~...`).
- `z-{hotfix}/`, `z-{revert}/` — юзер создаёт САМ, вручную, вне процесса.
- Без тильды — только не-процессные ветки (feature/... и т.п.).
- `branch_for()` в triage.sh, префиксы в e2e-process.sh, поиск PR в merge-gate.sh — ВСЕ создают ветки с `~`.

---

## 3. Контракты между частями

### 3.1 `agent-flow-triage` (cron, every 5 min)

**Inputs:** `gh issue list --label hermes --state open --json ...` (с прошлого тика).

**Outputs:** kanban-карточка с шаблоном:
```
title: <issue.title>
body:
  ## Source
  • repo: krikz/rob_box_project
  • issue: #<N>
  • url: <link>
  • labels: hermes, agent:<role>, ...
  ## Agent instruction (короткое, ≤3 строк)
  <TL;DR инструкция что делать>
  ## Branch + PR convention
  • branch: z-{agent}/<id>-<short-slug>  (процессная, с ~)
  • merge: PR `z-{e2e}/wip-<id>-<slug>` → `z-{e2e}/test-round-N` (служебные, с ~)
  ## Done criteria
  • CI all green
  • PR merged в test-round-N
  • e2e SUCCESS
  • Артефакты приложены в issue
  • Manual merge юзером
  ## Context (создаётся автоматически, triage.sh)
  repo, local_clone, base_branch, sources_of_truth, access
assignee: <agent:<role> или hermes>
priority: P<P0..P2>
```

**Идемпотентность:** при создании карточки пишет **коммент в issue** с маркером `kanban: t_<task_id>`. На следующем тике cron смотрит на наличие такого комментария — пропускает.

**Именование ветки (Q20):** `branch_for()` в triage.sh — ВСЕ процессные с `~`: `z-{agent}/<id>-<slug>`, `z-{e2e}/`, `z-{revert}/`, `z-{hotfix}/` (`z-{e2e}/`, `z-{revert}/`, `z-{hotfix}/`) — с тильдой в начале.

### 3.2 Agent-исполнитель (sub-agent)

**Inputs:** канбан-карточка `t_<id>`.

**Действия (пошагово):**

0. **Стартовый коммент в issue** (Q23): «взял в работу, ветка `z-{agent}/<id>-<slug>`, worktree <путь>».
1. Получает карточку через kanban-tool.
2. Создаёт worktree (свой собственный, не `_work`!) от `feature/harness-p0-foundation`:
   ```
   git worktree add -b z-{agent}/<id>-<slug> /home/builder/wt-<id> origin/feature/harness-p0-foundation
   ```
3. Делает код по инструкции из карточки. Если нужно — читает родительский issue.
4. **Commit style:** `feat/fix/docs(scope): ...`.
5. Push в `origin/z-{agent}/<id>-<slug>`. Открывает PR (base = develop). **Коммент в issue** (Q23): «PR #N открыт».
6. **Ждёт** merge-gate (см. §3.3).

**Комменты в issue (Q23):** воркер пишет, **когда есть что сказать о ходе** — нашёл причину, выбрал решение, возникло препятствие. Не штампы по каждому шагу. Минимум: старт + PR. Плюс содержательные: «причина бага в X, чиню так», «нашёл решение Y». CI/e2e-статусы комментят кроны (merge-gate/e2e-process) — воркер не дублирует.

**После merge-gate:**
7. Если merge-gate заblock'ил/unblock'ил с причиной (CI красный) — **коммент в issue** (Q23): «CI красный: <check> <лог>», чинит в ТОЙ ЖЕ карточке (коммит → push → ждёт зелёного), **НЕ создаёт новую карточку** (Q21).
8. **Ждёт** `e2e-process` (cron) — прогоняет e2e.
9. **После e2e SUCCESS** — коммент в issue с артефактами (см. §5).
10. **Ждёт** ручного merge юзером (PASS) ИЛИ следующей итерации (FAIL).
11. **После merge** (или `e2e:rejected`):
   - `gh pr merge` (юзер сделал) → удалить `z-{agent}/<id>-<slug>` и `z-{e2e}/wip-<id>-<slug>`:
     ```
     git push origin --delete z-{agent}/<id>-<slug>
     git push origin --delete z-{e2e}/wip-<id>-<slug>
     ```
   - `kanban complete` — **ТОЛЬКО после merge юзером** (Q22).
12. Финальный коммент в issue (Q23): «готово: PR #N смержен, ветка удалена».

### 3.3 `agent-flow-merge-gate` (cron, every 5 min)

**Inputs:** PR из `z-{agent}/<id>-*` (карточка выполнилась → ушла в блок).

**Checks:**
- `gh pr checks <NUM> --watch --exit-status` — все passed.
- `gh pr view <NUM> --json mergeable` — `MERGEABLE=true`.

**Action (Q21):**

1. **CI красный** → пишет коммент в карточку (и issue): «⛔ CI красный: <check_name> (<run_url>) — исправь в ветке <branch>» + `kanban unblock` → воркер снова берёт карточку и чинит в ТОЙ ЖЕ (без новых карточек).
2. **CI зелёный** → **ничего не делает** (карточка остаётся в блоке — ждёт e2e).
3. Дочерние CI-fix карточки НЕ создаются никогда.

### 3.4 `e2e-process` (cron, every 1 hour)

**Inputs:** карточки с label `needs-e2e` (ветка `z-{agent}/<id>-*` смержена в `z-{e2e}/test-round-N`).

**Статус карточки (Q22):** пока идёт e2e — карточка в `block` «ожидает e2e». `done` — ТОЛЬКО после ручного merge юзером.

**Алгоритм:**

1. **Определить N** — текущий номер round:
   - `git ls-remote origin | grep z-{e2e}/test-round-` → max N.
   - Если нет веток → создать `z-{e2e}/test-round-1` от `feature/harness-p0-foundation`.
   - Иначе → использовать `z-{e2e}/test-round-N` (max N).
2. **Резолв всех PR** с `needs-e2e` — `e2e-process` сам мержит:
   ```
   for pr in $(gh pr list --label needs-e2e --json number --jq '.[].number'); do
     branch=$(gh pr view $pr --json headRefName --jq .headRefName)
     base=$(gh pr view $pr --json baseRefName --jq .baseRefName)
     if [ "$base" = "z-{e2e}/test-round-$N" ]; then
       gh pr merge --squash --delete-branch $pr
     fi
   done
   ```
3. **OGG-генерация** (Phase 3+): для каждой issue с `e2e-done` лейблом — если нет OGG в коммитах, **offline на Katana**:
   ```
   ssh ros2@10.1.1.249 "cd /home/ros2/... && docker exec voice-assistant python3 gen_prov.py '<text>' <name>.wav && ffmpeg -i <name>.wav -c:a libopus -b:a 32k -ar 16000 -ac 1 <name>.ogg"
   git add .github/e2e/voice_commands/e2e_<id>/<name>.ogg
   git commit -m "e2e(<id>): add scenario <name>"
   git push origin z-{e2e}/test-round-N
   ```
4. **Запустить e2e прогон** через наш runner:
   ```
   gh workflow run "L-E2E Voice Test.yml" --ref z-{e2e}/test-round-N -f environment=test -f voice_file=.github/e2e/voice_commands/<file>.ogg -f volume=150 -f record_seconds=120
   gh run watch $RUNID --exit-status
   ```
5. **Скачать артефакты** + сгенерить mp3:
   ```
   gh run download --name run-<id>-dialog_e2e.wav
   ffmpeg -i run-<id>-dialog_e2e.wav -c:a libmp3lame -b:a 64k e2e_<id>.mp3
   ```
6. **Comment в issue** с артефактами (см. §5) + коммент в карточку + **`kanban unblock`** → **карточка сама решает по вердикту e2e** (Q22):
   - **e2e PASS** → ждёт юзера: смотрит артефакты, вливает PR вручную → `kanban complete`. Пока не влил — не done.
   - **e2e FAIL** → воркер снова берёт карточку (unblock), чинит в той же ветке → complete → блок → CI → e2e → **следующая итерация**.
7. **Создать новый round:**
   ```
   if e2e SUCCESS: 
     git fetch origin
     git checkout feature/harness-p0-foundation
     git branch z-{e2e}/test-round-$((N+1))
     git push origin z-{e2e}/test-round-$((N+1))
     # Удалить старые:
     for old in $(git branch -r | grep -oE 'z-{e2e}/test-round-[0-9]+' | sort -u | head -n -2); do
       git push origin --delete $old
     done
   else:
     # FAIL — добавить label `e2e:rejected`, ничего не создавать
   ```
8. **После e2e** карточка разблокирована с результатами (шаг 6) и **сама решает**:
   - PASS → ждёт ручного merge юзером → `kanban complete`.
   - FAIL → воркер берёт снова → следующая итерация (чинит → complete → блок → CI → e2e).

**State:** хранить `N` (текущий round) можно в **имени ветки** (max N) либо в файлике `state/e2e_round.txt` в репо (fallback).

### 3.5 Юзер (финальное решение)

**Действия:**

1. Создаёт issue с `hermes` label (может + `agent:<role>` и `priority:P*`).
2. Ждёт пока PR появится (не обязательно, опционально смотреть) — воркер комментит о ходе (Q23).
3. **Когда issue получает коммент с артефактами** (`e2e-done` label) — слушает mp3, читает diff, проверяет acceptance.
4. Если ОК → `gh pr merge --squash z-{agent}/<id>-<slug> → feature/harness-p0-foundation` **→ ТОЛЬКО ПОСЛЕ этого карточка `kanban complete`** (Q22).
5. Если НЕ ОК → `e2e:rejected` label + коммент «что не так», agent делает следующую итерацию.

---

## 4. Триггеры (стыковка с Hermes cronjob)

| Cron | Период | no_agent | script | deliver |
|------|--------|----------|--------|---------|
| `agent-flow-triage` | every 5m | true (pure bash) | `agent-flow-triage.sh` | local |
| `agent-flow-merge-gate` | every 5m | true (pure check) | `agent-flow-merge-gate.sh` | local |
| `e2e-process` | every 1h | false (LLM-driven, но orchestrator) | `agent-flow-e2e-process.sh` | local |

**Hermes cronjob pattern:**
```bash
# agent-flow-triage
cronjob.create(
  prompt="Проверь новые issues с label hermes. Для каждой новой issue создай канбан-карточку с шаблоном (см. §3.1). После создания карточки напиши comment в issue с маркером 'kanban: t_<task_id>'.",
  schedule="every 2m",
  skills=["github"],
  deliver="local"
)
```

---

## 5. Артефакты (что agent кладёт в issue после e2e SUCCESS)

Богатый набор (Вариант C по Q8). Шаблон:

```markdown
## 📊 e2e-доклад #<issue_id> (PR #<pr_num>) — <verdict PASS|FAIL>

### Verdict
✅ PASS (×N kейсов)

### Ссылка на run
[run #<id>](https://github.com/krikz/rob_box_project/actions/runs/<id>)

### Acceptance
- A11 wake+prefix: ✅
- A39 music RMS: ✅ (-16.6 dB > -25 dB threshold)
- A40 wake без префикса: ➖ не тестировалось
- A40b: ✅

### Timing breakdown
- VAD end → STT accept: 5.5с
- LLM streaming first chunk: 1.2с
- LLM full response: 4.1с
- TTS first chunk: 0.8с
- TTS finish: 7.2с

### Audio
- mp3: [attachment: e2e_<id>.mp3]
- wav: [GitHub Artifact run-<id>-dialog_e2e.wav]
- RMS: -16.6 dB

### ASR transcript (Vosk)
> «Робот, диджей Ленин» → «Текст ответа: ...»

### Diff summary
```
 src/.../dj_mode.py | +124 -47
 src/.../prompts/... | +15 -8
```

### Baseline comparison
| Метрика | До фикса | После фикса | Delta |
|---------|----------|-------------|-------|
| DJ transitions until stop | 8 | 24+ | +200% |
| Verdict v7 success rate | 10/10 | 10/10 | = |
```

---

## 6. Решения Q1..Q24 (зафиксировано)

| Q# | Решение |
|----|---------|
| Q1 | `hermes` + `agent:<role>`; default = `hermes` |
| Q2 | все CI зелёные + merge_state=clean + auto-label `needs-e2e`; ветка `z-{agent}/<id>-<slug>` |
| Q3 | resolver (sub-agent) берёт из очереди; OGG offline на Katana |
| Q4 | **Волны раз в час** через `z-{e2e}/test-round-N` |
| Q5 | Agent создаёт `z-{e2e}/wip-<id>-<slug>` → PR в test-round-N → CI → merge. `z-{agent}/<id>` живёт до merge в main |
| Q6 | race невозможен (push в свою wip-ветку) |
| Q7 | **Ручной merge юзером** после просмотра артефактов |
| Q8 | Богатый набор артефактов (verdict+run+log+audio+ASR+diff+acceptance+timing+RMS+baseline) |
| Q9 | Номерованные проходы `z-{e2e}/test-round-N`, N из предыдущей ветки или файлика |
| Q20 | Именование веток: агентские `z-{agent}/<id>-<slug>` (без `~`, удаляются после merge); служебные процесса — только `z-{e2e}/...`; `z-{hotfix}/`/`z-{revert}/` юзер создаёт сам вне процесса |
| Q21 | НЕ создавать CI-fix карточки — block/unblock исходной карточки |
| Q22 | Карточка после complete в блоке; merge-gate: красный CI → unblock; e2e: PASS → unblock с результатами (ждёт merge юзера), FAIL → воркер итерирует; done только после merge юзером |
| Q23 | Воркер комментит в issue содержательно: нашёл причину/решение/препятствие; минимум старт+PR; CI/e2e-статусы пишут кроны |
| Q24 | config.yaml ×19: default_branch develop |

---

## 7. MVP scope (Phase 1 + 2 + 3 mini)

### Phase 1 — Triage MVP ✅ (реализовано 08.08, live-тест #1048/#1049)

- [x] Cron `agent-flow-triage` в Hermes (cronjob create с prompt).
- [x] Парсинг issues через `gh issue list --label hermes --json ...`.
- [x] Создание kanban-карточки.
- [x] Идемпотентность через comment с `kanban: t_<task_id>`.
- [x] Тест: создать issue → 2 мин → карточка появилась → проверить comment в issue.

### Phase 2 — Agent branch+PR ✅ (реализовано 08.08, live-тест #1050)

- [x] Hand-off от карточки → sub-agent.
- [x] Создание `z-{agent}/<id>-<slug>` от develop.
- [x] Push + commit style.
- [x] Тест: 1 issue → PR готов (PR #1051, issue #1050).

### Phase 3 mini — Merge-gate + e2e (rolling-round)

- [x] Cron `agent-flow-merge-gate` (every 5m) — Q21: red → block, green → unblock + `needs-e2e` (переработан по ретроспективе 08.08).
- [ ] Cron `e2e-process` (every 1h) — генерация OGG offline + e2e прогон + артефакты; `z-{e2e}/` префиксы; карточка block до e2e, done после merge (Q22).
- [ ] Round-ветки `z-{e2e}/test-round-N` с rotation.
- [ ] Тест: 1 issue → через час e2e SUCCESS → comment с артефактами.

### Phase 4+ (не в MVP)

- Multi-model e2e matrix (A42)
- Master-plan на N фаз
- Dashboard
- Auto-merge

---

## 8. План реализации (пошаговый)

### Что нужно для Phase 1 (triage MVP)

**По пунктам:**

1. **Канбан-tool wrapper** — CLI для создания/обновления карточек (через существующий kanban API Hermes или новый wrapper). Если wrapper'а нет — Phase 1 невозможен. Решение: сделать минимальный wrapper на Python (3 дня), или использовать существующий kanban-tool если уже есть.
2. **Шаблон комментария** — канонический текст для `kanban: t_<id>` в issue (вынести в `docs/templates/agent_flow_card.md`).
3. **Cronjob** `agent-flow-triage`:
   ```
   prompt: «Проверь issues с label hermes (за период с прошлого тика). 
   Для каждой новой issue создай канбан-карточку через kanban-tool.
   После создания напиши comment в issue: 'kanban: t_<task_id>'.
   На следующем тике пропускай issue, у которых уже есть такой comment.
   Deliver: local (без спама в Telegram).»
   schedule: «every 2m»
   skills: ["github"]
   ```
4. **Идемпотентность** — state хранится в самой issue (comment marker). Никакой внешней БД не нужно.
5. **Документация** — `docs/design/AGENT_FLOW_PROPOSAL.md` (этот файл) уже описывает контракт.

**Acceptance для Phase 1:**
- Создаю issue с label `hermes`.
- В течение 2 мин появляется kanban-карточка с правильной структурой.
- В issue появляется comment с `kanban: t_<id>`.
- Повторный запуск cron'а не создаёт дубль.

### Что нужно для Phase 2 (agent branch+PR)

**По пунктам:**

1. **Selection logic** — `select_agent(role)` — какой sub-agent делает (architect/backend/devops). Определяется по label `agent:<role>` на issue или default `hermes`.
2. **Worktree management** — изоляция от race (как было с билд-раннерами 06.08). Один worktree на agent'а.
3. **Commit conventions** — `feat/fix/docs(scope): ...`.
4. **Push через rebase** — `git fetch + rebase origin/feature/harness-p0-foundation + push + pop stash` (паттерн уже воркеры используют).
5. **Hand-off протокол** — sub-agent получает карточку, читает issue, делает код, оставляет статус.

**Acceptance для Phase 2:**
- Issue `#<id>` с label `hermes` → через 30-60 мин есть PR от `z-{agent}/<id>-<slug>` в feature-ветку с правильным commit-message и `Closes #<id>`.

### Что нужно для Phase 3 mini (merge-gate + e2e)

**По пунктам:**

1. **Cron `agent-flow-merge-gate`** — pure-check через `gh pr checks` + `gh pr view --json mergeable`. Q21: red → `kanban block` + коммент в issue; green → `kanban unblock` + label `needs-e2e`. Дочерние CI-fix карточки НЕ создаются.
2. **Cron `e2e-process`** — orchestrator (LLM-driven):
   - Найти `z-{e2e}/test-round-N` (max N).
   - Резолвнуть PR с `needs-e2e` (если ещё не слиты).
   - Генерация OGG (offline на Katana).
   - `gh workflow run L-E2E Voice Test.yml --ref z-{e2e}/test-round-N`.
   - Скачать артефакт + ffmpeg→mp3 + RMS.
   - Comment в issue с артефактами.
   - Rotation: создать `z-{e2e}/test-round-N+1`, удалить `z-{e2e}/test-round-N-1`.
   - Q22: карточка в block до e2e; `done` только после merge юзером.
3. **OGG-генератор** — скрипт `scripts/gen_ogg.sh` (ssh на Katana + docker exec + ffmpeg).
4. **Артефакт-коммент** — шаблон в `docs/templates/e2e_artifacts.md`.

**Acceptance для Phase 3:**
- Issue → PR → CI зелёные → unblock + label `needs-e2e` → в течение часа e2e прогон на `z-{e2e}/test-round-N` → comment с артефактами в issue → round-N+1 создан → merge юзером → `kanban complete`.

### Что нужно для Phase 4+ (не MVP)

- Multi-model e2e matrix.
- Dashboard.
- Master-plan.
- Auto-merge.

---

## 9. Risks

| R | Risk | Mitigation |
|---|------|-----------|
| R1 | Agent плохо интерпретирует короткое тело карточки | Ссылка на родительское issue, никогда не обрезать полное тело |
| R2 | Race воркеров на общем workspace (было с билд-раннерами) | Отдельный worktree на agent'а, очистка между прогонами |
| R3 | MiniMax 429 — e2e не проходит | OGG-gen offline (Katana), fallback на deepseek/silero (уже есть) |
| R4 | Re-assign в GH ломает assignees | Только в канбане, не в GH (label `e2e-done` + comment) |
| R5 | Merge conflict в `z-{e2e}/test-round-N` | agent резолвит в `z-{e2e}/wip-<id>-*` (изолированная ветка), если не получается → `e2e:rejected` + manual |
| R6 | Agent удаляет ветку, но юзер ещё не посмотрел артефакты | Удалять `z-{agent}/<id>-<slug>` ТОЛЬКО после merge в main (не после e2e SUCCESS) |
| R7 | `z-{e2e}/test-round-N` accumulation | Раз в день cleanup (N-2 и старше удаляются) |
| R8 | ~~Kanban-tool wrapper не существует~~ | ✅ решено — Hermes kanban CLI работает (`hermes kanban create/show/block/unblock/complete`) |

---

## 10. Что НЕ входит

- Авто-merge (запрет Q5)
- Multi-model e2e matrix (A42 OPEN)
- Master-plan на N фаз
- Dashboard / observability

---

## 11. TODO / on-merge (после review юзером)

1. [x] Юзер review proposal'а (issue #1038 comment + коммит)
2. [x] Решения по §6 подтверждены (или поправлены) — Q1..Q9 + Q20..Q24
3. [x] Phase 1 (triage) реализована и протестирована (#1048/#1049)
4. [x] Phase 2 (agent branch+PR) реализована и протестирована (#1050 → PR #1051)
5. [ ] Phase 3: merge-gate переработан (Q21: block/unblock вместо CI-fix карточек)
6. [ ] Phase 3: e2e-process с `z-{e2e}/` префиксами + карточка block до e2e, done после merge (Q22)
7. [ ] SOUL ×19: контракт «комменты в issue о ходе» (Q23)
8. [ ] config.yaml ×19: default_branch develop (Q24)
9. [ ] Именование веток: `~` в triage.sh / e2e-process.sh / merge-gate.sh (Q20)
10. [ ] G-Run Tests на develop — зелёный (run 31251726974, 2m9s)
11. [ ] G-Auto-merge to Main — починен (workflow_call + inputs.*)

---

**Готовый артефакт:** данный файл — после согласования юзером → Phase 1.