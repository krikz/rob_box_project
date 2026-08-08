# Agent Flow Proposal — Issue → branch+PR → e2e round → manual merge → close

| Поле | Значение |
|------|----------|
| Документ | `docs/design/AGENT_FLOW_PROPOSAL.md` |
| Статус | **v1, decisions applied (Q1..Q9 закрыты 06.08 20:25)** — ожидает review юзера перед Phase 1 |
| Дата | 2026-08-06 |
| Автор | PM / юзер (krikz) → оформил: Hermes |
| Issue | **#1038** |
| Связанное | `docs/design/E2E_TESTING_DESIGN_v2.md`, `docs/design/CHILD_TASKS_PROPOSAL.md`, `.github/workflows/L-E2E Voice Test.yml`, канбан `t_*`, существующий cron `e2e-sound-delivery` |

---

## 0. TL;DR

Сквозной поток для агент-воркера, **от ручного создания Issue до ручного merge юзером**, с **автоматизацией** в середине:

```
human → Issue[hermes+agent:role]
   ↓ cron `agent-flow-triage` (every 2m)
kanban-карточка (assignee, инструкция, branch-naming)
   ↓ sub-agent по карточке
worktree → branch `agent/<id>-<slug>` → push
   ↓ cron `agent-flow-merge-gate` (every 2m)
CI зелёные + merge_state=clean → label `needs-e2e`
   ↓ cron `e2e-process` (rolling-round, every 1h)
resolver берёт задачу → мержит в `e2e/wip-<id>-<slug>` → PR в `e2e/test-round-N` → CI → merge в test-round-N → e2e прогон
   ↓ артефакты
issue-коммент: verdict + run + log + audio + ASR + diff + acceptance + timing + RMS + baseline
   ↓ человек (юзер)
ручной merge `agent/<id>-<slug>` → `feature/harness-p0-foundation`
   ↓ cleanup
удалить `agent/<id>-<slug>` и `e2e/wip-<id>-<slug>`, issue close
```

**Ручное:** Issue creation (юзер) и merge approval (юзер). Остальное — автомат.

---

## 1. Что УЖЕ работает (06.08, точки опоры)

| Компонент | Где | Статус |
|-----------|-----|--------|
| Kanban Hermes | `~/.hermes/kanban/boards/robbox/` | ✅ карточки `t_*` |
| Sub-agents | `claude-code`, `codex`, `opencode`, `hermes` | ✅ |
| e2e-script | `/tmp/e2e_series7.sh` (driver) | ✅ 10/10 verdicts v7 |
| e2e-watcher | `/tmp/e2e_watcher7.sh` (mp3-delivery через cron) | ⚠️ paused юзером |
| Отдельный e2e-раннер | `rob-box-e2e-1` (label `e2e`), `L-E2E Voice Test.yml` → `runs-on: e2e` | ✅ |
| GH access | `gh` (GOODWORKRINKZ) | ✅ |
| Репо | `krikz/rob_box_project` | ✅ |

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
| База ветки `e2e/test-round-N` | PR из `e2e/wip-<id>-<slug>` в test-round-N |
| `needs-e2e` | auto от merge-gate когда CI зелёные |
| `e2e-done` | после успешного e2e (с артефактами) |
| `e2e:rejected` | юзер отказал merge'у |

---

## 3. Контракты между частями

### 3.1 `agent-flow-triage` (cron, every 2 min)

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
  • branch: agent/<id>-<short-slug>
  • merge: PR `e2e/wip-<id>-<slug>` → `e2e/test-round-N`
  ## Done criteria
  • CI all green
  • PR merged в test-round-N
  • e2e SUCCESS
  • Артефакты приложены в issue
  • Manual merge юзером
assignee: <agent:<role> или hermes>
priority: P<P0..P2>
```

**Идемпотентность:** при создании карточки пишет **коммент в issue** с маркером `kanban: t_<task_id>`. На следующем тике cron смотрит на наличие такого комментария — пропускает.

### 3.2 Agent-исполнитель (sub-agent)

**Inputs:** канбан-карточка `t_<id>`.

**Действия (пошагово):**

1. Получает карточку через kanban-tool.
2. Создаёт worktree (свой собственный, не `_work`!) от `feature/harness-p0-foundation`:
   ```
   git worktree add -b agent/<id>-<slug> /home/builder/wt-<id> origin/feature/harness-p0-foundation
   ```
3. Делает код по инструкции из карточки. Если нужно — читает родительский issue.
4. **Commit style:** `feat/fix/docs(scope): ...`.
5. Push в `origin/agent/<id>-<slug>`.
6. **Ждёт** пока merge-gate не выставит `needs-e2e`.
7. Когда выставлен — переключается на `e2e/test-round-N`:
   ```
   git fetch origin
   git checkout -b e2e/wip-<id>-<slug> origin/e2e/test-round-N
   git merge --no-ff origin/agent/<id>-<slug>  # резолвит конфликты если есть
   git push origin e2e/wip-<id>-<slug>
   gh pr create --base e2e/test-round-N --head e2e/wip-<id>-<slug>
   ```
8. **Ждёт** пока `e2e-bot` (cron) не смёрджит `e2e/wip-<id>-<slug>` → `e2e/test-round-N` (CI зелёные).
9. **Ждёт** `e2e-process` (cron) — прогоняет e2e на свежем `e2e/test-round-N`.
10. **После e2e SUCCESS** — пишет коммент в issue с артефактами (см. §5).
11. **Ждёт** ручного merge юзером.
12. **После merge** (или `e2e:rejected`):
   - `gh pr merge` (юзер сделал) → удалить `agent/<id>-<slug>` и `e2e/wip-<id>-<slug>`:
     ```
     git push origin --delete agent/<id>-<slug>
     git push origin --delete e2e/wip-<id>-<slug>
     ```

### 3.3 `agent-flow-merge-gate` (cron, every 2 min)

**Inputs:** PR из `agent/<id>-*` в `feature/harness-p0-foundation` или комментарий-триггер от agent.

**Checks:**
- `gh pr checks <NUM> --watch --exit-status` — все passed.
- `gh pr view <NUM> --json mergeable` — `MERGEABLE=true`.

**Action:** добавить label `needs-e2e` к PR (если ещё нет).

### 3.4 `e2e-process` (cron, every 1 hour)

**Inputs:** PR с label `needs-e2e` в `e2e/test-round-N`.

**Алгоритм:**

1. **Определить N** — текущий номер round:
   - `git ls-remote origin | grep e2e/test-round-` → max N.
   - Если нет веток → создать `e2e/test-round-1` от `feature/harness-p0-foundation`.
   - Иначе → использовать `e2e/test-round-N` (max N).
2. **Резолв всех PR** с `needs-e2e` (если agent ещё не слил) — `e2e-process` сам мержит:
   ```
   for pr in $(gh pr list --label needs-e2e --json number --jq '.[].number'); do
     branch=$(gh pr view $pr --json headRefName --jq .headRefName)
     base=$(gh pr view $pr --json baseRefName --jq .baseRefName)
     if [ "$base" = "e2e/test-round-$N" ]; then
       gh pr merge --squash --delete-branch $pr
     fi
   done
   ```
3. **OGG-генерация** (Phase 3+): для каждой issue с `e2e-done` лейблом — если нет OGG в коммитах, **offline на Katana**:
   ```
   ssh ros2@10.1.1.249 "cd /home/ros2/... && docker exec voice-assistant python3 gen_prov.py '<text>' <name>.wav && ffmpeg -i <name>.wav -c:a libopus -b:a 32k -ar 16000 -ac 1 <name>.ogg"
   git add .github/e2e/voice_commands/e2e_<id>/<name>.ogg
   git commit -m "e2e(<id>): add scenario <name>"
   git push origin e2e/test-round-N
   ```
4. **Запустить e2e прогон** через наш runner:
   ```
   gh workflow run "L-E2E Voice Test.yml" --ref e2e/test-round-N -f environment=test -f voice_file=.github/e2e/voice_commands/<file>.ogg -f volume=150 -f record_seconds=120
   gh run watch $RUNID --exit-status
   ```
5. **Скачать артефакты** + сгенерить mp3:
   ```
   gh run download --name run-<id>-dialog_e2e.wav
   ffmpeg -i run-<id>-dialog_e2e.wav -c:a libmp3lame -b:a 64k e2e_<id>.mp3
   ```
6. **Comment в issue** с артефактами (см. §5).
7. **Создать новый round:**
   ```
   if e2e SUCCESS: 
     git fetch origin
     git checkout feature/harness-p0-foundation
     git branch e2e/test-round-$((N+1))
     git push origin e2e/test-round-$((N+1))
     # Удалить старые:
     for old in $(git branch -r | grep -oE 'e2e/test-round-[0-9]+' | sort -u | head -n -2); do
       git push origin --delete $old
     done
   else:
     # FAIL — добавить label `e2e:rejected`, ничего не создавать
   ```

**State:** хранить `N` (текущий round) можно в **имени ветки** (max N) либо в файлике `state/e2e_round.txt` в репо (fallback).

### 3.5 Юзер (финальное решение)

**Действия:**

1. Создаёт issue с `hermes` label (может + `agent:<role>` и `priority:P*`).
2. Ждёт пока PR появится (не обязательно, опционально смотреть).
3. **Когда issue получает коммент с артефактами** (`e2e-done` label) — слушает mp3, читает diff, проверяет acceptance.
4. Если ОК → `gh pr merge --squash agent/<id>-<slug> → feature/harness-p0-foundation`.
5. Если НЕ ОК → `e2e:rejected` label + коммент «что не так», agent делает следующую итерацию.

---

## 4. Триггеры (стыковка с Hermes cronjob)

| Cron | Период | no_agent | script | deliver |
|------|--------|----------|--------|---------|
| `agent-flow-triage` | every 2m | false (LLM-driven) | — | local |
| `agent-flow-merge-gate` | every 2m | true (pure check) | `gh pr checks` | local |
| `e2e-process` | every 1h | false (LLM-driven, но orchestrator) | — | local |

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

## 6. Решения Q1..Q9 (зафиксировано)

| Q# | Решение |
|----|---------|
| Q1 | `hermes` + `agent:<role>`; default = `hermes` |
| Q2 | все CI зелёные + merge_state=clean + auto-label `needs-e2e`; ветка `agent/<id>-<slug>` |
| Q3 | resolver (sub-agent) берёт из очереди; OGG offline на Katana |
| Q4 | **Волны раз в час** через `e2e/test-round-N` |
| Q5 | Agent создаёт `e2e/wip-<id>-<slug>` → PR в test-round-N → CI → merge. `agent/<id>` живёт до merge в main |
| Q6 | race невозможен (push в свою wip-ветку) |
| Q7 | **Ручной merge юзером** после просмотра артефактов |
| Q8 | Богатый набор артефактов (verdict+run+log+audio+ASR+diff+acceptance+timing+RMS+baseline) |
| Q9 | Номерованные проходы `e2e/test-round-N`, N из предыдущей ветки или файлика |

---

## 7. MVP scope (Phase 1 + 2 + 3 mini)

### Phase 1 — Triage MVP

- [ ] Cron `agent-flow-triage` в Hermes (cronjob create с prompt).
- [ ] Парсинг issues через `gh issue list --label hermes --json ...`.
- [ ] Создание kanban-карточки (нужен kanban-tool wrapper).
- [ ] Идемпотентность через comment с `kanban: t_<task_id>`.
- [ ] Тест: создать issue → 2 мин → карточка появилась → проверить comment в issue.

### Phase 2 — Agent branch+PR

- [ ] Hand-off от карточки → sub-agent.
- [ ] Создание `agent/<id>-<slug>` от feature-ветки.
- [ ] Push + commit style.
- [ ] Тест: 1 issue → через 30-60 мин PR готов.

### Phase 3 mini — Merge-gate + e2e (rolling-round)

- [ ] Cron `agent-flow-merge-gate` (every 2m) — label `needs-e2e`.
- [ ] Cron `e2e-process` (every 1h) — генерация OGG offline + e2e прогон + артефакты.
- [ ] Round-ветки `e2e/test-round-N` с rotation.
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
- Issue `#<id>` с label `hermes` → через 30-60 мин есть PR от `agent/<id>-<slug>` в feature-ветку с правильным commit-message и `Closes #<id>`.

### Что нужно для Phase 3 mini (merge-gate + e2e)

**По пунктам:**

1. **Cron `agent-flow-merge-gate`** — pure-check через `gh pr checks` + `gh pr view --json mergeable`. Label `needs-e2e` если оба ОК.
2. **Cron `e2e-process`** — orchestrator (LLM-driven):
   - Найти `e2e/test-round-N` (max N).
   - Резолвнуть PR с `needs-e2e` (если ещё не слиты).
   - Генерация OGG (offline на Katana).
   - `gh workflow run L-E2E Voice Test.yml --ref e2e/test-round-N`.
   - Скачать артефакт + ffmpeg→mp3 + RMS.
   - Comment в issue с артефактами.
   - Rotation: создать `e2e/test-round-N+1`, удалить `e2e/test-round-N-1`.
3. **OGG-генератор** — скрипт `scripts/gen_ogg.sh` (ssh на Katana + docker exec + ffmpeg).
4. **Артефакт-коммент** — шаблон в `docs/templates/e2e_artifacts.md`.

**Acceptance для Phase 3:**
- Issue → PR → CI зелёные → label `needs-e2e` → в течение часа e2e прогон на `e2e/test-round-N` → comment с артефактами в issue → round-N+1 создан.

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
| R5 | Merge conflict в `e2e/test-round-N` | agent резолвит в `e2e/wip-<id>-*` (изолированная ветка), если не получается → `e2e:rejected` + manual |
| R6 | Agent удаляет ветку, но юзер ещё не посмотрел артефакты | Удалять `agent/<id>-<slug>` ТОЛЬКО после merge в main (не после e2e SUCCESS) |
| R7 | `e2e/test-round-N` accumulation | Раз в день cleanup (N-2 и старше удаляются) |
| R8 | Kanban-tool wrapper не существует | Phase 1 blocker — нужно сделать wrapper или взять существующий |

---

## 10. Что НЕ входит

- Авто-merge (запрет Q5)
- Multi-model e2e matrix (A42 OPEN)
- Master-plan на N фаз
- Dashboard / observability

---

## 11. TODO / on-merge (после review юзером)

1. [ ] Юзер review proposal'а (issue #1038 comment + коммит)
2. [ ] Решения по §6 подтверждены (или поправлены)
3. [ ] Создать issue `feat(agent-flow): phase 1 triage MVP`
4. [ ] Сделать карточку канбана Phase 1
5. [ ] Проверить наличие kanban-tool wrapper (R8) — без него Phase 1 невозможен
6. [ ] Сделать wrapper если нет
7. [ ] Завести cron `agent-flow-triage` в Hermes
8. [ ] Тест на 1 issue — посмотреть, пришла ли карточка
9. [ ] Review результата юзером

---

**Готовый артефакт:** данный файл — после согласования юзером → Phase 1.