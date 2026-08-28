# e2e-rotation recovery: "blocker closed but PR not merged"

> Процедура восстановления после того, как issue-блокер закрыт, но блокирующий PR
> (например #1670 / #1673 / #1674 для #1668) был CLOSED без merge. Без восстановления
> watchdog-блок продолжает блокировать ротацию, fail-streak растёт бесконечно.

**Контекст:** ретро t_bb56f2a1 (28.08.2026) — fail-streak 17+ после #1668 CLOSED.

---

## Симптом

- L: E2E Voice Test падает N+ раз подряд без SUCCESS.
- В комментариях `needs-e2e` issues есть запись:
  ```
  agent-flow: ⏸️ e2e приостановлен: блокер #<N> — новый round не создаётся, пока блокер открыт
  ```
  но issue #N формально `CLOSED`.
- `gh run list --workflow="L: E2E Voice Test" --limit N` показывает только `failure`,
  последний success — несколько дней назад.
- `git ls-remote --heads origin 'z-{e2e}/test-round-*'` показывает последний
  round от 25-26.08, после — ничего.

## Корневая причина

`agent-flow-e2e-process.sh::detect_known_blocker()` (после ретро t_bb56f2a1) ищет
OPEN issue с signature в title/body. Сигнатура `no_wake_word` (по умолчанию) часто
совпадает с эпик-фичами, в которых встречается слово "wake-word" в контексте
(например #1684 Captain Bridge, #1506 e2e voice regressions и т.п.) → false-positive
блокер. Когда основной issue-блокер (#1668) закрывается, но epic-фича остаётся
OPEN — фильтр продолжает ловить её как блокер.

Вторая проблема: при `gh issue list --label X` пусто (GraphQL filter bug) +
REST `/issues?labels=X` fallback — JSON парсер Python в `collect_issues_json`
бросал `JSONDecodeError: Extra data: line 2 column 1`, скрипт падал с exit 1,
cron выдавал "script failed" без нового round'а.

## Recovery-процедура (по шагам)

#### 1. Подтвердить, что ротация реально застряла

```bash
export GH_CONFIG_DIR=/home/builder/.config/gh
# Последний round:
git ls-remote --heads origin 'z-{e2e}/test-round-*' | sort -t- -k3 -n | tail -3
# Последний успех:
gh run list --workflow="L: E2E Voice Test" --limit 25 \
  --json conclusion,createdAt,headBranch \
  | jq '[.[] | select(.conclusion == "success")] | .[0:3]'
```

Если последний round старше 1 дня и SUCCESS'ов нет → блок реальный.

#### 2. Проверить detect_known_blocker в dry-run

```bash
cd /home/builder/hermes-share/rob_box_project
# Загрузить detect_known_blocker
sed -n '380,500p' scripts/agent_flow/agent-flow-e2e-process.sh > /tmp/_detect.sh
GH_REPO=krikz/rob_box_project KNOWN_BLOCKER_SIGNATURES="no_wake_word" \
  bash -c 'source /tmp/_detect.sh; echo "BLOCKER=[$(detect_known_blocker)]"'
```

Если выводит `#<N>` где #N — feature/epic (не bug/process/voice/e2e) →
**false-positive**. Это и есть корень паузы.

#### 3. Снять блок (3 способа, в порядке приоритета)

**Способ A — Unpause через env (для Шифу/экстренно):**

```bash
E2E_FORCE_UNPAUSE=true bash /home/builder/.hermes/scripts/agent-flow-e2e-process.sh
```

Это пропустит ВСЕ блокер-чеки на один тик. **Использовать осторожно**: если
настоящий блокер не починен, e2e будет жечь CI minutes на заведомо падающие
сценарии.

**Способ B — Hot-fix KNOWN_BLOCKER_SIGNATURES:**

```bash
# Например, если сигнатура "no_wake_word" ловит эпики — добавить более точную:
export KNOWN_BLOCKER_SIGNATURES="exact_wake_word_signature"
# в /home/builder/.hermes/profiles/architect/cron/jobs.json — env для скрипта.
```

**Способ C — Process-fix (правильный путь, merge в develop):**

1. Создать ветку `z-devops/<id>-e2e-rotation-recovery` от develop.
2. Расширить `blocker_issue_for_sig` exclude-фильтр (feature, epic:*, xr,
   source:gsd).
3. Добавить false-positive guard в `detect_known_blocker`: если hit-issue не
   имеет bug/process/voice/e2e/regression меток → это явно не блокер.
4. Защитить `collect_issues_json` от JSON parse errors (try/except в inline
   python, fallback на оригинальный `issues_json`).
5. Открыть PR в develop, дождаться merge (Шифу делает merge руками по Q22).
6. После merge — sync всех 4 хост-путей (hardlink, через `os.link`):
   - `~/.hermes/scripts/agent-flow-e2e-process.sh`
   - `~/.hermes/profiles/agent-flow/scripts/agent-flow-e2e-process.sh`
   - `~/.hermes/profiles/architect/scripts/agent-flow-e2e-process.sh`
   - `~/.hermes/profiles/devops/scripts/agent-flow-e2e-process.sh`

Способ A — на одну сессию (E2E_FORCE_UNPAUSE в env cron'а), способ C —
навсегда.

#### 4. Resume e2e-rotation (force tick)

```bash
# Вариант 1: подождать cron tick (1h)
# Вариант 2: форсировать немедленный tick
nohup bash /home/builder/.hermes/scripts/agent-flow-e2e-process.sh > \
  /tmp/e2e_force_resume.log 2>&1 &
```

Следить:
```bash
tail -f /tmp/e2e_force_resume.log
# или cron output:
tail -f /home/builder/.hermes/profiles/architect/cron/output/73dcdece0619/*.md
```

#### 5. Verify новый round создаётся

```bash
# Через 5-10 мин должен появиться новый z-{e2e}/test-round-N+1:
git ls-remote --heads origin 'z-{e2e}/test-round-*' | sort -t- -k3 -n | tail -3
```

## Что НЕ делать

- ❌ **Руками** `gh issue close` для needs-e2e issues без разбора (нарушение
  Q22, могут быть потеряны свидетельства).
- ❌ **`git push` в develop без PR** — Шифу мержит руками (Q22).
- ❌ **Менять KNOWN_BLOCKER_SIGNATURES без документации** — следующий ретро
  потеряет контекст.
- ❌ **Удалять `~/.hermes/state/agent-flow-e2e-round-counter`** — потеряется
  нумерация round'ов (см. skill agent-flow-e2e-ops).

## Связанные

- ADR-0011 — KNOWN_BLOCKER_SIGNATURES (как соглашение по умолчанию)
- PR #1721 (fail-streak escalation) — комлементарный watchdog для fail-streak ≥ 5
- PR #1725 (auto-needs-review при fail-streak ≥ 5) — sweep готовых PR
- Issue #1668 — wake-gate regression (закрыт)
- Issue #1707 — worktree cleanup (закрыт вручную после #1710 merge)
- Skill `agent-flow-e2e-ops` — повседневная работа с e2e cron
- Skill `agent-flow-e2e-pipeline` — архитектура e2e-process / merge-gate / watchdog