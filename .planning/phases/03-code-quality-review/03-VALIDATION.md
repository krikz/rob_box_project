---
phase: 3
slug: code-quality-review
status: draft
nyquist_compliant: false
wave_0_complete: false
created: 2026-05-15
---

# Phase 3 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | pytest (existing) |
| **Config file** | `src/*/test/` directories |
| **Quick run command** | `python3 -m flake8 src/rob_box_voice/ src/rob_box_perception/ --max-line-length=120 --extend-ignore=E203,W503 --statistics 2>&1 \| tail -5` |
| **Full suite command** | `cd /home/ros2/rob_box_project && python3 -m pytest src/ -x -q 2>&1 \| tail -10` |
| **Estimated runtime** | ~30 seconds |

---

## Sampling Rate

- **After every task commit:** Run flake8 quick check on modified package
- **After every plan wave:** Full coverage check + grep для STUB markers
- **Before `/gsd-verify-work`:** All success criteria verified
- **Max feedback latency:** ~30 seconds

---

## Per-Task Verification Map

| Task ID | Plan | Wave | Requirement | Test Type | Automated Command | Status |
|---------|------|------|-------------|-----------|-------------------|--------|
| 03-01-01 | 01 | 1 | CQ-01, CQ-02 | docs | `grep -c "severity" .planning/CONCERNS_AUDIT.md` | ⬜ pending |
| 03-01-02 | 01 | 1 | CQ-01 | docs | `python3 -c "import json; d=json.load(open('tasks.json')); print(len([t for t in d if t.get('id','').startswith('BG-')]))"` | ⬜ pending |
| 03-02-01 | 02 | 1 | CQ-03 | cli | `python3 -m flake8 src/rob_box_voice/ --max-line-length=120 --statistics 2>&1 \| tail -5` | ⬜ pending |
| 03-02-02 | 02 | 1 | CQ-03 | docs | `test -f .planning/STATIC_ANALYSIS_REPORT.md && echo OK` | ⬜ pending |
| 03-03-01 | 03 | 1 | CQ-04 | docs | `test -f .planning/COVERAGE_REPORT.md && echo OK` | ⬜ pending |
| 03-04-01 | 04 | 1 | CQ-05 | docs | `test -f .planning/DIALOGUE_NODE_REFACTORING.md && echo OK` | ⬜ pending |
| 03-05-01 | 05 | 1 | CQ-06 | code | `grep -rn "# STUB:" src/rob_box_voice/ src/rob_box_mcp_tools/ \| wc -l` | ⬜ pending |
| 03-05-02 | 05 | 1 | CQ-06 | docs | `python3 -c "import json; d=json.load(open('tasks.json')); stubs=[t for t in d if 'STUB' in t.get('description','') or 'stub' in t.get('id','').lower()]; print(len(stubs))"` | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

Существующая инфраструктура покрывает все требования фазы. Новые тесты не создаются — фаза документирует, не исправляет.

*"Existing infrastructure covers all phase requirements."*

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| black/isort отчёт | CQ-03 | black/isort не установлены на dev-машине | Запустить через Docker: `docker run --rm -v $(pwd):/app python:3.10 sh -c "pip install black isort && black --check /app/src/rob_box_voice/ --line-length 120"` |
| CONCERNS.md severity review | CQ-02 | Требует экспертной оценки | Прочитать CONCERNS.md + TECH_DEBT.md, подтвердить корректность severity/disposition |

---

## Success Criteria (Phase 3 must_haves)

| Критерий | Команда проверки | Ожидаемый результат |
|----------|-----------------|---------------------|
| CQ-01: bugs в трекере | `python3 -c "import json; d=json.load(open('tasks.json')); print([t['id'] for t in d if t.get('id','').startswith('BG')])"` | BG-5, BG-6 присутствуют |
| CQ-02: TECH_DEBT.md создан | `test -f .planning/TECH_DEBT.md && echo OK` | OK |
| CQ-03: отчёт создан | `test -f .planning/STATIC_ANALYSIS_REPORT.md && echo OK` | OK |
| CQ-04: coverage report | `test -f .planning/COVERAGE_REPORT.md && echo OK` | OK |
| CQ-05: refactoring plan | `test -f .planning/DIALOGUE_NODE_REFACTORING.md && echo OK` | OK |
| CQ-06: STUB markers | `grep -rn "# STUB:" src/ --include="*.py" \| wc -l` | ≥ 4 |

---

## Validation Sign-Off

- [ ] All tasks have automated verify or manual test instructions
- [ ] Sampling continuity: каждый план заканчивается верификацией
- [ ] Wave 0 не требуется (только документация, не код)
- [ ] No watch-mode flags
- [ ] `nyquist_compliant: true` set in frontmatter после завершения

**Approval:** pending
