---
phase: 2
slug: review-structure
status: draft
nyquist_compliant: false
wave_0_complete: false
created: 2026-05-15
---

# Phase 2 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | bash / grep (структурная верификация) |
| **Config file** | none |
| **Quick run command** | `grep -r "COPY config\|COPY scripts" docker/ --include="Dockerfile*"` |
| **Full suite command** | см. Per-Task Verification Map ниже |
| **Estimated runtime** | ~10 seconds |

---

## Sampling Rate

- **After every task commit:** Run quick grep checks
- **After every plan wave:** Run full suite grep commands
- **Before `/gsd-verify-work`:** All success criteria must return 0 results
- **Max feedback latency:** 10 seconds

---

## Per-Task Verification Map

| Task ID | Plan | Wave | Requirement | Test Type | Automated Command | Status |
|---------|------|------|-------------|-----------|-------------------|--------|
| 02-01-01 | 01-01 | 1 | STRUCT-01 | grep | `grep -c "network_mode" docker/main/docker-compose.yaml` | ⬜ |
| 02-01-02 | 01-01 | 1 | STRUCT-02 | grep | `grep -rn "^COPY config\|^COPY scripts" docker/ --include="Dockerfile*"; echo "violations=$?"` | ⬜ |
| 02-02-01 | 02-02 | 1 | STRUCT-03 | grep | `grep -r "TODO" src/rob_box_perception/setup.py; echo $?` | ⬜ |
| 02-02-02 | 02-02 | 1 | STRUCT-03 | grep | `grep -r "your_email\|user@example" src/ --include="setup.py"` | ⬜ |
| 02-02-03 | 02-02 | 1 | STRUCT-03/04 | file_exists | `test -f src/rob_box_description/URDF_EXPORT/COLCON_IGNORE && echo OK` | ⬜ |
| 02-03-01 | 02-03 | 1 | STRUCT-05 | dir_absent | `test ! -d src/led_matrix_driver && echo "OK: removed"` | ⬜ |
| 02-03-02 | 02-03 | 1 | STRUCT-05 | ci_check | `grep "led_matrix_driver" .github/workflows/G-Lint\ Code.yml` | ⬜ |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

Существующая инфраструктура (grep, bash) покрывает все требования фазы. Wave 0 не требует создания новых файлов.

*Existing infrastructure covers all phase requirements.*

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| Исключения в docker-compose задокументированы комментариями | STRUCT-01 | Семантическая проверка | Проверить наличие `# non-ROS service` комментариев для voice-resources-init, supercollider, ollama, cadvisor, promtail |
| src/ros2leds — git submodule, placeholder emails неизменимы без PR | STRUCT-03 | Вне scope проекта | Убедиться, что файлы в src/ros2leds/ не модифицированы, задокументировать в STATE.md |

---

## Validation Sign-Off

- [ ] All tasks have `<automated>` verify or Wave 0 dependencies
- [ ] Sampling continuity: no 3 consecutive tasks without automated verify
- [ ] Wave 0 covers all MISSING references
- [ ] No watch-mode flags
- [ ] Feedback latency < 15s
- [ ] `nyquist_compliant: true` set in frontmatter

**Approval:** pending
