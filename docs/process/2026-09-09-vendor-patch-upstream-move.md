# 2026-09-09 — vendor patch upstream-move problem (issue #2332)

## Context

Issue #2332: `scripts/agent_flow/vendor/hermes-agent-skill-validation.patch`
больше не применяется в install.sh:

```
==> hermes-agent patch: scripts/agent_flow/vendor/hermes-agent-skill-validation.patch
  ERROR patch does not apply cleanly to /home/builder/.hermes/hermes-agent — upstream moved
         Regenerate with: bash scripts/agent_flow/agent-flow-regen-vendor-patch.sh scripts/agent_flow/vendor/hermes-agent-skill-validation.patch
```

Корневая причина: `agent-flow-regen-vendor-patch.sh` жёстко проверяет, что
patch трогает ровно 3 файла (`kanban_db.py`, `profiles.py`,
`test_kanban_db.py`), а наш patch уже другой — 2 файла
(`kanban.py` + `kanban_db.py`), потому что добавили `--force-scope` flag.

Также: live tree `~/.hermes/hermes-agent` находится на ветке
`z-devops/t_16a245cc-goal-mode-clean-exit-recovery`, которая содержит
коммит `6c2be533d feat(kanban): pre-create skill-validation + scope-hint
(ADR-0036 §4.1, t_51394ac6)` — это **уже наш vendor patch, интегрированный
руками в dev-ветку**. Upstream `origin/main` (ae43fd6df) наш фикс НЕ
содержит, но live tree отстаёт от рабочей ветки на 23082 коммита (HEAD
живой = `180291162 feat(telemetry)...`, а рабочая ветка ушла далеко вперёд).

## Strategy (Вариант C: сделать install.sh idempotent + helper generic)

### 1. install.sh: apply_hermes_agent_patch — добавить sentinel-detect

Перед `git apply --check` проверяем, нет ли в live tree сигнатуры фикса
(например, `def _profile_skill_names` в `hermes_cli/kanban_db.py`). Если
есть — patch уже применён (ручной merge / upstream включил) → SKIP с info,
не ERROR.

### 2. agent-flow-regen-vendor-patch.sh: generic N-file pattern

Сейчас helper жёстко проверяет 3-файловый pattern и падает на любых
других. Переписываем на generic N-file с auto-discovery:
  - Извлекаем из patch список файлов через `grep '^diff --git '`
  - Carry-over каждого файла из live → patched (т.е. patch по сути
    берёт `+` lines и вставляет обратно в контекст live файла)
  - Для kanban_db.py / kanban.py: insertion по anchor (hunk1/hunk2 split)
  - Для остальных файлов: `+` lines replace wholesale

### 3. Patch файл остаётся

Patch `hermes-agent-skill-validation.patch` остаётся в репо — он полезен
для свежего `origin/main` checkout'а, где наш фикс ещё не смержен.
Когда live tree включает фикс через dev-ветку (наш случай), sentinel-
detect в install.sh skip'ает patch.

## Tests

- `tests/test_vendor_patch_apply.sh` остаётся — он покрывает baseline
  "patch applies to fresh origin/main", и пока live отстаёт от рабочей
  ветки, patch остаётся валидным для origin/main.
- `tests/test_regen_vendor_patch.sh` обновляется под generic N-file:
  не требует 3-файловый pattern, должен пройти на текущем 2-файловом
  patch'е.
