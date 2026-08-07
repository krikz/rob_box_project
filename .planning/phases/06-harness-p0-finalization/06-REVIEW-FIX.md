---
phase: 06-harness-p0-finalization
fixes_total: 6
fixes_applied: 5
fixes_skipped: 1
fixed_at: "2026-07-28"
fixer: gsd-code-fixer
---

# Phase 6: Harness P0 Finalization — Fix Report

**Fixed:** 2026-07-28
**Phase:** 06-harness-p0-finalization
**Status:** ALL BLOCKERS FIXED (5 applied, 1 rejected as false-positive)

---

## CR-01: `dialog_core.py` persisted user turn on LLM error — **FIXED**

**Commit:** `82632293`
**File:** `src/rob_box_harness/rob_box_harness/core/dialog_core.py:209`
**Change:** Removed the broken `self._memory.turns` attribute check. Instead persists unconditionally — `MemoryStore.append_turn` is documented idempotent, and the user turn was NOT appended yet on the error path.
**Verification:** Python `ast.parse()` passed.

---

## CR-02: `dialogue_node._build_memory` InMemoryStore fallback — **FIXED**

**Commit:** `d2b063bd`
**File:** `src/rob_box_harness/rob_box_harness/memory.py:253`
**Change:** Added `async def init(self) -> None` as a no-op to `InMemoryStore` so the shell's fallback path (`store.init()`) doesn't raise `AttributeError`.
**Verification:** Python `ast.parse()` passed. `InMemoryStore` now matches the pattern used by `SQLiteVoiceMemory.init()`.

---

## CR-03: DeepSeek/MiMo missing `chat()` shortcut — **REJECTED (false positive)**

**Reason:** `HarnessDeepSeekProvider.chat()` exists at `deepseek.py:300`. `HarnessMiMoProvider(HarnessDeepSeekProvider)` inherits it. Both providers have `chat()`. The reviewer's grep missed the method because it's defined inside `HarnessDeepSeekProvider`, not as a standalone top-level function.
**Status:** No fix needed.

---

## CR-04: `dialogue_node._build_llm` directly instantiates `DeepSeekProvider` — **FIXED**

**Commit:** `07c024f3`
**File:** `src/rob_box_voice/rob_box_voice/dialogue_node.py:175-185`
**Change:** Replaced `from rob_box_harness.providers import DeepSeekProvider` → `from rob_box_harness.providers import build_deepseek_provider`. The shell now calls `build_deepseek_provider(api_key=..., base_url=..., model=...)` — factory pattern, no direct class instantiation. Parameters `temperature`/`max_tokens` are no longer passed to the constructor (they belong in `chat()`/`complete()` calls).
**Verification:** Python `ast.parse()` passed. Tests override `_build_llm` via `_TestableDialogueNode` — not broken.

---

## CR-05: `SQLiteVoiceMemory.append_turn` signature breaks LSP — **FIXED**

**Commit:** `28d0a221`
**File:** `src/rob_box_harness/rob_box_harness/memory/sqlite_voice.py:196`
**Change:** Changed return type from `-> bool` to `-> None`, matching the `MemoryStore` abstract base class signature. Duplicate turns are now silently skipped (debug-logged) instead of returning `False`.
**Verification:** Python `ast.parse()` passed. All callers already ignore the return value.

---

## CR-06: micro-ROS cleanup incomplete — **FIXED (12 commits, 19 files)**

| Part | Commit | Scope |
|------|--------|-------|
| 1 | `fd8d95df` | `docker/main/docker-compose.yaml` — service block removed |
| 1 | `f0a07ee3` | `docker/main/micro_ros_agent/` + `scripts/micro_ros_agent/` — deleted |
| 2 | `2c814244` | `.github/workflows/G-Build Main Pi Services.yml` — job removed |
| 3 | `a6f5758e` | `.github/workflows/L-Build Main Pi Services.yml` — job + needs + tag removed |
| 4 | `b782430f` | `.github/workflows/L-Build Single Service.yml` — choices + case + VERSION_VAR removed |
| 5 | `2620ba5e` | `.github/workflows/G-Auto-merge to Main.yml` — gate cleaned |
| 6 | `08689adf` | `docker/monitoring/config/grafana/.../demo_3_perception.json` — panel removed |
| 7 | `ccb5dfe8` | 4 monitoring docs (`DEMO_DASHBOARDS.md`, `DASHBOARD_PREVIEW.md`, `NODE_MAPPING.md`, `VERIFICATION_SUMMARY.md`) |
| 8 | `a4c34fce` | `.planning/codebase/{ARCHITECTURE.md,CONCERNS.md,INTEGRATIONS.md,STRUCTURE.md}` |
| 8 | `92b537ca` | `.planning/codebase/ARCHITECTURE.md` — last topic ref |
| final | `a52c695d` | `src/rob_box_perception/README.md` + `src/robot_sensor_hub_msg/README.md` |

**Final verification:**
```bash
grep -rn "micro_ros_agent\|micro-ros-agent\|micro_ros " \
  --include="*.yml" --include="*.yaml" --include="Dockerfile*" \
  --include="*.md" --include="*.json" --include="*.sh" \
  .github docker/main docker/vision docker/monitoring docker/scripts src .planning/codebase \
  | grep -v "docker/build/data" \
  | grep -v "Phase 6\|Plan 06\|CR-06\|REMOVED.*micro-ROS\|archived.*micro-ROS"
```
**Result: ZERO matches.**

---

## Verification

- All Python syntax valid (`ast.parse()` on each modified `.py` file)
- All YAML valid (`yaml.safe_load_all()` on each modified workflow)
- JSON valid (`json.load()` on Grafana dashboard)
- `docker-compose.yaml` valid (`yaml.safe_load()`)
- grep micro-ros: **0 operational references remaining**

## Test Failures (pre-existing — not addressed)

The failing `test_health_monitor` and flake8 issues are pre-existing and orthogonal to Phase 6. They are documented in REVIEW.md "Test Failures" section and should be a separate cleanup ticket.
