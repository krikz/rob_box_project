---
phase: 06-harness-p0-finalization
plan: 05
subsystem: voice
tags: [dialog, harness, state-machine, dialogue, voice-assistant]
requires:
  - phase: 06
    provides: "Plan 06-04 provided ROS2Transport and SQLiteVoiceMemory port implementations"
provides:
  - "DialogHarness: Harness[DialogState] wrapping the full dialogue pipeline (wake-word→STT→LLM→TTS)"
  - "DialogueStateMachine: pure state machine for IDLE/LISTENING/DIALOGUE/SILENCED with 8 transitions"
  - "SkillRegistry: simple skill dispatch for VoiceSettings, DJPlaylist, Mapping"
  - "Fixed circular imports in transport/ and memory/ package inits"
affects: ["rob_box_harness.harnesses", "rob_box_harness.core", "rob_box_harness.transport", "rob_box_harness.memory"]
tech-stack:
  added: []
  patterns:
    - "Parallel implementation: DialogHarness coexists with dialogue_node.py, switchable via config"
    - "Pure state machine: no I/O, no ROS2, no LLM — testable with plain pytest"
    - "Skill registry pattern: simple dict-based skill dispatch for harness-level tools"

key-files:
  created:
    - "src/rob_box_harness/rob_box_harness/harnesses/dialog.py"
    - "src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py"
    - "src/rob_box_harness/rob_box_harness/core/__init__.py"
  modified:
    - "src/rob_box_harness/rob_box_harness/harnesses/__init__.py"
    - "src/rob_box_harness/rob_box_harness/transport/__init__.py"
    - "src/rob_box_harness/rob_box_harness/memory/__init__.py"

key-decisions:
  - "DialogState lives in core.dialogue_state_machine, re-exported from harnesses.dialog for convenience"
  - "SkillRegistry uses a minimal Skill ABC rather than the full ToolProvider contract — dialog skills are few and stable"
  - "DummyLLMProvider provides a safe fallback when no real LLM provider is configured"
  - "Circular imports fixed by loading transport.py/memory.py modules via file-path importlib in package inits"

patterns-established:
  - "Harness adapter pattern: init() calls super().init() then overrides specific ports"
  - "Step() pattern: classify input → state machine → LLM turn → side-effects → save → return"
  - "Lazy package init: __getattr__ for heavy imports (ROS2Transport, SQLiteVoiceMemory)"

requirements-completed: ["HARN-DIALOG-08", "HARN-DSM-09"]

duration: 30min
completed: 2026-07-27
---

# Plan 06-05: DialogHarness + DialogueStateMachine

**Created the DialogHarness adapter and pure DialogueStateMachine — parallel implementation coexisting with existing dialogue_node.py.**

## Performance

- **Duration:** ~30 min
- **Started:** 2026-07-27
- **Completed:** 2026-07-27
- **Tasks:** 2
- **Files created:** 3, **Files modified:** 3

## Accomplishments
- Created `DialogueStateMachine` — pure state machine handling all 8 transitions (IDLE↔LISTENING↔DIALOGUE↔SILENCED) with zero I/O dependencies
- Created `DialogHarness` — `Harness[DialogState]` wrapping the full voice pipeline: wake-word classification → state transition → LLM turn → TTS side-effect → memory save
- Created `SkillRegistry` with 3 stub skills (VoiceSettings, DJPlaylist, Mapping)
- Fixed pre-existing circular imports in `transport/` and `memory/` package inits (discovered during verification)

## Task Commits

1. **Task W9: DialogueStateMachine** — `51a7529c` (feat(06-05): add DialogueStateMachine)
2. **Task W8: DialogHarness** — `8264c230` (feat(06-05): add DialogHarness)
3. **Circular import fix** — `f7d77ee7` (fix(06-05): resolve circular imports in transport/ and memory/ package inits)

## Files Created/Modified
- `src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py` — Pure DSM with DTMC-style transitions
- `src/rob_box_harness/rob_box_harness/core/__init__.py` — Core package init with DSM exports
- `src/rob_box_harness/rob_box_harness/harnesses/dialog.py` — DialogHarness + skills + DummyLLMProvider
- `src/rob_box_harness/rob_box_harness/harnesses/__init__.py` — Added DialogHarness export
- `src/rob_box_harness/rob_box_harness/transport/__init__.py` — Fixed circular import via file-path loading
- `src/rob_box_harness/rob_box_harness/memory/__init__.py` — Fixed circular import via file-path loading

## Decisions Made
- **DialogState home**: Defined in `core.dialogue_state_machine` (natural home for the DSM), re-exported from `harnesses.dialog` for import convenience
- **Skill pattern**: Used a minimal `Skill` ABC instead of the full `ToolProvider` contract — dialog skills are few and stable, not worth the full contract overhead
- **DummyLLMProvider**: Included as a built-in fallback so the harness can pass through its lifecycle without a configured LLM

## Deviations from Plan
- **Circular import fix**: The transport/ and memory/ package inits from Plan 06-04 had eager imports that caused circular dependency chains. Fixed by using file-path `importlib` loading for the base modules and `__getattr__` lazy loading for the heavy implementations.

## Issues Encountered
- Circular imports in `transport/__init__.py` and `memory/__init__.py` blocked all module imports. Diagnosed as a naming collision between `transport.py`/`memory.py` (module files) and `transport/`/`memory/` (package dirs) introduced in Plan 06-04. Fixed with importlib file-path loading.

## Next Phase Readiness
- Plan 06-06 (PersistentHarness + TelegramHarness) can proceed — DialogHarness pattern established
- dialogue_node.py is NOT modified — parallel implementation verified

---
*Plan: 06-05*
*Completed: 2026-07-27*
