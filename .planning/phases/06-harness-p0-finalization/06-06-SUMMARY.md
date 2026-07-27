---
phase: 06-harness-p0-finalization
plan: 06
subsystem: voice
tags: [persistent, telegram, harness, hardware-lifecycle, command-registry, bot]
requires:
  - phase: 06
    provides: "Plan 06-04 provided ROS2Transport and SQLiteVoiceMemory; Plan 06-05 provided DialogHarness pattern reference"
provides:
  - "PersistentHarness: lightweight harness unifying 6 persistent nodes (audio/stt/tts/sound/led/cmd) under shared lifecycle"
  - "HardwareLifecycle: connect/disconnect/health-check/restart-on-error for hardware devices"
  - "StatePublisher: unified status on /<node>/state topic"
  - "ParameterGuard: declare/validate/reload ROS parameters with type checking"
  - "TelegramHarness: harness wrapping Telegram bot logic with command registry, auth middleware, and snapshot store"
  - "TelegramCommandRegistry: declarative command → handler dispatch for 25 commands"
  - "AuthMiddleware: user-level access control at dispatcher level"
  - "SnapshotStore: ephemeral camera snapshot cache with TTL expiration"
affects: ["rob_box_harness.harnesses"]
tech-stack:
  added: []
  patterns:
    - "Parallel implementation: both harnesses coexist with existing nodes, switchable via config"
    - "HardwareLifecycle pattern: abstract connect/disconnect/health-check/restart — device-specific code stays in nodes"
    - "Command registry pattern: declarative command→handler mapping with type-safe dispatch"
    - "Auth middleware pattern: wrapping handlers for access control at the dispatcher level"

key-files:
  created:
    - "src/rob_box_harness/rob_box_harness/harnesses/persistent.py"
    - "src/rob_box_harness/rob_box_harness/harnesses/telegram.py"
  modified:
    - "src/rob_box_harness/rob_box_harness/harnesses/__init__.py"

key-decisions:
  - "PersistentHarness runs a 1 Hz health-check → state-publish loop (no external input)"
  - "TelegramHarness processes one dict-format update per step() — the outer long-polling loop stays in telegram_node.py"
  - "9 placeholder command handlers registered at init time; real implementations replace via registry"
  - "AuthMiddleware.open mode when no allowed_users configured (dev-friendly default)"
  - "SnapshotStore is ephemeral (dict-backed, no persistence) — matches camera_cache.py's 5 min TTL behavior"
  - "All existing node files remain untouched — verified by git diff"

patterns-established:
  - "Self-monitoring harness: step() ignores input, runs periodic health-check + state-publish"
  - "Command dispatch harness: step() classifies input as command vs text, routes accordingly"
  - "Placeholder handlers: stub implementations for 9 core commands, ready for real logic in P1"

requirements-completed: ["HARN-PERSIST-10", "HARN-TG-11"]

duration: 20min
completed: 2026-07-27
---

# Plan 06-06: PersistentHarness + TelegramHarness

**Created two production harnesses: PersistentHarness (unifying 6 hardware nodes) and TelegramHarness (wrapping Telegram bot with 9 command handlers, auth, and snapshot store).**

## Performance

- **Duration:** ~20 min
- **Started:** 2026-07-27
- **Completed:** 2026-07-27
- **Tasks:** 2
- **Files created:** 2, **Files modified:** 1

## Accomplishments
- Created `PersistentHarness` with 4 helper classes: `HardwareLifecycle`, `StatePublisher`, `ParameterGuard`, `PersistentState`
- Created `TelegramHarness` with 4 helper classes: `TelegramCommandRegistry` (9 registered handlers), `AuthMiddleware`, `SnapshotStore`, `TelegramState`
- Both harnesses subclass `Harness[StateT]` correctly with proper `init()` → `step()` → `teardown()` lifecycle
- Verified no existing node files were modified

## Task Commits

1. **Task W10: PersistentHarness** — `c6b1dd28` (feat(06-06): add PersistentHarness and TelegramHarness)
2. **Task W11: TelegramHarness** — `c6b1dd28` (same commit — both files committed together)

## Files Created/Modified
- `src/rob_box_harness/rob_box_harness/harnesses/persistent.py` — PersistentHarness + 4 helpers, ~245 lines
- `src/rob_box_harness/rob_box_harness/harnesses/telegram.py` — TelegramHarness + 4 helpers + 9 handlers, ~450 lines
- `src/rob_box_harness/rob_box_harness/harnesses/__init__.py` — Added PersistentHarness, TelegramHarness exports

## Decisions Made
- **PersistentHarness loop**: Runs at 1 Hz internally (health-check → state-publish). No external input needed.
- **Telegram update format**: Dict-based (`chat_id`, `user_id`, `command`, `text`) — keeps the harness testable without python-telegram-bot.
- **Placeholder handlers**: 9 stub command handlers registered (start, help, status, photo, voice, led, sound, navigate, stop). Real logic replaces via `TelegramCommandRegistry.register()`.
- **Auth default**: Open mode when no `allowed_users` configured — dev-friendly, production locks down via config.

## Deviations from Plan
None — plan executed as written.

## Issues Encountered
None.

## Next Phase Readiness
- Wave 2 is now complete (06-02, 06-05, 06-06)
- Wave 3 (06-07: test coverage) can proceed — all harness implementations are in place
- All 5 harness types are now available: Echo, Upper, Dialog, Persistent, Telegram

---
*Plan: 06-06*
*Completed: 2026-07-27*
