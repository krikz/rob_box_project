---
phase: 06-harness-p0-finalization
reviewed: 2026-07-28T12:00:00Z
depth: standard
files_reviewed: 22
files_reviewed_list:
  - src/rob_box_harness/rob_box_harness/providers/deepseek.py
  - src/rob_box_harness/rob_box_harness/providers/mimo.py
  - src/rob_box_harness/rob_box_harness/core/tool_registry.py
  - src/rob_box_harness/rob_box_harness/core/dialog_core.py
  - src/rob_box_harness/rob_box_harness/providers/minimax.py
  - src/rob_box_harness/rob_box_harness/providers/__init__.py
  - src/rob_box_harness/rob_box_harness/__init__.py
  - src/rob_box_harness/rob_box_harness/memory.py
  - src/rob_box_harness/rob_box_harness/memory/sqlite_voice.py
  - src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py
  - src/rob_box_voice/test/test_dialogue_shell.py
  - src/rob_box_voice/rob_box_voice/dialogue_node.py
  - src/rob_box_telegram/test/test_telegram_bridge.py
  - src/rob_box_telegram/rob_box_telegram/telegram_node.py
  - src/rob_box_telegram/rob_box_telegram/handlers/commands.py
  - src/rob_box_telegram/rob_box_telegram/handlers/messages.py
  - src/rob_box_telegram/rob_box_telegram/handlers/callbacks.py
  - src/rob_box_telegram/rob_box_telegram/voice_processor.py
  - src/rob_box_perception/rob_box_perception/perception_bridge.py
  - src/rob_box_perception/test/test_perception_bridge.py
  - src/rob_box_perception/rob_box_perception/context_aggregator_node.py
  - src/rob_box_perception/rob_box_perception/utils/node_monitor.py
findings:
  critical: 0
  warning: 0
  info: 1
  total: 1
status: clean
---

# Phase 6: Re-Review — Fix Verification Report

**Reviewed:** 2026-07-28
**Depth:** standard
**Files Reviewed:** 22
**Status:** clean — all 5 BLOCKERs verified fixed, no regressions

## Summary

Re-review of the same 22 source files after the gsd-code-fixer applied fixes for 5 of 6 BLOCKER findings (CR-03 was rejected as a false positive). All 5 BLOCKER fixes are verified as correctly applied and functional. No new bugs, broken indentation, unused imports, comment artifacts, or variable shadowing were introduced by the fixes. One minor INFO-level observation about orphaned ROS parameters is noted below but does not affect correctness.

---

## Fix Verification

### CR-01: `dialog_core.py` error-recovery branch — **verified_fixed**

**Commit:** `82632293`
**File:** `src/rob_box_harness/rob_box_harness/core/dialog_core.py:208-215`

The error-recovery branch no longer references the non-existent `self._memory.turns` attribute. Instead it unconditionally calls:

```python
await self._memory.append_turn(
    self._user_id, Turn(role="user", content=text)
)
```

`MemoryStore.append_turn` is documented idempotent; `SQLiteVoiceMemory` enforces a 5-second dedup window. The user turn is now correctly persisted on LLM error. No `AttributeError` risk.

✅ **Verified.**

---

### CR-02: `InMemoryStore.init()` no-op method — **verified_fixed**

**Commit:** `d2b063bd`
**File:** `src/rob_box_harness/rob_box_harness/memory.py:253-263`

`InMemoryStore` now exposes:

```python
async def init(self) -> None:
    """No-op for in-memory store. …"""
    return None
```

Matching the pattern of `SQLiteVoiceMemory.init()`. The shell's `_build_memory` fallback path (`store.init()`) no longer raises `AttributeError`.

✅ **Verified.**

---

### CR-03: DeepSeek/MiMo `chat()` method — **false_positive (unchanged)**

**File:** `src/rob_box_harness/rob_box_harness/providers/deepseek.py:300-320`

`HarnessDeepSeekProvider.chat()` exists at line ~300 with signature:

```python
async def chat(
    self,
    messages: Iterable[LLMMessage],
    *,
    temperature: float | None = None,
    max_tokens: int | None = None,
    top_p: float | None = None,
    tools: Iterable[Mapping[str, Any]] = (),
) -> LLMResponse:
```

`HarnessMiMoProvider(HarnessDeepSeekProvider)` inherits it. Both providers have `chat()`. The original reviewer's grep missed it because the method is defined inside `HarnessDeepSeekProvider`, not as a standalone function.

✅ **Confirmed false positive — no fix needed.**

---

### CR-04: `dialogue_node._build_llm` uses factory — **verified_fixed**

**Commit:** `07c024f3`
**File:** `src/rob_box_voice/rob_box_voice/dialogue_node.py:39, 179-185`

The shell now imports and uses the factory:

```python
from rob_box_harness.providers import build_deepseek_provider

def _build_llm(self) -> Any:
    return build_deepseek_provider(
        api_key=self.get_parameter("api_key").value or None,
        base_url=self.get_parameter("base_url").value or None,
        model=self.get_parameter("model").value or None,
    )
```

No direct `DeepSeekProvider` instantiation. The old import `from rob_box_harness.providers import DeepSeekProvider` has been correctly removed. `build_deepseek_provider` is re-exported from `rob_box_harness.providers.__init__` ✅.

✅ **Verified.**

---

### CR-05: `SQLiteVoiceMemory.append_turn` return type — **verified_fixed**

**Commit:** `28d0a221`
**File:** `src/rob_box_harness/rob_box_harness/memory/sqlite_voice.py:196`

Signature now matches the `MemoryStore` ABC:

```python
async def append_turn(self, scope: str, turn: Turn) -> None:
```

Previously returned `-> bool`. Duplicate turns are silently skipped (debug-logged). All callers in `DialogCore.process_input` already ignore the return value via `await self._memory.append_turn(...)` — no behavioral change.

✅ **Verified.**

---

## New Findings

### IN-01: ROS parameters `temperature` and `max_tokens` declared but not threaded to LLM calls

**File:** `src/rob_box_voice/rob_box_voice/dialogue_node.py:139-140` (declare), `dialog_core.py:195` (call site)

**Issue:** The shell declares `temperature` (0.7) and `max_tokens` (500) as ROS parameters, and the original `_build_llm` (pre-fix) attempted to pass them to the provider constructor — which would have raised `TypeError` because `HarnessDeepSeekProvider.__init__` does not accept these kwargs. The CR-04 fix correctly removed them from the constructor call, but neither the factory nor `DialogCore.process_input` threads them into LLM calls:

```python
# DialogCore.process_input line ~195
response = await self._llm.complete(messages)
#                                     ^^ no settings= passed
```

The upstream `DeepSeekProvider.complete()` uses its own model defaults. The ROS-configured `temperature=0.7` and `max_tokens=500` are **declared but never applied** to any LLM call. The `chat()` shortcut on `HarnessDeepSeekProvider` does accept `temperature=` and `max_tokens=` kwargs, but `DialogCore` calls `complete()`, not `chat()`.

**Fix:** Either:
- Pass `LLMSettings(temperature=..., max_tokens=...)` to `DialogCore` at construction and forward to `complete()`, or
- Switch `DialogCore` to call `llm.chat()` with temperature/max_tokens from a config object, or
- Remove the orphan ROS params if the upstream defaults are intentionally preferred.

**Severity:** Info — the upstream model defaults are reasonable; this is a configuration drift, not a correctness bug. The LLM still produces valid responses.

---

## Regression Check

| Check | Result |
|-------|--------|
| Unused imports introduced by fix | ✅ None — old `DeepSeekProvider` import removed; `build_deepseek_provider` is used |
| Broken indentation | ✅ None — all indentation consistent |
| Comment artifacts | ✅ None — no leftover `# FIXME` or dead comments from fix |
| Variable shadowing | ✅ None — `store` reassignment in `_build_memory` is intentional and type-safe |
| micro-ROS cleanup | ✅ Confirmed — CR-06 grep returned zero operational references per fix report |

---

_Reviewed: 2026-07-28T12:00:00Z_
_Reviewer: the agent (gsd-code-reviewer)_
_Depth: standard_
