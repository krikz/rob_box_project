# DialogueNode Integration Plan

## Current State Analysis

**File**: `dialogue_node.py` (2313 LOC)

### Identified Logic to Extract

#### 1. State Machine (Lines 188-194, 636-660, 2007-2017)
```python
# Current implementation
self.state = "IDLE"  # IDLE | LISTENING | DIALOGUE | SILENCED
self.silence_until = None
self.last_interaction_time = time.time()
self.dialogue_timeout = 30.0
```

**Replacement**: Use `DialogueManager` from core

#### 2. Wake Word Detection (Lines 406-417)
```python
def _has_wake_word(self, text: str) -> bool
def _remove_wake_word(self, text: str) -> str
```

**Replacement**: Use `DialogueManager.is_wake_word()` and `DialogueManager.remove_wake_word()`

#### 3. Silence Mode (Lines 419-474)
```python
def _is_silence_command(self, text: str) -> bool
def _is_unsilence_command(self, text: str) -> bool
def _handle_silence_command(self)
```

**Replacement**: Use `DialogueManager.is_silence_command()`, `DialogueManager.enable_silence()`

#### 4. Query Accumulation (Lines 217-225, 774-808)
```python
self.pending_queries = []
self.query_accumulation_timeout = 2.5
self.last_query_time = None
self.accumulation_timer = None
```

**Replacement**: Use `DialogueManager.add_query()`, `DialogueManager.get_accumulated_queries()`

### Integration Strategy

1. **Import DialogueManager**
   ```python
   from rob_box_voice.core.dialogue_manager import DialogueManager, DialogueState
   ```

2. **Replace initialization** (Lines 188-225)
   - Remove manual state variables
   - Create `self.dialogue_manager = DialogueManager(...)`

3. **Replace wake word methods** (Lines 406-417)
   - Remove `_has_wake_word` and `_remove_wake_word`
   - Use `self.dialogue_manager.is_wake_word()` and `remove_wake_word()`

4. **Replace silence methods** (Lines 419-474)
   - Remove `_is_silence_command`, `_is_unsilence_command`, `_handle_silence_command`
   - Use DialogueManager methods

5. **Replace state checks** (Throughout file)
   - Replace `self.state == "IDLE"` with `self.dialogue_manager.state == DialogueState.IDLE`
   - Replace state transitions with `self.dialogue_manager.transition_to()`

6. **Replace query accumulation** (Lines 774-808)
   - Remove manual query queue logic
   - Use DialogueManager query methods

### Expected LOC Reduction

| Section | Current LOC | After | Reduction |
|---------|------------|-------|-----------|
| State machine init | ~30 | ~5 | -25 |
| Wake word methods | ~12 | 0 | -12 |
| Silence methods | ~60 | 0 | -60 |
| Query accumulation | ~80 | ~20 | -60 |
| State checks/transitions | ~150 | ~50 | -100 |
| **Total** | **~332** | **~75** | **~257** |

**Estimated**: dialogue_node.py will reduce from 2313 to ~2056 LOC (-11%)

### Implementation Steps

1. ✅ Add DialogueManager import
2. ✅ Replace initialization
3. ✅ Replace wake word logic
4. ✅ Replace silence logic  
5. ✅ Replace state transitions
6. ✅ Replace query accumulation
7. ✅ Test integration
8. ✅ Commit changes

### Benefits

- ✅ Single source of truth for dialogue state
- ✅ Tested state machine (25+ tests)
- ✅ Cleaner, more maintainable code
- ✅ Easier to add new states/features
- ✅ ~257 lines removed from dialogue_node.py

### Risks

- ⚠️ Complex integration (largest file)
- ⚠️ Many state dependencies
- ⚠️ Need careful testing

### Mitigation

- Step-by-step integration
- Keep ROS callback structure intact
- Preserve all existing functionality
- Test after each change

## Status

**Created**: 2026-01-20
**Status**: Ready for implementation
**Next**: Begin integration
