# SC-Only Stranger Things Expansion Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add a more faithful SC-only Stranger Things palette and arrangement guidance so Rob Box generates a recognisable intro reduction rather than a generic retro-horror groove.

**Architecture:** Keep `scsynth` as the only synthesis engine, extend the repository-owned custom `.scd` palette with Stranger-specific synth roles, preload and register them during startup, and rewrite the prompt contract so the model uses a fixed layered arrangement with deterministic note reuse.

**Tech Stack:** SuperCollider, Renardo/FoxDot, Python, Docker, prompt text files

---

### Task 1: Lock the new Stranger Things contract with failing tests

**Files:**
- Modify: `src/rob_box_voice/test/unit/core/test_music_runtime_assets.py`
- Modify: `src/rob_box_voice/test/unit/core/test_sc_only_custom_synthdefs.py`

**Steps:**
1. Add tests asserting `CUSTOM_SC_ONLY_SYNTH_NAMES` now includes `strangerpulsepad`, `strangerarp`, and `strangerbrass`.
2. Add tests asserting custom synthdef files exist for those names.
3. Add tests asserting `foxdot_init.sc` preloads those synthdefs.
4. Add tests asserting `start_voice_assistant.sh` validates those synthdefs at startup.
5. Add prompt assertions that Stranger Things guidance requires a fixed layered structure, deterministic bass/arp motif reuse, heartbeat kick, and sparse brass/flute accents.

**Run:**

```bash
PYTHONPATH=src/rob_box_voice:src/rob_box_mcp_tools /home/builder/rob_box_project/.venv/bin/python -m pytest \
	src/rob_box_voice/test/unit/core/test_sc_only_custom_synthdefs.py \
	src/rob_box_voice/test/unit/core/test_music_runtime_assets.py -q
```

**Expected:** FAIL because the new synth names, files, startup preload, and prompt guidance do not exist yet.

### Task 2: Add Stranger-specific SC-only custom synthdefs and preload wiring

**Files:**
- Create: `docker/vision/voice_assistant/custom_synthdefs/strangerpulsepad.scd`
- Create: `docker/vision/voice_assistant/custom_synthdefs/strangerarp.scd`
- Create: `docker/vision/voice_assistant/custom_synthdefs/strangerbrass.scd`
- Modify: `docker/vision/voice_assistant/foxdot_init.sc`
- Modify: `docker/vision/scripts/voice_assistant/start_voice_assistant.sh`

**Steps:**
1. Add repository-owned `.scd` sources for the three missing Stranger Things roles.
2. Preload them during `sclang` startup.
3. Extend startup validation so missing Stranger synthdefs degrade the runtime explicitly.

**Run:** re-run the focused pytest command and confirm the asset assertions move to green.

### Task 3: Register Stranger synths in the Renardo runtime

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/core/sc_only_custom_synthdefs.py`

**Steps:**
1. Extend `CUSTOM_SC_ONLY_SYNTH_NAMES` with `strangerpulsepad`, `strangerarp`, and `strangerbrass`.
2. Preserve the current `FileSynthDef` fallback behavior so runtime registration still works in production.
3. Ensure generated code can use the new synth names directly in FoxDot syntax.

**Run:** re-run the focused pytest command and confirm registration assertions pass.

### Task 4: Rewrite Stranger Things prompting around the new arrangement contract

**Files:**
- Modify: `src/rob_box_voice/prompts/master_prompt_compact.txt`
- Modify: `src/rob_box_voice/prompts/skills/music_skill_prompt.txt`

**Steps:**
1. Add the new synth names to the safe palette sections.
2. Replace the current broad Stranger Things guidance with a fixed layered recipe: opening pad, delayed heartbeat, bass ostinato, arp hook, sparse brass/flute accents.
3. Require deterministic note patterns for bass and arp, not `PRand`-driven melodies.
4. Prefer `strangerpulsepad + retrobass + strangerarp + strangerbrass`, with `warmpad` and `supersawlead` as fallbacks.

**Run:** re-run the focused pytest command and confirm prompt assertions pass.

### Task 5: Verify

**Files:**
- Validate changed files for errors

**Steps:**
1. Run focused unit tests for runtime assets and custom synth registration.
2. Run focused MCP music-manager tests to ensure the runtime registration change did not regress music startup.
3. Run error checks on the modified Python files.
4. Summarize which parts are now structurally closer to the Strudel patch and which parts still remain approximate because the stack is SC-only.