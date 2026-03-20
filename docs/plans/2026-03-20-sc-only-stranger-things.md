# SC-Only Stranger Things Expansion Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add SC-only custom synth support so Rob Box can generate a closer Stranger Things-style arrangement without introducing a second audio engine.

**Architecture:** Keep `scsynth` as the only synthesis engine, preload repo-owned custom `.scd` SynthDefs during `sclang` startup, and register matching Python-side wrappers inside the Renardo runtime context.

**Tech Stack:** SuperCollider, Renardo/FoxDot, Python, Docker, prompt text files

---

### Task 1: Lock startup contract with failing tests

**Files:**
- Modify: `src/rob_box_voice/test/unit/core/test_music_runtime_assets.py`
- Create: `src/rob_box_voice/test/unit/core/test_sc_only_custom_synthdefs.py`

**Steps:**
1. Add tests asserting custom synthdef files exist for `warmpad`, `retrobass`, and `supersawlead`.
2. Add tests asserting `foxdot_init.sc` preloads those synthdefs.
3. Add tests asserting `start_voice_assistant.sh` validates those synthdefs at startup.
4. Add tests for Python-side registration helper behavior with a fake Renardo runtime.

### Task 2: Add SC-only custom synthdefs and preload wiring

**Files:**
- Create: `docker/vision/voice_assistant/custom_synthdefs/warmpad.scd`
- Create: `docker/vision/voice_assistant/custom_synthdefs/retrobass.scd`
- Create: `docker/vision/voice_assistant/custom_synthdefs/supersawlead.scd`
- Modify: `docker/vision/voice_assistant/Dockerfile`
- Modify: `docker/vision/voice_assistant/foxdot_init.sc`
- Modify: `docker/vision/scripts/voice_assistant/start_voice_assistant.sh`

**Steps:**
1. Add repository-owned `.scd` sources for the three synth roles.
2. Copy them into the voice-assistant image at build time.
3. Preload them during `sclang` startup.
4. Extend startup validation so missing custom synthdefs degrade the runtime explicitly.

### Task 3: Register custom synths in Renardo runtime

**Files:**
- Create: `src/rob_box_voice/rob_box_voice/core/sc_only_custom_synthdefs.py`
- Modify: `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/music.py`

**Steps:**
1. Add a small helper that registers Python-side wrappers for custom synth names.
2. Call that helper from `MusicManager._initialize_renardo()` after runtime import.
3. Ensure generated code can use `warmpad`, `retrobass`, and `supersawlead` as direct FoxDot synth names.

### Task 4: Update music prompting for the new palette

**Files:**
- Modify: `src/rob_box_voice/prompts/master_prompt_compact.txt`
- Modify: `src/rob_box_voice/prompts/skills/music_skill_prompt.txt`

**Steps:**
1. Add the new synth names to the safe palette sections.
2. Replace the current weak Stranger Things recipe with one built around the new SC-only synths.
3. Prefer `warmpad + retrobass + supersawlead` over `space + dub` for retro analog horror references.

### Task 5: Verify

**Files:**
- Validate changed files for errors

**Steps:**
1. Run focused unit tests for runtime assets and custom synth registration.
2. Run error checks on the modified Python files.
3. Summarize which parts are now SC-only approximations and which parts still remain approximate versus the original Strudel patch.