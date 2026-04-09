# FoxDot Runtime Repair Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Repair the Vision Pi FoxDot/Renardo runtime so plugin-dependent synths load correctly, broken SynthDef files no longer abort startup, and the active prompts match the real SuperCollider capabilities.

**Architecture:** Fix the problem in three layers: install `sc3-plugins` in both SuperCollider-related images, patch broken `renardo_lib` `.scd` files during image build, then add startup validation and prompt alignment on top of the repaired runtime.

**Tech Stack:** Docker, Ubuntu packages, SuperCollider, SC3 Plugins, ROS 2, Python, Bash, prompt text files

---

### Task 1: Add failing tests for synth patching and startup validation

**Files:**
- Create: `src/rob_box_voice/test/unit/core/test_music_stack_validation.py`
- Test: `src/rob_box_voice/test/unit/core/test_music_stack_validation.py`

**Step 1: Write the failing test**

Add tests for:
- detecting merge-conflict markers in `.scd` content
- detecting plugin-dependent UGens such as `MoogVCF`
- parsing `sclang` log text into healthy / degraded status

**Step 2: Run test to verify it fails**

Run: `pytest src/rob_box_voice/test/unit/core/test_music_stack_validation.py -v`
Expected: FAIL because the validation helpers do not exist yet

**Step 3: Write minimal implementation**

Create a small pure-Python helper module for music-stack validation logic.

**Step 4: Run test to verify it passes**

Run: `pytest src/rob_box_voice/test/unit/core/test_music_stack_validation.py -v`
Expected: PASS

### Task 2: Generalize the Renardo patch layer

**Files:**
- Create: `docker/vision/voice_assistant/patch_renardo_synthdefs.py`
- Modify: `docker/vision/voice_assistant/Dockerfile`
- Delete or replace usage of: `docker/vision/voice_assistant/fix_brass_scd.py`

**Step 1: Write the failing test**

Add tests that feed broken `.scd` text into the patch helper and assert repaired output for:
- `brass.scd`
- `organ.scd`

**Step 2: Run test to verify it fails**

Run: `pytest src/rob_box_voice/test/unit/core/test_music_stack_validation.py -v`
Expected: FAIL because patch functions are missing

**Step 3: Write minimal implementation**

Implement a build-time patch script that:
- locates `renardo_lib/SynthDefManagement/sclang_code/scsynth`
- rewrites known-bad files deterministically
- prints which patches were applied

Update the voice-assistant Dockerfile to call the new patch script during build.

**Step 4: Run test to verify it passes**

Run: `pytest src/rob_box_voice/test/unit/core/test_music_stack_validation.py -v`
Expected: PASS

### Task 3: Install SC3 plugins in both runtime images

**Files:**
- Modify: `docker/vision/voice_base/Dockerfile`
- Modify: `docker/vision/supercollider/Dockerfile`

**Step 1: Write the failing test**

Add a test that classifies `wobblebass` as plugin-dependent due to `MoogVCF` and asserts the runtime requirements are surfaced by the validation helper.

**Step 2: Run test to verify it fails**

Run: `pytest src/rob_box_voice/test/unit/core/test_music_stack_validation.py -v`
Expected: FAIL if plugin-dependency classification is not implemented yet

**Step 3: Write minimal implementation**

Install `sc3-plugins` packages in both images and keep package installation comments explicit about why both sides need the same extensions.

**Step 4: Run test to verify it passes**

Run: `pytest src/rob_box_voice/test/unit/core/test_music_stack_validation.py -v`
Expected: PASS

### Task 4: Add startup health validation for the FoxDot stack

**Files:**
- Create: `docker/vision/scripts/voice_assistant/validate_music_stack.py`
- Modify: `docker/vision/scripts/voice_assistant/start_voice_assistant.sh`

**Step 1: Write the failing test**

Add tests for validation outcomes such as:
- healthy log with registered OSCdef and no syntax errors
- degraded log with conflict-marker syntax errors
- degraded log with `SynthDef ... not found`

**Step 2: Run test to verify it fails**

Run: `pytest src/rob_box_voice/test/unit/core/test_music_stack_validation.py -v`
Expected: FAIL because the validation command or parser is incomplete

**Step 3: Write minimal implementation**

Implement a small validation script that reads `/tmp/sclang.log`, checks for fatal patterns, and verifies critical synth names. Invoke it from the startup script after `sclang` is launched.

**Step 4: Run test to verify it passes**

Run: `pytest src/rob_box_voice/test/unit/core/test_music_stack_validation.py -v`
Expected: PASS

### Task 5: Align active prompts with the repaired runtime

**Files:**
- Modify: `src/rob_box_voice/prompts/skills/music_skill_prompt.txt`
- Modify: `src/rob_box_voice/prompts/master_prompt_compact.txt`

**Step 1: Write the failing test**

Add assertions in a prompt-focused test or helper that the advertised synth palette does not contain names banned by the runtime validator.

**Step 2: Run test to verify it fails**

Run: `pytest src/rob_box_voice/test/unit/core/test_music_stack_validation.py -v`
Expected: FAIL because prompt/runtime consistency checks are not implemented yet

**Step 3: Write minimal implementation**

Update prompt text so the recommended synth palette matches the validated runtime and documents the fallback behavior for degraded startup.

**Step 4: Run test to verify it passes**

Run: `pytest src/rob_box_voice/test/unit/core/test_music_stack_validation.py -v`
Expected: PASS

### Task 6: Verify end-to-end with fresh evidence

**Files:**
- Validate modified files and runtime commands

**Step 1: Run focused unit tests**

Run: `pytest src/rob_box_voice/test/unit/core/test_music_stack_validation.py -v`
Expected: PASS

**Step 2: Run file error checks**

Run the workspace error checker on all modified Python files and prompt-adjacent files.
Expected: no new errors in changed files

**Step 3: Build or inspect runtime images**

Run the appropriate Docker build / startup verification commands for the music stack.
Expected:
- `sc3-plugins` available in both images
- `sclang` starts without syntax errors from broken `.scd`
- critical synth allowlist validates successfully

**Step 4: Summarize evidence**

Capture the exact verification outputs used to justify completion.