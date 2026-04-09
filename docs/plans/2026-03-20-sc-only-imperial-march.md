# SC-Only Imperial March Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add a stronger SC-only Imperial March palette and prompt guidance so the robot can render a more recognizable brass-and-low-strings march inside the existing Renardo runtime.

**Architecture:** Extend the existing repository-owned custom synthdef pipeline with two new synthdefs, preload and validate them during startup, expose Python-side wrappers in the Renardo runtime, and add prompt guidance plus regression tests that lock the contract. The actual Imperial March motif remains prompt-driven, but the runtime gains the timbral tools required to produce a better reduction.

**Tech Stack:** SuperCollider SynthDefs, Renardo/FoxDot runtime wrappers, Python unit tests, prompt text assets, shell startup validation.

---

### Task 1: Lock the new Imperial March runtime contract with tests

**Files:**
- Modify: `src/rob_box_voice/test/unit/core/test_sc_only_custom_synthdefs.py`
- Modify: `src/rob_box_voice/test/unit/core/test_music_runtime_assets.py`

**Step 1: Write the failing test**

Add assertions that:

```python
assert "imperialbrass" in CUSTOM_SC_ONLY_SYNTH_NAMES
assert "marchstrings" in CUSTOM_SC_ONLY_SYNTH_NAMES
```

and:

```python
for synth_name in ("warmpad", "retrobass", "supersawlead", "imperialbrass", "marchstrings"):
    synth_path = CUSTOM_SYNTHDEF_DIR / f"{synth_name}.scd"
    assert synth_path.exists()
```

Add prompt assertions that Imperial March / Star Wars guidance references `imperialbrass` and `marchstrings`.

**Step 2: Run test to verify it fails**

Run:

```bash
PYTHONPATH=src/rob_box_voice:src/rob_box_mcp_tools /home/builder/rob_box_project/.venv/bin/python -m pytest \
  src/rob_box_voice/test/unit/core/test_sc_only_custom_synthdefs.py \
  src/rob_box_voice/test/unit/core/test_music_runtime_assets.py -q
```

Expected: FAIL because the new synth names, files, and prompt guidance do not exist yet.

**Step 3: Write minimal implementation**

Do not implement production code yet beyond the minimum needed for the next task.

**Step 4: Run test to verify it still documents the missing behavior**

Re-run the same command and confirm the failure points are still the new Imperial March expectations.

**Step 5: Commit**

```bash
git add src/rob_box_voice/test/unit/core/test_sc_only_custom_synthdefs.py src/rob_box_voice/test/unit/core/test_music_runtime_assets.py
git commit -m "test(voice): lock imperial march runtime contract"
```

### Task 2: Add Imperial March custom synthdefs and runtime registration

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/core/sc_only_custom_synthdefs.py`
- Create: `docker/vision/voice_assistant/custom_synthdefs/imperialbrass.scd`
- Create: `docker/vision/voice_assistant/custom_synthdefs/marchstrings.scd`
- Modify: `docker/vision/voice_assistant/foxdot_init.sc`
- Modify: `docker/vision/scripts/voice_assistant/start_voice_assistant.sh`

**Step 1: Write the failing test**

Use the failures from Task 1 as the RED state.

**Step 2: Run test to verify it fails**

Run the same focused pytest command and confirm it fails for the missing synthdefs and preload contract.

**Step 3: Write minimal implementation**

Implement:

```python
CUSTOM_SC_ONLY_SYNTH_NAMES = (
    "warmpad",
    "retrobass",
    "supersawlead",
    "imperialbrass",
    "marchstrings",
)
```

Add matching `.scd` files and preload / startup validation entries.

**Step 4: Run test to verify it passes**

Run:

```bash
PYTHONPATH=src/rob_box_voice:src/rob_box_mcp_tools /home/builder/rob_box_project/.venv/bin/python -m pytest \
  src/rob_box_voice/test/unit/core/test_sc_only_custom_synthdefs.py \
  src/rob_box_voice/test/unit/core/test_music_runtime_assets.py -q
```

Expected: the synth registration and startup asset assertions pass, except any remaining prompt assertions from Task 3.

**Step 5: Commit**

```bash
git add src/rob_box_voice/rob_box_voice/core/sc_only_custom_synthdefs.py \
  docker/vision/voice_assistant/custom_synthdefs/imperialbrass.scd \
  docker/vision/voice_assistant/custom_synthdefs/marchstrings.scd \
  docker/vision/voice_assistant/foxdot_init.sc \
  docker/vision/scripts/voice_assistant/start_voice_assistant.sh
git commit -m "feat(voice): add imperial march sc-only synthdefs"
```

### Task 3: Teach prompts to use the new Star Wars march palette

**Files:**
- Modify: `src/rob_box_voice/prompts/master_prompt_compact.txt`
- Modify: `src/rob_box_voice/prompts/skills/music_skill_prompt.txt`

**Step 1: Write the failing test**

Use the remaining prompt failures from Task 1 as the RED state.

**Step 2: Run test to verify it fails**

Run the same focused pytest command and confirm failures mention missing `imperialbrass` / `marchstrings` prompt guidance.

**Step 3: Write minimal implementation**

Add guidance equivalent to:

```text
For Imperial March / Star Wars march style, prefer imperialbrass + marchstrings + strings or organ.
Use exact midinote motif fragments over random PRand melodies.
Keep drums sparse and martial, not club-oriented.
```

**Step 4: Run test to verify it passes**

Run the focused pytest command and confirm all targeted tests are green.

**Step 5: Commit**

```bash
git add src/rob_box_voice/prompts/master_prompt_compact.txt src/rob_box_voice/prompts/skills/music_skill_prompt.txt
git commit -m "feat(prompt): add imperial march guidance"
```

### Task 4: Verify the targeted feature end-to-end

**Files:**
- Modify: `docs/plans/2026-03-20-sc-only-imperial-march-design.md`
- Modify: `docs/plans/2026-03-20-sc-only-imperial-march.md`

**Step 1: Run targeted tests**

Run:

```bash
PYTHONPATH=src/rob_box_voice:src/rob_box_mcp_tools /home/builder/rob_box_project/.venv/bin/python -m pytest \
  src/rob_box_voice/test/unit/core/test_sc_only_custom_synthdefs.py \
  src/rob_box_voice/test/unit/core/test_music_runtime_assets.py -q
```

Expected: PASS.

**Step 2: Run editor diagnostics**

Check modified Python files for errors.

**Step 3: Inspect git diff**

Confirm the change stays scoped to the Imperial March palette, runtime registration, startup preload, and prompt guidance.

**Step 4: Commit verification/docs updates if needed**

```bash
git add docs/plans/2026-03-20-sc-only-imperial-march-design.md docs/plans/2026-03-20-sc-only-imperial-march.md
git commit -m "docs(voice): add imperial march design and plan"
```

**Step 5: Report outcome**

Summarize:

1. what changed
2. what was verified
3. what still remains before live robot audition