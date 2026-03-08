# Music Prompt Unification Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Unify active and legacy music prompts so they match the real tool architecture and no longer contain conflicting DJ, vocal, or synth rules.

**Architecture:** Keep `music_skill_prompt.txt` as the music-generation source of truth, move orchestration rules to the compositor layer, and align flat-mode prompts/tools with the direct-tool runtime.

**Tech Stack:** Python, ROS 2, OpenAI Agents SDK, prompt text files

---

### Task 1: Align music skill prompt with real tool capabilities

**Files:**
- Modify: `src/rob_box_voice/prompts/skills/music_skill_prompt.txt`

**Steps:**
1. Remove instructions that require unavailable speech tools inside `MusicSkill`.
2. Normalize DJ startup, vocal, and pattern-count rules.
3. Keep `execute_music_code()` and `set_dj_mode()` as the only DJ-control actions in the music skill.

### Task 2: Fix active compositor orchestration

**Files:**
- Modify: `src/rob_box_voice/prompts/compositor_prompt.txt`
- Modify: `src/rob_box_voice/rob_box_voice/dialogue_node.py`

**Steps:**
1. Add explicit skills-mode DJ orchestration rules to the compositor prompt.
2. Clarify inline DJ prompt wording so `set_dj_mode()` is described as a separate tool call, not Renardo code.
3. Keep speech in the compositor and music in `handle_music`.

### Task 3: Align flat-mode prompts and tools

**Files:**
- Modify: `src/rob_box_voice/prompts/master_prompt_compact.txt`
- Modify: `src/rob_box_voice/prompts/master_prompt_simple.txt`
- Modify: `src/rob_box_voice/rob_box_voice/dialogue_node.py`
- Modify: `src/rob_box_voice/rob_box_voice/skills/music_skill.py`

**Steps:**
1. Remove skills-mode-only references from `master_prompt_compact.txt`.
2. Mark `master_prompt_simple.txt` music handling as TTS-only demo behavior.
3. Expose `set_dj_mode()` in flat mode.
4. Fix misleading `search_samples()` documentation.

### Task 4: Verify

**Files:**
- Validate changed files for errors

**Steps:**
1. Run fresh error checks on modified files.
2. Review results and fix any issues.
3. Summarize final consistency changes with evidence.
