# FoxDot Runtime Repair Design

**Date:** 2026-03-20

## Goal

Stabilize the current FoxDot/Renardo music runtime on Vision Pi so plugin-dependent synths such as `wobblebass` can load, core synths such as `strings` stop disappearing during startup, and the DJ/music prompts stay aligned with the real runtime.

## Problem Summary

The current runtime has two distinct failure modes:

1. Plugin-dependent synths fail because `sc3-plugins` are not installed in the containers that run `sclang` and `scsynth`.
2. Core synths fail intermittently because the installed `renardo_lib` package contains broken `.scd` files with unresolved merge-conflict markers, which abort part of the SynthDef loading pipeline.

This leads to misleading success signals in the voice/music orchestration layer: `execute_music_code` reports success while SuperCollider silently drops one or more synth layers.

## Design

### 1. Normalize the SuperCollider runtime across both containers

Install `sc3-plugins` in both:

- `docker/vision/voice_base/Dockerfile` because `sclang` runs inside `voice-assistant`
- `docker/vision/supercollider/Dockerfile` because `scsynth` runs inside `supercollider`

The runtime must be symmetric. If `sclang` knows plugin UGens but `scsynth` does not, SynthDef compilation may succeed while playback still fails at runtime.

### 2. Replace one-off synth patches with an explicit Renardo patch layer

The existing `fix_brass_scd.py` proves the pip package already ships broken SynthDef files. Extend this pattern into a small patch set that repairs known-bad `.scd` files in `renardo_lib` during image build.

Initial patch targets:

- `brass.scd` (already patched today)
- `organ.scd` (confirmed merge-conflict markers)

If additional broken files are discovered while validating startup, patch them in the same build-time layer rather than mutating files manually on the robot.

### 3. Add startup validation for the music stack

After `sclang` starts in `docker/vision/scripts/voice_assistant/start_voice_assistant.sh`, validate the runtime before the voice nodes take traffic.

Validation should answer:

- Did `sclang` register the FoxDot OSC handler?
- Did `sclang` emit syntax errors while loading SynthDefs?
- Are critical synths available: `strings`, `wobblebass`, `cs80lead`, `space`, `pads`, `dub`?

The result should be explicit in logs so degraded music mode is diagnosable immediately.

### 4. Keep prompts aligned with the validated runtime

The prompt layer currently encourages synth names that may not exist in the live runtime. After the runtime is repaired, update the active prompts to keep the synth palette truthful.

Files to align:

- `src/rob_box_voice/prompts/skills/music_skill_prompt.txt`
- `src/rob_box_voice/prompts/master_prompt_compact.txt`

If validation indicates a degraded startup, prompts should prefer a conservative synth palette and avoid known-broken names.

### 5. Verify with focused tests and smoke checks

Use lightweight tests for the patch/validation logic and targeted verification commands for the Docker/runtime path. The main confidence signal is not a generic unit test but fresh evidence that:

- both images contain plugin support
- `sclang` starts cleanly
- the critical synth allowlist loads without `SynthDef ... not found`

## Expected Outcome

- `wobblebass` loads because `MoogVCF` and other extension UGens are available
- `strings` no longer disappears due to partial startup failure
- broken Renardo `.scd` files stop poisoning the load sequence
- runtime logs show a clear healthy/degraded music-stack state
- prompts no longer instruct the model to use synths that the runtime cannot support