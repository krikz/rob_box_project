# SC-Only Stranger Things Expansion Design

**Date:** 2026-03-20

## Goal

Bring the Rob Box music stack closer to the supplied Stranger Things Strudel reference without adding a second audio engine.

## Baseline Findings

1. The current deployment already includes the standard Ubuntu SuperCollider plugin baseline:
   - `sc3-plugins-language` in the voice-assistant image
   - `sc3-plugins-server` in the scsynth image
2. Ubuntu package search does not expose an additional richer official SC plugin family beyond `sc3-plugins*`.
3. The remaining gap is therefore not missing deb packages but missing runtime-usable synth identities that better match the target roles:
   - warm pad
   - retro synth bass
   - supersaw-like lead
4. Existing Rob Box synths already cover other roles well enough for this track:
   - `brass`
   - `flute`
   - `strings`
   - `wobblebass`
   - `pianovel`

## Design

1. Stay SC-only.
   - No `fluidsynth`, no SoundFont orchestration, no second audio engine.
2. Add custom SuperCollider SynthDefs to the voice-assistant image.
   - `warmpad`
   - `retrobass`
   - `supersawlead`
3. Preload these SynthDefs during `sclang` startup alongside the repaired Renardo synths.
4. Register Python-side wrappers for these SynthDefs inside the Renardo runtime context so generated code can use direct player syntax such as `p1 >> warmpad(...)`.
5. Update prompts so the music model prefers these synths for the Stranger Things family instead of the current weaker approximation `space + dub`.

## Architecture

### Audio Path

- `dialogue_node.py` / music tools continue generating Renardo/FoxDot code.
- `MusicManager` continues executing that code in the Renardo runtime.
- `sclang` preloads both upstream Renardo SynthDefs and repository-owned custom `.scd` files.
- `scsynth` remains the only synthesis engine.

### Runtime Registration

There are two distinct requirements for a custom synth to be usable:

1. The SC SynthDef source must be loaded into `sclang` and delivered to `scsynth`.
2. The Python runtime context must expose a `SynthDef("name")` wrapper so FoxDot syntax can reference that synth by name.

The design therefore adds:

- repo-owned `.scd` files copied into the image
- a small Python registration helper to inject named wrappers into Renardo runtime state

## Scope

### In Scope

- custom SC-only synthdefs for the three missing roles
- startup preload integration
- runtime registration integration
- tests that lock the preload/registration contract
- prompt updates for Stranger Things and related retro-synth directions

### Out of Scope

- SoundFont / General MIDI support
- additional audio services in Compose
- a full orchestral rewrite of every legacy music preset

## Expected Outcome

After deployment, the music stack should be able to generate a Stranger Things-like core arrangement using:

- `warmpad` for the opening harmonic bed
- `retrobass` for the synth-bass pulse
- `supersawlead` for the main arp/lead line
- existing `brass` / `flute` where appropriate

The result will still be an SC interpretation, not literal GM reproduction, but it should move materially closer to the supplied Strudel reference while preserving the current live-coding architecture.