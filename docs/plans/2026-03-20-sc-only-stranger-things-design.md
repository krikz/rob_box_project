# SC-Only Stranger Things Expansion Design

**Date:** 2026-03-20

## Goal

Bring the Rob Box music stack materially closer to the supplied Stranger Things Strudel reference without adding a second audio engine, improving both timbre and arrangement structure.

## Baseline Findings

1. The current deployment already includes the standard Ubuntu SuperCollider plugin baseline:
   - `sc3-plugins-language` in the voice-assistant image
   - `sc3-plugins-server` in the scsynth image
2. Ubuntu package search does not expose an additional richer official SC plugin family beyond `sc3-plugins*`.
3. The remaining gap is not missing deb packages but missing runtime-usable synth identities and prompt constraints that match the target musical roles:
   - pulsing warm opening pad
   - stepped retro synth bass
   - narrow animated arp lead
   - softer 80s synth brass accents
4. Existing Rob Box synths only partially cover those roles. `brass` and `flute` work as fallbacks, but they do not consistently steer the model toward the supplied arrangement.

## Design

1. Stay SC-only.
   - No `fluidsynth`, no SoundFont orchestration, no second audio engine.
2. Add three more repository-owned SuperCollider SynthDefs to complete the Stranger Things palette.
   - `strangerpulsepad` for the opening pad bed
   - `strangerarp` for the animated hook layer
   - `strangerbrass` for 80s synth-brass accents and melody support
3. Keep the existing `warmpad`, `retrobass`, and `supersawlead` synths available as fallbacks, but explicitly bias Stranger Things generation toward the new, more role-specific synths.
4. Preload all Stranger-specific SynthDefs during `sclang` startup alongside the repaired Renardo synths.
5. Register Python-side wrappers for these SynthDefs inside the Renardo runtime context so generated code can use direct player syntax such as `p1 >> strangerpulsepad(...)`.
6. Rewrite the prompt recipe so Stranger Things is generated as a layered arrangement rather than a generic retro-horror jam.

## Architecture

### Audio Path

- `dialogue_node.py` / music tools continue generating Renardo/FoxDot code.
- `MusicManager` continues executing that code in the Renardo runtime.
- `sclang` preloads both upstream Renardo SynthDefs and repository-owned custom `.scd` files.
- `scsynth` remains the only synthesis engine.
- Prompting shifts from broad palette hints to a constrained Stranger Things arrangement contract.

### Runtime Registration

There are two distinct requirements for a custom synth to be usable:

1. The SC SynthDef source must be loaded into `sclang` and delivered to `scsynth`.
2. The Python runtime context must expose a file-backed synth wrapper so FoxDot syntax can reference that synth by name.

The design therefore adds:

- repo-owned `.scd` files copied into the image
- a Python registration helper to inject named wrappers into Renardo runtime state
- Stranger Things prompt rules that pin down layer order, note pattern reuse, and safe instrumentation

### Arrangement Contract

The prompt recipe should bias the model toward this structure:

1. Opening harmonic bed on `strangerpulsepad` with long sustain, soft vibrato, and low-pass filtering.
2. Sparse heartbeat kick on `d1`, entering after the pad rather than immediately overcrowding the mix.
3. Fixed bass ostinato on `retrobass` using a deterministic note pattern.
4. Fixed lead arp on `strangerarp` or `supersawlead`, reusing the same pitch sequence above the bass.
5. Optional accent layers with `strangerbrass`, `brass`, or `flute`, used sparingly rather than as constant extra voices.

This keeps the result closer to the reference patch, where recognisability comes from layer order and note identity, not just synthwave timbre.

## Scope

### In Scope

- custom SC-only synthdefs for the three missing Stranger Things roles
- startup preload integration
- runtime registration integration
- tests that lock the preload, registration, and prompt contract
- prompt updates for Stranger Things and related retro-synth directions

### Out of Scope

- SoundFont / General MIDI support
- additional audio services in Compose
- a full orchestral rewrite of every legacy music preset

## Expected Outcome

After deployment, the music stack should generate a more recognisable Stranger Things intro reduction using:

- `strangerpulsepad` for the opening harmonic bed
- `retrobass` for the synth-bass pulse
- `strangerarp` or `supersawlead` for the main arp/lead line
- `strangerbrass` for synth-brass accents
- existing `flute` only as a restrained optional color layer

The result will still be an SC interpretation, not literal GM playback, but it should be closer both in timbre and in musical structure to the supplied Strudel reference while preserving the current live-coding architecture.