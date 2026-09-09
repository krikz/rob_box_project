# SC-Only Imperial March Design

**Date:** 2026-03-20

## Goal

Deliver the strongest possible SC-only Imperial March rendition in the Rob Box music stack while staying inside the existing Renardo + SuperCollider runtime.

## Constraints

1. Stay SC-only.
   - No external orchestral engine.
   - No SoundFont or FluidSynth layer.
2. Keep the runtime-compatible player budget.
   - `p1` through `p3`
   - `d1` through `d3`
3. Optimize for recognizability and weight, not full score reproduction.
4. Respect the existing fast-start music architecture.

## Baseline Findings

1. The current stack can already preload repository-owned custom synthdefs and expose them in the Renardo runtime.
2. Existing validated synths like `strings`, `brass`, and `organ` are useful, but not sufficient for the best SC-only Imperial March rendition.
3. The supplied PDF is a multi-page arranged score, so the practical target is a strong reduction rather than a note-for-note orchestral copy.
4. Imperial March depends more on exact motif intervals, rhythmic accents, and heavy low support than on broad pad complexity.

## Design

### Musical Reduction

Reduce the arrangement to four roles:

1. Main motif lead
   - carries the iconic march phrase
   - should be brass-forward and sharply articulated
2. Low string march support
   - anchors the harmony and pulse
   - should be dry, controlled, and weighty
3. Sustained dramatic bed
   - fills space behind the motif
   - should remain restrained and dark
4. Sparse march percussion
   - optional support only
   - should reinforce accents without sounding like EDM

### New SC-Only Palette

Add two new repository-owned synthdefs:

1. `imperialbrass`
   - primary use: the main melody
   - target character: ensemble brass attack, dark midrange body, short marcato tail
2. `marchstrings`
   - primary use: low supporting strings and rhythmic gravity
   - target character: low string section weight with tighter release than ambient pads

Reuse existing synths selectively:

1. `organ` or `warmpad` for restrained background sustain when needed
2. `strings` as a fallback, not as the main new identity
3. safe drum samples only for light accent support

### Note Representation

Encode the lead motif with `midinote` instead of scale degrees.

Why:

1. The Imperial March motif depends on exact interval placement.
2. `midinote` avoids scale reinterpretation drift.
3. It makes the reduction closer to the source phrase even within FoxDot syntax.

### Runtime Integration

The new synthdefs must follow the same contract already used by the Stranger Things SC-only palette:

1. `.scd` files live under `docker/vision/voice_assistant/custom_synthdefs`
2. `foxdot_init.sc` preloads them at startup
3. `start_voice_assistant.sh` validates them as critical synths
4. Python registration exposes them in the Renardo execution context

### Prompting

Update music prompts so Star Wars / Imperial March themed requests prefer:

1. `imperialbrass` for lead
2. `marchstrings` for low support
3. `strings` or `organ` for sustained backdrop
4. exact `midinote` motif fragments over generic random melodic patterns

The prompt should explicitly steer away from:

1. synthwave-like lead choices for this use case
2. dance-club kick patterns
3. Stranger Things palette defaults when the request is clearly Star Wars march material

## Expected Outcome

This will not produce a score-identical orchestral rendering, but it should produce a much stronger SC-only reduction with:

1. immediate motif recognizability
2. darker brass-forward lead tone
3. heavier low-end march support
4. better alignment between prompt guidance and runtime palette

## Out of Scope

1. Full orchestral score reproduction from PDF
2. Automatic OCR extraction from sheet music
3. Additional audio engines or sample libraries
4. Multi-section concert arrangement beyond the practical `p1`-`p3` runtime budget