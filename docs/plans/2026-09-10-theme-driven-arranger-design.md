# Theme-driven arranger — design

**Date:** 2026-09-10
**Status:** approved
**Issue context:** «даём ноты кузнечика — аранжировщик на выходе даёт произведение»

## Problem

The arranger (`rob_box_mcp_tools/core/arranger.py`) builds form and
development around material the LLM supplies, but it has no way to receive an
exact melody with its rhythm:

- `compose_music` has `lead_notes` (scale degrees) but **no `lead_dur`** —
  durations are owned by the arranger (`ROLE_DEFAULT_DUR` + `_dur_var`).
- `_motif_variants` plays the lead verbatim only on its first entrance, then
  transposes/inverts/retrogrades it per section — so an exact theme (Кузнечик)
  is destroyed after one section.

Known melodies therefore go through `execute_music_code` (raw Renardo from the
prompt library), bypassing the arranger entirely — no form, no arrangement.

## Goal

`compose_music(lead_notes=<тема>, lead_dur=<ритм темы>, + стиль)` →
the arranger keeps the theme verbatim for the whole piece and derives the rest
(bass from the theme, form, layer entrances/exits) itself.

## Non-goals (YAGNI)

- A code-level melody registry (`melody="кузнечик"`) — separate task.
- Auto-deriving pad from the theme — the library's SIMPLICITY RULE already says
  "melody + light bass + drums" for known tunes.
- Imperial March (`midinote=` path) — out of scope here.

## Design

### 1. `core/arranger.py`

- `Layer` gains `durs: Optional[Sequence[float]] = None` — exact note
  durations for a fixed theme.
- `spec_from_flat(...)` gains `lead_dur: Optional[str] = None`. When the lead
  has `lead_dur`, it is parsed with the existing `parse_notes` and stored as
  `Layer.durs`. Length mismatch vs `lead_notes` → `ArrangementError` (honest
  failure; Renardo silently truncates, which was the #1810 failure mode).
- `_render_layer`: when `layer.durs is not None` (fixed theme):
  - render the head as a plain degree list — **skip `_motif_variants`**
    (no transposition/inversion/retrograde);
  - emit `dur=[...]` verbatim — **skip `_dur_var`** (no density override).
  The amp envelope, filter sweep and form section entrances still apply —
  development comes from the form and the layers around the theme.

### 2. `tools/music.py` (`ComposeMusicTool`)

- New optional `lead_dur` parameter (string) + `lead_dur` kwarg in `execute()`,
  forwarded to `spec_from_flat`.

### 3. Catalog

- `python tools/gen_tool_catalog.py` to regenerate `_tool_catalog_data.py` so
  the LLM sees the new parameter; `test_tool_catalog_sync.py` must pass.

### 4. Prompt (`music_skill_prompt.txt`)

- Teach the LLM that degree-based library songs (Кузнечик/Ёлочка/Чижик/
  Собачий вальс) may be passed to `compose_music` with `lead_notes` +
  `lead_dur` to get a full arrangement, instead of bare `execute_music_code`.
  Honesty rule and `search_web` fallback unchanged.

## Backwards compatibility (hard requirement)

`lead_dur` is optional. Without it the fixed-theme branch never triggers and
`_motif_variants` / `_dur_var` run exactly as before. A regression test pins
this.

## Verification

- `test_arranger.py`: fixed theme renders without `Pvar` and with `dur=[...]`;
  mismatch raises `ArrangementError`.
- `test_tools/test_music.py`: `compose_music` without `lead_dur` unchanged.
- Live check after deploy: «сыграй кузнечика в дабстепе».
