# Music Prompt Unification Design

**Date:** 2026-03-08

## Goal

Bring Rob Box music prompting into a consistent state across active runtime prompts, legacy prompts, and inline DJ instructions.

## Source of Truth

The primary music behavior source remains `src/rob_box_voice/prompts/skills/music_skill_prompt.txt`.

## Design

1. `music_skill_prompt.txt` must describe only capabilities actually available to `MusicSkill`.
   - No `speak_text()` instructions inside the music skill prompt.
   - DJ behavior there is limited to music generation and `set_dj_mode()`.
2. `compositor_prompt.txt` becomes the active orchestration layer for music + speech in skills mode.
   - It decides when to call `handle_music`, `speak_text`, `save_dj_persona`, `save_dj_theme`, and `save_dj_set_plan`.
3. `master_prompt_compact.txt` is aligned for flat mode only.
   - It must not reference unavailable skills-mode tools such as `handle_music`.
4. Inline DJ prompts in `dialogue_node.py` must distinguish Renardo code from separate tool calls.
5. Tool docstrings in `music_skill.py` and flat-mode tool exposure in `dialogue_node.py` must match real behavior.

## Expected Outcome

- No impossible tool instructions inside `MusicSkill`
- No contradictory vocal / bass / DJ startup rules
- Clear separation between music generation and spoken DJ narration
- Flat mode and skills mode both remain internally coherent
