-- ============================================================================
-- Migration: 008_downgrade_github_presets.sql
-- Purpose:   GitHub-sourced presets seeded by 006_music_github_presets.sql
--            were imported verbatim from FoxDot/Renardo repos and were NEVER
--            adapted to the robot's hardware constraints (ReSpeaker 16 kHz
--            DAC, no limiter). They use:
--              * chop=          → audible clicks on 16 kHz
--              * players outside d1-d3 / p1-p3 (s1, m1, h1, c1, fx, b1, k1…)
--              * amp sums far above 0.8 (club_energy_128bpm ≈ 6.8)
--              * invented sample letters / patterns
--
--            The DJ auto-transition workflow calls list_tracks(min_rating=4)
--            and load_track(name=...) — which AUTO-PLAYS the returned track
--            code, so a rating-5 preset overwrites the LLM's own (correct)
--            music with the raw preset cacophony.
--
--            Downgrade every type='preset' track to rating=0 so
--            list_tracks(min_rating=4) no longer surfaces them. Tracks stay
--            in the library and can still be loaded explicitly by name.
--
-- Version: 8  (follows 007_generated_music_library.sql)
-- ============================================================================

UPDATE music_tracks
   SET rating = 0
 WHERE type = 'preset';
